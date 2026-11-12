#!/usr/bin/env python3
"""Execute one immutable workflow invocation and persist process/return evidence.

This helper never issues signals or ROS actions. A worker survives its caller;
an uncertain worker is inspected, never blindly relaunched. Long-lived actors
must have their own finite lifetime and exact-owner cleanup mechanism.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def sync_dir(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def write_once(path, value):
    raw = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode()
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as out:
        out.write(raw)
        out.flush()
        os.fsync(out.fileno())
    sync_dir(path.parent)


def identity(pid):
    proc = Path("/proc") / str(pid)
    fields = (proc / "stat").read_text().rsplit(")", 1)[1].split()
    cmd = (proc / "cmdline").read_bytes()
    if fields[0] in ("Z", "X") or not cmd:
        raise ProcessLookupError("process exited")
    status = (proc / "status").read_text()
    nspid = next((line.split()[1:] for line in status.splitlines() if line.startswith("NSpid:")), [])
    return {
        "pid": pid, "starttime_ticks": int(fields[19]), "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
        "pid_namespace": os.readlink(proc / "ns/pid"), "nspid": [int(x) for x in nspid],
        "cmdline_sha256": digest(cmd),
    }


def load_spec(path, expected):
    path = Path(path).resolve(strict=True)
    raw = path.read_bytes()
    if digest(raw) != expected:
        raise ValueError("invocation specification hash changed")
    spec = json.loads(raw)
    if spec.get("schema") != "terra.workflow.invocation.v1":
        raise ValueError("invalid invocation schema")
    argv = spec.get("argv")
    if not isinstance(argv, list) or not argv or any(not isinstance(x, str) or "\0" in x for x in argv):
        raise ValueError("argv must be nonempty NUL-free strings")
    if not Path(argv[0]).is_absolute() or not Path(spec["cwd"]).is_absolute():
        raise ValueError("program and cwd must be absolute")
    if spec.get("kind") not in ("command", "client", "spawn"):
        raise ValueError("unsupported invocation kind")
    return spec


def nested(obj, dotted):
    for key in dotted.split("."):
        if not isinstance(obj, dict) or key not in obj:
            raise ValueError("readiness field missing: " + dotted)
        obj = obj[key]
    return obj


def readiness(config):
    path = Path(config["path"])
    raw = path.read_bytes()
    if config.get("format", "json") == "jsonl":
        lines = raw.splitlines(keepends=True)
        candidates = []
        for line in lines:
            if not line.endswith(b"\n"):
                continue
            row = json.loads(line)
            try:
                if all(nested(row, field) == expected for field, expected in config.get("equals", {}).items()):
                    candidates.append(row)
            except ValueError:
                continue
        if len(candidates) != 1:
            raise ValueError("readiness JSONL must contain exactly one matching event")
        payload = candidates[0]
    elif config.get("format", "json") == "json":
        payload = json.loads(raw)
        for field, expected in config.get("equals", {}).items():
            if nested(payload, field) != expected:
                raise ValueError("readiness predicate differs: " + field)
    else:
        raise ValueError("unsupported readiness format")
    for binding in config.get("same_fields", []):
        source = json.loads(Path(binding["source_path"]).read_text())
        if nested(payload, binding["field"]) != nested(source, binding["source_field"]):
            raise ValueError("readiness identity differs: " + binding["field"])
    if "max_age_sec" in config:
        stamp = nested(payload, config.get("timestamp_field", "time_ns"))
        age = (time.time_ns() - stamp) / 1e9
        if not -0.05 <= age <= float(config["max_age_sec"]):
            raise ValueError("readiness is stale or future")
    if config.get("valid_until_field") and nested(payload, config["valid_until_field"]) <= time.time_ns():
        raise ValueError("readiness lease expired")
    return {"path": str(path), "sha256": digest(raw), "observed_unix_ns": time.time_ns(), "observed_payload": payload}


def child(spec_path, expected):
    spec = load_spec(spec_path, expected)
    directory = Path(spec["invocation_dir"])
    own = identity(os.getpid())
    own.update({"intended_argv_sha256": digest(json.dumps(spec["argv"], ensure_ascii=False, separators=(",", ":")).encode()), "recorded_unix_ns": time.time_ns()})
    write_once(directory / "child_identity.json", own)
    env = {key: os.environ[key] for key in spec.get("inherit_env", []) if key in os.environ}
    env.update(spec.get("env", {}))
    os.chdir(spec["cwd"])
    os.execve(spec["argv"][0], spec["argv"], env)


def worker(spec_path, expected):
    spec = load_spec(spec_path, expected)
    directory = Path(spec["invocation_dir"])
    if directory.resolve() != Path(spec_path).resolve().parent:
        raise ValueError("invocation directory differs from spec parent")
    own = identity(os.getpid())
    write_once(directory / "worker_started.json", {"identity": own, "spec_sha256": expected, "time_ns": time.time_ns()})
    result = {"schema": "terra.workflow.worker_result.v1", "stage_id": spec["stage_id"], "spec_sha256": expected, "kind": spec["kind"], "state": "WORKER_FAILED"}
    try:
        with (directory / "stdout.log").open("xb") as stdout, (directory / "stderr.log").open("xb") as stderr:
            argv = [sys.executable, "-B", str(Path(__file__).resolve()), "child", str(spec_path), expected]
            proc = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr, start_new_session=True)
            limit = time.monotonic() + 5
            identity_file = directory / "child_identity.json"
            while not identity_file.exists() and proc.poll() is None and time.monotonic() < limit:
                time.sleep(0.02)
            if identity_file.exists():
                child_identity = json.loads(identity_file.read_text())
                result["child_identity"] = child_identity
            else:
                result["child_identity"] = None
                result["identity_reason"] = "child wrapper did not produce identity; execution may be uncertain"
            if spec["kind"] == "spawn":
                if result["child_identity"] is None:
                    result.update(state="HELD", reason="actor identity missing")
                else:
                    ready_config = spec["ready"]
                    deadline = time.monotonic() + float(ready_config["timeout_sec"])
                    last_error = "readiness not yet observed"
                    while time.monotonic() < deadline:
                        if proc.poll() is not None:
                            result.update(state="HELD", reason="actor exited before readiness", returncode=proc.returncode)
                            break
                        try:
                            actual = identity(proc.pid)
                            if any(actual[k] != child_identity[k] for k in ("pid", "starttime_ticks", "boot_id", "pid_namespace")):
                                raise ValueError("actor birth identity changed")
                            ready = readiness(ready_config)
                            result.update(state="SPAWNED", actor_identity=actual, ready=ready)
                            break
                        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                            last_error = str(exc)
                            time.sleep(0.1)
                    else:
                        result.update(state="HELD", reason="readiness timeout: " + last_error, actor_may_still_run=True)
            else:
                code = proc.wait()
                if result["child_identity"] is None:
                    result.update(state="HELD", reason="child birth identity missing", returncode=code)
                else:
                    result.update(state="EXITED", returncode=code)
    except BaseException as exc:
        result.update(state="HELD", reason=f"{type(exc).__name__}: {exc}", effects_may_have_occurred=True)
    result["finished_unix_ns"] = time.time_ns()
    write_once(directory / "worker_result.json", result)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=["worker", "child"])
    p.add_argument("spec")
    p.add_argument("spec_sha256")
    args = p.parse_args()
    if args.mode == "child":
        child(args.spec, args.spec_sha256)
    else:
        worker(args.spec, args.spec_sha256)


if __name__ == "__main__":
    main()
