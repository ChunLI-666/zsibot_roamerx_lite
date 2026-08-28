#!/usr/bin/env python3
"""Serial, evidence-gated Matrix mapping batch orchestrator.

The manifest deliberately separates recording a mapping bag from offline
Lightning map construction.  Ground truth is allowed only in exploration and
coverage evidence; it is rejected from the formal offline mapping command.
"""
from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any, Mapping, Sequence

import yaml

SCHEMA = "robot_navigo.matrix_mapping_suite"
SCHEMA_VERSION = 1
FINAL_STATUSES = {"PASS", "FAIL", "UNAVAILABLE", "TIMEOUT", "INFRA_FAIL", "SKIPPED"}
REUSABLE_STATUSES = {"PASS"}
PHASES = ("collect_mapping_bag", "offline_lightning_mapping")
PLACEHOLDER = re.compile(r"\{([A-Za-z_][A-Za-z0-9_]*)\}")
FORBIDDEN_GLOBAL_CLEANUP = ("pkill", "killall")


class ManifestError(ValueError):
    pass


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def atomic_write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def read_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def resolve(value: Any, context: Mapping[str, Any]) -> Any:
    if isinstance(value, str):
        unknown = [match.group(1) for match in PLACEHOLDER.finditer(value) if match.group(1) not in context]
        if unknown:
            raise ManifestError(f"unknown template variable: {unknown[0]}")
        return PLACEHOLDER.sub(lambda match: str(context[match.group(1)]), value)
    if isinstance(value, list):
        return [resolve(item, context) for item in value]
    if isinstance(value, dict):
        return {key: resolve(item, context) for key, item in value.items()}
    return value


def resolve_fully(value: Any, context: Mapping[str, Any]) -> Any:
    current = value
    for _ in range(len(context) + 2):
        updated = resolve(current, context)
        if updated == current:
            return updated
        current = updated
    raise ManifestError("value contains a cyclic or unresolved template")


def command_contains_gt(command: Sequence[str]) -> bool:
    text = " ".join(command).lower()
    return any(token in text for token in ("ground_truth", "--gt", "/odom/mujoco_odom"))


def command_contains_global_cleanup(command: Sequence[str]) -> bool:
    text = " ".join(command).lower()
    return any(re.search(rf"(^|[\s;/]){token}([\s;&|]|$)", text)
               for token in FORBIDDEN_GLOBAL_CLEANUP)


def validate_manifest(manifest: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    if manifest.get("schema") != SCHEMA:
        errors.append(f"schema must be {SCHEMA!r}")
    if manifest.get("schema_version") != SCHEMA_VERSION:
        errors.append(f"schema_version must be {SCHEMA_VERSION}")
    scenes = manifest.get("scenes")
    if not isinstance(scenes, list) or not scenes:
        return errors + ["scenes must be a non-empty list"]
    names, ids, domains = set(), set(), set()
    for number, scene in enumerate(scenes):
        prefix = f"scenes[{number}]"
        if not isinstance(scene, Mapping):
            errors.append(f"{prefix} must be a mapping")
            continue
        name, scene_id = scene.get("name"), scene.get("scene_id")
        if not isinstance(name, str) or not name or name in names:
            errors.append(f"{prefix}.name must be unique and non-empty")
        else:
            names.add(name)
        if (not isinstance(scene_id, int) or isinstance(scene_id, bool) or
                scene_id < 0 or scene_id in ids):
            errors.append(f"{prefix}.scene_id must be a unique non-negative integer")
        else:
            ids.add(scene_id)
        if not isinstance(scene.get("available"), bool) or not isinstance(scene.get("enabled"), bool):
            errors.append(f"{prefix}.available and enabled must be booleans")
            continue
        if not scene["available"]:
            if not scene.get("reason"):
                errors.append(f"{prefix}.reason is required for unavailable scenes")
            continue
        domain = scene.get("ros_domain_id")
        if (not isinstance(domain, int) or isinstance(domain, bool) or
                not 0 <= domain <= 232 or domain in domains):
            errors.append(f"{prefix}.ros_domain_id must be a unique integer in [0, 232]")
        else:
            domains.add(domain)
        for field in ("mapping_bag", "localization_bag", "coverage_metadata",
                      "mapping_dataset_id", "localization_dataset_id"):
            if not isinstance(scene.get(field), str) or not scene[field]:
                errors.append(f"{prefix}.{field} is required")
        if (scene.get("mapping_dataset_id") and scene.get("mapping_dataset_id") ==
                scene.get("localization_dataset_id")):
            errors.append(f"{prefix} mapping and localization dataset IDs must differ")
        for phase in PHASES:
            definition = scene.get(phase)
            if not isinstance(definition, Mapping):
                errors.append(f"{prefix}.{phase} is required")
                continue
            command = definition.get("command")
            if not isinstance(command, list) or not command or not all(isinstance(x, str) for x in command):
                errors.append(f"{prefix}.{phase}.command must be a non-empty string list")
            elif command_contains_global_cleanup(command):
                errors.append(f"{prefix}.{phase}.command must not use pkill or killall")
            if phase == "offline_lightning_mapping" and isinstance(command, list) and command_contains_gt(command):
                errors.append(f"{prefix}.{phase}.command must not use GT in formal map generation")
            outputs = definition.get("required_outputs", [])
            if (not isinstance(outputs, list) or not outputs or
                    not all(isinstance(x, str) for x in outputs)):
                errors.append(f"{prefix}.{phase}.required_outputs must be a non-empty string list")
            if (phase == "collect_mapping_bag" and
                    scene.get("mapping_bag") not in outputs and
                    "{mapping_bag}" not in outputs):
                errors.append(f"{prefix}.{phase}.required_outputs must include mapping_bag")
            if phase == "offline_lightning_mapping":
                evidence = definition.get("required_evidence", [])
                if not isinstance(evidence, list) or scene.get("mapping_bag") not in evidence:
                    errors.append(
                        f"{prefix}.{phase}.required_evidence must include mapping_bag")
                if (isinstance(command, list) and
                        not any(scene.get("mapping_bag", "") in arg or
                                "{mapping_bag}" in arg for arg in command)):
                    errors.append(f"{prefix}.{phase}.command must explicitly consume mapping_bag")
    return errors


def input_signature(scene: Mapping[str, Any], phase: str, context: Mapping[str, Any]) -> str:
    payload = {"scene": scene, "phase": phase, "context": context}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, default=str).encode()).hexdigest()


def evidence_paths(definition: Mapping[str, Any], context: Mapping[str, Any]) -> list[Path]:
    return [Path(item).expanduser()
            for item in resolve_fully(definition.get("required_evidence", []), context)]


def coverage_gate(path: Path, mapping_bag: Path,
                  minimum_coverage_pct: float = 100.0) -> tuple[bool, str, dict[str, Any] | None]:
    value = read_json(path)
    if value is None:
        return False, f"coverage metadata is missing or invalid: {path}", None
    metadata = value.get("metadata")
    if value.get("schema") != "robot_navigo.map_coverage" or not isinstance(metadata, Mapping):
        return False, "coverage evidence must be map_coverage evaluate output", value
    percent = metadata.get("coverage_pct")
    try:
        threshold = float(minimum_coverage_pct)
        if not 0.0 < threshold <= 100.0:
            return False, f"invalid minimum coverage threshold: {threshold}", value
        if float(percent) + 1e-9 < threshold:
            return False, f"coverage is below gate ({percent!r} < {threshold})", value
    except (TypeError, ValueError):
        return False, "coverage metadata has no numeric coverage_pct", value
    if threshold >= 100.0 and metadata.get("uncovered_cells") != 0:
        return False, "100% coverage requires uncovered_cells=0", value
    if not metadata.get("map_fingerprint_sha256"):
        return False, "coverage evidence has no map fingerprint", value
    source = metadata.get("trajectory_source")
    if not isinstance(source, str) or Path(source).expanduser().resolve() != mapping_bag.resolve():
        return False, "coverage trajectory_source must be the mapping bag", value
    return True, f"coverage evidence accepted ({float(percent):.6f}%)", value


def bags_are_independent(mapping_bag: Path, localization_bag: Path,
                         mapping_dataset_id: str,
                         localization_dataset_id: str) -> tuple[bool, str]:
    if mapping_dataset_id == localization_dataset_id:
        return False, "mapping and localization/eval dataset IDs must differ"
    try:
        same = mapping_bag.resolve() == localization_bag.resolve()
    except OSError:
        same = str(mapping_bag) == str(localization_bag)
    if not same and mapping_bag.exists() and localization_bag.exists():
        try:
            same = os.path.samefile(mapping_bag, localization_bag)
        except OSError:
            pass
    return (not same, "mapping and localization/eval bags are independent" if not same else
            "mapping bag must differ from localization/eval bag")


def terminate_process_group(process: subprocess.Popen[Any], grace_sec: float = 5.0) -> None:
    """Terminate only the new session owned by this invocation."""
    if process.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=grace_sec)
            return
        except subprocess.TimeoutExpired:
            pass


def status_record(scene: Mapping[str, Any], phase: str, status: str, reason: str,
                  signature: str, started: str, elapsed: float, **extra: Any) -> dict[str, Any]:
    return {"schema": "robot_navigo.matrix_mapping_phase_status", "schema_version": 1,
            "scene": scene["name"], "scene_id": scene["scene_id"], "phase": phase,
            "status": status, "reason": reason, "started_at_utc": started,
            "ended_at_utc": utc_now(), "elapsed_sec": round(elapsed, 3),
            "input_signature": signature, **extra}


def run_phase(scene: Mapping[str, Any], phase: str, context: Mapping[str, Any], scene_dir: Path,
              resume: bool, dry_run: bool, timeout_override: float | None) -> dict[str, Any]:
    definition = resolve_fully(scene[phase], context)
    phase_dir = scene_dir / phase
    status_path = phase_dir / "phase_status.json"
    signature = input_signature(scene, phase, context)
    prior = read_json(status_path) if resume else None
    if prior and prior.get("status") in REUSABLE_STATUSES and prior.get("input_signature") == signature:
        prior_outputs = [Path(item) for item in prior.get("required_outputs", [])]
        if prior_outputs and all(path.exists() for path in prior_outputs):
            prior["resumed"] = True
            atomic_write_json(status_path, prior)
            return prior
    started, monotonic = utc_now(), time.monotonic()
    atomic_write_json(status_path, status_record(scene, phase, "RUNNING", "phase started", signature, started, 0.0))
    command = definition["command"]
    if dry_run:
        record = status_record(scene, phase, "SKIPPED", "dry-run", signature, started, 0.0,
                               command=command, required_outputs=definition.get("required_outputs", []))
        atomic_write_json(status_path, record); return record
    missing = [str(path) for path in evidence_paths(definition, context) if not path.exists()]
    if missing:
        record = status_record(scene, phase, "UNAVAILABLE", "required evidence is missing: " + ", ".join(missing), signature, started, time.monotonic()-monotonic)
        atomic_write_json(status_path, record); return record
    phase_dir.mkdir(parents=True, exist_ok=True)
    timeout = float(timeout_override or definition.get("timeout_sec", 1800))
    env = os.environ.copy(); env.update({"ROS_DOMAIN_ID": str(scene["ros_domain_id"]),
        "MATRIX_MAPPING_SCENE": str(scene["name"]), "MATRIX_MAPPING_PHASE": phase,
        "MATRIX_MAPPING_OUTPUT": str(phase_dir)})
    log = phase_dir / "command.log"; returncode = None
    try:
        with log.open("w", encoding="utf-8") as stream:
            stream.write(json.dumps({"argv": command, "cwd": context["scene_dir"], "environment": {k: env[k] for k in ("ROS_DOMAIN_ID", "MATRIX_MAPPING_SCENE", "MATRIX_MAPPING_PHASE", "MATRIX_MAPPING_OUTPUT")}}, ensure_ascii=False) + "\n")
            process = subprocess.Popen(command, cwd=context["scene_dir"], env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True, text=True)
            try: returncode = process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                terminate_process_group(process)
                record = status_record(scene, phase, "TIMEOUT", f"timeout after {timeout}s", signature, started, time.monotonic()-monotonic, command=command, returncode=returncode)
                atomic_write_json(status_path, record); return record
    except OSError as exc:
        record = status_record(scene, phase, "INFRA_FAIL", f"cannot start command: {exc}", signature, started, time.monotonic()-monotonic, command=command)
        atomic_write_json(status_path, record); return record
    outputs = [Path(item) for item in definition.get("required_outputs", [])]
    missing_outputs = [str(path) for path in outputs if not path.exists()]
    status, reason = (("PASS", "command and required artifacts completed")
                      if returncode == 0 and not missing_outputs else
                      ("FAIL", f"return code {returncode}; missing outputs: {missing_outputs}"))
    record = status_record(scene, phase, status, reason, signature, started, time.monotonic()-monotonic, command=command, returncode=returncode, required_outputs=[str(x) for x in outputs])
    atomic_write_json(status_path, record); return record


def run_scene(scene: Mapping[str, Any], run_dir: Path, variables: Mapping[str, Any], resume: bool,
              dry_run: bool, timeout_override: float | None) -> list[dict[str, Any]]:
    scene_dir = run_dir / str(scene["name"]); context = {**variables, **scene, "run_dir": str(run_dir), "scene_dir": str(scene_dir)}
    if not scene["available"] or not scene["enabled"]:
        reason = scene.get("reason", "scene disabled")
        rows = []
        for phase in PHASES:
            record = status_record(scene, phase, "UNAVAILABLE", reason, input_signature(scene, phase, context), utc_now(), 0.0)
            atomic_write_json(scene_dir / phase / "phase_status.json", record)
            rows.append(record)
        return rows
    mapping_bag = Path(resolve_fully(scene["mapping_bag"], context))
    eval_bag = Path(resolve_fully(scene["localization_bag"], context))
    independent, reason = bags_are_independent(
        mapping_bag, eval_bag, str(scene["mapping_dataset_id"]),
        str(scene["localization_dataset_id"]))
    if not independent:
        rows = []
        for phase in PHASES:
            record = status_record(scene, phase, "UNAVAILABLE", reason, input_signature(scene, phase, context), utc_now(), 0.0)
            atomic_write_json(scene_dir / phase / "phase_status.json", record); rows.append(record)
        return rows
    collect = run_phase(scene, PHASES[0], context, scene_dir, resume, dry_run, timeout_override)
    if dry_run and collect["status"] == "SKIPPED":
        offline = run_phase(scene, PHASES[1], context, scene_dir, resume, True, timeout_override)
        return [collect, offline]
    if collect["status"] != "PASS":
        reason = f"collection phase is {collect['status']}; offline mapping not executable"
        offline = status_record(scene, PHASES[1], "UNAVAILABLE", reason, input_signature(scene, PHASES[1], context), utc_now(), 0.0)
        atomic_write_json(scene_dir / PHASES[1] / "phase_status.json", offline); return [collect, offline]
    gate, reason, metadata = coverage_gate(
        Path(resolve_fully(scene["coverage_metadata"], context)), mapping_bag,
        float(scene.get("minimum_coverage_pct", 100.0)))
    if not gate:
        offline = status_record(scene, PHASES[1], "UNAVAILABLE", reason, input_signature(scene, PHASES[1], context), utc_now(), 0.0, coverage_metadata=metadata)
        atomic_write_json(scene_dir / PHASES[1] / "phase_status.json", offline); return [collect, offline]
    context["coverage_metadata"] = str(
        Path(resolve_fully(scene["coverage_metadata"], context)))
    offline = run_phase(scene, PHASES[1], context, scene_dir, resume, dry_run, timeout_override)
    offline["bag_independence"] = {
        "mapping_bag": str(mapping_bag),
        "mapping_dataset_id": scene["mapping_dataset_id"],
        "localization_eval_bag": str(eval_bag),
        "localization_dataset_id": scene["localization_dataset_id"],
        "verified": True,
    }
    atomic_write_json(scene_dir / PHASES[1] / "phase_status.json", offline)
    return [collect, offline]


@contextlib.contextmanager
def batch_lock(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+", encoding="utf-8") as stream:
        try: fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc: raise ManifestError(f"batch lock is already held: {path}") from exc
        try: yield
        finally: fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def load_manifest(path: Path) -> dict[str, Any]:
    try: value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc: raise ManifestError(f"cannot load suite: {exc}") from exc
    if not isinstance(value, dict): raise ManifestError("suite root must be a mapping")
    errors = validate_manifest(value)
    if errors: raise ManifestError("\n".join(errors))
    return value


def resolve_variables(raw: Mapping[str, Any], extra: Mapping[str, Any]) -> dict[str, Any]:
    resolved = dict(raw)
    for _ in range(len(raw) + 1):
        updated = resolve(raw, {**extra, **resolved})
        if updated == resolved:
            return updated
        resolved = updated
    raise ManifestError("variables contain a cyclic or unresolved template")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", required=True); parser.add_argument("--output", required=True)
    parser.add_argument("--lock-file"); parser.add_argument("--scene", action="append")
    parser.add_argument("--resume", action="store_true"); parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--timeout-sec", type=float)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        suite = Path(args.suite).resolve(); manifest = load_manifest(suite); run_dir = Path(args.output).resolve()
        lock = Path(args.lock_file).resolve() if args.lock_file else run_dir / ".matrix_mapping.lock"
        variables = resolve_variables(manifest.get("variables", {}), {"run_dir": str(run_dir)})
        selected = [s for s in manifest["scenes"] if not args.scene or s["name"] in args.scene]
        if args.scene and {str(item) for item in args.scene} - {str(s["name"]) for s in selected}:
            missing = sorted({str(item) for item in args.scene} - {str(s["name"]) for s in selected})
            raise ManifestError(f"unknown scenes requested: {', '.join(missing)}")
        with batch_lock(lock):
            rows = [row for scene in selected for row in run_scene(scene, run_dir, variables, args.resume, args.dry_run, args.timeout_sec)]
            summary = {"schema": "robot_navigo.matrix_mapping_batch_summary", "schema_version": 1, "generated_at_utc": utc_now(), "results": rows,
                       "dry_run": bool(args.dry_run),
                       "pass": bool(rows) and all(r["status"] == "PASS" for r in rows)}
            atomic_write_json(run_dir / "batch_summary.json", summary)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0 if args.dry_run or summary["pass"] else 1
    except ManifestError as exc:
        print(f"[FAIL] {exc}", file=sys.stderr); return 2


if __name__ == "__main__":
    raise SystemExit(main())
