#!/usr/bin/env python3
"""Resumable simulation workflow; default run is a side-effect-free dry run.

Actual execution requires a Main code-acceptance receipt bound to the exact
variant, commit and build receipts. Only the canonical mission client can send/reconcile
missions. The workflow owns stages and evidence, never waypoint decisions.
"""

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from types import SimpleNamespace
import uuid

import mission_ledger
import workflow_worker


HERE = Path(__file__).resolve().parent
NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
HASH = re.compile(r"^[0-9a-f]{64}$")
COMMIT = re.compile(r"^[0-9a-f]{40}$")
ROLES = ("environment", "map", "localization", "recording", "runtime", "mission", "closure", "mcap", "video", "video_qa", "acceptance")
CLIENT_OPS = {"inspect", "start", "import-closed-attempt", "resume", "reconcile", "cancel"}
CLIENT_ARGS = {"route", "identity", "policy", "closed-attempt-manifest", "ready", "resource-lock", "reason", "odom-topic", "scan-topic", "simulation-scan-max-age-sec", "simulation-first-leg-max-distance-m", "camera-topic", "camera-info-topic"}
ENV_KEYS = {"PATH", "LANG", "LC_ALL", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION", "FASTDDS_DEFAULT_PROFILES_FILE", "FASTRTPS_DEFAULT_PROFILES_FILE", "LD_LIBRARY_PATH", "PYTHONPATH", "AMENT_PREFIX_PATH", "PYTHONDONTWRITEBYTECODE", "PYTHONNOUSERSITE", "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "ROS_HOME", "ROS_LOG_DIR", "TMPDIR", "XDG_CACHE_HOME", "LIGHTNING_LOG_DIR"}


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def file_sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def sync_dir(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def write_once(path, raw):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as out:
        out.write(raw)
        out.flush()
        os.fsync(out.fileno())
    sync_dir(path.parent)


def atomic_json(path, data):
    raw = (json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    tmp = path.with_name(path.name + ".tmp." + uuid.uuid4().hex)
    write_once(tmp, raw)
    os.replace(tmp, path)
    sync_dir(path.parent)


def valid_name(value):
    if not isinstance(value, str) or not NAME.fullmatch(value):
        raise ValueError("invalid workflow/stage identifier")
    return value


def validate_files(records):
    if not isinstance(records, list):
        raise ValueError("pinned files must be a list")
    for item in records:
        path, expected = Path(item["path"]), item["sha256"]
        if not path.is_absolute() or not HASH.fullmatch(expected) or not path.is_file() or file_sha(path) != expected:
            raise ValueError("pinned input missing or hash mismatch: " + str(path))


def check_argv(argv, *, client=False):
    if not isinstance(argv, list) or not argv or any(not isinstance(x, str) or "\0" in x for x in argv):
        raise ValueError("argv must be NUL-free strings")
    if not Path(argv[0]).is_absolute():
        raise ValueError("program path must be absolute")
    if Path(argv[0]).name in ("sh", "bash", "dash", "zsh", "fish", "ssh") or "-c" in argv:
        raise ValueError("shell and code-string execution are not supported")
    joined = " ".join(argv).lower()
    if not client and ("mission_resume.py" in joined or "waypoint_batch.py" in joined or " action send_goal" in joined or " topic pub" in joined):
        raise ValueError("mission/action operations must use the Astra client stage")


def validate_plan(plan):
    if plan.get("schema") != "terra.workflow.execution_plan.v1":
        raise ValueError("invalid execution plan schema")
    valid_name(plan.get("workflow_run_id"))
    if plan.get("variant") not in ("segmented", "official") or not COMMIT.fullmatch(plan.get("commit", "")):
        raise ValueError("full commit and primary variant required")
    if not plan.get("build_receipts"):
        raise ValueError("at least one commit-bound build receipt is required")
    for entry in plan["build_receipts"]:
        if not Path(entry["path"]).is_absolute() or not HASH.fullmatch(entry["sha256"]):
            raise ValueError("invalid build receipt binding")
    stages = plan.get("stages")
    if not isinstance(stages, list) or not stages:
        raise ValueError("workflow stages required")
    seen, roles = set(), {}
    for index, stage in enumerate(stages):
        name = valid_name(stage.get("id"))
        if name in seen:
            raise ValueError("duplicate stage")
        if any(dep not in seen for dep in stage.get("depends_on", [])):
            raise ValueError("dependencies must refer to previous stages")
        seen.add(name)
        for role in stage.get("roles", []):
            if role not in ROLES:
                raise ValueError("unknown workflow role")
            roles.setdefault(role, []).append(index)
        kind = stage.get("kind")
        if kind not in ("command", "spawn", "client"):
            raise ValueError("unknown stage kind")
        if kind == "client":
            client = stage.get("client", {})
            if client.get("operation") not in CLIENT_OPS:
                raise ValueError("unknown Astra operation")
            check_argv(client.get("prefix"), client=True)
            if not isinstance(client.get("args", {}), dict) or not set(client.get("args", {})) <= CLIENT_ARGS:
                raise ValueError("unsupported client option; goal index/skip are forbidden")
            if not client.get("store") or not client.get("accepted_outcomes"):
                raise ValueError("client store and accepted outcomes required")
        else:
            check_argv(stage.get("argv"))
            if kind == "spawn":
                if stage.get("finite_owner_and_exact_cleanup") is not True or not stage.get("ready", {}).get("path"):
                    raise ValueError("spawn needs finite-owner cleanup contract and readiness receipt")
            elif not stage.get("success_files"):
                raise ValueError("command stage requires explicit success evidence")
        if not isinstance(stage.get("files", []), list):
            raise ValueError("stage files must be a list")
        commands = stage["client"]["prefix"] if kind == "client" else stage["argv"]
        pinned = {entry["path"] for entry in stage.get("files", [])}
        scripts = {arg for arg in commands if arg.endswith(".py") and Path(arg).is_absolute()}
        if not scripts or not scripts <= pinned:
            raise ValueError("every stage must pin its absolute Python entry script")
        if not Path(stage.get("cwd", "")).is_absolute():
            raise ValueError("stage cwd must be absolute")
    if any(role not in roles for role in ROLES):
        raise ValueError("full simulation plan is missing a required workflow role")
    # A single runtime launcher can own recording too, but no mission can precede
    # environment/map/localization/recording/runtime, or closure/QA precede it.
    mission = min(roles["mission"])
    if any(max(roles[x]) > mission for x in ("environment", "map", "localization", "recording", "runtime")):
        raise ValueError("mission precedes an admission stage")
    for earlier, later in (("mission", "closure"), ("closure", "mcap"), ("mcap", "video"), ("video", "video_qa"), ("video_qa", "acceptance")):
        if max(roles[earlier]) > min(roles[later]):
            raise ValueError("workflow role order is invalid")
    if not any(s.get("kind") == "client" and "mission" in s.get("roles", []) for s in stages):
        raise ValueError("mission stages must use the Astra client")
    return plan


def approval_check(plan, approval_path):
    approval_path = Path(approval_path).resolve(strict=True)
    from workflow_inputs import candidate_check,entrypoint_check
    approved = candidate_check(approval_path)
    entrypoint_check(plan,approved)
    binding=plan.get('input_binding',{}).get('candidate',{})
    if binding.get('path')!=str(approval_path) or binding.get('sha256')!=file_sha(approval_path):
        raise ValueError('candidate differs from prepared run input')
    if approved['variant']!=plan['variant']:
        raise ValueError('candidate variant differs from plan')
    if approved.get("commit") != plan["commit"]:
        raise ValueError("approved commit differs from execution plan")
    expected = {item["sha256"] for item in plan["build_receipts"]}
    if expected != {item["sha256"] for item in approved["build_receipts"]}:
        raise ValueError("approved build receipts differ from execution plan")
    receipt_types=set()
    for entry in plan["build_receipts"]:
        path = Path(entry["path"])
        if not path.is_file() or file_sha(path) != entry["sha256"]:
            raise ValueError("build receipt hash mismatch")
        receipt = json.loads(path.read_text())
        schema=receipt.get('schema')
        if schema not in ('matrix.navigation.build_receipt.v1','matrix.recovery.build_receipt.v1','matrix.waypoint.build_receipt.v1') or schema in receipt_types:
            raise ValueError('Three unambiguous typed component build receipts required')
        receipt_types.add(schema)
        if receipt.get("repository_commit") != plan["commit"] or receipt.get("source_files_match_commit") is not True:
            raise ValueError("build receipt is not bound to the accepted commit")
        sources=receipt.get('source_sha256')
        accepted_sources={row['name']:row['sha256'] for row in approved['source_files']}
        if not sources or any(accepted_sources.get(name)!=value for name,value in sources.items()):
            raise ValueError('build receipt sources differ from accepted candidate files')
        # Main authorizes byte-preserving relocation. The approved parent is
        # always checked above; the derived receipt may change only its prefix.
        if entry.get("runtime_path"):
            derived_path = Path(entry["runtime_path"]).resolve(strict=True)
            if file_sha(derived_path) != entry.get("runtime_sha256"):
                raise ValueError("relocated build receipt hash mismatch")
            derived = json.loads(derived_path.read_text())
            if set(derived) != set(receipt) or any(derived[k] != v for k, v in receipt.items() if k != "install_prefix"):
                raise ValueError("relocated receipt changes more than install_prefix")
            if not Path(derived.get("install_prefix", "")).is_absolute():
                raise ValueError("relocated install prefix must be absolute")
            receipt = derived
        install, artifacts = receipt.get("install_prefix"), receipt.get("artifact_sha256")
        if not install or not isinstance(artifacts,dict) or not artifacts or not Path(install).is_absolute():
            raise ValueError('Absolute install and nonempty build artifacts required')
        root = Path(install).resolve(strict=True)
        for name, expected_sha in artifacts.items():
            artifact = (root / name).resolve(strict=True)
            if not artifact.is_relative_to(root) or file_sha(artifact) != expected_sha:
                raise ValueError("installed build artifact mismatch: " + str(artifact))
    if receipt_types!={'matrix.navigation.build_receipt.v1','matrix.recovery.build_receipt.v1','matrix.waypoint.build_receipt.v1'}:
        raise ValueError('Navigation/recovery/waypoint component receipts all required')
    return {"path": str(approval_path), "sha256": file_sha(approval_path), "client_idempotency_tested": approved.get("client_request_idempotency_tested") is True}


def replace_run(value, run_dir):
    return value.replace("{run_dir}", str(run_dir))


def resolve_token(value, run_dir, traces=None):
    value = replace_run(value, run_dir)
    if not value.startswith("@json:"):
        return value
    reference = value[len("@json:"):]
    if "#" not in reference:
        raise ValueError("JSON token requires path#field")
    filename, field = reference.rsplit("#", 1)
    path = Path(filename).resolve(strict=True)
    if not path.is_relative_to(run_dir):
        raise ValueError("dynamic JSON input must belong to this workflow run")
    raw = path.read_bytes()
    item = workflow_worker.nested(json.loads(raw), field)
    if isinstance(item, bool) or not isinstance(item, (str, int, float)):
        raise ValueError("dynamic JSON value must be a scalar")
    if traces is not None:
        traces.append({"path": str(path), "sha256": sha(raw), "field": field, "value": str(item)})
    return str(item)


def stage_dir(run_dir, stage):
    return run_dir / "stages" / stage["id"]


def output_conditions(specs, run_dir):
    observations = []
    for item in specs:
        path = Path(replace_run(item["path"], run_dir))
        _, actual_hash, raw = mission_ledger.read_stable(path, include_data=bool(item.get("equals")))
        if item.get("sha256") and actual_hash != item["sha256"]:
            raise ValueError("stage output hash mismatch: " + str(path))
        if item.get("equals"):
            value = json.loads(raw)
            for field, expected in item["equals"].items():
                if workflow_worker.nested(value, field) != expected:
                    raise ValueError("stage output predicate mismatch: " + field)
        observations.append({"path": str(path), "sha256": actual_hash, "bytes": path.stat().st_size})
    return observations


def inspect_client(prefix, store, cwd, env, inherit_env, directory):
    command = list(prefix) + ["inspect", "--store", store, "--json"]
    environment = {k: os.environ[k] for k in inherit_env if k in os.environ}
    environment.update(env)
    proc = subprocess.run(command, cwd=cwd, env=environment, stdin=subprocess.DEVNULL, capture_output=True, timeout=20)
    write_once(directory / "inspect.stdout.json", proc.stdout)
    write_once(directory / "inspect.stderr.log", proc.stderr)
    if proc.returncode != 0:
        raise ValueError("Astra inspect failed")
    state = json.loads(proc.stdout)
    mission_ledger.require_fields(state, "matrix.mission.state.v1", mission_ledger.STATE_FIELDS)
    mission_ledger.canonical_uuid(state["mission_id"])
    mission_ledger.revision(state["revision"])
    return state, {"argv": command, "stdout_sha256": sha(proc.stdout), "stderr_sha256": sha(proc.stderr)}


def monitor_client(run_dir, stage, row):
    if stage["kind"] != "client" or not row["invocations"]:
        raise ValueError("monitor requires a started Astra client stage")
    invocation = Path(row["invocations"][-1]["directory"])
    spec = json.loads((invocation / "invocation.json").read_text())
    prefix = [replace_run(x, run_dir) for x in stage["client"]["prefix"]]
    store = Path(replace_run(stage["client"]["store"], run_dir)).resolve()
    if not store.is_relative_to(run_dir):
        raise ValueError("client store outside workflow run")
    directory = stage_dir(run_dir, stage) / "monitor" / uuid.uuid4().hex
    directory.mkdir(parents=True, exist_ok=False)
    observed, receipt = inspect_client(prefix, str(store), spec["cwd"], spec["env"], spec["inherit_env"], directory)
    receipt.update({"state_path": str(directory / "inspect.stdout.json"), "mission_id": observed["mission_id"], "revision": observed["revision"], "observed_ns": time.time_ns()})
    return receipt


def prepare_invocation(run_dir, plan, stage, request_id, attempt_dir):
    kind = stage["kind"]
    dynamic_inputs = []
    env = stage.get("env", {})
    inherit = stage.get("inherit_env", [])
    if not isinstance(env, dict) or any(not isinstance(k, str) or not isinstance(v, str) for k, v in env.items()):
        raise ValueError("env must contain only string keys and values")
    if not isinstance(inherit, list) or any(not isinstance(k, str) for k in inherit):
        raise ValueError("inherit_env must be a list")
    if not (set(env) | set(inherit)) <= ENV_KEYS:
        raise ValueError("environment contains a non-allowlisted variable")
    # Freeze the actual selected inherited values before persisting the command.
    env = {**{key: os.environ[key] for key in inherit if key in os.environ}, **env}
    inherit = []
    if kind == "client":
        client = stage["client"]
        prefix = [resolve_token(x, run_dir, dynamic_inputs) for x in client["prefix"]]
        store = Path(replace_run(client["store"], run_dir)).resolve()
        if not store.is_relative_to(run_dir):
            raise ValueError("client store must be inside this workflow run")
        argv = prefix + [client["operation"], "--store", str(store)]
        inspect = None
        if client["operation"] not in ("start", "import-closed-attempt", "inspect"):
            observed, inspect = inspect_client(prefix, str(store), stage["cwd"], env, inherit, attempt_dir)
            argv += ["--mission-id", observed["mission_id"], "--expect-revision", str(observed["revision"])]
        if client["operation"] == "inspect":
            argv.append("--json")
        else:
            argv += ["--request-id", request_id, "--workflow-run-id", plan["workflow_run_id"], "--output", str(attempt_dir / "client_output")]
            for key, value in sorted(client.get("args", {}).items()):
                if not isinstance(value, (str, int, float)) or isinstance(value, bool):
                    raise ValueError("unsupported Astra argument value")
                argv += ["--" + key, resolve_token(str(value), run_dir, dynamic_inputs)]
            argv.append("--execute")
        check_argv(argv, client=True)
        if client['operation'] in ('start','resume'):
            from workflow_inputs import source_check,checked_json
            client_dir=HERE.parent/'goal_pilot_source'
            if str(client_dir) not in sys.path:sys.path.insert(0,str(client_dir))
            from mission_state import remaining_state
            remaining=(remaining_state(observed) if inspect is not None else
                checked_json(plan['input_binding']['policy'])['mission_budget_sec'])
            source=plan['input_binding']['source']
            source_check(source['path'],minimum_lease_sec=max(0.,remaining)+300.,expected_sha256=source['sha256'])
    else:
        argv = [resolve_token(x, run_dir, dynamic_inputs) for x in stage["argv"]]
        check_argv(argv)
        inspect = None
    spec = {"schema": "terra.workflow.invocation.v1", "stage_id": stage["id"], "kind": kind, "invocation_dir": str(attempt_dir), "argv": argv, "cwd": stage["cwd"], "env": env, "inherit_env": inherit, "dynamic_inputs": dynamic_inputs}
    if kind == "spawn":
        spec["ready"] = dict(stage["ready"], path=replace_run(stage["ready"]["path"], run_dir))
        if "same_fields" in spec["ready"]:
            spec["ready"]["same_fields"] = [dict(x, source_path=replace_run(x["source_path"], run_dir)) for x in spec["ready"]["same_fields"]]
            if any(not Path(x["source_path"]).resolve().is_relative_to(run_dir) for x in spec["ready"]["same_fields"]):
                raise ValueError("readiness identity sources must belong to this workflow run")
    return spec, inspect


def process_alive(record):
    try:
        actual = workflow_worker.identity(record["pid"])
        return all(actual[k] == record[k] for k in ("pid", "starttime_ticks", "boot_id", "pid_namespace"))
    except (OSError, ValueError, KeyError, ProcessLookupError):
        return False


def copy_once(source, target, expected):
    path, actual, raw = mission_ledger.read_stable(source, include_data=True)
    if actual != expected:
        raise ValueError("client evidence hash mismatch: " + path)
    if target.exists():
        if file_sha(target) != actual:
            raise ValueError("previous client evidence copy differs")
    else:
        write_once(target, raw)
    return str(target)


def client_output(plan, stage, invocation_dir, code):
    raw = (invocation_dir / "stdout.log").read_bytes()
    value = json.loads(raw)
    if stage["client"]["operation"] == "inspect":
        mission_ledger.require_fields(value, "matrix.mission.state.v1", mission_ledger.STATE_FIELDS)
        mission_ledger.canonical_uuid(value["mission_id"]); mission_ledger.revision(value["revision"])
        return {"outcome": "INSPECTED", "state_stdout_sha256": sha(raw), "mission_id": value["mission_id"], "revision": value["revision"]}
    mission_ledger.require_fields(value, "matrix.mission.operation.v1", mission_ledger.OPERATION_FIELDS)
    if value["operation"] != stage["client"]["operation"] or value["request_id"] != stage["request_id"] or value["workflow_run_id"] not in (None, plan["workflow_run_id"]):
        raise ValueError("client operation identity differs from invocation")
    if value["exit_code"] != code:
        raise ValueError("client exit code differs from operation record")
    op_path = invocation_dir / "operation.copy.json"
    if op_path.exists():
        if op_path.read_bytes() != raw:
            raise ValueError("operation copy differs")
    else:
        write_once(op_path, raw)
    state_path = result_path = None
    if value["state_path"] is not None:
        state_path = copy_once(value["state_path"], invocation_dir / "state.copy.json", value["state_sha256"])
    if value["result_path"] is not None:
        result_path = copy_once(value["result_path"], invocation_dir / "result.copy.json", value["result_sha256"])
    args = SimpleNamespace(client_state=state_path, client_operation=str(op_path), client_result=result_path)
    refs = mission_ledger.client_refs(args, plan["workflow_run_id"])
    return {"outcome": value["operation_outcome"], "references": refs, "reason": value["reason"], "operation_sha256": sha(raw)}


def budget_failure_for_closure(run_dir, stage, invocation_dir, spec, code):
    """Inspect the authoritative client after a minimal FAILED_BUDGET error.

    This never sends a goal. Closure still separately verifies the stop's
    native evidence, UUID, live ownership and recorder before signalling.
    """
    if code != 12 or stage['client']['operation'] not in ('start', 'resume'):
        return None
    raw = (invocation_dir / 'stdout.log').read_bytes()
    value = json.loads(raw)
    if (value.get('schema') != 'matrix.mission.operation.v1' or value.get('operation_outcome') != 'HELD'
            or 'FAILED_BUDGET' not in str(value.get('reason', ''))):
        return None
    directory = invocation_dir / 'post_budget_inspection'; directory.mkdir()
    prefix = [resolve_token(x, run_dir) for x in stage['client']['prefix']]
    store = str(Path(replace_run(stage['client']['store'], run_dir)).resolve())
    state, trace = inspect_client(prefix, store, stage['cwd'], spec['env'], spec['inherit_env'], directory)
    attempt, stop = state.get('attempt'), state.get('stop')
    if (not isinstance(attempt, dict) or attempt.get('terminal_confirmed') is not True
            or attempt.get('children_quiescent') is not True or attempt.get('stopped') is not True
            or not isinstance(stop, dict) or stop.get('state') != 'PASSED'
            or state.get('result_flags', {}).get('operational_complete') is True):
        raise ValueError('budget exhausted without confirmed terminal/children/measured stop')
    path = directory / 'inspect.stdout.json'
    return {'outcome': 'FAILED_BUDGET', 'reason': value['reason'],
            'references': {'state': {'path': str(path), 'sha256': file_sha(path)}},
            'inspection': trace, 'error_stdout_sha256': sha(raw)}


def init(plan_path, run_dir):
    raw = Path(plan_path).read_bytes()
    plan = validate_plan(json.loads(raw))
    run_dir = Path(run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=False)
    (run_dir / "stages").mkdir()
    write_once(run_dir / "execution_plan.json", raw)
    rows = []
    for stage in plan["stages"]:
        rows.append({"id": stage["id"], "state": "PENDING", "request_id": str(uuid.uuid4()) if stage["kind"] == "client" and stage["client"]["operation"] != "inspect" else None, "invocations": []})
    state = {"schema": "terra.workflow.state.v1", "workflow_run_id": plan["workflow_run_id"], "plan_sha256": sha(raw), "stages": rows, "created_ns": time.time_ns(), "mission_authority": "ASTRA_CLIENT"}
    atomic_json(run_dir / "workflow_state.json", state)
    print(json.dumps({"run_dir": str(run_dir), "plan_sha256": sha(raw), "mode": "PREPARED_NO_SUBPROCESS"}))


def read_run(run_dir):
    run_dir = Path(run_dir).resolve(strict=True)
    raw = (run_dir / "execution_plan.json").read_bytes()
    plan = validate_plan(json.loads(raw))
    state = json.loads((run_dir / "workflow_state.json").read_text())
    if state.get("schema") != "terra.workflow.state.v1" or state.get("plan_sha256") != sha(raw) or state.get("workflow_run_id") != plan["workflow_run_id"]:
        raise ValueError("workflow state/plan identity mismatch")
    if [x["id"] for x in state["stages"]] != [x["id"] for x in plan["stages"]]:
        raise ValueError("workflow stage list differs")
    return run_dir, plan, state


def status(run_dir):
    run_dir, plan, state = read_run(run_dir)
    print(json.dumps({"run_dir": str(run_dir), "plan_sha256": state["plan_sha256"], "mission_authority": "ASTRA_CLIENT", "stages": state["stages"]}, ensure_ascii=False, indent=2))


def monitor(run_dir, stage_id, approval_path):
    run_dir, plan, state = read_run(run_dir)
    approval_check(plan, approval_path)
    for stage, row in zip(plan["stages"], state["stages"]):
        if stage["id"] == stage_id:
            validate_files(stage.get("files", []))
            print(json.dumps(monitor_client(run_dir, stage, row), ensure_ascii=False, indent=2))
            return
    raise ValueError("unknown client stage")


def dry_run(run_dir):
    run_dir, plan, state = read_run(run_dir)
    print(json.dumps({"mode": "DRY_RUN_NO_SUBPROCESS", "workflow_run_id": plan["workflow_run_id"], "variant": plan["variant"], "commit": plan["commit"], "stages": [{"id": s["id"], "kind": s["kind"], "roles": s.get("roles", []), "state": r["state"], "client_operation": s.get("client", {}).get("operation")} for s, r in zip(plan["stages"], state["stages"])], "note": "No client, shell, SSH, ROS node or simulator command was started."}, ensure_ascii=False, indent=2))


def execute(run_dir, approval_path, follow, *, close_only=False):
    run_dir, plan, state = read_run(run_dir)
    lock = (run_dir / "workflow.lock").open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise ValueError("another workflow driver holds the lock")
    try:
        run_dir,plan,state=read_run(run_dir)
        admission = approval_check(plan, approval_path)
        validate_files(plan.get("inputs", []))
        if state.get("approval_sha256") not in (None, admission["sha256"]):
            raise ValueError("approval changed within workflow")
        state["approval_sha256"] = admission["sha256"]
        atomic_json(run_dir / "workflow_state.json", state)
        last_monitor = {}
        while True:
            progressed = False
            for stage, row in zip(plan["stages"], state["stages"]):
                if close_only and not set(stage.get('roles',[])) & {'closure','mcap','video','video_qa','acceptance'}:
                    continue
                if row["state"] == "SUCCEEDED":
                    continue
                if row["state"] == "HELD":
                    print(json.dumps({"state": "HELD", "stage": stage["id"], "reason": row.get("reason")})); return 10
                def dependency_ready(dep):
                    prior=next(s for s in plan['stages'] if s['id']==dep)
                    observed=next(x for x in state['stages'] if x['id']==dep)
                    if observed['state']=='SUCCEEDED':return True
                    return (close_only and 'closure' in stage.get('roles',[]) and
                        stage.get('exact_owned_cleanup') is True and 'mission' in prior.get('roles',[]) and
                        observed['state']=='HELD')
                if not all(dependency_ready(dep) for dep in stage.get('depends_on',[])):
                    print(json.dumps({"state": "HELD", "stage": stage["id"], "reason": "dependency not complete"})); return 10
                # Previously launched long-running actors must still be the same
                # processes through the mission, unless this is a closure stage.
                if "closure" not in stage.get("roles", []) and not any(x.get("closed") for x in state["stages"]):
                    for prior in state["stages"]:
                        if prior.get("actor_identity") and not process_alive(prior["actor_identity"]):
                            row.update(state="HELD", reason="previous actor is no longer the recorded process")
                            atomic_json(run_dir / "workflow_state.json", state)
                            print(json.dumps({"state": "HELD", "stage": stage["id"], "reason": row["reason"]})); return 10
                validate_files(stage.get("files", []))
                if row["state"] == "PENDING":
                    directory = stage_dir(run_dir, stage)
                    directory.mkdir(parents=True, exist_ok=True)
                    invocation = directory / ("invocation_%03d" % len(row["invocations"]))
                    invocation.mkdir()
                    effective = dict(stage)
                    if stage["kind"] == "client" and row.get("next_operation"):
                        effective["client"] = dict(stage["client"], operation=row["next_operation"])
                    try:
                        spec, inspect = prepare_invocation(run_dir, plan, effective, row["request_id"], invocation)
                    except Exception as exc:
                        row.update(state="HELD", reason=f"preparation failed: {exc}")
                        atomic_json(run_dir / "workflow_state.json", state)
                        print(json.dumps({"state": "HELD", "stage": stage["id"], "reason": row["reason"]})); return 10
                    spec_raw = (json.dumps(spec, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode()
                    spec_file = invocation / "invocation.json"
                    write_once(spec_file, spec_raw)
                    row.update(state="INTENT", intent_unix_ns=time.time_ns(), argv=spec["argv"], argv_sha256=sha(json.dumps(spec["argv"], ensure_ascii=False, separators=(",", ":")).encode()), inspect=inspect)
                    row["invocations"].append({"directory": str(invocation), "spec_sha256": sha(spec_raw), "request_id": row["request_id"], "operation": effective.get("client", {}).get("operation")})
                    atomic_json(run_dir / "workflow_state.json", state)
                    with (invocation / "worker.log").open("xb") as log:
                        subprocess.Popen([sys.executable, "-B", str(HERE / "workflow_worker.py"), "worker", str(spec_file), sha(spec_raw)], stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
                    progressed = True
                invocation = Path(row["invocations"][-1]["directory"])
                result_file = invocation / "worker_result.json"
                started_file = invocation / "worker_started.json"
                if not result_file.exists():
                    if started_file.exists():
                        started = json.loads(started_file.read_text())
                        if started.get("spec_sha256") != row["invocations"][-1]["spec_sha256"]:
                            row.update(state="HELD", reason="worker specification identity mismatch")
                        elif not process_alive(started["identity"]):
                            child_file = invocation / "child_identity.json"
                            child_live = child_file.exists() and process_alive(json.loads(child_file.read_text()))
                            row.update(state="HELD", reason="worker lost; child may still be active" if child_live else "worker lost; invocation outcome unknown, reconcile without replay")
                        else:
                            row["state"] = "RUNNING"
                    # A crash between durable INTENT and worker startup remains
                    # unknown. Do not reissue even when the client is idempotent.
                    elif time.time_ns() - row.get("intent_unix_ns", 0) > 5_000_000_000:
                        row.update(state="HELD", reason="worker start not evidenced; invocation outcome unknown")
                    atomic_json(run_dir / "workflow_state.json", state)
                    if row["state"] == "HELD":
                        print(json.dumps({"state": "HELD", "stage": stage["id"], "reason": row["reason"]})); return 10
                    if not follow:
                        print(json.dumps({"state": "RUNNING", "stage": stage["id"]})); return 0
                    if stage["kind"] == "client" and time.monotonic() - last_monitor.get(stage["id"], 0) >= 5:
                        last_monitor[stage["id"]] = time.monotonic()
                        try:
                            row["latest_client_observation"] = monitor_client(run_dir, stage, row)
                            atomic_json(run_dir / "workflow_state.json", state)
                        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError, subprocess.TimeoutExpired):
                            # Monitoring is observational; the client/worker's
                            # result and runtime guards remain authoritative.
                            pass
                    time.sleep(0.2)
                    break
                result = json.loads(result_file.read_text())
                if result.get("spec_sha256") != row["invocations"][-1]["spec_sha256"]:
                    row.update(state="HELD", reason="worker result specification mismatch")
                elif result.get("state") == "SPAWNED" and stage["kind"] == "spawn":
                    row.update(state="SUCCEEDED", actor_identity=result["actor_identity"], evidence=result["ready"])
                elif result.get("state") == "EXITED":
                    try:
                        code = result["returncode"]
                        if stage["kind"] == "client":
                            bound_stage = dict(stage, request_id=row["request_id"])
                            if row["invocations"][-1].get("operation"):
                                bound_stage["client"] = dict(stage["client"], operation=row["invocations"][-1]["operation"])
                            persisted_spec = workflow_worker.load_spec(invocation / 'invocation.json', row['invocations'][-1]['spec_sha256'])
                            response = budget_failure_for_closure(run_dir, bound_stage, invocation, persisted_spec, code)
                            if response is None:
                                response = client_output(plan, bound_stage, invocation, code)
                            row["client_response"] = response
                            if response["outcome"] not in stage["client"]["accepted_outcomes"]:
                                # A terminal task failure is still a failure, but
                                # it must be possible to seal its recording. Do
                                # not proceed on an unknown UUID or absent stop.
                                closure_outcomes = stage["client"].get("terminal_outcomes_for_closure", [])
                                ref = response.get("references", {}).get("state", {})
                                if response["outcome"] not in closure_outcomes or not ref.get("path"):
                                    raise ValueError("client outcome is " + str(response["outcome"]))
                                observed = json.loads(Path(ref["path"]).read_text())
                                attempt, stop = observed.get("attempt"), observed.get("stop")
                                if (not isinstance(attempt, dict) or attempt.get("terminal_confirmed") is not True
                                        or attempt.get("children_quiescent") is not True or not isinstance(stop, dict)
                                        or stop.get("state") != "PASSED" or observed.get("result_flags", {}).get("operational_complete") is True):
                                    raise ValueError("terminal client failure lacks terminal/child/stop closure")
                                row["mission_terminal_outcome"] = response["outcome"]
                                row["stage_success_means"] = "failure retained; recording closure only, mission not accepted"
                        elif code not in stage.get("accepted_exit_codes", [0]):
                            raise ValueError("command return code " + str(code))
                        row["evidence"] = output_conditions(stage.get("success_files", []), run_dir)
                        row["state"] = "SUCCEEDED"
                        if "closure" in stage.get("roles", []):
                            row["closed"] = True
                    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                        row.update(state="HELD", reason="output validation failed: " + str(exc))
                else:
                    row.update(state="HELD", reason=result.get("reason", "worker failed or action uncertain"))
                row["worker_result_sha256"] = file_sha(result_file)
                atomic_json(run_dir / "workflow_state.json", state)
                if row["state"] == "HELD":
                    print(json.dumps({"state": "HELD", "stage": stage["id"], "reason": row["reason"]})); return 10
                progressed = True
            else:
                print(json.dumps({"state": "WORKFLOW_CLOSURE_FINISHED" if close_only else "WORKFLOW_STAGES_FINISHED", "note": "This is stage completion, not mission or physical acceptance; original mission states are retained."})); return 0
            if not follow:
                print(json.dumps({"state": "PROGRESSED"})); return 0
    finally:
        lock.close()


def reconcile(run_dir, stage_id, approval_path, follow):
    """Ask only the client to reconcile its existing UUID, then re-enter resume."""
    run_dir, plan, state = read_run(run_dir)
    lock = (run_dir / "workflow.lock").open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise ValueError("another workflow driver holds the lock")
    try:
        run_dir,plan,state=read_run(run_dir)
        admission = approval_check(plan, approval_path)
        if state.get("approval_sha256") not in (None, admission["sha256"]):
            raise ValueError("approval changed within workflow")
        state["approval_sha256"] = admission["sha256"]
        match = [(s, r) for s, r in zip(plan["stages"], state["stages"]) if s["id"] == stage_id]
        if len(match) != 1:
            raise ValueError("unknown client stage")
        stage, row = match[0]
        if stage["kind"] != "client" or stage["client"]["operation"] not in ("start", "resume") or row["state"] != "HELD":
            raise ValueError("reconciliation requires a held start/resume client stage")
        validate_files(plan.get("inputs", [])); validate_files(stage.get("files", []))
        if "ready" not in stage["client"].get("args", {}):
            raise ValueError("reconciliation requires the accepted live readiness input")
        for previous in row.get("invocations", []):
            directory = Path(previous["directory"])
            for name in ("worker_started.json", "child_identity.json"):
                path = directory / name
                if path.exists():
                    record = json.loads(path.read_text())
                    if name == "worker_started.json": record = record["identity"]
                    if process_alive(record):
                        # A completed worker may need a moment to exit after
                        # fsyncing its receipt; never race a live client.
                        until = time.monotonic() + 1.0
                        while process_alive(record) and time.monotonic() < until:
                            time.sleep(0.05)
                        if process_alive(record):
                            raise ValueError("original client or worker is still alive")
        recoveries = row.setdefault("reconciliations", [])
        for previous in recoveries:
            old_dir = Path(previous["directory"])
            child_path = old_dir / "child_identity.json"
            if previous["state"] == "HELD" and child_path.exists() and process_alive(json.loads(child_path.read_text())):
                raise ValueError("earlier reconciliation child is still alive")
            if previous["state"] == "HELD" and not (old_dir / "worker_result.json").exists():
                raise ValueError("earlier reconciliation result is unknown; do not send another request")
        pending = recoveries[-1] if recoveries and recoveries[-1]["state"] in ("INTENT", "RUNNING") else None
        if pending is None:
            number = len(recoveries)
            directory = stage_dir(run_dir, stage) / ("reconcile_%03d" % number)
            while directory.exists():
                number += 1
                directory = stage_dir(run_dir, stage) / ("reconcile_%03d" % number)
            rec_id = (stage_id[:100] + "-reconcile-%03d" % number)
            directory.mkdir(parents=True, exist_ok=False)
            allowed = {"identity", "ready", "resource-lock", "reason", "odom-topic", "scan-topic", "simulation-scan-max-age-sec", "simulation-first-leg-max-distance-m", "camera-topic", "camera-info-topic"}
            client = dict(stage["client"], operation="reconcile", args={k: v for k, v in stage["client"].get("args", {}).items() if k in allowed}, accepted_outcomes=["CANCELED", "COMPLETED"])
            recovery_stage = dict(stage, id=rec_id, client=client)
            request_id = str(uuid.uuid4())
            spec, inspect = prepare_invocation(run_dir, plan, recovery_stage, request_id, directory)
            observed = json.loads((directory / "inspect.stdout.json").read_text())
            previous_attempt = observed.get("attempt")
            if not isinstance(previous_attempt, dict) or previous_attempt.get("parent_action_uuid") is None:
                raise ValueError("client does not expose an existing owned UUID; no reconciliation sent")
            old_uuid = mission_ledger.canonical_uuid(previous_attempt["parent_action_uuid"])
            raw = (json.dumps(spec, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode()
            write_once(directory / "invocation.json", raw)
            pending = {"state": "INTENT", "directory": str(directory), "spec_sha256": sha(raw), "request_id": request_id, "original_action_uuid": old_uuid, "stage": recovery_stage, "inspect": inspect, "intent_unix_ns": time.time_ns()}
            recoveries.append(pending)
            atomic_json(run_dir / "workflow_state.json", state)
            with (directory / "worker.log").open("xb") as log:
                subprocess.Popen([sys.executable, "-B", str(HERE / "workflow_worker.py"), "worker", str(directory / "invocation.json"), sha(raw)], stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
        directory = Path(pending["directory"])
        while True:
            result_file = directory / "worker_result.json"
            if result_file.exists():
                result = json.loads(result_file.read_text())
                try:
                    if result.get("spec_sha256") != pending["spec_sha256"] or result.get("state") != "EXITED":
                        raise ValueError("reconciliation worker did not complete cleanly")
                    bound = dict(pending["stage"], request_id=pending["request_id"])
                    response = client_output(plan, bound, directory, result["returncode"])
                    pending["response"] = response
                    if response["outcome"] not in ("CANCELED", "COMPLETED"):
                        raise ValueError("client reconciliation outcome is " + str(response["outcome"]))
                    state_path = response["references"].get("state", {}).get("path")
                    if not state_path:
                        raise ValueError("client reconciliation lacks a state receipt")
                    observed = json.loads(Path(state_path).read_text())
                    attempt = observed.get("attempt")
                    if not isinstance(attempt, dict) or mission_ledger.canonical_uuid(attempt.get("parent_action_uuid")) != pending["original_action_uuid"]:
                        raise ValueError("client reconciled a different or unknown action UUID")
                    stop = observed.get("stop")
                    if attempt.get("terminal_confirmed") is not True or attempt.get("children_quiescent") is not True or not isinstance(stop, dict) or stop.get("state") != "PASSED":
                        raise ValueError("client has not confirmed parent, children and stop")
                    pending["state"] = "RECONCILED"
                    pending["worker_result_sha256"] = file_sha(result_file)
                    flags = observed.get("result_flags")
                    if isinstance(flags, dict) and flags.get("operational_complete") is True:
                        row.update(state="SUCCEEDED", client_response=response, completion_source="client_reconciliation_state")
                    elif observed.get("lifecycle") in ("RECOVERABLE_FAILURE", "PAUSED_INPUT_HEALTH"):
                        # The client's next resume derives its own next index and
                        # budget. This is a new CLI request, not a new Action UUID.
                        row.update(state="PENDING", request_id=str(uuid.uuid4()), next_operation="resume", prior_held_reason=row.get("reason"))
                        row.pop("reason", None)
                    else:
                        row.update(state="HELD", recovery_forbidden=True,
                                   reason="client lifecycle " + str(observed.get("lifecycle")) + "; no automatic resume")
                    atomic_json(run_dir / "workflow_state.json", state)
                    if row["state"] == "HELD":
                        print(json.dumps({"state": "HELD", "stage": stage_id, "reason": row["reason"]})); return 10
                    print(json.dumps({"state": "RECONCILED", "stage": stage_id, "next_workflow_state": row["state"], "client_action_uuid": pending["original_action_uuid"]}))
                    return 0
                except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                    pending.update(state="HELD", reason=str(exc), worker_result_sha256=file_sha(result_file))
                    atomic_json(run_dir / "workflow_state.json", state)
                    print(json.dumps({"state": "HELD", "stage": stage_id, "reason": str(exc)})); return 10
            started = directory / "worker_started.json"
            if started.exists() and not process_alive(json.loads(started.read_text())["identity"]):
                pending.update(state="HELD", reason="reconciliation worker lost; outcome unknown")
                atomic_json(run_dir / "workflow_state.json", state)
                print(json.dumps({"state": "HELD", "stage": stage_id, "reason": pending["reason"]})); return 10
            if not started.exists() and time.time_ns() - pending["intent_unix_ns"] > 5_000_000_000:
                pending.update(state="HELD", reason="reconciliation start unknown")
                atomic_json(run_dir / "workflow_state.json", state)
                print(json.dumps({"state": "HELD", "stage": stage_id, "reason": pending["reason"]})); return 10
            pending["state"] = "RUNNING"
            atomic_json(run_dir / "workflow_state.json", state)
            if not follow:
                print(json.dumps({"state": "RUNNING", "stage": stage_id, "operation": "reconcile"})); return 0
            time.sleep(0.2)
    finally:
        lock.close()


def auto_reconcile_allowed(stage, row):
    if row.get("recovery_forbidden") or stage["kind"] != "client" or stage["client"].get("auto_reconcile") is not True:
        return False
    if stage["client"]["operation"] not in ("start", "resume"):
        return False
    response = row.get("client_response")
    if response is not None:
        outcome, reason = response.get("outcome"), str(response.get("reason") or "")
    else:
        outcome, reason = None, ""
        if row.get("invocations"):
            path = Path(row["invocations"][-1]["directory"]) / "stdout.log"
            try:
                raw = json.loads(path.read_text())
                outcome, reason = raw.get("operation_outcome"), str(raw.get("reason") or "")
            except (OSError, ValueError, AttributeError):
                pass
    if outcome is not None:
        # Explicit task decisions, cancellations, budget/identity failures and
        # completed-but-malformed results must never be converted into a retry.
        return outcome == "HELD" and any(token in reason.upper() for token in ("UNKNOWN_DISPATCH", "OPERATION INTERRUPTED", "TERMINAL RESULT REMAINS UNKNOWN", "CHILD_NOT_QUIESCENT", "CHILD_OR_STOP_UNPROVEN"))
    reason = str(row.get("reason", ""))
    return "outcome unknown" in reason or "worker lost" in reason or "worker start not evidenced" in reason


def dispatch_budget_exhaustion(client):
    # The canonical client owns this rule, including per-index parent count.
    client_dir=HERE.parent/'goal_pilot_source'
    if str(client_dir) not in sys.path:sys.path.insert(0,str(client_dir))
    from mission_state import dispatch_budget_exhaustion as allowance
    return allowance(client)


def retry_recoverable_when_ready(run_dir, approval_path, stage_id):
    """Follow the client's recoverable lifecycle, without judging waypoint progress."""
    run_dir, plan, state = read_run(run_dir)
    approval_check(plan, approval_path)
    stage, row = next((s, r) for s, r in zip(plan["stages"], state["stages"]) if s["id"] == stage_id)
    if stage["kind"] != "client" or stage["client"].get("auto_resume_when_ready") is not True or row["state"] != "HELD":
        return False
    response = row.get("client_response") or {}
    ref = response.get("references", {}).get("state", {})
    if response.get("outcome") != "HELD" or not ref.get("path") or file_sha(ref["path"]) != ref.get("sha256"):
        return False
    client = json.loads(Path(ref["path"]).read_text())
    attempt, stop, budget = client.get("attempt"), client.get("stop"), client.get("budgets")
    if (client.get("lifecycle") not in ("RECOVERABLE_FAILURE", "PAUSED_INPUT_HEALTH")
            or not isinstance(attempt, dict) or attempt.get("terminal_confirmed") is not True
            or attempt.get("children_quiescent") is not True or not isinstance(stop, dict) or stop.get("state") != "PASSED"
            or not isinstance(budget, dict)):
        return False
    if dispatch_budget_exhaustion(client):
        return False
    deadline = budget.get("deadline_boottime_sec")
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if deadline is not None:
        if budget.get("deadline_boot_id") != boot:
            return False
        until = float(deadline)
    else:
        remaining = budget.get("remaining_sec")
        if isinstance(remaining, bool) or not isinstance(remaining, (float, int)) or remaining <= 0:
            return False
        until = time.clock_gettime(time.CLOCK_BOOTTIME) + remaining
    ready_arg = stage["client"].get("args", {}).get("ready")
    if not isinstance(ready_arg, str):
        return False
    ready_path = resolve_token(ready_arg, run_dir)
    while time.clock_gettime(time.CLOCK_BOOTTIME) < until:
        try:
            proof = workflow_worker.readiness({"path": ready_path, "equals": {"ready": True, "waypoint_ownership.exclusive_nav_client": True,
                    "waypoint_ownership.navigate_to_pose_active": [], "waypoint_ownership.follow_waypoints_active": [],
                    "recovery_runtime.verified": True}, "timestamp_field": "time_ns", "max_age_sec": .5, "valid_until_field": "valid_until_ns"})
            break
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
            time.sleep(1)
    else:
        return False
    lock = (run_dir / "workflow.lock").open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        _, current_plan, current = read_run(run_dir)
        current_stage, current_row = next((s, r) for s, r in zip(current_plan["stages"], current["stages"]) if s["id"] == stage_id)
        current_ref = (current_row.get("client_response") or {}).get("references", {}).get("state", {})
        if current_row["state"] != "HELD" or current_ref.get("sha256") != ref["sha256"]:
            return False
        current_row.update(state="PENDING", request_id=str(uuid.uuid4()), next_operation="resume", prior_held_reason=current_row.get("reason"),
                           retry_ready_observation=proof)
        current_row.pop("reason", None)
        atomic_json(run_dir / "workflow_state.json", current)
        print(json.dumps({"state": "CLIENT_RECOVERABLE_READY", "stage": stage_id, "client_lifecycle": client["lifecycle"]}))
        return True
    finally:
        lock.close()


def close_exhausted_recoverable(run_dir, approval_path, stage_id):
    """Advance to closure only; retain the client's failed state and stop evidence."""
    run_dir, plan, state = read_run(run_dir)
    approval_check(plan, approval_path)
    stage, row = next((s, r) for s, r in zip(plan['stages'], state['stages']) if s['id'] == stage_id)
    response = row.get('client_response') or {}
    ref = response.get('references', {}).get('state', {})
    if (stage['kind'] != 'client' or stage['client'].get('auto_resume_when_ready') is not True
            or row['state'] != 'HELD' or response.get('outcome') != 'HELD'
            or not ref.get('path') or file_sha(ref['path']) != ref.get('sha256')):
        return False
    client = json.loads(Path(ref['path']).read_text())
    operation = response.get('references', {}).get('operation')
    if operation:
        if file_sha(operation['path']) != operation.get('sha256'):
            return False
        op = json.loads(Path(operation['path']).read_text())
        if (op.get('state_sha256') != ref['sha256'] or op.get('mission_id') != client.get('mission_id')
                or op.get('revision') != client.get('revision')):
            return False
    if client.get('lifecycle') not in ('RECOVERABLE_FAILURE', 'PAUSED_INPUT_HEALTH'):
        return False
    exhausted = dispatch_budget_exhaustion(client)
    if not exhausted:
        return False
    attempt, stop = client.get('attempt') or {}, client.get('stop') or {}
    if (attempt.get('terminal_confirmed') is not True or attempt.get('children_quiescent') is not True
            or attempt.get('stopped') is not True or stop.get('state') != 'PASSED'
            or stop.get('source_action_uuid') != attempt.get('parent_action_uuid')):
        return False
    evidence, receipt = stop.get('evidence') or {}, stop.get('receipt') or {}
    if (not evidence.get('path') or file_sha(evidence['path']) != evidence.get('sha256')
            or receipt.get('observed') is not True or receipt.get('strict_stop_issues') != []):
        return False
    native = json.loads(Path(evidence['path']).read_text())
    if (native.get('action_uuid') != attempt['parent_action_uuid'] or native.get('status') not in (4, 5, 6)
            or native.get('children_quiescent') is not True
            or native.get('owned_goal_shutdown', {}).get('stop') != receipt):
        return False
    lock = (run_dir / 'workflow.lock').open('a+')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        _, _, current = read_run(run_dir)
        current_row = next(x for x in current['stages'] if x['id'] == stage_id)
        current_ref = (current_row.get('client_response') or {}).get('references', {}).get('state', {})
        if current_row['state'] != 'HELD' or current_ref.get('sha256') != ref['sha256']:
            return False
        current_row.update(state='SUCCEEDED', mission_terminal_outcome='FAILED_BUDGET',
            budget_exhaustion_observation=dict(exhausted, state_sha256=ref['sha256'],
                                               stop_evidence=evidence, checked_unix_ns=time.time_ns()),
            stage_success_means='budget prevents another dispatch; closure only, mission not accepted')
        current_row.pop('reason', None)
        atomic_json(run_dir / 'workflow_state.json', current)
        print(json.dumps({'state':'CLIENT_BUDGET_EXHAUSTED_CLOSE_ONLY','stage':stage_id}))
        return True
    finally:
        lock.close()


def close_expired_recoverable(run_dir, approval_path, stage_id):
    """Permit evidence closure when a stopped client-owned deadline expires."""
    run_dir, plan, state = read_run(run_dir);approval_check(plan, approval_path)
    stage,row=next((s,r) for s,r in zip(plan['stages'],state['stages']) if s['id']==stage_id)
    response=row.get('client_response') or {};ref=response.get('references',{}).get('state',{})
    if (stage['kind']!='client' or row['state']!='HELD' or response.get('outcome')!='HELD'
            or not ref.get('path') or file_sha(ref['path'])!=ref.get('sha256')):return False
    client=json.loads(Path(ref['path']).read_text());attempt=client.get('attempt') or {};stop=client.get('stop') or {};budget=client.get('budgets') or {}
    deadline=budget.get('deadline_boottime_sec');boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    now=time.clock_gettime(time.CLOCK_BOOTTIME)
    if (client.get('lifecycle') not in ('RECOVERABLE_FAILURE','PAUSED_INPUT_HEALTH') or
        attempt.get('terminal_confirmed') is not True or attempt.get('children_quiescent') is not True or
        attempt.get('stopped') is not True or stop.get('state')!='PASSED' or
        not isinstance(deadline,(int,float)) or isinstance(deadline,bool) or budget.get('deadline_boot_id')!=boot or now<deadline):return False
    lock=(run_dir/'workflow.lock').open('a+')
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        _,_,current=read_run(run_dir);current_row=next(x for x in current['stages'] if x['id']==stage_id)
        current_ref=(current_row.get('client_response') or {}).get('references',{}).get('state',{})
        if current_row['state']!='HELD' or current_ref.get('sha256')!=ref['sha256']:return False
        current_row.update(state='SUCCEEDED',mission_terminal_outcome='FAILED_BUDGET',
            budget_expiry_observation=dict(deadline_boottime_sec=deadline,observed_boottime_sec=now,boot_id=boot,state_sha256=ref['sha256']),
            stage_success_means='client deadline elapsed; closure only, mission not accepted')
        current_row.pop('reason',None);atomic_json(run_dir/'workflow_state.json',current)
        print(json.dumps({'state':'CLIENT_BUDGET_EXPIRED_CLOSE_ONLY','stage':stage_id}));return True
    finally:lock.close()


def follow_with_reconciliation(run_dir, approval_path):
    """Continue after a client-owned reconciliation when the plan opts in."""
    while True:
        rc = execute(run_dir, approval_path, True)
        if rc != 10:
            return rc
        _, plan, state = read_run(run_dir)
        held = [(s, r) for s, r in zip(plan["stages"], state["stages"]) if r["state"] == "HELD"]
        if len(held) != 1:
            return rc
        stage, row = held[0]
        if close_exhausted_recoverable(run_dir, approval_path, stage["id"]):
            continue
        if retry_recoverable_when_ready(run_dir, approval_path, stage["id"]):
            continue
        if close_expired_recoverable(run_dir, approval_path, stage["id"]):
            continue
        if not auto_reconcile_allowed(stage, row):
            return rc
        # The client alone decides whether the old UUID, health, stop and
        # budget permit continuation. A held/unknown reconciliation stops here.
        if reconcile(run_dir, stage["id"], approval_path, True) != 0:
            return 10


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="action", required=True)
    i = sub.add_parser("init"); i.add_argument("--plan", required=True); i.add_argument("--run-dir", required=True)
    r = sub.add_parser("run"); r.add_argument("--run-dir", required=True); r.add_argument("--execute", action="store_true"); r.add_argument("--main-acceptance"); r.add_argument("--follow", action="store_true")
    s = sub.add_parser("status"); s.add_argument("--run-dir", required=True)
    m = sub.add_parser("monitor"); m.add_argument("--run-dir", required=True); m.add_argument("--client-stage", required=True); m.add_argument("--main-acceptance", required=True)
    c = sub.add_parser("reconcile"); c.add_argument("--run-dir", required=True); c.add_argument("--client-stage", required=True); c.add_argument("--execute", action="store_true"); c.add_argument("--main-acceptance"); c.add_argument("--follow", action="store_true")
    a = p.parse_args()
    if a.action == "init":
        init(a.plan, a.run_dir); return 0
    if a.action == "status":
        status(a.run_dir); return 0
    if a.action == "monitor":
        monitor(a.run_dir, a.client_stage, a.main_acceptance); return 0
    if a.action == "reconcile":
        if not a.execute:
            print(json.dumps({"mode": "DRY_RUN_NO_SUBPROCESS", "operation": "reconcile", "stage": a.client_stage})); return 0
        if not a.main_acceptance:
            raise ValueError("reconcile --execute requires --main-acceptance")
        return reconcile(a.run_dir, a.client_stage, a.main_acceptance, a.follow)
    if not a.execute:
        dry_run(a.run_dir); return 0
    if not a.main_acceptance:
        raise ValueError("--execute requires --main-acceptance")
    if a.follow:
        return follow_with_reconciliation(a.run_dir, a.main_acceptance)
    return execute(a.run_dir, a.main_acceptance, False)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError, subprocess.TimeoutExpired) as exc:
        print(f"HOLD: {exc}", file=sys.stderr)
        raise SystemExit(10)
