#!/usr/bin/env python3
"""Adapt the old exact-PID closure to current owned identities and client receipts.

Default is inert. With Main's accepted build and --execute, signal only the
recorded planner observer and navigation launcher, at most once each. Never
cancel an Action here: the unified client owns that operation.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time

import mission_ledger
import workflow_driver as driver
import workflow_worker


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    return driver.file_sha(path)


def actor(plan, state, run, stage_id):
    rows = [x for x in state["stages"] if x["id"] == stage_id]
    stages = [x for x in plan["stages"] if x["id"] == stage_id]
    if len(rows) != 1 or rows[0].get("state") != "SUCCEEDED" or not rows[0].get("actor_identity"):
        raise ValueError("missing recorded owned actor: " + stage_id)
    if len(stages) != 1 or stages[0].get("kind") != "spawn" or rows[0].get("argv") != [driver.resolve_token(x, run) for x in stages[0]["argv"]]:
        raise ValueError("owned actor does not match its frozen spawn stage")
    return rows[0]


def client_state(state, stage_id):
    rows = [x for x in state["stages"] if x["id"] == stage_id]
    if len(rows) != 1:
        raise ValueError("missing client stage")
    ref = rows[0].get("client_response", {}).get("references", {}).get("state", {})
    path, expected = ref.get("path"), ref.get("sha256")
    if not path or not expected or digest(path) != expected:
        raise ValueError("missing or changed client state reference")
    result = read(path)
    mission_ledger.require_fields(result, "matrix.mission.state.v1", mission_ledger.STATE_FIELDS)
    mission_ledger.canonical_uuid(result["mission_id"])
    mission_ledger.revision(result["revision"])
    attempt, stop = result.get("attempt"), result.get("stop")
    if not isinstance(attempt, dict) or attempt.get("terminal_confirmed") is not True or attempt.get("children_quiescent") is not True:
        raise ValueError("client has not confirmed parent and children terminal")
    if not isinstance(stop, dict) or stop.get("state") != "PASSED":
        raise ValueError("client has not confirmed measured stop")
    if mission_ledger.canonical_uuid(stop.get("source_action_uuid")) != mission_ledger.canonical_uuid(attempt.get("parent_action_uuid")):
        raise ValueError("client stop receipt is bound to a different action")
    observed = stop.get("receipt")
    if not isinstance(observed, dict) or observed.get("observed") is not True or observed.get("strict_stop_issues") != []:
        raise ValueError("client native measured/commanded stop receipt is incomplete")
    evidence = stop.get("evidence")
    if not isinstance(evidence, dict):
        raise ValueError("client stop evidence binding is unavailable")
    receipt, receipt_hash = evidence.get("path"), evidence.get("sha256")
    if not receipt or not receipt_hash or digest(receipt) != receipt_hash:
        raise ValueError("client stop receipt is missing or changed")
    return {"state_path": path, "state_sha256": expected, "mission_id": result["mission_id"],
            "revision": result["revision"], "action_uuid": attempt.get("parent_action_uuid"),
            "stop_receipt_path": receipt, "stop_receipt_sha256": receipt_hash}


def closure_ready_proof(nav_dir, expected_run):
    ready = read(nav_dir / "ready.json")
    now = time.time_ns()
    age = (now - ready["time_ns"]) / 1e9
    ownership = ready["waypoint_ownership"]
    reason=ready.get('reason')
    source_failure=(reason=='Localization is not NORMAL with valid pose' or isinstance(reason,str) and
        reason.startswith(('Missing source ','Stale receive ','Stale/future stamp ','Stale/future actual TF ')))
    if (ready.get('ready') is not True and not source_failure):
        raise ValueError('Closure cannot accept unknown/ownership/private-ingress readiness failure')
    if (not -0.05 <= age <= 0.5 or not 0<ready["valid_until_ns"]-now<=550000000
            or ready["mission_deadline"]["run_id"] != expected_run
            or ownership.get("navigate_to_pose_active") != [] or ownership.get("follow_waypoints_active") != []
            or ownership.get('follow_path_active')!=[] or ownership.get('follow_path_status_publishers_match') is not True
            or ownership.get('status_publishers_match') is not True
            or not -.05<=(now-ownership.get('checked_unix_ns',0))/1e9<=.5
            or ready.get('recovery_runtime',{}).get('verified') is not True
            or ownership.get("exclusive_nav_client") is not True):
        raise ValueError("fresh live readiness or exclusive idle action proof unavailable")
    return {"ready_time_ns": ready["time_ns"], "checked_unix_ns": now, "run_id": expected_run,
            'source_health_ready':ready.get('ready'),'source_health_reason':reason,
            'ownership_checked_unix_ns':ownership['checked_unix_ns'],
            "navigate_to_pose_active": [], "follow_waypoints_active": [],'follow_path_active':[]}


def identical(current, expected):
    keys = ("pid", "starttime_ticks", "boot_id", "pid_namespace", "cmdline_sha256")
    return all(current.get(k) == expected.get(k) for k in keys)


def signal_owned(identity, marker, role, proof):
    if marker.exists():
        if driver.process_alive(identity):
            raise ValueError(role + " signal was already attempted; outcome uncertain")
        return {"role": role, "state": "PREVIOUS_SIGNAL_TARGET_ABSENT", "marker": str(marker)}
    if not driver.process_alive(identity):
        return {"role": role, "state": "TARGET_ALREADY_ABSENT", "marker": None}
    before = workflow_worker.identity(identity["pid"])
    if not identical(before, identity):
        raise ValueError(role + " process identity changed")
    fd = os.pidfd_open(identity["pid"])
    try:
        signal.pidfd_send_signal(fd, 0)
        if not identical(workflow_worker.identity(identity["pid"]), identity):
            raise ValueError(role + " process changed after pidfd open")
        intent = {"role": role, "identity": identity, "signal": "SIGINT", "proof": proof,
                  "requested_unix_ns": time.time_ns(), "signal_limit": 1}
        workflow_worker.write_once(marker, intent)
        signal.pidfd_send_signal(fd, signal.SIGINT)
        workflow_worker.write_once(marker.with_name(marker.stem + ".sent.json"), {"role": role, "sent_unix_ns": time.time_ns(), "intent_sha256": digest(marker)})
        return {"role": role, "state": "SIGINT_SENT_ONCE", "marker": str(marker)}
    finally:
        os.close(fd)


def wait_for(predicate, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = predicate()
        if value:
            return value
        time.sleep(0.2)
    raise ValueError("timed out waiting for owned closure evidence")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--workflow-dir", type=Path, required=True)
    p.add_argument("--runtime-stage", required=True); p.add_argument("--observer-stage", required=True)
    p.add_argument("--client-stage", required=True); p.add_argument("--nav-dir", type=Path, required=True)
    p.add_argument("--observer-summary", type=Path, required=True); p.add_argument("--marker-dir", type=Path, required=True)
    p.add_argument("--main-acceptance", type=Path, required=True); p.add_argument("--output", type=Path, required=True)
    p.add_argument("--timeout-sec", type=float, default=180.0); p.add_argument("--execute", action="store_true")
    a = p.parse_args()
    if not a.execute:
        print(json.dumps({"state": "DRY_RUN_NO_SIGNAL", "signals_sent": 0, "goals_sent": 0})); return 0
    run, plan, state = driver.read_run(a.workflow_dir)
    approval = driver.approval_check(plan, a.main_acceptance)
    if state.get("approval_sha256") != approval["sha256"]:
        raise ValueError("workflow approval identity differs")
    if not 0 < a.timeout_sec <= 300:
        raise ValueError("closure timeout outside bound")
    nav_dir = a.nav_dir.resolve(strict=True)
    if not nav_dir.is_relative_to(run):
        raise ValueError("navigation output must belong to workflow run")
    proof = client_state(state, a.client_stage)
    nav_stage, observer_stage = actor(plan, state, run, a.runtime_stage), actor(plan, state, run, a.observer_stage)
    nav, observer = nav_stage["actor_identity"], observer_stage["actor_identity"]
    if nav["pid"] == observer["pid"]:
        raise ValueError("observer and navigation identity cannot be the same")
    deadline = read(nav_dir / "recording_deadline.json")
    recorded = deadline["launcher"]
    if any(recorded.get(k) != nav.get(k) for k in ("pid", "starttime_ticks", "boot_id", "cmdline_sha256")):
        raise ValueError("navigation identity differs from its recording deadline")
    run_id = deadline["run_id"]
    def option(argv, flag):
        if argv.count(flag) != 1:
            raise ValueError("owned stage lacks unique " + flag)
        return argv[argv.index(flag) + 1]
    if (Path(option(nav_stage["argv"], "--output")).resolve() != nav_dir
            or option(observer_stage["argv"], "--run-id") != run_id
            or Path(option(observer_stage["argv"], "--output")).resolve() != a.observer_summary.resolve().parent
            or not a.observer_summary.resolve().is_relative_to(run)
            or not a.output.resolve().is_relative_to(run)):
        raise ValueError("owned output, observer run or closure output differs")
    proof["readiness"] = closure_ready_proof(nav_dir, run_id)
    marker_dir = a.marker_dir.resolve()
    if not marker_dir.is_relative_to(run):
        raise ValueError("closure markers must belong to workflow run")
    marker_dir.mkdir(parents=True, exist_ok=True)
    records = [signal_owned(observer, marker_dir / "observer.once.json", "observer", proof)]
    def footer():
        try:
            value = read(a.observer_summary)
            return value if value.get("footer_delivery", {}).get("delivery_confirmed") is True else None
        except (OSError, ValueError, json.JSONDecodeError):
            return None
    summary = wait_for(footer, min(20.0, a.timeout_sec))
    proof["observer_footer_sha256"] = digest(a.observer_summary)
    proof["readiness_after_footer"] = closure_ready_proof(nav_dir, run_id)
    records.append(signal_owned(nav, marker_dir / "navigation.once.json", "navigation", proof))
    def closed():
        try:
            shutdown = read(nav_dir / "recording_deadline_shutdown.json")
            audit = read(nav_dir / "final_audit.json")
            events = [json.loads(line) for line in (nav_dir / "events.jsonl").read_text().splitlines() if line.strip()]
            owned = [x for x in events if x.get("kind") == "owned_processes_closed"]
            codes = owned[-1].get("returncodes", {}) if owned else {}
            valid = (not driver.process_alive(nav) and not driver.process_alive(observer)
                     and shutdown.get("stage") == "recording_closed" and shutdown.get("run_id") == run_id
                     and bool(audit.get("mcap_sha256")) and (nav_dir / "bag/metadata.yaml").is_file()
                     and isinstance(codes, dict) and bool(codes) and all(x == 0 for x in codes.values())
                     and (nav_dir / "ready.json").is_file() and not (nav_dir / "ready.json").is_symlink())
            return {"shutdown": shutdown, "audit_sha256": digest(nav_dir / "final_audit.json"), "returncodes": codes} if valid else None
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
            return None
    closure = wait_for(closed, a.timeout_sec)
    result = {"schema": "terra.recording.exact_closure.v1", "complete": True, "run_id": run_id,
              "client": proof, "signals": records, "observer_footer_sha256": digest(a.observer_summary),
              "closure": closure, "goals_sent": 0, "physical_acceptance": False, "finished_unix_ns": time.time_ns()}
    workflow_worker.write_once(a.output, result)
    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print("HOLD: " + str(exc), file=sys.stderr)
        raise SystemExit(10)
