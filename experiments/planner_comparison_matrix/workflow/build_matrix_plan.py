#!/usr/bin/env python3
"""Materialize an executable Matrix stage plan from freshly verified bindings.

This is preparation only: it never launches a node, simulator, or Action. The
source and postprocessing stages are explicit inputs because source leases and
the canonical client schema must not be guessed or silently reused.
"""

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import sys

import workflow_driver as driver


HERE = Path(__file__).resolve().parent
FOOTPRINT = "[[-0.647,-0.530],[0.647,-0.530],[0.647,0.530],[-0.647,0.530]]"


def pin(path):
    path = Path(path).resolve(strict=True)
    return {"path": str(path), "sha256": driver.file_sha(path)}


def absolute(value):
    path = Path(value)
    if not path.is_absolute():
        raise ValueError("binding path must be absolute: " + value)
    return str(path)


def build(binding):
    if binding.get("schema") != "terra.matrix.bindings.v1":
        raise ValueError("invalid Matrix binding schema")
    variant = binding["variant"]
    if variant not in ("segmented", "official"):
        raise ValueError("only A/B primary variants are supported")
    env = binding["env"]
    if env.get("ROS_DOMAIN_ID") != "185":
        raise ValueError("isolated Matrix ROS domain 185 is required")
    cwd = absolute(binding["cwd"])
    launcher = Path(absolute(binding["nav"]["launcher"]))
    client = Path(absolute(binding["mission"]["client"]))
    observer = Path(absolute(binding["planner_observer"]))
    renderer = Path(absolute(binding["renderer"]))
    phone_qa = Path(absolute(binding["phone_qa"]))
    if launcher.name != "matrix_navigation.py" or client.name != "mission_resume.py":
        raise ValueError("expected frozen Matrix launcher and Astra mission client")
    py = absolute(binding.get("python", sys.executable))
    nav = binding["nav"]
    mission = binding["mission"]
    map_identity = absolute(binding["map_identity"])
    map_dir = absolute(binding["map_dir"])
    map_yaml = str(Path(map_dir) / "map.yaml")
    required_topics = json.loads(Path(absolute(binding["required_topics"])).read_text())
    if binding["performance_topic"] not in required_topics:
        raise ValueError("required MCAP topics must include the actual performance topic")
    tools={name:Path(absolute(value)) for name,value in binding["tools"].items()}
    stages = []
    def append(stage):
        stage = copy.deepcopy(stage)
        stage["depends_on"] = [stages[-1]["id"]] if stages else []
        stages.append(stage)
    for stage in binding.get("environment_stages",[]):
        append(stage)
    append({"id": "source25_map", "kind": "command", "roles": ["map"], "cwd": cwd, "env": env,
            "argv": [py, "-B", str(tools["verify_source25_map.py"]), "--identity", map_identity, "--map-dir", map_dir,
                     "--reserve-bytes", str(binding["map_reserve_bytes"]), "--output", "{run_dir}/source25_map_check.json"],
            "files": [pin(tools["verify_source25_map.py"])],
            "success_files": [{"path": "{run_dir}/source25_map_check.json", "equals": {"ok": True}}]})
    for stage in binding.get("localization_stages",[]):
        append(stage)
    nav_files = [launcher, launcher.parent / "recording_deadline.py", launcher.parent / "recovery_navigation.py",
                 launcher.parent / "static_layer_snapshot.py", launcher.parent / "run_matrix_navigation.sh"]
    nav_files += [Path(x) for x in nav.get("additional_files", [])]
    nav_argv = [py, "-B", str(launcher), "--map", map_yaml, "--footprint", FOOTPRINT, "--padding", "0.15",
                "--variant", variant, "--output", "{run_dir}/nav", "--with-waypoints", "--with-mission-recovery",
                "--source-snapshot", absolute(nav["source_snapshot"]), "--build-receipt", absolute(nav["build_receipt"]),
                "--message-install", absolute(nav["message_install"]), "--recovery-install", absolute(nav["recovery_install"]),
                "--recovery-build-receipt", absolute(nav["recovery_build_receipt"]),
                "--" + variant + "-install", absolute(nav["selected_install"]), "--odom-topic", "/odom/current_pose",
                "--scan-topic", nav.get("scan_topic", "/laser_scan_leveled_nav_floor_v1"),
                "--simulation-scan-max-age-sec", str(nav.get("simulation_scan_max_age_sec", 0.5)),
                "--max-runtime-sec", str(nav.get("max_runtime_sec", 2400)), "--run"]
    if not nav.get("waypoint_install") or not nav.get("waypoint_build_receipt"):
        raise ValueError("guarded waypoint install and source-bound receipt are required")
    waypoint_receipt = Path(absolute(nav["waypoint_build_receipt"]))
    waypoint_module = launcher.parent / "waypoint_runtime.py"
    nav_files += [waypoint_module, waypoint_receipt]
    nav_argv += ["--waypoint-install", absolute(nav["waypoint_install"]),
                 "--waypoint-build-receipt", str(waypoint_receipt)]
    if nav.get("health_tf_latest_only", True):
        nav_argv.append("--health-tf-latest-only")
    if nav.get("lifecycle_install"):
        nav_argv += ["--lifecycle-install", absolute(nav["lifecycle_install"]), "--lifecycle-build-receipt", absolute(nav["lifecycle_build_receipt"])]
    append({"id": "navigation_recording", "kind": "spawn", "roles": ["runtime", "recording"], "cwd": cwd,
            "env": env, "argv": nav_argv, "files": [pin(x) for x in nav_files], "finite_owner_and_exact_cleanup": True,
            "ready": {"path": "{run_dir}/nav/ready.json", "equals": {"ready": True, "variant": variant, "recovery_runtime.verified": True},
                      "timestamp_field": "time_ns", "max_age_sec": 0.5, "valid_until_field": "valid_until_ns", "timeout_sec": nav.get("ready_timeout_sec", 180)}})
    append({"id": "planner_observer", "kind": "spawn", "roles": [], "cwd": cwd, "env": env,
            "argv": [py, "-B", str(tools["planner_observer_entry.py"]), "--observer", str(observer),
                     "--identity-json", "{run_dir}/nav/runtime_manifest.json", "--deadline", "{run_dir}/nav/recording_deadline.json",
                     "--run-id", "@json:{run_dir}/nav/recording_deadline.json#run_id", "--output", "{run_dir}/planner_timing",
                     "--variant", "A" if variant == "segmented" else "B", "--run"],
            "files": [pin(tools["planner_observer_entry.py"]), pin(observer)], "finite_owner_and_exact_cleanup": True,
            "ready": {"path": "{run_dir}/planner_timing/planner_results.jsonl", "format": "jsonl", "equals": {"event": "observer_ready"},
                      "same_fields": [{"field": "run_id", "source_path": "{run_dir}/nav/recording_deadline.json", "source_field": "run_id"}], "timeout_sec": 90}})
    performance_stage = copy.deepcopy(binding["performance_stage"])
    if performance_stage.get("kind") != "spawn":
        raise ValueError("performance recorder must start before the mission and prove readiness")
    append(performance_stage)
    client_files = [client, client.parent / "mission_state.py", client.parent / "waypoint_batch.py", client.parent / "waypoint_contract.py"]
    client_files += [Path(x) for x in mission.get("additional_files", [])]
    base_client = {"prefix": [py, "-B", str(client)], "store": "{run_dir}/mission.sqlite", "auto_reconcile": True, "auto_resume_when_ready": True}
    args = {"route": absolute(mission["route"]), "identity": mission["identity"], "policy": absolute(mission["policy"]),
            "ready": "{run_dir}/nav/ready.json", "odom-topic": "/odom/current_pose", "scan-topic": nav.get("scan_topic", "/laser_scan_leveled_nav_floor_v1"),
            "simulation-scan-max-age-sec": str(nav.get("simulation_scan_max_age_sec", 0.5)),
            "simulation-first-leg-max-distance-m": str(mission.get("first_leg_max_distance_m", 8.5))}
    if mission['mode']!='new':raise ValueError('New version evaluations require a new case; use original run for resume')
    operation='start'
    append({"id": "mission_execute", "kind": "client", "roles": ["mission"], "cwd": cwd, "env": env,
            "files": [pin(x) for x in client_files], "client": dict(base_client, operation=operation, args=args, accepted_outcomes=["COMPLETED"],
                    terminal_outcomes_for_closure=["NEEDS_TASK_DECISION", "CANCELED", "FAILED_BUDGET"])})
    append({"id": "close_navigation", "kind": "command", "roles": ["closure"], "exact_owned_cleanup": True, "cwd": cwd, "env": env,
            "argv": [py, "-B", str(HERE / "close_recording.py"), "--workflow-dir", "{run_dir}", "--runtime-stage", "navigation_recording",
                     "--observer-stage", "planner_observer", "--client-stage", "mission_execute", "--nav-dir", "{run_dir}/nav",
                     "--observer-summary", "{run_dir}/planner_timing/summary.json", "--marker-dir", "{run_dir}/closure_markers",
                     "--main-acceptance", absolute(binding["main_acceptance"]), "--output", "{run_dir}/navigation_closed.json", "--execute"],
            "files": [pin(HERE / "close_recording.py"), pin(HERE / "workflow_driver.py"), pin(HERE / "workflow_worker.py"), pin(HERE / "mission_ledger.py")],
            "success_files": [{"path": "{run_dir}/navigation_closed.json", "equals": {"complete": True}}]})
    for stage in binding.get("source_cleanup_stages",[]):
        append(stage)
    append({"id": "mcap_audit", "kind": "command", "roles": ["mcap"], "cwd": cwd, "env": env,
            "argv": [py, "-B", str(tools["verify_recording.py"]), "--bag", "{run_dir}/nav/bag", "--final-audit", "{run_dir}/nav/final_audit.json",
                     "--static-audit", "{run_dir}/nav/static_layer_recording_audit.json", "--required-topics", absolute(binding["required_topics"]),
                     "--output", "{run_dir}/mcap_audit.json"], "files": [pin(tools["verify_recording.py"])],
            "success_files": [{"path": "{run_dir}/mcap_audit.json", "equals": {"read_to_eof": True, "integrity_pass": True, "static_snapshot_pass": True}}]})
    for stage in binding["analysis_and_video_input_stages"]:
        append(stage)
    append({"id": "phone_render", "kind": "command", "roles": ["video"], "cwd": cwd, "env": env,
            "argv": [py, "-B", str(renderer), "--input", "{run_dir}/nav/bag", "--closure-evidence", "{run_dir}/mcap_audit.json",
                     "--map-yaml", map_yaml, "--runtime-params", "{run_dir}/nav/runtime_parameters.json", "--route", absolute(mission["route"]),
                     "--case-dir", "@json:{run_dir}/video_inputs.json#case_dir", "--start-ns", "@json:{run_dir}/video_inputs.json#start_ns",
                     "--end-ns", "@json:{run_dir}/video_inputs.json#end_ns", "--fps", "3", "--align-start-ready", "--output", "{run_dir}/phone"],
            "files": [pin(renderer), pin(renderer.parent / "original_render_mobile_review.py")],
            "success_files": [{"path": "{run_dir}/phone/video_metadata.json"}]})
    append({"id": "phone_qa", "kind": "command", "roles": ["video_qa"], "cwd": cwd, "env": env,
            "argv": [py, "-B", str(phone_qa), "--video-dir", "{run_dir}/phone"], "files": [pin(phone_qa)],
            "success_files": [{"path": "{run_dir}/phone/encoded_QA/QA.json", "equals": {"encoded_checks_pass": True}}]})
    acceptance = copy.deepcopy(binding["acceptance_stage"])
    if "acceptance" not in acceptance.get("roles", []):
        raise ValueError("acceptance stage must own the acceptance role")
    existing_outputs = {item.get("path") for item in acceptance.get("success_files", [])}
    for required in ("{run_dir}/acceptance.json", "{run_dir}/manifest.json", "{run_dir}/report.txt"):
        if required not in existing_outputs:
            acceptance.setdefault("success_files", []).append({"path": required})
    append(acceptance)
    required_roles = {role for stage in stages for role in stage.get("roles", [])}
    if "acceptance" not in required_roles or not binding["analysis_and_video_input_stages"]:
        raise ValueError("fresh environment, localization, source cleanup and analysis adapters are required")
    inputs = copy.deepcopy(binding.get("inputs", []))
    for path in (map_identity, mission["route"], mission["policy"], nav["source_snapshot"], binding["required_topics"]):
        inputs.append(pin(path))
    plan = {"schema": "terra.workflow.execution_plan.v1", "workflow_run_id": binding["workflow_run_id"],
            "variant": variant, "commit": binding["commit"], "build_receipts": binding["build_receipts"], "inputs": inputs, "stages": stages}
    # workflow.py prepare adds the fixed current source admission and validates.
    return plan


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bindings", type=Path, required=True); p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    plan = build(json.loads(a.bindings.read_text()))
    with a.output.open("x") as out:
        json.dump(plan, out, ensure_ascii=False, indent=2); out.write("\n"); out.flush(); os.fsync(out.fileno())
    print(json.dumps({"plan": str(a.output), "sha256": driver.file_sha(a.output), "stages": len(plan["stages"]), "started_processes": 0}))


if __name__ == "__main__":
    main()
