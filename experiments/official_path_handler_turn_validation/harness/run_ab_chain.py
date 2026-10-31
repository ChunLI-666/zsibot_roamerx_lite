#!/usr/bin/env python3
"""Exercise real planner CSVs with the new official controller in local isolation."""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
assert os.environ.get("ROS_DOMAIN_ID") == "218"
evidence = root / "evidence"
cases = [
    ("free_same_xy_180", [0, 0, 0], [0, 0, math.pi], "free"),
    ("free_near_110", [0, 0, 0], [.2, 0, math.radians(110)], "free"),
    ("synthetic_door_goal90", [0, 0, math.radians(15)], [1.5, 0, math.pi / 2], "door"),
    ("synthetic_l_corridor", [0, 0, 0], [1, 1, math.pi / 2], "corridor"),
    ("recorded_terminal110_static_map",
     [-3.8007497322782706, 4.658441093749194, 3.0504716917604155],
     [-3.789631256533969, 4.698352909931362, 1.1210367137250565], "office"),
]
results = []
cpus = ",".join(map(str, sorted(os.sched_getaffinity(0))[-2:]))
for variant in ("raw_planner", "exact_yaw", "exact_yaw_slow_turn"):
    for name, start, goal, map_kind in cases:
        env = os.environ.copy()
        env.pop("PROBE_APPEND_REQUESTED_GOAL", None)
        env.pop("PROBE_ROTATE_SPEED", None)
        if variant != "raw_planner":
            env["PROBE_APPEND_REQUESTED_GOAL"] = "1"
        if variant == "exact_yaw_slow_turn":
            env["PROBE_ROTATE_SPEED"] = "0.1"
        csv = evidence / f"global_{name}_LatticeActions_ab.csv"
        cmd = ["nice", "-n", "15", "taskset", "-c", cpus,
               str(root / "build/planner_controller_probe"), str(root), str(csv),
               *map(str, start), *map(str, goal), map_kind,
               "--ros-args", "--log-level", "error"]
        run = subprocess.run(cmd, env=env, text=True, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, timeout=35, check=False)
        log = evidence / f"ab_chain_{name}_{variant}.log"
        log.write_text(run.stdout)
        row = {"scene": name, "variant": variant, "returncode": run.returncode,
               "csv_sha256": hashlib.sha256(csv.read_bytes()).hexdigest(),
               "requested_start": start, "requested_goal": goal,
               "log": str(log.relative_to(root))}
        for line in run.stdout.splitlines():
            try:
                value = json.loads(line)
            except ValueError:
                continue
            if value.get("planner_controller_summary"):
                row.update(value)
        results.append(row)
        print(json.dumps(row), flush=True)

report = {
    "scope": "Local x86 component closed loop, real Jazzy Smac Lattice planner CSVs, frozen new official RPP/path-handler interface with complete-rotation helper patch; ideal body, static maps, no BT/SDK/hardware/board changes.",
    "planner_configuration": {
        "control_file": "inputs/global_planner/diagnostic_small_radius_same_actions.json",
        "metadata_turning_radius_m": .01,
        "analytic_expansion_max_length_m": .001,
        "tolerance_m": 0,
        "footprint_m": [[.30,.15],[.30,-.15],[-.30,-.15],[-.30,.15]],
        "footprint_padding_m": .035,
    },
    "exact_yaw_adaptation": "Append the requested yaw at the unchanged planner terminal XY; do not claim it is raw planner output. Actual motion is densely swept against the footprint.",
    "results": results,
    "all_exact_yaw_passed": all(r.get("success") == 1 for r in results
                                if r["variant"] != "raw_planner"),
}
(evidence / "ab_chain_result.json").write_text(json.dumps(report, indent=2) + "\n")
