#!/usr/bin/env bash
set -e -o pipefail
probe_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${OFFICIAL_PROBE_CLEAN_ENV:-0}" != 1 ]]; then
  exec env -i HOME="$HOME" PATH=/usr/bin:/bin LANG=C.UTF-8 OFFICIAL_PROBE_CLEAN_ENV=1 \
    /bin/bash --noprofile --norc "$probe_root/run_planner_chain_isolated.sh" "$@"
fi
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=218 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_HOME="$probe_root/runtime_home" ROS_LOG_DIR="$probe_root/runtime_logs"
export PROBE_GOAL_XY=0.01 PROBE_RUN_TAG=_tight_xy
ulimit -c 0
mkdir -p "$probe_root/evidence" "$ROS_HOME" "$ROS_LOG_DIR"
probe_cpus="$(python3 -c 'import os;print(",".join(map(str,sorted(os.sched_getaffinity(0))[-2:])))')"
cmake -S "$probe_root" -B "$probe_root/build" > "$probe_root/evidence/chain_configure.log" 2>&1
nice -n 15 taskset -c "$probe_cpus" cmake --build "$probe_root/build" \
  --target global_planner_probe planner_controller_probe native_path_probe complete_rotation_tests -j1 \
  > "$probe_root/evidence/chain_build.log" 2>&1
ctest --test-dir "$probe_root/build" --output-on-failure > "$probe_root/evidence/chain_ctest.log" 2>&1
PROBE_PLANNER=LatticeActions PROBE_GLOBAL_TOLERANCE=0 \
  PROBE_LATTICE_FILE="$probe_root/inputs/global_planner/diagnostic_small_radius_same_actions.json" \
  PROBE_GLOBAL_TAG=_ab timeout 40s nice -n 15 taskset -c "$probe_cpus" \
  "$probe_root/build/global_planner_probe" "$probe_root" --ros-args --log-level error \
  > "$probe_root/evidence/global_ab_planner_repro.log" 2>&1
python3 "$probe_root/harness/run_ab_chain.py" > "$probe_root/evidence/chain_repro_stdout.jsonl" 2>&1
python3 - "$probe_root" <<'PY'
import json,pathlib,sys
p=pathlib.Path(sys.argv[1]);r=json.loads((p/'evidence/ab_chain_tight_xy_result.json').read_text())
assert r['all_exact_yaw_passed'], 'At least one adapted planner/controller scenario failed'
assert len(r['results'])==15
print(json.dumps({'adapted_cases_passed':10,'raw_controls_retained':5,
  'scope':'Local component closed loop only; no robot velocity output, board, or production switch'}))
PY
