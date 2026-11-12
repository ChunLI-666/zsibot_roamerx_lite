#!/usr/bin/env bash
set -e -o pipefail
probe_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
probe_variant="${1:-upstream}"
case "$probe_variant" in
  upstream) probe_target=official_runtime_probe ;;
  complete-rotation) probe_target=official_runtime_corrected_probe ;;
  *) echo 'Use upstream or complete-rotation' >&2; exit 2 ;;
esac
if [[ "${OFFICIAL_PROBE_CLEAN_ENV:-0}" != 1 ]]; then
  exec env -i HOME="$HOME" PATH=/usr/bin:/bin LANG=C.UTF-8 OFFICIAL_PROBE_CLEAN_ENV=1 \
    /bin/bash --noprofile --norc "$probe_root/run_isolated.sh" "$@"
fi
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=218
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_HOME="$probe_root/runtime_home"
export ROS_LOG_DIR="$probe_root/runtime_logs"
ulimit -c 0
mkdir -p "$probe_root/evidence" "$ROS_HOME" "$ROS_LOG_DIR"
probe_cpu_list="$(python3 -c 'import os;print(",".join(map(str,sorted(os.sched_getaffinity(0))[-2:])))')"
cmake -S "$probe_root" -B "$probe_root/build" > "$probe_root/evidence/repro_configure.log" 2>&1
nice -n 15 taskset -c "$probe_cpu_list" cmake --build "$probe_root/build" \
  --target "$probe_target" -j1 > "$probe_root/evidence/repro_build.log" 2>&1
python3 - "$probe_root" "$probe_target" <<'PY'
import pathlib,subprocess,sys
p=pathlib.Path(sys.argv[1]);dependencies=subprocess.check_output(['ldd',str(p/'build'/sys.argv[2])],text=True)
(p/'evidence/repro_runtime_libraries.log').write_text(dependencies)
if 'not found' in dependencies or '/home/charles/project/colcon_ws/install/' in dependencies:
    raise SystemExit('Rejected mixed or unresolved runtime libraries')
PY
timeout 40s nice -n 15 taskset -c "$probe_cpu_list" "$probe_root/build/$probe_target" \
  --ros-args --log-level error > "$probe_root/evidence/repro_runtime.log" 2>&1
python3 - "$probe_root" "$probe_variant" <<'PY'
import json,pathlib,sys
p=pathlib.Path(sys.argv[1]);rows=[]
for line in (p/'evidence/repro_runtime.log').read_text().splitlines():
    try: rows.append(json.loads(line))
    except ValueError: pass
summaries={r['summary']:r for r in rows if 'summary' in r}
for name in ['same_xy_180','near_terminal110','midway_90','door_straight','cancel_and_new_goal']:
    assert summaries[name]['success']==1,name
assert any(r.get('scenario')=='blocked_rotation' and r.get('rejected') for r in rows)
for name in ['legacy_invalid_goal_rejected','legacy_goal_feedback_result','legacy_cancel_result']:
    assert any(r.get('wire_test')==name and r.get('success') for r in rows),name
if sys.argv[2]=='complete-rotation':
    assert abs(90-summaries['midway_90']['resume_yaw_deg'])<=5,'Intermediate turn was not completed before translation'
report={'native_ros_checks_passed':True,'variant':sys.argv[2],'scenarios':summaries,
        'wire_tests':[r for r in rows if 'wire_test' in r],
        'scope':'Frozen official path-interface revision, Jazzy compatibility adapters, ideal kinematics and synthetic map. No planner, BT, SDK, robot, or production activation.'}
(p/'evidence/repro_result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False))
PY
