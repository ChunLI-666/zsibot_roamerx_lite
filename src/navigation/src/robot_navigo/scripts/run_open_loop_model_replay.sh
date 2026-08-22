#!/usr/bin/env bash
# Compare MPPI motion models against recorded pose, TF, and LaserScan inputs.
# This is open-loop: recorded robot motion is replayed and no robot command
# transport node is launched.
set -eo pipefail

usage() {
  cat <<'EOF'
Usage: run_open_loop_model_replay.sh --model {DiffDrive|Omni} --output DIR [options]

Options:
  --bag DIR       Replay input bag.
  --map FILE      Navigation map YAML.
  --params FILE   Base navigation parameters.
  --goal YAML     NavigateToPose goal YAML.
  --start-offset SEC
                  Seconds from the input bag start at which playback begins.
  --duration SEC  Playback duration in seconds.
EOF
}

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../../../../.." && pwd)"
BAG="/home/charles/datasets/rock_dog/0718/rosbag2_2026_07_18-05_59_29"
MAP="${ROOT_DIR}/data/hz_office_2f_part_0718/map.yaml"
PARAMS="${ROOT_DIR}/src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/params/navigo_params.yaml"
RELAY="${ROOT_DIR}/docs/progress/2026-08-01_control_replay_ab/replay_timestamp_relay.py"
GOAL="{pose: {header: {frame_id: map}, pose: {position: {x: -4.299777984619141, y: -4.1937665939331055, z: 0.0}, orientation: {z: -0.8845188639714154, w: 0.4665044257868479}}}}"
MODEL=""
OUTPUT=""
START_OFFSET=120
DURATION=160

while [[ $# -gt 0 ]]; do
  case "$1" in
    --model) MODEL="$2"; shift 2 ;;
    --output) OUTPUT="$2"; shift 2 ;;
    --bag) BAG="$2"; shift 2 ;;
    --map) MAP="$2"; shift 2 ;;
    --params) PARAMS="$2"; shift 2 ;;
    --goal) GOAL="$2"; shift 2 ;;
    --start-offset) START_OFFSET="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$MODEL" == "DiffDrive" || "$MODEL" == "Omni" ]] || {
  echo "--model must be DiffDrive or Omni" >&2; exit 2;
}
[[ -n "$OUTPUT" ]] || { echo "--output is required" >&2; exit 2; }
[[ -d "$BAG" && -f "$MAP" && -f "$PARAMS" && -f "$RELAY" ]] || {
  echo "Missing replay input, map, parameters, or timestamp relay" >&2; exit 2;
}
[[ ! -e "$OUTPUT" ]] || { echo "Output already exists: $OUTPUT" >&2; exit 2; }

source /opt/ros/jazzy/setup.bash
source "${ROOT_DIR}/install/setup.bash"

mkdir -p "$OUTPUT"
TEMP_PARAMS="$(mktemp --suffix=.yaml)"
PIDS=()

cleanup() {
  local code=$?
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  rm -f "$TEMP_PARAMS"
  exit "$code"
}
trap cleanup EXIT INT TERM

# Keep all parameters identical except the model. DiffDrive ignores vy even
# though the end-to-end command contract remains configured for Omni.
sed "s/motion_model: \"[A-Za-z]*\"/motion_model: \"${MODEL}\"/" "$PARAMS" > "$TEMP_PARAMS"

ros2 bag play "$BAG" --clock 100 --start-paused --disable-keyboard-controls \
  --start-offset "$START_OFFSET" --playback-duration "$DURATION" \
  --topics /tf /tf_static /odom/current_pose /laser_scan \
  --remap /tf:=/replay/tf_raw /odom/current_pose:=/replay/odom_raw /laser_scan:=/replay/scan_raw \
  >"${OUTPUT}/player.log" 2>&1 &
PIDS+=("$!")
sleep 1

python3 "$RELAY" --ros-args -p use_sim_time:=true \
  >"${OUTPUT}/relay.log" 2>&1 &
PIDS+=("$!")

ros2 launch robot_navigo bringup_launch.py \
  use_sim_time:=true map:="$MAP" params_file:="$TEMP_PARAMS" \
  >"${OUTPUT}/navigation.log" 2>&1 &
PIDS+=("$!")

for _ in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -qx /navigate_to_pose; then
    break
  fi
  sleep 1
done
ros2 action list 2>/dev/null | grep -qx /navigate_to_pose || {
  echo "NavigateToPose action server did not start" >&2; exit 1;
}

ros2 bag record -o "${OUTPUT}/output" \
  /tf /odom/current_pose /laser_scan /plan /transformed_global_plan /trajectories \
  /local_costmap/costmap_raw /global_costmap/costmap_raw \
  /cmd_vel_nav /cmd_vel /controller_server/debug \
  >"${OUTPUT}/recorder.log" 2>&1 &
PIDS+=("$!")
sleep 1

ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}' \
  >"${OUTPUT}/resume.log" 2>&1

# The action endpoint can be discoverable before its lifecycle node is active.
# Wait for TF-driven activation so the test goal is not rejected spuriously.
for _ in $(seq 1 30); do
  if ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q 'active'; then
    break
  fi
  sleep 1
done
ros2 lifecycle get /bt_navigator >"${OUTPUT}/bt_navigator_state.txt" 2>&1 || true
grep -q 'active' "${OUTPUT}/bt_navigator_state.txt" || {
  echo "bt_navigator did not become active" >&2; exit 1;
}

timeout "${DURATION}" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "${GOAL}" >"${OUTPUT}/action.log" 2>&1 || true

sleep 2
echo "model=${MODEL}" > "${OUTPUT}/run_metadata.txt"
echo "input_bag=${BAG}" >> "${OUTPUT}/run_metadata.txt"
echo "start_offset=${START_OFFSET}" >> "${OUTPUT}/run_metadata.txt"
echo "duration=${DURATION}" >> "${OUTPUT}/run_metadata.txt"
echo "map=${MAP}" >> "${OUTPUT}/run_metadata.txt"
echo "base_params=${PARAMS}" >> "${OUTPUT}/run_metadata.txt"
echo "Replay complete: ${OUTPUT}/output"
