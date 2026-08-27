#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: matrix_closed_loop_run.sh --matrix-root DIR --map FILE [options]

Runs Matrix sensors and physics, Lightning localization, Navigo, recording,
and a scored A-to-B test. MuJoCo ground truth is recorded/evaluated only.

Options:
  --workspace DIR          Colcon workspace (default: inferred from Matrix root)
  --lightning-config FILE  Matrix Lightning config
  --params-file FILE       Navigo parameters (default: package config)
  --robot ID               Matrix robot ID (default: 1 / xgb)
  --scene ID               Matrix scene ID (default: 1 / warehouse)
  --relative-x METERS      Test goal forward offset (default: 1.0)
  --relative-y METERS      Test goal lateral offset (default: 0.0)
  --relative-yaw RADIANS   Test goal yaw offset (default: 0.0)
  --timeout SECONDS        Navigation timeout (default: 120)
  --result-dir DIR         Result directory
  --headless               Use UE off-screen rendering (default: visible window)
  --no-sim                 Reuse an already-running Matrix simulator
  --no-record              Do not record a rosbag
  --keep-running           Leave simulation/navigation running after the test
EOF
}

MATRIX_ROOT=""
MAP_FILE=""
WORKSPACE=""
LIGHTNING_CONFIG=""
PARAMS_FILE=""
ROBOT_ID=1
SCENE_ID=1
RELATIVE_X=1.0
RELATIVE_Y=0.0
RELATIVE_YAW=0.0
NAV_TIMEOUT=120
RESULT_DIR=""
START_SIM=1
RECORD=1
KEEP_RUNNING=0
RENDER_MODE=visible

while [[ $# -gt 0 ]]; do
  case "$1" in
    --matrix-root) MATRIX_ROOT=$2; shift 2 ;;
    --map) MAP_FILE=$2; shift 2 ;;
    --workspace) WORKSPACE=$2; shift 2 ;;
    --lightning-config) LIGHTNING_CONFIG=$2; shift 2 ;;
    --params-file) PARAMS_FILE=$2; shift 2 ;;
    --robot) ROBOT_ID=$2; shift 2 ;;
    --scene) SCENE_ID=$2; shift 2 ;;
    --relative-x) RELATIVE_X=$2; shift 2 ;;
    --relative-y) RELATIVE_Y=$2; shift 2 ;;
    --relative-yaw) RELATIVE_YAW=$2; shift 2 ;;
    --timeout) NAV_TIMEOUT=$2; shift 2 ;;
    --result-dir) RESULT_DIR=$2; shift 2 ;;
    --headless) RENDER_MODE=offscreen; shift ;;
    --no-sim) START_SIM=0; shift ;;
    --no-record) RECORD=0; shift ;;
    --keep-running) KEEP_RUNNING=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf '[FAIL] unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

MATRIX_IFACE=$(ip -o -4 address show | awk '$4 == "192.168.234.1/32" { print $2; exit }')
if [[ -z "$MATRIX_IFACE" ]]; then
  printf '[FAIL] Matrix control IP is missing. Run once:\n' >&2
  printf '  sudo ip link add matrix0 type dummy\n' >&2
  printf '  sudo ip address add 192.168.234.1/32 dev matrix0\n' >&2
  printf '  sudo ip link set matrix0 up\n' >&2
  exit 2
fi
if [[ "$MATRIX_IFACE" == "lo" ]]; then
  printf '[FAIL] 192.168.234.1 must not share lo with 127.0.0.1; UE DDS will crash.\n' >&2
  printf 'Move it to a dedicated dummy interface named matrix0.\n' >&2
  exit 2
fi

if [[ -z "${XAUTHORITY:-}" && -r "/run/user/$(id -u)/gdm/Xauthority" ]]; then
  export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
fi

USABLE_DISPLAY=""
for candidate in "${DISPLAY:-}" :0 :1 :10; do
  [[ -n "$candidate" ]] || continue
  if DISPLAY="$candidate" xdpyinfo >/dev/null 2>&1; then
    USABLE_DISPLAY=$candidate
    break
  fi
done
[[ -n "$USABLE_DISPLAY" ]] || {
  printf '[FAIL] no usable X display for MuJoCo (tried current DISPLAY, :0, :10)\n' >&2
  exit 2
}
export DISPLAY=$USABLE_DISPLAY
if [[ "$RENDER_MODE" == "visible" ]] && ! command -v xdotool >/dev/null 2>&1; then
  printf '[FAIL] xdotool is required to verify the visible Matrix window\n' >&2
  exit 2
fi

[[ -n "$MATRIX_ROOT" && -d "$MATRIX_ROOT" ]] || {
  printf '[FAIL] --matrix-root must name the Matrix repository\n' >&2; exit 2;
}
MATRIX_ROOT=$(realpath "$MATRIX_ROOT")
[[ -f "$MATRIX_ROOT/scripts/run_sim.sh" ]] || {
  printf '[FAIL] missing safe Matrix entry: %s/scripts/run_sim.sh\n' "$MATRIX_ROOT" >&2; exit 2;
}
[[ -n "$MAP_FILE" && -f "$MAP_FILE" ]] || {
  printf '[FAIL] --map must name map.yaml\n' >&2; exit 2;
}
MAP_FILE=$(realpath "$MAP_FILE")
MAP_DIR=$(dirname "$MAP_FILE")
[[ -f "$MAP_DIR/index.txt" ]] || {
  printf '[FAIL] Lightning map index is missing: %s/index.txt\n' "$MAP_DIR" >&2; exit 2;
}
[[ -f "$MAP_DIR/0.pcd" ]] || {
  printf '[FAIL] Lightning map chunk is missing: %s/0.pcd\n' "$MAP_DIR" >&2; exit 2;
}
if [[ -f "$MAP_DIR/SHA256SUMS" ]]; then
  if ! (cd "$MAP_DIR" && sha256sum --check --quiet SHA256SUMS); then
    printf '[FAIL] Lightning map archive checksum verification failed: %s\n' \
      "$MAP_DIR/SHA256SUMS" >&2
    exit 2
  fi
  printf '[PASS] Lightning map archive checksums verified\n'
fi

if [[ -z "$WORKSPACE" ]]; then
  WORKSPACE=$(realpath "$MATRIX_ROOT/../../..")
fi
if [[ -z "$LIGHTNING_CONFIG" ]]; then
  LIGHTNING_CONFIG="$WORKSPACE/src/lightning-lm/config/sim_matrix.yaml"
fi
[[ -f "$LIGHTNING_CONFIG" ]] || {
  printf '[FAIL] Lightning config not found: %s\n' "$LIGHTNING_CONFIG" >&2; exit 2;
}

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    ROS_DISTRO=jazzy
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    ROS_DISTRO=humble
  else
    printf '[FAIL] no supported ROS installation found\n' >&2; exit 2
  fi
fi
set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$WORKSPACE/install/setup.bash"
set -u

PANGOLIN_LIB="$WORKSPACE/src/lightning-lm/thirdparty/Pangolin-0.9.3/build"
if [[ -d "$PANGOLIN_LIB" ]]; then
  export LD_LIBRARY_PATH="$PANGOLIN_LIB:${LD_LIBRARY_PATH:-}"
fi

if [[ -z "$PARAMS_FILE" ]]; then
  PACKAGE_PREFIX=$(ros2 pkg prefix robot_navigo)
  PARAMS_FILE="$PACKAGE_PREFIX/share/robot_navigo/params/navigo_params.yaml"
fi
[[ -f "$PARAMS_FILE" ]] || {
  printf '[FAIL] Navigo params not found: %s\n' "$PARAMS_FILE" >&2; exit 2;
}

if [[ -z "$RESULT_DIR" ]]; then
  RESULT_DIR="$WORKSPACE/log/matrix_closed_loop_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$RESULT_DIR"
RESULT_DIR=$(realpath "$RESULT_DIR")
RUNTIME_CONFIG="$RESULT_DIR/sim_matrix.runtime.yaml"
RUNTIME_MAP="$RESULT_DIR/map.runtime.yaml"
RUNTIME_NAV_PARAMS="$RESULT_DIR/matrix_navigo.runtime.yaml"
sed -E "s|^([[:space:]]*map_path:).*|\1 $MAP_DIR/|" \
  "$LIGHTNING_CONFIG" > "$RUNTIME_CONFIG"

MAP_IMAGE=$(awk '$1 == "image:" { print $2; exit }' "$MAP_FILE")
[[ -n "$MAP_IMAGE" ]] || { printf '[FAIL] map image is missing\n' >&2; exit 2; }
if [[ "$MAP_IMAGE" != /* ]]; then
  MAP_IMAGE="$MAP_DIR/$MAP_IMAGE"
fi
MAP_IMAGE=$(realpath "$MAP_IMAGE")
sed -E \
  -e "s|^image:.*|image: $MAP_IMAGE|" \
  -e 's|^free_thresh:.*|free_thresh: 0.25|' \
  "$MAP_FILE" > "$RUNTIME_MAP"

# Matrix TF is sensor-time correct but arrives roughly 0.2s after the scan
# (measured P95 0.265s). Preserve the 0.3s TF lookup budget, while making the
# simulator-only freshness thresholds cover the observed tail latency. Real
# robot parameters remain unchanged.
python3 - "$PARAMS_FILE" "$RUNTIME_NAV_PARAMS" <<'PY'
import sys
import yaml

source_path, output_path = sys.argv[1:]
with open(source_path, encoding='utf-8') as source:
    params = yaml.safe_load(source)
for costmap_name in ('local_costmap', 'global_costmap'):
    costmap = params[costmap_name][costmap_name]['ros__parameters']
    costmap['transform_tolerance'] = 0.3
    costmap['obstacle_layer']['scan']['expected_update_rate'] = 0.5
params['controller_server']['ros__parameters']['costmap_update_timeout'] = 0.6
params['controller_server']['ros__parameters']['FollowPath']['transform_tolerance'] = 0.5
with open(output_path, 'w', encoding='utf-8') as output:
    yaml.safe_dump(params, output, sort_keys=False)
PY

# TiledMap resolves chunks from system.map_path and deliberately ignores the
# historical path column in index.txt. Use the archive itself as cwd so a run
# cannot accidentally depend on an older data/new_map directory.
LIGHTNING_CWD="$MAP_DIR"

SIM_PID=""
NAV_PID=""
LOCALIZATION_PID=""
BAG_PID=""
MATRIX_MC_CONFIG="$MATRIX_ROOT/src/robot_mc/build/export/config/robot-defaults.yaml"
MATRIX_MC_CONFIG_BACKUP=""

stop_group() {
  local pid=$1
  [[ -n "$pid" ]] || return 0
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 -- "-$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
}

stop_bag() {
  local pid=$1
  [[ -n "$pid" ]] || return 0
  # rosbag2 finalizes metadata on SIGINT. Give it enough time to flush instead
  # of falling through the generic two-second hard-kill path.
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  for _ in $(seq 1 100); do
    if ! kill -0 -- "-$pid" 2>/dev/null; then
      wait "$pid" 2>/dev/null || true
      return 0
    fi
    sleep 0.1
  done
  stop_group "$pid"
  wait "$pid" 2>/dev/null || true
}

matrix_process_pids() {
  local pid process_cwd process_exe
  for pid in $(pgrep -x zsibot_mujoco 2>/dev/null || true) \
             $(pgrep -x mc_ctrl 2>/dev/null || true); do
    process_cwd=$(readlink -f "/proc/$pid/cwd" 2>/dev/null || true)
    process_exe=$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)
    if [[ "$process_cwd" == "$MATRIX_ROOT"/* || "$process_cwd" == "$MATRIX_ROOT" ||
          "$process_exe" == "$MATRIX_ROOT"/* ]]; then
      printf '%s\n' "$pid"
    fi
  done
  pgrep -f "^${MATRIX_ROOT}/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux/zsibot_mujoco_ue-Linux-Shipping" \
    2>/dev/null || true
}

stop_matrix_processes() {
  local signal=$1 pids
  pids=$(matrix_process_pids)
  [[ -n "$pids" ]] || return 0
  # Matrix run_sim.sh starts these children in the background and installs its
  # traps after wait, so they may be reparented to PID 1 on external shutdown.
  kill "-$signal" $pids 2>/dev/null || true
}

cleanup_owned_matrix() {
  [[ "$START_SIM" == "1" ]] || return 0
  stop_matrix_processes TERM
  for _ in $(seq 1 20); do
    [[ -z "$(matrix_process_pids)" ]] && return 0
    sleep 0.1
  done
  stop_matrix_processes KILL
}

prepare_matrix_autonomy_input() {
  [[ "$START_SIM" == "1" ]] || return 0
  [[ -f "$MATRIX_MC_CONFIG" ]] || {
    printf '[FAIL] Matrix MC config not found: %s\n' "$MATRIX_MC_CONFIG" >&2
    return 1
  }
  grep -Eq '^[[:space:]]*use_gamepad[[:space:]]*:' "$MATRIX_MC_CONFIG" || {
    printf '[FAIL] Matrix MC config has no use_gamepad setting: %s\n' \
      "$MATRIX_MC_CONFIG" >&2
    return 1
  }
  MATRIX_MC_CONFIG_BACKUP="$RESULT_DIR/robot-defaults.original.yaml"
  cp "$MATRIX_MC_CONFIG" "$MATRIX_MC_CONFIG_BACKUP"
  sed -Ei 's/^([[:space:]]*use_gamepad[[:space:]]*:[[:space:]]*).*/\1 0/' \
    "$MATRIX_MC_CONFIG"
}

restore_matrix_input_config() {
  [[ -n "$MATRIX_MC_CONFIG_BACKUP" && -f "$MATRIX_MC_CONFIG_BACKUP" ]] || return 0
  cp "$MATRIX_MC_CONFIG_BACKUP" "$MATRIX_MC_CONFIG"
  MATRIX_MC_CONFIG_BACKUP=""
}

cleanup() {
  local status=$?
  if [[ "$KEEP_RUNNING" == "0" ]]; then
    stop_bag "$BAG_PID"
    stop_group "$LOCALIZATION_PID"
    stop_group "$NAV_PID"
    stop_group "$SIM_PID"
    cleanup_owned_matrix
  fi
  restore_matrix_input_config
  exit "$status"
}
trap cleanup EXIT INT TERM

wait_for_topic() {
  local topic=$1 timeout_sec=$2 deadline
  deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    # Matrix sensor publishers use BEST_EFFORT. A RELIABLE probe discovers the
    # topic but never receives a sample, which looks like a startup failure.
    if timeout 3 ros2 topic echo "$topic" --once \
        --qos-reliability best_effort >/dev/null 2>&1; then
      return 0
    fi
  done
  return 1
}

start_matrix() {
  local log_file=$1
  local ros_underlay="/opt/ros/$ROS_DISTRO/setup.bash"
  local -a matrix_args=(
    bash "$MATRIX_ROOT/scripts/run_sim.sh" "$ROBOT_ID" "$SCENE_ID")
  if [[ "$RENDER_MODE" == "offscreen" ]]; then
    matrix_args+=(offrender)
  fi
  printf '[RUN] starting Matrix with pure ROS %s underlay\n' "$ROS_DISTRO"
  # Matrix bundles ROS libraries in its UE executable and its helper nodes are
  # built against the system ROS installation. Overlay libraries from
  # Lightning/Nav can otherwise produce intermittent typesupport failures.
  setsid bash --noprofile --norc -c '
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
    unset LD_LIBRARY_PATH PYTHONPATH RMW_IMPLEMENTATION
    source "$1"
    shift
    exec "$@"
  ' _ "$ros_underlay" "${matrix_args[@]}" \
    >"$log_file" 2>&1 &
  SIM_PID=$!
}

wait_for_visible_matrix_window() {
  [[ "$RENDER_MODE" == "visible" ]] || return 0
  local deadline=$((SECONDS + 60)) pid window_id window_name
  while (( SECONDS < deadline )); do
    for pid in $(pgrep -f '^.*/zsibot_mujoco_ue-Linux-Shipping' 2>/dev/null || true); do
      window_id=$(xdotool search --onlyvisible --pid "$pid" 2>/dev/null | head -1 || true)
      if [[ -n "$window_id" ]]; then
        window_name=$(xdotool getwindowname "$window_id" 2>/dev/null || true)
        xdotool windowactivate --sync "$window_id" 2>/dev/null || true
        printf '[PASS] Matrix window visible on DISPLAY=%s: id=%s title=%s\n' \
          "$DISPLAY" "$window_id" "${window_name:-unknown}" | \
          tee "$RESULT_DIR/window_info.txt"
        return 0
      fi
    done
    sleep 1
  done
  printf '[FAIL] Matrix UE process has no visible X11 window on DISPLAY=%s\n' \
    "$DISPLAY" | tee "$RESULT_DIR/window_info.txt" >&2
  return 1
}

verify_recording() {
  local bag_dir=$1 info_file="$RESULT_DIR/bag_info.txt" topic
  ros2 bag info "$bag_dir" >"$info_file"
  local -a required_topics=(
    /livox/lidar /imu/data_raw /odom/current_pose /lightning/debug
    /lightning/loc_status /tf /tf_static /map
    /matrix_closed_loop/goal_pose /plan /transformed_global_plan /trajectories
    /local_costmap/costmap /global_costmap/costmap /controller_server/debug
    /laser_scan /lightning_bridge/debug /cmd_vel_nav /cmd_vel /cmd_vel_safe
    /nav_safety_gate/gate_status)
  for topic in "${required_topics[@]}"; do
    if ! grep -Fq "Topic: $topic |" "$info_file"; then
      printf '[FAIL] recorded bag is missing required topic: %s\n' "$topic" >&2
      return 1
    fi
  done
  printf '[PASS] recorded bag contains all required localization/navigation topics\n'
}

sensor_ready=0
if [[ "$START_SIM" == "1" ]]; then
  prepare_matrix_autonomy_input
  cleanup_owned_matrix
  for attempt in 1 2; do
    matrix_log="$RESULT_DIR/matrix.log"
    [[ "$attempt" == "1" ]] || matrix_log="$RESULT_DIR/matrix.retry.log"
    start_matrix "$matrix_log"
    if wait_for_topic /livox/lidar 60 &&
        wait_for_topic /imu/data_raw 20 &&
        wait_for_topic /odom/mujoco_odom 20 &&
        ros2 run robot_navigo matrix_closed_loop_preflight.sh sensors \
          >"$RESULT_DIR/preflight_sensors.attempt${attempt}.log" 2>&1; then
      sensor_ready=1
      break
    fi
    stop_group "$SIM_PID"
    cleanup_owned_matrix
    SIM_PID=""
    if [[ "$attempt" == "1" ]]; then
      printf '[WARN] Matrix sensors did not become ready; retrying once\n' >&2
      sleep 5
    fi
  done
else
  if wait_for_topic /livox/lidar 60 &&
      wait_for_topic /imu/data_raw 20 &&
      wait_for_topic /odom/mujoco_odom 20 &&
      ros2 run robot_navigo matrix_closed_loop_preflight.sh sensors \
        >"$RESULT_DIR/preflight_sensors.log" 2>&1; then
    sensor_ready=1
  fi
fi

[[ "$sensor_ready" == "1" ]] || {
  printf '[FAIL] Matrix sensors unavailable; see %s/matrix*.log\n' "$RESULT_DIR" >&2
  exit 1
}
if [[ "$START_SIM" == "1" ]]; then
  cp "$RESULT_DIR/preflight_sensors.attempt${attempt}.log" \
    "$RESULT_DIR/preflight_sensors.log"
fi
cat "$RESULT_DIR/preflight_sensors.log"
if ! wait_for_visible_matrix_window; then
  exit 1
fi
# mc_ctrl reads this setting only during startup. Restore the user's manual
# simulator configuration as soon as autonomous input ownership is established.
restore_matrix_input_config

printf '[RUN] starting Lightning + Navigo closed loop\n'
setsid ros2 launch robot_navigo matrix_lightning_closed_loop.launch.py \
  lightning_config:="$RUNTIME_CONFIG" \
  lightning_cwd:="$LIGHTNING_CWD" \
  map:="$RUNTIME_MAP" \
  params_file:="$RUNTIME_NAV_PARAMS" \
  start_lightning:=false \
  >"$RESULT_DIR/navigation.log" 2>&1 &
NAV_PID=$!

STAND_SERVICE=/matrix_vel_cmd_lcm_publisher/stand_up
printf '[RUN] standing Matrix robot before localization initialization\n'
stand_ok=0
for stand_attempt in 1 2 3; do
  if timeout 30 ros2 service call "$STAND_SERVICE" std_srvs/srv/Trigger '{}' \
      >"$RESULT_DIR/standup.attempt${stand_attempt}.log" 2>&1; then
    cp "$RESULT_DIR/standup.attempt${stand_attempt}.log" "$RESULT_DIR/standup.log"
    stand_ok=1
    break
  fi
done
if [[ "$stand_ok" != "1" ]]; then
  printf '[FAIL] Matrix stand-up request failed\n' >&2
  cat "$RESULT_DIR"/standup.attempt*.log >&2
  exit 1
fi
sleep 4

printf '[RUN] starting Lightning after stand-up settling\n'
setsid bash -c 'cd "$1" && exec ros2 run lightning run_loc_online \
  --config "$2" --ros-args -p use_sim_time:=false' \
  _ "$LIGHTNING_CWD" "$RUNTIME_CONFIG" \
  >"$RESULT_DIR/localization.log" 2>&1 &
LOCALIZATION_PID=$!

# Endpoint ownership does not require a valid pose. Run it while FP/NDT is
# initializing so the bounded ACTIVE-state global-match holdover is reserved
# for the navigation test instead of CLI discovery.
if ! timeout 180 ros2 run robot_navigo matrix_closed_loop_preflight.sh closed-loop \
    >"$RESULT_DIR/preflight_closed_loop.log" 2>&1; then
  printf '[FAIL] closed-loop preflight failed\n' >&2
  cat "$RESULT_DIR/preflight_closed_loop.log" >&2
  exit 1
fi
cat "$RESULT_DIR/preflight_closed_loop.log"

if ! wait_for_topic /odom/current_pose 180; then
  printf '[FAIL] Lightning produced no navigation odometry within 180s\n' >&2
  grep -E '\[INIT\]|初始化|NDT|FP候选' "$RESULT_DIR/localization.log" | tail -40 >&2 || true
  exit 1
fi

if [[ "$RECORD" == "1" ]]; then
  STORAGE_ARGS=()
  if ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1; then
    STORAGE_ARGS=(-s mcap)
  fi
  setsid ros2 bag record --include-hidden-topics "${STORAGE_ARGS[@]}" \
    -o "$RESULT_DIR/closed_loop_bag" \
    /livox/lidar /imu/data_raw /odom/current_pose /odom/mujoco_odom \
    /lightning/loc_status /lightning/pose_valid /lightning/debug \
    /tf /tf_static /map /plan /transformed_global_plan /trajectories \
    /local_costmap/costmap /global_costmap/costmap \
    /controller_server/debug /laser_scan /lightning_bridge/debug \
    /cmd_vel_nav /cmd_vel /cmd_vel_safe /nav_safety_gate/gate_status \
    /matrix_closed_loop/goal_pose /navigate_to_pose/_action/status \
    /navigate_to_pose/_action/feedback \
    >"$RESULT_DIR/rosbag.log" 2>&1 &
  BAG_PID=$!
  sleep 2
fi

set +e
ros2 run robot_navigo matrix_closed_loop_e2e.py \
  --relative-x "$RELATIVE_X" \
  --relative-y "$RELATIVE_Y" \
  --relative-yaw "$RELATIVE_YAW" \
  --timeout "$NAV_TIMEOUT" \
  --output "$RESULT_DIR/result.json" \
  2>&1 | tee "$RESULT_DIR/e2e.log"
TEST_STATUS=${PIPESTATUS[0]}
set -e

stop_bag "$BAG_PID"
BAG_PID=""
if [[ "$RECORD" == "1" ]] && ! verify_recording "$RESULT_DIR/closed_loop_bag"; then
  TEST_STATUS=1
fi
printf '[RESULT] artifacts: %s\n' "$RESULT_DIR"
exit "$TEST_STATUS"
