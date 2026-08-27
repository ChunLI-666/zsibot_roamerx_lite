#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: matrix_closed_loop_preflight.sh sensors|closed-loop

  sensors      Check Matrix sensor topics before starting Lightning/Nav.
  closed-loop  Check Lightning ownership, TF, obstacle scan and command chain.
EOF
}

[[ $# -eq 1 ]] || { usage >&2; exit 2; }
PHASE=$1
[[ "$PHASE" == "sensors" || "$PHASE" == "closed-loop" ]] || {
  usage >&2
  exit 2
}

pass() { printf '[PASS] %s\n' "$*"; }
fail() { printf '[FAIL] %s\n' "$*" >&2; exit 1; }

topic_info() {
  ros2 topic info "$1" --verbose --no-daemon --spin-time 0.5 2>/dev/null || true
}

publisher_nodes() {
  topic_info "$1" | awk '
    /Node name:/ { node=$3 }
    /Endpoint type: PUBLISHER/ { print node }
  '
}

subscriber_nodes() {
  topic_info "$1" | awk '
    /Node name:/ { node=$3 }
    /Endpoint type: SUBSCRIPTION/ { print node }
  '
}

require_topic_type() {
  local topic=$1 expected=$2 actual='' deadline
  deadline=$((SECONDS + 10))
  while (( SECONDS < deadline )); do
    actual=$(timeout 4 ros2 topic type "$topic" \
      --no-daemon --spin-time 0.5 2>/dev/null || true)
    [[ "$actual" == "$expected" ]] && {
      pass "$topic type: $actual"
      return 0
    }
    sleep 0.2
  done
  fail "$topic type is '${actual:-missing}', expected '$expected'"
}

reject_ground_truth_tf() {
  local graph duplicate_nodes
  graph=$(topic_info /tf)
  if grep -Eq 'Node name: (pub_tf|tf_manager)' <<<"$graph"; then
    fail 'ground-truth pub_tf/tf_manager is publishing /tf'
  fi
  if grep -Eq 'Node name: (sim_tf_bridge|odom_to_tf_broadcaster)' <<<"$graph"; then
    fail 'a ground-truth odom bridge is publishing /tf'
  fi
  if pgrep -af '(^|/)(pub_tf|tf_manager)([[:space:]]|$)' >/dev/null; then
    fail 'ground-truth pub_tf/tf_manager process is running'
  fi
  duplicate_nodes=$(ros2 node list --no-daemon --spin-time 0.5 2>/dev/null |
    sort | uniq -d || true)
  [[ -z "$duplicate_nodes" ]] || fail "duplicate ROS node names: $duplicate_nodes"
  pass 'no ground-truth or duplicate navigation TF publisher detected'
}

require_single_publisher() {
  local topic=$1 expected=$2 nodes='' count=0 deadline
  deadline=$((SECONDS + 10))
  while (( SECONDS < deadline )); do
    nodes=$(publisher_nodes "$topic")
    count=$(grep -c . <<<"$nodes" || true)
    if [[ "$count" == "1" ]] && grep -Eq "^${expected}$" <<<"$nodes"; then
      pass "$topic has one publisher: $nodes"
      return 0
    fi
    sleep 0.2
  done
  fail "$topic publisher count is $count, expected '$expected' (${nodes:-none})"
}

require_subscriber() {
  local topic=$1 expected=$2 nodes='' deadline
  deadline=$((SECONDS + 10))
  while (( SECONDS < deadline )); do
    nodes=$(subscriber_nodes "$topic")
    if grep -Eq "^${expected}$" <<<"$nodes"; then
      pass "$topic is consumed by $expected"
      return 0
    fi
    sleep 0.2
  done
  fail "$topic has no expected subscriber '$expected' (${nodes:-none})"
}

reject_ground_truth_consumers() {
  local nodes bad
  nodes=$(subscriber_nodes /odom/mujoco_odom)
  bad=$(grep -Ev '^(matrix_closed_loop_e2e|_ros2cli_[^[:space:]]*)$' <<<"$nodes" || true)
  [[ -z "$bad" ]] || fail "ground truth is consumed by non-evaluation nodes: $bad"
  pass 'MuJoCo ground truth is evaluation-only'
}

reject_ground_truth_tf
require_topic_type /livox/lidar sensor_msgs/msg/PointCloud2
require_topic_type /imu/data_raw sensor_msgs/msg/Imu

if [[ "$PHASE" == "sensors" ]]; then
  require_topic_type /odom/mujoco_odom nav_msgs/msg/Odometry
  pass 'sensor ownership preflight complete; native probe verified liveness'
  exit 0
fi

require_topic_type /odom/current_pose nav_msgs/msg/Odometry
require_topic_type /lightning/loc_status std_msgs/msg/UInt8
require_topic_type /lightning/pose_valid std_msgs/msg/Bool
require_topic_type /laser_scan sensor_msgs/msg/LaserScan
require_topic_type /cmd_vel_safe geometry_msgs/msg/Twist

require_single_publisher /odom/current_pose lightning_slam
require_single_publisher /laser_scan matrix_pointcloud_scan_projector
require_subscriber /cmd_vel_safe matrix_vel_cmd_lcm_publisher
reject_ground_truth_consumers

pgrep -x mc_ctrl >/dev/null || fail 'mc_ctrl is not running; ROS-to-LCM is not a physical loop'
pass 'mc_ctrl process is running'

# Humble's action/lifecycle CLI discovery is unreliable for hidden endpoints
# in a composed container. The long-lived E2E node checks message liveness,
# localization NORMAL, TF, lifecycle state and NavigateToPose readiness in one
# native rclpy context instead of spawning independent one-shot CLI probes.
pass 'closed-loop ownership preflight complete; native E2E checks runtime readiness'
