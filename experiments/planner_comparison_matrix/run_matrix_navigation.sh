#!/usr/bin/env bash
set -euo pipefail
# Keep externally supplied FastDDS peer/interface settings; remove inherited product overlays.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
set +u
source /opt/ros/jazzy/setup.bash
set -u
export AMENT_PREFIX_PATH=/opt/ros/jazzy
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec /usr/bin/python3 "$(dirname "$0")/matrix_navigation.py" "$@"
