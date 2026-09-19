# Production controller feedback experiments

This is a test-only executable. It loads the actual `navigo_core::Controller`
plugin with pluginlib and calls `computeVelocityCommands()`. The returned command
passes through the configured SDK-style deadbands and speed caps, then drives an
ideal SE(2) plant; that new pose and executed velocity are the next controller
input. It does not start a robot command bridge, publish actuator topics, launch
Matrix, or modify host networking. `COLCON_IGNORE` keeps this standalone test
project out of normal colcon discovery.

`prepare_cases.py` builds cases using the archived warehouse map and three 0829
recorded local-path geometries. Warehouse occupancy is frozen; unknown pixels
remain unknown (`free_thresh=0.196`, matching the existing Matrix runner).
Inflation is precomputed identically for all variants. The costmap does not load
an active InflationLayer, so MPPI emits its missing-layer optimization warning;
full footprint collision checking remains enabled. The footprint is the current
0.60 x 0.30 m rectangle plus 0.01 m padding on each side, identical in all arms.

The plant uses an independent rectangle/cell separating-axis collision oracle.
It checks polygon interiors, unknown cells, and map boundaries at 0.01 s
substeps. `--self-test` exercises those cases, including an obstacle missed by a
boundary-only checker. This is a discretized planar collision oracle, not a
measured leg-swing envelope or a quadruped dynamics simulation.

The controller and an actual `StoppedGoalChecker` receive the same XY/yaw/stopped
thresholds in both groups (0.25 m, 0.25 rad, 0.01 m/s, 0.01 rad/s; stateful=false).
Success additionally requires the current command to be stopped. The harness
executes neither a BT nor a progress checker or NavigateToPose action. It cannot
validate behavior recovery, command ownership, perception freshness, or hardware
response. A deliberate 5 s clock gap triggers controller reset in speed-limit
cases, while the test robot remains stationary during that gap.

0829 feedback trials start from the recorded pose with a stationary plant and
follow the first recorded **local path endpoint**, not the full original mission
goal. Their bounded costmap is empty: historical costmaps and map/odom transforms
are not reconstructed. These cases preserve the difficult heading/path geometry
without claiming to reproduce the physical office scene. Shadow trials instead
feed recorded poses/velocities and original source timestamps with that fixed
first local path; their commands do not drive the subsequent recorded pose.
They must not be scored as closed-loop arrival tests.

## Reproduce

From the canonical workspace (no worktree is required):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
TEST_SOURCE=src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/controller_feedback
TEST_OUTPUT=analysis_outputs/feature_navigation_20260908
mkdir -p "$TEST_OUTPUT"
touch "$TEST_OUTPUT/COLCON_IGNORE"
cmake -S "$TEST_SOURCE" -B "$TEST_OUTPUT/harness_build"
cmake --build "$TEST_OUTPUT/harness_build" -j2
"$TEST_OUTPUT/harness_build/controller_feedback" --self-test
python3 "$TEST_SOURCE/prepare_cases.py" --workspace . --output "$TEST_OUTPUT/cases"
python3 "$TEST_SOURCE/run_cases.py" --root "$TEST_OUTPUT" --variant candidate
python3 "$TEST_SOURCE/run_cases.py" --root "$TEST_OUTPUT" --variant feature_disabled
python3 "$TEST_SOURCE/analyze_results.py" --root "$TEST_OUTPUT"
```

The baseline variant additionally requires the existing frozen
`$TEST_OUTPUT/baseline_install/navigo_mppi_controller` directory. It is an
**as-built snapshot** of the installation taken before this change, not a newly
compiled source baseline. `run_cases.py --variant baseline` selects this prefix
and its libraries. The primary controlled comparison is `feature_disabled` vs
`candidate`, which loads the same freshly built feature libraries with the
experimental policy disabled/enabled. Source parameters are frozen from
`6942974`; candidate parameters merge `forward_alignment_experiment.yaml`.

The runner assigns separate local-only ROS domains (baseline 179, candidate 180,
feature_disabled 181), limits each subprocess to 90 wall-clock seconds, removes
only its own stale CSV before running, and records commands, environment,
outputs, and exceptions. The 180 s trial budget is simulated time. Three seeds
(42/43/44) compare optimizer sensitivity; they are not independent real-robot
trials. Identical seed repetition should compare trajectory and command CSV
fields as well as captured diagnostic modes.

Cases are reported separately: reachable feedback, intentional HOLD/blocked
rotation, and historical shadow. `comparison.json` does not turn expected HOLD
or shadow endpoint failure into a navigation success/failure rate. Acceptance
also checks nonnegative normal vx, zero lateral commands, executable yaw,
independent collision results, and post-reset speed bounds.
