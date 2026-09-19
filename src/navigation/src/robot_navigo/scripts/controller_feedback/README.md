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
full footprint collision checking remains enabled. `--footprint-file` supplies the
raw convex polygon, padding, inflation support radius and assumptions to all arms.
The default `legacy_footprint.yaml` is only a historical regression fixture. Use
`../../params/zsl1_model_envelope.yaml` for the model-derived body envelope.
The harness verifies its SHA256 before starting ROS and records the actual
Costmap2DROS padded polygon in each CSV `.footprint.yaml` sidecar. The input YAML
and actual float-converted polygon are both retained; do not reuse a result for
a different geometry SHA.

The plant uses an independent convex-polygon/cell separating-axis collision oracle.
It checks polygon interiors, unknown cells, and map boundaries at 0.01 s
substeps. `--self-test` exercises those cases, including an obstacle missed by a
boundary-only checker, exact grid-edge contacts, convex nonrectangles, and
concavity rejection. Touching an occupied/unknown cell counts as collision.
Production uses its own rasterized guard, so a geometrically passable tight door
may still be rejected. This is a discretized planar collision oracle, not a
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
goal. The original cases use an empty bounded costmap. Optional
`--recorded-costmap-dir` adds separately named `recorded_context_*` cases from
externally reconstructed frozen odom-frame costmap snapshots. These preserve the
original publisher inflation/self-clearing and finite rolling window; they do not
reconstruct future perception or prove the new footprint was used in perception. These cases preserve the difficult heading/path geometry
without claiming to reproduce the physical office scene. Shadow trials instead
feed recorded poses/velocities and original source timestamps with that fixed
first local path; their commands do not drive the subsequent recorded pose.
They must not be scored as closed-loop arrival tests.

## Reproduce

Treat `legacy_verified/` and `zsl1_model_verified/` as frozen evidence: read their
existing CSV, `comparison.json` and `acceptance_checks.json` directly. Do not run
`run_cases.py` or `prepare_cases.py` against either directory: the runner replaces
its CSV/log files, and analysis/plot scripts also rewrite derived files.

From the canonical workspace, the following creates a unique new output directory
for the model-envelope matrix, including recorded frozen contexts:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
TEST_SOURCE=src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/controller_feedback
mkdir -p analysis_outputs
TEST_OUTPUT="$(mktemp -d "$PWD/analysis_outputs/controller_feedback_repro.XXXXXX")"
touch "$TEST_OUTPUT/COLCON_IGNORE"
cmake -S "$TEST_SOURCE" -B "$TEST_OUTPUT/harness_build"
cmake --build "$TEST_OUTPUT/harness_build" -j2
"$TEST_OUTPUT/harness_build/controller_feedback" --self-test
python3 "$TEST_SOURCE/prepare_cases.py" --workspace . --output "$TEST_OUTPUT/cases" \
  --footprint-file "$TEST_SOURCE/../../params/zsl1_model_envelope.yaml" --geometry-cases \
  --recorded-costmap-dir analysis_outputs/zsl1_goal_20260910/recorded_costmaps
# Reuse archived libraries read-only; all new CSV/log files go into TEST_OUTPUT.
ln -s "$PWD/analysis_outputs/feature_navigation_20260908/baseline_install" "$TEST_OUTPUT/baseline_install"
python3 "$TEST_SOURCE/run_cases.py" --root "$TEST_OUTPUT" --variant candidate
python3 "$TEST_SOURCE/run_cases.py" --root "$TEST_OUTPUT" --variant feature_disabled
python3 "$TEST_SOURCE/run_cases.py" --root "$TEST_OUTPUT" --variant baseline
python3 "$TEST_SOURCE/analyze_results.py" --root "$TEST_OUTPUT"
python3 "$TEST_SOURCE/plot_geometry_results.py" --root "$TEST_OUTPUT"
```

To repeat the legacy matrix, create another fresh directory and replace
`--footprint-file` with `$TEST_SOURCE/legacy_footprint.yaml`, omitting
`--recorded-costmap-dir`. Do not reuse a completed output directory. Existing
frozen maps and the archived baseline libraries are read as inputs only.

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
rotation, historical shadow, provably too-narrow geometry, guard-clearance
diagnostics, invalid initial overlap, and recorded frozen context. Synthetic
door widths are measured from their rasterized openings; a gap below the polygon
minimum support width is impossible at every heading. A guard-clearance
diagnostic is not a proof of physical impossibility. `comparison.json` does not turn expected HOLD
or shadow endpoint failure into a navigation success/failure rate. Acceptance
also checks nonnegative normal vx, zero lateral commands, executable yaw,
independent collision results, and post-reset speed bounds.
