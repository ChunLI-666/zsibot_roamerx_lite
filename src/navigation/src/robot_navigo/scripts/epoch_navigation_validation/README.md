# Real Nav2 epoch integration validation

`run_stack.py` launches actual planner, path smoother, BT navigator, controller,
velocity smoother and safety gate processes. The fixture replaces the external
world and localization input: map-backed raycast scans, planar command-feedback
motion, odometry, source-stamped TF and typed localization epochs. No actuator
bridge, LCM transport or physical robot process is launched. The strict installed
BT and production controller generate the plans and commands.

Use ROS Jazzy and the built workspace. Every run requires a **new** output
directory; the runner rejects an existing directory. Recorded evidence directories
are read-only inputs for offline audit, never a destination for another run.

```bash
cd /home/charles/project/colcon_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
trial_dir=$(mktemp -d analysis_outputs/epoch_reproduction_XXXXXXXX)
python3 src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/epoch_navigation_validation/run_stack.py \
  --workspace . --scenario normal --domain 188 --output "$trial_dir/normal"
```

Domains 188–191 and localhost discovery are enforced; coordinate domain ownership
before running. Run at most two stacks concurrently to limit CPU-related TTL
jitter. Only this trial's child process groups are stopped during cleanup. Do not
rebuild or edit source while a run is active: source, installed dependency hashes
and `ldd` are captured before/after, and changed inputs invalidate the result.

Supported cases are `normal`, `loss_recovery`, `clock_pause`,
`controller_silence`, `cancel`, `pending_planner`, `pending_smoother`,
`bare_recovery`, `gate_restart`, `command_reorder`, `preempt`,
`localization_session`, `pose_mismatch`, `future_tf`, `stuck_replan`,
`through_poses`. Each scenario asserts its actual action acknowledgement,
injection evidence and expected terminal/stop behavior. Successful navigation
also checks independent final XY/yaw against the effective goal-checker values.
The default two-metre warehouse route uses the model footprint overlay.

The transparent planner/smoother transport proxies retain the actual backend,
hold its first result, reject its cancellation, and release it only after a fresh
epoch has authorized motion. The test verifies actual forwarding and equal
canonical field hashes. This tests **late results**, not delayed action ACKs or
the controller current/pending-goal cancellation race.

`audit_events.py` independently requires each nonzero safe sample to have a nearby
smoothed command, an unchanged source identity/sequence/age from a real raw
controller command, exact active intent/execution authority, and matching
localization and gate identities. A 50 ms cross-topic DDS observation window,
30 ms source-age observation margin and 2 ms future tolerance are explicit
measurement allowances. Execution expiry is 300 ms; intent expiry is 400 ms.
These are observer checks, not a second control implementation. Empty/all-zero
traces cannot be scored as integration passes. Unit negative controls include
missing and mismatched authorities, wrong controller/boot, future timestamps,
refreshed relay source times and NaN output.

```bash
python3 -m pytest -q src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/epoch_navigation_validation
python3 src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/epoch_navigation_validation/audit_events.py \
  --events "$trial_dir/normal/wire_events.jsonl" --output "$trial_dir/strict_reaudit.json"
```

The map-loaded instance remains constant; a supported physical map-switch flow
is not simulated by rewriting its identity. Localization corrections and process
sessions are typed fixtures, not the real Lightning estimator. Clock and failure
deadlines use host monotonic time. The plant is a command-driven planar
kinematic model, with no measured quadruped gait or independent full-stack SAT
collision scoring. The separate controller-feedback geometric matrix is separate
evidence and must not be combined into a full-stack zero-collision claim.
