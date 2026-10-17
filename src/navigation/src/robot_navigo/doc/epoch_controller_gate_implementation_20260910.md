# Typed controller, smoother and safety gate implementation

This change runs on `feature/forward-aligned-navigation`, after interface/BT commits
`b2c0d4e` and `842f38f`. `enable_epoch_contract` defaults to false for compatibility;
the separate epoch experiment overlay enables the strict chain. No robot motion,
deployment or merge is part of this verification.

## Authorization and actual computation

The strict controller exposes `follow_path_epoch`, and does not instantiate the
legacy `follow_path` action server. It requires the localization identity, gate
challenge, navigator task and plan sequence to agree with an active intent. It
publishes installed execution state only after installing the typed path and
checking TF against the localization output at that output's source timestamp.
Default position and rotation agreement tolerances are both `1e-5`.

The pose used for MPPI, progress and goal checks is the validated typed pose
transformed at its original timestamp into the controller costmap frame. It is
not a separate latest-TF lookup. The controller checks authorization before and
after computation and emits `/cmd_vel_epoch_raw` only for that installed token.
A new plan in the same task preserves progress timing and controller warm state;
a new task, localization identity or gate challenge resets those states. Cancel
and error paths terminate the current action only, preserving a pending action
for the action server worker to accept.

The shared helper pins the first loaded map instance. A changed instance stops
the node from authorizing paths against its old grid. Heartbeat, task, plan and
positive localization-source timestamp watermarks reject rollback. Not-ready
messages with empty output timestamps revoke immediately without either
resetting the source watermark or falsely declaring a time regression.

The smoother accepts only typed raw commands in strict mode. It retains the
controller command sequence, Linux boot identity and `CLOCK_MONOTONIC` source
time while adding its own relay session and sequence. Interpolation cannot renew
the original command lease. Source expiry or authorization loss clears the
interpolation state and emits zero without assigning a new source time.

The final gate requires matching localization, intent, installed execution and
typed command; command age is checked using the same-host monotonic clock and
boot identity. A fresh command is not itself proof of an installed path. The
gate rotates its challenge on an armed authorization failure or emergency stop,
so restoration of a heartbeat cannot revive a previously authorized plan.

## Normal terminal handshake

The first full-stack arrival trial exposed a termination loop: the controller's
normal inactive `goal_reached` execution state caused a challenge rotation before
the BT consumed the FollowPath result. Normal completion now seals the exact
installed token, clears cached commands, publishes zero and disarms the gate
without changing its challenge. Only the controller session that installed the
current token can seal it. Later `controller_idle` or active heartbeats cannot
unseal that token; command replay remains rejected. New tasks can install new
authority. This does not exempt abnormal termination or expiry during active
execution from revocation.

## Verification

On ROS 2 Jazzy, core, MPPI, controller and smoother were rebuilt with two workers.
MPPI was explicitly rebuilt because the controller interface gained a virtual
`reset()` method. The final gate-only adjustment rebuilt `robot_navigo`.

- `test_epoch_authority`: 8 C++ tests passed.
- `test_epoch_gate`: 16 tests passed, including source/relay ordering, source
  freeze, zero-stamp revocation, map/session rollback, real-node challenge
  rotation, terminal sealing, terminal TTL, replay rejection and a subsequent
  task installation.
- `test_nav_safety_gate`: 9 existing legacy tests passed.

CTest evidence is in workspace `analysis_outputs/zsl1_goal_20260910/`:
`epoch_authority_ctest.log`, `epoch_gate_ctest.log`,
`epoch_navigation_build.log`, `epoch_navigation_incremental_build.log`, and
`epoch_terminal_build.log`. Gate CTest uses localhost domain 189, separate from
full-stack validation domain 188. These unit and component checks are not a
claim that the independent full-stack fault matrix has passed; that report is
maintained by the validation agent. Codacy tooling was not available.
