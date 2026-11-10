# Matrix planner comparison

This launcher starts only navigation, the existing safety gate and full MCAP
recording against an already running Matrix sensor bridge and native Lightning
localization. It never publishes replacement TF/odometry, sends goals, or starts
an SDK owner. Use the isolated domain 185 network environment. An independent
goal pilot sends each explicitly recorded Action.

Select `official`, `segmented`, or the auxiliary `hybrid` control with `--variant`.
Supply `--map`, `--footprint`, `--padding`, `--output`, the appropriate explicit
`--official-install` or `--segmented-install`, `--build-receipt`, and a custom-message
`--message-install` prefix. Preparation
is the default; `--run` starts the stack. The build receipt must bind selected
sources to a full local commit and list installation artifact SHA256 values.
Every listed artifact, including the segmented executable, is checked before
launch. The actual loaded Nav2 libraries and runtime parameters are also saved.
The message prefix is copied and applied only to the recorder process. Its
CustomMsg, projection debug, build metadata and Lightning debug types must load,
and recorder subscriptions must be present before READY. Do not substitute an
old full navigation overlay. A bag without these interfaces is incomplete even
when its raw sensors and front camera are available.

Every new `--run` arms an independent, detached recording deadline before ROS
setup or runtime children. `--max-runtime-sec` defaults to 600 and accepts only
finite values in `(0, 7200]`; startup time counts from the launcher's Linux process
start, using `CLOCK_BOOTTIME`, independently of agent/SSH lifetime or ROS time.
Arming must acknowledge successfully or startup fails. The frozen helper runs
with Python's isolated import path, no terminal, no inherited stdio pipes and no
ROS dependency. Linux `pidfd_open` and `pidfd_send_signal` support is required.
It binds the PID, process start ticks, complete command line hash, boot ID,
unique run ID and output directory before sending one launcher SIGINT through
the retained pidfd. Identity changes fail closed without a signal. PID reuse
cannot redirect the signal. Before each owned executable, a dormant stdlib-only
wrapper self-registers its PID, role and run/launch identity and waits for the
watchdog's retained-pidfd acknowledgement. It then uses exec with the same PID;
the watchdog verifies the final full argv against the frozen command plan,
including a direct ELF-interpreter shebang expansion. Unsupported ambiguous
shebangs fail closed. The launcher waits for that final identity acknowledgement
before continuing. Even if the parent freezes or exits immediately after Popen,
the real executable cannot run unregistered. An unacknowledged wrapper exits
without exec when its watchdog exits or the fixed deadline passes. The child's
initial environment binds the run ID, output directory, role and launch token.
Source services are excluded; the independent watchdog uses a minimal PATH/LANG
environment while owned targets retain their supplied ROS/DDS environment.

`recording_deadline.json` records the fixed deadline and launcher identity;
`recording_deadline_armed.json` records the helper identity; the separate
`recording_deadline_events.jsonl` records arming, deadline/reason, signal or
identity rejection. On entry to the existing shutdown path, the launcher
atomically writes a `shutdown_started` stage to `recording_deadline_shutdown.json`
and records each closure stage in `events.jsonl`; this leaves the deadline armed.
Only after all owned processes have exited does it write `recording_closed`,
before MCAP decoding/hashing. The helper verifies retained child pidfds are
closed before disarming. Command producers still stop before the smoother/gate
zero tail and recorder. If no confirmed closure appears within 15 seconds of
the deadline SIGINT, or the launcher exits while owned children remain alive,
the independent helper applies the same order: exact registered command
producers receive SIGINT, a 1 second zero tail elapses, remaining navigation
children receive SIGINT, another 0.5 seconds elapses, and the recorder receives
SIGINT. Each child identity and initial run binding is rechecked immediately
before its one fallback signal. `recording_deadline_children.json` and
`recording_deadline_children_armed.json` audit registration and acknowledgement;
per-role plan and self-registration JSON files bind wrapper and final argv;
fallback phases/results are in the helper event log. It waits up to 30 seconds
for child exit and reports any remaining child without escalation. These are
normal SIGINT requests, not a hard bound under kernel hangs or disk faults.
Long post-closure hashing receives no further deadline signals. Existing
recordings, build receipts and running launchers are not modified.
Run the non-ROS process checks with
`python3 experiments/planner_comparison_matrix/test_recording_deadline.py`.

For a frozen deployment without `.git`, pass `--source-snapshot <JSON>`. Build
the deployment from the retained canonical commit (for example with `git archive`),
then generate the manifest from the exact exported bytes. No linked worktree or
second Git checkout is needed. The manifest has this versioned shape:

```json
{
  "schema_version": 1,
  "origin": {
    "kind": "canonical_checkout_snapshot",
    "canonical_repository": "/absolute/original/canonical/repository",
    "commit": "<40 lowercase hex characters>",
    "branch": "<observed canonical branch>",
    "status": "<exact git status --short output; empty when clean>"
  },
  "files_sha256": {
    "experiments/planner_comparison_matrix/matrix_navigation.py": "<SHA256>",
    "experiments/planner_comparison_matrix/static_layer_snapshot.py": "<SHA256>",
    "experiments/planner_comparison_matrix/recording_deadline.py": "<SHA256>",
    "experiments/planner_comparison_matrix/run_matrix_navigation.sh": "<SHA256>",
    "experiments/planner_comparison_matrix/matrix_profile.yaml": "<SHA256>",
    "src/navigation/src/robot_navigo/scripts/nav_safety_gate.py": "<SHA256>"
  }
}
```

Replace every placeholder with the captured value. Include the selected
`--template` file if it differs from `matrix_profile.yaml`. Every listed file,
including additional entries, must have its lowercase SHA256 checked against a
normalized path relative to the deployed navigation root. Missing required
entries, duplicate JSON keys, malformed origin/schema/hash values, `..`, absolute
paths, external templates and symlinks escaping that root are rejected before
creating the run directory. The source manifest is copied unchanged into the run
and recorded with its digest and origin. Before runtime setup, its copied bytes
must still match that recorded digest and its source hashes are checked again.
`source_repository_observation` is null
because the deployment does not inspect Git. Its recorded canonical commit is
an origin observation, not an assertion that the deployed host has that checkout
or that the selected binaries were built there. The independent build receipt
and full artifact checks remain required. Omitting this option preserves the
existing canonical Git observation. Domain 185 also supports a verified same-host
loopback environment; transport selection does not change source timestamps or
freshness limits.

The fresh map retains 1 cm geometry and its mapping evidence uses 5 cm cells.
Navigation uses a 20 m rolling global costmap and 6 m local costmap at 1 cm.
Both publish full raw master costmaps; the recorder includes hidden topics,
actions, front camera and calibration, original sensors, commands, TF and diagnostics.
The launcher's two raw master costmap evidence subscriptions request reliable,
transient-local delivery with depth 1 to match the retained publisher samples.
Sensor subscriptions, the 6 s costmap evidence check, and freshness limits stay unchanged.
Each of the four complete frozen static-layer diagnostic topics is captured once
as its original CDR under `/experiment/static_layer_snapshot/...`, with a sidecar
hash and reliable delivery acknowledgement. The original `/map` and master
costmaps remain fully recorded. After closure, the MCAP must contain exactly one
message per snapshot alias with the original CDR hash. A map reload requires a new run. The old all-topic
v2 recording contained about 113 GB of uncompressed CDR, 92% from repeated complete
static-layer copies; that recording remains intact. The revised policy removes
those repeated subscriptions, and its exact exclusions are saved in each manifest.
The launcher writes a 0.5 s readiness lease and continuously revokes it for stale
odom/TF/scan/camera or invalid localization. The pilot must additionally check
current inputs immediately before each goal. Runtime command guards remain active
even if the observer exits.
Use a local DDS initial-peer range sufficient for all participants (the validated
November 5 environment uses 64). For the verified local fragmented CustomMsg loss,
the projector supports `sensor_input_reliable:=true`; its original default remains
false. This changes neither sampling stamps nor the 0.3 s stale-input rejection.

`matrix_profile.yaml` is an explicit open-area dynamic profile, separate from the
strict ideal-plant reference in `profile_template.yaml`. Before goal experiments,
Matrix native forward input 0.20 (nominal policy vx 0.4) measured about 0.441 m/s;
smaller inputs could remain stationary. Native yaw 0.30 measured about 0.2 rad/s.
Therefore the profile requests nominal vx 0.4, rotation 0.3, and does not silently
boost low commands downstream. Angular acceleration 6 permits a 0.3 policy step
at 20 Hz, within the tested input step amplitude; this is not a measured body
acceleration claim. The default final/phase XY tolerance is 0.15 m and yaw is
0.08 rad, applied identically to both schemes. Measured zero-command stopping
displacement was about 0.056 m; this sample is not a guaranteed bound. Each case
must still report exact settled original-goal error.

For the November 5 open-area runs, the conservative footprint is a square with
vertices at x/y = +/-0.74 m, plus 0.15 m padding. A triangle-inequality bound over
all model meshes and joint translations gives radius <=0.732237 m independently
of joint rotation. This deliberately conservative envelope does not establish
narrow-door capability. No result here establishes physical robot acceptance.

Keep every attempted run, including startup failures, network outages and
controller rejections. An Action success alone is insufficient: verify the
original goal after stopping, ordered turns, collision checks, camera continuity,
MCAP readability and concrete code/config/map identities.

For an authorized native multi-waypoint experiment, prepare/run with
`--with-waypoints`. The default single-goal profile is unchanged. The option adds
the installed Jazzy 1.3.13 `nav2_waypoint_follower/waypoint_follower` lifecycle
server and activates it with the existing nodes. Its effective parameters are
read back: `stop_on_failure=true`, `loop_rate=20`, and the native
`WaitAtWaypoint` task pauses for 1000 ms at every reached point. The new server is
an exact owned deadline role and stops with the command producers before the
smoother/gate zero tail. Its executable, core/plugin DSOs, package version and
Action/message definitions are hashed separately from the unchanged selected
planner/controller build receipt. The READY lease binds these identities and
the independent batch client source hashes. Include `waypoint_batch.py`,
`waypoint_contract.py`, the unchanged `pilot_scan_timing.py` and
`message_info_contract.py` in a frozen source deployment manifest.

The independent `goal_pilot_source/waypoint_batch.py` defaults to observation.
Its authorized `--execute --ready <lease> --route <route.json> --output <new-dir>`
submits exactly one native `FollowWaypoints` request with `goal_index=0` and
`number_of_loops=0`. Internally the official server sends one `NavigateToPose`
per leg, preserving the existing single-goal BT and terminal-goal binding.
This is sequential native waypoint execution; it does not plan one concatenated
global route. It stops on a failed leg without automatic retries or skipped
points. Its route contract is:

```json
{
  "schema_version": 1,
  "frame_id": "map",
  "executable": true,
  "mission_budget_sec": 300,
  "waypoints": [
    {"id": "first", "pose": [1.0, 1.0, 0.0], "require_stopped": true},
    {"id": "turn180", "pose": [1.0, 1.0, 3.141592653589793], "require_stopped": true, "tags": ["in_place_180"]}
  ]
}
```

The coordinates above illustrate the schema and are not an approved scene
route. An executable route needs 2–256 unique IDs and finite poses, including
each original heading. Repeated XY with a different heading remains a distinct
ordered point. Each adjacent leg and the first goal from the final live start
must be shorter than 8 m; this necessary check does not prove that the complete
path and padded body stay inside the current rolling costmap or mapped free
space. A route may cover the full map through successive local windows.
`executable=false` design documents are rejected. The preregistered mission
budget must be finite and in `(0,3300]` seconds; an explicit client `--timeout`
must equal it. Before sending, the client checks the launcher's fixed
`CLOCK_BOOTTIME` deadline and same boot ID, requiring the full budget plus
30 seconds for stopping/closure. Stop observation is bounded to `[.3,8]` seconds
so an override cannot consume that reserve. The launcher retains its default 600-second
deadline and explicit maximum 7200 seconds. Choose a sufficient explicit
launcher deadline before starting a long experiment.

The batch client inherits the unchanged a63b observer's 13 explicit
subscriptions, sensor QoS, map/odom frame checks, original `.3s` old/`.05s`
future input limits, JPEG/Info checks and `.5s` READY lease. It requires 10
continuous healthy, measured-and-commanded-stopped seconds after Action
discovery within a total preflight budget no greater than 40 seconds. It never
uses GT odometry for navigation. Native feedback selects the expected index;
each point independently requires fresh `/lightning/pose` within `.15m/.08rad`
and a continuous `.3s` measured stop from `/odom/current_pose` and commanded stop
from `/cmd_vel_safe`. A feedback advance before this proof cancels this client's
own batch. Original map poses, per-point entry/arrival/pass times, phase,
sampling/receive ages, observed motion and all missed-waypoint results are
retained. `/plan` endpoint matching and feedback-to-plan receipt latency remain
explicit geometric/time inferences, not planner CPU/search durations.

The launcher separately watches the Jazzy Action graph, native Action status
and child feedback without adding to the client's 13 sensor/evidence
subscriptions. READY requires exactly the native follower as the only
`NavigateToPose` client and the expected unique servers/status publishers; an
already active goal prevents initial READY. It continuously revokes READY when
this ownership changes. Native statuses are event-driven, not periodic health
heartbeats. Observed child UUIDs/feedback are recorded through the launcher
lease and full hidden-topic MCAP; parent/child association has no direct UUID
field and remains contextual under exclusive ownership.

Cancellation has a specific upstream boundary: this client targets only its
own nonzero `FollowWaypoints` UUID, including a missing goal response. The
official 1.3.13 server internally calls `async_cancel_all_goals()` on its
`NavigateToPose` endpoint. This experiment therefore supports exclusive domain
185 ownership only; it is unsuitable for a shared navigation server. The
upstream server also refreshes each child goal's intent stamp at dispatch;
original route intent and sensor sample stamps stay separately recorded.
Every possibly sent goal reaches own-UUID cancellation and an independent stop
observation in `finally`, including goal-response timeout and runtime exceptions.
The new client handles SIGINT/SIGTERM as flags and keeps its ROS context alive
until this cleanup finishes. A missing stop observation remains explicitly false.
Measured and commanded stop velocities must all be finite; the unchanged a63b
observer is not edited.
The safe-command velocity components must also be present and finite during
motion; this check applies no stop-speed threshold while navigating.
The new client also preserves raw native pose XYZ/quaternion values and rejects
nonfinite components before trusting a projected yaw. It performs no quaternion
normalization or coordinate conversion; legal missing scan-range values retain
their existing interpretation.

Long-route scan MessageInfo and state evidence use a bounded 4096-record
nonblocking queue and a dedicated low-frequency writer. Safety callbacks do no
JSON serialization or file I/O. The original first-invalid snapshot and ring of
50 samples are retained; the old 2048-sample trace cap does not apply to this
new client. Queue overflow, writer failure or an incomplete drain is a permanent
evidence gap and makes the final attempt invalid. Instrumentation failure does
not replace the own-UUID cancellation/finally path. Nonfinite numeric observations
are encoded only during writer/final persistence as a
`{"nonfinite_float":"NaN"}` (or `+Infinity`/`-Infinity`) object, preserving their
location without changing the original safety caches or first-invalid references.
Run the additional offline
checks with `python3 experiments/planner_comparison_matrix/test_native_waypoints.py`.

### Full-map Lattice search candidate (2026-11-07)

The Matrix profile explicitly sets `GridBased.cost_penalty: 1.0` for all three
runtime variants. The prior profile omitted this parameter and used the
installed Jazzy 1.3.13 default of 2.0. On the frozen published costmap preceding
B3's N001 failure, the official Lattice reached the 1,000,000-iteration cap
in about 1.55 seconds with 2.0; changing only this soft-cost weight to 1.0
returned a path in about 226 milliseconds. Every returned pose and connecting
sweep passed the existing filled, padded rectangular body check. This is an
offline candidate, not full-route navigation acceptance or an A/B winner.

The map, 1 cm grid, primitive file, 2 second planning budget, iteration cap,
footprint, padding and original goal stay fixed. The selected planner binary
retains its separate build receipt; a configuration commit does not rebuild it.
The fixture, typed effective parameters, original and candidate outputs,
source analysis and independent review are under
`analysis_outputs/planner_comparison_20261105/` in
`native_B3_planner_frozen_costmap_benchmark_solmax_v1` and
`native_B3_lattice_parameter_candidates_solmax_v1`. The published costmap and
nearest Native pose sample are the available offline fixture, not an exact
recording of the planner's internal locked map or TF lookup.

### Separate recording and mission budgets (2026-11-07)

The optional recording deadline now accepts up to 7200 seconds; its default
remains 600 seconds. Select enough recording time for setup, the independently
bounded route and final stop/closure. B4 sent no goal because its fixed 3600
second window had only 3267.6 seconds left before dispatch, below the unchanged
3300 second mission plus 30 second reserve. Keep B4 as a zero-goal preparation
case, not a planner failure. The next full-route recording explicitly requests
7200 seconds, while its route still has a 3300 second execution budget.

This changes only the recording supervisor's finite upper limit. The absolute
CLOCK_BOOTTIME deadline, retained-pidfd ownership, stop order, default timeout,
planning budget, sensor-age checks and measured stop requirements are unchanged.
Inputs have their own deadline and must remain available throughout each case.

Segmented A original-goal binding

The segmented profile explicitly selects `cost_penalty: 0.0`, the original 2 s
planning budget, and terminal goal binding on both producer and consumer.
`robot_feasible_navigation::goal_region` exports stateless cell proof and wire
parsing through `robot_feasible_navigation::rfn_goal_cell_binding`. The final
original goal pose stays on the wire while only the checked prefix is executable.
Current original-goal XY/yaw, ordered phases and measured stopping remain required.
The vendor search and filled footprint checks are unchanged. This configuration
is an offline-supported candidate; a full Native A run and comparison remain pending.

The request-local obstacle index also stores each row's exact blocked columns.
SAT checks visit those same cells in the original order, avoiding scans of empty
cells without changing geometry, unknown classification or the planning budget.

### Native short-route controller recovery (2026-11-07)

The segmented controller feeds RPP a temporary same-XY path at the measured body
origin during an ordered rotation. Its target yaw and the coordinator's original
path, XY anchor, signed progress and external goal remain unchanged. Within the
existing yaw tolerance it commands zero and waits for measured stopping. A stopped
rotation outside its original anchor fails explicitly instead of advancing.

The segmented Matrix tree permits one controller-recoverable failure per original
NavigateToPose goal. It delays one second with BehaviorTree.CPP, computes a fresh
full path from the stopped body and starts a new FollowPath. It neither resends the
parent goal nor records the failed phase as visited. Planner errors are excluded
from this recovery. The official and Hybrid trees retain their single sequence.

The segmented Matrix profile requests nominal RPP rotation at 0.15 rad/s and uses
stopped-goal thresholds of 0.03 m/s and 0.05 rad/s with a continuous 0.3 s dwell.
Original goal tolerances remain 0.15 m and
0.08 rad; physical-input freshness remains 0.3 s. Both costmaps inflate around
unknown cells so inflation cannot turn an original unknown cell into a traversable
inscribed-cost cell. These changes require native runtime verification; a build or
static path check alone does not establish navigation acceptance.
All package consumers of this header must be rebuilt together. Ownership metadata
uses the READY 500 ms lease; physical-input freshness and stopped visits retain
their original contracts. Unknown cells seed inflation as hard obstacles.

### A8 bounded tracking candidate (2026-11-08)

The segmented experiment uses a fixed 0.75 m RPP lookahead. Original A5 recordings
showed that the previous 0.25 m carrot triggered autonomous RPP rotation during
an ordered TRACK phase. The TRACK pure-spin rejection and explicit signed ROTATE
phases remain enforced; the rotation threshold is still 0.5 rad.

The read-only `segments.max_angular_velocity` parameter defaults to 0.3 rad/s and
must be finite in `(0, 0.3]`. The controller rejects invalid raw commands before
limiting angular velocity, then checks the filled footprint sweep of that bounded
command and publishes the same command. This avoids checking a tighter turn while
the downstream smoother executes a clipped turn. The fixed lookahead override
applies only to the segmented variant. Smoother transients and simulator tracking
still require runtime validation; these settings are not a navigation acceptance.

批量测试客户端的原首段请求上限仍为 8 m。显式仿真选项
`--simulation-first-leg-max-distance-m 8.5` 只在既有仿真模式且生效 global
window 至少为 20 m 时放行；各原路线相邻段仍须小于 8 m。它用于原目标失败
后实际起点的有限偏差，完整机身、地图边界与碰撞可行性仍由规划服务检查。
首段实测距离、请求上限及原始 pose 时间戳独立记录；TF/停稳/最终目标容差不变。

`--health-tf-latest-only` 为显式仿真下的只读 launcher 健康监听器选择动态 TF
BEST_EFFORT/KEEP_LAST/depth1；默认仍为 tf2 自带配置，静态 TF 配置不变。
B27 录包在拒绝前已收到约 102 ms 时间龄的连续 Native TF，但 launcher 报告
自身缓存超限。这个候选减少监听旧队列，并逐次记录 callback/DDS 时间、原 TF
stamp 与 lookup 实际结果，供区分接收和消费延迟。Native TF 发布、控制器、
原 300 ms 判断和导航取消条件均保持，首次错误不会被延后或改判为健康。

### A11 accepted queue-state consistency (2026-11-09)

A7's raw search path contained a 2.236 cm backwards moving turn. A synthetic
all-free near-goal replay reproduced the same failure with the A8 binary. Two
accepted entries for one Lattice node had identical float `f` priorities but
different `g` values (11.1425276 versus 11.1425266). Restoring the older queued
pose and primitive left the newer graph parent in place, producing that invalid
edge. Independent instrumented replay recorded the exact mismatched chain.

The private segmented search now snapshots the accepted Lattice `g` and discards
an entry whose exact value has changed before restoring its pose or primitive.
The start cost is initialized before enqueueing, so the start is not mistaken
for a stale entry. This is a queue-state version check, with no numeric tolerance
change. Other node types keep their queue behavior. The full near-goal planner
regression retains the original goal binding and 2 s planning limit; collision,
signed segment execution, sensor freshness and stopped-goal checks remain.
This fixes the reproduced search-path defect; complete simulation navigation
still requires new recordings. Installed official B libraries are unchanged.

## A12: retain same-goal progress across controller phases

A failed FINAL shortcut stays disabled for the same original goal across queued
and fresh compatible replans. Certified ordered advancement can permit a new
shortcut. Installing a phase no longer resets the goal's PoseProgressChecker;
healthy zero/anchor waits now run that checker before returning. A restored phase
outputs zero for its first cycle so its original anchor is checked on the next
cycle before motion. The existing 15 second anchor wait survives temporary FINAL
activation and compatible reinstallation.

Original-goal success still requires the existing geometry, checked zero and
continuous .3 second original-odometry/steady stopped dwell. A stopped valid
observation gets a bounded .5 second priority window over the progress deadline;
compatible replans cannot restart that window. Moving, checker failure and failed
zero checks do not grant priority. This prevents both repeated replan/dwell-reset
livelock and premature failure of a legitimate .3 second final stop.

The actual-server regression fixture uses the real PoseProgressChecker with
protected IO seams and synthetic poses. Negative controls retain failures for
the exact A11 source and the rejected first design. The final build adds ten
phase regressions to the existing package tests. No simulation route or physical
acceptance follows from these software checks.

This new Matrix-only A profile requests .3 rad/s normal rotation, matching B's
nominal request and staying within the unchanged .3 rad/s command cap. A8 used
.15 rad/s. This is an explicit new profile input, not a proven reduction in body
drift. Goal/TF/scan/footprint thresholds and forward path restrictions retain
their previous values. All server/header consumers must be rebuilt together.

## A13: prefer obstacle clearance in the segmented simulation profile

The Matrix-only segmented planner now uses `cost_penalty=1` instead of zero.
On the original A9 bag's nearest global-costmap publications, 2.720 seconds
before and 12.907 milliseconds after the planner failure, the installed A12
planner reproduces the same filled-segment rejection at zero cost. Changing only
this penalty returns a fully swept path on both frozen inputs, in .82 and .91
seconds. The start is the nearest recorded Native pose to the planner's goal
acceptance; these publications are not an exact snapshot under its map mutex.

This is a configuration candidate for a new simulation recording. The A12 C++
artifacts, full-body sweep, original goals, planning deadline and stopped/freshness
checks remain the inputs for that recording. Its Python/profile commit must be
recorded separately from the reused A12 compiled-source commit. The late map
captured 303 seconds after the failure did not reproduce the rejection and is
retained as a separate, unsuccessful diagnostic.

## A14: retain short translations until the next rotation is safe

A10 produced valid plans, but the controller consumed a 22 mm translation while
still inside its original 15 cm handoff region. Cached command samples had zero
translation. A later full-body rotation was free at the planned anchor and blocked
at the actual body position on both nearest published costmaps. These are frozen
publication inputs, not exact controller mutex snapshots.

The optional server flag `segments.require_next_rotation_clearance` defaults to
false. The Matrix A profile enables it: the immediate ordered rotation must pass
the existing filled sweep at the actual body origin before TRACK may stop and
advance. Both phase evaluation and same-cycle velocity computation check it.
Measured stop and the original XY/ordered progress remain mandatory. A retained
TRACK that leaves that original handoff region fails with `NoValidControl` for
the existing single original-goal recovery; moving cannot indefinitely renew it.

For these TRACK segments only, `.005 m` is an RPP endpoint branch trigger, not a
new acceptance requirement. The Matrix profile also sets the implicit path-heading
threshold to pi so that a displaced micro-target requests forward tracking rather
than an unplanned spin. Explicit co-located ROTATE keeps its goal-heading branch.
The original final goal, .3 second stopped dwell, freshness limits, footprint,
angular cap and one-second actual-command sweep are unchanged. Frozen A10 inputs
show possible forward commands and a narrow window for the next rotation; these
offline checks do not establish dynamic or whole-route acceptance.

## A15: center admission for the terminal rotation

A11 reached 34 targets. F011 entered the normal FINAL rotation near the edge of
the original goal region, lost that region, and restored its ordered path. Its
first controller attempt failed the progress check; the second ended at the
existing 15 second unstarted-rotation wait. The small planner log contains no
planning error. These logs do not establish a physical drift bound or the cause
of every strict timing violation.

`segments.terminal_rotation_entry_xy_tolerance` defaults to zero for the A14
behavior. Matrix A sets it to `.05 m`, limited by the original XY tolerance.
This is an entry condition about the original goal. The original `.15/.08`
acceptance and checked-zero `.3 s` stopped dwell still take priority.

The shortcut and the final TRACK handoff use the same center condition. A tail
of consecutive ROTATE phases is one terminal run, including turns split at
90 degrees. Before its first filled-checked nonzero rotation command, both phase
checking and the fresh compute pose must satisfy center admission. Admission
then survives internal chunks, while restore and newly installed plans clear
it. Ordinary turns needed to approach the center retain A14 behavior.

The final TRACK uses an RPP-only proxy narrowed by the goal-cell binding error,
including a path with no trailing ROTATE. The original GoalChecker is unchanged.
A retained terminal approach is bounded by the original goal region, so a
quantized endpoint cannot reject a valid original-goal stopped acceptance.
An unstarted terminal run outside the center requests the existing same-goal
replan instead of rotating or waiting for an impossible translation inside a
ROTATE phase. Full filled sweeps, speed limits, freshness and progress budgets
are unchanged. This candidate does not establish full-route acceptance and is
not evidence that `.05 m` admission resolves B34's separate failure.

## Durable mission checkpoints and terminal cause (2026-11-10)

The opt-in mission CLI binds the first same-UUID terminal cause to its unchanged
raw result file and SHA. A later reconciliation file supplies its own stop and
child-quiescence observation; it does not replace the original cancellation
cause or clear strict timing failures. Missing, modified or conflicting evidence
holds the mission. Explicit operator cancellation remains canceled.

For executing mission operations, one storage thread creates, uses and closes
SQLite and retains the original owner locks. FULL/DELETE, transaction commits,
file fsync and directory fsync remain required. The main ROS executor continues
input processing and exact-UUID cancellation while these checkpoints finish.
Critical visit transitions use a bounded ordered queue; failures retain an
uncertain attempt rather than claiming a completed checkpoint.

After durable SEND_INTENT and both exports, the same operation re-establishes
the original stable preflight inside its original deadline, checks fresh inputs,
identity, stop, first-leg distance and remaining budget, then sends at most once
with the already reserved UUID. Exiting before invoking send_goal_async permits
only a current-owner NOT_SENT_AFTER_INTENT record with null native status and no
budget refund. A new process never automatically dispatches an uncertain intent
or reuses a proven-not-sent reservation. The new mission_storage.py module is
included in launcher required sources, capability SHA and CLI source copies.

These are software contracts for isolated experiments. They do not migrate old
A16/B38 live databases or establish route, simulator or physical acceptance.
