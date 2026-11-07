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

The official B Matrix profile now explicitly uses `cost_penalty: 0.2`. On the
separate frozen B3 and B7 published costmaps this retains a positive soft cost
preference and more search margin than 1.0, including B7 start-yaw perturbations
of +/-0.08 rad. The original 2 s/1,000,000-iteration limits, body footprint and
filled sweep checks remain unchanged. Published maps and nearest recorded Native
poses are offline proxies, not the unrecorded planner-lock inputs; these results
do not establish full-route or live timing acceptance.

Ownership metadata uses the existing READY 500 ms lease. Its older 300 ms
boundary is still recorded once per observation as metadata latency evidence;
sensor, TF, command and independent stopped-visit checks retain their contracts.
The official Matrix variant derives its frozen lattice file by changing only
the metadata motion model to `omni`, selecting the installed official SE2
heuristic. All forward/rotate primitive arrays remain unchanged; analytic
connectors remain disabled. The original and derived file hashes are recorded.

Official/Hybrid Matrix runs explicitly enable failure-only planner snapshots.
The wrapper copies the actual locked grid and request, then writes a numbered
snapshot after releasing its locks if planning or output validation fails.
Search wall time, thread CPU and original cancel-check counts are separate from
capture and write overhead. Actual base iteration counts remain unknown.

### Initial Pose-goal region (official variant only)

The official Matrix Pose-goal profile explicitly enables
`FollowPath.initial_goal_region_shortcut`; the controller plugin defaults it to
false. After fresh TF/odometry and current filled-body checks, the first control
cycle of each plan may select a complete collision-checked shortest rotation at
the actual body origin when the body already lies inside the original goal XY
region. Dense signed heading checkpoints retain the selected half-turn direction.
The same original goal, live geometry checks, original tolerances, ordered
terminal cursor and command/body sweeps govern completion. The independent
waypoint client still requires a measured and commanded stop.

That one opportunity is consumed even if the body is outside the region or the
rotation is blocked; normal ordered prefix execution then proceeds. Cancellation
and reset clear the opportunity. This opt-in defines Pose-goal semantics: it may
replace an entire unstarted prefix when its destination region is already
occupied by the actual body. Use ordered-prefix semantics for tasks that require
executing an arbitrary supplied FollowPath loop or every intermediate constraint.

Recorded B3 N000 was already within 8.94 cm of its original goal (15 cm XY
tolerance); its path required 3.424 radians of rotation for an original yaw
difference of 0.141 radians. The new option targets that extra motion. Pure
geometry/state tests and builds are reported separately from actual Matrix
closed-loop acceptance, which still requires full-route recordings.
