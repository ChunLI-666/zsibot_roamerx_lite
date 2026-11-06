# Full-scene raw acquisition owner

`full_scene_mapping.py` is a default-HOLD acquisition tool. It is separate from
Nav2, localization and the two-planner experiment. Preparation and fixtures open
no ROS context, UDP socket or relay child. Source review does not authorize
execution, prove whole-scene coverage or approve a route.

The only supported mode is `controlled_static_sim_scene`, scope
`MAPPING_ACQUISITION_ONLY`. Root must approve the complete physical XML/resources,
finite drivable floor domain, meter coordinates and current scene instance, and
freeze the premise that no other actor spawns or changes the static scene. The
un-eroded trinary physical oracle supplies free evidence: 0 unknown, 1 free,
2 occupied. Unknown/outside is blocked. GT and this oracle are used only for
raw-data collection; they must never become Nav maps, TF, poses, localization
seeds, localization status, or planner acceptance evidence. Dynamic-scene mode
is closed.

## Geometry and source-time evidence

The full filled square has half extents 0.74 m and padding 0.15 m. SAT tests all
overlapping cells, including interior cells, following the retained
`robot_feasible_navigation/footprint_sweep.hpp` checker. It adds the same extra
whole-cell guard. It does not apply a second map erosion. Translation plus corner
motion between sweep samples is at most half a grid cell. Point-hit sweep tests
add half a cell to cover the interval between samples. Every command rechecks
the actual pose, filled-body sweep, original-sample age uncertainty, motion
drift, braking distance/angle reserve and the existing relay's packet/watchdog
tail. The tail uses the maximum 0.15 s command-renewal interval + 0.20 s packet
age + 0.25 s relay watchdog. A computation that overruns command renewal or
freshness does not renew a positive command.

The sweep uses a command-time pose-age upper bound:
`min(0.30, health_header_age + 0.10)` seconds. The 0.10 s computation window
starts before health reads the time and includes cloud transformation, graph
queries, geometry and owned-process identity checks. Inside `OwnedRelay.send`,
after the PID check, the final pure guard checks the same pose snapshot at the
actual packet issued timestamp: elapsed computation <=0.10 s, age <=the swept
upper bound, original input/header/receipt freshness, source clock and owner
latch. Positive sends require this final guard. Packet transport delay remains
bounded by the unchanged relay's 0.20 s packet-age test and is included in the
stop tail. A slowed computation stops instead of inventing a fresh packet for
an insufficient sweep. These are fresh-observation constraints plus reviewed
SIM motion reserves, not a claim of zero physical tracking error.

Root must freeze SIM engineering bounds in `motion_model`; these are assumptions
supported by finite observed ranges, not a hard physical safety certificate.
Fields are `max_truth_speed_mps`, `max_truth_yaw_rate_rps`,
`stop_distance_reserve_m`, `stop_angle_reserve_rad`,
`forward_drift_reserve_m`, `turn_drift_reserve_m`,
`forward_yaw_reserve_rad`, `max_roll_pitch_rad`, `body_z_min_m`,
`body_z_max_m`, `floor_z_m`, `floor_hit_tolerance_m`, and `obstacle_top_m`.
`basis` must be `root_reviewed_engineering_bound_sim_only` and
`motion_model_reviewed` must be true. `floor_height_model` is flat and must bind
the same `floor_z_m`. A lying robot must not be admitted by widening standing
height bounds. Even a zero packet from the frozen relay contains action=walk,
so fresh standing height/attitude and measured stationary GT are checked before
creating that child. Any official stand operation is root's separate action.

Original `/odom` uses header `map`, child `BASE_LINK`; these labels are preserved.
They are not Nav's map/base_link contract. Root must bind numeric correspondence
evidence to the physical world. Only an explicitly verified identity numeric
alias is implemented; a nonidentity relation requires an appropriately
transformed acquisition oracle and a new review. The alias object contains
`verified:true`, `observed_gt_frame:"map"`,
`observed_gt_child_frame:"BASE_LINK"`,
`numeric_transform:"identity_verified_against_raw_scan"`, and
`evidence_file`/`evidence_sha256` present in `artifacts_sha256`.

The raw Matrix cloud is sensor-local FLU despite its `map` frame label. The tool
requires the actual packed 26-byte point ABI and frozen sensor configuration.
It transforms hits by actual `T_body_lidar` then the original GT SE3 interpolated
at the original cloud stamp, with bracketing GT samples at most 50 ms apart.
It never substitutes now() for a sample stamp. Raw quaternion/position, IMU,
point values, fresh headers/callback receipts and stamp advance are checked.
The 0.30 s old / 0.05 s future limits remain. Scan Inf/NaN gaps are not free
evidence. A fresh derived scan with exactly the original lidar stamp is required
as projector health evidence, but is not used to shrink the body or clear space.

The raw hit cloud supplies collision contradictions, freshness and visibility
statistics. Empty/invalid clouds fail. Every positive near return is retained;
exact-origin ambiguity fails. The reviewed simulator configuration must exclude
the mounting actor. There is no geometric whole-footprint self mask. Hits above
the reviewed flat-floor tolerance and below `obstacle_top_m` inside the current
body/stopping corridor stop acquisition. Hit gaps, occlusion and azimuth bins
without hits never certify volumetric free space. The static oracle premise is
the basis for unobserved space.

## Route and root release

Root supplies independent JSON; the tool invents no exploration route.
`map_coverage.py` is reused to load the acquisition grid and can separately help
root construct route candidates. The physical grid is not loaded into Nav.

Route schema 1 requires the scope/mode above, `executable:false` by default,
`scene_instance_id` (full captured scene process identity),
`truth_frame_id:"map"`, `truth_child_frame_id:"BASE_LINK"`,
`start_anchor_truth:[x,y,yaw]`, a finite `max_runtime_sec` in (55,3600], and
`waypoints:[{id,pose_truth:[x,y,yaw],require_stopped:true}]` with unique IDs.
Default budget is 600 s. The budget includes 25 s reserved for cleanup; mission
control stops before that reserve. Start requires 10 continuous stationary
seconds and a fresh start within 0.10 m / 0.08 rad of the frozen anchor. Each
point requires a measured stop; final capture requires 20 continuous stationary
seconds. A stop requires finite fresh advancing GT, linear norm <=0.03 m/s,
angular norm <=0.05 rad/s, zero commanded axes, and pose stability over the
interval. Point tolerances are 0.15 m / 0.08 rad. Failures are not retried.

An optional incoming `leg_policy` closes narrow bridge behavior:

```json
{"kind":"straight_fixed_heading","travel_yaw_rad":0.0,
 "max_yaw_error_rad":0.05,"max_cross_track_m":0.05}
```

The values above are illustrative, not a released bridge. The heading must
match the frozen forward segment; max yaw/cross-track limits cannot exceed
0.08 rad / 0.15 m. The tool stops and aligns at the segment's frozen start,
then sends only forward commands until the end, then stops before target-yaw
rotation. Yaw/tube/endpoint violations stop with HOLD; no in-leg correction,
lateral motion or 180-degree turn occurs. A fixed-yaw square path requiring
lateral translation is not a feasible forward-only route. Nominal full-route
SAT checks every rotation, translation and fixed-heading tube before execution;
fresh stopping envelopes can still require HOLD during control.
Bridge yaw/tube/end checks also run inside the final packet guard against this
tick's post-spin GT, so an older valid outer-loop observation cannot admit a
packet after a newly observed violation.

The oracle adapter requires `oracle_kind` =
`physical_collision_floor_projection_not_slam_map`, `body_half:[0.74,0.74]`,
`padding:0.15`, `footprint_inflation_in_grid_m:0`, `grid_yaml`,
the actual `T_body_lidar:{xyz,rpy_rad}`, `sensor_config_json`,
`point_coordinates:"sensor_local_flu_despite_raw_frame_id_map"`, and
`self_return_policy` =
`root_verified_simulator_ignore_mounting_actor_no_geometric_footprint_mask`.
`root_geometry_audit` must explicitly approve `complete`,
`coordinate_verified`, `static_scene_verified`, and `root_domain_approved`.
All XML/resource/floor/coordinate/config/grid artifacts must be listed by path
and SHA256. Current mapper candidates keep those approvals false until root
freezes the actual audit. The grid is at most 5 cm resolution.

Execution additionally requires an explicit once-only root release with
`allow_execute:true`, unique 32-hex `release_id`, exact route/oracle SHA256,
absolute unique output, same scene identity and budget, and exact
`sources_sha256` for the five runtime source dependencies listed by
`source_files()`. It binds this boot/network namespace; full core identities
(`pid,starttime_ticks,cmdline_sha256,argv,boot_id`) of retired
`old_sim_command_adapter`, `old_sim_udp_relay`, `old_navigation`; and trusted
`allowed_udp_source_processes` / `allowed_external_udp_sockets`.

`controller_pid_namespace` is bound to the controller's actual `/proc/self/ns/pid`.
The host UE may be outside that container PID namespace. Its identity is an
explicit root external attestation, not a direct monitor implemented here:
`scene_identity_proof` requires `mode:"root_external_host_attestation"`, an
absolute `evidence_path`, actual `evidence_sha256`, and `host_pid_namespace`.
The evidence JSON must equal the declared captured scene identity, including
that host PID namespace, and its path/SHA must be an artifact of the exact
physical oracle bound by the release. Result and READY state both record this
basis and both namespaces. A captured identity is not a current liveness lease;
root remains responsible for checking the host engine/7447 instance and keeping
the approved static scene unchanged. The controller does not claim to detect a
host UE replacement through its container `/proc`.

UDP inventory is a cooperative closed-world process/namespace check, not a
kernel interception of every sendto(). Root must audit each authorized process
and external socket; an unattributed or newly observed writer is not implicitly
allowed. Exact old identities must be absent/zombie before starting our relay.
PID reuse does not authorize the new process. Every 0.1 s the tool rechecks
retirement, recorder/relay identities and the entire UDP inventory. Any observed
violation is latched; audit freshness expires after 0.3 s. Concurrent Nav action
services or Nav/adapter velocity publishers are refused. Only this tool's
copied frozen `sim_udp_relay.py` child writes simulator command UDP. The source
relay is unchanged. Commands are only nominal vx=0.4 (raw forward=0.2), or
wz=+/-0.3, or zero; no simultaneous axes, reverse, lateral or micro-speed.

Root starts the raw recorder first. Release binds `recorder_ready_path` and its
SHA256; the receipt scope is `RAW_RECORDING_ONLY`. Receipt/release must match
`case_id`, `recording_run_id`, recorder core identity, deadline,
`node_name`/`node_namespace`, and required topic set. Release names are
`recorder_node_name` / `recorder_node_namespace` (currently `/`).
`recorder_deadline_steady_ns` must leave the entire controller budget plus 5 s.
The enclosing recording owner budgets its separate success tail and closure
within its own unchanged 3600 s cap. Required original/derived lidar, IMU, GT,
front camera, CameraInfo, static TF, clock and experiment events must actually
have the exact recorder subscription; sensor publishers must match the release
and their node identities must be unique. Camera presence in the bag does not
by itself prove successful image encoding or full-scene coverage.

Execute environment must match `MAPPING_OWNER_RUN_ID=<release_id>` and
`MAPPING_OWNER_OUTPUT=<absolute controller output>`, isolated domain185 and
same-host FastDDS. The root release is consumed by an exclusive once marker;
the tool never removes it and never overwrites an attempt directory. A new
attempt needs root's new release, not an automatic retry.

## Evidence and exit behavior

`preparation.json` always records source hashes and the non-executable template.
The preparation-only invocation is:

```sh
python3 experiments/planner_comparison_matrix/sim_runtime/full_scene_mapping.py --output /absolute/new/preparation_directory
```

With reviewed route/oracle but without `--execute`, it also records the nominal
`route_geometry_audit.json`; no ROS context, relay or socket is created.
`runtime_ready.json` is written atomically only after fresh 10 s preflight and
anchor checks. It is evidence, not additional motion permission.

`state_trace.jsonl` retains original stamps/frames/pose/twist/receipt/age facts,
cloud-to-GT association, command packet and owner/visibility audit. Full cloud
bytes remain in the independently owned raw bag. A bounded dedicated writer
does JSON/I/O outside sensor callbacks. Overflow or writer error stops control
and marks evidence incomplete; raw MCAP does not excuse callback staleness.

On any exception/signal/timeout, cleanup first zeros only its owned relay,
observes advancing measured GT + commanded stop for 0.3 s (at most 8 s), closes
its stdin so the unchanged relay takes its EOF/final-zero path, waits 3 s, and
uses exact pidfd SIGINT plus another 3 s only if that same child remains. It
does not kill arbitrary processes, cancel Nav goals or stop the raw recorder.
Observation/context errors still reach child close. A missing measured stop,
unclosed child, close error or writer gap cannot produce successful acceptance.
Startup retains the Popen child's pidfd immediately, then waits for the frozen
relay's post-exec READY before sampling full argv/identity. Failure at any stage
closes that owned child; before READY the retained pidfd identifies it for the
bounded cleanup signal, without guessing another process by name.

Final `result.json` includes `classification`, `scope`, `mode`, `run_id`,
`owner_process_identity`, relay identity/hash/returncode/closed/errors,
`stop_verified`, `final_stop_observation` with original truth and zero command,
and writer completion/gap facts. `runtime_binding` binds root-release, route,
oracle, runtime sources, output, case, recording run and ready-receipt hashes.
Success classification is `controlled_static_sim_raw_acquisition_completed`;
`formal_navigation_acceptance` always remains false. The parent should inspect
result only after the exact controller pidfd exits and independently verify all
these fields. Its later camera/MCAP/coverage review is a separate acceptance.

The source-only fixtures cover filled-body corner/interior collisions,
intermediate rotation, stop reserve, raw occluder/empty/nonfinite/stale inputs,
original-stamp transforms, lying precheck, fixed bridge violations, old/alien
writer identities, once markers, deadline/source/receipt bindings, default HOLD,
and zero/close on observation failures. They do not run ROS, a simulator or UDP.
