# Smac planner profiles and motion primitives

## Explicit Hybrid field experiment (2026-10-31)

The canonical navigation branch is `fix/arm64-smac-planner` at base
`419f28b68b3713fbd5107a528dccec7222e9d663`, with uncommitted changes.
This identifies the source candidate, not the running robot build.
The legacy default remains `planner_backend:=fork_navfn`.

The reviewed field candidate explicitly selects:

```text
planner_backend:=smac_hybrid
smac_planner_params_file:=<robot_navigo-share>/params/smac_hybrid_field_experiment.yaml
map:=<LocMap-maps>/rock_dog22_refined_1cm_evidence5cm_20261031/map.yaml
```

Use the installed product launch/wrapper. No field shadow launch or external
lattice JSON is needed for Hybrid. The real new map has a different frame;
do not reuse old-bag start/goal coordinates or its recovery pose.

The official `nav2_planner/planner_server` hosts `SmacPlannerHybrid` in a
separate process with `/opt/ros/jazzy/lib` first, avoiding C++ ABI/SONAME
collisions with the Navigo fork. It uses the same standard planning Action
and path interface, with no new TF, odometry, or velocity source. The fork
planner is disabled. Global costmap plugins are official; the existing fork
ControllerServer/MPPI and local costmap remain the tracking implementation.
Smac Compute->Follow trees neither start nor lifecycle-manage the unused
SimpleSmoother server; legacy startup keeps it.

The profile uses DUBIN, a configured minimum curve radius of 0.5m, 72 heading
bins, 1cm global/local costmaps, no downsampling or path smoothing, and a 1m
heuristic lookup cache. Cache size does not bound feasible path length.
MPPI selects `DiffDrive` with `vy_max=0`, preserving the existing forward/yaw
speed limits, deadbands, footprint checks and downstream safety chain.
Vendor SDK documentation and current control software impose nonzero command
minimums (vx .05m/s, vy .10m/s, wz .02rad/s); these are not measured physical
minimum speeds. Disabling only the smoother deadband can expose downstream
small-command boosting and is not a deployable low-speed solution.

`navigate_to_pose_smac_hybrid_wait.xml` preserves SE2 path orientations and
uses Wait/retry/failure instead of Spin/BackUp/Clear recovery. Controller
angular commands remain possible and require validation. The 1000ms shared
Action-ACK/Service response wait and this recovery policy do not repair the
separate late-ACK cancellation/cleanup issue.

Evidence is deliberately separated: the installed x86/Jazzy 1.3.13 product
passed three fixed new-map SE2 closed-loop cases (two doorway runs and a
55-degree initial heading) with a bounded two-thread component executor.
A separate doorway case also passed using the actual field-default
`component_container_isolated --use_multi_threaded_executor`, with no bounded
container or thread override (133 threads observed). These four cases include
independent filled-body sweep and cleanup; the first three do not certify
default-container behavior, and thread count is not a performance guarantee.
ARM/Jazzy 1.3.11 passed new-map planner-only validation in about 100ms; it
does not validate controller, live localization/sensors, SDK or leg dynamics.
This source guide does not certify the complete ARM build, field activation
or physical-doorway acceptance; consult the current release manifests and
public delivery baseline for that status.

The official 1cm/0.5m/16-heading lattice generator timed out at 600s without
an output JSON; no old 5cm file was scaled or relabeled. Hybrid uses built-in
primitives and is independent of that unfinished route. Old-map failures and
the deadband-zero counterexample remain in the workspace analysis evidence.
See the workspace public baseline `docs/system_baseline/operations_delivery.md`
and `analysis_outputs/smac_integration_20261031` for exact manifests/results.

## Optional 5cm lattice resources (separate rollback experiments)

`omni_5cm_r050.json` is copied unchanged from the Apache-2.0 Nav2
`nav2_smac_planner` 1.3.13 sample `5cm_resolution/0.5m_turning_radius/omni/output.json`.
See the adjacent provenance JSON for its SHA-256.

The primitive set has 16 nonuniform headings and 144 trajectories, including
in-place rotations and lateral translations. The 0.5 m radius describes the
curved primitives; it is not a measured minimum turning radius of the dog.
Robot footprint and padding remain in the costmap configuration.

Use only with an actual 0.05 m global costmap. A static layer may adopt the
loaded map resolution; setting the costmap parameter alone is insufficient
when the loaded map is 0.01 m. A different resolution needs matching primitives.

This opt-in artifact does not establish actuator, dynamic tracking, or physical
doorway clearance acceptance. The default fork planner remains independent.

`diff_5cm_r050.json` is the corresponding unchanged upstream differential-drive
sample (112 trajectories, 32 in-place rotations, 16 headings). Its adjacent
provenance JSON records the separate hash. Pair it with MPPI `DiffDrive` and
`vy_max=0` through the explicit opt-in differential profile; do not pair Omni
lateral primitives with a controller that cannot execute lateral motion.
This profile preserves the existing nonzero forward/yaw command thresholds and
makes no claim that the physical SDK supports arbitrary low-speed commands.
