# Minimal passive navigation diagnostics

This package provides artifact build identity, bounded producer-local sideband
handoff and a separate read-only observer. It never calls TF, costmaps, algorithms
or business services. Source algorithms keep their original decisions.

After building the overlay, run:

```sh
ros2 launch robot_debug_tools debug_observer.launch.py use_sim_time:=true
```

Use `use_sim_time:=false` for wall-clock operation. This launch starts only the
observer. Source nodes expose `enable_debug_observations` (default true) to turn
off optional sideband sampling; artifact metadata remains available.
For C++ teardown fault isolation only, additionally setting
`enable_debug_build_info=false` creates neither new C++ publisher nor worker.
The default is true; this is not the normal version-traceable acceptance mode.

`/debug/observations` contains actual checker returns, finite original branch
observations, and transport/wrapper facts. `/debug/view` is a bounded display
projection. `/debug/build_info` comes from each actual compiled artifact or
installed Python package. Standard business messages remain unchanged.

Action state is keyed by endpoint and full UUID. Checker records are tied to a
FollowPath UUID and installed path revision; cancellation, terminal state, path
replacement, unevaluated preconditions and an observation gap invalidate the
current display. XY/yaw differences use the two actual checker inputs and mean
current followed path endpoint geometry, not navigation remaining path length.

Native Action source binding requires exact publisher identity. Jazzy does not
expose publisher GID in Python MessageInfo, so this binding stays unknown; node
name uniqueness alone is insufficient across restart. Typed source observations
still carry their own embedded build identity. Metadata never substitutes the
observer's commit for an unknown external source or plugin.

Publication happens on independent workers. Business callbacks use bounded
nonblocking queue handoff and contain diagnostic exceptions. Dropped counters
are observability loss, not algorithm health. The observer counter records bounded
view-cache evictions; it is not a DDS packet-loss measurement. SDK sendto/wrapper returns do not
prove successful physical execution. Never subscribe to the old heavy
`/controller_server/debug` in the normal observer/recording layout.

Local validation entry points are in
`robot_navigo/scripts/minimal_debug_validation/README.md`; they isolate ROS to
localhost and never launch hardware adapters. Historical bag replay cannot
validate arrival. Closed-loop SE2 validation runs real navigation algorithms but
does not validate actual robot dynamics, real SDK or Lightning on synthetic data.
