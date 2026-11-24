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

## Offline MCAP replay

Build and source this package, then use the same self-contained MCAP you open in
Foxglove. Install the optional desktop Python packages separately (for example in
a virtual environment with access to the sourced ROS overlay).

```sh
python3 -m pip install mcap mcap-ros2-support
ros2 run robot_debug_tools debug_replay_view viewing.mcap debug-043.html \
  --start 282.9 --duration 1.4
```

Open the generated HTML in a browser. The input is an indexed MCAP file and
times are relative to its first log timestamp. Expand an event to compare its
recorded facts, original record locator and complete decoded message. At narrow
widths the two panes stack. Use `--topic` (repeatable) or a shorter time window to
focus on one module. The report shows when the requested record limit truncated
the view.

For an older split bag, run the command on one original MCAP split at a time.
If the file has empty Action schemas, pass `--schemas-from action_channels_recovered.mcap`
with the matching historical schema companion. Only definitions for missing
schemas are read from the companion. An unresolvable record keeps its time,
topic and CDR hash and is marked as unavailable for decoding. The tool reads
ROS 2 CDR with `ros2msg` schemas. It never publishes, starts a ROS node, or
rewrites a recording. Use the native Foxglove layout for 3D and normal replay.

ROS header time, MCAP log/publish time, BT event time and watchdog durations
remain independent. Action UUIDs are displayed per goal. Invalid values and
unbound build metadata remain unknown. A ROS zero-velocity output is evidence of
a command, with physical execution separate. A recorded observer snapshot is
shown with the observation and publication times it contains.

## Building an immutable Git archive

Normal checkouts retain the existing HEAD/dirty-source fingerprint behavior.
A source directory without Git metadata remains unknown by default. For an
archive build, explicitly set `ROBOT_DEBUG_SOURCE_MANIFEST_LIGHTNING` (or
`ROBOT_DEBUG_SOURCE_MANIFEST_NAVIGATION`, according to the target repository ID)
to an absolute proof JSON path. The generator also accepts `--source-manifest`.
This is a build identity capability; it does not change algorithm decisions.

The proof has exactly these fields:

```json
{
  "schema": "robot_debug.source_manifest.v1",
  "repository_id": "lightning",
  "git_repository": "/absolute/actual/repository",
  "git_commit": "<full 40-character actual commit object ID>",
  "git_tree": "<that commit's full 40-character tree ID>",
  "source_root": "/absolute/immutable/archive",
  "files": [
    {"path": "src/example.cc", "mode": "100644", "git_blob": "<40-character blob ID>", "sha256": "<64-character raw blob SHA256>"}
  ]
}
```

`files` must enumerate the entire committed tree, including non-code files and
symlinks. The generator checks real commit/tree objects in `git_repository`, every
path, Git file mode, blob ID and SHA256 against the archive, and rejects missing
or extra files, submodules, source-root aliases and escaping/cyclic symlinks.
Symlink hashes cover the raw link target bytes. The normal `--repo` directory may
be a subdirectory of `source_root`. Keep proof files, build outputs and generated
Python caches outside the immutable source root.

An explicit invalid proof fails the build before emitting a new identity. A valid
proof yields the actual review commit and `dirty_state=1`, meaning the verified
archive exactly matches that commit; it makes no claim about the canonical
checkout's dirty state. Create and retain a real review commit for unmerged
sources; a commit string or another checkout's HEAD is insufficient. Build IDs
bind the exact proof bytes, commit/tree, repository/artifact IDs and actual target
build context. The wire BuildStamp keeps its original six fields; the validated
proof is installed alongside `debug_build.json` as `source_manifest.json`.
Use an independently built tool prefix when adding this capability to a frozen
experiment; do not overwrite historical generators or evidence.
