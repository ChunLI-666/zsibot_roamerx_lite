# Whole-Warehouse raw recorder parent, v1

Source-only preparation; real recording and controller execution remain HOLD pending root review and a separate GO. This directory adds no ROS Node, TF/clock/velocity publisher, source restart, controller launcher, controller signal, or deployment. It never changes a sampling stamp or re-encodes camera bytes. The existing canonical `recording_deadline.py` is copied unchanged; the parent registers only `recorder`.

The parent creates a new output directory and freezes its four runtime sources, config/release, every root-enumerated script/ELF/scene/config/mesh/oracle/route artifact, actual Python ELF and ros2 CLI script. A new once marker precedes the first Popen, including the read-only Git snapshot. It then records each canonical repository's exact HEAD, porcelain status and binary diff; an unexpected HEAD/dirty state fails before recording. All untracked participating sources must be enumerated as artifacts. Root must provide a complete inventory; the program cannot infer an omitted external mesh, plugin or scene resource.

The exact recorder command is:

```text
/opt/ros/jazzy/bin/ros2 bag record --node-name full_scene_raw_recorder_<recording_run_id> -a --include-hidden-topics -s mcap --storage-preset-profile zstd_fast --max-cache-size 268435456 -o <new_output>/bag
```

The unchanged helper arms a detached Linux pidfd guard before the recorder. Its dormant owned-exec wrapper cannot exec the recorder until the guard has retained and acknowledged its pidfd; the parent also retains its own recorder pidfd. Ready requires verified exec, live exact source identities, and an opened MCAP file with its original magic. `recorder_ready.json` binds source/frozen-input/Git-snapshot/config/package hashes, exact recorder identity, `node_name`/`node_namespace` (also `recorder_node_*` aliases), case/run IDs and a steady deadline. It explicitly reports `graph_subscription_verified:false` and `authorizes_controller_or_motion:false`. Root must inspect the actual recorder subscriptions and current source readiness before separately releasing the controller.

The recording budget is finite and at most 3600 seconds from the parent process start, including freeze and startup. At recorder ready its remaining budget must cover the preparation wait (at most 120 seconds), the controller's complete finite budget, an additional 20-second raw tail, and 15 seconds of close reserve. The controller's own budget already includes its 25-second cleanup reserve. This parent does not change the existing Nav 3600-second limit or source-age bounds `[-0.05,0.3]`.

After ready, root creates the separately reviewed mapping root release, launches the exact frozen controller with `MAPPING_OWNER_RUN_ID=<controller release_id>` and `MAPPING_OWNER_OUTPUT=<new_output>/controller`, and immediately writes `controller_binding.json` atomically. The binding carries the exact five-field identity (`pid`, `starttime_ticks`, `cmdline_sha256`, `argv`, `boot_id`), release path/hash, ready hash, scope, case, both run IDs and output directory. The parent opens and retains this controller pidfd before checking `/proc` identity/environment and writes `controller_binding_ack.json`. The descriptor is observation-only and is never passed to a signal call. A controller that exits before it can be bound is rejected; root should bind immediately after Popen. No binding within the bounded preparation wait closes the recorder normally.

`mapping_result_adapter.py` matches the frozen `full_scene_mapping.py` result schema: scope/mode/run/owner identity, exact runtime binding including ready SHA, completed classification, original-frame measured zero/stop, relay closed with zero return code/no close errors, and closed complete evidence writer. Stop freshness is checked at the controller's own observation instant: original header age `[-0.05,0.3]` seconds and monotonic receipt age `[0,0.3]` seconds. The bound process must have exited before its result is accepted. A failure, malformed/unbound result, or missing result closes the recorder immediately after observing that exit. Only a successful result earns the extra 20-second raw tail; this is additional to the controller's own end-stability interval. Its basis is the controller's measured stop and closed sole relay, with original sources still identity-checked; the parent does not independently subscribe to `/odom` during the tail. Tail motion acceptance remains a raw-data review task.

On normal cleanup the parent sends SIGINT only through its retained recorder pidfd after rechecking the exact wrapper/exec identity and original run/output/role/launch environment. There are no process-group signals, TERM/KILL escalation, source/controller signals, restarts, overwrite, or deletion. Parent exit or a blocked parent is covered by the existing detached guard. Its deadline is a bounded SIGINT trigger, not a guarantee against a kernel/D-state hang, ignored SIGINT or failed storage. A parent killed before audit may leave only helper events plus a closed bag; root must then run the read-only audit separately.

After recorder exit, the guard is disarmed before slow hashing. `recording_result.json` includes metadata hash, streaming MCAP SHA/size, topic counts, summary schema hashes/encoding/size and count parity. The summary reader follows the [official MCAP specification](https://mcap.dev/spec), reads only footer/summary records, and never decodes compressed chunks or CDR/image payloads. It checks required topics have positive counts/nonempty CDR schemas and matching totals. It does not claim per-message timestamps, JPEG byte equality, payload CRC, replay or mapping acceptance from summary alone. All original CDR bytes, including full camera `data`, are preserved by the recorder command.

## Root staging sequence (not executed here)

1. Fill `recording_config.HOLD.template.json` with a new case/run/output, the final reviewed whole-scene route/oracle/controller hashes, exact controller argv, current environment/source identities and complete participating artifact inventory. Include scene host-process identity and generation evidence in the `scene_binding` artifact; only local same-namespace source identities are probed directly. Set exact canonical repository HEAD/status/diff expectations after controller sources are frozen.
2. Copy the reviewed four runtime files together. Source the original `samehost_runtime/network/environment.bash`, then `message_runtime/environment.bash` (message-only prefix). Fill the recorder-only release template using the actual config/package hashes, current boot ID/net namespace and unique output. `allow_execute` stays false until root GO. Defaults do not authorize any process.
3. A preparation check creates no process or output directory:

   ```bash
   /usr/bin/python3 <reviewed_package>/full_scene_recording_owner.py --config <config.json> --output <new_output>
   ```

4. After recorder-only GO, use the same command with `--execute --root-release <recording_release.json>` from a detached, logged parent. Wait for `<new_output>/recorder_ready.json`, verify its real hash, source evidence and actual subscriptions. Ready is recording-only.
5. Build the controller's separate root release with the exact ready path/hash, identity, node names, required topics and deadline. Launch the reviewed controller with the two environment bindings and immediately atomically write `controller_binding.json`. The parent never launches it. Review `controller_binding_ack.json` and the controller's own `runtime_ready.json` as evidence; they add no new motion authorization.
6. Inspect `recording_result.json`, `controller_result.frozen.json`, MCAP metadata and hashes. A full mapping success additionally requires `recording_evidence_complete:true`; a successful process return or summary alone is insufficient. The old corner map and 3/27-point routes are not inputs to this package.

The templates remain HOLD and contain deliberately incomplete identities/paths. No real runtime command has been executed by this task.

## Offline verification

`standin_tests_v7/test_result.json` records eight local Python stand-in process cases (extra success tail, failed/missing result, finite no-client wait, exact-identity rejection, parent deadline, abrupt parent exit, once-before-first-Popen) and two pure result-adapter tests rejecting stale original header/receipt times. Fixtures also exercise summary/metadata count agreement with an advertised zero-message topic. Fixture injection exists only in `_test_runner.py`, outside the production CLI. No ROS, real recorder, socket, SDK or deployment is used.

`helper_evidence_reuse.json` verifies helper SHA `e850bc78bfe46d0678cfd1984995c688bf4f7ad76c1b806ccfb36c4bc944b80c` and test SHA `66b437b1712d66b7a276e3a50e64eddab6baa344d80d18509ba9b772f7e7c0e0` match canonical, originally tested native-waypoint review/committed sources and a retained process-test run. The existing 18-case evidence is `native_waypoints_v1/deadline_process_tests_v1`; its independent terminal record reports 18 tests/OK. Those tests were not rerun here. Earlier `standin_tests_v1` preserves an adapter clock-domain failure; the fix compares monotonic stop evidence to the parent's monotonic binding instant instead of `/proc` BOOTTIME start ticks. All original failed evidence is retained.

Codacy analyze tools are unavailable in the active tool inventory. No analyzer pass or dependency installation is claimed. No Git branch switch, commit, push or worktree operation is performed.

## Root source corrections, v3

The parent now reads only stable PID/start/boot birth fields after retaining
the recorder pidfd; full wrapper/exec identities are checked after the helper's
exec ACK. Controller release binding also requires the identical scene instance.
If the controller exits between a pidfd readiness check and a `/proc` read,
only a second readable check of that retained pidfd permits the normal exited
result path. An identity mismatch or an unreadable pidfd still fails.

The original failed `standin_tests_v8_root_fix` is retained. The corrected
`standin_tests_v9_root_fix` passed all 12 cases, including early empty argv and
wrong-scene regressions. These are harmless local process fixtures; actual ROS
recording and movement remain subject to a separate root runtime release.
# v5 canonical source export provenance delta

For a runtime source export without `.git`, a reviewed `repositories` entry may
use `kind: "frozen_source_export"`; see the two new HOLD examples. Root must fill
the absolute `export_root` and in-export `manifest_path`, exact manifest SHA,
complete `expected_origin`, and `root_reviewed: true`. The root release still
binds the exact config and current four-file package SHA. Existing complete
artifact-inventory review remains mandatory; the manifest must appear there
with the same absolute path and SHA, so the original freeze copies it.

The manifest has exactly `schema: "canonical_git_source_export.v1"`, `origin`,
and nonempty `files_sha256`. Origin has exactly canonical_repository_path,
branch, full40hex head, status_porcelain and diff_sha256. These are canonical
origin metadata, not observed Git on the runtime host. Head does not substitute
for dirty status, diff digest, or actual exported source bytes.

The export contains exactly its listed participating source leaves plus the
manifest. Paths are normalized relative POSIX paths with no dot/parent escape;
absolute source paths, symlink roots/ancestors/directories/leaves/manifests,
special files and unlisted leaves are rejected. No-follow descriptor traversal
and streamed SHA verification run before release and again at recording start.
The run evidence retains the original manifest and every source verification,
with `observed_git_on_runtime_host: false`. It does not claim runtime Git status.

The legacy/default Git snapshot body is unchanged. Recording, owner/identity,
age, deadline3600, controller binding, stop/result and20s success-tail gates are
unchanged. The v5 delta is source provenance only; no runtime release is granted.

