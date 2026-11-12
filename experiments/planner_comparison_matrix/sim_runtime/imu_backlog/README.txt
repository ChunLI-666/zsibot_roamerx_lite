SIM-only continuous-IMU backlog preservation candidate (HOLD)

Files
imu_backlog_gate.py: pure Gate subclass, original frozen CDR decoder reused.
adapter_backlog_shim.py: inherits the read-only frozen Adapter; only specializes
IMU gate/audit and diagnostics. Does not copy/change lidar conversion, IMU
conversion, TF, camera or source-clock code.
test_imu_backlog.py: no ROS; actual pure Gate and AST-extracted shim methods.
source_manifest.json, verification.json, candidate.patch, tests.output.txt.

Frozen dependencies
../full_scene_private_sensor_bootstrap_v2/source/{imu_gate.py,
adapter_with_gate.py,matrix_sensor_contract.py}; SHA checked before import/use.
These original files and private_sensor_adapter.py are unchanged.

Policy
imu_backlog_max_age_sec defaults to .3 and may explicitly be set in [.3,1].
The .3 default delegates the entire inspect call to the original Gate, keeping
the original decisions/state/events. It is not a true-header-gap relaxation.
When enabled, an original queued sample older than .3 but no older than the
configured maximum can pass only with an existing accepted anchor, correct
frame, all37 finite numbers, strictly advancing original stamp and actual
original-header gap <=100000000ns. Original future allowance stays50ms;
callback receipt age stays<=300ms; wall/steady receipt cannot regress.
Every true original-header gap >100ms, including100ms+1ns, remains fatal;
bad/future/missing/malformed/too-old inputs remain rejected. No anchor repair.

Each accepted old queued sample generates marked_backlog=true, actual gap,
original age, limits, reason, original CDR/base64/hash and permanent
strict_input_timing_status=FAIL, strict_input_timing_accepted=false.
The original fresh() remains <=.3s. Forwarding old queued samples does not
declare them current or permit control while TF/Odom freshness is invalid.
After catching up, diagnostics retains WARN/strict FAIL; while not fresh it
still reports ERROR. No threshold/guard in Nav, scan, TF/Odom or body changes.

Adapter audit
Topic /simulation/sensor_adapter/imu_backlog (reliable depth1024) and a required
new append-only JSONL path preserve every marked sample. Each line is written
and flushed before forwarding, and the file is fsynced on controlled teardown.
There is no fsync for every2ms IMU, avoiding synchronous disk waits on each
queued sample. OS/power-failure durability of each individual pre-close record
is not claimed. Audit write/flush or publish failures raise before forwarding;
there is no silent drop or unmarked fallback.
The inherited imu_to_local deep-copies numeric values and stamp and changes
only the known frame label to imu_link. Original raw wire CDR stays in audit.

Root integration only; this agent did not execute any source
Executing this shim as a script always prints ROOT_HOLD and imports no ROS.
Only Root's separate exact-release/PID-guarded launcher may call:
    adapter_backlog_shim.run(runtime_config,
        imu_backlog_max_age_sec=1.0, backlog_audit_path=NEW_JSONL_PATH)
The module directory must be on sys.path. Domain185 is checked by run().
Root owns the no-Nav stop/restart of old adapter140 and LM/SDK ownership, new
source release/runtime SHA binding, and controls reuse of bridge47/projector97/
UE/leveler2476. No live process is modified here. No new approval flow added.

Why the previous candidate is not for execution
Sibling native_marked_imu_gap_candidate_solmax_v1 is retained analysis only.
Root's compiled-LM/LIO evidence shows a true>.1s IMU gap is unsafe to bridge
with one latest sample. This candidate preserves continuous2ms queued original
samples and never allows a larger original-header gap.

Verification boundary
18 no-ROS tests include a350-sample virtual2ms queue after750ms scheduling
delay, with236 marked samples, original timestamps unchanged and final fresh
under.3; strict FAIL persists. Old policy on this virtual queue rejects stale
then rejects accepted-anchor gaps. Tests include100ms/+1ns,1s/+1ns,
future50ms/+1ns, all37 numeric fields, both CDR endian forms and -0 bits,
receipt regression, no initial old anchor/no756s anchor repair, audit failure,
default HOLD, inherited conversion/TF/camera and sticky diagnostics severity.
This is pure/source evidence, not a live DDS/load/LM navigation acceptance.
No Node, ROS CLI, goal, live build or process restart occurred.
Codacy MCP analysis tool is unavailable; not run, no dependency installed.

Canonical deployment notes
The frozen base directory is an explicit absolute workspace experiment path,
pinned by SHA. Controlled SIGINT/SIGTERM set a stop flag with rclpy signal
handling disabled; audit flush+fsync and Node destruction precede ROS shutdown.
start_native_tracking_inputs.py owns only LM and SDK children; default HOLD
validates a closed Native full-T_map_imu seed without treating it as current.
Root's separate source supervisor retires the failed sensor actor and starts
this shim. Bridge, legacy scan projector, UE and the separate leveled projector
remain retained. Source PID/hash binding must identify the new sensor actor.
