# Epoch BackUp isolation experiment

This runner sends the real `BackUpEpoch` action to the production controller server.
It starts the actual navigation processes, velocity smoother and safety gate in
ROS domain 190, with a sensor-only SE2 plant. It never starts an SDK or hardware
bridge. The plant follows `/cmd_vel_safe`; the independent SAT geometry oracle
checks actual swept motion, not the production collision predicate.

`distance` is a **maximum accumulated travel budget**, not exact-distance arrival.
The controller stops early if its braking-safe speed is below the configured
minimum command speed. Success requires at least 1 cm of rearward projection
and fresh measured translation/angular velocity <=0.005 for 200 ms. Results expose
projection, accumulated path length, installed odometry origin and configured
minimum speed. A zero-effect request is not success.

The experiment copies the current smoother vx deadband into
`controller_server.epoch_backup_minimum_speed`. Neither this configured value,
the 0.2 m/s² relay deceleration requirement nor the model-only footprint is a
measurement of physical gait or actuator performance.

After sourcing ROS and this workspace:

```bash
python3 /home/charles/project/colcon_ws/src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/epoch_backup_validation/run_backup_stack.py \
  --workspace /home/charles/project/colcon_ws \
  --output /home/charles/project/colcon_ws/analysis_outputs/epoch_backup_trial_new \
  --scenario free
```

Each output directory must be new. Other cases: blocked, cancel, loss, epoch,
deadline, odom_freeze, moving_feedback, delayed_odom, small_budget, no_effective,
invalid, replan, slow_controller, slow_relay, slow_gate. `slow_controller` changes
only the independent BackUp frequency; ordinary MPPI retains its original rate.
`delayed_odom` preserves message stamps while delaying odometry 250 ms and sets
relay x deceleration to -0.2 m/s². `moving_feedback` keeps reporting -0.03 m/s after
the plant actually stops, so accepting the action as successful is a failure.

`--map` and `--initial x y yaw` select an existing map/environment. Without them,
a known synthetic free-room or rear-wall scene is generated. This is a software
closed loop, not a replay that could supply real counterfactual rear-facing scans.
The independent geometry audit uses the current model-only footprint; actual
swing-leg envelope, RK/SDK latency and real braking remain real-machine review work.

The opt-in BackUp action uses its own 20 Hz control loop. Its conservative reaction
budget is 0.3 s maximum odometry source lag + producer command TTL (0.3 s by
default) + controller/relay/gate periods (0.05 s each), or 0.75 s. Each component
rejects an incompatible frequency; relay deceleration must be at least 0.2 m/s².
The velocity cap solves v*t + v²/(2a) <= remaining path budget - 0.005 m.
The default production TRACK controller rate is unchanged.

After each run, score publication-time causality while retaining callback traces:

```bash
python3 /home/charles/project/colcon_ws/src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/epoch_backup_validation/audit_backup.py /absolute/output/directory
```

DDS missing timestamps, unstable wall-to-monotonic conversion or boundary ambiguity
are reported as unscorable. Historical callback-based audit failures remain in the
report; source publication times are not fabricated from observer receipt time.

For the complete automatic BT chain, use `run_auto_recovery.py` with the same
`--workspace` and a fresh `--output`. It sends only NavigateToPose. A temporary
traction constraint triggers the real 15 s progress checker; it releases on the
BT's BackUp installation, then requires the original mission to replan and reach
its frozen production XY/yaw goal tolerance. Both `enable_epoch_contract` and
`enable_epoch_backup_recovery` must be explicitly enabled; the navigator selects
only its packaged reviewed BackUp XML. Production defaults keep recovery off.

Follow-up direct cases `rear_during_braking` and `recorded_stall` record their
injection preconditions. The rear obstacle changes the same temporal world used
by scan generation and the independent oracle, and polls the real local costmap
for lethal cells. It is unverified if visibility is only observed after motion
has stopped. The stall case requires actual negative velocity before sleeping
350 ms; its expected outcome is fail/withdraw/stop. Existing-map free execution
and artificial executor stalls do not establish the cause of earlier unprofiled
scheduling delays.

Formal final audits now run after process close and plant finish. Full wall-time
motion is integrated through the simulated watchdog, including shutdown tails;
new complete constant-Twist intervals receive exact curved sweep checks. The
plant remains an ideal follower of the safe smoothed command, without joint or
gait dynamics. See `doc/epoch_backup_followup_review_20260911.md` for preserved
failures, protocol coverage limits and exact run manifests.
