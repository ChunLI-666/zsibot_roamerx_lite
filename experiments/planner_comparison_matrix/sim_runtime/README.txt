Matrix simulation command transport

sim_command_adapter.py is the preserved simulation adapter with an explicit
--relay-mode ssh|local selector. The default SSH destination remains the approved
simulation host charles@192.168.1.219. Local mode executes the adjacent unchanged
sim_udp_relay.py using the current Python interpreter. It requires Matrix on the
same host at UDP 127.0.0.1:7447; it does not use an SDK or a physical robot target.

Before starting local mode, verify the Matrix process and its loopback endpoint,
stop the earlier SSH command writer by its verified PID, and confirm its relay
closed with zero commands. Only one command adapter/relay may write to Matrix.
Use the isolated domain185 loopback ROS environment with native Lightning pose
and the existing safety gate. Record the command/debug topics in every run.

The original velocity bounds, unsupported-axis rejection, 0.3s odometry age,
0.05s future limit, 0.25s command timeout, relay 0.2s age and 0.25s watchdog are
unchanged. The relay sends five zero packets when stdin closes. Nominal policy
velocity is not a calibrated SI body response, and small commands are not boosted.

Local mode removes the SSH hop only. Offline guard and mocked watchdog checks
do not establish actual runtime behavior, stopping performance or navigation
acceptance; the same-host pipeline still needs full recording/load validation.
