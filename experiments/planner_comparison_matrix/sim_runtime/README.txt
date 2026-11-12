Matrix simulation command transport

sim_command_adapter.py is the preserved simulation adapter with an explicit
--relay-mode ssh|local selector. The default SSH destination remains the approved
simulation host charles@192.168.1.219. Local mode executes the adjacent
sim_udp_relay.py using the current Python interpreter. The explicit --sim-host
selector allows only 127.0.0.1 (default) or the simulation host 192.168.1.219,
with fixed UDP7447. Private network namespaces must use --relay-mode local
--sim-host 192.168.1.219 to reach the host UE. No physical robot is targeted.

Before starting local mode, verify the Matrix process and its selected endpoint,
and confirm the previous acquisition/SSH command writer and relay have closed
with zero commands. The child READY line must name the selected host:7447.
Nonloopback hosts are rejected in SSH mode; its original remote command is retained. Only one command adapter/relay may write to Matrix.
Use the isolated domain185 loopback ROS environment with native Lightning pose
and the existing safety gate. Record the command/debug topics in every run.

The original velocity bounds, unsupported-axis rejection, 0.3s odometry age,
0.05s future limit, 0.25s command timeout, relay 0.2s age and 0.25s watchdog are
unchanged. The relay sends five zero packets when stdin closes. Nominal policy
velocity is not a calibrated SI body response, and small commands are not boosted.

Local mode removes the SSH hop; an explicit host supports the private namespace. Offline guard and mocked watchdog checks
do not establish actual runtime behavior, stopping performance or navigation
acceptance; the same-host pipeline still needs full recording/load validation.
