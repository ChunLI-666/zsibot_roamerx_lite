# Matrix official waypoint follower overlay

The C++ class, header and executable entry point are pinned to navigation2
1.3.13, commit `f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501`. `UPSTREAM.json`
records the original files and hashes; `callback_nonblocking.patch` is the only
behavior change. The upstream header and main are unchanged.

An old NavigateToPose result can arrive after `future_goal_handle_` has been
replaced by a new pending goal response. Blocking on that future inside the
mutually exclusive callback group prevents its own executor from processing
the response. The guard checks validity and readiness without waiting, then
null handle and UUID, before applying the original result status handling.

This relies on the reviewed Jazzy rclcpp_action contract: the send-goal response
sets its promise before invoking the goal-response callback and before
`make_result_aware()` sends GetResult. A legitimate current result therefore
cannot arrive before its own future is ready. No generation counter is needed.
Rejected goals retain the upstream FAILED response handling. Reassess this
ordering when changing ROS or Nav2 versions.

The package and core SONAME are unique. It loads the original underlay task
plugins; it neither replaces `/opt` nor uses the old Navigo 1.1.18 fork. Matrix
`--with-waypoints` requires `--waypoint-install` plus
`--waypoint-build-receipt`, with verified source, executable and mapped DSO.
Existing original geometry, stop rules, budgets and exclusive action ownership
are unchanged. Cancel remains exact at the FollowWaypoints parent client;
upstream child cancellation still requires exclusive NavigateToPose ownership.

`COLCON_IGNORE` keeps this experiment out of automatic workspace discovery.
Build directly with CMake in a new isolated prefix against `/opt/ros/jazzy`; never install over an
existing receipt. `callback_probe` exercises real DDS and the linked core in
localhost domain 190. It exposes the production child callback seam, not the
full parent loop. Full parent tests additionally cover success, rejection,
repeated nonzero index starts and cancellation. Synthetic tests do not establish
the exact B40 runtime cause, Matrix acceptance or physical acceptance.
