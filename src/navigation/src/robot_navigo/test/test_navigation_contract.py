#!/usr/bin/env python3

import pathlib
import sys

import yaml


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    package_dir = pathlib.Path(sys.argv[1])
    params = yaml.safe_load((package_dir / 'params' / 'navigo_params.yaml').read_text())

    require('velocity_smoother' in params, 'velocity_smoother node key is missing')
    require('velocity_optimizer' not in params, 'stale velocity_optimizer node key is present')
    velocity = params['velocity_smoother']['ros__parameters']
    require(velocity['max_velocity'] == [0.15, 0.15, 0.1],
            'velocity smoother does not permit the configured lateral speed')
    require(velocity['min_velocity'] == [-0.15, -0.15, -0.1],
            'velocity smoother lateral reverse limit is inconsistent')
    require(velocity['deadband_velocity'] == [0.05, 0.10, 0.02],
            'SDK velocity deadbands are inconsistent')
    require(velocity['max_accel'][1] > 0.0 and velocity['max_decel'][1] < 0.0,
            'velocity smoother lateral acceleration is disabled')
    require(velocity['velocity_timeout'] <= 0.3, 'stale commands persist too long')

    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_map = params['global_costmap']['global_costmap']['ros__parameters']
    require(local['global_frame'] == 'odom',
            'local costmap must be isolated from map-frame localization corrections')
    for name, costmap in [('local', local), ('global', global_map)]:
        require('obstacle_layer' in costmap['plugins'], f'{name} obstacle layer is missing')
        require(costmap['obstacle_layer']['scan']['topic'] == '/laser_scan',
                f'{name} costmap is not observing /laser_scan')
        require(costmap['inflation_layer']['inflation_radius'] >= 0.35,
                f'{name} inflation is below the robot safety margin')
        require(costmap['always_send_full_costmap'] is False,
                f'{name} costmap still publishes full grids continuously')

    controller = params['controller_server']['ros__parameters']
    follow_path = controller['FollowPath']
    require(follow_path['motion_model'] == 'Omni', 'FollowPath is not using the Omni model')
    require(follow_path['vy_max'] == 0.15, 'MPPI lateral limit is inconsistent')
    require(follow_path['vx_max'] == velocity['max_velocity'][0],
            'MPPI vx limit differs from velocity smoother')
    require(follow_path['vy_max'] == velocity['max_velocity'][1],
            'MPPI vy limit differs from velocity smoother')
    require(follow_path['wz_max'] == velocity['max_velocity'][2],
            'MPPI wz limit differs from velocity smoother')
    require(controller['FollowPath']['vx_min'] < 0.0,
            'normal reverse capability must remain available')
    require(controller['costmap_update_timeout'] <= 0.3,
            'controller costmap fail-closed timeout is too long')
    require(controller['progress_checker']['plugin'].endswith('PoseProgressChecker'),
            'pose progress checking is not configured')
    require(params['smoother_server']['ros__parameters']['simple_smoother']['plugin'] ==
            'nav2_smoother::SimpleSmoother', 'configured smoother plugin does not exist')

    tree = (package_dir.parent / 'navigo_bt_navigator' / 'behavior_trees' /
            'navigate_to_pose_w_replanning_and_recovery.xml').read_text()
    require('<SmoothPath' in tree, 'default NavigateToPose tree does not smooth paths')
    require('unsmoothed_path="{raw_path}"' in tree, 'planner output bypasses SmoothPath')
    require('check_for_collisions="true"' in tree, 'smoothed path collision check is disabled')

    navigation_launch = (package_dir / 'launch' / 'navigation_launch.py').read_text()
    require("'smoother_server'" in navigation_launch,
            'smoother_server is absent from the lifecycle set')
    require("package='nav2_smoother'" in navigation_launch,
            'nav2_smoother server is not launched')
    require('namespace=namespace' in navigation_launch,
            'standalone nav2_smoother does not follow the navigation namespace')
    require(navigation_launch.count("package='nav2_smoother'") == 1,
            'nav2_smoother must stay in one ABI-isolated process')
    require("additional_env={'LD_LIBRARY_PATH': smoother_library_path}" in navigation_launch,
            'nav2_smoother does not isolate upstream costmap libraries')

    for launch_name in ('lightning_nav_bringup.launch.py', 'zsibot_nav_bringup.launch.py'):
        launch = (package_dir / 'launch' / launch_name).read_text()
        require("executable='livox_scan_projector'" in launch,
                f'{launch_name} does not use the direct scan projector')
        require("'enable_livox_converter',\n        default_value='false'" in launch,
                f'{launch_name} enables PointCloud2 conversion by default')

    zsibot_launch = (package_dir / 'launch' / 'zsibot_nav_bringup.launch.py').read_text()
    require("'max_linear_y': 0.15" in zsibot_launch,
            'cmd_vel_to_zsibot still clamps lateral navigation commands to zero')

    matrix_launch = (package_dir / 'launch' /
                     'matrix_lightning_closed_loop.launch.py').read_text()
    require("package='pub_tf'" not in matrix_launch and
            "executable='pub_tf'" not in matrix_launch and
            '/odom/ground_truth' not in matrix_launch,
            'Matrix Lightning closed loop must not use pub_tf or ground-truth odometry')
    require("executable='livox_scan_projector'" in matrix_launch,
            'Matrix closed loop does not use the C++ scan projector')
    require("'input_type': 'pointcloud2'" in matrix_launch and
            "'livox_input_topic': '/livox/lidar'" in matrix_launch and
            "'laserscan_output_topic': '/laser_scan'" in matrix_launch,
            'Matrix PointCloud2 bridge topics or conversion mode are inconsistent')
    require("'cmd_vel_topic': '/cmd_vel_safe'" in matrix_launch or
            "('/cmd_vel', '/cmd_vel_safe')" in matrix_launch,
            'Matrix LCM adapter bypasses the navigation safety gate')
    require("'use_sim_time': False" in matrix_launch and
            "'use_sim_time': 'false'" in matrix_launch,
            'Matrix closed loop must use wall-clock time in bridge and navigation')
    require("'waypoint_follower_package': 'nav2_waypoint_follower'" in matrix_launch,
            'Matrix Humble bringup does not select its compatible waypoint follower')
    require("'waypoint_task_executor_plugin': "
            "'nav2_waypoint_follower::WaitAtWaypoint'" in matrix_launch,
            'Matrix Humble waypoint task plugin is incompatible')
    require('cwd=lightning_cwd' in matrix_launch,
            'Matrix launch cannot resolve relative point-cloud paths in map index')
    require("'lcm_channel': 'interface'" in matrix_launch and
            "'matrix_legacy_gamepad_schema': True" in matrix_launch and
            "'lcm_type_hash_override': 0x040E446810CD12CD4" in matrix_launch and
            "'navigation_mode': 1" in matrix_launch and
            "'publish_rate_hz': 50.0" in matrix_launch,
            'Matrix command adapter does not match the packaged mc_ctrl wire contract')

    matrix_adapter = (package_dir / 'src' / 'vel_cmd_lcm_publisher.cpp').read_text()
    require('encoded.begin() + kLegacyFieldOffset' in matrix_adapter and
            'encoded.insert' in matrix_adapter,
            'Matrix legacy encoder does not add the packaged decoder field')
    require('PublishLatestCommand' in matrix_adapter and
            '1.0 / publish_rate_hz' in matrix_adapter,
            'Matrix command adapter does not publish cached commands at a fixed rate')
    require('latest_vx_ = 0.0;' in matrix_adapter and
            'latest_vy_ = 0.0;' in matrix_adapter and
            'latest_wz_ = 0.0;' in matrix_adapter,
            'Matrix stand-up can replay a cached nonzero velocity')
    require("'vx_to_stick_scale': 2.0" in matrix_launch and
            "'vy_to_stick_scale': 1.0 / 0.3" in matrix_launch and
            "'wz_to_stick_scale': 1.0" in matrix_launch,
            'Matrix physical velocity commands are not normalized to gamepad axes')

    matrix_e2e = (package_dir / 'scripts' / 'matrix_closed_loop_e2e.py').read_text()
    require("'/odom/mujoco_odom'" in matrix_e2e,
            'Matrix E2E test does not record evaluation ground truth')
    require("lookup_transform(\n                'map', 'base_link'" in matrix_e2e,
            'Matrix E2E test does not score the global Lightning pose')
    require(matrix_e2e.count('self.create_publisher(') == 1 and
            'PoseStamped,' in matrix_e2e and
            "'/matrix_closed_loop/goal_pose'" in matrix_e2e and
            'TransformBroadcaster' not in matrix_e2e,
            'Matrix E2E evaluator publishes more than the action-goal mirror')

    matrix_runner = (package_dir / 'scripts' / 'matrix_closed_loop_run.sh').read_text()
    require('"$MATRIX_ROOT/scripts/run_sim.sh"' in matrix_runner,
            'Matrix runner does not use the no-ground-truth-TF simulator entry')
    require('"$MATRIX_ROOT/run_sim.sh"' not in matrix_runner,
            'Matrix runner can invoke the legacy pub_tf entry')
    require('--qos-reliability best_effort' in matrix_runner,
            'Matrix sensor readiness probes do not match BEST_EFFORT publishers')
    require('for attempt in 1 2' in matrix_runner and
            'matrix.retry.log' in matrix_runner,
            'Matrix runner does not retry a failed simulator cold start')
    require('timeout 180 ros2 run robot_navigo matrix_closed_loop_preflight.sh' in
            matrix_runner,
            'Matrix closed-loop preflight budget is shorter than its bounded checks')
    require(matrix_runner.index('matrix_closed_loop_preflight.sh closed-loop') <
            matrix_runner.index('wait_for_topic /odom/current_pose 180'),
            'Matrix ownership preflight consumes the ACTIVE-state holdover window')
    require('bash --noprofile --norc' in matrix_runner and
            'unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH' in
            matrix_runner and
            'unset LD_LIBRARY_PATH PYTHONPATH RMW_IMPLEMENTATION' in matrix_runner,
            'Matrix process is not isolated from Lightning/Nav overlay libraries')
    require('start_lightning:=false' in matrix_runner and
            'standing Matrix robot before localization initialization' in matrix_runner and
            'starting Lightning after stand-up settling' in matrix_runner,
            'Matrix runner does not stand and settle the robot before localization')
    require(matrix_runner.index('standing Matrix robot before localization initialization') <
            matrix_runner.index('starting Lightning after stand-up settling'),
            'Matrix stand-up/localization startup order is reversed')
    require('for stand_attempt in 1 2 3' in matrix_runner and
            'timeout 30 ros2 service call "$STAND_SERVICE"' in matrix_runner and
            'ros2 service type "$STAND_SERVICE"' not in matrix_runner,
            'Matrix stand-up still depends on flaky ROS CLI graph discovery')
    require('matrix_navigo.runtime.yaml' in matrix_runner and
            "for costmap_name in ('local_costmap', 'global_costmap')" in matrix_runner and
            "costmap['transform_tolerance'] = 0.3" in matrix_runner and
            "['expected_update_rate'] = 0.5" in matrix_runner and
            "['costmap_update_timeout'] = 0.6" in matrix_runner and
            "['FollowPath']['transform_tolerance'] = 0.5" in matrix_runner and
            'params_file:="$RUNTIME_NAV_PARAMS"' in matrix_runner,
            'Matrix costmap timing override is missing or can affect real-robot params')
    require('readlink -f "/proc/$pid/cwd"' in matrix_runner and
            'readlink -f "/proc/$pid/exe"' in matrix_runner,
            'Matrix cleanup can kill unrelated host controller processes')
    require('stop_bag()' in matrix_runner and
            'kill -INT -- "-$pid"' in matrix_runner and
            'stop_bag "$BAG_PID"' in matrix_runner,
            'Matrix runner does not let rosbag finalize metadata on shutdown')
    require('RENDER_MODE=visible' in matrix_runner and
            '--headless) RENDER_MODE=offscreen' in matrix_runner and
            'wait_for_visible_matrix_window' in matrix_runner and
            'xdotool search --onlyvisible --pid' in matrix_runner and
            'window_info.txt' in matrix_runner,
            'Matrix runner does not default to or verify visible rendering')
    require('verify_recording()' in matrix_runner and
            '/matrix_closed_loop/goal_pose' in matrix_runner and
            '--include-hidden-topics' in matrix_runner,
            'Matrix runner does not enforce a complete regression recording')
    require("'/matrix_closed_loop/goal_pose'" in matrix_e2e and
            'self.goal_pose_publisher.publish(goal.pose)' in matrix_e2e,
            'Matrix action goal is not mirrored into the regression bag')

    controller_server = (package_dir.parent / 'navigo_path_controller' / 'src' /
                         'controller_server.cpp').read_text()
    goal_check_start = controller_server.index('bool ControllerServer::isGoalReached()')
    goal_check_body = controller_server[goal_check_start:goal_check_start + 1800]
    require('if (!nav_2d_utils::transformPose(' in goal_check_body and
            'Cannot evaluate goal reached' in goal_check_body and
            'return false;' in goal_check_body,
            'Controller can report goal reached after a failed goal transform')

    require("parser.add_argument('--max-yaw-rmse'" in matrix_e2e and
            "parser.add_argument('--max-yaw-error'" in matrix_e2e and
            "parser.add_argument('--goal-yaw-tolerance'" in matrix_e2e,
            'Matrix E2E mixes linear and angular acceptance thresholds')
    require("'smoother_startup_sec'" in matrix_e2e and
            "'safety_gate_sec'" in matrix_e2e and
            "'--max-smoother-startup-latency'" in matrix_e2e and
            "'--max-safety-gate-latency'" in matrix_e2e,
            'Matrix E2E does not distinguish smoother and safety-gate latency')
    require("'controller_command_rate'" in matrix_e2e and
            "controller_stats['max_interval_sec'] <= 0.25" in matrix_e2e,
            'Matrix E2E allows controller recovery to hide command dropouts')
    require("'source_frequency_hz'" in matrix_e2e and
            "odom_stats['source_max_interval_sec'] <= 0.25" in matrix_e2e and
            "'timing_basis': 'receive_monotonic'" in matrix_e2e,
            'Matrix E2E conflates source cadence with recorder arrival jitter')
    require("'localization_status_rate'" in matrix_e2e and
            "'safety_gate_remained_open'" in matrix_e2e and
            "'/nav_safety_gate/gate_status'" in matrix_e2e,
            'Matrix E2E does not detect frozen localization or a closed safety gate')
    require("self.gate_monitoring_active = True" in matrix_e2e and
            "if self.gate_monitoring_active and msg.data != LOC_NORMAL" in matrix_e2e and
            "self.gate_monitoring_active = False" in matrix_e2e,
            'Matrix E2E counts expected idle CMD_TIMEOUT as an active-motion failure')

    matrix_preflight = (package_dir / 'scripts' /
                        'matrix_closed_loop_preflight.sh').read_text()
    require('deadline=$((SECONDS + 10))' in matrix_preflight,
            'Matrix preflight does not tolerate bounded DDS discovery churn')
    require(matrix_preflight.count('deadline=$((SECONDS + 10))') >= 3,
            'Matrix endpoint ownership checks do not retry DDS discovery')
    require('--qos-reliability best_effort' in matrix_preflight,
            'Matrix preflight liveness probes do not match sensor QoS')
    require('deadline=$((SECONDS + 15))' in matrix_preflight,
            'Matrix liveness probes do not tolerate bounded DDS discovery churn')
    require(matrix_preflight.count('--no-daemon') >= 3,
            'Matrix preflight still depends on stale ROS CLI daemon discovery')
    require(matrix_preflight.count('--spin-time 0.5') >= 3 and
            '--spin-time 2' not in matrix_preflight,
            'Matrix preflight serial graph discovery consumes localization holdover')
    require('timeout 8 ros2 action list' not in matrix_preflight,
            'Matrix preflight still uses daemon-only action discovery')
    require('ActionClient' in matrix_e2e and 'server_is_ready()' in matrix_e2e,
            'Matrix E2E does not perform native action-server readiness checking')
    require('GetState' in matrix_e2e and 'State.PRIMARY_STATE_ACTIVE' in matrix_e2e and
            "f'/{name}/get_state'" in matrix_e2e,
            'Matrix E2E does not verify lifecycle state through native services')
    require('_wait_for_lifecycle_active' in matrix_e2e and
            'Lifecycle node did not become active' in matrix_e2e,
            'Matrix E2E does not tolerate lifecycle activation latency')
    require('ros2 lifecycle get' not in matrix_preflight,
            'Matrix preflight still uses unreliable lifecycle CLI discovery')

    require("default_value='navigo_waypoint_follower'" in navigation_launch,
            'real-robot waypoint follower default was changed')


if __name__ == '__main__':
    main()
