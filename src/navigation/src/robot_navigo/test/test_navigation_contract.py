#!/usr/bin/env python3

import pathlib
import re
import sys

import yaml


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def motion_constant(source, name):
    match = re.search(
        rf'constexpr\s+double\s+{name}\s*=\s*([0-9.]+)', source)
    require(match is not None, f'motion contract is missing {name}')
    return float(match.group(1))


def main():
    package_dir = pathlib.Path(sys.argv[1])
    params = yaml.safe_load((package_dir / 'params' / 'navigo_params.yaml').read_text())
    motion_header = (package_dir / 'include' / 'robot_navigo' / 'motion_limits.hpp').read_text()

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
    contract_max = [
        motion_constant(motion_header, 'kNavigationMaxVx'),
        motion_constant(motion_header, 'kNavigationMaxVy'),
        motion_constant(motion_header, 'kNavigationMaxWz'),
    ]
    contract_min = [
        motion_constant(motion_header, 'kExecutableMinVx'),
        motion_constant(motion_header, 'kExecutableMinVy'),
        motion_constant(motion_header, 'kExecutableMinWz'),
    ]
    require(velocity['max_velocity'] == contract_max,
            'velocity smoother and C++ navigation limits differ')
    require(velocity['deadband_velocity'] == contract_min,
            'velocity smoother and C++ executable minimums differ')

    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_map = params['global_costmap']['global_costmap']['ros__parameters']
    require(local['global_frame'] == 'odom',
            'local costmap must be isolated from map-frame localization corrections')
    require(params['behavior_server']['ros__parameters']['global_frame'] ==
            local['global_frame'],
            'behavior recovery frame must match the local collision costmap frame')
    require(params['bt_navigator']['ros__parameters']['bt_loop_duration'] == 100,
            'behavior tree loop must remain at the RK3588 production rate of 10 Hz')
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
    require('VelocityDeadbandCritic' in follow_path['critics'],
            'MPPI does not account for the executable velocity deadband')
    require(follow_path['VelocityDeadbandCritic']['deadband_velocities'] ==
            velocity['deadband_velocity'],
            'MPPI and velocity smoother deadbands differ')
    require(controller['FollowPath']['vx_min'] < 0.0,
            'normal reverse capability must remain available')
    require(controller['costmap_update_timeout'] <= 0.3,
            'controller costmap fail-closed timeout is too long')
    require(params['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False,
            'indoor production planner must not traverse unknown cells')
    require(controller['progress_checker']['plugin'].endswith('PoseProgressChecker'),
            'pose progress checking is not configured')
    require(params['smoother_server']['ros__parameters']['simple_smoother']['plugin'] ==
            'nav2_smoother::SimpleSmoother', 'configured smoother plugin does not exist')

    safety_gate = (package_dir / 'scripts' / 'nav_safety_gate.py').read_text()
    require("self.declare_parameter('cmd_timeout_ms', 300)" in safety_gate and
            'self.watchdog_timer = self.create_timer(' in safety_gate and
            'self.publish_zero(self.GATE_CMD_TIMEOUT)' in safety_gate,
            'navigation safety gate does not continuously enforce command timeout')

    replay_runner = (package_dir / 'scripts' / 'run_open_loop_model_replay.sh').read_text()
    require("grep -Eq '^active \\[3\\]$'" in replay_runner,
            'open-loop replay treats inactive lifecycle state as active')
    require('wait "$PLAYER_PID" || true' in replay_runner,
            'open-loop replay recording can end before its fixed input window')

    tree = (package_dir.parent / 'navigo_bt_navigator' / 'behavior_trees' /
            'navigate_to_pose_w_replanning_and_recovery.xml').read_text()
    require('<SmoothPath' in tree, 'default NavigateToPose tree does not smooth paths')
    require('unsmoothed_path="{raw_path}"' in tree, 'planner output bypasses SmoothPath')
    require('check_for_collisions="true"' in tree, 'smoothed path collision check is disabled')

    navigation_launch = (package_dir / 'launch' / 'navigation_launch.py').read_text()
    require('UnlessCondition(use_composition)' in navigation_launch,
            'standalone launch condition does not parse use_composition safely')
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
    require('TimerAction(' in navigation_launch and 'period=3.0' in navigation_launch,
            'standalone lifecycle manager is not delayed for DDS endpoint discovery')

    for launch_name in ('lightning_nav_bringup.launch.py', 'zsibot_nav_bringup.launch.py'):
        launch = (package_dir / 'launch' / launch_name).read_text()
        require("executable='livox_scan_projector'" in launch,
                f'{launch_name} does not use the direct scan projector')
        require("'enable_livox_converter',\n        default_value='false'" in launch,
                f'{launch_name} enables PointCloud2 conversion by default')

    zsibot_launch = (package_dir / 'launch' / 'zsibot_nav_bringup.launch.py').read_text()
    require("'max_linear_y': 0.15" in zsibot_launch,
            'cmd_vel_to_zsibot still clamps lateral navigation commands to zero')


if __name__ == '__main__':
    main()
