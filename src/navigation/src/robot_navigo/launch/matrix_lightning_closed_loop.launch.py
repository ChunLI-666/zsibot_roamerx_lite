"""Matrix closed-loop navigation driven exclusively by Lightning localization.

Matrix physics and motor control must already be running via Matrix's run_sim.sh.
Do not run pub_tf or run_sim_with_nav.sh: those publish ground-truth navigation TF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_navigo_share = get_package_share_directory('robot_navigo')
    default_params = os.path.join(robot_navigo_share, 'params', 'navigo_params.yaml')

    lightning_config = LaunchConfiguration('lightning_config')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    start_lightning = LaunchConfiguration('start_lightning')
    nav2_delay = LaunchConfiguration('nav2_delay')
    lightning_cwd = LaunchConfiguration('lightning_cwd')
    matrix_min_abs_vx = LaunchConfiguration('matrix_min_abs_vx')
    matrix_min_abs_vy = LaunchConfiguration('matrix_min_abs_vy')
    matrix_min_abs_wz = LaunchConfiguration('matrix_min_abs_wz')

    lightning = ExecuteProcess(
        condition=IfCondition(start_lightning),
        cmd=['ros2', 'run', 'lightning', 'run_loc_online',
             '--config', lightning_config,
             '--ros-args', '-p', 'use_sim_time:=false'],
        cwd=lightning_cwd,
        name='matrix_lightning_localization',
        output='both',
    )

    # Matrix publishes PointCloud2, while the real robot publishes Livox
    # CustomMsg. Both use the same C++ projection core; only the subscriber
    # type differs, so this path has no Python per-point conversion overhead.
    scan_projector = Node(
        package='robot_navigo',
        executable='livox_scan_projector',
        name='matrix_pointcloud_scan_projector',
        output='both',
        parameters=[{
            'use_sim_time': False,
            'input_type': 'pointcloud2',
            'livox_input_topic': '/livox/lidar',
            'laserscan_output_topic': '/laser_scan',
            'bridge_debug_topic': '/lightning_bridge/debug',
            'target_frame': 'base_link',
            # Matrix config.json uses centimetres for sensor mounting offsets.
            'lidar_x': 0.13011,
            'lidar_y': 0.02329,
            'lidar_z': 0.17598,
            'lidar_roll': 0.0,
            'lidar_pitch': 0.0,
            'lidar_yaw': 0.0,
            'min_height': 0.05,
            'max_height': 0.45,
            'range_min': 0.1,
            'range_max': 8.5,
            'sensor_qos_depth': 1,
            'max_sensor_age_sec': 0.5,
        }],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_navigo_share, 'launch', 'lightning_nav_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map': map_yaml,
            'params_file': params_file,
            'nav2_delay': nav2_delay,
            'nav_cmd_timeout_ms': '300',
            # Isolate lifecycle, costmap and controller callbacks. On the
            # Matrix host, one delayed component previously starved bonds and
            # shut the complete composed navigation stack down.
            'use_composition': 'False',
            # The Matrix PointCloud2 projector above owns /laser_scan.
            'enable_laserscan': 'false',
            'enable_livox_converter': 'false',
            # The custom waypoint follower targets a newer Nav2 API than the
            # server's Humble installation. It is not part of NavigateToPose.
            'waypoint_follower_package': 'nav2_waypoint_follower',
            'waypoint_follower_executable': 'waypoint_follower',
            'waypoint_follower_plugin': 'nav2_waypoint_follower::WaypointFollower',
            'waypoint_task_executor_plugin': 'nav2_waypoint_follower::WaitAtWaypoint',
        }.items(),
    )

    # The safety gate owns the actuator boundary. Remapping the absolute input
    # prevents this adapter from bypassing localization loss or emergency stop.
    matrix_command_adapter = Node(
        package='robot_navigo',
        executable='vel_cmd_lcm_pub',
        name='matrix_vel_cmd_lcm_publisher',
        output='both',
        parameters=[{
            'platform': 'MUJOCO',
            'cmd_vel_topic': '/cmd_vel_safe',
            # Matrix mc_ctrl subscribes to this gamepad_lcmt channel.
            'lcm_channel': 'interface',
            # The packaged Matrix mc_ctrl uses an older, wire-compatible LCM
            # schema whose generated type hash differs from this repository.
            'lcm_type_hash_override': 0x040E446810CD12CD4,
            'matrix_legacy_gamepad_schema': True,
            'navigation_mode': 1,
            'publish_rate_hz': 50.0,
            'cmd_timeout_ms': 300,
            'min_abs_vx': ParameterValue(matrix_min_abs_vx, value_type=float),
            'min_abs_vy': ParameterValue(matrix_min_abs_vy, value_type=float),
            'min_abs_wz': ParameterValue(matrix_min_abs_wz, value_type=float),
            # ROS commands are physical SI units; Matrix gamepad analog axes
            # are normalized by the configured gait limits (0.5/0.3/1.0).
            'vx_to_stick_scale': 2.0,
            'vy_to_stick_scale': 1.0 / 0.3,
            'wz_to_stick_scale': 1.0,
            # Matrix gamepad +X is right strafe, opposite ROS base_link +Y.
            'invert_lateral_axis': True,
            'enable_stance_service': True,
            'stand_button_hold_ms': 500,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'lightning_config',
            description='Absolute path to the Matrix Lightning localization YAML'),
        DeclareLaunchArgument(
            'lightning_cwd', default_value=os.getcwd(),
            description='Working directory used to resolve relative PCD paths in index.txt'),
        DeclareLaunchArgument(
            'map',
            description='Absolute path to map.yaml from the same Lightning map'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Navigo parameter file'),
        DeclareLaunchArgument(
            'start_lightning', default_value='true',
            description='Start run_loc_online in this launch'),
        DeclareLaunchArgument(
            'nav2_delay', default_value='15.0',
            description='Nav startup delay; safety gate still blocks until localization is NORMAL'),
        DeclareLaunchArgument(
            'matrix_min_abs_vx', default_value='0.05',
            description='Matrix adapter minimum executable absolute X velocity; set 0 for ablation'),
        DeclareLaunchArgument(
            'matrix_min_abs_vy', default_value='0.10',
            description='Matrix adapter minimum executable absolute Y velocity; set 0 for ablation'),
        DeclareLaunchArgument(
            'matrix_min_abs_wz', default_value='0.02',
            description='Matrix adapter minimum executable absolute yaw rate; set 0 for ablation'),
        lightning,
        scan_projector,
        navigation,
        matrix_command_adapter,
    ])
