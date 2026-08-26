"""
Lightning Navigation Bringup Launch File

This launch file starts the complete navigation stack with Lightning-LM integration:
1. Lightning Bridge (TF + Odom + Livox + LaserScan)
2. Navigation Safety Gate
3. Navigation stack (via bringup_launch.py)

Log management:
    All node logs are written to ~/log/nav_YYYYMMDD_HHMMSS/
    A combined 'all.log' aggregates all nodes with timestamps.
    A symlink ~/log/nav_latest always points to the most recent session.

Usage:
    ros2 launch robot_navigo lightning_nav_bringup.launch.py \
        map:=/path/to/map.yaml \
        params_file:=/path/to/navigo_params.yaml \
        use_sim_time:=true

Note: Lightning-LM localization should be started separately before running this launch file.
"""

import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    SetEnvironmentVariable,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _setup_log_dir(context):
    """Create timestamped log directory and 'latest' symlink."""
    log_base = os.path.expanduser('~/log')
    os.makedirs(log_base, exist_ok=True)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_dir = os.path.join(log_base, f'nav_{timestamp}')
    os.makedirs(log_dir, exist_ok=True)

    latest_link = os.path.join(log_base, 'nav_latest')
    if os.path.islink(latest_link):
        os.unlink(latest_link)
    os.symlink(log_dir, latest_link)

    return [
        LogInfo(msg=f'Nav logs: {log_dir}'),
        SetEnvironmentVariable('ROS_LOG_DIR', log_dir),
        # Aggregate all node logs into one file for real-time monitoring
        ExecuteProcess(
            cmd=['bash', '-c',
                 f'sleep 2 && tail -F {log_dir}/*.log > {log_dir}/all.log 2>/dev/null'],
            name='log_aggregator',
            output='log',
        ),
    ]


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('robot_navigo')
    pkg_dir = get_package_share_directory('robot_navigo')
    launch_dir = os.path.join(pkg_dir, 'launch')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    nav2_delay = LaunchConfiguration('nav2_delay')
    nav_cmd_timeout_ms = LaunchConfiguration('nav_cmd_timeout_ms')

    # Lightning Bridge configurations
    enable_livox_converter = LaunchConfiguration('enable_livox_converter')
    enable_laserscan = LaunchConfiguration('enable_laserscan')
    waypoint_follower_package = LaunchConfiguration('waypoint_follower_package')
    waypoint_follower_executable = LaunchConfiguration('waypoint_follower_executable')
    waypoint_follower_plugin = LaunchConfiguration('waypoint_follower_plugin')
    waypoint_task_executor_plugin = LaunchConfiguration('waypoint_task_executor_plugin')

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bag) clock if true')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'params', 'navigo_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_nav2_delay_cmd = DeclareLaunchArgument(
        'nav2_delay',
        default_value='3.0',
        description='Delay (seconds) before starting Nav2 to wait for TF chain')

    declare_nav_cmd_timeout_cmd = DeclareLaunchArgument(
        'nav_cmd_timeout_ms',
        default_value='300',
        description='Stop actuator output if smoothed cmd_vel is stale for this many milliseconds')

    declare_enable_livox_converter_cmd = DeclareLaunchArgument(
        'enable_livox_converter',
        default_value='false',
        description='Enable optional Livox CustomMsg to PointCloud2 visualization output')

    declare_enable_laserscan_cmd = DeclareLaunchArgument(
        'enable_laserscan',
        default_value='true',
        description='Enable direct Livox CustomMsg to LaserScan projection')

    declare_waypoint_follower_package_cmd = DeclareLaunchArgument(
        'waypoint_follower_package', default_value='navigo_waypoint_follower')
    declare_waypoint_follower_executable_cmd = DeclareLaunchArgument(
        'waypoint_follower_executable', default_value='waypoint_follower')
    declare_waypoint_follower_plugin_cmd = DeclareLaunchArgument(
        'waypoint_follower_plugin',
        default_value='navigo_waypoint_follower::WaypointFollower')
    declare_waypoint_task_executor_plugin_cmd = DeclareLaunchArgument(
        'waypoint_task_executor_plugin',
        default_value='navigo_waypoint_follower::WaitAtWaypoint')

    # Optional visualization path. It is deliberately outside the navigation
    # hot path so PointCloud2 packing cannot delay obstacle observations.
    pointcloud_bridge = Node(
        condition=IfCondition(enable_livox_converter),
        package='robot_navigo',
        executable='lightning_bridge.py',
        name='lightning_pointcloud_bridge',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_tf_bridge': False,
            'enable_odom_publisher': False,
            'enable_livox_converter': True,
            'enable_laserscan': False,
            'publish_lidar_static_tf': False,
            'sensor_qos_depth': 1,
            'max_sensor_age_sec': 0.3,
            'livox_input_topic': '/livox/lidar',
            'pointcloud_output_topic': '/livox/lidar/pointcloud2',
            'bridge_debug_topic': '/lightning_pointcloud_bridge/debug',
        }]
    )

    # Navigation hot path: one C++ pass over CustomMsg, no intermediate cloud.
    scan_projector = Node(
        condition=IfCondition(enable_laserscan),
        package='robot_navigo',
        executable='livox_scan_projector',
        name='livox_scan_projector',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'sensor_qos_depth': 1,
            'max_sensor_age_sec': 0.3,
            'livox_input_topic': '/livox/lidar',
            'laserscan_output_topic': '/laser_scan',
            'bridge_debug_topic': '/lightning_bridge/debug',
            'target_frame': 'base_link',
            'min_height': 0.05,
            'max_height': 0.45,
            'angle_min': -3.141592653589793,
            'angle_max': 3.141592653589793,
            'angle_increment': 0.0087,
            'range_min': 0.1,
            'range_max': 50.0,
            'exclude_robot_footprint': True,
            'use_inf': True,
        }]
    )

    # Step 2: Navigation Safety Gate (blocks cmd_vel when localization not NORMAL)
    nav_safety_gate = Node(
        package='robot_navigo',
        executable='nav_safety_gate.py',
        name='nav_safety_gate',
        output='both',
        parameters=[{
            'watchdog_timeout_ms': 200,
            'cmd_timeout_ms': ParameterValue(nav_cmd_timeout_ms, value_type=int),
            'cmd_vel_input_topic': '/cmd_vel',
            'cmd_vel_output_topic': '/cmd_vel_safe',
            'loc_status_topic': '/lightning/loc_status',
            'emergency_stop_topic': '/emergency_stop',
        }]
    )

    # Step 3: Navigation stack (delayed start to wait for TF chain)
    nav2_launch_delayed = TimerAction(
        period=nav2_delay,
        actions=[
            LogInfo(msg='Starting Navigo navigation stack after waiting for TF chain...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'bringup_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml_file,
                    'params_file': params_file,
                    'autostart': autostart,
                    'waypoint_follower_package': waypoint_follower_package,
                    'waypoint_follower_executable': waypoint_follower_executable,
                    'waypoint_follower_plugin': waypoint_follower_plugin,
                    'waypoint_task_executor_plugin': waypoint_task_executor_plugin,
                }.items()
            )
        ]
    )

    # Create launch description
    ld = LaunchDescription()

    # Log management (must be first - sets ROS_LOG_DIR before nodes start)
    ld.add_action(OpaqueFunction(function=_setup_log_dir))

    # Add declarations
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_nav2_delay_cmd)
    ld.add_action(declare_nav_cmd_timeout_cmd)
    ld.add_action(declare_enable_livox_converter_cmd)
    ld.add_action(declare_enable_laserscan_cmd)
    ld.add_action(declare_waypoint_follower_package_cmd)
    ld.add_action(declare_waypoint_follower_executable_cmd)
    ld.add_action(declare_waypoint_follower_plugin_cmd)
    ld.add_action(declare_waypoint_task_executor_plugin_cmd)

    # Start immediately: sensor projection + optional visualization + safety gate
    ld.add_action(pointcloud_bridge)
    ld.add_action(scan_projector)
    ld.add_action(nav_safety_gate)

    # Delayed start: Navigation stack
    ld.add_action(nav2_launch_delayed)

    return ld
