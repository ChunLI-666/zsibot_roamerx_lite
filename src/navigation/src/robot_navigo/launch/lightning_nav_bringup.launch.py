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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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

    # Lightning Bridge configurations
    enable_livox_converter = LaunchConfiguration('enable_livox_converter')
    enable_laserscan = LaunchConfiguration('enable_laserscan')

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

    declare_enable_livox_converter_cmd = DeclareLaunchArgument(
        'enable_livox_converter',
        default_value='true',
        description='Enable Livox CustomMsg to PointCloud2 conversion')

    declare_enable_laserscan_cmd = DeclareLaunchArgument(
        'enable_laserscan',
        default_value='true',
        description='Enable LaserScan generation from PointCloud2')

    # Step 1: Lightning Bridge (starts immediately)
    # Provides: TF bridge, Odometry publisher, Livox converter, LaserScan generator
    lightning_bridge = Node(
        package='robot_navigo',
        executable='lightning_bridge.py',
        name='lightning_bridge',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            # TF and Odom are now published natively by lightning_slam.
            'enable_tf_bridge': False,
            'enable_odom_publisher': False,
            # Livox and LaserScan (configurable)
            'enable_livox_converter': enable_livox_converter,
            'enable_laserscan': enable_laserscan,
            # Topics
            'livox_input_topic': '/livox/lidar',
            'pointcloud_output_topic': '/livox/lidar/pointcloud2',
            'laserscan_output_topic': '/laser_scan',
            'odom_output_topic': '/odom/current_pose',
            # LaserScan parameters
            'target_frame': 'base_link',
            'min_height': 0.4,
            'max_height': 0.5,
            'enable_ground_filter': False,
            'ground_filter_distance': 0.12,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'range_min': 0.5,
            'range_max': 50.0,
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
    ld.add_action(declare_enable_livox_converter_cmd)
    ld.add_action(declare_enable_laserscan_cmd)

    # Start immediately: Lightning Bridge + Safety Gate
    ld.add_action(lightning_bridge)
    ld.add_action(nav_safety_gate)

    # Delayed start: Navigation stack
    ld.add_action(nav2_launch_delayed)

    return ld
