"""
ZsiBot Full Navigation Launch File

This launch file starts the complete navigation stack for real robot deployment:
1. Lightning Bridge (TF + Odom + Livox + LaserScan)
2. Navigation Safety Gate
3. Navigo Navigation Stack
4. cmd_vel_to_zsibot (velocity command to robot SDK)
5. Emergency Stop node

Log management:
    All node logs are written to ~/log/nav_YYYYMMDD_HHMMSS/
    A combined 'all.log' aggregates all nodes with timestamps.
    A symlink ~/log/nav_latest always points to the most recent session.

Usage:
    ros2 launch robot_navigo zsibot_nav_bringup.launch.py \
        robot_ip:=192.168.234.1 \
        local_ip:=192.168.234.100 \
        map:=/path/to/map.yaml

Note: Lightning-LM localization should be started separately.
"""

import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    GroupAction,
    SetEnvironmentVariable,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
        ExecuteProcess(
            cmd=['bash', '-c',
                 f'sleep 2 && tail -F {log_dir}/*.log > {log_dir}/all.log 2>/dev/null'],
            name='log_aggregator',
            output='log',
        ),
    ]


def generate_launch_description():
    # Get package directories
    navigo_pkg = get_package_share_directory('robot_navigo')
    cmd_vel_pkg = get_package_share_directory('cmd_vel_to_zsibot')
    navigo_launch_dir = os.path.join(navigo_pkg, 'launch')

    # ==================== Launch Configurations ====================
    # Robot network
    robot_ip = LaunchConfiguration('robot_ip')
    local_ip = LaunchConfiguration('local_ip')
    local_port = LaunchConfiguration('local_port')

    # Navigation
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    nav2_delay = LaunchConfiguration('nav2_delay')

    # Robot behavior
    auto_standup = LaunchConfiguration('auto_standup')
    enable_emergency_stop = LaunchConfiguration('enable_emergency_stop')

    # Sensor processing
    enable_livox_converter = LaunchConfiguration('enable_livox_converter')
    enable_laserscan = LaunchConfiguration('enable_laserscan')

    # ==================== Declare Arguments ====================
    # Robot network arguments
    declare_robot_ip_cmd = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.234.1',
        description='IP address of the ZsiBot robot')

    declare_local_ip_cmd = DeclareLaunchArgument(
        'local_ip',
        default_value='192.168.234.100',
        description='Local IP address for UDP communication with robot')

    declare_local_port_cmd = DeclareLaunchArgument(
        'local_port',
        default_value='43988',
        description='Local UDP port for robot communication')

    # Navigation arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (false for real robot)')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(navigo_pkg, 'params', 'navigo_params.yaml'),
        description='Full path to the Navigo parameters file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_nav2_delay_cmd = DeclareLaunchArgument(
        'nav2_delay',
        default_value='3.0',
        description='Delay before starting Nav2 to wait for TF chain')

    # Robot behavior arguments
    declare_auto_standup_cmd = DeclareLaunchArgument(
        'auto_standup',
        default_value='false',
        description='Automatically stand up robot on startup (USE WITH CAUTION!)')

    declare_enable_emergency_stop_cmd = DeclareLaunchArgument(
        'enable_emergency_stop',
        default_value='true',
        description='Launch emergency stop node')

    # Sensor processing arguments
    declare_enable_livox_converter_cmd = DeclareLaunchArgument(
        'enable_livox_converter',
        default_value='true',
        description='Enable Livox CustomMsg to PointCloud2 conversion')

    declare_enable_laserscan_cmd = DeclareLaunchArgument(
        'enable_laserscan',
        default_value='true',
        description='Enable LaserScan generation')

    # ==================== Nodes ====================

    # 1. Lightning Bridge (TF + Odom + Livox + LaserScan)
    lightning_bridge = Node(
        package='robot_navigo',
        executable='lightning_bridge.py',
        name='lightning_bridge',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_tf_bridge': False,
            'enable_odom_publisher': False,
            'enable_livox_converter': enable_livox_converter,
            'enable_laserscan': enable_laserscan,
            'livox_input_topic': '/livox/lidar',
            'pointcloud_output_topic': '/livox/lidar/pointcloud2',
            'laserscan_output_topic': '/laser_scan',
            'odom_output_topic': '/odom/current_pose',
            'target_frame': 'base_link',
            'min_height': 0.05,
            'max_height': 0.45,
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

    # 2. Navigation Safety Gate (blocks cmd_vel when localization not NORMAL)
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

    # 3. cmd_vel_to_zsibot (velocity to robot SDK)
    cmd_vel_to_zsibot = Node(
        package='cmd_vel_to_zsibot',
        executable='cmd_vel_to_zsibot_node',
        name='cmd_vel_to_zsibot',
        output='both',
        parameters=[{
            'robot_ip': robot_ip,
            'local_ip': local_ip,
            'local_port': local_port,
            'cmd_vel_topic': '/cmd_vel_safe',
            'max_linear_x': 0.15,
            'max_linear_y': 0.0,
            'max_angular_z': 0.1,
            'cmd_timeout': 0.5,
            'publish_rate': 100.0,
            'auto_standup': auto_standup,
        }]
    )

    # 4. Emergency Stop node (optional but recommended)
    emergency_stop = Node(
        condition=IfCondition(enable_emergency_stop),
        package='robot_navigo',
        executable='emergency_stop.py',
        name='emergency_stop',
        output='both',
        parameters=[{
            'emergency_stop_topic': '/emergency_stop',
            'publish_rate': 50.0,
        }],
        prefix='xterm -e',  # Launch in separate terminal for keyboard input
    )

    # 5. Navigation stack (delayed start)
    nav2_launch_delayed = TimerAction(
        period=nav2_delay,
        actions=[
            LogInfo(msg='Starting Navigo navigation stack...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(navigo_launch_dir, 'bringup_launch.py')
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

    # ==================== Launch Description ====================
    ld = LaunchDescription()

    # Log management (must be first - sets ROS_LOG_DIR before nodes start)
    ld.add_action(OpaqueFunction(function=_setup_log_dir))

    # Declare arguments
    ld.add_action(declare_robot_ip_cmd)
    ld.add_action(declare_local_ip_cmd)
    ld.add_action(declare_local_port_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_nav2_delay_cmd)
    ld.add_action(declare_auto_standup_cmd)
    ld.add_action(declare_enable_emergency_stop_cmd)
    ld.add_action(declare_enable_livox_converter_cmd)
    ld.add_action(declare_enable_laserscan_cmd)

    # Start nodes
    ld.add_action(LogInfo(msg='Starting ZsiBot Navigation System...'))
    ld.add_action(LogInfo(msg='  [1/5] Lightning Bridge'))
    ld.add_action(lightning_bridge)

    ld.add_action(LogInfo(msg='  [2/5] Navigation Safety Gate'))
    ld.add_action(nav_safety_gate)

    ld.add_action(LogInfo(msg='  [3/5] cmd_vel_to_zsibot'))
    ld.add_action(cmd_vel_to_zsibot)

    ld.add_action(LogInfo(msg='  [4/5] Emergency Stop (if enabled)'))
    ld.add_action(emergency_stop)

    ld.add_action(LogInfo(msg='  [5/5] Navigo Navigation (delayed)'))
    ld.add_action(nav2_launch_delayed)

    return ld
