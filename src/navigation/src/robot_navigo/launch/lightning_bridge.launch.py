"""
Lightning Bridge Launch File

This launch file starts the Lightning Bridge node which provides:
1. TF Bridge: map->odom->base_link->livox_frame
2. Odometry Publisher: /odom/current_pose
3. Livox Converter: CustomMsg -> PointCloud2
4. LaserScan Generator: PointCloud2 -> LaserScan
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('robot_navigo')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # TF and Odom
    enable_tf_bridge = LaunchConfiguration('enable_tf_bridge')
    enable_odom_publisher = LaunchConfiguration('enable_odom_publisher')

    # Livox and LaserScan
    enable_livox_converter = LaunchConfiguration('enable_livox_converter')
    enable_laserscan = LaunchConfiguration('enable_laserscan')

    # Topics
    livox_input_topic = LaunchConfiguration('livox_input_topic')
    pointcloud_output_topic = LaunchConfiguration('pointcloud_output_topic')
    laserscan_output_topic = LaunchConfiguration('laserscan_output_topic')
    odom_output_topic = LaunchConfiguration('odom_output_topic')

    # LaserScan parameters
    target_frame = LaunchConfiguration('target_frame')
    min_height = LaunchConfiguration('min_height')
    max_height = LaunchConfiguration('max_height')
    angle_min = LaunchConfiguration('angle_min')
    angle_max = LaunchConfiguration('angle_max')
    angle_increment = LaunchConfiguration('angle_increment')
    range_min = LaunchConfiguration('range_min')
    range_max = LaunchConfiguration('range_max')

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    declare_enable_tf_bridge_cmd = DeclareLaunchArgument(
        'enable_tf_bridge',
        default_value='true',
        description='Enable TF bridge (odom->base_link, base_link->livox_frame)')

    declare_enable_odom_publisher_cmd = DeclareLaunchArgument(
        'enable_odom_publisher',
        default_value='true',
        description='Enable Odometry publisher (/odom/current_pose)')

    declare_enable_livox_converter_cmd = DeclareLaunchArgument(
        'enable_livox_converter',
        default_value='true',
        description='Enable Livox CustomMsg to PointCloud2 conversion')

    declare_enable_laserscan_cmd = DeclareLaunchArgument(
        'enable_laserscan',
        default_value='true',
        description='Enable LaserScan generation from PointCloud2')

    declare_livox_input_topic_cmd = DeclareLaunchArgument(
        'livox_input_topic',
        default_value='/livox/lidar',
        description='Input topic for Livox CustomMsg')

    declare_pointcloud_output_topic_cmd = DeclareLaunchArgument(
        'pointcloud_output_topic',
        default_value='/livox/lidar/pointcloud2',
        description='Output topic for PointCloud2')

    declare_laserscan_output_topic_cmd = DeclareLaunchArgument(
        'laserscan_output_topic',
        default_value='/laser_scan',
        description='Output topic for LaserScan')

    declare_odom_output_topic_cmd = DeclareLaunchArgument(
        'odom_output_topic',
        default_value='/odom/current_pose',
        description='Output topic for Odometry')

    declare_target_frame_cmd = DeclareLaunchArgument(
        'target_frame',
        default_value='base_link',
        description='Target frame for LaserScan')

    declare_min_height_cmd = DeclareLaunchArgument(
        'min_height',
        default_value='-0.5',
        description='Minimum height for LaserScan points')

    declare_max_height_cmd = DeclareLaunchArgument(
        'max_height',
        default_value='1.0',
        description='Maximum height for LaserScan points')

    declare_angle_min_cmd = DeclareLaunchArgument(
        'angle_min',
        default_value='-3.14159',
        description='Minimum angle for LaserScan')

    declare_angle_max_cmd = DeclareLaunchArgument(
        'angle_max',
        default_value='3.14159',
        description='Maximum angle for LaserScan')

    declare_angle_increment_cmd = DeclareLaunchArgument(
        'angle_increment',
        default_value='0.0087',
        description='Angle increment for LaserScan (~0.5 degrees)')

    declare_range_min_cmd = DeclareLaunchArgument(
        'range_min',
        default_value='0.5',
        description='Minimum range for LaserScan')

    declare_range_max_cmd = DeclareLaunchArgument(
        'range_max',
        default_value='50.0',
        description='Maximum range for LaserScan')

    # Lightning Bridge node
    lightning_bridge_node = Node(
        package='robot_navigo',
        executable='lightning_bridge.py',
        name='lightning_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # TF and Odom
            'enable_tf_bridge': enable_tf_bridge,
            'enable_odom_publisher': enable_odom_publisher,
            # Livox and LaserScan
            'enable_livox_converter': enable_livox_converter,
            'enable_laserscan': enable_laserscan,
            # Topics
            'livox_input_topic': livox_input_topic,
            'pointcloud_output_topic': pointcloud_output_topic,
            'laserscan_output_topic': laserscan_output_topic,
            'odom_output_topic': odom_output_topic,
            # LaserScan parameters
            'target_frame': target_frame,
            'min_height': min_height,
            'max_height': max_height,
            'angle_min': angle_min,
            'angle_max': angle_max,
            'angle_increment': angle_increment,
            'range_min': range_min,
            'range_max': range_max,
            'use_inf': True,
        }]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add declarations
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_enable_tf_bridge_cmd)
    ld.add_action(declare_enable_odom_publisher_cmd)
    ld.add_action(declare_enable_livox_converter_cmd)
    ld.add_action(declare_enable_laserscan_cmd)
    ld.add_action(declare_livox_input_topic_cmd)
    ld.add_action(declare_pointcloud_output_topic_cmd)
    ld.add_action(declare_laserscan_output_topic_cmd)
    ld.add_action(declare_odom_output_topic_cmd)
    ld.add_action(declare_target_frame_cmd)
    ld.add_action(declare_min_height_cmd)
    ld.add_action(declare_max_height_cmd)
    ld.add_action(declare_angle_min_cmd)
    ld.add_action(declare_angle_max_cmd)
    ld.add_action(declare_angle_increment_cmd)
    ld.add_action(declare_range_min_cmd)
    ld.add_action(declare_range_max_cmd)

    # Add node
    ld.add_action(lightning_bridge_node)

    return ld
