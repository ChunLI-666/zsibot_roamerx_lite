# Copyright 2024 ZsiBot Team
# Licensed under the Apache License, Version 2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('cmd_vel_to_zsibot')
    
    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'params.yaml')
    
    # Declare launch arguments
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the configuration file'
    )
    
    declare_robot_ip = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.168.168',
        description='IP address of the ZsiBot robot'
    )
    
    declare_local_ip = DeclareLaunchArgument(
        'local_ip',
        default_value='192.168.168.2',
        description='Local IP address for UDP communication'
    )
    
    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='Topic name for velocity commands'
    )
    
    declare_auto_standup = DeclareLaunchArgument(
        'auto_standup',
        default_value='false',
        description='Automatically stand up the robot on startup'
    )

    # Create node
    cmd_vel_to_zsibot_node = Node(
        package='cmd_vel_to_zsibot',
        executable='cmd_vel_to_zsibot_node',
        name='cmd_vel_to_zsibot',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'robot_ip': LaunchConfiguration('robot_ip'),
                'local_ip': LaunchConfiguration('local_ip'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'auto_standup': LaunchConfiguration('auto_standup'),
            }
        ],
        remappings=[
            ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
        ]
    )

    return LaunchDescription([
        declare_config_file,
        declare_robot_ip,
        declare_local_ip,
        declare_cmd_vel_topic,
        declare_auto_standup,
        cmd_vel_to_zsibot_node,
    ])
