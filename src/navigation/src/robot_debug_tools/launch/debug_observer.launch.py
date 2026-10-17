"""Start only the passive Debug observer, without launching navigation or hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use /clock during local replay/simulation; set false for live observation.'),
        Node(
            package='robot_debug_tools', executable='debug_observer',
            name='debug_observer', output='screen',
            parameters=[{'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)}],
        ),
    ])
