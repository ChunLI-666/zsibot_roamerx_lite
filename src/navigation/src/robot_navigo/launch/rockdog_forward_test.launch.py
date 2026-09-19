"""Explicit forward-only test profile on the existing non-epoch robot launch."""
import copy
import os
import tempfile
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetLaunchConfiguration, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def merge(target, override):
    for key, value in override.items():
        if isinstance(value, dict):
            merge(target.setdefault(key, {}), value)
        else:
            target[key] = copy.deepcopy(value)


def build_profile(base_file, share):
    """Pure profile assembly, also exercised offline without starting nodes."""
    share = Path(share)
    result = yaml.safe_load(Path(base_file).read_text())
    for name in ('rockdog_forward_test_overlay.yaml', 'zsl1_model_footprint_overlay.yaml'):
        merge(result, yaml.safe_load((share/'params'/name).read_text()))
    bt = result['bt_navigator']['ros__parameters']
    bt['default_nav_to_pose_bt_xml'] = str(share/'params/forward_test/navigate_to_pose.xml')
    bt['default_nav_through_poses_bt_xml'] = str(share/'params/forward_test/navigate_through_poses.xml')
    # No alternative BT can successfully call an automatic motion recovery server.
    result['behavior_server']['ros__parameters']['behavior_plugins'] = ['wait']
    return result


def configure(context):
    share = get_package_share_directory('robot_navigo')
    base = LaunchConfiguration('base_params_file').perform(context)
    params = build_profile(base, share)
    with tempfile.NamedTemporaryFile(mode='w', prefix='rockdog_forward_test_', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        path = stream.name
    def cleanup(_context):
        Path(path).unlink(missing_ok=True)
        return []
    return [SetLaunchConfiguration('params_file', path),
            LogInfo(msg=['Forward-motion test profile: ', path,
                '; model footprint, forward controller, stopped goal checker; automatic motion recoveries disabled.']),
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))]


def generate_launch_description():
    share = get_package_share_directory('robot_navigo')
    return LaunchDescription([
        DeclareLaunchArgument('base_params_file', default_value=os.path.join(share,'params/navigo_params.yaml'),
                              description='Existing deployment parameters; test motion settings are applied afterward.'),
        DeclareLaunchArgument('auto_standup', default_value='false'),
        OpaqueFunction(function=configure),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch/zsibot_nav_bringup.launch.py'))),
    ])
