#!/usr/bin/env python3
"""Check the dependency graph seen by colcon/rosdep in both runtime profiles."""
from pathlib import Path
import sys
import unittest

from catkin_pkg.package import parse_package


PACKAGE_DIR = Path(sys.argv.pop(1)).resolve()


def dependencies(environment):
    package = parse_package(str(PACKAGE_DIR / 'package.xml'))
    package.evaluate_conditions(environment)
    return {dependency.name for group in (
        package.build_depends, package.build_export_depends, package.exec_depends)
        for dependency in group if dependency.evaluated_condition}


class ArmDependencyProfile(unittest.TestCase):
    def test_unset_environment_preserves_developer_dependencies(self):
        legacy = {'navigation2', 'slam_toolbox', 'OpenCV', 'yaml-cpp',
                  'python3-numpy', 'python3-scipy', 'rosbag2_py', 'ros2bag',
                  'rosbag2_storage_mcap'}
        self.assertTrue(legacy <= dependencies({}))
        self.assertEqual(dependencies({}), dependencies({'ROBOT_NAVIGO_ARM_RUNTIME': '0'}))

    def test_arm_graph_excludes_developer_stack_but_keeps_runtime_plugins(self):
        active = dependencies({'ROBOT_NAVIGO_ARM_RUNTIME': '1'})
        self.assertFalse(active & {'navigation2', 'slam_toolbox', 'OpenCV', 'yaml-cpp',
                                  'python3-numpy', 'python3-scipy', 'rosbag2_py',
                                  'ros2bag', 'rosbag2_storage_mcap'})
        self.assertTrue({'nav2_common', 'nav2_lifecycle_manager', 'nav2_smoother',
                         'navigo_path_planner', 'navigo_navfn_planner',
                         'navigo_path_controller', 'navigo_mppi_controller',
                         'navigo_bt_navigator', 'navigo_behavior_tree',
                         'navigo_map_server', 'navigo_costmap_2d', 'navigo_core',
                         'navigo_behaviors', 'navigo_velocity_optimizer',
                         'robot_debug_tools', 'robot_debug_msgs', 'robots_dog_msgs',
                         'navigo_epoch_msgs', 'livox_ros_driver2'} <= active)
        self.assertNotIn('navigo_waypoint_follower', active)


if __name__ == '__main__':
    unittest.main()
