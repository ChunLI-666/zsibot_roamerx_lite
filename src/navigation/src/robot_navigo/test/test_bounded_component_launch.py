#!/usr/bin/env python3
"""Resolve container launch selection without executing nodes or includes."""
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node


PACKAGE_DIR = Path(sys.argv.pop(1)).resolve()


def resolved(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def load_description(filename):
    spec = importlib.util.spec_from_file_location('container_launch_under_test',
                                                 PACKAGE_DIR / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch.object(module, 'get_package_share_directory', return_value=str(PACKAGE_DIR)):
        return module.generate_launch_description()


def default_context(description, **overrides):
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    context.launch_configurations.update(overrides)
    return context


def selected_actions(entities, context):
    for entity in entities:
        if entity.condition is not None and not entity.condition.evaluate(context):
            continue
        yield entity
        if isinstance(entity, GroupAction):
            yield from selected_actions(entity.get_sub_entities(), context)
        elif isinstance(entity, TimerAction):
            yield from selected_actions(entity.actions, context)


class ContainerLaunchContract(unittest.TestCase):
    def test_legacy_default_and_explicit_worker_budgets(self):
        description = load_description('bringup_launch.py')
        for bounded, budget, expected_package, expected_executable in (
                ('false', '2', 'rclcpp_components', 'component_container_isolated'),
                ('true', '2', 'robot_navigo', 'bounded_component_container'),
                ('true', '0', 'robot_navigo', 'bounded_component_container')):
            with self.subTest(bounded=bounded, budget=budget):
                context = default_context(description, use_composition='true',
                                          use_bounded_component_container=bounded,
                                          component_executor_threads=budget)
                nodes = [a for a in selected_actions(description.entities, context)
                         if isinstance(a, Node)]
                self.assertEqual(len(nodes), 1)
                node = nodes[0]
                self.assertEqual(resolved(context, node.node_package), expected_package)
                self.assertEqual(resolved(context, node.node_executable), expected_executable)
                arguments = [resolved(context, a) for a in node.cmd[1:4]]
                if bounded == 'true':
                    self.assertEqual(arguments, ['--executor-threads', budget, '--ros-args'])
                else:
                    self.assertEqual(arguments[0], '--use_multi_threaded_executor')

    def test_noncomposed_bringup_starts_no_container(self):
        description = load_description('bringup_launch.py')
        for bounded in ('true', 'false'):
            context = default_context(description, use_composition='false',
                                      use_bounded_component_container=bounded)
            self.assertFalse(any(isinstance(a, Node)
                                 for a in selected_actions(description.entities, context)))

    def test_top_level_defaults_and_forwarding(self):
        for filename in ('bringup_launch.py', 'lightning_nav_bringup.launch.py'):
            description = load_description(filename)
            context = default_context(description)
            self.assertEqual(context.launch_configurations['use_bounded_component_container'],
                             'false')
            self.assertEqual(context.launch_configurations['component_executor_threads'], '2')
        description = load_description('lightning_nav_bringup.launch.py')
        for budget in ('0', '2'):
            context = default_context(description, use_bounded_component_container='true',
                                      component_executor_threads=budget)
            includes = [a for a in selected_actions(description.entities, context)
                        if isinstance(a, IncludeLaunchDescription)]
            self.assertEqual(len(includes), 1)
            arguments = {resolved(context, key): resolved(context, value)
                         for key, value in includes[0].launch_arguments}
            self.assertEqual(arguments['use_bounded_component_container'], 'true')
            self.assertEqual(arguments['component_executor_threads'], budget)
            self.assertEqual(arguments['use_composition'], 'true')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='bounded_launch_test_') as log_dir:
        with patch.dict(os.environ, {'ROS_LOG_DIR': log_dir}):
            unittest.main()
