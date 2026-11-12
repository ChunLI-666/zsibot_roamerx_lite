#!/usr/bin/env python3
"""Resolve the planner service switch without starting nodes or initializing ROS."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction,
)
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.utilities import evaluate_parameters, to_parameters_list
import yaml


PACKAGE_DIR = Path(sys.argv.pop(1)).resolve()


def resolved(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def load(filename):
    spec = importlib.util.spec_from_file_location('smac_launch_under_test',
                                                 PACKAGE_DIR / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch.object(module, 'get_package_share_directory', return_value=str(PACKAGE_DIR)):
        description = module.generate_launch_description()
    return module, description


def context_for(description, **values):
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    context.launch_configurations.update(values)
    return context


def actions(entities, context):
    for entity in entities:
        if entity.condition is not None and not entity.condition.evaluate(context):
            continue
        yield entity
        if isinstance(entity, GroupAction):
            yield from actions(entity.get_sub_entities(), context)
        elif isinstance(entity, TimerAction):
            yield from actions(entity.actions, context)


class SmacPlannerLaunchContract(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='smac_launch_contract_')
        self.addCleanup(self.tmp.cleanup)
        self.map_file = Path(self.tmp.name) / 'map.yaml'
        self.map_file.write_text('resolution: 0.05\n')
        self.primitive = Path(self.tmp.name) / 'omni.json'
        self.primitive.write_text(json.dumps({
            'lattice_metadata': {'motion_model': 'omni', 'grid_resolution': .05}}))

    def configured(self, module, description, **values):
        selected = dict(planner_backend='smac_lattice', map=str(self.map_file),
                        smac_lattice_file=str(self.primitive))
        selected.update(values)
        context = context_for(description, **selected)
        for action in module._prepare_planner_parameters(context, str(PACKAGE_DIR)):
            action.execute(context)
        selected = Path(context.launch_configurations['planner_selected_params_file'])
        if selected != PACKAGE_DIR / 'params' / 'navigo_params.yaml':
            self.addCleanup(selected.unlink, missing_ok=True)
        return context, yaml.safe_load(selected.read_text())

    def test_legacy_default_uses_original_file_and_planner_in_both_layouts(self):
        module, description = load('navigation_launch.py')
        for composed in ('true', 'false'):
            context = context_for(description, use_composition=composed)
            self.assertEqual(context.launch_configurations['planner_backend'], 'fork_navfn')
            for action in module._prepare_planner_parameters(context, str(PACKAGE_DIR)):
                action.execute(context)
            self.assertEqual(context.launch_configurations['planner_selected_params_file'],
                             str(PACKAGE_DIR / 'params' / 'navigo_params.yaml'))
            selected = list(actions(description.entities, context))
            self.assertFalse(any(isinstance(a, Node) and
                                 resolved(context, a.node_package) == 'nav2_planner'
                                 for a in selected))
            if composed == 'true':
                loads = [a for a in selected if isinstance(a, LoadComposableNodes)]
                nodes = loads[0]._LoadComposableNodes__composable_node_descriptions
                planners = [n for n in nodes if resolved(context, n.package) == 'navigo_path_planner'
                            and (n.condition() is None or n.condition().evaluate(context))]
            else:
                planners = [a for a in selected if isinstance(a, Node) and
                            resolved(context, a.node_package) == 'navigo_path_planner']
            self.assertEqual(len(planners), 1)

    def test_smac_has_exactly_one_standalone_planner_and_no_fork_component(self):
        module, description = load('navigation_launch.py')
        for composed in ('true', 'false'):
            context, _ = self.configured(module, description, use_composition=composed)
            selected = list(actions(description.entities, context))
            packages = [resolved(context, a.node_package) for a in selected if isinstance(a, Node)]
            self.assertEqual(packages.count('nav2_planner'), 1)
            self.assertNotIn('navigo_path_planner', packages)
            if composed == 'true':
                loads = [a for a in selected if isinstance(a, LoadComposableNodes)]
                nodes = [n for n in loads[0]._LoadComposableNodes__composable_node_descriptions
                         if n.condition() is None or n.condition().evaluate(context)]
                self.assertNotIn('navigo_path_planner', [resolved(context, n.package) for n in nodes])
                manager = next(n for n in nodes if resolved(context, n.package) == 'nav2_lifecycle_manager')
                managed = evaluate_parameters(context, manager.parameters)[0]['node_names']
            else:
                manager = next(a for a in selected if isinstance(a, Node) and
                               resolved(context, a.node_package) == 'nav2_lifecycle_manager')
                managed = evaluate_parameters(context, manager._Node__parameters)[0]['node_names']
            self.assertEqual(managed.count('planner_server'), 1)

    def test_overlay_preserves_local_geometry_and_speed_limits(self):
        module, description = load('navigation_launch.py')
        original = yaml.safe_load((PACKAGE_DIR / 'params' / 'navigo_params.yaml').read_text())
        _, actual = self.configured(module, description)
        self.assertEqual(actual['local_costmap'], original['local_costmap'])
        self.assertEqual(actual['velocity_smoother'], original['velocity_smoother'])
        expected_controller = copy.deepcopy(original['controller_server'])
        expected_controller['ros__parameters']['FollowPath']['PathAlignCritic'][
            'use_path_orientations'] = True
        self.assertEqual(actual['controller_server'], expected_controller)
        planner = actual['planner_server']['ros__parameters']['GridBased']
        self.assertEqual(planner['plugin'], 'nav2_smac_planner::SmacPlannerLattice')
        self.assertFalse(planner['smooth_path'])
        self.assertFalse(planner['allow_reverse_expansion'])
        self.assertNotIn('use_astar', planner)
        global_map = actual['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual(global_map['footprint'], original['global_costmap'][
            'global_costmap']['ros__parameters']['footprint'])
        for layer, name in [('static_layer', 'StaticLayer'), ('obstacle_layer', 'ObstacleLayer'),
                            ('inflation_layer', 'InflationLayer')]:
            self.assertEqual(global_map[layer]['plugin'], 'nav2_costmap_2d::' + name)
        self.assertEqual(global_map['inflation_layer']['inflation_radius'], .4)
        self.assertEqual(global_map['footprint_padding'], .035)
        self.assertEqual(actual['local_costmap']['local_costmap']['ros__parameters'].get(
            'footprint_padding', .01), .01)
        self.assertEqual(actual['bt_navigator']['ros__parameters']['default_server_timeout'], 1000)
        tree = ET.parse(actual['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'])
        self.assertEqual(len(list(tree.iter('SmoothPath'))), 0)
        self.assertEqual(len(list(tree.iter('ComputePathToPose'))), 1)
        self.assertEqual(len(list(tree.iter('FollowPath'))), 1)

    def test_rejects_map_resolution_mismatch_and_unsupported_primitives(self):
        module, description = load('navigation_launch.py')
        self.map_file.write_text('resolution: 0.01\n')
        with self.assertRaisesRegex(RuntimeError, 'does not match smac_lattice profile'):
            self.configured(module, description)
        self.map_file.write_text('resolution: 0.05\n')
        self.primitive.write_text(json.dumps({
            'lattice_metadata': {'motion_model': 'ackermann', 'grid_resolution': .05}}))
        with self.assertRaisesRegex(RuntimeError, 'only explicit Omni or DiffDrive'):
            self.configured(module, description)

    def test_diffdrive_overlay_preserves_execution_deadbands_and_limits(self):
        module, description = load('navigation_launch.py')
        self.primitive.write_text(json.dumps({
            'lattice_metadata': {'motion_model': 'diff', 'grid_resolution': .05}}))
        original = yaml.safe_load((PACKAGE_DIR / 'params' / 'navigo_params.yaml').read_text())
        _, actual = self.configured(module, description, smac_planner_params_file=str(
            PACKAGE_DIR / 'params' / 'smac_diffdrive.yaml'))
        expected = copy.deepcopy(original['controller_server'])
        expected['ros__parameters']['FollowPath'].update(motion_model='DiffDrive', vy_max=0.0)
        expected['ros__parameters']['FollowPath']['PathAlignCritic'].update(
            use_path_orientations=True, offset_from_furthest=5)
        self.assertEqual(actual['controller_server'], expected)
        self.assertEqual(actual['velocity_smoother'], original['velocity_smoother'])
        for name in ('local_costmap', 'global_costmap'):
            costmap = actual[name][name]['ros__parameters']
            self.assertEqual(costmap['footprint_padding'], .035)
            self.assertEqual(costmap['footprint'], original[name][name]['ros__parameters']['footprint'])
            self.assertEqual(costmap['inflation_layer']['inflation_radius'], .4)
            self.assertEqual(costmap['inflation_layer']['cost_scaling_factor'], 3.)
        self.assertEqual(actual['planner_server']['ros__parameters']['GridBased']['plugin'],
                         'nav2_smac_planner::SmacPlannerLattice')
        self.assertEqual(actual['bt_navigator']['ros__parameters']['default_server_timeout'], 1000)

    def test_rejects_lattice_controller_mismatch(self):
        module, description = load('navigation_launch.py')
        self.primitive.write_text(json.dumps({
            'lattice_metadata': {'motion_model': 'diff', 'grid_resolution': .05}}))
        with self.assertRaisesRegex(RuntimeError, 'requires FollowPath.motion_model DiffDrive'):
            self.configured(module, description)
        self.primitive.write_text(json.dumps({
            'lattice_metadata': {'motion_model': 'omni', 'grid_resolution': .05}}))
        with self.assertRaisesRegex(RuntimeError, 'requires FollowPath.motion_model Omni'):
            self.configured(module, description, smac_planner_params_file=str(
                PACKAGE_DIR / 'params' / 'smac_diffdrive.yaml'))

    def test_rejects_footprint_wider_than_reviewed_inflation_radius(self):
        module, description = load('navigation_launch.py')
        source = yaml.safe_load((PACKAGE_DIR / 'params' / 'navigo_params.yaml').read_text())
        source['local_costmap']['local_costmap']['ros__parameters']['footprint'] = (
            '[[.5,.3],[.5,-.3],[-.5,-.3],[-.5,.3]]')
        invalid = Path(self.tmp.name) / 'wide_footprint.yaml'
        invalid.write_text(yaml.safe_dump(source))
        with self.assertRaisesRegex(RuntimeError, 'inflation radius must cover'):
            self.configured(module, description, params_file=str(invalid))

    def test_board_parameter_file_cannot_reintroduce_short_inflation(self):
        module, description = load('navigation_launch.py')
        source = yaml.safe_load((PACKAGE_DIR / 'params' / 'navigo_params.yaml').read_text())
        for name in ('local_costmap', 'global_costmap'):
            layer = source[name][name]['ros__parameters']['inflation_layer']
            layer['inflation_radius'] = .3
            layer['cost_scaling_factor'] = 8.0
        source['controller_server']['ros__parameters']['FollowPath'][
            'PreferForwardCritic']['cost_weight'] = 20.0
        board = Path(self.tmp.name) / 'board_params.yaml'
        board.write_text(yaml.safe_dump(source))
        before = board.read_bytes()
        _, actual = self.configured(module, description, params_file=str(board))
        for name in ('local_costmap', 'global_costmap'):
            layer = actual[name][name]['ros__parameters']['inflation_layer']
            self.assertEqual(layer['inflation_radius'], .4)
            self.assertEqual(layer['cost_scaling_factor'], 3.0)
        self.assertEqual(actual['controller_server']['ros__parameters']['FollowPath'][
            'PreferForwardCritic']['cost_weight'], 20.0)
        self.assertEqual(board.read_bytes(), before)

    def test_parameters_and_upstream_namespace_are_applied_once(self):
        module, description = load('navigation_launch.py')
        context, _ = self.configured(module, description, namespace='dog', ros_namespace='/dog')
        selected = list(actions(description.entities, context))
        upstream = next(a for a in selected if isinstance(a, Node) and
                        resolved(context, a.node_package) == 'nav2_planner')
        upstream._perform_substitutions(context)  # No execute() or ROS process.
        self.assertEqual(upstream.expanded_node_namespace, '/dog')
        parameters = evaluate_parameters(context, upstream._Node__parameters)
        effective = yaml.safe_load(Path(parameters[0]).read_text())
        self.assertIn('planner_server', effective['dog'])
        self.assertNotIn('dog', effective['dog'])
        self.assertFalse(any(isinstance(a, Node) and resolved(context, a.node_package) ==
                             'nav2_smoother' for a in selected))
        legacy_context = context_for(description, namespace='dog', ros_namespace='/dog')
        for action in module._prepare_planner_parameters(legacy_context, str(PACKAGE_DIR)):
            action.execute(legacy_context)
        legacy_selected = list(actions(description.entities, legacy_context))
        smoother = next(a for a in legacy_selected if isinstance(a, Node) and
                        resolved(legacy_context, a.node_package) == 'nav2_smoother')
        smoother._perform_substitutions(legacy_context)
        self.assertEqual(smoother.expanded_node_namespace, '/dog')

    def test_hybrid_uses_no_primitive_file_and_wait_only_policy(self):
        module, description = load('navigation_launch.py')
        self.map_file.write_text('resolution: 0.01\n')
        for composed in ('true', 'false'):
            context, actual = self.configured(module, description, planner_backend='smac_hybrid',
                use_composition=composed, smac_lattice_file='/missing/unused.json',
                smac_planner_params_file=str(PACKAGE_DIR / 'params' / 'smac_hybrid_field_experiment.yaml'))
            selected = list(actions(description.entities, context))
            packages = [resolved(context, a.node_package) for a in selected if isinstance(a, Node)]
            self.assertEqual(packages.count('nav2_planner'), 1)
            self.assertNotIn('navigo_path_planner', packages)
            if composed == 'true':
                load_action = next(a for a in selected if isinstance(a, LoadComposableNodes))
                nodes = load_action._LoadComposableNodes__composable_node_descriptions
                self.assertNotIn('navigo_path_planner', [resolved(context, n.package) for n in nodes
                    if n.condition() is None or n.condition().evaluate(context)])
            planner = actual['planner_server']['ros__parameters']['GridBased']
            self.assertEqual(planner['plugin'], 'nav2_smac_planner::SmacPlannerHybrid')
            self.assertNotIn('lattice_filepath', planner)
            self.assertNotIn('use_astar', planner)
            self.assertEqual(planner['minimum_turning_radius'], .5)
            self.assertEqual(planner['angle_quantization_bins'], 72)
            self.assertEqual(planner['lookup_table_size'], 1.)
            self.assertFalse(planner['downsample_costmap'])
            self.assertFalse(planner['smooth_path'])
            for name in ('local_costmap', 'global_costmap'):
                self.assertEqual(actual[name][name]['ros__parameters']['resolution'], .01)
            tree = ET.parse(actual['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'])
            for tag in ('Spin', 'BackUp', 'ClearEntireCostmap', 'SmoothPath'):
                self.assertEqual(list(tree.iter(tag)), [])
            self.assertEqual(len(list(tree.iter('Wait'))), 1)
            original = yaml.safe_load((PACKAGE_DIR / 'params' / 'navigo_params.yaml').read_text())
            self.assertEqual(actual['velocity_smoother'], original['velocity_smoother'])
            follow = actual['controller_server']['ros__parameters']['FollowPath']
            self.assertEqual(follow['motion_model'], 'DiffDrive')
            self.assertEqual(follow['vy_max'], 0.)
            self.assertEqual(follow['VelocityDeadbandCritic'], original['controller_server'][
                'ros__parameters']['FollowPath']['VelocityDeadbandCritic'])
            self.assertTrue(follow['CostCritic']['consider_footprint'])

    def test_hybrid_rejects_wrong_map_and_invalid_search_model(self):
        module, description = load('navigation_launch.py')
        arguments = dict(planner_backend='smac_hybrid', smac_lattice_file='/missing/unused.json',
            smac_planner_params_file=str(PACKAGE_DIR / 'params' / 'smac_hybrid_field_experiment.yaml'))
        with self.assertRaisesRegex(RuntimeError, 'does not match smac_hybrid profile'):
            self.configured(module, description, **arguments)
        self.map_file.write_text('resolution: 0.01\n')
        invalid = yaml.safe_load(Path(arguments['smac_planner_params_file']).read_text())
        invalid['planner_server']['ros__parameters']['GridBased']['motion_model_for_search'] = 'REEDS_SHEPP'
        bad = Path(self.tmp.name) / 'invalid_hybrid.yaml';bad.write_text(yaml.safe_dump(invalid))
        arguments['smac_planner_params_file'] = str(bad)
        with self.assertRaisesRegex(RuntimeError, 'requires DUBIN'):
            self.configured(module, description, **arguments)

    def test_smac_omits_idle_smoother_and_waypoint_switch_is_preserved(self):
        module, description = load('navigation_launch.py')
        for backend in ('fork_navfn', 'smac_lattice', 'smac_hybrid'):
            for composed in ('true', 'false'):
                for waypoint in ('true', 'false'):
                    # ParameterValue caches once per launched description; each
                    # matrix row represents an independent launch instance.
                    module, description = load('navigation_launch.py')
                    values = dict(planner_backend=backend, use_composition=composed,
                                  enable_waypoint_follower=waypoint)
                    if backend == 'fork_navfn':
                        context = context_for(description, **values)
                        for action in module._prepare_planner_parameters(context, str(PACKAGE_DIR)):
                            action.execute(context)
                    elif backend == 'smac_hybrid':
                        self.map_file.write_text('resolution: 0.01\n')
                        context, _ = self.configured(module, description, **values,
                            smac_lattice_file='/missing/unused.json', smac_planner_params_file=str(
                                PACKAGE_DIR / 'params' / 'smac_hybrid_field_experiment.yaml'))
                    else:
                        self.map_file.write_text('resolution: 0.05\n')
                        context, _ = self.configured(module, description, **values)
                    selected = list(actions(description.entities, context))
                    smoothers = [a for a in selected if isinstance(a, Node) and
                                 resolved(context, a.node_package) == 'nav2_smoother']
                    self.assertEqual(len(smoothers), int(backend == 'fork_navfn'))
                    if composed == 'true':
                        load_action = next(a for a in selected if isinstance(a, LoadComposableNodes))
                        components = [n for n in load_action._LoadComposableNodes__composable_node_descriptions
                                      if n.condition() is None or n.condition().evaluate(context)]
                        manager = next(n for n in components if resolved(context, n.package) == 'nav2_lifecycle_manager')
                        managed = evaluate_parameters(context, manager.parameters)[0]['node_names']
                        waypoint_count = sum(resolved(context, n.package) == 'navigo_waypoint_follower'
                                             for n in components)
                    else:
                        manager = next(a for a in selected if isinstance(a, Node) and
                                       resolved(context, a.node_package) == 'nav2_lifecycle_manager')
                        managed = evaluate_parameters(context, manager._Node__parameters)[0]['node_names']
                        waypoint_count = sum(isinstance(a, Node) and resolved(context, a.node_package) ==
                                             'navigo_waypoint_follower' for a in selected)
                    self.assertEqual('smoother_server' in managed, backend == 'fork_navfn')
                    self.assertEqual(managed.count('planner_server'), 1)
                    self.assertEqual('waypoint_follower' in managed, waypoint == 'true')
                    self.assertEqual(waypoint_count, int(waypoint == 'true'))

    def test_backend_arguments_forward_through_both_includes(self):
        for filename in ('lightning_nav_bringup.launch.py', 'bringup_launch.py'):
            _, description = load(filename)
            context = context_for(description, planner_backend='smac_lattice',
                                  smac_planner_params_file='/tmp/reviewed.yaml',
                                  smac_lattice_file='/tmp/omni.json')
            includes = [a for a in actions(description.entities, context)
                        if isinstance(a, IncludeLaunchDescription)]
            self.assertEqual(len(includes), 1)
            keys = {'planner_backend', 'smac_planner_params_file', 'smac_lattice_file'}
            arguments = {resolved(context, k): resolved(context, v)
                         for k, v in includes[0].launch_arguments if resolved(context, k) in keys}
            self.assertEqual(arguments['planner_backend'], 'smac_lattice')
            self.assertEqual(arguments['smac_planner_params_file'], '/tmp/reviewed.yaml')
            self.assertEqual(arguments['smac_lattice_file'], '/tmp/omni.json')

    def test_complete_profile_reaches_container_child_and_component_once(self):
        # Reproduce the real failure: base costmap parameters differ from the
        # selected profile, while the component's own parameters are correct.
        base = yaml.safe_load((PACKAGE_DIR / 'params' / 'navigo_params.yaml').read_text())
        for name in ('local_costmap', 'global_costmap'):
            costmap = base[name][name]['ros__parameters']
            costmap.update(resolution=.05, footprint_padding=.01)
            costmap['inflation_layer'].update(inflation_radius=.3, cost_scaling_factor=8.)
        source = Path(self.tmp.name) / 'field_base.yaml'
        source.write_text(yaml.safe_dump(base))
        for backend in ('fork_navfn', 'smac_lattice', 'smac_hybrid'):
            for composed in ('true', 'false'):
                for bounded in ('true', 'false'):
                    for namespace in ('', 'dog'):
                        with self.subTest(backend=backend, composed=composed,
                                          bounded=bounded, namespace=namespace):
                            module, description = load('bringup_launch.py')
                            self.map_file.write_text('resolution: %s\n' % (
                                '.01' if backend == 'smac_hybrid' else '.05'))
                            self.primitive.write_text(json.dumps({'lattice_metadata': {
                                'motion_model': 'diff', 'grid_resolution': .05}}))
                            profile = ('smac_hybrid_field_experiment.yaml' if backend == 'smac_hybrid'
                                       else 'smac_diffdrive.yaml')
                            context = context_for(description, params_file=str(source),
                                planner_backend=backend, map=str(self.map_file),
                                smac_planner_params_file=str(PACKAGE_DIR / 'params' / profile),
                                smac_lattice_file=str(self.primitive), use_composition=composed,
                                use_bounded_component_container=bounded, namespace=namespace,
                                use_namespace='true' if namespace else 'false')
                            prepare = next(a for a in description.entities if isinstance(a, OpaqueFunction))
                            group = next(a for a in description.entities if isinstance(a, GroupAction))
                            self.assertLess(description.entities.index(prepare), description.entities.index(group))
                            with patch.object(module._planner_parameters, 'selected_parameter_file',
                                              wraps=module._planner_parameters.selected_parameter_file) as selected:
                                for action in prepare.execute(context) or []:
                                    action.execute(context)
                                selected.assert_called_once()
                            prepared = Path(context.launch_configurations['planner_selected_params_file'])
                            if prepared != source:
                                self.addCleanup(prepared.unlink, missing_ok=True)
                            parent_actions = list(actions(description.entities, context))
                            containers = [a for a in parent_actions if isinstance(a, Node)]
                            self.assertEqual(len(containers), int(composed == 'true'))
                            expected_resolution = .01 if backend == 'smac_hybrid' else .05
                            expected_padding = .01 if backend == 'fork_navfn' else .035
                            expected_inflation = .3 if backend == 'fork_navfn' else .4
                            expected_scaling = 8. if backend == 'fork_navfn' else 3.
                            if containers:
                                container = containers[0]
                                self.assertEqual(resolved(context, container.node_executable),
                                    'bounded_component_container' if bounded == 'true'
                                    else 'component_container_isolated')
                                files = [p for p in evaluate_parameters(context, container._Node__parameters)
                                         if isinstance(p, Path)]
                                self.assertEqual(len(files), 1)
                                self.addCleanup(files[0].unlink, missing_ok=True)
                                # This is the process-global file seen by the
                                # child's freshly constructed NodeOptions.
                                local = {p.name: p.value for p in to_parameters_list(context,
                                    'local_costmap', '/' + namespace + '/local_costmap' if namespace
                                    else '/local_costmap', files)}
                                self.assertEqual(local['resolution'], expected_resolution)
                                self.assertEqual(local['footprint_padding'], expected_padding)
                                self.assertEqual(local['inflation_layer.inflation_radius'], expected_inflation)
                                self.assertEqual(local['inflation_layer.cost_scaling_factor'], expected_scaling)
                            include = next(a for a in parent_actions if isinstance(a, IncludeLaunchDescription))
                            child_arguments = {resolved(context, k): resolved(context, v)
                                               for k, v in include.launch_arguments}
                            self.assertEqual(child_arguments['_planner_parameters_prepared'], 'true')
                            child_module, child_description = load('navigation_launch.py')
                            child_context = context_for(child_description, **child_arguments)
                            with patch.object(child_module._planner_parameters, 'selected_parameter_file',
                                              side_effect=AssertionError('profile must not merge twice')):
                                for action in child_module._prepare_planner_parameters(
                                        child_context, str(PACKAGE_DIR)):
                                    action.execute(child_context)
                            child_source = Path(child_context.launch_configurations[
                                'planner_selected_params_file'])
                            self.assertEqual(yaml.safe_load(child_source.read_text()),
                                             yaml.safe_load(prepared.read_text()))
                            child_data = yaml.safe_load(child_source.read_text())
                            self.assertEqual(child_data['local_costmap']['local_costmap'][
                                'ros__parameters']['resolution'], expected_resolution)
                            self.assertEqual(child_data['controller_server']['ros__parameters'][
                                'FollowPath']['motion_model'], 'Omni' if backend == 'fork_navfn' else 'DiffDrive')

    def test_invalid_profile_stops_before_container_start(self):
        module, description = load('bringup_launch.py')
        context = context_for(description, planner_backend='smac_hybrid', map=str(self.map_file),
                              smac_planner_params_file=str(PACKAGE_DIR / 'params' /
                                                           'smac_hybrid_field_experiment.yaml'))
        prepare = next(a for a in description.entities if isinstance(a, OpaqueFunction))
        with self.assertRaisesRegex(RuntimeError, 'does not match smac_hybrid profile'):
            prepare.execute(context)
        self.assertNotIn('planner_selected_params_file', context.launch_configurations)


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='smac_launch_logs_') as log_dir:
        with patch.dict(os.environ, {'ROS_LOG_DIR': log_dir}):
            unittest.main()
