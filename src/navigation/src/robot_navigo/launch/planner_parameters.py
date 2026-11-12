"""Prepare a complete planner profile before any container or child costmap starts."""
import json
import math
import os
import tempfile

import yaml
from launch.substitutions import LaunchConfiguration


def _merge_parameters(base, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge_parameters(base[key], value)
        else:
            base[key] = value


def selected_parameter_file(context, package_dir, source=None):
    """Keep the legacy file untouched; prepare the opt-in ROS service boundary."""
    source = source or LaunchConfiguration('params_file').perform(context)
    if LaunchConfiguration('planner_backend').perform(context) == 'fork_navfn':
        return source

    with open(source, encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)
    overlay_path = LaunchConfiguration('smac_planner_params_file').perform(context)
    with open(overlay_path, encoding='utf-8') as stream:
        overlay = yaml.safe_load(stream)
    backend = LaunchConfiguration('planner_backend').perform(context)
    map_path = LaunchConfiguration('map').perform(context)
    if not map_path:
        raise RuntimeError('Smac requires an explicit map YAML to verify resolution')
    with open(map_path, encoding='utf-8') as stream:
        map_resolution = float(yaml.safe_load(stream)['resolution'])
    metadata = None
    if backend == 'smac_lattice':
        primitive_path = os.path.abspath(LaunchConfiguration('smac_lattice_file').perform(context))
        with open(primitive_path, encoding='utf-8') as stream:
            metadata = json.load(stream)['lattice_metadata']
        motion_models = {'omni': 'Omni', 'diff': 'DiffDrive'}
        if metadata.get('motion_model') not in motion_models:
            raise RuntimeError('smac_lattice supports only explicit Omni or DiffDrive primitives')
        selected_resolution = float(metadata['grid_resolution'])
        expected_model = motion_models[metadata['motion_model']]
        expected_plugin = 'nav2_smac_planner::SmacPlannerLattice'
    elif backend == 'smac_hybrid':
        selected_resolution = float(overlay['global_costmap']['global_costmap'][
            'ros__parameters']['resolution'])
        expected_model = 'DiffDrive'
        expected_plugin = 'nav2_smac_planner::SmacPlannerHybrid'
    else:
        raise RuntimeError('Unsupported planner backend: ' + backend)
    if not math.isclose(map_resolution, selected_resolution, rel_tol=0.0, abs_tol=1e-8):
        raise RuntimeError(
            'Map resolution %.6fm does not match %s profile %.6fm; '
            'StaticLayer may resize the costmap. Select a matching profile/map.'
            % (map_resolution, backend, selected_resolution))
    if parameters.get('bt_navigator', {}).get('ros__parameters', {}).get(
            'enable_epoch_contract', False):
        raise RuntimeError('Smac is not validated with the experimental epoch contract')

    # Remove Navfn-private keys before selecting the independent official plugin.
    parameters['planner_server']['ros__parameters'].pop('GridBased', None)
    _merge_parameters(parameters, overlay)
    planner = parameters['planner_server']['ros__parameters']['GridBased']
    if planner.get('plugin') != expected_plugin:
        raise RuntimeError(backend + ' requires plugin ' + expected_plugin)
    controller = parameters['controller_server']['ros__parameters']['FollowPath']
    if controller.get('motion_model') != expected_model:
        raise RuntimeError(
            '%s requires FollowPath.motion_model %s, got %s'
            % (backend, expected_model, controller.get('motion_model')))
    if expected_model == 'DiffDrive' and float(controller.get('vy_max', 0.0)) != 0.0:
        raise RuntimeError('DiffDrive planning requires FollowPath.vy_max 0.0')
    if backend == 'smac_lattice':
        planner['lattice_filepath'] = primitive_path
    else:
        if 'lattice_filepath' in planner:
            raise RuntimeError('Hybrid planner must not use lattice metadata or files')
        if planner.get('motion_model_for_search') != 'DUBIN' or float(
                planner.get('minimum_turning_radius', 0.0)) < .5:
            raise RuntimeError('Hybrid profile requires DUBIN and minimum_turning_radius >=.5m')
        if planner.get('downsample_costmap', True) or planner.get('smooth_path', True):
            raise RuntimeError('Hybrid profile requires full-resolution unsmoothed SE2 paths')
        if not math.isclose(selected_resolution, .01, rel_tol=0.0, abs_tol=1e-8):
            raise RuntimeError('The Hybrid field experiment requires a matching 1cm map/profile')
    parameters['global_costmap']['global_costmap']['ros__parameters'][
        'resolution'] = selected_resolution
    for name in ('local_costmap', 'global_costmap'):
        costmap = parameters[name][name]['ros__parameters']
        footprint = costmap.get('footprint', [])
        if isinstance(footprint, str):
            footprint = yaml.safe_load(footprint)
        if not footprint:
            raise RuntimeError('smac_lattice requires an explicit polygon footprint in ' + name)
        padding = float(costmap.get('footprint_padding', 0.01))
        radius = max(math.hypot(float(x) + math.copysign(padding, float(x)),
                                float(y) + math.copysign(padding, float(y)))
                     for x, y in footprint)
        if float(costmap['inflation_layer']['inflation_radius']) < radius:
            raise RuntimeError(
                '%s inflation radius must cover the padded footprint circumscribed radius %.4fm'
                % (name, radius))
    tree = ('navigate_to_pose_smac_hybrid_wait.xml' if backend == 'smac_hybrid'
            else 'navigate_to_pose_smac_lattice.xml')
    parameters['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = os.path.join(
        package_dir, 'behavior_trees', tree)
    with tempfile.NamedTemporaryFile(mode='w', prefix='robot_navigo_smac_',
                                     suffix='.yaml', delete=False, encoding='utf-8') as stream:
        yaml.safe_dump(parameters, stream, sort_keys=False)
        selected_file = stream.name
    return selected_file

