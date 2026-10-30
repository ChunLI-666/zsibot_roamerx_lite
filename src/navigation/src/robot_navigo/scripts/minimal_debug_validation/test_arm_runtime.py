"""Lean-runtime runner selection without ROS, navigation or child processes."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

import closed_loop
import common


def parsed_args(tmp_path, arm_runtime=False):
    values = ['--install-prefix', str(tmp_path / 'overlay'), '--output', str(tmp_path),
              '--map', str(tmp_path / 'map.yaml'), '--domain', '123']
    if arm_runtime:
        values.append('--arm-runtime')
    return common.arguments('test').parse_args(values)


@pytest.mark.parametrize('arm_runtime', [False, True])
def test_launch_only_adds_explicit_runtime_selection(tmp_path, arm_runtime):
    calls = []
    processes = SimpleNamespace(start=lambda name, command: calls.append((name, command)))
    args = parsed_args(tmp_path, arm_runtime)
    assert args.arm_runtime is arm_runtime
    common.start_stack(processes, args, tmp_path / 'params.yaml', scan=True)
    assert [name for name, _ in calls] == ['observer', 'navigation']
    command = calls[-1][1]
    selection = ['enable_waypoint_follower:=false', 'enable_livox_converter:=false']
    assert all((value in command) is arm_runtime for value in selection)
    assert 'use_sim_time:=true' in command and 'enable_laserscan:=true' in command
    assert not any(value.startswith(('enable_odom_bridge:=', 'cmd_vel_topic:=')) for value in command)


@pytest.mark.parametrize('arm_runtime', [False, True])
def test_manifest_matches_selected_package_set(monkeypatch, tmp_path, arm_runtime):
    args = parsed_args(tmp_path, arm_runtime)
    args.observer_off = True
    args.install_prefix.mkdir()
    resolved = []
    def package_prefix(name):
        resolved.append(name)
        if arm_runtime and name == 'navigo_waypoint_follower':
            pytest.fail('Lean runtime must not resolve an uninstalled waypoint package')
        return str(args.install_prefix)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages',
                        SimpleNamespace(get_package_prefix=package_prefix))
    monkeypatch.setenv('LOCNAV_PROFILE', '0')
    for relative in ('lib/robot_navigo/livox_scan_projector',
                     'lib/robot_navigo/nav_safety_gate.py',
                     'lib/navigo_path_controller/controller_server',
                     'lib/navigo_path_planner/planner_server',
                     'lib/navigo_bt_navigator/bt_navigator',
                     'lib/nav2_smoother/smoother_server',
                     'lib/libnav2_lifecycle_manager_core.so', 'lib/libbondcpp.so'):
        path = args.install_prefix / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('non-ELF test identity')
    report = common.runtime_manifest(args, 'closed_loop')
    assert report['arm_runtime'] is arm_runtime
    assert ('navigo_waypoint_follower' in resolved) is not arm_runtime
    assert ('exact7' if arm_runtime else 'exact8') in report['preplay_consumers_scope']


@pytest.mark.parametrize('arm_runtime', [False, True])
def test_all_scenarios_forward_selected_profile(monkeypatch, tmp_path, arm_runtime):
    argv = ['closed_loop.py', '--install-prefix', str(tmp_path / 'overlay'),
            '--output', str(tmp_path), '--map', str(tmp_path / 'map.yaml'),
            '--domain', '123', '--scenario', 'all']
    if arm_runtime:
        argv.append('--arm-runtime')
    monkeypatch.setattr(sys, 'argv', argv)
    monkeypatch.setattr(closed_loop, 'bootstrap', lambda args: None)
    commands = []
    def run(command, **kwargs):
        commands.append(command)
        output = Path(command[command.index('--output') + 1])
        output.mkdir()
        (output / 'result.json').write_text('{"success": true}')
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(closed_loop.subprocess, 'run', run)
    assert closed_loop.main() == 0
    assert len(commands) == 4
    assert all(('--arm-runtime' in command) is arm_runtime for command in commands)
    report = json.loads((tmp_path / 'result.json').read_text())
    assert report['success'] and report['arm_runtime'] is arm_runtime
