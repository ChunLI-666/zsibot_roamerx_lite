"""Pre-exec localization affinity tests; no ROS workloads or CPU benchmark."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest
import yaml

import common
import replay_support


def config_file(tmp_path, policy):
    path = tmp_path / 'lightning_config.yaml'
    path.write_text(yaml.safe_dump(policy))
    return path


@pytest.mark.parametrize('policy', [{}, {'performance': {}},
                                    {'performance': {'cpu_affinity': []}}])
def test_disabled_keeps_original_command_without_platform_or_tool_queries(monkeypatch, tmp_path, policy):
    def forbidden(*args):
        pytest.fail('Disabled affinity must not query platform affinity or taskset')
    monkeypatch.setattr(replay_support.os, 'sched_getaffinity', forbidden, raising=False)
    monkeypatch.setattr(replay_support.shutil, 'which', forbidden)
    command = ['ros2', 'run', 'lightning', 'run_loc_online', '--config', 'same.yaml']
    assert replay_support.localization_command(command, config_file(tmp_path, policy)) is command


@pytest.mark.parametrize('policy', [[], {'performance': []},
    {'performance': {'cpu_affinity': None}}, {'performance': {'cpu_affinity': '0,1'}},
    {'performance': {'cpu_affinity': [True]}}, {'performance': {'cpu_affinity': [0.0]}},
    {'performance': {'cpu_affinity': ['0']}}, {'performance': {'cpu_affinity': [0, 0]}}])
def test_malformed_policy_fails_closed(tmp_path, policy):
    with pytest.raises(ValueError):
        replay_support.localization_command(['native'], config_file(tmp_path, policy))


@pytest.mark.parametrize('cpus', [[-1], [3], [0, 3]])
def test_unavailable_cpu_fails_closed(monkeypatch, tmp_path, cpus):
    monkeypatch.setattr(replay_support.os, 'sched_getaffinity', lambda pid: {0, 2}, raising=False)
    with pytest.raises(ValueError, match='outside the caller allowed affinity'):
        replay_support.localization_command(['native'], config_file(tmp_path,
            {'performance': {'cpu_affinity': cpus}}))


def test_missing_taskset_fails_closed(monkeypatch, tmp_path):
    monkeypatch.setattr(replay_support.os, 'sched_getaffinity', lambda pid: {0, 2}, raising=False)
    monkeypatch.setattr(replay_support.shutil, 'which', lambda name: None)
    with pytest.raises(RuntimeError, match='requires taskset'):
        replay_support.localization_command(['native'], config_file(tmp_path,
            {'performance': {'cpu_affinity': [0, 2]}}))


def test_missing_affinity_api_fails_closed(monkeypatch, tmp_path):
    monkeypatch.delattr(replay_support.os, 'sched_getaffinity', raising=False)
    with pytest.raises(RuntimeError, match='requires Linux affinity support'):
        replay_support.localization_command(['native'], config_file(tmp_path,
            {'performance': {'cpu_affinity': [0]}}))


def test_affinity_query_error_is_not_an_unbudgeted_fallback(monkeypatch, tmp_path):
    def unavailable(pid):
        raise OSError('affinity unavailable')
    monkeypatch.setattr(replay_support.os, 'sched_getaffinity', unavailable, raising=False)
    with pytest.raises(OSError, match='affinity unavailable'):
        replay_support.localization_command(['native'], config_file(tmp_path,
            {'performance': {'cpu_affinity': [0]}}))


def test_effective_yaml_preserves_argument_order_and_original_command(monkeypatch, tmp_path):
    monkeypatch.setattr(replay_support.os, 'sched_getaffinity', lambda pid: {0, 2}, raising=False)
    monkeypatch.setattr(replay_support.shutil, 'which', lambda name: '/usr/bin/taskset')
    path = config_file(tmp_path, {'performance': {'cpu_affinity': [2, 0]}})
    command = ['native', '--config', str(path), '--argument', 'value with spaces']
    original = command.copy()
    actual = replay_support.localization_command(command, path)
    assert actual == ['/usr/bin/taskset', '--cpu-list', '2,0', *original]
    assert command == original
    # The effective file, rather than an earlier in-memory source copy, controls startup.
    path.write_text('performance: {cpu_affinity: []}\n')
    assert replay_support.localization_command(command, path) is command


@pytest.mark.skipif(not hasattr(os, 'sched_getaffinity') or shutil.which('taskset') is None,
                    reason='Linux taskset is required for actual child inheritance')
def test_real_child_and_new_thread_inherit_mask_without_changing_parent_or_other_child(tmp_path):
    original = sorted(os.sched_getaffinity(0))
    cpus = original[:2]
    assert cpus
    program = '''import json, os, threading
result = dict(main=sorted(os.sched_getaffinity(0)))
worker = threading.Thread(target=lambda: result.update(worker=sorted(os.sched_getaffinity(0))))
worker.start()
worker.join()
print(json.dumps(result))
'''
    command = [sys.executable, '-c', program]
    wrapped = replay_support.localization_command(command, config_file(tmp_path,
        {'performance': {'cpu_affinity': cpus}}))
    child = subprocess.run(wrapped, capture_output=True, text=True, check=True, timeout=10)
    assert json.loads(child.stdout) == dict(main=cpus, worker=cpus)
    assert sorted(os.sched_getaffinity(0)) == original
    other = subprocess.run(command, capture_output=True, text=True, check=True, timeout=10)
    assert json.loads(other.stdout) == dict(main=original, worker=original)


def test_runtime_manifest_records_binding_environment(monkeypatch, tmp_path):
    overlay = tmp_path / 'overlay'
    overlay.mkdir()
    packages = SimpleNamespace(get_package_prefix=lambda name: str(overlay))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    for relative in ('lib/robot_navigo/livox_scan_projector',
                     'lib/robot_navigo/nav_safety_gate.py',
                     'lib/navigo_path_controller/controller_server',
                     'lib/navigo_path_planner/planner_server',
                     'lib/navigo_bt_navigator/bt_navigator',
                     'lib/nav2_smoother/smoother_server',
                     'lib/libnav2_lifecycle_manager_core.so', 'lib/libbondcpp.so'):
        path = overlay / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('non-ELF test identity')
    binding = dict(OMP_PLACES='{0},{2}', OMP_PROC_BIND='close',
                   GOMP_CPU_AFFINITY='0 2', KMP_AFFINITY='disabled')
    for name, value in binding.items():
        monkeypatch.setenv(name, value)
    monkeypatch.setenv('LOCNAV_PROFILE', '0')
    args = SimpleNamespace(install_prefix=overlay, output=tmp_path, domain=123,
                           observer_off=True, no_composition=False)
    report = common.runtime_manifest(args, 'closed_loop')
    assert {name: report['environment'][name] for name in binding} == binding
