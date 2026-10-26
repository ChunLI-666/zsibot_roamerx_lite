"""Profile integration contract tests; no ROS nodes or hardware are started."""
import json
import os
from pathlib import Path
import signal
import sys
from types import SimpleNamespace

import pytest

import common


def capture_starts(monkeypatch, output):
    calls = []
    def start(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(pid=123456789)
    monkeypatch.setattr(common.subprocess, 'Popen', start)
    return common.Processes(output), calls


def close_streams(manager):
    for _, _, stream in manager.items:
        stream.close()


@pytest.mark.parametrize('value', [None, '0'])
def test_disabled_does_not_wrap_or_create_performance_output(monkeypatch, tmp_path, value):
    monkeypatch.delenv('LOCNAV_PROFILE', raising=False)
    if value is not None:
        monkeypatch.setenv('LOCNAV_PROFILE', value)
    manager, calls = capture_starts(monkeypatch, tmp_path)
    try:
        for name in ('localization', 'navigation', 'player', 'recorder', 'observer', 'plant'):
            command = ['workload', name]
            manager.start(name, command)
            assert calls[-1][0] == command
            assert not manager.stages[name]['profiling_enabled']
        assert not (tmp_path / 'performance').exists()
    finally:
        close_streams(manager)


def test_enabled_wraps_only_workloads_and_keeps_parent_preload_absent(monkeypatch, tmp_path):
    monkeypatch.setenv('LOCNAV_PROFILE', '1')
    monkeypatch.setenv('LOCNAV_PROFILE_BACKEND', 'resources')
    monkeypatch.delenv('LD_PRELOAD', raising=False)
    manager, calls = capture_starts(monkeypatch, tmp_path)
    try:
        for name in ('localization', 'navigation', 'player', 'recorder', 'observer', 'plant'):
            command = ['workload', name]
            manager.start(name, command)
            actual, kwargs = calls[-1]
            if name in ('localization', 'navigation'):
                assert actual[:2] == ['/usr/bin/python3', str(common.HERE.parent / 'performance/run.py')]
                assert actual[-3:] == ['--', *command]
                assert actual[actual.index('--output') + 1] == str(tmp_path / 'performance' / name)
                assert ('--signal-root-only' in actual) == (name == 'navigation')
                assert manager.stages[name]['profiling_enabled']
            else:
                assert actual == command
                assert not manager.stages[name]['profiling_enabled']
            assert kwargs['env'] is None
        assert 'LD_PRELOAD' not in os.environ
    finally:
        close_streams(manager)


def test_explicit_process_environment_controls_wrapping(monkeypatch, tmp_path):
    monkeypatch.setenv('LOCNAV_PROFILE', '1')
    manager, calls = capture_starts(monkeypatch, tmp_path)
    try:
        disabled = dict(os.environ, LOCNAV_PROFILE='0')
        manager.start('navigation', ['native'], env=disabled)
        assert calls[-1][0] == ['native']
        assert calls[-1][1]['env'] is disabled
    finally:
        close_streams(manager)


def test_profiling_does_not_disable_default_environment_guard(monkeypatch):
    monkeypatch.setenv('LOCNAV_PROFILE', '1')
    monkeypatch.setenv('LD_PRELOAD', '/unapproved/library.so')
    with pytest.raises(RuntimeError, match='temporary overrides'):
        common.require_default_environment()
    monkeypatch.delenv('LD_PRELOAD')
    monkeypatch.delenv('MAGICK_THREAD_LIMIT', raising=False)
    common.require_default_environment()
    monkeypatch.setenv('LOCNAV_PROFILE', 'yes')
    with pytest.raises(RuntimeError, match='0 or 1'):
        common.require_default_environment()


@pytest.mark.parametrize('enabled', [False, True])
def test_runtime_provenance_identifies_profiled_environment(monkeypatch, tmp_path, enabled):
    overlay = tmp_path / 'overlay'
    overlay.mkdir()
    library = tmp_path / 'libprofiler.so'
    library.write_bytes(b'private profiler identity')
    # Stub package identities, not ROS execution. Targets are tiny non-ELF files,
    # so runtime_manifest never invokes ldd in this test.
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
        path.write_text('test identity')
    monkeypatch.setenv('LOCNAV_PROFILE', '1' if enabled else '0')
    monkeypatch.setenv('LOCNAV_PROFILE_LIB', str(library))
    monkeypatch.setenv('LOCNAV_PROFILE_BACKEND', 'gperftools')
    monkeypatch.delenv('LD_PRELOAD', raising=False)
    monkeypatch.delenv('MAGICK_THREAD_LIMIT', raising=False)
    args = SimpleNamespace(install_prefix=overlay, output=tmp_path, domain=123,
                           observer_off=True, no_composition=False)
    result = common.runtime_manifest(args, 'closed_loop')
    assert result['profiling_enabled'] is enabled
    assert result['default_environment'] is not enabled
    assert result['temporary_environment_overrides_absent']
    if enabled:
        assert result['profiling_library'] == dict(path=str(library), sha256=common.sha256(library))
        assert any('/performance/run.py' in row['path'] for row in result['files'])
    else:
        assert result['profiling_library'] is None
        assert not any('/performance/run.py' in row['path'] for row in result['files'])


def test_cleanup_accepts_deliberate_sigint_but_rejects_hidden_failure_or_escalation():
    summary = dict(status='interrupted', command_returncode=0, received_signal=2,
                   remaining_owned_pids=[], cleanup_signals=[dict(signal='SIGINT')])
    assert common.profile_cleanup_failures(summary, True) == []
    for code in (-signal.SIGINT, 128 + signal.SIGINT):
        assert common.profile_cleanup_failures(dict(summary, command_returncode=code), True) == []
        assert common.profile_cleanup_failures(dict(summary, command_returncode=code), False)
    for changed in (dict(command_returncode=-11), dict(remaining_owned_pids=[42]),
                    dict(cleanup_signals=[dict(signal='SIGTERM')]), dict(status='runner_error')):
        assert common.profile_cleanup_failures(dict(summary, **changed), True)


def test_profile_wrapper_cleanup_audits_actual_native_log(monkeypatch, tmp_path):
    manager = common.Processes(tmp_path)
    stream = (tmp_path / 'navigation.log').open('w')
    process = SimpleNamespace(pid=123456789, returncode=130, poll=lambda: 130)
    manager.items.append(('navigation', process, stream))
    manager.stage('navigation', 'RUNNING', profiling_enabled=True)
    profile = tmp_path / 'performance/navigation'
    profile.mkdir(parents=True)
    native_failure = '[ERROR] [native-1]: process has died [pid 42, exit code -11]'
    (profile / 'command.log').write_text(native_failure + '\n')
    (profile / 'summary.json').write_text(json.dumps(dict(status='interrupted', command_returncode=0,
        received_signal=2, remaining_owned_pids=[], cleanup_signals=[])))
    def absent_group(pid, number):
        raise ProcessLookupError
    monkeypatch.setattr(common.os, 'killpg', absent_group)
    try:
        manager.stop('navigation')
        assert manager.stages['navigation']['child_failures'] == [native_failure]
        assert not manager.cleanup_valid()
    finally:
        stream.close()

@pytest.mark.parametrize('workers', [None, 0, 2])
def test_bounded_executor_budget_forwarded_without_navigation_parameter_changes(workers, tmp_path):
    calls = []
    manager = SimpleNamespace(start=lambda name, command: calls.append((name, command)))
    args = SimpleNamespace(observer_off=True, map=tmp_path / 'map.yaml', output=tmp_path,
                           no_composition=False, component_executor_threads=workers)
    common.start_stack(manager, args, tmp_path / 'params.yaml', scan=True)
    name, command = calls[0]
    assert name == 'navigation'
    assert 'enable_laserscan:=true' in command
    assert 'params_file:=' + str(tmp_path / 'params.yaml') in command
    assert ('use_bounded_component_container:=true' in command) == (workers is not None)
    if workers is not None:
        assert 'component_executor_threads:=' + str(workers) in command


def daily_profile_fixture(monkeypatch, tmp_path, native=False):
    monkeypatch.delenv('AMENT_PREFIX_PATH', raising=False)
    prefix = tmp_path / 'installed'
    entry = prefix / 'lib/lightning/run_loc_online'
    entry.parent.mkdir(parents=True)
    entry.write_bytes(b'\x7fELFfixture' if native else b'#!/usr/bin/python3\n# daily entry fixture\n')
    entry.chmod(0o755)
    policy = prefix / 'share/lightning/daily_startup.json'
    policy.parent.mkdir(parents=True)
    (policy.parent / 'daily_entry.sha256').write_text(common.sha256(entry) + '\n')
    policy.write_text(json.dumps(dict(schema='lightning.daily_startup.policy.v1',
                                      entry_sha256=common.sha256(entry))))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages',
                        SimpleNamespace(get_package_prefix=lambda name: str(prefix)))
    return entry, policy


def test_daily_shim_profiling_keeps_ros2_and_python_clean(monkeypatch, tmp_path):
    entry, _ = daily_profile_fixture(monkeypatch, tmp_path)
    command = ['ros2', 'run', 'lightning', 'run_loc_online', '--config', '/fixture/base.yaml',
               '--ros-args', '-p', 'use_sim_time:=true']
    env = dict(LOCNAV_PROFILE='1', LOCNAV_PROFILE_BACKEND='gperftools')
    manager, calls = capture_starts(monkeypatch, tmp_path)
    try:
        manager.start('localization', command, env=env)
        actual, kwargs = calls[-1]
        assert actual[-len(command):] == command
        assert '--prefix' not in actual and '--native-allow-executable' not in actual
        assert '--backend' in actual and 'resources' in actual  # Outer tree monitor only.
        patch = kwargs['env']
        assert patch['LIGHTNING_STARTUP_PROFILE_OUTPUT_ROOT'] == str(
            tmp_path / 'performance/localization/native')
        assert not any(patch.get(k) for k in ('LD_PRELOAD', 'CPUPROFILE', 'HEAPPROFILE'))
        assert 'LIGHTNING_STARTUP_PROFILE_OUTPUT_ROOT' not in env
        detail = manager.stages['localization']['native_CPU_profiling']
        assert detail['daily_entry']['entry_sha256'] == common.sha256(entry)
    finally:
        close_streams(manager)


@pytest.mark.parametrize('change', ['missing-identity', 'wrong-schema', 'wrong-sha', 'changed-entry'])
def test_unbound_daily_script_cannot_receive_profiler(monkeypatch, tmp_path, change):
    entry, policy = daily_profile_fixture(monkeypatch, tmp_path)
    if change == 'missing-identity':
        (policy.parent / 'daily_entry.sha256').unlink()
    elif change == 'changed-entry':
        entry.write_text('# changed script\n')
    else:
        value = json.loads(policy.read_text())
        value['schema' if change == 'wrong-schema' else 'entry_sha256'] = 'invalid'
        policy.write_text(json.dumps(value))
    with pytest.raises(RuntimeError, match='policy|differs|identity'):
        common.native_profile_command('localization',
            ['ros2', 'run', 'lightning', 'run_loc_online', '--config', '/fixture/base.yaml'],
            tmp_path, dict(LOCNAV_PROFILE='1'))


def test_direct_elf_profile_prefix_remains_native_only(monkeypatch, tmp_path):
    entry, _ = daily_profile_fixture(monkeypatch, tmp_path, native=True)
    command, detail = common.native_profile_command('localization',
        ['ros2', 'run', 'lightning', 'run_loc_online', '--config', '/fixture/base.yaml'],
        tmp_path, dict(LOCNAV_PROFILE='1'))
    assert '--prefix' in command
    assert str(entry.resolve()) in command[command.index('--prefix') + 1]
    assert 'environment_patch' not in detail


def test_daily_profile_output_override_is_rejected(monkeypatch, tmp_path):
    daily_profile_fixture(monkeypatch, tmp_path)
    with pytest.raises(RuntimeError, match='Conflicting daily'):
        common.native_profile_command('localization',
            ['ros2', 'run', 'lightning', 'run_loc_online', '--config', '/fixture/base.yaml'],
            tmp_path, dict(LOCNAV_PROFILE='1', LIGHTNING_STARTUP_PROFILE_OUTPUT_ROOT='/other'))


def test_parent_child_entry_resolution_mismatch_is_rejected(monkeypatch, tmp_path):
    daily_profile_fixture(monkeypatch, tmp_path)
    monkeypatch.setenv('AMENT_PREFIX_PATH', '/parent/selected/prefix')
    with pytest.raises(RuntimeError, match='matching parent/child'):
        common.native_profile_command('localization',
            ['ros2', 'run', 'lightning', 'run_loc_online', '--config', '/fixture/base.yaml'],
            tmp_path, dict(LOCNAV_PROFILE='1', AMENT_PREFIX_PATH='/child/other/prefix'))


def test_known_shim_without_ci6_policy_preserves_native_profile_boundary(monkeypatch, tmp_path):
    entry, policy = daily_profile_fixture(monkeypatch, tmp_path)
    policy.unlink()
    prefix = entry.parents[2]
    legacy = prefix / 'libexec/lightning/run_loc_online'
    legacy.parent.mkdir(parents=True)
    legacy.write_bytes(b'\x7fELFlegacy fixture')
    legacy.chmod(0o755)
    profiler = prefix / 'lib/lightning/profile_workload'
    profiler.write_text('#!/usr/bin/python3\n# fixture\n')
    profiler.chmod(0o755)
    original = ['ros2', 'run', 'lightning', 'run_loc_online', '--config', '/fixture/base.yaml']
    command, detail = common.native_profile_command('localization', original, tmp_path, dict(LOCNAV_PROFILE='1'))
    assert command == original and '--prefix' not in command
    assert detail['daily_entry']['selected_mode'] == 'legacy_no_policy'
    assert detail['daily_entry']['legacy_native'] == str(legacy)
    assert detail['environment_patch'] == dict(
        LIGHTNING_STARTUP_PROFILE_OUTPUT_ROOT=str(tmp_path / 'performance/localization/native'))


@pytest.mark.parametrize('state', ['valid', 'broken', 'wrong-sha'])
def test_symlink_install_entry_identity_is_checked_by_actual_content(monkeypatch, tmp_path, state):
    entry, policy = daily_profile_fixture(monkeypatch, tmp_path)
    identity = policy.parent / 'daily_entry.sha256'
    identity.unlink()
    generated = tmp_path / 'generated-entry.sha256'
    if state != 'broken':
        generated.write_text((common.sha256(entry) if state == 'valid' else '0' * 64) + '\n')
    identity.symlink_to(generated)
    call = lambda: common.native_profile_command('localization',
        ['ros2', 'run', 'lightning', 'run_loc_online', '--config', '/fixture/base.yaml'],
        tmp_path, dict(LOCNAV_PROFILE='1'))
    if state == 'valid':
        command, detail = call()
        assert '--prefix' not in command
        assert detail['daily_entry']['entry_sha256'] == common.sha256(entry)
    else:
        with pytest.raises(RuntimeError, match='identity'):
            call()
