"""Safety and independence tests for the validation harness; no ROS nodes start."""
import math
from pathlib import Path
import signal
import struct
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

import common
from replay_support import (CdrStreamDigest, coverage_report, full_schedule,
                            player_command, require_natural_eof, validate_playback_options,
                            finish_original_source)
from world import Scene, integrate, error_to_goal, write_open_map


def test_plant_exact_body_twist_and_independent_geometry():
    pose = integrate([0., 0., 0.], [1., 0., 1.], math.pi / 2)
    assert np.allclose(pose, [1., 1., math.pi / 2])
    # Subdividing an interval cannot change the commanded path.
    split = [0., 0., 0.]
    for _ in range(100):
        split = integrate(split, [.3, -.1, .2], .01)
    assert np.allclose(split, integrate([0., 0., 0.], [.3, -.1, .2], 1.))
    assert error_to_goal([0., 0., -math.pi + .1], [0., 0., math.pi - .1])['yaw_rad'] == pytest.approx(.2)


def test_sensor_scan_comes_from_map_not_navigation(tmp_path):
    scene = Scene(write_open_map(tmp_path / 'map'))
    scan = np.asarray(scene.scan([4., 4., 0.]))
    assert len(scan) == 360
    assert 3.8 < scan[0] < 4.1  # physical perimeter, not synthetic clear-to-goal
    assert 3.8 < scan[180] < 4.1
    assert scan[45] > scan[0]


def test_preparation_preserves_algorithm_profile(tmp_path):
    args = SimpleNamespace(config=None, closed_loop=False,
        nav_params=common.NAV_SOURCE / 'params/navigo_params.yaml',
        map=tmp_path / 'map.yaml', output=tmp_path)
    before = yaml.safe_load(args.nav_params.read_text())
    _, after = common.prepare_navigation(args)
    def without_transport(value):
        if isinstance(value, dict):
            return {key: without_transport(child) for key, child in value.items()
                    if key not in ('use_sim_time', 'enable_epoch_contract', 'yaml_filename')}
        return value
    assert without_transport(before) == {key: value for key, value in without_transport(after).items()
                                        if key != 'nav_safety_gate'}
    assert without_transport(after['nav_safety_gate']) == {'ros__parameters': {}}
    assert after['controller_server']['ros__parameters']['general_goal_checker']['plugin'].endswith('SimpleGoalChecker')
    for node in ('controller_server', 'bt_navigator', 'planner_server', 'velocity_smoother'):
        assert after[node]['ros__parameters']['enable_epoch_contract'] is False


def test_reject_epoch_bt_instead_of_silently_changing_it(tmp_path):
    source = yaml.safe_load((common.NAV_SOURCE / 'params/navigo_params.yaml').read_text())
    source['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = '/tmp/with_epoch.xml'
    config = tmp_path / 'input.yaml'
    config.write_text(yaml.safe_dump(source))
    args = SimpleNamespace(config=config, closed_loop=True, nav_params=config,
                           map=tmp_path / 'map.yaml', output=tmp_path)
    with pytest.raises(ValueError, match='Epoch BT'):
        common.prepare_navigation(args)


def test_observations_off_changes_only_diagnostic_enable(tmp_path):
    args = SimpleNamespace(config=None, closed_loop=False,
        nav_params=common.NAV_SOURCE / 'params/navigo_params.yaml',
        map=tmp_path / 'map.yaml', output=tmp_path, observations_off=False)
    _, enabled = common.prepare_navigation(args)
    args.observations_off = True
    _, disabled = common.prepare_navigation(args)
    for node in ('map_server', 'planner_server', 'controller_server',
                 'behavior_server', 'bt_navigator', 'velocity_smoother', 'nav_safety_gate'):
        assert disabled[node]['ros__parameters'].pop('enable_debug_observations') is False
    assert disabled == enabled


def test_cadence_keeps_clock_regression_visible():
    result = common.cadence([1_000_000_000, 1_100_000_000, 1_090_000_000, 1_200_000_000])
    assert result['nonpositive_intervals'] == 1
    assert result['max_sec'] == pytest.approx(.11)
    assert result['intervals'] == 2


def test_recorder_never_subscribes_expensive_controller_debug():
    assert '/controller_server/debug' not in common.RECORD_TOPICS
    assert '/debug/observations' in common.RECORD_TOPICS
    assert '/debug/view' in common.RECORD_TOPICS
    assert '/debug/build_info' in common.RECORD_TOPICS
    assert '/follow_path/_action/status' in common.RECORD_TOPICS
    assert '/tf_static' in common.RECORD_TOPICS
    assert '/goal_pose' in common.RECORD_TOPICS
    assert '/trajectories' in common.RECORD_TOPICS
    assert '/transformed_global_plan' in common.RECORD_TOPICS


def test_full_source_forbids_caps_offsets_and_reserves_teardown_before_natural_eof():
    args = SimpleNamespace(bag=Path('/original/bag'), rate=1., start_offset=0.,
        duration=None, observe=15., full_bag_mode=True)
    validate_playback_options(args)
    command = player_command(args, ['/livox/lidar', '/livox/imu'])
    assert '--start-paused' in command
    assert '--playback-duration' not in command
    assert '--start-offset' not in command
    assert '--loop' not in command
    for name, value in [('start_offset', 1.), ('duration', 500.)]:
        changed = SimpleNamespace(**vars(args))
        setattr(changed, name, value)
        with pytest.raises(ValueError, match='forbids'):
            validate_playback_options(changed)
    start = 1_789_797_180_499_343_995
    metadata = dict(starting_time=dict(nanoseconds_since_epoch=start),
                    duration=dict(nanoseconds=512_480_544_568))
    schedule = full_schedule(metadata, 270., 20., 5.)
    assert schedule['duration_seconds'] == pytest.approx(512.480544568)
    assert schedule['goal_ns'] == start + 270_000_000_000
    assert schedule['teardown_ns'] < schedule['end_ns']
    assert schedule['end_ns']-schedule['teardown_ns'] == 20_000_000_000
    with pytest.raises(ValueError, match='precede'):
        full_schedule(metadata, 500., 20., 5.)
    with pytest.raises(ValueError, match='Reserve'):
        full_schedule(metadata, 270., 5., 5.)


def test_full_source_evidence_rejects_missing_tail_reorder_and_early_player_exit():
    def cdr(seconds, value):
        return b'\x00\x01\x00\x00' + struct.pack('<iII', seconds, 123, value)
    samples = [cdr(100, 1), cdr(101, 2), cdr(102, 3)]
    expected = CdrStreamDigest()
    for data in samples:
        expected.add(data)
    complete = CdrStreamDigest()
    for data in samples:
        complete.add(data)
    assert coverage_report({'lidar': expected.snapshot()}, {'lidar': complete.snapshot()})['valid']
    for selected in (samples[:-1], [samples[0], samples[2], samples[1]]):
        altered = CdrStreamDigest()
        for data in selected:
            altered.add(data)
        assert not coverage_report({'lidar': expected.snapshot()}, {'lidar': altered.snapshot()})['valid']
    assert require_natural_eof(SimpleNamespace(poll=lambda: 0)) == 0
    for code in (None, -2, 1):
        with pytest.raises(RuntimeError):
            require_natural_eof(SimpleNamespace(poll=lambda: code))


def test_full_source_domain_accepts_long_latched_header_but_rejects_wall_time():
    start = 1_789_797_180_499_343_995
    end = start + 512_480_544_568
    window = (start-2_000_000_000, end+2_000_000_000)
    assert not common.timestamp_domain_valid(end, start)
    assert common.timestamp_domain_valid(end, start, source_window_ns=window)
    assert not common.timestamp_domain_valid(1_792_227_400_000_000_000, start,
                                             source_window_ns=window)
    assert not common.timestamp_domain_valid(end, end+3_000_000_000,
                                             source_window_ns=window)


def test_navigation_finishes_before_source_eof_and_complete_tail_is_still_consumed():
    order = []
    player = SimpleNamespace(returncode=None, poll=lambda: player.returncode)
    digest = CdrStreamDigest()
    digest.add(b'\x00\x01\x00\x00' + struct.pack('<iI', 100, 0))
    expected = {'lidar': digest.snapshot()}
    class Monitor:
        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000))
        def lifecycle_shutdown(self, processes):
            assert player.poll() is None
            order.append('navigation_finalized_with_live_source')
        def record(self, name, data):
            assert data['player_still_running']
        def raw_source_report(self):
            return expected
        def spin_until(self, predicate, timeout, label, processes, allow):
            assert allow == ('player',)
            if 'natural EOF' in label:
                assert player.poll() is None
                order.append('complete_original_tail_then_natural_eof')
                player.returncode = 0
            assert predicate()
        def spin_for(self, seconds, processes, allow):
            assert player.poll() == 0
            order.append('drain_after_eof')
    report = finish_original_source(Monitor(), object(), player,
        dict(start_ns=100_000_000_000, end_ns=612_480_544_568), expected, 1.)
    assert report['valid'] and report['natural_eof']
    assert order == ['navigation_finalized_with_live_source',
        'complete_original_tail_then_natural_eof', 'drain_after_eof']


def test_acceptance_rejects_temporary_library_or_thread_workaround(monkeypatch):
    monkeypatch.delenv('LD_PRELOAD', raising=False)
    monkeypatch.delenv('MAGICK_THREAD_LIMIT', raising=False)
    common.require_default_environment()
    for variable, value in [('LD_PRELOAD', '/tmp/manager.so'), ('MAGICK_THREAD_LIMIT', '1')]:
        monkeypatch.setenv(variable, value)
        with pytest.raises(RuntimeError, match='temporary overrides'):
            common.require_default_environment()
        monkeypatch.delenv(variable)


def test_topic_goal_adoption_excludes_history_and_rejects_ambiguous_new_uuid():
    old, new, other = (tuple([i] * 16) for i in (1, 2, 3))
    def row(stamp, identities):
        return dict(event='/navigate_to_pose/_action/status', monotonic_ns=stamp,
            data=dict(status_list=[dict(goal_info=dict(goal_id=dict(uuid=list(identity))), status=2)
                                   for identity in identities]))
    events = [row(1, [old]), row(5, [old, new])]
    assert common.new_navigation_goal_uuid(events, {old}, 3) == new
    assert common.new_navigation_goal_uuid(events, {old, new}, 3) is None
    events.append(row(6, [other]))
    with pytest.raises(RuntimeError, match='Ambiguous'):
        common.new_navigation_goal_uuid(events, {old}, 3)


def test_recording_preserves_source_stamp_and_rejects_wall_sim_domain_mismatch():
    source = SimpleNamespace(sec=1_789_797_600, nanosec=123)
    message = SimpleNamespace(header=SimpleNamespace(stamp=source))
    values = common.message_stamp_ns(message)
    assert values == [1_789_797_600_000_000_123]
    assert common.timestamp_domain_valid(values[0] + 100_000_000, values[0])
    assert not common.timestamp_domain_valid(1_792_227_400_000_000_000, values[0])
    assert source.sec == 1_789_797_600 and source.nanosec == 123
    zero = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)))
    assert common.message_stamp_ns(zero) == []


def test_recorder_selects_sim_time_and_keeps_static_scene_durable(tmp_path):
    commands = []
    class Capture:
        def start(self, name, command):
            commands.append((name, command))
    common.start_recording(Capture(), SimpleNamespace(output=tmp_path))
    assert '--use-sim-time' in commands[0][1]
    qos = yaml.safe_load((tmp_path / 'record_qos.yaml').read_text())
    for topic in ('/tf_static', '/map', '/debug/build_info'):
        assert qos[topic]['durability'] == 'transient_local'


def test_actual_stack_launcher_has_no_actuator_adapter(tmp_path):
    commands = []
    class Capture:
        def start(self, name, command):
            commands.append((name, command))
    args = SimpleNamespace(observer_off=False, no_composition=False,
                           map=tmp_path / 'map.yaml', output=tmp_path)
    common.start_stack(Capture(), args, tmp_path / 'params.yaml', scan=True)
    assert {name for name, _ in commands} == {'observer', 'navigation'}
    navigation = next(command for name, command in commands if name == 'navigation')
    assert 'lightning_nav_bringup.launch.py' in navigation
    assert 'enable_laserscan:=true' in navigation
    assert 'log_base_dir:=' + str(tmp_path / 'nav_logs') in navigation
    assert all(not any('lcm' in value.lower() or 'udp' in value.lower() or 'sdk' in value.lower()
                       for value in command) for _, command in commands)


def test_recovery_seed_is_preserved_without_writing_original(tmp_path):
    source = tmp_path / 'source'
    (source / 'data').mkdir(parents=True)
    seed = source / 'data/recover_pose.txt'
    seed.write_text('1 2 3 0 0 0 1\n')
    output = tmp_path / 'output'
    output.mkdir()
    config = {}
    record = common.isolate_recovery_seed(config, output, source)
    assert record['existed']
    runtime = Path(config['lidar_loc']['recover_pose_path'])
    assert runtime.read_bytes() == seed.read_bytes()
    runtime.write_text('runtime can change\n')
    assert seed.read_text() == '1 2 3 0 0 0 1\n'


def test_cleanup_does_not_signal_external_process(tmp_path):
    external = subprocess.Popen(['sleep', '30'], start_new_session=True)
    manager = common.Processes(tmp_path)
    owned = manager.start('owned', ['sleep', '30'])
    try:
        manager.close()
        assert owned.poll() is not None
        assert external.poll() is None
        assert manager.stages['owned']['group_alive'] is False
        assert manager.stages['owned']['signals'] == ['SIGINT']
        assert manager.stages['owned']['pid'] == owned.pid
        assert manager.stages['owned']['command'] == ['sleep', '30']
    finally:
        external.send_signal(signal.SIGTERM)
        external.wait(timeout=3)


def test_launch_success_does_not_hide_native_child_crash(tmp_path):
    log = tmp_path / 'navigation.log'
    log.write_text('[INFO] lifecycle finalized\n[ERROR] component: process has died [exit code -6]\n')
    assert common.child_failures(log) == ['[ERROR] component: process has died [exit code -6]']
    manager = common.Processes(tmp_path)
    manager.stages['navigation'] = dict(group_alive=False, signals=['SIGINT'],
                                     child_failures=common.child_failures(log))
    assert not manager.cleanup_valid()


def test_only_expected_non_ros_log_tail_sigint_is_allowed(tmp_path):
    path = tmp_path / 'navigation.log'
    signal = '[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)\n'
    tail = '[ERROR] [log_aggregator-1]: process has died [pid 42, exit code -2, cmd bash]\n'
    native = '[ERROR] [component_container_isolated-2]: process has died [pid 43, exit code -6, cmd native]\n'
    path.write_text(signal + tail + native)
    result = common.child_exit_audit(path, deliberate_sigint=True)
    assert result['normal_log_process_signal_exits'] == [tail.strip()]
    assert result['failures'] == [native.strip()]
    assert len(common.child_exit_audit(path, deliberate_sigint=False)['failures']) == 2
    path.write_text(tail + signal)
    assert common.child_exit_audit(path, deliberate_sigint=True)['failures'] == [tail.strip()]
    path.write_text(signal + tail.replace('exit code -2', 'exit code -11'))
    assert common.child_exit_audit(path, deliberate_sigint=True)['failures']


def test_lifecycle_core_must_be_exact_overlay_library_even_if_old_bytes_match(tmp_path):
    overlay = tmp_path / 'overlay'
    expected = overlay / 'lib/libnav2_lifecycle_manager_core.so'
    expected.parent.mkdir(parents=True)
    expected.write_bytes(b'fixed local library')
    other = tmp_path / 'opt/libnav2_lifecycle_manager_core.so'
    other.parent.mkdir()
    other.write_bytes(expected.read_bytes())
    assert common.lifecycle_core_identity(expected, overlay)['valid']
    assert not common.lifecycle_core_identity(other, overlay)['valid']


def test_manager_bond_must_use_selected_overlay_library(tmp_path):
    overlay = tmp_path / 'overlay'
    fixed = overlay / 'lib/libbondcpp.so'
    fixed.parent.mkdir(parents=True)
    fixed.write_bytes(b'fixed destructor library')
    old = tmp_path / 'opt/libbondcpp.so'
    old.parent.mkdir()
    old.write_bytes(b'original library')
    assert common.selected_library_identity(fixed, overlay, 'libbondcpp.so')['valid']
    assert not common.selected_library_identity(old, overlay, 'libbondcpp.so')['valid']


def test_mcap_schema_audit_reads_both_summary_records(tmp_path):
    def string(value):
        return struct.pack('<I', len(value)) + value
    def record(opcode, payload):
        return struct.pack('<BQ', opcode, len(payload)) + payload
    magic = b'\x89MCAP0\r\n'
    schema = struct.pack('<H', 7) + string(b'robot_debug_msgs/msg/StatusObservation') + string(b'ros2msg') + string(b'bool evaluated\n')
    channel = struct.pack('<HH', 3, 7) + string(b'/debug/observations') + string(b'cdr') + struct.pack('<I', 0)
    path = tmp_path / 'summary.mcap'
    path.write_bytes(magic + record(3, schema) + record(4, channel) + magic)
    schemas, channels = common.mcap_summary_schemas(path)
    assert schemas[7]['encoding'] == 'ros2msg'
    assert schemas[7]['data_size'] > 0
    assert channels[3]['schema_id'] == 7


def test_mcap_schema_audit_rejects_truncation(tmp_path):
    path = tmp_path / 'bad.mcap'
    path.write_bytes(b'\x89MCAP0\r\n' + struct.pack('<BQ', 3, 10000) + b'\x89MCAP0\r\n')
    with pytest.raises(ValueError, match='Truncated'):
        common.mcap_summary_schemas(path)


def test_complete_mcap_writer_preserves_every_topic_cdr_and_timestamp(tmp_path):
    from schema_recovery import Message, write_companion, messages
    from collections import Counter
    rows = [Message('/goal_pose', b'original pose CDR', 123, 121, 8),
            Message('/follow_path/_action/status', b'original action CDR', 126, 125, 0),
            Message('/goal_pose', b'another original pose', 127, 124, 9)]
    types = {'/goal_pose': 'geometry_msgs/msg/PoseStamped',
             '/follow_path/_action/status': 'action_msgs/msg/GoalStatusArray'}
    class Definitions:
        def full(self, name):
            return '# original schema for ' + name + '\nuint8 value\n'
    path = tmp_path / 'viewing.mcap'
    write_companion(path, rows, types, Definitions())
    assert Counter(messages(path, types)) == Counter(rows)
    schemas, channels = common.mcap_summary_schemas(path, include_data=True)
    assert len(channels) == 2
    assert all(s['encoding'] == 'ros2msg' and s['data'].startswith('# original schema')
               for s in schemas.values())
