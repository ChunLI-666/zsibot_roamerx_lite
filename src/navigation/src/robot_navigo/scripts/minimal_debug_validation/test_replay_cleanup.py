"""Full-source error cleanup tests. No ROS node, player or workload starts."""
from types import SimpleNamespace
from unittest.mock import Mock
import struct

import pytest
import yaml

import common
from replay import full_source_snapshot, shutdown_navigation_once, write_raw_playback_qos
from replay_support import CdrStreamDigest, coverage_report, player_command


def monitor(received=None, finalized=False):
    instance = SimpleNamespace(
        counts={'lifecycle_finalized': 1} if finalized else {},
        events=[dict(event='full_source_navigation_finalized',
                     data=dict(player_still_running=True))] if finalized else [],
        raw_sources={},
        raw_source_report=Mock(return_value=received or {}),
        lifecycle_shutdown=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=99)))
    return instance


def digest(count=2, checksum='same'):
    return dict(count=count, first_header_ns=10, last_header_ns=20,
                ordered_cdr_sha256=checksum)


def test_tail_timeout_after_finalization_never_repeats_shutdown():
    observer = monitor(finalized=True)
    # finish_original_source may fail after recording lifecycle_finalized but
    # before returning, leaving replay's local success flag False.
    assert not shutdown_navigation_once(observer, object(), navigation_finalized=False)
    observer.lifecycle_shutdown.assert_not_called()


def test_successful_full_source_flag_also_skips_shutdown():
    observer = monitor()
    assert not shutdown_navigation_once(observer, object(), navigation_finalized=True)
    observer.lifecycle_shutdown.assert_not_called()


def test_missing_finalization_still_shuts_down():
    observer, processes = monitor(), object()
    assert shutdown_navigation_once(observer, processes)
    observer.lifecycle_shutdown.assert_called_once_with(processes)


def test_shutdown_failure_is_not_suppressed():
    observer = monitor()
    observer.lifecycle_shutdown.side_effect = TimeoutError('shutdown response')
    with pytest.raises(TimeoutError, match='shutdown response'):
        shutdown_navigation_once(observer, object())


def test_count_and_hash_failure_are_kept_after_natural_eof():
    observer = monitor({'/imu': digest(1, 'different')}, finalized=True)
    report = full_source_snapshot(observer, SimpleNamespace(poll=lambda: 0),
        dict(start_ns=1, end_ns=100), {'/imu': digest()},
        validation_error='TimeoutError: drain complete final serialized raw samples')
    assert not report['valid']
    assert {row['field'] for row in report['errors']} == {'count', 'ordered_cdr_sha256'}
    assert report['received']['/imu']['count'] == 1
    assert report['natural_eof'] and report['navigation_finalized_before_eof']
    assert not report['drain_completed']
    assert report['validation_error'].startswith('TimeoutError:')


def test_late_coverage_match_does_not_clear_failed_drain():
    observer = monitor({'/imu': digest()}, finalized=True)
    report = full_source_snapshot(observer, SimpleNamespace(poll=lambda: 0),
        dict(start_ns=1, end_ns=100), {'/imu': digest()}, validation_error='drain timed out')
    assert report['valid']
    assert not report['drain_completed']
    assert report['validation_error'] == 'drain timed out'


def test_pre_player_failure_keeps_missing_raw_evidence():
    observer = monitor()
    del observer.raw_sources
    report = full_source_snapshot(observer, None, dict(start_ns=1, end_ns=100),
                                  {'/imu': digest()}, validation_error='occupied domain')
    assert not report['valid'] and report['received'] == {}
    assert report['player_returncode'] is None and not report['natural_eof']
    observer.raw_source_report.assert_not_called()


def test_player_error_is_not_mislabeled_natural_eof():
    report = full_source_snapshot(monitor({'/imu': digest()}),
        SimpleNamespace(poll=lambda: -6), dict(start_ns=1, end_ns=100), {'/imu': digest()})
    assert report['player_returncode'] == -6
    assert not report['natural_eof']


def test_optional_raw_stamp_ledger_does_not_change_strict_hash():
    plain, ledger = CdrStreamDigest(), CdrStreamDigest(keep_stamps=True)
    for stamp in (100, 200, 350):
        data = b'\x00\x01\x00\x00' + struct.pack('<iI', 12, stamp)
        plain.add(data)
        ledger.add(data)
    assert ledger.snapshot()['header_stamps_ns'] == [12_000_000_100, 12_000_000_200, 12_000_000_350]
    assert 'header_stamps_ns' not in plain.snapshot()
    assert coverage_report({'/imu': plain.snapshot()}, {'/imu': ledger.snapshot()})['valid']


def test_raw_player_and_recorder_use_identical_private_queues(tmp_path):
    sensors = ['/livox/lidar', '/livox/imu']
    path = write_raw_playback_qos(tmp_path, sensors)
    args = SimpleNamespace(output=tmp_path, bag='/original', rate=1., full_bag_mode=True,
                           raw_playback_qos=path, record_topics_extra=sensors)
    command = player_command(args, sensors)
    option = command.index('--qos-profile-overrides-path')
    assert command[option + 1] == str(tmp_path / 'raw_playback_qos.yaml')
    assert command[command.index('--rate') + 1] == '1.0'
    assert command[command.index('--topics') + 1:] == sensors
    player_qos = yaml.safe_load(path.read_text())
    expected = {topic: dict(history='keep_last', depth=2000, reliability='reliable',
                            durability='volatile') for topic in sensors}
    assert player_qos == expected
    processes = SimpleNamespace(start=Mock())
    common.start_recording(processes, args)
    recorder_qos = yaml.safe_load((tmp_path / 'record_qos.yaml').read_text())
    assert {topic: recorder_qos[topic] for topic in sensors} == expected
    assert recorder_qos['/tf_static']['durability'] == 'transient_local'
    assert '/odom/current_pose' not in recorder_qos


def test_closed_loop_recorder_does_not_add_raw_overrides(tmp_path):
    common.start_recording(SimpleNamespace(start=Mock()), SimpleNamespace(output=tmp_path))
    qos = yaml.safe_load((tmp_path / 'record_qos.yaml').read_text())
    assert '/livox/lidar' not in qos and '/livox/imu' not in qos
