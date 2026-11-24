from hashlib import sha256
from pathlib import Path
import subprocess
import sys

import pytest

from robot_debug_tools.replay_projection import project
from robot_debug_tools.replay_view import main


def test_gate_validity_is_separate_from_boolean_and_zero():
    row = project('/debug/observations', {
        'header': {'stamp': {'sec': 1789797464, 'nanosec': 54317790}},
        'component': 'safety_gate', 'goal_id_valid': False, 'goal_id': {'uuid': [0] * 16},
        'gate': [{'actual_branch': 'watchdog_cmd_timeout', 'gate_status': 5,
                  'input_zero_valid': False, 'input_is_zero': False, 'output_is_zero': True,
                  'cmd_elapsed_valid': True, 'cmd_elapsed_ms': 333.131408}],
    })
    assert row['title'] == 'watchdog_cmd_timeout'
    assert row['facts']['ROS source stamp'] == '1789797464.054317790'
    assert row['facts']['UUID'] == 'Unknown'
    facts = row['facts']['Native observations'][0]
    assert facts['Input is zero'] == 'Unknown'
    assert facts['Output is zero'] is True
    assert facts['Command elapsed (steady ms)'] == 333.131408
    assert facts['Localization heartbeat elapsed (steady ms)'] == 'Unknown'
    assert facts['Physical execution'] == 'Unknown'
    gate = {'input_zero_valid': True, 'input_is_zero': False, 'cmd_elapsed_valid': True, 'cmd_elapsed_ms': 0.0}
    facts = project('/debug/observations', {'gate': [gate]})['facts']['Native observations'][0]
    assert facts['Input is zero'] is False
    assert facts['Command elapsed (steady ms)'] == 0.0


def test_action_multiple_uuids_and_unknown_enum():
    data = {'status_list': [
        {'status': value, 'goal_info': {'goal_id': {'uuid': [identifier] * 16}}}
        for identifier, value in [(1, 2), (2, 6), (3, 99), (4, 0)]
    ]}
    row = project('/scope/follow_path/_action/status', data)['facts']
    assert row['Endpoint'] == '/scope/follow_path'
    assert [state['UUID'] for state in row['Goal states']] == [f'{index:02x}' * 16 for index in (1, 2, 3, 4)]
    assert [state['Native status'] for state in row['Goal states']] == [
        'EXECUTING (2)', 'ABORTED (6)', 'Unknown (99)', 'UNKNOWN (0)',
    ]
    assert row['State change source stamp'] == 'Unknown'


def test_observer_build_and_legacy_fields_are_not_inferred():
    row = project('/debug/view', {'reporter_build': {'known': True, 'artifact_id': 'observer'},
                                  'actions': [{'value': 2, 'source_binding_valid': False,
                                               'source_build': {'known': True, 'artifact_id': 'wrong'}}]})
    assert row['builds']['Reporter build']['artifact_id'] == 'observer'
    assert row['facts']['Action observations'][0]['Action source build'] == 'Unknown'
    loc = project('/lightning/debug', {'loc_status': 1, 'imu_stamp': 4.25})
    assert loc['title'] == 'NORMAL (1)'
    assert loc['facts']['Native reason'] == 'Unknown'
    assert loc['facts']['IMU source time (s)'] == 4.25
    assert loc['facts']['LIO source time (s)'] == 'Unknown'


def test_bt_reset_and_headerless_velocity():
    events = [{'node_name': 'Root', 'previous_status': previous, 'current_status': current}
              for previous, current in [('RUNNING', 'FAILURE'), ('FAILURE', 'IDLE')]]
    facts = project('/behavior_tree_log', {'event_log': events})['facts']
    assert [event['Current'] for event in facts['Transitions in recorded order']] == ['FAILURE', 'IDLE']
    assert facts['Action UUID'] == 'Unknown'
    assert project('/cmd_vel_safe', {})['title'] == 'Output validity unknown'
    zero = dict.fromkeys('xyz', 0.0)
    row = project('/cmd_vel_safe', {'linear': zero, 'angular': zero})
    assert row['title'] == 'Zero output' and row['facts']['Physical execution'] == 'Unknown'


def test_legacy_mcap_uses_schema_companion_without_duplicate_messages(tmp_path):
    pytest.importorskip('mcap')
    ros_writer = pytest.importorskip('mcap_ros2.writer').Writer
    from mcap.reader import make_reader
    from mcap.writer import Writer

    companion = tmp_path / 'schemas.mcap'
    topic = '/nav_safety_gate/gate_status'
    with companion.open('wb') as stream, ros_writer(stream) as writer:
        schema = writer.register_msgdef('std_msgs/msg/UInt8', 'uint8 data\n')
        writer.write_message(topic, schema, {'data': 5}, log_time=100, publish_time=90)
    with companion.open('rb') as stream:
        _, _, original = next(make_reader(stream).iter_messages())

    legacy = tmp_path / 'legacy.mcap'
    with legacy.open('wb') as stream:
        writer = Writer(stream)
        writer.start(profile='ros2')
        schema_id = writer.register_schema('std_msgs/msg/UInt8', '', b'')
        channel = writer.register_channel(topic, 'cdr', schema_id)
        for index in range(3):
            writer.add_message(channel, 100 + index, original.data, 90 + index, sequence=index)
        writer.finish()
    original_file = legacy.read_bytes()

    missing = tmp_path / 'missing.html'
    main([str(legacy), str(missing), '--limit', '1'])
    unknown = missing.read_text()
    assert 'Decode unavailable' in unknown and 'Missing or unsupported schema/encoding' in unknown
    assert 'limit reached' in unknown and unknown.count('class="event"') == 1

    restored = tmp_path / 'restored.html'
    script = Path(__file__).resolve().parents[1] / 'scripts/debug_replay_view'
    subprocess.run([sys.executable, str(script), str(legacy), str(restored),
                    '--schemas-from', str(companion), '--start', '0.000000001',
                    '--duration', '0.000000001'], check=True)
    report = restored.read_text()
    assert report.count('class="event"') == 1 and 'limit reached' not in report
    assert 'CMD_TIMEOUT (5)' in report and '&quot;data&quot;: 5' in report
    assert '<dt>Log time (ns)</dt><dd>101</dd>' in report
    assert '<dt>Publish time (ns)</dt><dd>91</dd>' in report
    assert sha256(original.data).hexdigest() in report
    assert legacy.read_bytes() == original_file
    with pytest.raises(SystemExit):
        main([str(legacy), str(legacy)])
    assert legacy.read_bytes() == original_file
