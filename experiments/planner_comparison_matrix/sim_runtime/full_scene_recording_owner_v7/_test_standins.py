#!/usr/bin/python3
"""Harmless fixture processes only: no ROS, sockets, controller or SDK imports."""
import json
import os
from pathlib import Path
import signal
import struct
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import recording_deadline as deadline


def atomic(path, value):
    deadline.atomic_json(Path(path), value)


def record(bag):
    bag.mkdir()
    (bag/'fixture_0.mcap').write_bytes(b'\x89MCAP0\r\n')
    alive = [True]
    signal.signal(signal.SIGINT, lambda n, f: alive.__setitem__(0, False))
    (bag/'standin_identity.json').write_text(json.dumps(deadline.process_identity(os.getpid())))
    with (bag/'standin_frames.bin').open('wb') as frames:
        while alive[0]:
            frames.write(b'harmless standin frame\n')
            frames.flush()
            time.sleep(.02)
    def string(value):
        raw = value.encode()
        return struct.pack('<I', len(raw)) + raw
    def rec(opcode, data):
        return struct.pack('<BQ', opcode, len(data)) + data
    magic = b'\x89MCAP0\r\n'
    prefix = magic + rec(1, string('ros2') + string('standin')) + rec(15, struct.pack('<I', 0))
    summary = rec(3, struct.pack('<H', 1) + string('std_msgs/msg/String') + string('ros2msg') + string('string data\n'))
    summary += rec(4, struct.pack('<HH', 1, 1) + string('/fixture') + string('cdr') + struct.pack('<I', 0))
    summary += rec(11, struct.pack('<QHIIIIQQI', 1, 1, 1, 0, 0, 0, 1, 1, 10) + struct.pack('<HQ', 1, 1))
    raw = prefix + summary + rec(2, struct.pack('<QQI', len(prefix), 0, 0)) + magic
    (bag/'fixture_0.mcap').write_bytes(raw)
    metadata = dict(rosbag2_bagfile_information=dict(storage_identifier='mcap', relative_file_paths=['fixture_0.mcap'],
        message_count=1, topics_with_message_count=[dict(topic_metadata=dict(name='/fixture', type='std_msgs/msg/String',
        serialization_format='cdr'), message_count=1), dict(topic_metadata=dict(name='/discovered_no_messages',
        type='std_msgs/msg/String', serialization_format='cdr'), message_count=0)]))
    import yaml
    (bag/'metadata.yaml').write_text(yaml.safe_dump(metadata))


def controller(path, mode):
    config = json.loads(path.read_text())
    expected = config['controller']
    output = Path(expected['output_directory'])
    output.mkdir()
    time.sleep(.35)  # Parent binds the live exact identity before a result can exist.
    if mode == 'missing':
        return
    if mode == 'hang':
        time.sleep(7.)
        return
    release = json.loads(Path(expected['root_release_path']).read_text())
    import hashlib
    release_sha = hashlib.sha256(Path(expected['root_release_path']).read_bytes()).hexdigest()
    binding = dict(root_release_sha256=release_sha, route_sha256=expected['route_sha256'], oracle_sha256=expected['oracle_sha256'],
        sources_sha256=expected['sources_sha256'], output_directory=expected['output_directory'], case_id=expected['case_id'],
        recording_run_id=expected['recording_run_id'], recorder_ready_sha256=release['recorder_ready_sha256'])
    truth = dict(xyz=[0., 0., .33], q=[0., 0., 0., 1.], twist=[0.]*6, stamp_ns=time.time_ns(),
                 received_steady=time.monotonic(), frame_id='map', child_frame_id='BASE_LINK')
    result = dict(scope='MAPPING_ACQUISITION_ONLY', mode='controlled_static_sim_scene', run_id=expected['run_id'],
        owner_process_identity=deadline.process_identity(os.getpid()), runtime_binding=binding, formal_navigation_acceptance=False,
        classification='controlled_static_sim_raw_acquisition_completed' if mode == 'success' else 'HOLD_fixture_failure',
        stop_verified=True, relay_closed=True, relay_close_errors=[], relay_returncode=0,
        writer=dict(evidence_complete=True, dropped=0, worker_error=None, writer_alive=False),
        final_stop_observation=dict(truth=truth, command=[0., 0.], checked_unix_ns=time.time_ns(),
                                    checked_steady_ns=time.monotonic_ns(), continuous_seconds=.3))
    atomic(output/'result.json', result)


if __name__ == '__main__':
    if sys.argv[1] == 'recorder':
        record(Path(sys.argv[2]))
    else:
        controller(Path(sys.argv[2]), sys.argv[3])
