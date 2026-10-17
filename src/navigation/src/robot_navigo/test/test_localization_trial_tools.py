#!/usr/bin/env python3
"""Offline tool contracts; starts no localization or actuator node."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'


def load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


TRIAL = load('run_localization_online_trial')
PREPARE = load('prepare_localization_dropout_bag')


class TrialTest(unittest.TestCase):
    def test_node_exit_after_successful_player_is_failure_even_with_zero_exit(self):
        for code in (0, 1, -9):
            passed, reason = TRIAL.assess_trial(False, {'player': 0, 'node': code}, None, {})
            self.assertFalse(passed)
            self.assertIn('before runner-requested shutdown', reason)
        self.assertTrue(TRIAL.assess_trial(False, {'player': 0, 'node': None}, None, {})[0])

    def test_other_failures_cannot_be_reported_as_success(self):
        good_codes = {'player': 0, 'node': None}
        for args in ((True, good_codes, None, {}), (False, {'player': 2, 'node': None}, None, {}),
                     (False, good_codes, 'exception', {}), (False, {}, None, {}),
                     (False, good_codes, None, {'node': {'group_alive_after_cleanup': True}}),
                     (False, good_codes, None, {'node': {'signals': ['SIGINT', 'SIGKILL'], 'return_code': -9}}),
                     (False, good_codes, None, {'node': {'signals': ['SIGINT'], 'return_code': -11}}),
                     (False, good_codes, None, {'node': {'signals': [], 'return_code': 0}})):
            self.assertFalse(TRIAL.assess_trial(*args)[0])

    def test_exit_during_last_postplayback_spin_is_detected(self):
        node = mock.Mock()
        node.poll.side_effect = [None, 0]
        ticks = iter([0., 0., 5.])
        spin = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, 'post-playback'):
            TRIAL.observe_source_loss(node, spin, clock=lambda: next(ticks))
        spin.assert_called_once()

    def test_cleanup_signals_owned_group_even_when_leader_has_exited(self):
        process = mock.Mock(pid=12345, returncode=0)
        process.poll.return_value = 0
        with mock.patch.object(TRIAL, 'group_exists', side_effect=[True, False, False, False]), \
                mock.patch.object(TRIAL.os, 'killpg') as kill:
            result = TRIAL.stop_process_group(process, grace_periods=(.01, .01, .01))
        kill.assert_called_once_with(12345, TRIAL.signal.SIGINT)
        self.assertFalse(result['group_alive_after_cleanup'])

    def test_real_owned_session_cleanup_does_not_kill_unrelated_session(self):
        owned = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'], start_new_session=True,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'], start_new_session=True,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            result = TRIAL.stop_process_group(owned, grace_periods=(.3, .3, .3))
            self.assertIsNotNone(owned.poll())
            self.assertFalse(result['group_alive_after_cleanup'])
            self.assertIsNone(unrelated.poll())
        finally:
            TRIAL.stop_process_group(owned, grace_periods=(.1, .1, .1))
            TRIAL.stop_process_group(unrelated, grace_periods=(.1, .1, .1))

    def test_nonfinite_timeout_rejected_before_ros_import(self):
        for value in ('nan', 'inf', '-inf'):
            command = [sys.executable, str(SCRIPTS / 'run_localization_online_trial.py'),
                       '--binary-dir', '/missing', '--config', '/missing', '--map', '/missing',
                       '--bag', '/missing', '--output', '/tmp/unused_zsl1_trial_test_output', '--timeout=' + value]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Invalid isolated ROS domain or timeout', result.stderr)

    def test_end_to_end_node_exit_after_player_preserves_failed_evidence(self):
        try:
            import rclpy  # noqa: F401 -- integration dependency probe
        except ImportError:
            self.skipTest('rclpy is required for the real status-only child process')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binaries = root / 'binaries'
            binaries.mkdir()
            fake_node = binaries / 'run_loc_online'
            fake_node.write_text("""#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import UInt8
rclpy.init(args=[])
node = Node('trial_exit_regression_publisher')
pub = node.create_publisher(UInt8, '/lightning/loc_status', 10)
end = time.monotonic() + 2.
while time.monotonic() < end:
    pub.publish(UInt8(data=1))
    rclpy.spin_once(node, timeout_sec=.02)
node.destroy_node()
rclpy.shutdown()
""")
            fake_node.chmod(0o755)
            (binaries / 'liblightning.libs.so').write_bytes(b'unused regression placeholder')
            (binaries / 'libextra.so.1').write_bytes(b'extra shared object provenance')
            fake_player = binaries / 'ros2'
            fake_player.write_text('#!/usr/bin/env python3\nimport time\ntime.sleep(.1)\n')
            fake_player.chmod(0o755)
            (root / 'config.yaml').write_text('system: {}\n')
            (root / 'map').write_text('test map provenance')
            (root / 'bag').write_text('test player input provenance')
            env = dict(os.environ, PATH=str(binaries) + os.pathsep + os.environ['PATH'])
            result = subprocess.run([sys.executable, str(SCRIPTS / 'run_localization_online_trial.py'),
                '--binary-dir', str(binaries), '--config', str(root / 'config.yaml'),
                '--map', str(root / 'map'), '--bag', str(root / 'bag'),
                '--output', str(root / 'output'), '--domain', '187', '--timeout', '10'],
                env=env, capture_output=True, text=True, timeout=30)
            self.assertNotEqual(result.returncode, 0)
            evidence = json.loads((root / 'output' / 'trial.json').read_text())
            self.assertFalse(evidence['passed'])
            self.assertIn('post-playback', evidence['failure'])
            self.assertEqual(evidence['pre_cleanup_return_codes'], {'player': 0, 'node': 0})
            self.assertTrue(evidence['events'])
            self.assertTrue(evidence['input_provenance']['bag'])
            self.assertEqual(evidence['binary_sha256']['libextra.so.1'], TRIAL.digest_file(binaries / 'libextra.so.1'))
            self.assertFalse(any(item['group_alive_after_cleanup'] for item in evidence['cleanup'].values()))

    def test_input_hashes_cover_nested_map_and_file_content(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'tile').mkdir()
            (root / 'tile' / 'map.pcd').write_bytes(b'points')
            before = TRIAL.input_files(root)
            (root / 'tile' / 'map.pcd').write_bytes(b'changed')
            after = TRIAL.input_files(root)
            self.assertEqual(len(before), 1)
            self.assertNotEqual(before[0]['sha256'], after[0]['sha256'])


class BagTest(unittest.TestCase):
    def test_source_child_output_is_rejected_without_writing(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'new_output'
            result = subprocess.run([sys.executable, str(SCRIPTS / 'prepare_localization_dropout_bag.py'),
                                     '--input', directory, '--output', str(output)], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('outside the source bag', result.stderr)
            self.assertFalse(output.exists())

    def test_real_mcap_copy_preserves_payload_receipt_and_drop_boundary(self):
        try:
            import rosbag2_py
        except ImportError:
            self.skipTest('ROS rosbag2_py is required for actual MCAP readback')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, output = root / 'source', root / 'output'
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(source), storage_id='mcap'),
                        rosbag2_py.ConverterOptions('', ''))
            topics = ['/livox/lidar', '/livox/imu']
            for topic in topics:
                writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type='std_msgs/msg/String',
                                                             serialization_format='cdr'))
            original = []
            for second in range(4):
                for topic in topics:
                    # Deliberately opaque payload with timestamp-like bytes: the tool must never deserialize it.
                    payload = bytes([0, 1, 0, 0, second]) + topic.encode()
                    stamp = (second + 1) * 1_000_000_000
                    original.append((stamp, topic, payload))
                    writer.write(topic, payload, stamp)
            del writer
            source_hashes = TRIAL.input_files(source)
            result = subprocess.run([sys.executable, str(SCRIPTS / 'prepare_localization_dropout_bag.py'),
                                     '--input', str(source), '--output', str(output), '--duration', '3',
                                     '--drop-start', '1', '--drop-duration', '1'], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(source_hashes, TRIAL.input_files(source))
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='mcap'),
                        rosbag2_py.ConverterOptions('', ''))
            actual = []
            while reader.has_next():
                topic, payload, stamp = reader.read_next()
                actual.append((stamp, topic, payload))
            expected = [row for row in original if not (row[1] == '/livox/lidar' and row[0] == 2_000_000_000)]
            self.assertEqual(actual, expected)
            manifest = json.loads((output / 'dropout_manifest.json').read_text())
            self.assertTrue(manifest['readback_identical'])
            self.assertEqual(manifest['dropped_counts'], {'/livox/lidar': 1})
            self.assertEqual(manifest['kept_stream_sha256'], PREPARE.stream_digest(expected))


if __name__ == '__main__':
    unittest.main()
