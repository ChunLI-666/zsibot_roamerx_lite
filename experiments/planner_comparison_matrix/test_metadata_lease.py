"""Offline targeted checks; no ROS imports, goals or runtime."""
import ast
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE/'goal_pilot_source'))

from waypoint_contract import validate_ownership, ownership_metadata_lease_deviation

SOURCE = HERE/'goal_pilot_source'
FROZEN = SOURCE/'pilot_scan_timing.py'
EVENTS = HERE/'test_fixtures/B8_ownership_lease.json'


class MetadataLeaseTests(unittest.TestCase):
    def setUp(self):
        self.event = json.loads(EVENTS.read_text())
        self.ready = deepcopy(self.event['latest_ready'])
        self.now = self.event['ownership_validation_unix_ns']
        self.ownership = self.ready['waypoint_ownership']
        self.uuid = self.ownership['follow_waypoints_active'][0]['uuid']

    def frozen_read_ready(self, ready):
        source = FROZEN.read_bytes()
        self.assertEqual(hashlib.sha256(source).hexdigest(), 'a63bce9490f55438e7af5d16025e4dd20d86293cec12f1560d403ecb629cbe31')
        function = next(node for node in ast.parse(source).body if isinstance(node, ast.FunctionDef) and node.name == 'read_ready')
        namespace = dict(time=SimpleNamespace(time_ns=lambda: self.now), json=json, math=math, hashlib=hashlib)
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(FROZEN), 'exec'), namespace)
        return namespace['read_ready'](SimpleNamespace(read_bytes=lambda: json.dumps(ready).encode()), ready['variant'])

    def test_actual_B8_ownership_and_READY_age_permitted(self):
        self.assertEqual(self.now-self.ownership['checked_unix_ns'], 300_267_712)
        self.assertEqual(self.now-self.ready['time_ns'], 300_043_823)
        validate_ownership(self.ownership, self.now, self.uuid)
        self.frozen_read_ready(self.ready)

    def test_metadata_500ms_boundary_and_future_50ms(self):
        for age_ns, allowed in [(500_000_000, True), (500_000_001, False), (-50_000_000, True), (-50_000_001, False)]:
            with self.subTest(age_ns=age_ns):
                ownership = dict(self.ownership, checked_unix_ns=self.now-age_ns)
                if allowed:
                    validate_ownership(ownership, self.now, self.uuid)
                else:
                    with self.assertRaises(ValueError):
                        validate_ownership(ownership, self.now, self.uuid)

    def test_foreign_uuid_and_exclusivity_unchanged(self):
        foreign = deepcopy(self.ownership)
        foreign['follow_waypoints_active'][0]['uuid'] = 'f'*32
        with self.assertRaisesRegex(ValueError, 'Foreign batch'):
            validate_ownership(foreign, self.now, self.uuid)
        for field in ['exclusive_nav_client', 'status_publishers_match']:
            with self.assertRaisesRegex(ValueError, 'exclusively owned'):
                validate_ownership(dict(self.ownership, **{field: False}), self.now, self.uuid)
        with self.assertRaisesRegex(ValueError, 'already active'):
            validate_ownership(self.ownership, self.now)

    def test_child_goal_precedence_unchanged(self):
        ownership = dict(self.ownership, navigate_to_pose_active=[dict(uuid='child', accepted_stamp_ns=self.now-50_000_001)])
        with self.assertRaisesRegex(ValueError, 'predates this batch'):
            validate_ownership(ownership, self.now, self.uuid, self.now)

    def test_READY_expiry_and_old_or_future_lease_still_rejected(self):
        for changes in [dict(valid_until_ns=self.now), dict(time_ns=self.now-500_000_001), dict(time_ns=self.now+50_000_001)]:
            with self.subTest(changes=changes), self.assertRaisesRegex(ValueError, 'launcher_lease'):
                self.frozen_read_ready(dict(self.ready, **changes))

    def test_deviation_deduplicated_by_original_check_ns(self):
        seen = set()
        deviation = ownership_metadata_lease_deviation(self.ready, self.now, seen)
        self.assertEqual(deviation['ownership_validation_unix_ns'], self.now)
        self.assertEqual(deviation['checked_unix_ns'], self.ownership['checked_unix_ns'])
        self.assertIs(deviation['latest_ready'], self.ready)
        self.assertFalse(deviation['physical_input_classification_affected'])
        self.assertTrue(deviation['within_effective_metadata_lease'])
        self.assertIsNone(ownership_metadata_lease_deviation(self.ready, self.now+1, seen))
        fresh = dict(self.ready, waypoint_ownership=dict(self.ownership, checked_unix_ns=self.now))
        self.assertIsNone(ownership_metadata_lease_deviation(fresh, self.now, seen))
        newer = dict(self.ready, waypoint_ownership=dict(self.ownership, checked_unix_ns=self.ownership['checked_unix_ns']+1))
        self.assertIsNotNone(ownership_metadata_lease_deviation(newer, self.now+1, seen))


if __name__ == '__main__':
    unittest.main(verbosity=2)
