#!/usr/bin/env python3
"""Targeted receiver-only checks; no ROS imports, contexts, Nodes, clocks or goals."""
import ast
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
import time
from types import MethodType, SimpleNamespace
import unittest

HERE = Path(__file__).resolve().parent
CODE = HERE/'goal_pilot_source'
FROZEN = CODE/'pilot_scan_timing.py'
spec = importlib.util.spec_from_file_location('candidate_waypoint_contract', CODE/'waypoint_contract.py')
contract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(contract)
assert hashlib.sha256(FROZEN.read_bytes()).hexdigest() == 'a63bce9490f55438e7af5d16025e4dd20d86293cec12f1560d403ecb629cbe31'


def extracted(path, name, namespace, class_name=None):
    root = ast.parse(path.read_text())
    nodes = root.body if class_name is None else next(n for n in root.body if isinstance(n, ast.ClassDef) and n.name == class_name).body
    node = next(n for n in nodes if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


def ready_frame(issued=1_800_000_000_000_000_000):
    return dict(ready=True, time_ns=issued, valid_until_ns=issued+500_000_000,
                variant='official', action_server='/navigate_to_pose',
                goal_tolerances=dict(xy=.15, yaw=.08, stateful=False),
                params_sha256='a'*64, build_receipt_sha256='b'*64)


class MemoryStream(io.BytesIO):
    def __init__(self, raw, counter):
        super().__init__(raw)
        self.counter = counter

    def read(self, *args):
        self.counter['reads'] += 1
        return super().read(*args)


class MemoryPath:
    def __init__(self, raw):
        self.raw = raw
        self.counter = dict(opens=0, reads=0)

    def open(self, mode):
        assert mode == 'rb'
        self.counter['opens'] += 1
        return MemoryStream(self.raw, self.counter)


class ReceiverCaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)
        self.path = self.directory/'ready.json'
        self.now = ready_frame()['time_ns']+100_000_000
        # Dependency injection into a separately compiled original function AST.
        # The time module and production frozen source remain untouched.
        self.frozen = SimpleNamespace(read_ready=extracted(FROZEN, 'read_ready',
            dict(json=json, math=math, hashlib=hashlib,
                 time=SimpleNamespace(time_ns=lambda: self.now))))
        self.path.write_text(json.dumps(ready_frame()))

    def tearDown(self):
        self.temp.cleanup()

    def test_exact_single_read_and_optional_stat_failure_do_not_change_result(self):
        raw=self.path.read_bytes(); path=MemoryPath(raw); capture=contract.ReadyReadCapture(path)
        decoded, _, sha=self.frozen.read_ready(capture, 'official')
        self.assertEqual(decoded, ready_frame()); self.assertEqual(sha, hashlib.sha256(raw).hexdigest())
        self.assertEqual(capture.raw, raw); self.assertEqual(capture.reads, 1)
        self.assertEqual(path.counter, dict(opens=1, reads=1))
        self.assertEqual(len(capture.capture_errors), 2)

    def test_fd_stat_and_bytes_stay_on_same_inode_across_atomic_replace(self):
        before=self.path.stat(); raw=self.path.read_bytes(); real=self.path
        replacement=self.directory/'replacement';replacement.write_bytes(b'{"different":true}')
        class RenameAfterOpen:
            def open(_, mode):
                stream=real.open(mode);replacement.replace(real);return stream
        capture=contract.ReadyReadCapture(RenameAfterOpen())
        decoded, _, sha=self.frozen.read_ready(capture, 'official')
        self.assertEqual(decoded, ready_frame());self.assertEqual(capture.raw, raw)
        self.assertEqual(sha, hashlib.sha256(raw).hexdigest())
        self.assertEqual(capture.fd_identity['before_read']['ino'], before.st_ino)
        self.assertEqual(capture.fd_identity['after_read']['ino'], before.st_ino)
        self.assertNotEqual(real.stat().st_ino, before.st_ino)

    def test_real_clock_call_bounds_do_not_claim_frozen_internal_now(self):
        capture=contract.ReadyReadCapture(self.path)
        bounds=dict(call_begin_unix_ns=time.time_ns(), call_begin_steady_ns=time.monotonic_ns())
        self.frozen.read_ready(capture, 'official')
        bounds.update(call_end_unix_ns=time.time_ns(), call_end_steady_ns=time.monotonic_ns())
        evidence=capture.failure(ValueError('test_evidence_only'), bounds)['metadata']
        self.assertIsNone(evidence['frozen_internal_now_ns'])
        for suffix in ['unix_ns','steady_ns']:
            self.assertLessEqual(bounds['call_begin_'+suffix], capture.timing['read_begin_'+suffix])
            self.assertLessEqual(capture.timing['read_begin_'+suffix], capture.timing['read_end_'+suffix])
            self.assertLessEqual(capture.timing['read_end_'+suffix], bounds['call_end_'+suffix])

    def test_original_frozen_eight_lease_boundaries_with_capture(self):
        issued=ready_frame()['time_ns']
        cases=[('fresh',issued+100_000_000,True), ('last_ns_before_expiry',issued+499_999_999,True),
               ('expiry_exact',issued+500_000_000,False), ('too_old',issued+500_000_001,False),
               ('future_49ms',issued-49_000_000,True), ('future_50ms_exact',issued-50_000_000,True),
               ('future_over_50ms',issued-50_000_001,False), ('far_future',issued-100_000_000,False)]
        for label, now, accepted in cases:
            with self.subTest(label=label):
                self.now=now
                if accepted:self.frozen.read_ready(contract.ReadyReadCapture(self.path), 'official')
                else:
                    with self.assertRaisesRegex(ValueError, 'launcher_lease_missing_expired_or_future'):
                        self.frozen.read_ready(contract.ReadyReadCapture(self.path), 'official')

    def test_invalid_json_keeps_exact_raw_and_original_exception(self):
        raw=b'{invalid-JSON\x00';path=MemoryPath(raw);capture=contract.ReadyReadCapture(path)
        with self.assertRaises(json.JSONDecodeError) as failure:
            self.frozen.read_ready(capture, 'official')
        evidence=capture.failure(failure.exception, {})
        self.assertEqual(evidence['raw_bytes'], raw)
        self.assertEqual(evidence['metadata']['sha256'], hashlib.sha256(raw).hexdigest())
        self.assertEqual(path.counter, dict(opens=1,reads=1))

    def pilot(self):
        log=[];snapshot=dict(last={'pose':dict(raw_pose_values_finite=True),
                                   '/cmd_vel_safe':dict(vx=0.,vy=0.,wz=0.)})
        namespace=dict(time=time, math=math, frozen=self.frozen, ReadyReadCapture=contract.ReadyReadCapture,
                       finite_velocity=contract.finite_velocity, strict_stopped=contract.strict_stopped,
                       ownership_metadata_lease_deviation=contract.ownership_metadata_lease_deviation,
                       validate_ownership=contract.validate_ownership, waypoint_binding=contract.waypoint_binding)
        execution=extracted(CODE/'waypoint_batch.py','execution_ready', namespace, 'WaypointPilot')
        pilot=SimpleNamespace(check_interruption=lambda:None,
            input_ready=lambda **kw:(True,[],snapshot), args=SimpleNamespace(scan_aging_confirm_sec=0.,execute=True,ready=self.path,variant='official'),
            runtime_binding=None,native_binding=None,owned_identity=None,goal_sent_ns=None,
            ownership_metadata_reported_check_ns=set(), first_ready_read_failure=None,
            latest_ready=dict(last_passing_cache=True), latest_ready_sha='c'*64,
            scan_timing_record_check=lambda *args:None, emit=lambda kind,**fields:log.append(dict(kind=kind,**fields)),
            event_pub=SimpleNamespace(get_subscription_count=lambda:1))
        pilot.execution_ready=MethodType(execution,pilot)
        return pilot,log

    def test_first_failure_exact_bytes_retained_once_last_passing_cache_separate(self):
        self.now=ready_frame()['time_ns']+600_000_000
        pilot,events=self.pilot();first_raw=self.path.read_bytes()
        valid,issues,_=pilot.execution_ready();self.assertFalse(valid)
        first=pilot.first_ready_read_failure
        self.assertEqual(first['raw_bytes'],first_raw)
        self.assertEqual(first['metadata']['sha256'],hashlib.sha256(first_raw).hexdigest())
        self.assertEqual(pilot.latest_ready,dict(last_passing_cache=True))
        self.assertFalse(events[-1]['latest_ready_is_failing_read'])
        self.assertTrue(events[-1]['first_failed_read_capture'])
        self.path.write_bytes(b'new invalid bytes')
        pilot.execution_ready()
        self.assertIs(pilot.first_ready_read_failure,first)
        self.assertEqual(first['raw_bytes'],first_raw)
        self.assertFalse(events[-1]['first_failed_read_capture'])
        self.assertFalse(events[-1]['latest_ready_is_failing_read'])
        self.assertIn('readiness:launcher_lease_missing_expired_or_future',issues)

    def test_post_read_binding_failure_does_not_claim_failed_read_capture(self):
        pilot,events=self.pilot();valid,issues,_=pilot.execution_ready()
        self.assertFalse(valid);self.assertIsNone(pilot.first_ready_read_failure)
        self.assertEqual(events[-1]['stage'],'waypoint_binding')
        self.assertFalse(events[-1]['first_failed_read_capture'])
        self.assertTrue(events[-1]['latest_ready_is_failing_read'])

    def test_read_open_failure_does_not_invent_raw_payload(self):
        self.path.unlink();pilot,events=self.pilot();pilot.execution_ready()
        failure=pilot.first_ready_read_failure
        self.assertIsNone(failure['raw_bytes'])
        self.assertFalse(failure['metadata']['exact_failed_read_bytes_retained'])
        self.assertIsNone(failure['metadata']['sha256'])
        self.assertEqual(failure['metadata']['exception_type'],'FileNotFoundError')

    def test_finish_persists_raw_after_writer_closed_and_refuses_overwrite(self):
        self.now=ready_frame()['time_ns']+600_000_000
        pilot,_=self.pilot();pilot.execution_ready();raw=pilot.first_ready_read_failure['raw_bytes'];closed=[]
        pilot.writer=SimpleNamespace(close=lambda:(closed.append(True) or dict(evidence_complete=True)))
        pilot.output=self.directory;pilot.scan_timing_capture_errors=0;pilot.scan_timing_last_capture_error=None
        pilot.scan_timing_firstbad=pilot.scan_timing_latest_check=pilot.scan_timing_last_good_check=None
        pilot.scan_timing_ring=[]
        finish=extracted(CODE/'waypoint_batch.py','finish_evidence',dict(evidence_json=contract.evidence_json),'WaypointPilot')
        result=finish(pilot)
        self.assertEqual(closed,[True]);self.assertEqual((self.directory/'ready_read_failure.raw').read_bytes(),raw)
        meta=json.loads((self.directory/'ready_read_failure.json').read_text())
        self.assertEqual(meta['sha256'],hashlib.sha256(raw).hexdigest())
        self.assertIsNone(meta['frozen_internal_now_ns']);self.assertEqual(result['first_ready_read_failure'],meta)
        with self.assertRaises(FileExistsError):finish(pilot)


if __name__=='__main__':
    unittest.main(verbosity=2)
