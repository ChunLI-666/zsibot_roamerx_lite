"""No ROS imports/Nodes: pure contracts and the actual batch method under fake clocks."""
import ast
import copy
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('candidate_contract', ROOT/'goal_pilot_source/waypoint_contract.py')
contract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(contract)
ISSUE = 'scan:invalid_header_age'


def snapshot(stamp=1_794_025_485_720_771_937, age=.300022422, received=.188411977):
    return dict(last=dict(
        scan=dict(stamp_ns=stamp, header_age_s=age, receive_age_s=received,
                  frame_id='base_link', ranges=723, confirmation_payload_valid=True),
        pose=dict(stamp_ns=100, x=0., y=0., yaw=0., frame_id='map',
                  header_age_s=.01, receive_age_s=.01),
        odom=dict(vx=0., vy=0., wz=0., frame_id='odom', child_frame_id='base_link',
                  header_age_s=.01, receive_age_s=.01),
        **{'/cmd_vel_safe':dict(vx=0.,vy=0.,wz=0.,receive_age_s=.01)}))


class Clock:
    def __init__(self):
        self.ns = 1_000_000_000
    def monotonic_ns(self):
        return self.ns
    def monotonic(self):
        return self.ns/1e9


class FakeRos:
    def __init__(self, clock, increments):
        self.clock, self.increments, self.spins = clock, list(increments), 0
    def ok(self):
        return True
    def spin_once(self, node, timeout_sec):
        self.spins += 1
        self.clock.ns += self.increments.pop(0) if self.increments else int(timeout_sec*1e9)


class Pilot:
    def __init__(self, seconds=.05, checks=(), increments=()):
        self.args = SimpleNamespace(scan_aging_confirm_sec=seconds, timeout=3300., cancel_after=None)
        self.clock = Clock()
        self.ros = FakeRos(self.clock, increments)
        tree = ast.parse((ROOT/'goal_pilot_source/waypoint_batch.py').read_text())
        node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'WaypointPilot')
        method = copy.deepcopy(next(n for n in node.body if isinstance(n, ast.FunctionDef) and n.name == 'confirm_scan_aging'))
        namespace = dict(time=self.clock, rclpy=self.ros,
                         scan_aging_confirmation_eligible=contract.scan_aging_confirmation_eligible,
                         scan_aging_confirmation_outcome=contract.scan_aging_confirmation_outcome)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),
                     str(ROOT/'goal_pilot_source/waypoint_batch.py'), 'exec'), namespace)
        self.run = namespace['confirm_scan_aging'].__get__(self)
        self.cancel_requested = False
        self.goal_sent_steady = 0.
        self.feedback_overflow = 0
        self.scan_timing_latest_check = dict(check_enter_steady_ns=self.clock.ns)
        self.scan_aging_episodes, self.artifact_invalid_reasons, self.events = [], [], []
        self.checks = list(checks)
        self.ordered = contract.OrderedVisits([dict(id='N000',pose=[0.,0.,0.],tags=[])])
        self.ordered.feedback(0,100,100)
        self.ordered.observe(snapshot(age=.2),900_000_000,100,True)
        self.last = None
    def emit(self, kind, **fields):
        self.events.append(copy.deepcopy(dict(kind=kind, **fields)))
    def invalidate_artifact(self, issues):
        self.artifact_invalid_reasons.extend(x for x in issues if x not in self.artifact_invalid_reasons)
    def observe_ordered(self, healthy, current):
        self.ordered.observe(current,self.clock.ns,self.clock.ns,healthy)
    def check_interruption(self):
        pass
    def execution_ready(self):
        if self.checks:
            self.last = self.checks.pop(0)
        if self.last is None:
            self.last = ([ISSUE],snapshot())
        issues, current = self.last
        return not issues,list(issues),copy.deepcopy(current)


class ConfirmationTests(unittest.TestCase):
    def test_budget_boundaries(self):
        for value in [0.,.000001,.049,.05]:
            contract.validate_scan_aging_confirmation(value)
        for value in [-.0001,.050000001,.6,math.nan,math.inf,True,None]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                contract.validate_scan_aging_confirmation(value)

    def test_only_header_aging_is_eligible(self):
        self.assertTrue(contract.scan_aging_confirmation_eligible([ISSUE],snapshot()))
        for issues in [[],['scan:missing_or_stale_receive'],[ISSUE,'odom:invalid_header_age'],
                       [ISSUE,'map->odom:missing_stale_future_or_regressed'],
                       [ISSUE,'readiness:Waypoint action ownership observation is stale or future'],
                       [ISSUE,'localization_not_NORMAL'],[ISSUE,'scan_frame_or_payload_invalid']]:
            with self.subTest(issues=issues):
                self.assertFalse(contract.scan_aging_confirmation_eligible(issues,snapshot()))

    def test_malformed_or_future_scan_never_eligible(self):
        for key,value in [('header_age_s',.3),('header_age_s',-.051),('header_age_s',math.nan),
                          ('receive_age_s',.300000001),('receive_age_s',math.nan),('receive_age_s',-.001),
                          ('frame_id','map'),('ranges',0),('stamp_ns',0),('stamp_ns',True),
                          ('confirmation_payload_valid',False)]:
            current = snapshot()
            current['last']['scan'][key] = value
            with self.subTest(key=key,value=value):
                self.assertFalse(contract.scan_aging_confirmation_eligible([ISSUE],current))
        self.assertFalse(contract.scan_aging_confirmation_eligible([ISSUE],dict(last={})))

    def test_payload_nan_fatal_positive_inf_is_normal(self):
        msg = SimpleNamespace(angle_min=-math.pi,angle_max=math.pi,angle_increment=.01,
                              time_increment=0.,scan_time=.1,range_min=.1,range_max=80.,ranges=[1.,math.inf])
        self.assertTrue(contract.scan_confirmation_payload_valid(msg))
        for values in [[1.,math.nan],[1.,-math.inf],[]]:
            msg.ranges = values
            self.assertFalse(contract.scan_confirmation_payload_valid(msg))
        msg.ranges = [1.]
        msg.angle_increment = math.nan
        self.assertFalse(contract.scan_confirmation_payload_valid(msg))

    def test_original_point_three_limit_stays_exact(self):
        original = snapshot()['last']['scan']['stamp_ns']
        for age in [.3,-.05]:
            self.assertEqual('recovered',contract.scan_aging_confirmation_outcome([],snapshot(original+1,age),original,49_000_000,50_000_000))
        for age in [.300000001,-.050000001,math.nan]:
            self.assertEqual('fatal',contract.scan_aging_confirmation_outcome([],snapshot(original+1,age),original,49_000_000,50_000_000))

    def test_new_original_stamp_required(self):
        original = snapshot()['last']['scan']['stamp_ns']
        for stamp in [original,original-1,0]:
            self.assertEqual('fatal',contract.scan_aging_confirmation_outcome([],snapshot(stamp,.2),original,1,50_000_000))

    def test_exact_fifty_ms_allowed_and_one_ns_late_rejected(self):
        original = snapshot()['last']['scan']['stamp_ns']
        current = snapshot(original+100_000_000,.2)
        self.assertEqual('recovered',contract.scan_aging_confirmation_outcome([],current,original,50_000_000,50_000_000))
        self.assertEqual('expired',contract.scan_aging_confirmation_outcome([],current,original,50_000_001,50_000_000))

    def test_default_zero_does_not_spin_invalidate_or_reset(self):
        pilot = Pilot(seconds=0.)
        before = pilot.ordered.dwell_start
        valid,issues,current = pilot.run([ISSUE],snapshot(),SimpleNamespace(done=lambda:False))
        self.assertFalse(valid)
        self.assertEqual([ISSUE],issues)
        self.assertEqual(0,pilot.ros.spins)
        self.assertEqual([],pilot.scan_aging_episodes)
        self.assertEqual([],pilot.artifact_invalid_reasons)
        self.assertEqual(before,pilot.ordered.dwell_start)

    def test_actual_b5_cache_delay_recovers_but_strict_fail_persists(self):
        original = snapshot()['last']['scan']['stamp_ns']
        fresh = snapshot(original+100_000_000,.201840699,.0001)
        # B5: callback enter minus initial check enter; .806326ms was the
        # separate callback-enter minus check-complete measurement.
        pilot = Pilot(checks=[([],fresh)],increments=[1_811_550])
        valid,issues,current = pilot.run([ISSUE],snapshot(),SimpleNamespace(done=lambda:False))
        self.assertTrue(valid)
        self.assertEqual([],issues)
        self.assertEqual(original+100_000_000,current['last']['scan']['stamp_ns'])
        self.assertEqual([ISSUE],pilot.artifact_invalid_reasons)
        self.assertEqual('FAIL',pilot.scan_aging_episodes[0]['strict_timing_status'])
        self.assertFalse(pilot.scan_aging_episodes[0]['strict_timing_accepted'])
        self.assertEqual('performance_violation',pilot.events[0]['kind'])
        self.assertIsNone(pilot.ordered.dwell_start)
        pilot.observe_ordered(True,current)
        self.assertEqual(pilot.clock.ns,pilot.ordered.dwell_start)
        self.assertIsNone(pilot.ordered.visits[0]['passed'])

    def test_callback_can_run_late_but_cannot_be_accepted_late(self):
        fresh = snapshot(snapshot()['last']['scan']['stamp_ns']+100_000_000,.2)
        pilot = Pilot(checks=[([],fresh)],increments=[50_000_001])
        self.assertFalse(pilot.run([ISSUE],snapshot(),SimpleNamespace(done=lambda:False))[0])
        self.assertEqual('expired',pilot.scan_aging_episodes[0]['outcome'])

    def test_persistent_aging_still_fatal(self):
        pilot = Pilot()
        self.assertFalse(pilot.run([ISSUE],snapshot(),SimpleNamespace(done=lambda:False))[0])
        self.assertLessEqual(pilot.ros.spins,10)
        self.assertEqual('expired',pilot.scan_aging_episodes[0]['outcome'])

    def test_other_guard_during_confirmation_is_fatal_on_first_check(self):
        for issue in ['odom:invalid_header_age','scan:missing_or_stale_receive',
                      'readiness:Waypoint action ownership observation is stale or future',
                      'map->odom:missing_stale_future_or_regressed','localization_pose_invalid']:
            pilot = Pilot(checks=[([ISSUE,issue],snapshot())],increments=[800_000])
            with self.subTest(issue=issue):
                self.assertFalse(pilot.run([ISSUE],snapshot(),SimpleNamespace(done=lambda:False))[0])
                self.assertEqual(1,pilot.ros.spins)
                self.assertEqual('fatal',pilot.scan_aging_episodes[0]['outcome'])

    def test_existing_terminal_cancel_deadline_and_signal_guards(self):
        for setup in ['terminal','canceled','timeout','scheduled','signal']:
            pilot = Pilot()
            terminal = SimpleNamespace(done=lambda:setup=='terminal')
            if setup=='canceled': pilot.cancel_requested=True
            if setup=='timeout': pilot.args.timeout=1.
            if setup=='scheduled': pilot.args.cancel_after=.5
            if setup=='signal':
                def interrupted(): raise InterruptedError('signal')
                pilot.check_interruption=interrupted
                with self.assertRaises(InterruptedError): pilot.run([ISSUE],snapshot(),terminal)
                continue
            with self.subTest(setup=setup):
                self.assertFalse(pilot.run([ISSUE],snapshot(),terminal)[0])
                self.assertEqual(0,pilot.ros.spins)

    def test_ownership_boundaries_match_ready_lease_and_diagnostic_exists(self):
        ownership = dict(exclusive_nav_client=True,status_publishers_match=True,checked_unix_ns=1_000_000_000)
        for now in [950_000_000,1_500_000_000]: contract.validate_ownership(ownership,now)
        for now in [949_999_999,1_500_000_001]:
            with self.assertRaisesRegex(ValueError,'stale or future'): contract.validate_ownership(ownership,now)
        text=(ROOT/'goal_pilot_source/waypoint_batch.py').read_text()
        self.assertIn("self.emit('readiness_validation_failed'",text)
        self.assertIn('ownership_validation_unix_ns=ownership_checked_now_ns',text)

    def test_actual_ready_exception_records_exact_now_and_original_manifest(self):
        tree=ast.parse((ROOT/'goal_pilot_source/waypoint_batch.py').read_text())
        cls=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=='WaypointPilot')
        method=copy.deepcopy(next(node for node in cls.body if isinstance(node,ast.FunctionDef) and node.name=='execution_ready'))
        for checked,expected_age_ns in [(800_000_000,500_000_001),(1_350_000_002,-50_000_001)]:
            ready=dict(waypoint_ownership=dict(exclusive_nav_client=True,status_publishers_match=True,checked_unix_ns=checked))
            current=snapshot(age=.1)
            current['last']['pose']['raw_pose_values_finite']=True
            captured=[]
            fake=SimpleNamespace(args=SimpleNamespace(execute=True,scan_aging_confirm_sec=0.,ready=None,variant='official'),
                check_interruption=lambda:None,input_ready=lambda **kw:(True,[],current),
                runtime_binding=None,native_binding=None,owned_identity=None,goal_sent_ns=None,
                ownership_metadata_reported_check_ns=set(),
                first_ready_read_failure=None,
                event_pub=SimpleNamespace(get_subscription_count=lambda:1),
                emit=lambda kind,**fields:captured.append(dict(kind=kind,**fields)),
                scan_timing_record_check=lambda *args:None)
            namespace=dict(time=SimpleNamespace(time_ns=lambda:1_300_000_001,monotonic_ns=lambda:500),
                ReadyReadCapture=contract.ReadyReadCapture,
                frozen=SimpleNamespace(read_ready=lambda *args:(ready,{},'abc')),
                finite_velocity=contract.finite_velocity,waypoint_binding=lambda *args:{},
                validate_ownership=contract.validate_ownership,
                ownership_metadata_lease_deviation=contract.ownership_metadata_lease_deviation)
            exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),
                         'waypoint_batch.py','exec'),namespace)
            valid,issues,_=namespace['execution_ready'](fake)
            self.assertFalse(valid)
            self.assertEqual(['readiness:Waypoint action ownership observation is stale or future'],issues)
            failure=next(event for event in captured if event['kind']=='readiness_validation_failed')
            self.assertEqual('validate_ownership',failure['stage'])
            self.assertTrue(failure['latest_ready_is_failing_read'])
            self.assertEqual(expected_age_ns,failure['ownership_validation_unix_ns']-
                             failure['latest_ready']['waypoint_ownership']['checked_unix_ns'])



if __name__ == '__main__':
    unittest.main(verbosity=2)
