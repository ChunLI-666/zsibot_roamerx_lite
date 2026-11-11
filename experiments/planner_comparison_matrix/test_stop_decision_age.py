"""Actual normal methods, no ROS init; all original age domains at stop decision."""
from collections import Counter
from contextlib import ExitStack
import copy
from pathlib import Path
import sys
from types import MethodType,SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).parent/'goal_pilot_source'))
import waypoint_batch as batch
from waypoint_contract import strict_stopped


class DecisionAgeTests(unittest.TestCase):
    wall=1_000_000_000_000
    steady=1000.

    def node(self):
        common=dict(received_steady=self.steady,received_unix_ns=self.wall)
        pose=dict(common,stamp_ns=self.wall,frame_id='map',x=0.,y=0.,yaw=0.)
        velocity=dict(common,vx=0.,vy=0.,wz=0.)
        last=dict(pose=pose,odom=dict(velocity,stamp_ns=self.wall,frame_id='odom',child_frame_id='base_link'),
            loc_status=dict(common,value=1),pose_valid=dict(common,value=True),
            scan=dict(common,stamp_ns=self.wall,frame_id='base_link',ranges=360,confirmation_payload_valid=True),
            camera=dict(common,stamp_ns=self.wall,decoded_shape=[64,64,3]),camera_info=dict(common,width=64,height=64,fx=100.,fy=100.))
        last['/cmd_vel_safe']=dict(velocity)
        transforms={key:dict(stamp_ns=self.wall,regressed=False,advanced_steady=self.steady)
            for key in [('map','odom'),('odom','base_link')]}
        ready=dict(time_ns=self.wall,valid_until_ns=self.wall+500_000_000,
            waypoint_ownership=dict(checked_unix_ns=self.wall,exclusive_nav_client=True,status_publishers_match=True,
                navigate_to_pose_active=[],follow_waypoints_active=[]))
        n=SimpleNamespace(last=last,transforms=transforms,counts=Counter(),goal_sent_ns=1,owned_identity=None,
            simulation_scan_max_age_sec=.3,args=SimpleNamespace(execute=True),latest_ready=ready,
            strict_scan_latency_first_fault=None,strict_scan_latency_count=0,strict_scan_latency_seen=set(),
            events=[],artifact_invalid_reasons=[])
        n.emit=lambda kind,**fields:n.events.append((kind,fields))
        n.invalidate_artifact=lambda issues:n.artifact_invalid_reasons.extend(issues)
        n.scan_timing_record_check=MethodType(batch.frozen.Pilot.scan_timing_record_check,n)
        # Bind the production functions themselves, without editing/extracting AST.
        n.current_summary=MethodType(batch.frozen.Pilot.current_summary,n)
        n.input_ready=MethodType(batch.frozen.Pilot.input_ready,n)
        n.decision_ready=MethodType(batch.WaypointPilot.decision_ready,n)
        n.stop_decision_ready=MethodType(batch.WaypointPilot.stop_decision_ready,n)
        return n

    def decision(self,n,elapsed):
        n.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=self.wall+int(elapsed*1e9)))
        before=copy.deepcopy((n.last,n.transforms,n.latest_ready))
        with ExitStack() as stack:
            stack.enter_context(patch('time.monotonic',return_value=self.steady+elapsed))
            stack.enter_context(patch('time.monotonic_ns',return_value=int((self.steady+elapsed)*1e9)))
            stack.enter_context(patch('time.time_ns',return_value=self.wall+int(elapsed*1e9)))
            stack.enter_context(patch.object(batch.frozen,'read_ready',side_effect=AssertionError('second READY read')))
            stack.enter_context(patch.object(batch.rclpy,'spin_once',side_effect=AssertionError('new callback pump')))
            healthy,issues,snapshot=n.stop_decision_ready()
            stopped,stop_issues=strict_stopped(snapshot)
        self.assertEqual((n.last,n.transforms,n.latest_ready),before)
        return healthy and stopped,issues+stop_issues,snapshot

    def test_original_cache_passes_then_odom_header_ages_out(self):
        n=self.node();n.last['odom']['stamp_ns']-=200_000_000
        self.assertTrue(self.decision(n,0.)[0])
        ok,issues,snapshot=self.decision(n,.2)
        self.assertFalse(ok);self.assertIn('odom:invalid_header_age',issues)
        self.assertAlmostEqual(snapshot['last']['odom']['header_age_s'],.4)

    def test_common_gate_enforces_stopped_only_when_requested_and_has_no_io(self):
        n=self.node();n.last['odom']['vx']=.1
        n.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=self.wall))
        with patch('time.monotonic',return_value=self.steady),patch('time.monotonic_ns',return_value=int(self.steady*1e9)), \
                patch('time.time_ns',return_value=self.wall), \
                patch.object(Path,'read_text',side_effect=AssertionError('decision-time file read')), \
                patch.object(Path,'read_bytes',side_effect=AssertionError('decision-time file read')), \
                patch.object(batch.rclpy,'spin_once',side_effect=AssertionError('decision-time pump')):
            self.assertTrue(n.decision_ready()[0])
            self.assertFalse(n.decision_ready(require_stopped=True)[0])
            n.mission=SimpleNamespace(store=SimpleNamespace(remaining=lambda:0.))
            valid,issues,_=n.decision_ready()
            self.assertFalse(valid);self.assertTrue(any('FAILED_BUDGET' in issue for issue in issues))

    def test_every_participating_local_source_and_receive_age_is_rechecked(self):
        for key in ['pose','odom','scan']:
            with self.subTest(header=key):
                n=self.node();n.last[key]['stamp_ns']-=200_000_000
                self.assertFalse(self.decision(n,.2)[0])
        for key in ['pose','odom','scan','loc_status','pose_valid','/cmd_vel_safe']:
            with self.subTest(receive=key):
                n=self.node();n.last[key]['received_steady']-=.2
                self.assertFalse(self.decision(n,.2)[0])
        for key in [('map','odom'),('odom','base_link')]:
            for field in ['stamp_ns','advanced_steady']:
                with self.subTest(tf=key,field=field):
                    n=self.node();n.transforms[key][field]-=200_000_000 if field=='stamp_ns' else .2
                    self.assertFalse(self.decision(n,.2)[0])

    def test_ready_expiry_issue_age_and_ownership_absolute_age(self):
        n=self.node();n.latest_ready['valid_until_ns']=self.wall+100_000_000
        self.assertTrue(self.decision(n,0.)[0]);self.assertFalse(self.decision(n,.15)[0])
        n=self.node();n.latest_ready['time_ns']-=400_000_000
        self.assertIn('readiness:launcher_lease_missing_expired_or_future',self.decision(n,.2)[1])
        n=self.node();n.latest_ready['waypoint_ownership']['checked_unix_ns']-=400_000_000
        self.assertTrue(self.decision(n,0.)[0])
        self.assertIn('readiness:Waypoint action ownership observation is stale or future',self.decision(n,.2)[1])

    def test_camera_retains_preflight_only_policy_and_original_half_second_limit(self):
        n=self.node();n.last['camera']['stamp_ns']-=400_000_000
        self.assertTrue(self.decision(n,.2)[0])
        n.goal_sent_ns=None
        self.assertFalse(self.decision(n,.2)[0])

    def test_operational_scan_half_second_profile_reages_consumer_and_producer(self):
        for target in ['consumer_header','consumer_receive','producer_header','producer_receive']:
            with self.subTest(target=target):
                n=self.node();n.simulation_scan_max_age_sec=.5
                obs=dict(header_ns=self.wall,validation_clock_begin_ns=self.wall,validation_clock_end_ns=self.wall,
                    validation_steady_begin_ns=int(self.steady*1e9),validation_steady_end_ns=int(self.steady*1e9),
                    receive_steady_seconds=self.steady,frame_id='base_link',ranges=360)
                n.latest_ready['producer_scan_observation']=obs
                if target=='consumer_header':n.last['scan']['stamp_ns']-=400_000_000
                if target=='consumer_receive':n.last['scan']['received_steady']-=.4
                if target=='producer_header':obs['header_ns']-=400_000_000
                if target=='producer_receive':obs['receive_steady_seconds']-=.4
                self.assertTrue(self.decision(n,0.)[0])
                self.assertFalse(self.decision(n,.2)[0])

    def test_decision_scan_operational_point_four_latches_strict_fault_and_deduplicates(self):
        for target in ['consumer_header','consumer_receive','producer_header','producer_receive']:
            with self.subTest(target=target):
                n=self.node();n.simulation_scan_max_age_sec=.5
                obs=dict(header_ns=self.wall,validation_clock_begin_ns=self.wall,validation_clock_end_ns=self.wall,
                    validation_steady_begin_ns=int(self.steady*1e9),validation_steady_end_ns=int(self.steady*1e9),
                    receive_steady_seconds=self.steady,frame_id='base_link',ranges=360)
                n.latest_ready['producer_scan_observation']=obs
                if target=='consumer_header':n.last['scan']['stamp_ns']-=200_000_000
                if target=='consumer_receive':n.last['scan']['received_steady']-=.2
                if target=='producer_header':obs['header_ns']-=200_000_000
                if target=='producer_receive':obs['receive_steady_seconds']-=.2
                # The original full-check-time strict evidence is clean.
                self.assertTrue(self.decision(n,0.)[0]);self.assertIsNone(n.strict_scan_latency_first_fault)
                self.assertTrue(n.scan_timing_latest_check['valid'])
                self.assertTrue(self.decision(n,.2)[0])
                self.assertFalse(n.scan_timing_latest_check['valid'])
                first=n.strict_scan_latency_first_fault
                self.assertEqual(first['strict_timing_status'],'FAIL')
                self.assertEqual(n.strict_scan_latency_count,1)
                self.assertIn('scan:strict_300ms_latency_violation',n.artifact_invalid_reasons)
                deviation=first['deviations'][0]
                if target.startswith('producer'):
                    self.assertEqual(deviation['source'],'producer_original_scan_bounds')
                    self.assertEqual(deviation['original_observation'],obs)
                    key='header_age_bounds_s' if target.endswith('header') else 'receive_age_bounds_s'
                    self.assertAlmostEqual(deviation[key][0],.4)
                else:self.assertEqual(deviation['source'],'consumer_original_scan')
                self.assertTrue(self.decision(n,.21)[0]);self.assertEqual(n.strict_scan_latency_count,1)
                self.assertEqual(len(n.events),1)
                # A newer original scan can recover current health, never the verdict.
                n.last['scan'].update(stamp_ns=self.wall+200_000_000,received_steady=self.steady+.2)
                obs.update(header_ns=self.wall+200_000_000,receive_steady_seconds=self.steady+.2)
                self.assertTrue(self.decision(n,.22)[0]);self.assertTrue(n.scan_timing_latest_check['valid'])
                self.assertIs(n.strict_scan_latency_first_fault,first)
                self.assertEqual(n.strict_scan_latency_count,1)


if __name__=='__main__':unittest.main()
