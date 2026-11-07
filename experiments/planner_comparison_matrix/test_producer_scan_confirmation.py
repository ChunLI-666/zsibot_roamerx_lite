"""Current strict-READY false frame, unchanged metadata, bounded scan confirmation; no ROS."""
import ast,copy,hashlib,importlib.util,json,math
from pathlib import Path
from types import MethodType,SimpleNamespace
import unittest
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('producer_scan_contract',HERE/'goal_pilot_source/waypoint_contract.py');c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)
import test_scan_aging_confirmation as aging
import test_ready_read_capture as receiver
NOW=1_800_000_000_000_000_000;TOPIC='/laser_scan_leveled';OWN='1'*32
ISSUE='readiness:launcher_not_ready:Stale/future stamp '+TOPIC

def frame(reason=None):
    reason=reason or 'Stale/future stamp '+TOPIC
    obs=dict(topic=TOPIC,header_ns=NOW-310_000_000,frame_id='base_link',ranges=723,
      receive_steady_seconds=99.98,validation_clock_begin_ns=NOW-1_000_000,validation_clock_end_ns=NOW,
      validation_steady_begin_ns=100_000_000_000,validation_steady_end_ns=100_001_000_000,failure_reason=reason)
    return dict(ready=False,reason=reason,time_ns=NOW,valid_until_ns=NOW+500_000_000,variant='official',action_server='/navigate_to_pose',
      params_sha256='a'*64,build_receipt_sha256='b'*64,goal_tolerances=dict(xy=.15,yaw=.08,stateful=False),
      waypoint_action_server=c.ACTION,waypoint_contract=dict(package_version='1.3.13',parameters=c.WAYPOINT_PARAMETERS,
        number_of_loops=0,goal_index=0,underlay_files_sha256={'underlay':'c'*64},client_sources_sha256={'client':'d'*64}),
      mission_deadline=dict(run_id='fixed',manifest_sha256='e'*64,launcher_boot_id='boot',deadline_boottime_sec=3300.),
      waypoint_ownership=dict(exclusive_nav_client=True,status_publishers_match=True,checked_unix_ns=NOW,
        follow_waypoints_active=[dict(uuid=OWN,status=2)],navigate_to_pose_active=[]),producer_scan_observation=obs)
def bindings(ready):
    return {k:ready[k] for k in ['variant','action_server','goal_tolerances','params_sha256','build_receipt_sha256']},c.waypoint_binding(ready)
def verify(ready,original=None,now=NOW):
    original=original or ready;binding,native=bindings(original)
    return c.producer_scan_confirmation_failure(json.dumps(ready).encode(),'official',binding,native,{'client':'d'*64},OWN,NOW-100_000_000,now,TOPIC)
def new_ready(original):
    ready=copy.deepcopy(original);ready.update(ready=True,reason=None,time_ns=NOW+20_000_000)
    obs=ready['producer_scan_observation'];obs.update(header_ns=NOW-210_000_000,validation_clock_begin_ns=NOW+19_000_000,validation_clock_end_ns=NOW+20_000_000,failure_reason=None)
    return ready
class ContractTests(unittest.TestCase):
    def test_current_original_raw_readonly_verified_never_rewritten_true(self):
        ready=frame();before=copy.deepcopy(ready);raw=json.dumps(ready).encode();result=verify(ready)
        self.assertEqual(ready,before);self.assertIs(result['ready']['ready'],False);self.assertEqual(result['sha256'],hashlib.sha256(raw).hexdigest());self.assertEqual(result['original_stamp_ns'],NOW-310_000_000)
    def test_same_scan_stale_receive_allowed_only_with_proven_old_receipt(self):
        ready=frame('Stale receive '+TOPIC);ready['producer_scan_observation']['receive_steady_seconds']=99.69
        self.assertEqual(verify(ready)['original_stamp_ns'],NOW-310_000_000)
        ready['producer_scan_observation']['receive_steady_seconds']=99.9
        with self.assertRaises(ValueError):verify(ready)
    def test_future_wrong_topic_missing_context_and_other_reason_rejected(self):
        for mutation in [lambda r:r['producer_scan_observation'].update(header_ns=NOW+60_000_000),lambda r:r.update(reason='Stale receive /odom/current_pose'),lambda r:r['producer_scan_observation'].update(topic='/other'),lambda r:r.pop('producer_scan_observation'),lambda r:r['producer_scan_observation'].update(frame_id='lidar')]:
            ready=frame();mutation(ready)
            with self.subTest(ready=ready),self.assertRaises((ValueError,TypeError)):verify(ready)
    def test_not_ready_cannot_mask_expiry_graph_foreign_UUID_or_changed_binding(self):
        original=frame()
        mutations=[lambda r:r.update(valid_until_ns=NOW),lambda r:r.update(time_ns=NOW+50_000_001),lambda r:r['waypoint_ownership'].update(checked_unix_ns=NOW-500_000_001),lambda r:r['waypoint_ownership'].update(exclusive_nav_client=False),lambda r:r['waypoint_ownership'].update(follow_waypoints_active=[dict(uuid='f'*32)]),lambda r:r.update(params_sha256='f'*64),lambda r:r['mission_deadline'].update(run_id='foreign'),lambda r:r['waypoint_contract'].update(client_sources_sha256={'client':'f'*64})]
        for mutation in mutations:
            ready=copy.deepcopy(original);mutation(ready)
            with self.subTest(ready=ready),self.assertRaises(ValueError):verify(ready,original)
    def test_recovery_requires_new_strict_ready_and_both_later_original_stamps(self):
        original=verify(frame());ready=new_ready(original['ready']);snapshot=aging.snapshot(original['original_stamp_ns']+100_000_000,.2)
        outcome=lambda r,s=snapshot:c.producer_scan_confirmation_outcome([],s,original,r,20_000_000,50_000_000,False)
        self.assertEqual(outcome(ready),'recovered')
        for mutation in [lambda r:r.update(ready=False),lambda r:r.update(time_ns=original['issued_ns']),lambda r:r['producer_scan_observation'].update(header_ns=original['original_stamp_ns'])]:
            changed=copy.deepcopy(ready);mutation(changed);self.assertNotEqual(outcome(changed),'recovered')
        self.assertNotEqual(outcome(ready,aging.snapshot(original['original_stamp_ns'],.2)),'recovered')
    def test_persistent_other_guard_future_and_late_never_recover(self):
        original=verify(frame());ready=new_ready(original['ready']);snapshot=aging.snapshot(original['original_stamp_ns']+100_000_000,.2)
        self.assertEqual(c.producer_scan_confirmation_outcome([ISSUE],snapshot,original,original['ready'],10_000_000,50_000_000,True),'pending')
        self.assertEqual(c.producer_scan_confirmation_outcome(['odom:invalid_header_age'],snapshot,original,ready,10_000_000,50_000_000,False),'fatal')
        self.assertEqual(c.producer_scan_confirmation_outcome([],snapshot,original,ready,50_000_001,50_000_000,False),'expired')
        ready['producer_scan_observation']['header_ns']=NOW+100_000_000
        self.assertEqual(c.producer_scan_confirmation_outcome([],snapshot,original,ready,20_000_000,50_000_000,False),'fatal')
    def test_actual_confirmation_permanently_records_false_strictFAIL_and_exact_raw(self):
        ready=frame();fresh=new_ready(ready);own=aging.snapshot(NOW-210_000_000,.2)
        pilot=aging.Pilot(increments=[10_000_000]);pilot.args.variant='official';pilot.args.scan_topic=TOPIC
        pilot.runtime_binding,pilot.native_binding=bindings(ready);pilot.loaded_sources={'client':'d'*64};pilot.owned_identity=c.OwnedGoalIdentity(bytes.fromhex(OWN));pilot.goal_sent_ns=NOW-100_000_000
        raw=json.dumps(ready).encode();pilot.current_ready_read_failure=dict(raw_bytes=raw,metadata=dict(sha256=hashlib.sha256(raw).hexdigest()))
        pilot.latest_ready=ready;ns=dict(time=SimpleNamespace(time_ns=lambda:NOW),producer_scan_confirmation_failure=c.producer_scan_confirmation_failure)
        producer=receiver.extracted(HERE/'goal_pilot_source/waypoint_batch.py','producer_scan_confirmation_failure',ns,'WaypointPilot')
        ns['producer_scan_confirmation_failure']=c.producer_scan_confirmation_failure
        pilot.producer_scan_confirmation_failure=MethodType(producer,pilot)
        # The actual confirmation method resolves its helper in its original AST namespace.
        pilot.run.__func__.__globals__['producer_scan_confirmation_outcome']=c.producer_scan_confirmation_outcome
        def execution():pilot.latest_ready=fresh;return True,[],own
        pilot.execution_ready=execution
        self.assertTrue(pilot.run([ISSUE],own,SimpleNamespace(done=lambda:False))[0])
        episode=pilot.scan_aging_episodes[0];self.assertEqual(episode['strict_timing_status'],'FAIL');self.assertFalse(episode['strict_timing_accepted']);self.assertEqual(pilot.producer_scan_failure_reads[0][1]['raw_bytes'],raw);self.assertIn(ISSUE,pilot.artifact_invalid_reasons)
    def test_default_zero_does_not_read_or_accept_producer_failure(self):
        pilot=aging.Pilot(seconds=0.);pilot.producer_scan_confirmation_failure=lambda *a:(_ for _ in ()).throw(AssertionError('must remain default strict'))
        self.assertFalse(pilot.run([ISSUE],aging.snapshot(),SimpleNamespace(done=lambda:False))[0]);self.assertFalse(pilot.scan_aging_episodes)
    def test_runtime_first_failure_is_not_preflight_slot_or_lastpassing_cache(self):
        fixture=receiver.ReceiverCaptureTests('runTest');fixture.setUp();self.addCleanup(fixture.tearDown)
        fixture.now=receiver.ready_frame()['time_ns']+600_000_000;pilot,_=fixture.pilot();pilot.execution_ready();preflight=pilot.first_ready_read_failure
        raw=b'{exact second runtime failure';fixture.path.write_bytes(raw);pilot.goal_sent_ns=1;pilot.owned_identity=c.OwnedGoalIdentity(bytes.fromhex(OWN));pilot.execution_ready()
        self.assertIs(pilot.first_ready_read_failure,preflight);self.assertEqual(pilot.first_runtime_ready_read_failure['raw_bytes'],raw);self.assertEqual(pilot.current_ready_read_failure['raw_bytes'],raw);self.assertEqual(pilot.latest_ready,dict(last_passing_cache=True))
if __name__=='__main__':unittest.main(verbosity=2)
