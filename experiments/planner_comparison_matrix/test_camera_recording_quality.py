"""Original B37 trigger snapshots through actual frozen and new guards; no ROS."""
import ast
import copy
import json
import math
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
import uuid

from test_native_waypoints import load_method
import test_simulation_scan_profile as scan_tests
from test_mission_state import fixture
from mission_state import MissionStore, atomic_json

HERE=Path(__file__).resolve().parent
def function(path,name,scope):
    node=next(n for n in ast.walk(ast.parse(path.read_text())) if isinstance(n,ast.FunctionDef) and n.name==name)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),scope)
    return scope[name]

original=function(HERE/'goal_pilot_source/pilot_scan_timing.py','input_ready',dict(math=math))
producer_observation=function(HERE/'matrix_navigation.py','camera_recording_observation',{})

def fresh_snapshot():
    age=dict(receive_age_s=.01,header_age_s=.01,stamp_ns=100)
    return dict(last=dict(pose=dict(age,frame_id='map',x=0.,y=0.,yaw=0.,raw_pose_values_finite=True),
        odom=dict(age,frame_id='odom',child_frame_id='base_link',vx=0.,vy=0.,wz=0.),
        loc_status=dict(age,value=1),pose_valid=dict(age,value=True),
        scan=dict(age,frame_id='base_link',ranges=723,confirmation_payload_valid=True),
        camera=dict(age,decoded_shape=[360,640,3]),camera_info=dict(width=640,height=360,fx=100.,fy=100.),
        **{'/cmd_vel_safe':dict(age,vx=0.,vy=0.,wz=0.)}),
        tf={k:dict(header_age_s=.01,advance_age_s=.01,regressed=False) for k in ['map->odom','odom->base_link']})

def pilot(snapshot,running=True):
    obj=NS(goal_sent_ns=1 if running else None,args=NS(execute=False,scan_aging_confirm_sec=0.),
        simulation_scan_max_age_sec=.5,simulation_scan_sequence_fault=None,
        strict_scan_latency_first_fault=None,strict_scan_latency_seen=set(),strict_scan_latency_count=0,
        events=[],invalid=[],check_interruption=lambda:None,current_summary=lambda:copy.deepcopy(snapshot),
        scan_timing_record_check=lambda *a:None)
    obj.input_ready=lambda **kw:original(obj,**kw)
    obj.emit=lambda kind,**kw:obj.events.append((kind,kw));obj.invalidate_artifact=lambda issues:obj.invalid.extend(issues)
    return obj

class CameraRecordingTests(unittest.TestCase):
    def test_actual_B37_two_trigger_snapshots_keep_scan_strict_fail_without_camera_cancel(self):
        workspace=Path(os.environ['MATRIX_EVIDENCE_ROOT']) if os.environ.get('MATRIX_EVIDENCE_ROOT') else next(
            (a for a in HERE.parents if (a/'analysis_outputs').is_dir()),HERE)
        root=workspace/'analysis_outputs/planner_comparison_20261105/space_agent_recovery_20261110/terra/B37_handoff/stages/mission_execute/invocation_000/client_output'
        if not root.exists():self.skipTest('Sealed B37 evidence unavailable')
        for attempt in ['attempt_00','attempt_01']:
            with self.subTest(attempt=attempt):
                path=root/attempt/'scan_timing_evidence.json';raw=path.read_bytes()
                snap=json.loads(raw)['first_runtime_invalid']['readiness_check']['snapshot'];p=pilot(snap)
                self.assertIn('camera_missing_stale_or_decode_failed',original(p)[1])
                valid,issues,observed=load_method('execution_ready')[0](p)
                self.assertTrue(valid,issues);self.assertEqual(observed,snap)
                self.assertEqual(p.camera_recording_quality['status'],'FAIL')
                self.assertEqual(p.strict_scan_latency_first_fault['strict_timing_status'],'FAIL')
                self.assertEqual(path.read_bytes(),raw)
    def test_preflight_retains_stale_decode_calibration_shape_checks(self):
        for change in [lambda s:s['last']['camera'].update(header_age_s=.51),
                       lambda s:s['last']['camera'].update(decoded_shape=None),
                       lambda s:s['last']['camera_info'].update(fx=0),
                       lambda s:s['last']['camera_info'].update(width=100)]:
            s=fresh_snapshot();change(s)
            self.assertFalse(load_method('execution_ready')[0](pilot(s,False))[0])
    def test_runtime_missing_camera_records_failure_and_latches_after_recovery(self):
        s=fresh_snapshot();s['last'].pop('camera');p=pilot(s)
        check=load_method('execution_ready')[0]
        self.assertTrue(check(p)[0]);self.assertEqual(p.camera_recording_quality['status'],'FAIL')
        p.current_summary=fresh_snapshot
        self.assertTrue(check(p)[0]);self.assertEqual(p.camera_recording_quality['status'],'FAIL')
        self.assertEqual(len(p.events),1)
    def test_camera_split_does_not_weaken_nav_health_or_stop(self):
        mutations=[lambda s:s['last']['scan'].update(header_age_s=.501),
            lambda s:s['last']['odom'].update(header_age_s=.301),
            lambda s:s['last']['pose'].update(x=math.nan),
            lambda s:s['tf']['map->odom'].update(header_age_s=.301),
            lambda s:s['last']['pose_valid'].update(value=False),
            lambda s:s['last']['loc_status'].update(value=0)]
        for mutate in mutations:
            s=fresh_snapshot();s['last']['camera']['header_age_s']=.6;mutate(s)
            self.assertFalse(load_method('execution_ready')[0](pilot(s))[0])
        s=fresh_snapshot();s['last']['odom']['vx']=.04
        self.assertFalse(load_method('execution_ready')[0](pilot(s),require_stopped=True)[0])
    def test_normal_launcher_health_ignores_runtime_camera_but_records_original_stamp(self):
        env=scan_tests.SimulationScanProfileTests().environment(.5,.4)
        env['latest']['camera']=(.2,scan_tests.stamp(.6));raw=copy.deepcopy(env['latest']['camera'][1].header.stamp)
        env['source_health']()
        evidence=producer_observation(env['latest'],'camera',10**9,1.)
        self.assertEqual(evidence['status'],'FAIL');self.assertEqual(evidence['original_stamp_ns'],400000000)
        self.assertEqual(env['latest']['camera'][1].header.stamp,raw)
        env['latest'].pop('camera');env['source_health']()
        self.assertEqual(producer_observation(env['latest'],'camera',10**9,1.)['issues'],['camera_missing'])
    def test_real_mission_store_persists_camera_fail_without_progress_or_budget_reset(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);store=MissionStore(root/'mission.db');raw,ident,policy,_=fixture()
            try:
                store.create(raw,ident,policy);a=store.reserve(str(uuid.uuid4()),root)
                result=dict(action_uuid=a['parent_action_uuid'],status=6,error_code=0,missed_waypoints=[dict(index=0,error_code=106)],
                    children_quiescent=True,final_measured_and_commanded_stop=True,
                    owned_goal_shutdown=dict(stop=dict(observed=True,strict_stop_issues=[])),
                    camera_recording_quality=dict(status='FAIL',strict_timing_status='FAIL'))
                path=root/'native_result.json';atomic_json(path,result);store.finish_attempt(result,True,result_path=path)
                state=store.export(root/'export')
                self.assertEqual(state['result_flags']['strict_timing_status'],'FAIL')
                self.assertEqual(state['result_flags']['camera_recording_quality_status'],'FAIL')
                self.assertEqual(state['progress']['accepted_prefix_count'],0)
                self.assertEqual(state['budgets']['mission_dispatches_reserved'],1)
            finally:store.close()
    def test_empty_front_camera_bag_is_delivery_fail(self):
        tree=ast.parse((HERE/'matrix_navigation.py').read_text())
        expr=next(n.value for n in ast.walk(tree) if isinstance(n,ast.keyword) and n.arg=='front_camera_delivery_status')
        code=compile(ast.Expression(expr),'<normal final recording audit>','eval')
        for counts in [{},{'image':4},{'info':4}]:
            self.assertEqual(eval(code,dict(topic_counts=counts,args=NS(camera_topic='image',camera_info_topic='info'))),'FAIL')

if __name__=='__main__':unittest.main()
