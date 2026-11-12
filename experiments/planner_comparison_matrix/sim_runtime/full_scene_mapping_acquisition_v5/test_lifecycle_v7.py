"""Pure health-entry/checkpoint regressions; no ROS or actual filesystem sync."""
import ast,copy,math
from pathlib import Path
from types import SimpleNamespace as N
import unittest
from unittest import mock
import full_scene_mapping as owner
import full_scene_mapping_contract as c
from test_v4_core import Clock,model,truth,tick_fixture,policy,target
from test_startup_v6 import actual_function,install

HERE=Path(__file__).resolve().parent
def health_fixture(refresh=True,stop=False,fatal=False):
    clock=Clock();m=model();m['max_truth_age_sec']=.025;sample=truth(clock,age=.0663);sample.update(advanced_steady=clock.monotonic())
    latest=dict(truth=sample);cache=dict(stamp_ns=clock.time_ns(),raw_received_steady=clock.monotonic(),points=[],pose={})
    state=dict(contract=c,time=clock,math=math,clock_pair=None,route={'max_runtime_sec':180.},started=clock.monotonic(),model=m,
        writer=N(summary=lambda:dict(dropped=0,worker_error=None)),audit=dict(valid=True,first_violation=None,checked_steady=clock.monotonic()),
        imu_gate_fault=None,raw_imu_gate=N(fresh=lambda *a:True),latest=latest,clouds=[],scans=[],cloud_cache=cache,stopped=[],
        release=dict(expected_sensor_publishers={},recorder_node_name='fixture_recorder',recorder_node_namespace='/',required_recorded_topics=['/odom']),
        node=N(get_publishers_info_by_topic=lambda t:[],get_service_names_and_types=lambda:[],get_node_names_and_namespaces=lambda:[('fixture_recorder','/')],
            get_subscriptions_info_by_topic=lambda t:[N(node_name='fixture_recorder',node_namespace='/')]))
    def spin():
        clock.sleep(.01)
        if stop:state['stopped'].append(2)
        if fatal:state['imu_gate_fault']={'reason':'fixture fatal'}
        if refresh:sample.update(truth(clock,age=.005),advanced_steady=clock.monotonic())
        for key,frame in [('raw_imu','map'),('imu','imu_link')]:latest[key]=dict(stamp_ns=clock.time_ns(),received_steady=clock.monotonic(),advanced_steady=clock.monotonic(),frame_id=frame,stamp_regressed=False,acceleration=[0.,0.,9.8],angular_velocity=[0.,0.,0.])
        cache.update(stamp_ns=clock.time_ns(),raw_received_steady=clock.monotonic())
    state['callbacks']=N(spin=spin);return state,install(actual_function('health'),state)

class Lifecycle(unittest.TestCase):
    def test_non_tick_health_refreshes_original_stamp_and_never_restamps_cache(self):
        state,health=health_fixture();old=state['latest']['truth']['stamp_ns'];new,yaw,cache,age=health()
        self.assertNotEqual(new['stamp_ns'],old);self.assertEqual(new['stamp_ns'],state['time'].time_ns()-5_000_000);self.assertAlmostEqual(age,.005)
        state,health=health_fixture(refresh=False);old=state['latest']['truth']['stamp_ns']
        with self.assertRaises(c.Hold):health()
        self.assertEqual(state['latest']['truth']['stamp_ns'],old)
    def test_stop_or_fatal_during_new_pump_prevents_health_admission(self):
        state,health=health_fixture(stop=True)
        with self.assertRaises(InterruptedError):health()
        state,health=health_fixture(fatal=True)
        with self.assertRaisesRegex(c.Hold,'latched raw IMU'):health()
    def test_original_final_age_and_issued_renewal_guards_still_reject(self):
        ns,p,rows=tick_fixture();bridge=((0,0,0),target(p),p)
        with mock.patch.object(c,'guard_hits',side_effect=lambda *a:ns['time'].sleep(.031) or {}),self.assertRaises(c.Hold):ns['tick'](bridge=bridge,lateral_raw=.4)
        self.assertFalse(rows)
        ns,p,rows=tick_fixture();ns['relay'].delay=.11
        with self.assertRaisesRegex(c.Hold,'write/dispatch'):ns['tick'](bridge=((0,0,0),target(p),p),lateral_raw=.4)
        self.assertAlmostEqual(ns['time'].monotonic()-ns['last_send'],.11)
    def test_checkpoint_uses_bounded_emit_preserving_every_original_receipt_field(self):
        old_path=HERE.parents[1]/'full_scene_mapping_lateral_owner_v6/review_source_v6/full_scene_mapping.py'
        if not old_path.exists():
            relative=Path('full_scene_mapping_lateral_owner_v6/review_source_v6/full_scene_mapping.py')
            candidates=[parent/relative for parent in HERE.parents]
            candidates.extend(parent/'analysis_outputs/planner_comparison_20261105'/relative for parent in HERE.parents)
            old_path=next((candidate for candidate in candidates if candidate.is_file()),old_path)
        self.assertTrue(old_path.is_file(),'Frozen revision6 baseline fixture missing from workspace analysis_outputs')
        old=ast.parse(old_path.read_text());new=ast.parse((HERE/'full_scene_mapping.py').read_text())
        previous=next(n for n in ast.walk(old) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='atomic_json' and n.args and 'runtime_ready.json' in ast.unparse(n.args[0]))
        current=next(n for n in ast.walk(new) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='emit' and n.args and isinstance(n.args[0],ast.Constant) and n.args[0].value=='runtime_ready')
        self.assertEqual(ast.dump(previous.args[1],include_attributes=False),ast.dump(current.keywords[0].value,include_attributes=False))
        self.assertFalse(any(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='atomic_json' and 'runtime_ready' in ast.unparse(n) for n in ast.walk(new)))
        clock=Clock();rows=[];m=model();m['max_truth_age_sec']=.025
        release=dict(release_id='a'*32,route_sha256='b'*64,oracle_sha256='c'*64,sources_sha256={'fixture':'d'*64},controller_pid_namespace='fixture',scene_identity_proof={})
        state=dict(contract=c,time=clock,writer=N(submit=lambda name,value:rows.append((name,value)) or True),release=release,
            own={'pid':1},relay=N(identity={'pid':2},source_sha256='e'*64),release_sha256='f'*64,lateral_admission={},output=Path('/fixture'),truth={'stamp_ns':123},model=m,
            atomic_json=mock.Mock(side_effect=AssertionError('live checkpoint fsync forbidden')))
        install(actual_function('emit'),state)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[ast.Expr(value=copy.deepcopy(current))],type_ignores=[])),'<actual v7 checkpoint>','exec'),state)
        self.assertEqual(len(rows),1);self.assertEqual(rows[0][0],'events');self.assertEqual(rows[0][1]['kind'],'runtime_ready');self.assertEqual(rows[0][1]['receipt']['truth']['stamp_ns'],123)
        self.assertEqual(clock.monotonic(),10.);state['atomic_json'].assert_not_called()
        state['writer'].submit=lambda *a:False
        with self.assertRaisesRegex(c.Hold,'writer rejected'):exec(compile(ast.fix_missing_locations(ast.Module(body=[ast.Expr(value=copy.deepcopy(current))],type_ignores=[])),'<rejected v7 checkpoint>','exec'),state)
    def test_writer_failure_still_HOLD_before_GT_and_close_failure_code_is_preserved(self):
        state,health=health_fixture();state['writer'].summary=lambda:dict(dropped=0,worker_error={'message':'fixture'})
        with self.assertRaisesRegex(c.Hold,'writer evidence gap'):health()
        tree=ast.parse((HERE/'full_scene_mapping.py').read_text())
        self.assertTrue(any(isinstance(n,ast.If) and ast.unparse(n.test)=="not result['writer']['evidence_complete']" for n in ast.walk(tree)))

if __name__=='__main__':unittest.main(verbosity=2)
