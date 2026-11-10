"""Production reconcile CLI and durable evidence binding; transport is a no-ROS fixture."""
import copy
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
import uuid

HERE=Path(__file__).resolve().parent
SOURCE=HERE/'goal_pilot_source'
sys.path.insert(0,str(SOURCE))
from mission_state import MissionStore, Hold, atomic_json, digest, encoded
from test_mission_state import fixture


def observation(action_uuid,stopped):
    return dict(action_uuid=action_uuid,status=5,error_code=0,missed_waypoints=[],
        children_quiescent=True,final_measured_and_commanded_stop=stopped,
        owned_goal_shutdown=dict(stop=dict(observed=stopped,strict_stop_issues=[] if stopped else ['no_stop'])))


def cli_fixture():
    """Keep main(), run_attempt(), SQLite and file export real; replace only ROS I/O."""
    receipt=Path(sys.argv[2]);sample=json.loads(receipt.read_text())
    sys.argv=[str(SOURCE/'mission_resume.py'),*sys.argv[3:]]
    trace_path=receipt.with_suffix('.trace.json')
    trace=dict(goals_sent=0,cancelled_uuids=[],queried_uuids=[],destroyed=False,shutdown=False)
    rclpy=ModuleType('rclpy');rclpy.init=lambda **kwargs:None
    rclpy.ok=lambda:not trace['shutdown'];rclpy.spin_once=lambda *args,**kwargs:None
    rclpy.shutdown=lambda:trace.update(shutdown=True)
    signals=ModuleType('rclpy.signals');signals.SignalHandlerOptions=SimpleNamespace(NO=0)
    action=ModuleType('nav2_msgs.action')
    action.FollowWaypoints=SimpleNamespace(Impl=SimpleNamespace(GetResultService=SimpleNamespace(Request=SimpleNamespace)))
    messages=ModuleType('unique_identifier_msgs.msg');messages.UUID=lambda **kwargs:SimpleNamespace(**kwargs)
    transport=ModuleType('waypoint_batch');transport.freeze_sources=lambda *args:{}
    class Pilot:
        def __init__(self,options,output,route,source_manifest,mission):
            self.mission=mission;self.options=options;self.route=route
            self.ordered=mission.ordered(route);self.stop_evidence=sample['owned_goal_shutdown']['stop']
            self.strict_scan_latency_first_fault=None;self.scan_aging_episodes=[]
            self.latest_ready={};self.recovery_handoffs=[]
        def run_batch(self):
            if sample.get('fixture_signal_before_send'):
                from test_mission_storage import signal_before_send_batch
                return signal_before_send_batch(self,self.options,self.route,trace)
            if sample.get('fixture_unsent_start'):
                self.mission.reserve()
                return dict(status=None,classification='fixture_post_intent_health_not_restored')
            raise AssertionError('Reconciliation must never send a new goal')
        def cancel_own(self,reason):
            trace['cancelled_uuids'].append(str(uuid.UUID(bytes=self.owned_identity.value)))
        def create_client(self,service,name):
            assert name=='/follow_waypoints/_action/get_result'
            def call(request):
                trace['queried_uuids'].append(str(uuid.UUID(bytes=bytes(request.goal_id.uuid))))
                response=SimpleNamespace(status=sample['status'],result=SimpleNamespace(
                    error_code=sample.get('error_code',0),error_msg=sample.get('error_msg',''),
                    missed_waypoints=[SimpleNamespace(**v) for v in sample.get('missed_waypoints',[])]))
                return SimpleNamespace(done=lambda:True,result=lambda:response)
            return SimpleNamespace(wait_for_service=lambda **kwargs:True,call_async=call)
        def spin_until(self,predicate,timeout):return predicate()
        def execution_ready(self):
            self.latest_ready=dict(waypoint_ownership=dict(navigate_to_pose_active=[],follow_waypoints_active=[],
                follow_path_active=[],follow_path_status_publishers_match=True,exclusive_nav_client=True,checked_unix_ns=time.time_ns()))
        def finalize_own_goal(self):return copy.deepcopy(sample['owned_goal_shutdown'])
        def finish_evidence(self):return dict(evidence_complete=True)
        def destroy_node(self):trace['destroyed']=True
    transport.WaypointPilot=Pilot
    sys.modules.update({'rclpy':rclpy,'rclpy.signals':signals,'nav2_msgs.action':action,
        'unique_identifier_msgs.msg':messages,'waypoint_batch':transport})
    if os.environ.get('MATRIX_TEST_TAMPER_RESULT')=='1':
        import mission_state
        def tamper(path,value):
            atomic_json(path,value)
            if Path(path).name=='native_result.json':
                changed=copy.deepcopy(value);changed['owned_goal_shutdown']['stop']['observed']=False
                atomic_json(path,changed)
        mission_state.atomic_json=tamper
    try:runpy.run_path(str(SOURCE/'mission_resume.py'),run_name='__main__')
    finally:atomic_json(trace_path,trace)


class EvidenceBindingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.store=MissionStore(self.root/'mission.db')
        raw,identity,policy,_=fixture();self.store.create(raw,identity,policy)
        self.attempt=self.store.reserve(str(uuid.uuid4()),self.root/'attempt_01')
        self.original=observation(self.attempt['parent_action_uuid'],False)
        self.original_path=self.root/'original.json';atomic_json(self.original_path,self.original)
        self.store.finish_attempt(self.original,True,result_path=self.original_path)
    def tearDown(self):self.store.close();self.temp.cleanup()
    def test_latest_observation_and_history_keep_original_dispatch(self):
        before=self.store.state();old_raw=self.original_path.read_bytes()
        result=observation(self.attempt['parent_action_uuid'],True)
        path=self.root/'reconcile.json';atomic_json(path,result)
        self.store.finish_attempt(result,True,result_path=path)
        self.store.finish_attempt(result,True,result_path=path)
        state=self.store.export(self.root/'export');evidence=dict(path=str(path),sha256=digest(path.read_bytes()))
        self.assertEqual(state['stop']['evidence'],evidence)
        self.assertEqual(state['stop']['receipt'],json.loads(path.read_bytes())['owned_goal_shutdown']['stop'])
        self.assertEqual(state['attempt']['native_result'],evidence)
        self.assertEqual(state['attempts'][-1]['native_result_history'],[before['attempt']['native_result'],evidence])
        self.assertEqual(state['attempt']['output'],before['attempt']['output'])
        self.assertEqual(state['budgets']['mission_dispatches_reserved'],1)
        self.assertEqual(self.original_path.read_bytes(),old_raw)
    def test_missing_wrong_or_stale_evidence_cannot_change_state(self):
        before=self.store.state();result=observation(self.attempt['parent_action_uuid'],True)
        for case in ('missing','invalid_json','old_result','wrong_uuid','wrong_status','wrong_stop','wrong_children'):
            with self.subTest(case=case):
                path=self.root/(case+'.json');persisted=copy.deepcopy(result);children=True
                if case=='old_result':persisted=self.original
                if case=='wrong_uuid':persisted['action_uuid']=str(uuid.uuid4())
                if case=='wrong_status':persisted['status']=6
                if case=='wrong_stop':persisted['owned_goal_shutdown']['stop']['observed']=False
                if case=='wrong_children':children=False
                if case!='missing':atomic_json(path,persisted)
                if case=='invalid_json':path.write_text('{')
                with self.assertRaises(Hold):self.store.finish_attempt(result,children,result_path=path)
                self.assertEqual(self.store.state(),before)
    def test_reusing_recorded_path_with_changed_bytes_is_rejected(self):
        before=self.store.state();result=observation(self.attempt['parent_action_uuid'],True)
        atomic_json(self.original_path,result)
        with self.assertRaisesRegex(Hold,'reused'):
            self.store.finish_attempt(result,True,result_path=self.original_path)
        self.assertEqual(self.store.state(),before)


class ReconcileCliTests(unittest.TestCase):
    def run_cli(self,root,before,sample,operation='reconcile',tamper=False):
        db=root/'mission.db';receipt=root/'transport.json';atomic_json(receipt,sample)
        # Clone the state into an isolated offline DB; never open the live mission DB.
        with_db=MissionStore(db)
        with_db.db.execute('INSERT INTO mission VALUES (1,?,?)',(before['revision'],encoded(before)))
        with_db.close()
        args=[operation,'--execute','--store',str(db),'--resource-lock',str(root/'resource.lock'),
            '--mission-id',before['mission_id'],'--expect-revision',str(before['revision']),
            '--request-id',str(uuid.uuid4()),'--ready',str(root/'unused_ready.json'),'--output',str(root/'operation')]
        env=os.environ.copy();env.update(ROS_DOMAIN_ID='185',PYTHONDONTWRITEBYTECODE='1',MATRIX_TEST_TAMPER_RESULT='1' if tamper else '0')
        command=[sys.executable,str(Path(__file__).resolve()),'--cli-fixture',str(receipt),*args]
        process=subprocess.run(command,env=env,capture_output=True,text=True,timeout=15)
        self.assertEqual(process.returncode,10 if tamper else 11,process.stderr+process.stdout)
        trace=json.loads(receipt.with_suffix('.trace.json').read_text())
        self.assertEqual(trace['goals_sent'],0)
        self.assertEqual(trace['cancelled_uuids'],[before['attempt']['parent_action_uuid']])
        self.assertEqual(trace['queried_uuids'],trace['cancelled_uuids'])
        self.assertTrue(trace['destroyed']);self.assertTrue(trace['shutdown'])
        state=MissionStore.inspect(db)
        if tamper:
            self.assertEqual(state,before);self.assertIn('evidence differs',process.stdout);return
        response=json.loads(process.stdout);export=json.loads((root/'operation/state.json').read_text())
        result=json.loads((root/'operation/result.json').read_text())
        self.assertEqual(state,export);self.assertEqual(result['stop'],state['stop'])
        path=Path(state['stop']['evidence']['path']);native=json.loads(path.read_bytes())
        self.assertEqual(path,root/'operation'/('settle' if before['attempt']['terminal_confirmed'] else 'reconcile')/'native_result.json')
        self.assertEqual(state['stop']['evidence']['sha256'],digest(path.read_bytes()))
        self.assertEqual(state['stop']['receipt'],native['owned_goal_shutdown']['stop'])
        self.assertEqual(state['stop']['receipt'],sample['owned_goal_shutdown']['stop'])
        self.assertEqual(state['attempt']['native_result_history'],[before['attempt']['native_result'],state['stop']['evidence']])
        self.assertEqual(state['attempt']['output'],before['attempt']['output'])
        self.assertEqual(state['progress'],before['progress'])
        for key,value in before['budgets'].items():
            if key!='remaining_sec':self.assertEqual(state['budgets'][key],value,key)
        self.assertLessEqual(state['budgets']['remaining_sec'],before['budgets']['remaining_sec'])
        self.assertEqual(response['state_sha256'],digest((root/'operation/state.json').read_bytes()))
        self.assertEqual(response['result_sha256'],digest((root/'operation/result.json').read_bytes()))
        self.assertEqual(state['attempts'][:-1],before['attempts'][:-1])
        repeated=subprocess.run(command,env=env,capture_output=True,text=True,timeout=15)
        self.assertEqual(repeated.returncode,11,repeated.stderr)
        self.assertEqual(json.loads(repeated.stdout),response)
        self.assertEqual(MissionStore.inspect(db),state)
        with_db=MissionStore(db)
        try:
            with self.assertRaisesRegex(Hold,'FAILED_BUDGET'):with_db.reserve(str(uuid.uuid4()),root/'forbidden')
        finally:with_db.close()
        return state
    def synthetic_state(self,root):
        store=MissionStore(root/'seed.db');raw,identity,policy,_=fixture();store.create(raw,identity,policy)
        for i in range(3):
            attempt=store.reserve(str(uuid.uuid4()),root/('attempt_'+str(i)))
            result=observation(attempt['parent_action_uuid'],i<2);path=root/('result_'+str(i)+'.json');atomic_json(path,result)
            store.finish_attempt(result,True,result_path=path)
        state=store.state();store.close()
        # Exercise the old-state upgrade path without an existing history field.
        state['attempt'].pop('native_result_history');state['attempts'][-1]=copy.deepcopy(state['attempt'])
        return state,observation(attempt['parent_action_uuid'],True)
    def test_real_cli_reconcile_and_cancel_preserve_exhausted_budget(self):
        for operation,terminal in [('reconcile',True),('cancel',True),('reconcile',False)]:
            with self.subTest(operation=operation,terminal=terminal),tempfile.TemporaryDirectory() as directory:
                root=Path(directory);before,sample=self.synthetic_state(root)
                before['attempt']['terminal_confirmed']=terminal;before['attempts'][-1]=copy.deepcopy(before['attempt'])
                original=Path(before['attempt']['native_result']['path']);old_raw=original.read_bytes()
                self.run_cli(root,before,sample,operation)
                self.assertEqual(original.read_bytes(),old_raw)
    def test_cli_evidence_mismatch_holds_and_cleans_up(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);before,sample=self.synthetic_state(root)
            self.run_cli(root,before,sample,tamper=True)
    def test_real_a15_observations_and_cli_binding(self):
        workspace=Path(os.environ['MATRIX_EVIDENCE_ROOT']) if os.environ.get('MATRIX_EVIDENCE_ROOT') else next(
            (ancestor for ancestor in HERE.parents if (ancestor/'analysis_outputs').is_dir()),HERE)
        evidence=workspace/'analysis_outputs/planner_comparison_20261105/space_agent_recovery_20261110/astra/reconcile_evidence_fix_v1/A15_readonly'
        original=evidence/'invocation_000/client_output/attempt_01/native_result.json'
        settled=evidence/'reconcile_000/client_output/settle/native_result.json'
        if not settled.exists():self.skipTest('Read-only A15 evidence unavailable')
        self.assertEqual(digest(original.read_bytes()),'773a240166a9f81fefc5ba959aca9f19f75941428d19a2c0849df88aa9d09685')
        self.assertEqual(digest(settled.read_bytes()),'e5946a695dc559dafec8b6f61fd24f410f103151839a0ee53419832dacd8a21c')
        before=json.loads((evidence/'invocation_000/state.copy.json').read_text());sample=json.loads(settled.read_text())
        broken=json.loads((evidence/'reconcile_000/state.copy.json').read_text())
        self.assertEqual(broken['stop']['receipt'],sample['owned_goal_shutdown']['stop'])
        self.assertEqual(broken['stop']['evidence'],before['stop']['evidence'])
        self.assertFalse(json.loads(original.read_text())['owned_goal_shutdown']['stop']['observed'])
        self.assertEqual(before['budgets']['mission_dispatches_reserved'],3)
        self.assertEqual(before['progress']['accepted_prefix_count'],5)
        # Relocate only the isolated test-state reference to the sealed identical
        # bytes. Production must HOLD if its recorded historical path is missing.
        old=before['attempt']['native_result'];self.assertEqual(old['sha256'],digest(original.read_bytes()))
        relocated=dict(path=str(original),sha256=old['sha256'])
        before['attempt']['native_result']=relocated
        before['attempt']['native_result_history']=[relocated]
        before['attempts'][-1]=copy.deepcopy(before['attempt'])
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);store=MissionStore(root/'exact.db')
            store.db.execute('INSERT INTO mission VALUES (1,?,?)',(before['revision'],encoded(before)))
            try:
                store.finish_attempt(sample,True,result_path=settled)
                self.assertEqual(store.state()['stop']['evidence']['sha256'],digest(settled.read_bytes()))
                self.assertEqual(store.state()['attempt']['native_result_history'][0],before['attempt']['native_result'])
            finally:store.close()
            self.run_cli(root,before,sample)


if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--cli-fixture':cli_fixture()
    else:unittest.main()
