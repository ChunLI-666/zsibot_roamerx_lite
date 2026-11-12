"""Actual no-ROS continuation CLI, durable transactions and A15 evidence fixtures."""
import copy
import json
import os
from pathlib import Path
import runpy
import sqlite3
import subprocess
import sys
import tempfile
import time
import unittest
import uuid
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
SOURCE=HERE/'goal_pilot_source';sys.path.insert(0,str(SOURCE))
from mission_state import MissionStore, Hold, atomic_json, boot_clock, digest, encoded
from mission_continuation import CLIENT_SOURCES, main as continuation_main
from mission_resume import Session, parser
from test_mission_state import fixture as route_fixture


def reference(path):return dict(path=str(path.resolve()),sha256=digest(path.read_bytes()))

def save(root,name,value):
    path=root/name;atomic_json(path,value);return reference(path)

def stop_result(action,stopped=True):
    return dict(action_uuid=action,status=5,error_code=0,missed_waypoints=[],children_quiescent=True,
        final_measured_and_commanded_stop=stopped,owned_goal_shutdown=dict(stop=dict(observed=stopped,strict_stop_issues=[] if stopped else ['unproven'])))


def synthetic_state(root):
    raw,identity,policy,historical=route_fixture();document=json.loads(raw)
    document['mission_budget_sec']=1800
    for i,item in enumerate(document['waypoints']):item['id']='F%03d'%(11+i)
    raw=json.dumps(document).encode();policy['mission_budget_sec']=1800;identity['variant']='segmented'
    historical['route_sha256']=digest(raw);historical['finished_ns']=historical['created_ns']+140783957763
    for i,visit in enumerate(historical['ordered']['visits']):visit['id']=document['waypoints'][i]['id']
    store=MissionStore(root/'seed.db');store.create(raw,identity,policy);store.import_closed(historical,digest(historical))
    first=store.reserve(str(uuid.uuid4()),root/'attempt_00')
    with store.transaction('FIXTURE_VISIT') as s:
        s['candidates']['4']=copy.deepcopy(historical['ordered']['visits'][4]);store._promote(s,5,'fixture_verified_visit')
    result=stop_result(first['parent_action_uuid']);result.update(status=6,missed_waypoints=[dict(index=5,error_code=206)])
    path=root/'first.json';atomic_json(path,result);store.finish_attempt(result,True,result_path=path)
    second=store.reserve(str(uuid.uuid4()),root/'attempt_01');original=stop_result(second['parent_action_uuid'],False)
    path=root/'original.json';atomic_json(path,original);store.finish_attempt(original,True,result_path=path)
    state=store.state();store.close()
    # Model the sealed pre-fix export: new true stop receipt, stale false-stop reference.
    state['attempt']['stopped']=True;state['attempts'][-1]=copy.deepcopy(state['attempt'])
    state['stop'].update(state='PASSED',receipt=stop_result(second['parent_action_uuid'])['owned_goal_shutdown']['stop'])
    state['result_flags']['strict_timing_status']='FAIL'
    return state


class Fixture:
    def __init__(self,root,duration=1200.,with_b=False,actual=False):
        self.root=root;self.actual=actual
        if actual:
            workspace=Path(os.environ['MATRIX_EVIDENCE_ROOT']) if os.environ.get('MATRIX_EVIDENCE_ROOT') else next(
                (ancestor for ancestor in HERE.parents if (ancestor/'analysis_outputs').is_dir()),HERE)
            source=workspace/'analysis_outputs/planner_comparison_20261105/space_agent_recovery_20261110'
            evidence=source/'astra/reconcile_evidence_fix_v1'
            old=evidence/'A15_readonly/reconcile_000/state.copy.json'
            if not old.exists():raise unittest.SkipTest('Read-only actual A15 evidence unavailable')
            self.state=json.loads(old.read_text());(root/'old_state.json').write_bytes(old.read_bytes())
            self.old_ref=reference(root/'old_state.json')
            path=evidence/'A15_readonly/reconcile_000/client_output/settle/native_result.json'
            (root/'settled.json').write_bytes(path.read_bytes());self.stop_ref=reference(root/'settled.json')
            (root/'closure.json').write_bytes((evidence/'A15_closure_readonly.json').read_bytes())
            self.closure_ref=reference(root/'closure.json');self.closed=json.loads((root/'closure.json').read_text())
            self.design=json.loads((source/'main_continuation_design_approval.json').read_text())
            self.boot=self.design['credit_policy']['start_boot_id'];self.start=self.design['credit_policy']['start_boottime_sec']
            self.start_ns=self.closed['closure']['shutdown']['time_ns']
        else:
            self.state=synthetic_state(root);self.boot,now=boot_clock();self.start=now-1000.;self.start_ns=time.time_ns()-1000000000000
            self.state['budgets'].update(deadline_boot_id=self.boot,deadline_boottime_sec=self.start+1219.102373649,remaining_sec=1584.403899032)
            self.old_ref=save(root,'old_state.json',self.state)
            self.stop_ref=save(root,'settled.json',stop_result(self.state['attempt']['parent_action_uuid']))
            self.closed=self.closure(self.state,self.old_ref,self.start,self.start_ns)
            self.closed['client']['reconciled_native_result_sha256']=self.stop_ref['sha256']
            self.closure_ref=save(root,'closure.json',self.closed)
            self.design=dict(schema='main.continuation.design_approval.v1',decision='APPROVED_WITH_SCOPED_CREDIT_DEFINITION',
                operation='authorize-continuation',motion_in_operation=False,mission_id=self.state['mission_id'],parent_cap_after=10,
                per_index_parent_cap=3,per_index_child_slots=6,credit_policy=dict(start_boot_id=self.boot,start_boottime_sec=self.start,
                    start_evidence_sha256=self.closure_ref['sha256'],maximum_grant_sec=2400,fresh_full_1800_reset=False))
        self.end=self.start+duration;self.end_ns=self.start_ns+int(duration*1e9);self.now=self.end+.1;self.now_ns=self.end_ns+100000000
        self.identity={k:v for k,v in self.state['identity'].items() if not k.startswith('route_')}
        self.identity.update(source_sha='c'*40,verified=True,runtime_parameters_sha256='d'*64,runtime_identities_sha256='e'*64)
        self.identity_ref=save(root,'new_identity.json',self.identity)
        self.ready=dict(ready=True,time_ns=self.now_ns,valid_until_ns=self.now_ns+500000000,variant='segmented',mission_identity=self.identity,recovery_runtime=dict(verified=True),
            waypoint_contract=dict(mission_resume=dict(schema='matrix.mission.resume_capability.v1',bt_retries=1,
                client_sources_sha256={name:digest((SOURCE/name).read_bytes()) for name in CLIENT_SOURCES})),
            waypoint_ownership=dict(navigate_to_pose_active=[],follow_waypoints_active=[],follow_path_active=[],
                follow_path_status_publishers_match=True,exclusive_nav_client=True,checked_unix_ns=self.now_ns))
        self.ready_path=root/'ready.json';atomic_json(self.ready_path,self.ready)
        captured=copy.deepcopy(self.ready);captured['time_ns']=self.end_ns;captured['waypoint_ownership']['checked_unix_ns']=self.end_ns
        registration=save(root,'registry.json',dict(fixture_only=True,registered_A_clients=[],registered_A_actions=[],observation='no ROS calls'))
        self.interval=dict(schema='matrix.mission.inactive_interval.v1',mission_id=self.state['mission_id'],boot_id=self.boot,
            start_boottime_sec=self.start,end_boottime_sec=self.end,end_unix_ns=self.end_ns,
            start_closure_sha256=self.closure_ref['sha256'],accounting_scope='cooperative_single_owner',
            registered_a_execution_clients=[],registered_a_action_uuids=[],registration_evidence=[registration],
            registered_b_mission_ids=[],b_runs=[],new_identity_sha256=self.identity_ref['sha256'],end_ready=save(root,'end_ready.json',captured))
        if with_b:
            b=copy.deepcopy(self.state);b['mission_id']=str(uuid.uuid4());b['variant']='official';b['identity'].update(variant='official',source_sha='b'*40)
            for a in b['attempts']:a['parent_action_uuid']=str(uuid.uuid4())
            b['attempt']=copy.deepcopy(b['attempts'][-1]);b_ref=save(root,'B_state.json',b)
            bs=self.start+10.;be=self.end-10.;bc=self.closure(b,b_ref,be,self.end_ns-10000000000)
            self.interval['registered_b_mission_ids']=[b['mission_id']]
            self.interval['b_runs']=[dict(state=b_ref,closure=save(root,'B_closure.json',bc),source_sha='b'*40,start_boottime_sec=bs,end_boottime_sec=be)]
        design_ref=save(root,'design.json',self.design)
        if actual:
            # Supplemental approval binds the exact original file bytes, not a reserialized JSON copy.
            (root/'design.json').write_bytes((source/'main_continuation_design_approval.json').read_bytes())
            design_ref=reference(root/'design.json')
        self.authorization=dict(schema='matrix.mission.continuation_authorization.v1',approved_by='Main',decision='APPROVED',fixture_only=True,
            authorization_id=str(uuid.uuid4()),mission_id=self.state['mission_id'],expected_revision=self.state['revision'],
            expected_state_sha256=digest(self.state),old_identity_epoch_id=self.state['identity_epoch_id'],
            old_identity_sha256=digest(self.state['identity']),old_policy_sha256=digest(self.state['policy']),new_identity_epoch_id=str(uuid.uuid4()),
            new_source_sha=self.identity['source_sha'],new_identity=self.identity_ref,old_state_snapshot=self.old_ref,
            design_approval=design_ref,new_policy=dict(self.state['policy'],max_parent_dispatches=10,max_parent_dispatches_per_index=3),
            stop_result=self.stop_ref,closure=self.closure_ref,inactive_interval=save(root,'interval.json',self.interval),
            credit=dict(boot_id=self.boot,start_boottime_sec=self.start,end_boottime_sec=self.end,grant_sec=min(duration,2400.)))
        self.authorization_path=root/'TEST_ONLY_authorization.json';self.refresh()
        self.db=root/'mission.db';self.store=MissionStore(self.db)
        self.store.db.execute('INSERT INTO mission VALUES (1,?,?)',(self.state['revision'],encoded(self.state)))
        self.request=str(uuid.uuid4())
    @staticmethod
    def closure(state,state_ref,boottime,unix_ns):
        run=uuid.uuid4().hex
        return dict(schema='terra.recording.exact_closure.v1',complete=True,run_id=run,
            client=dict(mission_id=state['mission_id'],revision=state['revision'],action_uuid=state['attempt']['parent_action_uuid'],state_sha256=state_ref['sha256']),
            closure=dict(shutdown=dict(run_id=run,stage='recording_closed',reason='all_owned_processes_waited',boottime_sec=boottime,time_ns=unix_ns),
            returncodes={name:0 for name in ['recorder','nav_safety_gate','matrix_zero_handoff','planner_server','controller_server','bt_navigator','waypoint_follower','velocity_smoother','lifecycle_manager']}))
    def refresh(self):
        atomic_json(self.authorization_path,self.authorization);self.sha=digest(self.authorization_path.read_bytes())
    def clock(self):
        from contextlib import ExitStack
        stack=ExitStack();stack.enter_context(patch('mission_state.boot_clock',return_value=(self.boot,self.now)))
        stack.enter_context(patch('mission_continuation.boot_clock',return_value=(self.boot,self.now)))
        stack.enter_context(patch('time.time_ns',return_value=self.now_ns));return stack
    def apply(self):
        self.refresh()
        with self.clock():return self.store.authorize_continuation(self.authorization,self.sha,self.ready_path)
    def args(self,output=None,execute=True):
        self.refresh();self.store.close()
        return ['authorize-continuation',*(['--execute'] if execute else []),'--store',str(self.db),'--resource-lock',str(self.root/'resource.lock'),
            '--mission-id',self.state['mission_id'],'--expect-revision',str(self.authorization['expected_revision']),
            '--request-id',self.request,'--authorization',str(self.authorization_path),'--authorization-sha256',self.sha,
            '--identity',str(self.root/'new_identity.json'),'--ready',str(self.ready_path),'--output',str(output or self.root/'output')]
    def cli(self,args):
        clock=self.root/'clock.json';atomic_json(clock,dict(boot=self.boot,boottime=self.now,unix_ns=self.now_ns))
        return subprocess.run([sys.executable,str(Path(__file__).resolve()),'--cli-clock',str(clock),*args],capture_output=True,text=True,timeout=15)


class ContinuationTests(unittest.TestCase):
    def setUp(self):self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
    def tearDown(self):self.temp.cleanup()
    def v3_supplement(self,f,maximum=14400):
        value=dict(schema='main.continuation.design_supplement.v3',
            decision='APPROVED_BOUNDED_A_INACTIVE_CREDIT_CAP_EXTENSION',mission_id=f.state['mission_id'],
            previous_approval=f.authorization['design_approval'],maximum_grant_sec=maximum,
            original_navigation_budget_sec=1800,actual_grant_not_yet_issued=True)
        f.authorization['design_supplement']=save(f.root,'supplement_v3.json',value)
        f.authorization['credit']['grant_sec']=min(f.end-f.start,maximum)
        return value
    def test_v3_cap_from_approval_preserves_exact_elapsed_and_history(self):
        for maximum,duration in ((14400,1200.125),(14400,14400.),(14400,15000.125),
                (86400,86400.),(1500.25,1800.125),(.25,.125),(1,1.)):
            with self.subTest(maximum=maximum,duration=duration),tempfile.TemporaryDirectory() as directory:
                f=Fixture(Path(directory),duration=duration,with_b=duration>20);self.v3_supplement(f,maximum)
                try:
                    receipt=f.apply();s=receipt['state'];grant=min(f.end-f.start,maximum)
                    self.assertEqual(s['budgets']['deadline_boottime_sec'],f.state['budgets']['deadline_boottime_sec']+grant)
                    self.assertEqual(s['budgets']['authorized_inactive_credit_sec'],grant)
                    self.assertAlmostEqual(s['budgets']['remaining_sec'],
                        f.state['budgets']['deadline_boottime_sec']-f.start-(f.now-f.end)-max(0,f.end-f.start-maximum))
                    self.assertNotEqual(s['budgets']['deadline_boottime_sec'],f.now+1800)
                    for key in ('mission_id','route','progress','current_target','result_flags'):
                        self.assertEqual(s[key],f.state[key])
                    for key,value in f.state['budgets'].items():
                        if key not in ('deadline_boottime_sec','remaining_sec'):self.assertEqual(s['budgets'][key],value)
                    self.assertEqual(s['policy']['mission_budget_sec'],1800)
                    self.assertEqual(receipt['transition']['credit']['maximum_sec'],maximum)
                    self.assertEqual(receipt['transition']['evidence']['design_supplement'],f.authorization['design_supplement'])
                    self.assertEqual(f.apply(),receipt)
                finally:f.store.close()
    def test_v3_rejects_malformed_or_out_of_range_caps_before_mutation(self):
        cases=[None,True,False,'14400',0,-1,86400.0001,10**1000,float('nan'),float('inf'),float('-inf'),'missing','exponent']
        for maximum in cases:
            with self.subTest(maximum=str(maximum)[:30]),tempfile.TemporaryDirectory() as directory:
                f=Fixture(Path(directory));value=self.v3_supplement(f)
                try:
                    value['maximum_grant_sec']=maximum
                    if maximum=='missing':value.pop('maximum_grant_sec')
                    raw=json.dumps(value)
                    if maximum=='exponent':raw=raw.replace('"exponent"','1e309')
                    path=f.root/'invalid_cap.json';path.write_text(raw)
                    f.authorization['design_supplement']=reference(path)
                    with self.assertRaises(Hold):f.apply()
                    self.assertEqual(f.store.state(),f.state)
                    self.assertEqual(f.store.db.execute('SELECT count(*) FROM continuations').fetchone()[0],0)
                    self.assertEqual(f.store.db.execute('SELECT count(*) FROM events').fetchone()[0],0)
                finally:f.store.close()
    def test_v3_cannot_bypass_authority_interval_or_separate_B_accounting(self):
        cases=['schema','decision','mission','approval','budget','issued','sha','over_grant','under_grant','boot',
            'future_end','registered_A','missing_B','B_source','state','revision','source','active_goal']
        for case in cases:
            with self.subTest(case=case),tempfile.TemporaryDirectory() as directory:
                f=Fixture(Path(directory),duration=7000.,with_b=True);value=self.v3_supplement(f)
                try:
                    if case=='schema':value['schema']='main.continuation.design_supplement.v4'
                    if case=='decision':value['decision']='PENDING'
                    if case=='mission':value['mission_id']=str(uuid.uuid4())
                    if case=='approval':value['previous_approval']={'sha256':'0'*64}
                    if case=='budget':value['original_navigation_budget_sec']=14400
                    if case=='issued':value['actual_grant_not_yet_issued']=False
                    f.authorization['design_supplement']=save(f.root,'changed_v3.json',value)
                    if case=='sha':f.authorization['design_supplement']['sha256']='0'*64
                    if case=='over_grant':f.authorization['credit']['grant_sec']+=.25
                    if case=='under_grant':f.authorization['credit']['grant_sec']-=.25
                    if case=='boot':f.authorization['credit']['boot_id']=str(uuid.uuid4())
                    if case=='future_end':f.authorization['credit']['end_boottime_sec']=f.now+1
                    if case=='registered_A':f.interval['registered_a_execution_clients']=['A-active']
                    if case=='missing_B':f.interval['b_runs']=[]
                    if case=='B_source':f.interval['b_runs'][0]['source_sha']='0'*40
                    f.authorization['inactive_interval']=save(f.root,'changed_interval.json',f.interval)
                    if case=='state':f.authorization['expected_state_sha256']='0'*64
                    if case=='revision':f.authorization['expected_revision']+=1
                    if case=='source':f.authorization['new_source_sha']='0'*40
                    if case=='active_goal':
                        f.ready['waypoint_ownership']['follow_path_active']=['active'];atomic_json(f.ready_path,f.ready)
                    with self.assertRaises(Hold):f.apply()
                    self.assertEqual(f.store.state(),f.state)
                finally:f.store.close()
    def test_v3_equal_or_expired_extended_deadline_still_fails(self):
        for duration,delay in ((14400.,0.),(14400.,.25),(16000.,None)):
            with self.subTest(duration=duration,delay=delay),tempfile.TemporaryDirectory() as directory:
                f=Fixture(Path(directory),duration=duration);self.v3_supplement(f)
                try:
                    if delay is not None:
                        f.now=f.state['budgets']['deadline_boottime_sec']+14400+delay
                        f.now_ns=f.end_ns+round((f.now-f.end)*1e9)
                        f.ready.update(time_ns=f.now_ns,valid_until_ns=f.now_ns+500000000)
                        f.ready['waypoint_ownership']['checked_unix_ns']=f.now_ns;atomic_json(f.ready_path,f.ready)
                    with self.assertRaisesRegex(Hold,'FAILED_BUDGET'):f.apply()
                    self.assertEqual(f.store.state(),f.state)
                    self.assertEqual(f.store.db.execute('SELECT count(*) FROM continuations').fetchone()[0],0)
                finally:f.store.close()
    def test_v3_transaction_rollback_retry_and_second_credit(self):
        f=Fixture(self.root,duration=7000.);self.v3_supplement(f)
        try:
            f.store.db.execute("CREATE TRIGGER fail_v3 BEFORE INSERT ON continuations BEGIN SELECT RAISE(ABORT,'fixture failure'); END")
            with self.assertRaises(sqlite3.DatabaseError):f.apply()
            self.assertEqual(f.store.state(),f.state)
            self.assertEqual(f.store.db.execute('SELECT count(*) FROM events').fetchone()[0],0)
            f.store.db.execute('DROP TRIGGER fail_v3');first=f.apply();after=f.store.state()
            self.assertEqual(f.apply(),first)
            f.authorization['credit']['grant_sec']+=1
            with self.assertRaisesRegex(Hold,'different bytes'):f.apply()
            self.assertEqual(f.store.state(),after)
            f.authorization.update(authorization_id=str(uuid.uuid4()),expected_revision=after['revision'],expected_state_sha256=digest(after))
            with self.assertRaisesRegex(Hold,'already applied'):f.apply()
            self.assertEqual(f.store.state(),after)
        finally:f.store.close()
    def test_v3_committed_authorization_recovers_after_export_failure(self):
        f=Fixture(self.root,duration=7000.);self.v3_supplement(f)
        try:
            args=parser().parse_args(f.args())
            with f.clock(),patch('mission_continuation.write_once',side_effect=OSError('fixture export interruption')):
                with self.assertRaises(OSError):continuation_main(args)
            committed=MissionStore.inspect(f.db)
            process=f.cli(f.args());self.assertEqual(process.returncode,0,process.stderr+process.stdout)
            self.assertEqual(MissionStore.inspect(f.db),committed)
            self.assertEqual(json.loads((self.root/'output/state.json').read_text()),committed)
        finally:f.store.close()
    def test_actual_A15_v3_approval_cli_preserves_history_and_last_F016_parent(self):
        f=Fixture(self.root,duration=7000.,actual=True)
        try:
            workspace=Path(os.environ['MATRIX_EVIDENCE_ROOT']) if os.environ.get('MATRIX_EVIDENCE_ROOT') else next(
                (a for a in HERE.parents if (a/'analysis_outputs').is_dir()),HERE)
            source=workspace/'analysis_outputs/planner_comparison_20261105/space_agent_recovery_20261110/main_continuation_design_supplement_v3.json'
            if not source.exists():self.skipTest('Actual Main v3 approval unavailable')
            raw=source.read_bytes();copied=self.root/'real_v3.json';copied.write_bytes(raw)
            maximum=json.loads(raw)['maximum_grant_sec']
            f.authorization['design_supplement']=reference(copied)
            grant=min(f.end-f.start,maximum);f.authorization['credit']['grant_sec']=grant
            args=f.args(execute=False);before=f.db.read_bytes();result=f.cli(args)
            self.assertEqual(result.returncode,0,result.stderr);self.assertEqual(f.db.read_bytes(),before)
            self.assertFalse((self.root/'output').exists())
            result=f.cli(f.args());self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(json.loads(result.stdout)['goals_sent'],0)
            s=MissionStore.inspect(f.db)
            for key in ('mission_id','route','progress','current_target','result_flags'):self.assertEqual(s[key],f.state[key])
            self.assertEqual(s['budgets']['deadline_boottime_sec'],f.state['budgets']['deadline_boottime_sec']+grant)
            self.assertEqual(s['budgets']['mission_dispatches_reserved'],3)
            self.assertEqual(s['budgets']['per_index_parent_dispatches'][5],2)
            self.assertEqual(s['budgets']['per_index_reserved'][5],4)
            self.assertEqual(s['result_flags']['strict_timing_status'],'FAIL')
            hashes={p.name:digest(p.read_bytes()) for p in (self.root/'output').iterdir()}
            repeated=f.cli(f.args());self.assertEqual(repeated.returncode,0,repeated.stderr)
            self.assertEqual(repeated.stdout,result.stdout);self.assertEqual(MissionStore.inspect(f.db),s)
            self.assertEqual({p.name:digest(p.read_bytes()) for p in (self.root/'output').iterdir()},hashes)
            f.store=MissionStore(f.db)
            with f.clock():
                Session(f.store,str(uuid.uuid4()),self.root/'next').validate_ready(f.ready)
                attempt=f.store.reserve(str(uuid.uuid4()),self.root/'next')
                self.assertEqual(attempt['start_index'],5)
                closed=stop_result(attempt['parent_action_uuid']);path=self.root/'next_result.json';atomic_json(path,closed)
                f.store.finish_attempt(closed,True,result_path=path)
                with self.assertRaisesRegex(Hold,'FAILED_BUDGET'):f.store.reserve(str(uuid.uuid4()),self.root/'forbidden')
            self.assertEqual(f.store.state()['budgets']['per_index_parent_dispatches'][5],3)
            self.assertEqual(f.store.state()['budgets']['per_index_reserved'][5],6)
            self.assertEqual(source.read_bytes(),raw)
        finally:f.store.close()
    def supplement(self,f):
        value=dict(schema='main.continuation.design_supplement.v2',
            decision='APPROVED_BOUNDED_A_INACTIVE_CREDIT_CAP_EXTENSION',mission_id=f.state['mission_id'],
            previous_approval=f.authorization['design_approval'],maximum_grant_sec=5400,
            original_navigation_budget_sec=1800,actual_grant_not_yet_issued=True)
        f.authorization['design_supplement']=save(f.root,'supplement.json',value)
        f.authorization['credit']['grant_sec']=min(f.end-f.start,5400.)
        return value
    def test_supplemented_cap_preserves_elapsed_and_is_idempotent(self):
        for duration in (3000.,5400.,6000.,7000.):
            with self.subTest(duration=duration),tempfile.TemporaryDirectory() as directory:
                f=Fixture(Path(directory),duration=duration,with_b=True);self.supplement(f)
                try:
                    if duration==7000.:
                        with self.assertRaisesRegex(Hold,'FAILED_BUDGET'):f.apply()
                        self.assertEqual(f.store.state(),f.state)
                    else:
                        receipt=f.apply();state=receipt['state'];grant=min(duration,5400.)
                        self.assertEqual(state['budgets']['authorized_inactive_credit_sec'],grant)
                        self.assertEqual(state['budgets']['deadline_boottime_sec'],f.state['budgets']['deadline_boottime_sec']+grant)
                        self.assertEqual(state['progress'],f.state['progress']);self.assertEqual(state['result_flags'],f.state['result_flags'])
                        self.assertEqual(receipt['transition']['credit']['maximum_sec'],5400)
                        self.assertEqual(receipt['transition']['evidence']['design_supplement'],f.authorization['design_supplement'])
                        self.assertEqual(f.store.authorize_continuation(f.authorization,f.sha,f.ready_path),receipt)
                finally:f.store.close()
    def test_supplement_cannot_change_authority_budget_or_grant_without_observed_time(self):
        for case in ['missing','wrong_parent','wrong_mission','wrong_cap','reset_budget','decision','over_grant','sha','extra_second_credit']:
            with self.subTest(case=case),tempfile.TemporaryDirectory() as directory:
                f=Fixture(Path(directory),duration=3000.);supplement=self.supplement(f)
                try:
                    if case=='missing':f.authorization.pop('design_supplement')
                    if case=='wrong_parent':supplement['previous_approval']={'sha256':'0'*64}
                    if case=='wrong_mission':supplement['mission_id']=str(uuid.uuid4())
                    if case=='wrong_cap':supplement['maximum_grant_sec']=5401
                    if case=='reset_budget':supplement['original_navigation_budget_sec']=7200
                    if case=='decision':supplement['decision']='PENDING'
                    if case=='over_grant':f.authorization['credit']['grant_sec']=5400
                    if case in ['wrong_parent','wrong_mission','wrong_cap','reset_budget','decision']:
                        f.authorization['design_supplement']=save(f.root,'changed_supplement.json',supplement)
                    if case=='sha':f.authorization['design_supplement']['sha256']='0'*64
                    if case=='extra_second_credit':
                        f.apply();f.authorization['authorization_id']=str(uuid.uuid4())
                    before=f.store.state()
                    with self.assertRaises(Hold):f.apply()
                    self.assertEqual(f.store.state(),before)
                finally:f.store.close()
    def test_actual_A15_supplemented_production_cli_keeps_real_history(self):
        f=Fixture(self.root,duration=6000.,actual=True)
        try:
            workspace=Path(os.environ['MATRIX_EVIDENCE_ROOT']) if os.environ.get('MATRIX_EVIDENCE_ROOT') else next(
                (a for a in HERE.parents if (a/'analysis_outputs').is_dir()),HERE)
            source=workspace/'analysis_outputs/planner_comparison_20261105/space_agent_recovery_20261110/main_continuation_design_supplement_v2.json'
            if not source.exists():self.skipTest('Actual Main supplemental approval unavailable')
            copied=self.root/'real_supplement.json';copied.write_bytes(source.read_bytes())
            f.authorization['design_supplement']=reference(copied);f.authorization['credit']['grant_sec']=5400.
            result=f.cli(f.args());self.assertEqual(result.returncode,0,result.stderr)
            state=json.loads((self.root/'output/state.json').read_text())
            self.assertEqual(state['progress'],f.state['progress']);self.assertEqual(state['current_target'],f.state['current_target'])
            self.assertEqual(state['budgets']['deadline_boottime_sec'],f.state['budgets']['deadline_boottime_sec']+5400)
            self.assertEqual(state['budgets']['mission_dispatches_reserved'],3)
            self.assertEqual(state['budgets']['per_index_parent_dispatches'][5],2)
            self.assertEqual(state['budgets']['per_index_reserved'][5],4)
            self.assertEqual(state['result_flags']['strict_timing_status'],'FAIL')
        finally:f.store.close()
    def test_atomic_epoch_preserves_progress_consumption_and_attempt_provenance(self):
        f=Fixture(self.root,with_b=True)
        try:
            receipt=f.apply();s=f.store.state();old=f.state
            self.assertEqual(receipt['before_state_sha256'],digest(old));self.assertEqual(receipt['after_state_sha256'],digest(s))
            self.assertEqual(receipt['state'],s);self.assertEqual(s['progress'],old['progress']);self.assertEqual(s['result_flags'],old['result_flags'])
            self.assertEqual(s['current_target'],old['current_target']);self.assertEqual(s['mission_id'],old['mission_id'])
            for key,value in old['budgets'].items():
                if key not in ('deadline_boottime_sec','remaining_sec'):self.assertEqual(s['budgets'][key],value,key)
            self.assertEqual(s['budgets']['deadline_boottime_sec'],old['budgets']['deadline_boottime_sec']+1200)
            self.assertAlmostEqual(s['budgets']['remaining_sec'],s['budgets']['deadline_boottime_sec']-f.now)
            self.assertEqual(s['policy']['max_parent_dispatches'],10);self.assertEqual(s['budgets']['mission_dispatches_reserved'],3)
            self.assertEqual(s['identity_policy_epochs'][0]['identity'],old['identity'])
            self.assertEqual(s['identity_policy_epochs'][0]['policy'],old['policy'])
            for a in s['attempts']:self.assertEqual(a['identity_epoch_id'],old['identity_epoch_id'])
            self.assertEqual(s['attempts'][0]['source_provenance']['kind'],'imported_closed_result')
            self.assertNotIn('source_sha',s['attempts'][0]['source_provenance'])
            for a in s['attempts'][1:]:self.assertEqual(a['source_provenance']['source_sha'],old['identity']['source_sha'])
            self.assertEqual(s['stop']['evidence'],f.stop_ref);self.assertIn(old['attempt']['native_result'],s['attempt']['native_result_history'])
            self.assertEqual(json.loads((self.root/'old_state.json').read_text()),old)
            self.assertEqual(f.store.authorize_continuation(f.authorization,f.sha,f.ready_path),receipt)
            self.assertEqual(f.store.state(),s)
        finally:f.store.close()
    def test_credit_cap_and_expired_deadline_do_not_reset_clock(self):
        for duration in (3000.,4000.):
            with self.subTest(duration=duration),tempfile.TemporaryDirectory() as d:
                f=Fixture(Path(d),duration=duration)
                try:
                    if duration==4000.:
                        with self.assertRaisesRegex(Hold,'FAILED_BUDGET'):f.apply()
                        self.assertEqual(f.store.state(),f.state)
                    else:
                        s=f.apply()['state'];self.assertEqual(s['budgets']['authorized_inactive_credit_sec'],2400)
                        self.assertAlmostEqual(s['budgets']['remaining_sec'],619.002373649,places=5)
                        self.assertNotEqual(s['budgets']['deadline_boottime_sec'],f.now+1800)
                finally:f.store.close()
    def test_rejects_wrong_authority_state_identity_policy_stop_and_credit(self):
        cases=['authority','mission','revision','state_hash','old_source','new_epoch','map','config','source','policy_reset','per_index_reset',
            'stop_uuid','stop_false','stop_sha','closure','boot','future_end','over_credit','interval_A','missing_B','B_source','stale_ready','expired_ready','active_goal','client_source','missing_audit']
        for case in cases:
            with self.subTest(case=case),tempfile.TemporaryDirectory() as d:
                f=Fixture(Path(d),with_b=True);a=f.authorization
                try:
                    if case=='authority':a['approved_by']='another'
                    if case=='mission':a['mission_id']=str(uuid.uuid4())
                    if case=='revision':a['expected_revision']+=1
                    if case=='state_hash':a['expected_state_sha256']='0'*64
                    if case=='old_source':a['old_identity_sha256']='0'*64
                    if case=='new_epoch':a['new_identity_epoch_id']=f.state['identity_epoch_id']
                    if case in ('map','config','source'):
                        key={'map':'map_contract_sha256','config':'effective_config_sha256','source':'source_sha'}[case]
                        identity=copy.deepcopy(f.identity);identity[key]='0'*len(identity[key]);a['new_identity']=save(f.root,'new_identity.json',identity)
                    if case=='policy_reset':a['new_policy']['mission_budget_sec']=1801
                    if case=='per_index_reset':a['new_policy']['max_attempts_per_index']=8
                    if case in ('stop_uuid','stop_false'):
                        result=json.loads((f.root/'settled.json').read_text())
                        if case=='stop_uuid':result['action_uuid']=str(uuid.uuid4())
                        else:result['owned_goal_shutdown']['stop']['observed']=False
                        a['stop_result']=save(f.root,'settled.json',result)
                    if case=='stop_sha':a['stop_result']['sha256']='0'*64
                    if case=='closure':a['closure']['sha256']='0'*64
                    if case=='boot':a['credit']['boot_id']=str(uuid.uuid4())
                    if case=='future_end':a['credit']['end_boottime_sec']=f.now+1
                    if case=='over_credit':a['credit']['grant_sec']+=1
                    if case in ('interval_A','missing_B','B_source','missing_audit'):
                        interval=copy.deepcopy(f.interval)
                        if case=='interval_A':interval['registered_a_action_uuids']=[str(uuid.uuid4())]
                        if case=='missing_B':interval['b_runs']=[]
                        if case=='B_source':interval['b_runs'][0]['source_sha']='0'*40
                        if case=='missing_audit':interval['registration_evidence']=[]
                        a['inactive_interval']=save(f.root,'interval.json',interval)
                    if case in ('stale_ready','expired_ready','active_goal','client_source'):
                        ready=copy.deepcopy(f.ready)
                        if case=='stale_ready':ready['time_ns']-=1000000000
                        if case=='expired_ready':ready['valid_until_ns']=f.now_ns
                        if case=='active_goal':ready['waypoint_ownership']['follow_path_active']=['active']
                        if case=='client_source':ready['waypoint_contract']['mission_resume']['client_sources_sha256'].pop('mission_continuation.py')
                        atomic_json(f.ready_path,ready)
                    with self.assertRaises((Hold,ValueError)):f.apply()
                    self.assertEqual(f.store.state(),f.state)
                    self.assertEqual(f.store.db.execute('SELECT count(*) FROM continuations').fetchone()[0],0)
                    self.assertEqual(f.store.db.execute('SELECT count(*) FROM events').fetchone()[0],0)
                finally:f.store.close()
    def test_transaction_failure_rolls_back_state_event_and_credit_receipt(self):
        f=Fixture(self.root)
        try:
            f.store.db.execute("CREATE TRIGGER fail_receipt BEFORE INSERT ON continuations BEGIN SELECT RAISE(ABORT,'fixture failure'); END")
            with self.assertRaises(sqlite3.DatabaseError):f.apply()
            self.assertEqual(f.store.state(),f.state)
            self.assertEqual(f.store.db.execute('SELECT count(*) FROM events').fetchone()[0],0)
            f.store.db.execute('DROP TRIGGER fail_receipt');f.apply()
            self.assertEqual(f.store.db.execute('SELECT count(*) FROM continuations').fetchone()[0],1)
        finally:f.store.close()
    def test_duplicate_and_second_authorization_cannot_grant_twice(self):
        f=Fixture(self.root)
        try:
            first=f.apply();after=f.store.state()
            f.authorization['credit']['grant_sec']+=1
            with self.assertRaisesRegex(Hold,'different bytes'):f.apply()
            self.assertEqual(f.store.state(),after)
            f.authorization['authorization_id']=str(uuid.uuid4());f.authorization['expected_revision']=after['revision']
            f.authorization['expected_state_sha256']=digest(after)
            with self.assertRaisesRegex(Hold,'already applied'):f.apply()
            self.assertEqual(f.store.state(),after)
        finally:f.store.close()
    def test_normal_session_checks_identity_and_enforces_last_F016_parent(self):
        f=Fixture(self.root)
        try:
            f.apply()
            with f.clock():
                Session(f.store,str(uuid.uuid4()),self.root/'next').validate_ready(f.ready)
                old_ready=copy.deepcopy(f.ready);old_ready['mission_identity']=dict(f.state['identity'],verified=True)
                with self.assertRaisesRegex(Hold,'IDENTITY_MISMATCH'):Session(f.store,str(uuid.uuid4()),self.root).validate_ready(old_ready)
                attempt=f.store.reserve(str(uuid.uuid4()),self.root/'next')
                self.assertEqual(attempt['start_index'],5);self.assertEqual(attempt['identity_epoch_id'],f.authorization['new_identity_epoch_id'])
                self.assertEqual(attempt['source_provenance']['source_sha'],'c'*40)
                result=stop_result(attempt['parent_action_uuid']);path=self.root/'next_result.json';atomic_json(path,result)
                f.store.finish_attempt(result,True,result_path=path)
                self.assertEqual(f.store.state()['budgets']['mission_dispatches_reserved'],4)
                self.assertEqual(f.store.state()['budgets']['per_index_parent_dispatches'][5],3)
                self.assertEqual(f.store.state()['budgets']['per_index_reserved'][5],6)
                with self.assertRaisesRegex(Hold,'FAILED_BUDGET'):f.store.reserve(str(uuid.uuid4()),self.root/'forbidden')
        finally:f.store.close()
    def test_real_cli_dry_run_commit_and_idempotent_frozen_exports(self):
        f=Fixture(self.root,with_b=True)
        try:
            args=f.args(execute=False);before=f.db.read_bytes();process=f.cli(args)
            self.assertEqual(process.returncode,0,process.stderr+process.stdout);self.assertEqual(f.db.read_bytes(),before)
            self.assertFalse((self.root/'output').exists())
            args=f.args();process=f.cli(args);self.assertEqual(process.returncode,0,process.stderr+process.stdout)
            response=json.loads(process.stdout);state=MissionStore.inspect(f.db);self.assertEqual(response['goals_sent'],0)
            output=self.root/'output';hashes={p.name:digest(p.read_bytes()) for p in output.iterdir()}
            repeated=f.cli(args);self.assertEqual(repeated.returncode,0,repeated.stderr+repeated.stdout)
            self.assertEqual(json.loads(repeated.stdout),response);self.assertEqual(MissionStore.inspect(f.db),state)
            self.assertEqual({p.name:digest(p.read_bytes()) for p in output.iterdir()},hashes)
            self.assertEqual(response['state_sha256'],digest((output/'state.json').read_bytes()))
            self.assertEqual(response['result_sha256'],digest((output/'result.json').read_bytes()))
            receipt=json.loads((output/'continuation_receipt.json').read_text());self.assertEqual(receipt['state'],state)
            self.assertEqual(receipt['after_state_sha256'],digest(state))
        finally:f.store.close()
    def test_committed_authorization_recovers_after_export_failure(self):
        f=Fixture(self.root)
        try:
            args=parser().parse_args(f.args())
            with f.clock(),patch('mission_continuation.write_once',side_effect=OSError('fixture export interruption')):
                with self.assertRaises(OSError):continuation_main(args)
            committed=MissionStore.inspect(f.db)
            process=f.cli(f.args());self.assertEqual(process.returncode,0,process.stderr+process.stdout)
            self.assertEqual(MissionStore.inspect(f.db),committed)
            self.assertEqual(json.loads((self.root/'output/state.json').read_text()),committed)
        finally:f.store.close()
    def test_real_A15_sealed_evidence_through_cli(self):
        f=Fixture(self.root,actual=True)
        try:
            self.assertEqual(f.stop_ref['sha256'],'e5946a695dc559dafec8b6f61fd24f410f103151839a0ee53419832dacd8a21c')
            self.assertEqual(f.closure_ref['sha256'],'285be8e71b1ce3ac77c8ff369f5b16b8436f4e23b4deb19a5b089288c2b65267')
            process=f.cli(f.args());self.assertEqual(process.returncode,0,process.stderr+process.stdout)
            state=MissionStore.inspect(f.db)
            self.assertEqual(state['mission_id'],'fb3b2004-7dfe-4ca6-b215-7023ea992026')
            self.assertEqual(state['progress'],f.state['progress']);self.assertEqual(state['budgets']['mission_dispatches_reserved'],3)
            self.assertEqual(state['budgets']['per_index_parent_dispatches'][5],2)
            self.assertEqual(state['budgets']['per_index_reserved'][5],4)
            self.assertEqual(state['result_flags']['strict_timing_status'],'FAIL')
            self.assertEqual(state['stop']['evidence'],f.stop_ref)
            self.assertEqual(state['attempt']['native_result_history'][0],f.state['attempt']['native_result'])
            self.assertEqual(json.loads((self.root/'old_state.json').read_text()),f.state)
        finally:f.store.close()


def cli_clock():
    clock=json.loads(Path(sys.argv[2]).read_text());sys.argv=[str(SOURCE/'mission_resume.py'),*sys.argv[3:]]
    with patch('mission_state.boot_clock',return_value=(clock['boot'],clock['boottime'])), \
         patch('mission_continuation.boot_clock',return_value=(clock['boot'],clock['boottime'])), \
         patch('time.time_ns',return_value=clock['unix_ns']):
        try:runpy.run_path(str(SOURCE/'mission_resume.py'),run_name='__main__')
        finally:
            if 'rclpy' in sys.modules:raise AssertionError('Continuation CLI imported ROS')

if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--cli-clock':cli_clock()
    else:unittest.main()
