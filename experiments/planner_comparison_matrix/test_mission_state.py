"""Offline mission transactions and normal CLI behavior. No ROS or motion."""
import contextlib,copy,io,json,math,sys,tempfile,time,unittest,uuid
from pathlib import Path
from unittest.mock import patch
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'goal_pilot_source'))
from mission_state import MissionStore,Hold,digest,atomic_json
from mission_resume import main,Session
from test_native_waypoints import snapshot
from waypoint_contract import AdvancingStop,evidence_json,angular_difference

def fixture():
    document=dict(schema_version=1,frame_id='map',mission_budget_sec=100.,waypoints=[
        dict(id='F%02d'%i,pose=[0.,0.,0.],require_stopped=True) for i in range(7)])
    raw=json.dumps(document).encode()
    identity=dict(variant='official',source_sha='a'*40,**{k:'b'*64 for k in
        ['map_contract_sha256','footprint_sha256','acceptance_policy_sha256','effective_config_sha256']})
    policy=dict(mission_budget_sec=100.,max_parent_dispatches=3,max_attempts_per_index=6,bt_retries=1)
    visits=[dict(index=i,id=w['id'],original_goal_map=w['pose'],passed=dict(snapshot=snapshot(),
        continuous_stopped_s=.31,advancing_odom_dwell={'source_first_ns':1,'source_last_ns':310000001}))
        for i,w in enumerate(document['waypoints'])]
    result=dict(action_uuid=str(uuid.uuid4()),status=6,error_code=0,missed_waypoints=[dict(index=4,error_code=106)],
        final_measured_and_commanded_stop=True,route_sha256=digest(raw),ordered=dict(visits=visits),
        created_ns=1000000000,finished_ns=11000000000)
    return raw,identity,policy,result

class MissionTests(unittest.TestCase):
    def test_typed_stopped_recovery_codes_preserve_unknown_zero_boundary(self):
        from mission_state import failure_decision
        cases={60001:'RECOVERABLE_FAILURE',60002:'PAUSED_INPUT_HEALTH',
            60003:'HOLD',60004:'NEEDS_TASK_DECISION',60005:'RECOVERABLE_FAILURE',
            0:'NEEDS_TASK_DECISION',60100:'NEEDS_TASK_DECISION'}
        for code,expected in cases.items():
            result=dict(status=6,error_code=0,missed_waypoints=[dict(index=0,error_code=code)])
            self.assertEqual(failure_decision(result,True,True,True)[0],expected)
            self.assertEqual(failure_decision(result,True,True,False)[0],'HOLD')

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.store=MissionStore(self.root/'state.db',self.root/'resource.lock')
        self.raw,self.identity,self.policy,self.historical=fixture()
        self.store.create(self.raw,self.identity,self.policy)
    def tearDown(self):self.store.close();self.temp.cleanup()
    def finish(self,result,children):
        result=dict(result,children_quiescent=children)
        path=self.root/(str(uuid.uuid4())+'.json');atomic_json(path,result)
        self.store.finish_attempt(result,children,result_path=path)
    def test_import_prefix_four_and_full_absolute_resume(self):
        self.store.import_closed(self.historical,digest(self.historical))
        session=Session(self.store,str(uuid.uuid4()),self.root/'attempt')
        self.assertEqual(session.start_index,4)
        tracker=session.ordered(self.store.state()['route'])
        self.assertEqual(len(tracker.visits),7)
        self.assertTrue(all(v['passed'] for v in tracker.visits[:4]))
        self.assertIsNone(tracker.visits[4]['passed'])
        tracker.feedback(4,1,1);self.assertIsNone(tracker.violation)
        self.assertEqual(self.store.state()['budgets']['per_index_reserved'],[2,2,2,2,2,0,0])
        self.assertEqual(self.store.remaining(),90.)
    def test_duplicate_geometry_does_not_accept_a_later_index(self):
        self.store.import_closed(self.historical,digest(self.historical))
        tracker=Session(self.store,str(uuid.uuid4()),self.root).ordered(self.store.state()['route'])
        tracker.feedback(5,1,1);self.assertIsNotNone(tracker.violation)
        self.assertEqual(self.store.state()['progress']['next_index'],4)
    def test_send_intent_survives_restart_and_blocks_duplicate(self):
        first=self.store.reserve(str(uuid.uuid4()),self.root)
        self.store.close();self.store=MissionStore(self.root/'state.db',self.root/'resource.lock')
        with self.assertRaisesRegex(Hold,'UNKNOWN_DISPATCH'):self.store.reserve(str(uuid.uuid4()),self.root)
        self.assertEqual(self.store.state()['attempt']['parent_action_uuid'],first['parent_action_uuid'])
    def test_transaction_rollback_and_resource_lock(self):
        revision=self.store.state()['revision']
        with self.assertRaises(RuntimeError):
            with self.store.transaction('test') as s:s['progress']['next_index']=5;raise RuntimeError()
        self.assertEqual(self.store.state()['revision'],revision)
        self.assertEqual(self.store.state()['progress']['next_index'],0)
        with self.assertRaises(BlockingIOError):MissionStore(self.root/'other.db',self.root/'resource.lock')
    def test_parent_error_zero_does_not_complete_and_refund_once(self):
        a=self.store.reserve(str(uuid.uuid4()),self.root)
        result=dict(action_uuid=a['parent_action_uuid'],status=6,error_code=0,missed_waypoints=[dict(index=0,error_code=106)],
            final_measured_and_commanded_stop=True)
        self.finish(result,True);self.finish(result,True)
        self.assertFalse(self.store.state()['result_flags']['operational_complete'])
        self.assertEqual(self.store.state()['budgets']['per_index_reserved'],[2,0,0,0,0,0,0])
        for number in range(2):
            request=str(uuid.uuid4())
            if number==1:
                self.assertEqual(self.store.state()['hold_reason'],'REPEATED_FAILURE_REVIEW_REQUIRED')
                self.store.review_retry(request,'Reviewed original failure for the third budgeted attempt')
            a=self.store.reserve(request,self.root);result['action_uuid']=a['parent_action_uuid'];self.finish(result,True)
        self.assertEqual(self.store.state()['budgets']['per_index_reserved'][0],6)
        self.assertEqual(self.store.state()['budgets']['per_index_parent_dispatches'][0],3)
        request=str(uuid.uuid4())
        with self.assertRaisesRegex(Hold,'FAILED_BUDGET'):
            self.store.review_retry(request,'A review does not reset the exhausted original budget')
            self.store.reserve(request,self.root)
    def test_failure_classification_is_persisted_by_terminal_entry(self):
        a=self.store.reserve(str(uuid.uuid4()),self.root)
        result=dict(action_uuid=a['parent_action_uuid'],status=6,error_code=0,
            final_measured_and_commanded_stop=True,missed_waypoints=[dict(index=0,error_code=208)])
        self.finish(result,True)
        self.assertEqual(self.store.state()['lifecycle'],'NEEDS_TASK_DECISION')
        with self.assertRaisesRegex(Hold,'Conflicting'):
            self.finish(dict(result,status=5),True)
        with self.assertRaises(Hold):self.store.reserve(str(uuid.uuid4()),self.root)
        with self.assertRaises(Hold):self.store.review_retry(str(uuid.uuid4()),'Other task decisions need their own authority')
        for name,children,signals,expected in [('input',True,[],'PAUSED_INPUT_HEALTH'),
                ('operator',True,[2],'CANCELED'),('unproven_child',False,[],'HOLD')]:
            self.store.close();self.store=MissionStore(self.root/(name+'.db'),self.root/'resource.lock')
            self.store.create(self.raw,self.identity,self.policy)
            a=self.store.reserve(str(uuid.uuid4()),self.root)
            observation=dict(action_uuid=a['parent_action_uuid'],status=5,error_code=0,missed_waypoints=[],
                final_measured_and_commanded_stop=True,cancel_reason='input_became_invalid:scan_old',stop_signals=signals)
            self.finish(observation,children);self.assertEqual(self.store.state()['lifecycle'],expected)
            if expected in ('CANCELED','HOLD'):
                with self.assertRaises(Hold):self.store.reserve(str(uuid.uuid4()),self.root)
    def test_runtime_identity_is_checked_against_mission(self):
        session=Session(self.store,str(uuid.uuid4()),self.root)
        source=HERE/'goal_pilot_source'
        ready=dict(variant='official',recovery_runtime=dict(verified=True),mission_identity=dict(self.identity,verified=True),
            waypoint_contract=dict(mission_resume=dict(schema='matrix.mission.resume_capability.v1',bt_retries=1,
            client_sources_sha256={n:digest((source/n).read_bytes()) for n in ('mission_state.py','mission_resume.py','mission_continuation.py','mission_storage.py')})))
        session.validate_ready(ready)
        for key in ('source_sha','effective_config_sha256','map_contract_sha256','footprint_sha256'):
            altered=copy.deepcopy(ready);altered['mission_identity'][key]='c'*len(self.identity[key])
            with self.assertRaisesRegex(Hold,'IDENTITY_MISMATCH'):session.validate_ready(altered)
    def test_advancing_500hz_and_duplicate_sample(self):
        stop=AdvancingStop()
        for i in range(160):
            s=snapshot();s['last']['odom'].update(stamp_ns=1000000000+i*2000000,received_unix_ns=i)
            passed=stop.observe(s,1000000000+i*2000000,True)
        self.assertTrue(passed)
        self.assertFalse(stop.observe(s,2000000000,True))
        s['last']['odom']['received_unix_ns']+=1
        self.assertFalse(stop.observe(s,2000000001,True))

class CliTests(unittest.TestCase):
    def test_start_resume_import_idempotency_and_unknown_dispatch(self):
        for operation in ('start','import-closed-attempt'):
            with self.subTest(operation=operation),tempfile.TemporaryDirectory() as d:
                root=Path(d);raw,identity,policy,historical=fixture()
                (root/'route.json').write_bytes(raw)
                closure=dict(schema='matrix.mission.closed_runtime.v1',action_uuid=historical['action_uuid'],
                    parent_terminal_status=historical['status'],all_owned_processes_terminated=True,recorder_closed=True,
                    checked_unix_ns=historical['finished_ns'])
                for name,value in [('identity',identity),('policy',policy),('historical',historical),('closure',closure)]:
                    (root/(name+'.json')).write_text(json.dumps(value))
                manifest=dict(result_path=str(root/'historical.json'),result_sha256=digest((root/'historical.json').read_bytes()),
                    evidence_files=[dict(path=str(root/'closure.json'),sha256=digest((root/'closure.json').read_bytes()))])
                manifest.update(schema='matrix.mission.closed_attempt.v1',historical_bt_retries=1,authorized_new_source_sha=identity['source_sha'],
                    closure=manifest['evidence_files'][0],**{k:identity[k] for k in ('map_contract_sha256','footprint_sha256','acceptance_policy_sha256')})
                (root/'manifest.json').write_text(json.dumps(manifest))
                args=[operation,'--execute','--store',str(root/'mission.db'),'--resource-lock',str(root/'lock'),
                    '--route',str(root/'route.json'),'--identity',str(root/'identity.json'),'--policy',str(root/'policy.json'),
                    '--request-id',str(uuid.uuid4()),'--output',str(root/'out'),'--ready',str(root/'ready.json'),
                    '--closed-attempt-manifest',str(root/'manifest.json')]
                def dispatch(args,store,output,reconcile=False):
                    if reconcile:raise Hold('unknown server result')
                    store.reserve(args.request_id,output)
                    return dict(goals_sent=1)
                with patch('mission_resume.run_attempt',side_effect=dispatch) as run,contextlib.redirect_stdout(io.StringIO()) as captured:
                    code=main(args);calls=run.call_count;repeated=main(args)
                lines=captured.getvalue().splitlines()
                self.assertEqual(json.loads(lines[0]),json.loads(lines[1]));self.assertEqual(code,repeated)
                self.assertEqual(run.call_count,calls)
                state=MissionStore.inspect(root/'mission.db')
                resume=['resume','--execute','--store',str(root/'mission.db'),'--resource-lock',str(root/'lock'),
                    '--mission-id',state['mission_id'],'--expect-revision',str(state['revision']),
                    '--request-id',str(uuid.uuid4()),'--output',str(root/'resume'),'--ready',str(root/'ready.json')]
                if operation=='start':
                    with patch('mission_resume.run_attempt',side_effect=dispatch) as run:
                        with self.assertRaisesRegex(Hold,'unknown'):main(resume)
                        with contextlib.redirect_stdout(io.StringIO()) as out:self.assertEqual(main(resume),10)
                        self.assertIn('interrupted',out.getvalue());self.assertEqual(run.call_count,1)
                    self.assertEqual(MissionStore.inspect(root/'mission.db')['budgets']['mission_dispatches_reserved'],1)
                else:
                    with patch('mission_resume.run_attempt',side_effect=dispatch) as run,contextlib.redirect_stdout(io.StringIO()) as out:
                        self.assertEqual(main(resume),10);self.assertEqual(main(resume),10)
                        self.assertEqual(run.call_count,1)
                    self.assertEqual(json.loads(out.getvalue().splitlines()[0]),json.loads(out.getvalue().splitlines()[1]))
def residual_protocol_fixture(store,root,mutate=None):
    """Five original indices, real public Store calls, explicit protocol evidence.

    These finite snapshots prove schema/transaction behavior only. Normal CLI
    tests independently produce the same evidence from real received DDS inputs.
    """
    from types import SimpleNamespace
    raw,identity,policy,example=fixture();document=json.loads(raw)
    document['waypoints']=document['waypoints'][:5];document['mission_budget_sec']=300.
    raw=json.dumps(document).encode();policy.update(mission_budget_sec=300.,max_parent_dispatches=10,max_parent_dispatches_per_index=3)
    store.create(raw,identity,policy);attempt=store.reserve(str(uuid.uuid4()),root/'source_attempt')
    store.acknowledge(attempt['parent_action_uuid'],True)
    visits=copy.deepcopy(example['ordered']['visits'][:5])
    for index,visit in enumerate(visits):
        visit['entered']=dict(steady_ns=1_000_000+index,unix_ns=1_000_000+index,
            basis='explicit protocol fixture original feedback entry')
    store.observe(attempt['parent_action_uuid'],SimpleNamespace(active=4,visits=visits))
    state=store.state();assert state['progress']['accepted_prefix_count']==4
    pose=snapshot(heading=.12,stamp=1_370_000_000)
    pose['last']['odom']['stamp_ns']=1_370_000_000
    pose['last']['pose'].update(raw_pose_values_finite=True,raw_position=dict(x=0.,y=0.,z=0.),
        raw_quaternion=dict(x=0.,y=0.,z=math.sin(.12/2),w=math.cos(.12/2)))
    observation=dict(schema='matrix.stop_callback_observation.v1',settled=True,callback_invalidations=0,
        started_steady_ns=1_000_000_000,finished_steady_ns=1_370_000_000,deadline_steady_ns=2_000_000_000,
        advancing_odom_dwell=dict(zero_stamp=1_000_000_000,first_stamp=1_050_000_000,last_stamp=1_360_000_000,
            first_steady=1_050_000_000,last_steady=1_360_000_000))
    stop=dict(observed=True,strict_stop_issues=[],snapshot=copy.deepcopy(pose),callback_observation=observation)
    binding=dict(action_uuid=attempt['parent_action_uuid'],identity_epoch_id=state['identity_epoch_id'],
        original_index=4,original_goal_map=state['route'][4]['pose'],stop_callback_invalidations=0)
    predicates=dict(execution_healthy=True,not_cancelled=True,native_success=True,ordered_complete=True,
        measured_stop=True,stop_continuity_current=True,final_pose_fresh=True,final_xy_within=True,final_yaw_within=False)
    result=dict(action_uuid=attempt['parent_action_uuid'],status=4,error_code=0,error_msg='',missed_waypoints=[],
        functional_status='FAIL',children_quiescent=True,final_measured_and_commanded_stop=True,
        settled_measured_and_commanded_stop=True,settled_within_original_tolerance=False,
        ordered=dict(all_visited=True,violation=None,visits=visits),evidence_writer=dict(evidence_complete=True),
        final_snapshot=copy.deepcopy(pose),owned_goal_shutdown=dict(stop=copy.deepcopy(stop)),
        final_goal_validation=dict(schema='matrix.mission.final_goal_validation.v1',predicates=predicates,**binding),
        current_final_goal_observation=dict(schema='matrix.mission.current_final_goal.v1',healthy=True,issues=[],
            stop_continuity_current=True,snapshot=copy.deepcopy(pose),**binding),
        strict_scan_timing_status='FAIL',created_ns=attempt['created_ns'],finished_ns=time.time_ns())
    if mutate is not None:mutate(result)
    # Production evidence preserves invalid raw numerics as tagged valid JSON;
    # atomic_json intentionally rejects a fixture's direct bare NaN instead.
    result=json.loads(evidence_json(result))
    path=root/('source_result_'+str(uuid.uuid4())+'.json');atomic_json(path,result)
    store.finish_attempt(result,True,result_path=path)
    return attempt,result,path,visits


class FinalResidualPredicateTests(unittest.TestCase):
    def scenario(self,mutate=None):
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup);root=Path(temporary.name)
        store=MissionStore(root/'state.db');self.addCleanup(store.close)
        attempt,result,path,visits=residual_protocol_fixture(store,root,mutate)
        return store,attempt,result,path

    def test_complete_predicates_and_typed_stop_proof_qualify_without_rewriting_native_authority(self):
        store,attempt,result,path=self.scenario();s=store.state();correction=s['final_goal_correction']
        self.assertEqual((s['lifecycle'],s['hold_reason']),('RECOVERABLE_FAILURE','FINAL_GOAL_RESIDUAL_ERROR'))
        self.assertEqual(s['progress']['accepted_prefix_count'],4);self.assertEqual(s['progress']['next_index'],4)
        self.assertEqual(correction['source_attempt_id'],attempt['attempt_id'])
        self.assertEqual(correction['source_action_uuid'],attempt['parent_action_uuid'])
        self.assertIsNone(correction['consumed_attempt_id']);self.assertEqual(s['result_flags']['strict_timing_status'],'FAIL')
        self.assertEqual(s['attempt']['native_status'],4)
        self.assertEqual(s['attempt']['terminal_authority']['native']['status'],4)
        self.assertEqual(json.loads(path.read_text())['missed_waypoints'],[])

    def test_post_pass_motion_last_sample_and_phase_evolve_without_changing_the_stable_visit_proof(self):
        def mutate(r):
            v=r['ordered']['visits'][4];v['motion']=dict(samples=43,signed_yaw_rad=.12)
            v['last_sample']=dict(stamp_ns=1_370_000_000,yaw=.12);v['phase']='observed_after_original_pass'
        store,attempt,result,path=self.scenario(mutate);s=store.state()
        self.assertIsNotNone(s['final_goal_correction'])
        self.assertEqual(s['candidates']['4']['passed'],result['ordered']['visits'][4]['passed'])
        self.assertNotEqual(s['candidates']['4'].get('last_sample'),result['ordered']['visits'][4]['last_sample'])

    def test_any_changed_complete_pass_entry_or_original_identity_rejects_the_qualification(self):
        for fault in ('passed_extra','passed_dwell','entered_changed','entered_missing','entered_zero','id','index','goal'):
            def mutate(r):
                v=r['ordered']['visits'][4]
                if fault=='passed_extra':v['passed']['extra_proof_field']='changed'
                elif fault=='passed_dwell':v['passed']['continuous_stopped_s']=.32
                elif fault=='entered_changed':v['entered']['unix_ns']+=1
                elif fault=='entered_missing':v.pop('entered')
                elif fault=='entered_zero':v['entered']['steady_ns']=0
                elif fault=='id':v['id']='wrong-original-id'
                elif fault=='index':v['index']=3
                elif fault=='goal':v['original_goal_map']=[.001,0.,0.]
            with self.subTest(fault=fault):self.assertIsNone(self.scenario(mutate)[0].state()['final_goal_correction'])

    def test_original_yaw_interval_and_xy_boundary(self):
        # The approved envelope applies to the canonical normalized value.
        # raw .16 normalizes to .16000000000000003 on this Python/libm and is
        # conservatively outside; the adjacent inside sample is tested exactly.
        inside=math.nextafter(.16,0.)
        self.assertLessEqual(abs(angular_difference(inside,0.)),.16)
        self.assertGreater(abs(angular_difference(.16,0.)),.16)
        for value,allowed in [(.08,False),(.080001,True),(inside,True),(.16,False),(.160001,False)]:
            def mutate(r):
                for pose in [r['final_snapshot']['last']['pose'],r['current_final_goal_observation']['snapshot']['last']['pose']]:
                    pose['yaw']=value;pose['raw_quaternion'].update(z=math.sin(value/2),w=math.cos(value/2))
            with self.subTest(yaw=value):
                s=self.scenario(mutate)[0].state();self.assertEqual(s['final_goal_correction'] is not None,allowed)
        for value,allowed in [(.15,True),(.150001,False)]:
            def mutate(r):
                for pose in [r['final_snapshot']['last']['pose'],r['current_final_goal_observation']['snapshot']['last']['pose']]:
                    pose['x']=value;pose['raw_position']['x']=value
            with self.subTest(xy=value):self.assertEqual(self.scenario(mutate)[0].state()['final_goal_correction'] is not None,allowed)

    def test_missing_stop_counter_short_original_or_steady_dwell_and_raw_z_nan_never_qualify(self):
        for fault in ('proof_counter','current_counter','source_dwell','steady_dwell','raw_z_original','raw_z_current','unhealthy','writer'):
            def mutate(r):
                if fault=='proof_counter':r['final_goal_validation'].pop('stop_callback_invalidations')
                elif fault=='current_counter':r['current_final_goal_observation'].pop('stop_callback_invalidations')
                elif fault in ('source_dwell','steady_dwell'):
                    d=r['owned_goal_shutdown']['stop']['callback_observation']['advancing_odom_dwell']
                    d['last_stamp' if fault=='source_dwell' else 'last_steady']=1_349_999_999
                elif fault.startswith('raw_z_'):
                    s=r['final_snapshot'] if fault.endswith('original') else r['current_final_goal_observation']['snapshot']
                    s['last']['pose']['raw_position']['z']=math.nan
                elif fault=='unhealthy':r['final_goal_validation']['predicates']['execution_healthy']=False
                elif fault=='writer':r['evidence_writer']['evidence_complete']=False
            with self.subTest(fault=fault):
                s=self.scenario(mutate)[0].state();self.assertIsNone(s['final_goal_correction'])
                self.assertEqual(s['progress']['accepted_prefix_count'],4)

    def test_current_uuid_epoch_payload_and_predicates_are_independent_requirements(self):
        for fault in ('uuid','epoch','frame','safe_nan','current_health','another_predicate'):
            def mutate(r):
                c=r['current_final_goal_observation']
                if fault=='uuid':c['action_uuid']=str(uuid.uuid4())
                elif fault=='epoch':c['identity_epoch_id']='wrong-epoch'
                elif fault=='frame':c['snapshot']['last']['pose']['frame_id']='odom'
                elif fault=='safe_nan':c['snapshot']['last']['/cmd_vel_safe']['wz']=math.nan
                elif fault=='current_health':c['healthy']=False;c['issues']=['pose:bad_current_payload']
                elif fault=='another_predicate':r['final_goal_validation']['predicates']['ordered_complete']=False
            with self.subTest(fault=fault):self.assertIsNone(self.scenario(mutate)[0].state()['final_goal_correction'])


if __name__=='__main__':unittest.main()
