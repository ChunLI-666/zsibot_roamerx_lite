"""Offline mission transactions and normal CLI behavior. No ROS or motion."""
import contextlib,copy,io,json,sys,tempfile,unittest,uuid
from pathlib import Path
from unittest.mock import patch
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'goal_pilot_source'))
from mission_state import MissionStore,Hold,digest,atomic_json
from mission_resume import main,Session
from test_native_waypoints import snapshot
from waypoint_contract import AdvancingStop

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
        for _ in range(2):
            a=self.store.reserve(str(uuid.uuid4()),self.root);result['action_uuid']=a['parent_action_uuid'];self.finish(result,True)
        self.assertEqual(self.store.state()['budgets']['per_index_reserved'][0],6)
        self.assertEqual(self.store.state()['budgets']['per_index_parent_dispatches'][0],3)
        with self.assertRaisesRegex(Hold,'FAILED_BUDGET'):self.store.reserve(str(uuid.uuid4()),self.root)
    def test_failure_classification_is_persisted_by_terminal_entry(self):
        a=self.store.reserve(str(uuid.uuid4()),self.root)
        result=dict(action_uuid=a['parent_action_uuid'],status=6,error_code=0,
            final_measured_and_commanded_stop=True,missed_waypoints=[dict(index=0,error_code=208)])
        self.finish(result,True)
        self.assertEqual(self.store.state()['lifecycle'],'NEEDS_TASK_DECISION')
        with self.assertRaisesRegex(Hold,'Conflicting'):
            self.finish(dict(result,status=5),True)
        a=self.store.reserve(str(uuid.uuid4()),self.root)
        result['action_uuid']=a['parent_action_uuid']
        result['cancel_reason']='input_became_invalid:scan_old';result['missed_waypoints']=[];result['status']=5
        self.finish(result,True)
        self.assertEqual(self.store.state()['lifecycle'],'PAUSED_INPUT_HEALTH')
        result['stop_signals']=[2];self.finish(result,True)
        self.assertEqual(self.store.state()['lifecycle'],'CANCELED')
        self.finish(result,False)
        self.assertEqual(self.store.state()['lifecycle'],'HOLD')
    def test_runtime_identity_is_checked_against_mission(self):
        session=Session(self.store,str(uuid.uuid4()),self.root)
        source=HERE/'goal_pilot_source'
        ready=dict(variant='official',recovery_runtime=dict(verified=True),mission_identity=dict(self.identity,verified=True),
            waypoint_contract=dict(mission_resume=dict(schema='matrix.mission.resume_capability.v1',bt_retries=1,
            client_sources_sha256={n:digest((source/n).read_bytes()) for n in ('mission_state.py','mission_resume.py','mission_continuation.py')})))
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
if __name__=='__main__':unittest.main()
