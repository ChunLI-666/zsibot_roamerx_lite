"""Real sealed A16 cause/stop separation and fail-closed history validation."""
import copy,json,os,subprocess,sys,tempfile,unittest,uuid
from pathlib import Path
from test_mission_state import fixture
from test_mission_reconcile_evidence import observation
from mission_state import MissionStore,Hold,atomic_json,digest,encoded

HERE=Path(__file__).resolve().parent
class CauseTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.store=MissionStore(self.root/'mission.db');raw,identity,policy,_=fixture()
        self.store.create(raw,identity,policy);a=self.store.reserve(str(uuid.uuid4()),self.root/'attempt')
        self.original=observation(a['parent_action_uuid'],False)
        self.original.update(cancel_reason='input_became_invalid:scan:invalid_header_age',finished_ns=10,strict_scan_timing_status='FAIL')
        self.path=self.root/'original.json';atomic_json(self.path,self.original)
        self.store.finish_attempt(self.original,True,result_path=self.path)
    def tearDown(self):self.store.close();self.temp.cleanup()
    def finish(self,changes=None):
        result=observation(self.original['action_uuid'],True);result.update(finished_ns=20,reconciliation=True)
        result.update(changes or {});path=self.root/(str(uuid.uuid4())+'.json');atomic_json(path,result)
        self.store.finish_attempt(result,True,result_path=path);return path
    def test_stop_uses_new_file_and_cause_uses_original_without_rewriting_either(self):
        before=self.store.state();raw=self.path.read_bytes();latest=self.finish();s=self.store.state()
        self.assertEqual(s['lifecycle'],'PAUSED_INPUT_HEALTH')
        self.assertEqual(s['attempt']['terminal_authority']['evidence'],before['attempt']['native_result'])
        self.assertEqual(s['stop']['evidence']['path'],str(latest))
        self.assertIsNone(json.loads(latest.read_text()).get('cancel_reason'))
        self.assertEqual(self.path.read_bytes(),raw)
        self.assertEqual(s['progress'],before['progress'])
        self.assertEqual(s['budgets']['per_index_reserved'],before['budgets']['per_index_reserved'])
        self.assertEqual(s['result_flags']['strict_timing_status'],'FAIL')
    def test_missing_tampered_cross_uuid_conflict_and_regressed_history_hold(self):
        before=self.store.state();raw=self.path.read_bytes()
        for case in ['missing','tampered','uuid','status','time']:
            with self.subTest(case=case):
                changes={}
                if case=='missing':self.path.unlink()
                if case=='tampered':self.path.write_text('{}')
                if case=='uuid':changes['action_uuid']=str(uuid.uuid4())
                if case=='status':changes['status']=6
                if case=='time':changes['finished_ns']=1
                with self.assertRaises(Hold):self.finish(changes)
                self.assertEqual(self.store.state(),before)
                self.path.write_bytes(raw)
    def test_explicit_operator_cancel_takes_priority_without_changing_original_cause(self):
        self.finish(dict(operator_cancel_requested=True));s=self.store.state()
        self.assertEqual(s['lifecycle'],'CANCELED')
        self.assertTrue(s['attempt']['terminal_authority']['cancel_reason'].startswith('input_became_invalid:'))
    def test_unknown_cancel_remains_hold(self):
        self.finish();a=self.store.reserve(str(uuid.uuid4()),self.root/'next')
        r=observation(a['parent_action_uuid'],True);p=self.root/'unknown.json';atomic_json(p,r)
        self.store.finish_attempt(r,True,result_path=p)
        self.assertEqual(self.store.state()['hold_reason'],'UNKNOWN_CANCEL_CAUSE')
    def test_immutable_attempt_epoch_binding_cannot_be_reused(self):
        with self.store.transaction('FIXTURE') as s:s['attempt']['identity_epoch_id']=str(uuid.uuid4())
        before=self.store.state()
        with self.assertRaisesRegex(Hold,'binding'):self.finish()
        self.assertEqual(self.store.state(),before)

class A16Tests(unittest.TestCase):
    def test_sealed_a16_real_store_and_real_cli_reconciliation(self):
        ws=Path(os.environ.get('MATRIX_EVIDENCE_ROOT','/home/charles/project/colcon_ws'))
        p=ws/'analysis_outputs/planner_comparison_20261105/space_agent_recovery_20261110/terra/A16_evidence_snapshot_v1/run/stages/mission_execute'
        original=p/'invocation_000/client_output/attempt_03/native_result.json';later=p/'reconcile_000/client_output/settle/native_result.json'
        if not original.exists():self.skipTest('sealed A16 unavailable')
        self.assertEqual(digest(original.read_bytes()),'3f35c1493903826dcbcfdaa1f1a71bd6e9a5d94b13bd9b5c3d8835984c248c36')
        self.assertEqual(digest(later.read_bytes()),'a4f77a3842c1f0f2617f823443cf4c29260f5cbb1cd102c15b4efdc3a2d7c065')
        before=json.loads((p/'invocation_000/client_output/state.json').read_text());sample=json.loads(later.read_text())
        ref=dict(path=str(original),sha256=digest(original.read_bytes()))
        before['attempt']['native_result']=ref;before['attempt']['native_result_history']=[ref]
        before['attempts'][-1]=copy.deepcopy(before['attempt'])
        for cli in [False,True]:
            with self.subTest(cli=cli),tempfile.TemporaryDirectory() as d:
                root=Path(d);db=root/'mission.db';store=MissionStore(db)
                store.db.execute('INSERT INTO mission VALUES (1,?,?)',(before['revision'],encoded(before)))
                if not cli:
                    store.finish_attempt(sample,True,result_path=later);store.finish_attempt(sample,True,result_path=later)
                    store.close()
                else:
                    store.close();transport=root/'transport.json';atomic_json(transport,sample)
                    cmd=[sys.executable,str(HERE/'test_mission_reconcile_evidence.py'),'--cli-fixture',str(transport),'reconcile','--execute','--store',str(db),
                        '--resource-lock',str(root/'resource.lock'),'--mission-id',before['mission_id'],'--expect-revision',str(before['revision']),
                        '--request-id',str(uuid.uuid4()),'--ready',str(root/'unused.json'),'--output',str(root/'operation')]
                    process=subprocess.run(cmd,env=dict(os.environ,ROS_DOMAIN_ID='185'),text=True,capture_output=True,timeout=15)
                    self.assertEqual(process.returncode,11,process.stderr+process.stdout)
                    self.assertEqual(json.loads(transport.with_suffix('.trace.json').read_text())['goals_sent'],0)
                state=MissionStore.inspect(db)
                self.assertEqual(state['lifecycle'],'PAUSED_INPUT_HEALTH');self.assertEqual(state['progress'],before['progress'])
                self.assertEqual(state['progress']['accepted_prefix_count'],3);self.assertEqual(state['current_target']['id'],'F018')
                for k in ['mission_dispatches_reserved','per_index_reserved','per_index_parent_dispatches']:
                    self.assertEqual(state['budgets'][k],before['budgets'][k])
                self.assertEqual(state['attempt']['terminal_authority']['evidence'],ref)
                self.assertEqual(state['attempt']['native_status'],5)
                self.assertEqual(state['result_flags']['strict_timing_status'],'FAIL')
                self.assertTrue(state['attempt']['stopped'])
                self.assertEqual(digest(original.read_bytes()),ref['sha256'])
if __name__=='__main__':unittest.main()
