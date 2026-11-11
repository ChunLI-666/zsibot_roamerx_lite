"""Private file-only protocol runs; actual accepted receipts, no ROS or server."""
import contextlib,copy,fcntl,hashlib,importlib.util,io,json,os,subprocess,sys,tempfile,time,unittest,uuid
from pathlib import Path
from unittest.mock import patch
OUT=Path(os.environ['P6_TEST_OUTPUT']).resolve();OUT.mkdir(exist_ok=False,parents=True)
CANDIDATE=Path(os.environ['P6_CANDIDATE']).resolve();BUSINESS=Path(json.loads(CANDIDATE.read_text())['entrypoints']['workflow_driver.py']).parent
sys.path.insert(0,str(BUSINESS));import workflow_driver as old_driver
import workflow_worker

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
TOOL=load('p6_retry_tool',os.environ['P6_TOOL']);FUTURE=load('p6_future_driver',os.environ['P6_FUTURE_DRIVER'])
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def encoded(x):return (json.dumps(x,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False)+'\n').encode()
class InterruptedForTest(RuntimeError):pass
class P6FileRetry(unittest.TestCase):
    def fixture(self,name):
        root=Path(tempfile.mkdtemp(prefix=name+'_',dir=OUT));run=root/'run';plan=json.loads(Path(os.environ['P6_PLAN']).read_text());plan['workflow_run_id']='P6_PRIVATE_'+uuid.uuid4().hex
        plan['input_binding']['candidate']=dict(path=str(CANDIDATE),sha256=sha(CANDIDATE));path=root/'plan.json';path.write_bytes(encoded(plan))
        with contextlib.redirect_stdout(io.StringIO()):old_driver.init(path,run)
        state=json.loads((run/'workflow_state.json').read_text());state['approval_sha256']=sha(CANDIDATE)
        for row in state['stages']:row['state']='SUCCEEDED'
        target=next(r for r in state['stages'] if r['id']=='mission_execute');target.update(state='HELD',reason='preparation failed: Private domain185 required')
        actor=workflow_worker.identity(os.getpid());next(r for r in state['stages'] if r['id']=='navigation_recording')['actor_identity']=actor
        old_driver.atomic_json(run/'workflow_state.json',state);empty=run/'stages/mission_execute/invocation_000';empty.mkdir(parents=True)
        (run/'nav').mkdir();deadline=dict(run_id='P6_PRIVATE_RECORDING',launcher=actor,deadline_boottime_sec=time.clock_gettime(time.CLOCK_BOOTTIME)+600.)
        (run/'nav/recording_deadline.json').write_bytes(encoded(deadline))
        permit=dict(schema='matrix.workflow.unstarted_preparation_retry.v1',decision='APPROVED',tool_sha256=sha(TOOL.__file__),candidate=dict(path=str(CANDIDATE),sha256=sha(CANDIDATE)),run_dir=str(run),plan_sha256=state['plan_sha256'],workflow_run_id=state['workflow_run_id'],before_state_sha256=sha(run/'workflow_state.json'),stage_id='mission_execute',request_id=target['request_id'],held_reason=target['reason'],recording_deadline=dict(path=str(run/'nav/recording_deadline.json'),sha256=sha(run/'nav/recording_deadline.json')),recording_run_id=deadline['run_id'],launcher=actor,minimum_remaining_sec=30.,fixture_boundary='Synthetic operator permit for this private test directory only; real accepted code/software refs are unmodified')
        p=root/'permit.json';p.write_bytes(encoded(permit));return dict(root=root,run=run,plan=plan,state=state,permit=permit,path=p,empty=empty,deadline=deadline)
    def refresh(self,f):
        old_driver.atomic_json(f['run']/'workflow_state.json',f['state']);f['permit']['before_state_sha256']=sha(f['run']/'workflow_state.json')
        (f['run']/'nav/recording_deadline.json').write_bytes(encoded(f['deadline']));f['permit']['recording_deadline']['sha256']=sha(f['run']/'nav/recording_deadline.json');f['path'].write_bytes(encoded(f['permit']))
    def retry(self,f,execute=True):
        with patch.dict(os.environ,ROS_DOMAIN_ID='185'),patch.object(subprocess,'Popen',side_effect=AssertionError('Retry must not spawn any process')):
            return TOOL.retry(f['path'],execute)
    def check_applied(self,f,before):
        state=json.loads((f['run']/'workflow_state.json').read_text());row=next(r for r in state['stages'] if r['id']=='mission_execute')
        self.assertEqual(row['state'],'PENDING');self.assertEqual(row['request_id'],f['permit']['request_id']);self.assertEqual(len(row['preparation_retry_history']),1)
        self.assertEqual((f['run']/'execution_plan.json').read_bytes(),encoded(f['plan']));self.assertEqual(sha(CANDIDATE),f['permit']['candidate']['sha256'])
        audit=f['run']/'preparation_retries'/sha(f['path']);self.assertEqual((audit/'before_state.json').read_bytes(),before);self.assertTrue((audit/'empty_invocation_000').is_dir());self.assertFalse(f['empty'].exists());self.assertFalse((f['run']/'mission.sqlite').exists())
    def reject(self,f,pattern=None):
        before=(f['run']/'workflow_state.json').read_bytes()
        if pattern:
            with self.assertRaisesRegex((ValueError,BlockingIOError),pattern):self.retry(f)
        else:
            with self.assertRaises((ValueError,BlockingIOError)):self.retry(f)
        self.assertEqual((f['run']/'workflow_state.json').read_bytes(),before)
    def test_dryrun_then_real_requeue_preserves_original_identity_and_never_spawns(self):
        f=self.fixture('positive');before=(f['run']/'workflow_state.json').read_bytes();self.assertEqual(self.retry(f,False)['status'],'DRY_RUN');self.assertEqual((f['run']/'workflow_state.json').read_bytes(),before);self.assertTrue(f['empty'].is_dir())
        result=self.retry(f);self.assertEqual(result['status'],'APPLIED');self.assertFalse(result['starts_worker']);self.check_applied(f,before)
    def test_same_permit_is_idempotent_even_after_original_driver_progress_and_missing_result(self):
        f=self.fixture('idempotent');self.retry(f);before=(f['run']/'workflow_state.json').read_bytes();self.assertEqual(self.retry(f)['status'],'ALREADY_REQUEUED');self.assertEqual((f['run']/'workflow_state.json').read_bytes(),before)
        audit=f['run']/'preparation_retries'/sha(f['path']);(audit/'result.json').unlink();state=json.loads(before);row=next(r for r in state['stages'] if r['id']=='mission_execute');row.update(state='RUNNING',invocations=[dict(directory='private_later_invocation')]);old_driver.atomic_json(f['run']/'workflow_state.json',state);later=(f['run']/'workflow_state.json').read_bytes()
        self.assertEqual(self.retry(f)['status'],'ALREADY_REQUEUED');self.assertEqual((f['run']/'workflow_state.json').read_bytes(),later)
    def test_interruption_before_after_rename_and_after_durable_state_reenters_once(self):
        for point in ['before_rename','after_rename','after_state']:
            with self.subTest(point=point):
                f=self.fixture(point);before=(f['run']/'workflow_state.json').read_bytes();rename=TOOL.os.rename;atomic=old_driver.atomic_json
                def renamed(a,b):
                    if point=='before_rename':raise InterruptedForTest(point)
                    rename(a,b)
                    if point=='after_rename':raise InterruptedForTest(point)
                def committed(path,value):
                    atomic(path,value)
                    if Path(path)==f['run']/'workflow_state.json':raise InterruptedForTest(point)
                with patch.object(TOOL.os,'rename',side_effect=renamed) if point!='after_state' else patch.object(old_driver,'atomic_json',side_effect=committed):
                    with self.assertRaises(InterruptedForTest):self.retry(f)
                result=self.retry(f);self.assertIn(result['status'],['APPLIED','ALREADY_REQUEUED']);self.check_applied(f,before)
    def test_store_all_sidecars_and_nonempty_invocation_refuse_without_mutation(self):
        for suffix in ['', '-wal','-shm','-journal','.lock','ACTUAL_SUFFIX_LOCK','BROKEN_SYMLINK','INVOCATION_FILE']:
            with self.subTest(suffix=suffix):
                f=self.fixture('residue');db=f['run']/'mission.sqlite'
                if suffix=='ACTUAL_SUFFIX_LOCK':db.with_suffix('.lock').touch()
                elif suffix=='BROKEN_SYMLINK':db.symlink_to('missing_original_store')
                elif suffix=='INVOCATION_FILE':(f['empty']/'invocation.json').write_text('{}')
                else:Path(str(db)+suffix).touch()
                self.reject(f)
    def test_unknown_target_fields_next_operation_or_recorded_invocation_refuse(self):
        for field,value in [('next_operation','resume'),('client_response',{}),('unknown',True),('invocations',[dict(directory='known_invocation')])]:
            with self.subTest(field=field):
                f=self.fixture('unknown');next(r for r in f['state']['stages'] if r['id']=='mission_execute')[field]=value;self.refresh(f);self.reject(f)
    def test_real_lock_contention_refuses_without_mutation(self):
        f=self.fixture('locked')
        with (f['run']/'workflow.lock').open('a+') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);self.reject(f)
    def test_expired_dead_actor_and_changed_operator_reference_refuse(self):
        for fault in ['expired','dead_actor','bad_candidate_ref']:
            with self.subTest(fault=fault):
                f=self.fixture(fault)
                if fault=='expired':f['deadline']['deadline_boottime_sec']=time.clock_gettime(time.CLOCK_BOOTTIME)-1.;self.refresh(f)
                elif fault=='dead_actor':
                    child=subprocess.Popen(['/usr/bin/sleep','.05']);dead=workflow_worker.identity(child.pid);child.wait();next(r for r in f['state']['stages'] if r['id']=='planner_observer')['actor_identity']=dead;self.refresh(f)
                else:f['permit']['candidate']['sha256']='0'*64;f['path'].write_bytes(encoded(f['permit']))
                self.reject(f)
    def test_future_parent_domain_guard_runs_before_state_or_spawn_but_exempts_close_only(self):
        f=self.fixture('future_domain');before=(f['run']/'workflow_state.json').read_bytes()
        with patch.dict(os.environ,ROS_DOMAIN_ID='0'),patch.object(subprocess,'Popen',side_effect=AssertionError('Domain guard must precede spawn')):
            with self.assertRaisesRegex(ValueError,'Private domain185'):FUTURE.execute(f['run'],CANDIDATE,False)
            self.assertEqual((f['run']/'workflow_state.json').read_bytes(),before)
            with contextlib.redirect_stdout(io.StringIO()):result=FUTURE.execute(f['run'],CANDIDATE,False,close_only=True)
            self.assertEqual(result,0)
        self.assertEqual(next(r for r in json.loads((f['run']/'workflow_state.json').read_text())['stages'] if r['id']=='mission_execute')['state'],'HELD')

if __name__=='__main__':unittest.main()
