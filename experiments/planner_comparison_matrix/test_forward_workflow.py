"""Fixed workflow CLI/authority regressions. All actors/proofs are SYNTHETIC.

Actors are finite local Python processes without ROS, sockets or simulator.
Typed scanfit/NDT inputs exercise schema binding only, never sensor acceptance.
"""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import uuid

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
WORKFLOW=HERE/'workflow'
sys.path.insert(0,str(WORKFLOW));sys.path.insert(0,str(HERE/'goal_pilot_source'))
import workflow_inputs as inputs
import workflow_driver as driver
import mission_state

ACTOR_CODE='''import ctypes,sys,time
ctypes.CDLL(sys.argv[2])
print("SYNTHETIC confirm: 2/2",flush=True)
print("SYNTHETIC INITIALIZING -> ACTIVE",flush=True)
time.sleep(float(sys.argv[3]))
'''
OWNER_CODE='''import hashlib,json,os,signal,subprocess,sys,time
from pathlib import Path
def identity(pid):
 p=Path('/proc')/str(pid);s=(p/'stat').read_text().rsplit(')',1)[1].split();raw=(p/'cmdline').read_bytes()
 return dict(pid=pid,ppid=int(s[1]),starttime_ticks=int(s[19]),cmdline_sha256=hashlib.sha256(raw).hexdigest(),
  argv=[x.decode() for x in raw.split(b'\\0') if x],boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
  pid_namespace=os.readlink(p/'ns/pid'),network_namespace=os.readlink(p/'ns/net'))
p=Path(sys.argv[sys.argv.index('--plan')+1]);plan=json.loads(p.read_text());out=Path(plan['output']);out.mkdir()
log=(out/'actor.log').open('w');children=[];started=time.time_ns();boot=time.clock_gettime(time.CLOCK_BOOTTIME)
child=subprocess.Popen(plan['argv'],stdout=log,stderr=subprocess.STDOUT);children.append(child)
if plan.get('relay_argv'):children.append(subprocess.Popen(plan['relay_argv'],stdout=log,stderr=subprocess.STDOUT))
time.sleep(.07)
row=dict(schema='terra.owned_actor.started.v1',role=plan['role'],plan_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
 started_ns=started,deadline_boottime_sec=boot+plan['max_runtime_sec'],owner=identity(os.getpid()),child=identity(child.pid))
if len(children)>1:row['relay']=identity(children[1].pid)
(out/'started.json').write_text(json.dumps(row))
def stop(*args):raise SystemExit(0)
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
try:time.sleep(plan['max_runtime_sec'])
finally:
 for process in children:
  if process.poll() is None:process.terminate()
 for process in children:
  try:process.wait(timeout=2)
  except subprocess.TimeoutExpired:process.kill();process.wait()
 log.close()
'''
STAGE_CODE='''import json,sys,time
from pathlib import Path
time.sleep(float(sys.argv[2]));path=Path(sys.argv[1]);path.parent.mkdir(parents=True,exist_ok=True)
path.write_text(json.dumps(dict(ok=True,synthetic_protocol_only=True)))
'''


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        base=os.environ.get('MATRIX_WORKFLOW_TEST_OUTPUT')
        if base:
            self.root=Path(base).resolve()/self._testMethodName;self.root.mkdir(parents=True,exist_ok=False);self.temp=None
        else:
            self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.processes=[];self.sequence=0;self.commands=[]
        self.environment=dict(os.environ,ROS_DOMAIN_ID='185',PYTHONDONTWRITEBYTECODE='1')
        self.boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.context=dict(boot_id=self.boot,pid_namespace=os.readlink('/proc/self/ns/pid'),network_namespace=os.readlink('/proc/self/ns/net'))
        self.binary=Path(sys.executable).resolve()
        self.library=Path('/usr/lib/x86_64-linux-gnu/libm.so.6').resolve()
        self.config=self.write('config.json',dict(synthetic_protocol_only=True))
        self.map=self.write('map_identity.json',dict(map_id='SYNTHETIC',sensor_acceptance=False))
        points=[dict(id='P%02d'%i,pose=[3*math.cos(i*2*math.pi/40),3*math.sin(i*2*math.pi/40),0.],require_stopped=True) for i in range(40)]
        points.append(dict(points[0],id='P00_CLOSE'))
        self.route=self.write('route.json',dict(schema_version=1,frame_id='map',mission_budget_sec=1800.,waypoints=points))
        self.policy=self.write('policy.json',dict(mission_budget_sec=1800.,max_parent_dispatches=10,
            max_parent_dispatches_per_index=3,max_attempts_per_index=6,bt_retries=1))
        self.owner=self.root/'owner.py';self.owner.write_text(OWNER_CODE)
        self.actor=self.root/'actor.py';self.actor.write_text(ACTOR_CODE)
        self.adapter=self.root/'sim_command_adapter.py';self.adapter.write_text(ACTOR_CODE)
        self.relay=self.root/'sim_udp_relay.py';self.relay.write_text(ACTOR_CODE)
        self.stage=self.root/'stage.py';self.stage.write_text(STAGE_CODE)

    def tearDown(self):
        self.stop_actors()
        (self.root/'CLI_COMMANDS.json').write_text(json.dumps(self.commands,indent=2)+'\n')
        if self.temp:self.temp.cleanup()

    def stop_actors(self):
        for process in self.processes:
            if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
        for process in self.processes:
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=2)
        self.processes=[]

    def write(self,name,value):
        path=self.root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value,sort_keys=True,indent=2)+'\n');return path

    def cli(self,*args,expected=0,timeout=30):
        self.sequence+=1;command=[str(self.binary),'-B',str(WORKFLOW/'workflow.py'),*map(str,args)]
        result=subprocess.run(command,env=self.environment,capture_output=True,text=True,timeout=timeout)
        row=dict(command=command,returncode=result.returncode,stdout=result.stdout,stderr=result.stderr)
        self.commands.append(row);self.write('command_%02d.json'%self.sequence,row)
        self.assertEqual(result.returncode,expected,result.stdout+result.stderr)
        return result

    def source(self,source_id='SYNTHETIC-A',lease=2400.):
        roles={};tag=source_id+'/'
        for role in ('sensor','localization','floor','command'):
            script=self.adapter if role=='command' else self.actor
            argv=[str(self.binary),str(script),str(self.config),str(self.library),str(lease)]
            plan=dict(schema='terra.owned_actor.v1',role='synthetic_'+role,argv=argv,output=str(self.root/(tag+role+'_actor')),
                executable_sha256=inputs.file_sha(self.binary),max_runtime_sec=lease,
                inputs=[inputs.pin(self.binary),inputs.pin(self.config),inputs.pin(self.library),inputs.pin(script)],**self.context)
            if role!='localization':plan.pop('executable_sha256')  # historical Python actor receipt shape
            if role=='command':
                plan['relay_argv']=[str(self.binary),str(self.relay),str(self.config),str(self.library),str(lease)]
                plan['inputs'].append(inputs.pin(self.relay))
            path=self.write(tag+role+'_plan.json',plan)
            process=subprocess.Popen([str(self.binary),str(self.owner),'--plan',str(path),'--plan-sha256',inputs.file_sha(path)],
                stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
            self.processes.append(process);started=Path(plan['output'])/'started.json';end=time.monotonic()+4.
            while not started.exists() and time.monotonic()<end:
                self.assertIsNone(process.poll(),'fixture owner exited');time.sleep(.01)
            self.assertTrue(started.exists(),'fixture started receipt missing')
            roles[role]=dict(plan=inputs.pin(path),started=inputs.pin(started),result_path=str(Path(plan['output'])/'result.json'),interpreter=inputs.pin(self.binary))
        started=json.loads(Path(roles['localization']['started']['path']).read_text())
        pairs=[];fits=[]
        for index in range(3):
            row={kind:dict(header_ns=(index+1)*100+offset,cdr_sha256=hashlib.sha256((kind+str(index)).encode()).hexdigest(),
                received_unix_ns=started['started_ns']+1_000_000+index) for offset,kind in enumerate(('cloud','pose','imu'))}
            pairs.append(row);fit=dict(fraction_within_0_1m=.95)
            for kind,value in row.items():fit.update({kind+'_header_ns':value['header_ns'],kind+'_sha256':value['cdr_sha256']})
            fits.append(fit)
        capture=self.write(tag+'capture.json',dict(schema='terra.source11.stationary_capture.v1',pairs=pairs,status_values=[1],valid_values=[True],synthetic_protocol_only=True))
        fit=self.write(tag+'fit.json',dict(schema='terra.source.stationary_native_scanfit.v1',stationary_capture_sha256=inputs.file_sha(capture),
            valid_pair_count=3,pairs=fits,rejected_pairs=[],status_values=[1],valid_values=[True],synthetic_protocol_only=True))
        log=Path(json.loads(Path(roles['localization']['plan']['path']).read_text())['output'])/'actor.log'
        ndt=self.root/(tag+'ndt.txt');ndt.write_text(log.read_text())
        native=dict(binary=inputs.pin(self.binary),library=inputs.pin(self.library),runtime_config=inputs.pin(self.config),process=started['child'])
        release=self.write(tag+'release.json',dict(schema='matrix.simulation.localization_release.v1',source_id=source_id,GT_used=False,
            map_identity_sha256=inputs.file_sha(self.map),config_sha256=inputs.file_sha(self.config),native_binary_sha256=native['binary']['sha256'],
            native_library_sha256=native['library']['sha256'],runtime_config_sha256=native['runtime_config']['sha256'],
            localization_plan_sha256=roles['localization']['plan']['sha256'],localization_started_sha256=roles['localization']['started']['sha256'],
            fresh_scanfit_pass=True,ndt_confirm_2_of_2=True,stationary_capture=inputs.pin(capture),stationary_scanfit=inputs.pin(fit),
            ndt_confirmation=inputs.pin(ndt),synthetic_protocol_only=True))
        return self.write(tag+'source.json',dict(schema='matrix.simulation.source.v1',source_id=source_id,ros_domain_id=185,
            map_identity=inputs.pin(self.map),config=inputs.pin(self.config),native=native,roles=roles,localization_release=inputs.pin(release),**self.context))

    def candidate(self,label='A',variant='official',extra_sources=()):
        sources=[]
        for path in sorted([*WORKFLOW.glob('*.py'),*(HERE/'goal_pilot_source').glob('*.py')]):
            sources.append(dict(inputs.pin(path),name=str(path.relative_to(REPO))))
        for path in (self.owner,self.actor,self.adapter,self.relay,self.stage):
            sources.append(dict(inputs.pin(path),name='synthetic_fixture/'+path.name))
        known={row['path'] for row in sources}
        for path in extra_sources:
            path=Path(path).resolve()
            if str(path) in known:continue
            name=str(path.relative_to(REPO)) if path.is_relative_to(REPO) else 'synthetic_fixture/'+path.name
            sources.append(dict(inputs.pin(path),name=name));known.add(str(path))
        install=self.root/('install_'+label);install.mkdir();(install/'fixture_artifact').write_text('SYNTHETIC '+label)
        commit=('a' if label=='A' else 'b')*40
        receipts=[]
        for component in ('navigation','recovery','waypoint'):
            receipt=self.write('build_'+label+'_'+component+'.json',dict(schema='matrix.'+component+'.build_receipt.v1',
                repository_commit=commit,source_files_match_commit=True,
                source_sha256={row['name']:row['sha256'] for row in sources},install_prefix=str(install),
                artifact_sha256={'fixture_artifact':inputs.file_sha(install/'fixture_artifact')},synthetic_protocol_only=True))
            receipts.append(inputs.pin(receipt))
        value=dict(schema='matrix.simulation.candidate.v1',status='ACCEPTED',variant=variant,commit=commit,
            source_files=sources,build_receipts=receipts,client_request_idempotency_tested=True,
            entrypoints={**{path.name:str(path) for path in WORKFLOW.glob('*.py')},'mission_client':str(HERE/'goal_pilot_source/mission_resume.py')},
            scope=dict(environment='matrix_private_simulation',ros_domain_id=185,map_identity_sha256=inputs.file_sha(self.map),
                policy_sha256=inputs.file_sha(self.policy),original_route=inputs.pin(self.route)),synthetic_protocol_only=True)
        binding=inputs.candidate_binding(value)
        value['code_reviews']=[inputs.pin(self.write('review_'+label+'.json',dict(schema='matrix.simulation.code_review.v1',
            decision='APPROVED',candidate_binding_sha256=binding,synthetic_protocol_only=True)))]
        value['software_tests']=[inputs.pin(self.write('tests_'+label+'.json',dict(schema='matrix.simulation.software_tests.v1',
            status='PASS',candidate_binding_sha256=binding,synthetic_protocol_only=True)))]
        return self.write('candidate_'+label+'.json',value)

    def template(self,pause=0.,operation='start'):
        stages=[]
        def command(name,roles,delay=0.):
            path='{run_dir}/'+name+'.json'
            return dict(id=name,kind='command',roles=roles,cwd=str(self.root),env={'PATH':'/usr/bin:/bin'},
                argv=[str(self.binary),str(self.stage),path,str(delay)],files=[inputs.pin(self.stage)],
                success_files=[dict(path=path,equals={'ok':True})])
        stages.append(command('prepare_inputs',['environment','map','localization','recording','runtime'],pause))
        client=dict(id='mission',kind='client',roles=['mission'],depends_on=['prepare_inputs'],cwd=str(HERE),
            env={'ROS_DOMAIN_ID':'185','PATH':'/usr/bin:/bin','PYTHONDONTWRITEBYTECODE':'1'},
            files=[inputs.pin(HERE/'goal_pilot_source/mission_resume.py')],
            client=dict(prefix=[str(self.binary),'-B',str(HERE/'goal_pilot_source/mission_resume.py')],operation=operation,
                store='{run_dir}/mission.db',args={'ready':str(self.root/'unused_READY.json')},accepted_outcomes=['COMPLETED']))
        stages.append(client)
        prior='mission'
        for role in ('closure','mcap','video','video_qa','acceptance'):
            stage=command(role,[role]);stage['depends_on']=[prior]
            if role=='closure':stage['exact_owned_cleanup']=True
            stages.append(stage);prior=role
        return self.write('template.json',dict(schema='terra.workflow.execution_plan.v1',stages=stages))

    def prepare(self,candidate,source,template,run_id='FIXTURE'):
        run=self.root/run_id
        self.cli('prepare','--candidate',candidate,'--source',source,'--map-identity',self.map,'--route',self.route,
            '--policy',self.policy,'--template',template,'--run-id',run_id,'--run-dir',run)
        return run

    def test_two_exact_candidates_prepare_reuse_and_default_run_is_dry(self):
        source=self.source();template=self.template();candidate=self.candidate();run=self.prepare(candidate,source,template)
        state=(run/'workflow_state.json').read_bytes();self.prepare(candidate,source,template)
        self.assertEqual((run/'workflow_state.json').read_bytes(),state)
        self.cli('dry-run','--run-dir',run);self.cli('run','--run-dir',run,'--candidate',candidate)
        self.cli('status','--run-dir',run);self.assertEqual((run/'workflow_state.json').read_bytes(),state)
        first=json.loads(state);request=next(r['request_id'] for r in first['stages'] if r['id']=='mission')
        self.assertIsNotNone(request);self.assertFalse((run/'mission.db').exists())
        self.stop_actors();new_source=self.source('SYNTHETIC-B')
        self.cli('source-check','--source',new_source,'--output',self.root/'source_B_admitted.json')
        second=self.candidate('B','segmented');other=self.prepare(second,new_source,template,'FIXTURE_B')
        self.assertEqual(json.loads((other/'execution_plan.json').read_text())['commit'],'b'*40)
        result=self.cli('prepare','--candidate',second,'--source',new_source,'--map-identity',self.map,'--route',self.route,
            '--policy',self.policy,'--template',template,'--run-id','FIXTURE','--run-dir',run,expected=10)
        self.assertIn('different immutable inputs',result.stderr)
        self.assertEqual((run/'workflow_state.json').read_bytes(),state)

    def test_source_normal_cli_and_raw_proof_or_actor_tampering_rejected(self):
        source=self.source();self.cli('source-check','--source',source,'--output',self.root/'admission.json')
        result=json.loads((self.root/'admission.json').read_text());self.assertTrue(result['ok'] and result['live'])
        original=json.loads(source.read_text());bad=copy.deepcopy(original)
        bad['native']['process']['starttime_ticks']+=1;path=self.write('bad_native.json',bad)
        self.cli('source-check','--source',path,'--output',self.root/'rejected.json',expected=10)
        release=json.loads(Path(original['localization_release']['path']).read_text())
        fit=json.loads(Path(release['stationary_scanfit']['path']).read_text());fit['pairs'][0]['fraction_within_0_1m']=.89
        badfit=self.write('bad_fit.json',fit);release['stationary_scanfit']=inputs.pin(badfit)
        badrelease=self.write('bad_release.json',release);bad=copy.deepcopy(original);bad['localization_release']=inputs.pin(badrelease)
        self.cli('source-check','--source',self.write('bad_proof.json',bad),'--output',self.root/'rejected_proof.json',expected=10)

    def test_old_typed_review_or_changed_stage_code_cannot_accept_new_candidate(self):
        source=self.source();template=self.template();a=self.candidate();b=self.candidate('B','segmented')
        av=json.loads(a.read_text());bv=json.loads(b.read_text());bv['code_reviews']=av['code_reviews']
        wrong=self.write('wrong_review.json',bv)
        self.cli('prepare','--candidate',wrong,'--source',source,'--map-identity',self.map,'--route',self.route,
            '--policy',self.policy,'--template',template,'--run-id','REJECT','--run-dir',self.root/'REJECT',expected=10)
        self.stage.write_text(STAGE_CODE+'\n# changed after acceptance\n')
        self.cli('prepare','--candidate',a,'--source',source,'--map-identity',self.map,'--route',self.route,
            '--policy',self.policy,'--template',template,'--run-id','REJECT_CODE','--run-dir',self.root/'REJECT_CODE',expected=10)

    def test_dispatch_rechecks_remaining_lease_after_successful_admission(self):
        source=self.source(lease=2103.);candidate=self.candidate();template=self.template(pause=5.)
        run=self.prepare(candidate,source,template)
        result=self.cli('run','--run-dir',run,'--candidate',candidate,'--execute','--follow',expected=10,timeout=25)
        state=json.loads((run/'workflow_state.json').read_text());rows={r['id']:r for r in state['stages']}
        self.assertEqual(rows['source_admission']['state'],'SUCCEEDED')
        self.assertEqual(rows['prepare_inputs']['state'],'SUCCEEDED')
        self.assertEqual(rows['mission']['state'],'HELD');self.assertIn('Insufficient finite source lease',result.stdout)
        self.assertFalse((run/'mission.db').exists());request=rows['mission']['request_id']
        # Seed a real durable, unresolved UUID. Closing the workflow may seal
        # fixture evidence, but cannot alter this mission's authority/history.
        identity=dict(variant='official',source_sha='a'*40,**{key:'b'*64 for key in
            ('map_contract_sha256','footprint_sha256','acceptance_policy_sha256','effective_config_sha256')})
        store=mission_state.MissionStore(run/'mission.db')
        store.create(self.route.read_bytes(),identity,json.loads(self.policy.read_text()))
        attempt=store.reserve(str(uuid.uuid4()),run/'unknown_attempt');authority=store.state();store.close()
        original_db=(run/'mission.db').read_bytes()
        self.cli('status','--run-dir',run)
        self.assertEqual(next(r['request_id'] for r in json.loads((run/'workflow_state.json').read_text())['stages'] if r['id']=='mission'),request)
        self.cli('close','--run-dir',run,'--candidate',candidate,'--execute','--follow',timeout=25)
        final=json.loads((run/'workflow_state.json').read_text());self.assertEqual(final['stages'][2]['state'],'HELD')
        self.assertEqual(final['stages'][2]['request_id'],request)
        self.assertEqual((run/'mission.db').read_bytes(),original_db)
        self.assertEqual(mission_state.MissionStore.inspect(run/'mission.db'),authority)
        self.assertFalse(authority['attempt']['terminal_confirmed']);self.assertEqual(authority['attempt']['parent_action_uuid'],attempt['parent_action_uuid'])

    def test_budget_rules_are_canonical_for_parent_child_and_global_caps(self):
        from test_mission_state import fixture
        raw,identity,policy,_=fixture();policy.update(max_parent_dispatches=10,max_parent_dispatches_per_index=3)
        store=mission_state.MissionStore(self.root/'budget.db');store.create(raw,identity,policy)
        try:
            original=store.state();self.assertIsNone(driver.dispatch_budget_exhaustion(original))
            for kind in ('parent','child','global'):
                state=copy.deepcopy(original)
                if kind=='parent':state['budgets']['per_index_parent_dispatches'][0]=3
                elif kind=='child':state['budgets']['per_index_reserved'][0]=6
                else:state['budgets']['mission_dispatches_reserved']=10
                self.assertEqual(driver.dispatch_budget_exhaustion(state),mission_state.dispatch_budget_exhaustion(state))
                self.assertIsNotNone(driver.dispatch_budget_exhaustion(state))
                with store.transaction('FIXTURE_PROSPECTIVE_CAP') as committed:
                    committed['budgets']=copy.deepcopy(state['budgets'])
                before=store.state()
                with self.assertRaisesRegex(mission_state.Hold,'FAILED_BUDGET'):
                    store.reserve(str(uuid.uuid4()),self.root/('forbidden_'+kind))
                self.assertEqual(store.state(),before)
            sentinel={'fixture':'canonical rule'}
            with patch.object(mission_state,'dispatch_budget_exhaustion',return_value=sentinel) as authority:
                self.assertIs(driver.dispatch_budget_exhaustion(original),sentinel);authority.assert_called_once_with(original)
        finally:store.close()

    def test_closure_source_failure_keeps_fresh_ownership_guards_and_default_is_inert(self):
        import close_recording
        now=1_000_000_000_000;nav=self.root/'closure_nav';nav.mkdir()
        ready=dict(ready=False,reason='Stale receive /laser_scan',time_ns=now,valid_until_ns=now+500_000_000,
            mission_deadline={'run_id':'fixture_closure'},recovery_runtime={'verified':True},
            waypoint_ownership=dict(navigate_to_pose_active=[],follow_waypoints_active=[],follow_path_active=[],
                follow_path_status_publishers_match=True,status_publishers_match=True,
                exclusive_nav_client=True,checked_unix_ns=now))
        def proof(value):
            self.write('closure_nav/ready.json',value)
            with patch.object(close_recording.time,'time_ns',return_value=now):return close_recording.closure_ready_proof(nav,'fixture_closure')
        self.assertFalse(proof(ready)['source_health_ready'])
        for kind in ('ownership_stale','active_parent','active_child','active_follow','ingress','unknown','producer_stale','unverified_recovery','publishers'):
            bad=copy.deepcopy(ready)
            if kind=='ownership_stale':bad['waypoint_ownership']['checked_unix_ns']=now-600_000_000
            elif kind.startswith('active_'):
                key={'active_parent':'follow_waypoints_active','active_child':'navigate_to_pose_active','active_follow':'follow_path_active'}[kind]
                bad['waypoint_ownership'][key]=[{'uuid':'synthetic','status':2}]
            elif kind=='ingress':bad['reason']='Private ingress endpoint detected'
            elif kind=='unknown':bad['reason']='Ownership cannot be determined'
            elif kind=='producer_stale':bad['time_ns']=now-600_000_000
            elif kind=='unverified_recovery':bad['recovery_runtime']['verified']=False
            else:bad['waypoint_ownership']['status_publishers_match']=False
            with self.subTest(kind=kind),self.assertRaises(ValueError):proof(bad)
        output=self.root/'not_created_close.json'
        command=[str(self.binary),'-B',str(WORKFLOW/'close_recording.py'),'--workflow-dir',str(self.root/'not_started'),
            '--runtime-stage','runtime','--observer-stage','observer','--client-stage','mission','--nav-dir',str(nav),
            '--observer-summary',str(self.root/'absent_summary.json'),'--marker-dir',str(self.root/'markers'),
            '--main-acceptance',str(self.root/'absent_candidate.json'),'--output',str(output)]
        result=subprocess.run(command,env=self.environment,capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stderr);self.assertEqual(json.loads(result.stdout)['signals_sent'],0)
        self.assertFalse(output.exists());self.assertFalse((self.root/'markers').exists())

    def test_execute_and_reconcile_reread_state_only_after_obtaining_lock(self):
        source=self.source();candidate=self.candidate();run=self.prepare(candidate,source,self.template())
        real_read=driver.read_run;real_flock=driver.fcntl.flock
        for operation in ('execute','reconcile'):
            state=json.loads((run/'workflow_state.json').read_text())
            for row in state['stages']:row['state']='SUCCEEDED'
            if operation=='reconcile':state['stages'][2]['state']='HELD'
            driver.atomic_json(run/'workflow_state.json',state)
            calls=[];locked=[]
            def flock(*args):
                value=real_flock(*args);locked.append(True);return value
            def read(path):
                result=real_read(path);calls.append(bool(locked))
                if len(calls)==1:
                    newer=copy.deepcopy(result[2]);newer['fixture_concurrent_update']=operation
                    newer['stages'][2]['state']='SUCCEEDED'
                    driver.atomic_json(run/'workflow_state.json',newer)
                return result
            with patch.object(driver,'read_run',side_effect=read),patch.object(driver.fcntl,'flock',side_effect=flock), \
                    patch.object(driver,'inspect_client',side_effect=AssertionError('stale state dispatched a client')):
                if operation=='execute':driver.execute(run,candidate,False,close_only=True)
                else:
                    with self.assertRaisesRegex(ValueError,'held start/resume'):driver.reconcile(run,'mission',candidate,False)
            self.assertGreaterEqual(len(calls),2);self.assertEqual(calls[:2],[False,True])
            retained=json.loads((run/'workflow_state.json').read_text())
            self.assertEqual(retained['fixture_concurrent_update'],operation)
            self.assertEqual(retained['stages'][2]['state'],'SUCCEEDED')

    def recoverable_projection(self):
        source=self.source();template=json.loads(self.template().read_text())
        client=next(x for x in template['stages'] if x['kind']=='client')
        client['client'].update(auto_resume_when_ready=True,auto_reconcile=True)
        candidate=self.candidate();run=self.prepare(candidate,source,self.write('recovery_template.json',template))
        identity=dict(variant='official',source_sha='a'*40,**{key:'b'*64 for key in
            ('map_contract_sha256','footprint_sha256','acceptance_policy_sha256','effective_config_sha256')})
        store=mission_state.MissionStore(run/'mission.db')
        try:
            store.create(self.route.read_bytes(),identity,json.loads(self.policy.read_text()))
            attempt=store.reserve(str(uuid.uuid4()),run/'already_terminal');store.acknowledge(attempt['parent_action_uuid'],True)
            result=dict(action_uuid=attempt['parent_action_uuid'],status=6,error_code=0,
                missed_waypoints=[dict(index=0,error_code=60002)],children_quiescent=True,
                final_measured_and_commanded_stop=True,owned_goal_shutdown={'stop':{'observed':True,'strict_stop_issues':[]}})
            native=self.write('recoverable_native.json',result)
            store.finish_attempt(result,True,result_path=native);observed=store.export(run/'already_terminal')
        finally:store.close()
        workflow=json.loads((run/'workflow_state.json').read_text());row=next(x for x in workflow['stages'] if x['id']=='mission')
        row.update(state='HELD',reason='client outcome is HELD',client_response={'outcome':'HELD',
            'references':{'state':inputs.pin(run/'already_terminal/state.json')}})
        mission_state.atomic_json(run/'workflow_state.json',workflow)
        return run,candidate,workflow,observed

    def test_canceled_reconcile_publishes_latest_state_for_exact_closure_and_keeps_history(self):
        """Resume a completed protocol worker through the real workflow driver."""
        import close_recording
        source=self.source();candidate=self.candidate();run=self.prepare(candidate,source,self.template())
        plan=json.loads((run/'execution_plan.json').read_text());workflow=json.loads((run/'workflow_state.json').read_text())
        stage=next(x for x in plan['stages'] if x['id']=='mission');row=next(x for x in workflow['stages'] if x['id']=='mission')
        original_dir=run/'stages/mission/invocation_000';original_dir.mkdir(parents=True)
        rec_dir=run/'stages/mission/reconcile_000';rec_dir.mkdir(parents=True)
        identity=dict(variant='official',source_sha='a'*40,**{key:'b'*64 for key in
            ('map_contract_sha256','footprint_sha256','acceptance_policy_sha256','effective_config_sha256')})
        store=mission_state.MissionStore(run/'mission.db')
        store.create(self.route.read_bytes(),identity,json.loads(self.policy.read_text()))
        attempt=store.reserve(row['request_id'],original_dir/'client_output');store.acknowledge(attempt['parent_action_uuid'],True)
        owned=attempt['parent_action_uuid']
        native=dict(action_uuid=owned,status=None,error_code=None,missed_waypoints=None,children_quiescent=True,
            final_measured_and_commanded_stop=True,stop_signals=[signal.SIGINT],
            owned_goal_shutdown={'stop':{'observed':True,'strict_stop_issues':[]}})
        native_path=self.write('original_unknown_native.json',native)
        store.finish_attempt(native,True,result_path=native_path)

        def output(directory,operation,request,outcome,code,reason):
            state=store.export(directory/'client_output')
            response=dict(schema='matrix.mission.operation.v1',operation=operation,request_id=request,
                workflow_run_id=plan['workflow_run_id'],operation_outcome=outcome,exit_code=code,
                mission_id=state['mission_id'],revision=state['revision'],attempt_id=state['attempt']['attempt_id'],
                parent_action_uuid=owned,state_path=str(directory/'client_output/state.json'),
                state_sha256=inputs.file_sha(directory/'client_output/state.json'),
                result_path=str(directory/'client_output/result.json'),result_sha256=inputs.file_sha(directory/'client_output/result.json'),
                reason=reason,result_flags=state['result_flags'])
            mission_state.atomic_json(directory/'stdout.log',response)
            return state

        old_state=output(original_dir,'start',row['request_id'],'HELD',10,'UNKNOWN_DISPATCH')
        old_response=driver.client_output(plan,dict(stage,request_id=row['request_id']),original_dir,10)
        store.close()
        request=str(uuid.uuid4());recovery_stage=copy.deepcopy(stage)
        recovery_stage['id']='mission-reconcile-000';recovery_stage['client'].update(operation='reconcile',accepted_outcomes=['CANCELED','COMPLETED'])
        spec,inspect=driver.prepare_invocation(run,plan,recovery_stage,request,rec_dir)
        raw=(json.dumps(spec,sort_keys=True,separators=(',',':'))+'\n').encode()
        (rec_dir/'invocation.json').write_bytes(raw);spec_sha=hashlib.sha256(raw).hexdigest()
        store=mission_state.MissionStore(run/'mission.db')
        try:
            native.update(status=5,error_code=0,missed_waypoints=[],reconciliation=True,stop_signals=[])
            reconciled_native=self.write('reconciled_canceled_native.json',native)
            store.finish_attempt(native,True,result_path=reconciled_native)
            fresh_state=output(rec_dir,'reconcile',request,'CANCELED',11,'OPERATOR_CANCEL')
        finally:store.close()
        pending=dict(state='RUNNING',directory=str(rec_dir),spec_sha256=spec_sha,request_id=request,
            original_action_uuid=owned,stage=recovery_stage,inspect=inspect,intent_unix_ns=time.time_ns())
        mission_state.atomic_json(rec_dir/'worker_result.json',dict(state='EXITED',returncode=11,spec_sha256=spec_sha))
        row.update(state='HELD',reason='UNKNOWN_DISPATCH',client_response=old_response,invocations=[],reconciliations=[pending])
        mission_state.atomic_json(run/'workflow_state.json',workflow)
        originals={p:p.read_bytes() for p in original_dir.rglob('*') if p.is_file()}
        self.assertEqual(driver.reconcile(run,'mission',candidate,True),10)
        final=json.loads((run/'workflow_state.json').read_text());current=next(x for x in final['stages'] if x['id']=='mission')
        self.assertEqual(current['state'],'HELD');self.assertTrue(current['recovery_forbidden'])
        self.assertGreater(current['client_response']['references']['revision'],old_state['revision'])
        self.assertEqual(current['client_response']['references']['revision'],fresh_state['revision'])
        self.assertEqual(current['client_response']['references']['parent_action_uuid'],owned)
        self.assertIn(old_response,[x['response'] for x in current['client_response_history']])
        proof=close_recording.client_state(final,'mission')
        self.assertEqual(proof['revision'],fresh_state['revision']);self.assertEqual(proof['action_uuid'],owned)
        for path,original in originals.items():self.assertEqual(path.read_bytes(),original)
        self.assertEqual(mission_state.MissionStore.inspect(run/'mission.db')['budgets']['mission_dispatches_reserved'],1)

    def test_projection_same_revision_semantics_and_newer_unsafe_never_fall_back(self):
        import close_recording
        import mission_evidence
        from test_mission_state import fixture
        raw,identity,policy,_=fixture();store=mission_state.MissionStore(self.root/'projection.db')
        store.create(raw,identity,policy);attempt=store.reserve(str(uuid.uuid4()),self.root/'projection_attempt')
        result=dict(action_uuid=attempt['parent_action_uuid'],status=5,error_code=0,missed_waypoints=[],
            children_quiescent=True,final_measured_and_commanded_stop=True,stop_signals=[signal.SIGINT],
            owned_goal_shutdown={'stop':{'observed':True,'strict_stop_issues':[]}})
        native=self.write('projection_native.json',result)
        store.finish_attempt(result,True,result_path=native);state=store.export(self.root/'projection_output');store.close()
        first={'outcome':'CANCELED','references':{'state':inputs.pin(self.root/'projection_output/state.json')}}
        row={'id':'mission','state':'HELD','recovery_forbidden':True}
        driver.publish_client_projection(row,first,'operation',attempt['parent_action_uuid'])
        pretty=self.root/'same_revision_pretty.json';pretty.write_text(json.dumps(state,indent=3,sort_keys=False)+'\n')
        self.assertNotEqual(inputs.file_sha(pretty),first['references']['state']['sha256'])
        formatted={'outcome':'CANCELED','references':{'state':inputs.pin(pretty)}}
        driver.publish_client_projection(row,formatted,'reconcile',attempt['parent_action_uuid'])
        history=copy.deepcopy(row['client_response_history'])
        driver.publish_client_projection(row,copy.deepcopy(formatted),'reconcile',attempt['parent_action_uuid'])
        self.assertEqual(row['client_response_history'],history)
        self.assertEqual(close_recording.client_state({'stages':[row]},'mission')['revision'],state['revision'])
        unpublished=copy.deepcopy(state);unpublished['revision']+=100
        unpublished_response={'outcome':'CANCELED','references':{'state':inputs.pin(self.write('unpublished_newer.json',unpublished))}}
        row['reconciliations']=[{'state':'HELD','response':unpublished_response}]
        self.assertEqual(mission_evidence.latest({'stages':[row]},'mission')[2],formatted['references']['state'])
        original_files={p:p.read_bytes() for p in (native,pretty,self.root/'projection_output/state.json')}
        for kind in ('revision_regression','same_revision_conflict','different_mission','different_identity'):
            altered=copy.deepcopy(state)
            if kind=='revision_regression':altered['revision']-=1
            elif kind=='same_revision_conflict':altered['hold_reason']='CONFLICTING_SAME_REVISION'
            elif kind=='different_mission':altered['mission_id']=str(uuid.uuid4())
            else:altered['identity']['source_sha']='c'*40
            response={'outcome':'CANCELED','references':{'state':inputs.pin(self.write(kind+'.json',altered))}}
            rejected=copy.deepcopy(row)
            with self.subTest(kind=kind),self.assertRaises(ValueError):
                driver.publish_client_projection(rejected,response,'reconcile',attempt['parent_action_uuid'])
            with self.subTest(kind=kind),self.assertRaises(ValueError):
                close_recording.client_state({'stages':[rejected]},'mission')
            with self.subTest(kind=kind),self.assertRaises(ValueError):
                mission_evidence.latest({'stages':[rejected]},'mission')
        unsafe=copy.deepcopy(state);unsafe['revision']+=1;unsafe['lifecycle']='HOLD';unsafe['hold_reason']='UNKNOWN_DISPATCH'
        unsafe['attempt'].update(terminal_confirmed=False,children_quiescent=False,stopped=False)
        unsafe['stop']={'state':'UNPROVEN'}
        response={'outcome':'HELD','references':{'state':inputs.pin(self.write('newer_unknown.json',unsafe))}}
        driver.publish_client_projection(row,response,'reconcile',attempt['parent_action_uuid'])
        self.assertEqual(row['client_response'],response)
        self.assertNotIn('client_projection_invalid',row)
        self.assertEqual(mission_evidence.latest({'stages':[row]},'mission')[3]['revision'],unsafe['revision'])
        with self.assertRaises(ValueError):close_recording.client_state({'stages':[row]},'mission')
        for path,original in original_files.items():self.assertEqual(path.read_bytes(),original)

    def test_new_reconcile_intent_and_unknown_worker_invalidate_old_safe_projection(self):
        import close_recording
        import mission_evidence
        run,candidate,workflow,observed=self.recoverable_projection()
        row=next(x for x in workflow['stages'] if x['id']=='mission')
        self.assertEqual(close_recording.client_state(workflow,'mission')['revision'],observed['revision'])
        previous=row['client_response'];original=(run/'already_terminal/state.json').read_bytes()
        real_popen=subprocess.Popen;workers=[]
        def spawn(argv,*args,**kwargs):
            if len(argv)>2 and str(argv[2])==str(WORKFLOW/'workflow_worker.py'):
                workers.append(list(argv));return None
            return real_popen(argv,*args,**kwargs)
        with patch.object(driver.subprocess,'Popen',side_effect=spawn):
            self.assertEqual(driver.reconcile(run,'mission',candidate,False),0)
        state=json.loads((run/'workflow_state.json').read_text());current=next(x for x in state['stages'] if x['id']=='mission')
        self.assertEqual(len(workers),1);self.assertTrue(current['client_projection_invalid'])
        self.assertEqual(current['client_response'],previous)
        with self.assertRaises(ValueError):close_recording.client_state(state,'mission')
        with self.assertRaises(ValueError):mission_evidence.latest(state,'mission')
        current['reconciliations'][-1]['intent_unix_ns']-=6_000_000_000
        mission_state.atomic_json(run/'workflow_state.json',state)
        with patch.object(driver.subprocess,'Popen',side_effect=AssertionError('unknown worker must not be relaunched')):
            self.assertEqual(driver.reconcile(run,'mission',candidate,False),10)
        held=json.loads((run/'workflow_state.json').read_text());current=next(x for x in held['stages'] if x['id']=='mission')
        self.assertTrue(current['client_projection_invalid']);self.assertEqual(current['reconciliations'][-1]['state'],'HELD')
        with self.assertRaises(ValueError):close_recording.client_state(held,'mission')
        self.assertEqual((run/'already_terminal/state.json').read_bytes(),original)
        self.assertEqual(mission_state.MissionStore.inspect(run/'mission.db')['budgets']['mission_dispatches_reserved'],1)

    def test_invalid_projection_blocks_retry_and_budget_close_before_and_after_final_flock(self):
        run,candidate,workflow,observed=self.recoverable_projection()
        original_request=next(x for x in workflow['stages'] if x['id']=='mission')['request_id']
        db_bytes=(run/'mission.db').read_bytes()
        cases=[(driver.retry_recoverable_when_ready,'retry'),(driver.close_exhausted_recoverable,'exhausted'),
            (driver.close_expired_recoverable,'expired')]
        for function,kind in cases:
            client=copy.deepcopy(observed)
            if kind=='exhausted':
                client['budgets']['per_index_parent_dispatches'][0]=3;client['budgets']['per_index_reserved'][0]=6
            elif kind=='expired':client['budgets']['deadline_boottime_sec']=time.clock_gettime(time.CLOCK_BOOTTIME)-1.
            reference=inputs.pin(self.write('client_'+kind+'.json',client))
            state=copy.deepcopy(workflow);row=next(x for x in state['stages'] if x['id']=='mission')
            row['client_response']={'outcome':'HELD','references':{'state':reference}}
            row['client_projection_invalid']={'reason':'new_reconciliation_pending'}
            mission_state.atomic_json(run/'workflow_state.json',state)
            with self.subTest(kind=kind,phase='entry'),patch.object(driver.fcntl,'flock',side_effect=AssertionError('invalid projection reached mutation lock')):
                self.assertFalse(function(run,candidate,'mission'))
            row.pop('client_projection_invalid');mission_state.atomic_json(run/'workflow_state.json',state)
            now=time.time_ns();self.write('unused_READY.json',dict(ready=True,time_ns=now,valid_until_ns=now+500_000_000,
                waypoint_ownership=dict(exclusive_nav_client=True,navigate_to_pose_active=[],follow_waypoints_active=[]),
                recovery_runtime={'verified':True}))
            real_flock=driver.fcntl.flock;injected=[]
            def flock(stream,operation):
                value=real_flock(stream,operation)
                if operation & driver.fcntl.LOCK_EX and not injected:
                    latest=json.loads((run/'workflow_state.json').read_text())
                    current=next(x for x in latest['stages'] if x['id']=='mission')
                    current['client_projection_invalid']={'reason':'concurrent_reconciliation_started_before_mutation'}
                    mission_state.atomic_json(run/'workflow_state.json',latest);injected.append(True)
                return value
            with self.subTest(kind=kind,phase='locked'),patch.object(driver.fcntl,'flock',side_effect=flock):
                self.assertFalse(function(run,candidate,'mission'))
            self.assertEqual(injected,[True])
            retained=json.loads((run/'workflow_state.json').read_text());current=next(x for x in retained['stages'] if x['id']=='mission')
            self.assertEqual(current['state'],'HELD');self.assertEqual(current['request_id'],original_request)
            self.assertTrue(current['client_projection_invalid']);self.assertEqual((run/'mission.db').read_bytes(),db_bytes)

    def video_projection_fixture(self,name,accepted=False):
        """Real Store receipts; native observations are explicit no-ROS fixtures."""
        from test_mission_state import fixture
        run=self.root/name;run.mkdir();raw,identity,policy,example=fixture()
        store=mission_state.MissionStore(run/'mission.db');store.create(raw,identity,policy)
        originals={};observed=[]
        try:
            for number in range(2 if accepted else 1):
                directory=run/('attempt_%02d'%number);directory.mkdir()
                attempt=store.reserve(str(uuid.uuid4()),directory);store.acknowledge(attempt['parent_action_uuid'],True)
                visits=copy.deepcopy(example['ordered']['visits'])
                for visit in visits[1:]:visit['passed']=None
                visits[0]['entered']={'unix_ns':1_000_000_000+number*2_000_000_000}
                visits[0]['passed'].update(unix_ns=2_000_000_000+number*2_000_000_000,
                    steady_ns=2_000_000_000+number*2_000_000_000)
                visits[0]['motion']={'signed_yaw_rad':.125+number}
                promoted=accepted and number==1
                store.observe(attempt['parent_action_uuid'],SimpleNamespace(active=1 if promoted else 0,visits=visits))
                native=dict(action_uuid=attempt['parent_action_uuid'],status=6,error_code=0,
                    missed_waypoints=[dict(index=1 if promoted else 0,error_code=60002)],
                    children_quiescent=True,final_measured_and_commanded_stop=True,ordered={'visits':visits},
                    owned_goal_shutdown={'stop':{'observed':True,'strict_stop_issues':[]}})
                native_path=directory/'native_result.json';mission_state.atomic_json(native_path,native)
                events=directory/'events.jsonl';events.write_text(json.dumps(dict(kind='goal_send',unix_ns=1_000_000_000+number*2_000_000_000,
                    action_uuid=attempt['parent_action_uuid']))+'\n'+json.dumps(dict(kind='action_result',unix_ns=2_000_000_000+number*2_000_000_000,
                    action_uuid=attempt['parent_action_uuid'],status=6))+'\n')
                store.finish_attempt(native,True,result_path=native_path)
                observed.append((copy.deepcopy(attempt),copy.deepcopy(native),inputs.pin(native_path)))
                originals.update({p:p.read_bytes() for p in (native_path,events)})
            state=store.export(run/'client_output')
        finally:store.close()
        reference=inputs.pin(run/'client_output/state.json')
        mission_state.atomic_json(run/'workflow_state.json',dict(stages=[dict(id='mission_execute',state='HELD',
            client_response_source='operation',client_response={'outcome':'HELD','references':{'state':reference}})]))
        mission_state.atomic_json(run/'mcap_audit.json',dict(read_to_eof=True,integrity_pass=True,
            first_log_ns=500_000_000,last_log_ns=5_000_000_000,synthetic_protocol_only=True))
        originals[run/'client_output/state.json']=(run/'client_output/state.json').read_bytes()
        return run,state,observed,originals

    def consumer(self,script,run,*,success=True):
        command=[str(self.binary),'-B',str(WORKFLOW/script),'--run-dir',str(run)]
        result=subprocess.run(command,env=self.environment,capture_output=True,text=True,timeout=15)
        row=dict(command=command,returncode=result.returncode,stdout=result.stdout,stderr=result.stderr)
        self.commands.append(row);self.write('consumer_'+run.name+'_'+script+'.json',row)
        if success:self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        else:self.assertNotEqual(result.returncode,0,result.stdout+result.stderr)
        return result

    def test_canonical_analysis_and_video_cli_reject_invalid_projection_before_reading_mcap(self):
        run,state,observations,originals=self.video_projection_fixture('invalid_consumers')
        workflow=json.loads((run/'workflow_state.json').read_text());row=workflow['stages'][0]
        old=copy.deepcopy(row['client_response']);newer=copy.deepcopy(state);newer['revision']+=100
        higher={'outcome':'HELD','references':{'state':inputs.pin(self.write('unpublished_safe_high_revision.json',newer))}}
        row.update(client_projection_invalid={'reason':'new_reconciliation_outcome_unknown'},
            client_response_history=[{'source':'operation','response':old}],
            reconciliations=[{'state':'RECONCILED','response':higher}])
        mission_state.atomic_json(run/'workflow_state.json',workflow)
        for script in ('analyze_run.py','prepare_video_inputs.py'):
            result=self.consumer(script,run,success=False)
            self.assertIn('latest client observation is pending or invalid',result.stderr.lower())
        for name in ('metrics.json','video_inputs.json','video_case'):self.assertFalse((run/name).exists())
        for path,raw in originals.items():self.assertEqual(path.read_bytes(),raw)

    def test_video_zero_accepted_keeps_native_pass_only_as_bound_observation(self):
        run,state,observations,originals=self.video_projection_fixture('zero_accepted')
        self.assertEqual(state['progress']['accepted_prefix_count'],0)
        self.consumer('prepare_video_inputs.py',run)
        derived=json.loads((run/'video_case/result.json').read_text());summary=json.loads((run/'video_inputs.json').read_text())
        self.assertEqual(sum(bool(x.get('passed')) for x in derived['ordered']['visits']),0)
        self.assertFalse(derived['ordered']['all_visited']);self.assertEqual(summary['accepted_visits'],0)
        retained=derived['unaccepted_native_pass_observations'];self.assertEqual(len(retained),1)
        attempt,native,reference=observations[0];observation=retained[0]
        self.assertEqual(observation['attempt_id'],attempt['attempt_id'])
        self.assertEqual(observation['parent_action_uuid'],attempt['parent_action_uuid'])
        self.assertEqual(observation['native_result'],reference)
        self.assertEqual(observation['visit'],native['ordered']['visits'][0])
        self.assertEqual(derived['status'],native['status'])
        self.assertEqual(json.loads(Path(observation['native_result']['path']).read_text())['status'],6)
        for path,raw in originals.items():self.assertEqual(path.read_bytes(),raw)

    def test_video_same_index_accepts_only_receipt_source_and_keeps_earlier_attempt_observation(self):
        run,state,observations,originals=self.video_projection_fixture('accepted_later_source',accepted=True)
        self.assertEqual(state['progress']['accepted_prefix_count'],1)
        self.consumer('prepare_video_inputs.py',run)
        derived=json.loads((run/'video_case/result.json').read_text());receipt=state['progress']['accepted_receipts'][0]
        first,first_native,first_ref=observations[0];second,second_native,second_ref=observations[1]
        self.assertNotEqual(first['parent_action_uuid'],second['parent_action_uuid'])
        self.assertEqual(receipt['source_attempt_id'],second['attempt_id'])
        self.assertEqual(derived['ordered']['visits'][0],receipt['visit'])
        self.assertEqual(derived['ordered']['visits'][0]['passed']['unix_ns'],4_000_000_000)
        self.assertEqual(sum(bool(x.get('passed')) for x in derived['ordered']['visits']),1)
        self.assertFalse(derived['ordered']['all_visited'])
        retained=derived['unaccepted_native_pass_observations'];self.assertEqual(len(retained),1)
        self.assertEqual(retained[0]['index'],0);self.assertEqual(retained[0]['attempt_id'],first['attempt_id'])
        self.assertEqual(retained[0]['parent_action_uuid'],first['parent_action_uuid'])
        self.assertEqual(retained[0]['native_result'],first_ref)
        self.assertEqual(retained[0]['visit'],first_native['ordered']['visits'][0])
        self.assertEqual(derived['status'],second_native['status'])
        for path,raw in originals.items():self.assertEqual(path.read_bytes(),raw)

    def test_builder_prepares_pinned_current_template_and_close_defaults_to_no_signal(self):
        source=self.source();original_render=self.root/'original_render_mobile_review.py';original_render.write_text(STAGE_CODE)
        extra=[HERE/name for name in ('matrix_navigation.py','recording_deadline.py','recovery_navigation.py',
            'static_layer_snapshot.py','run_matrix_navigation.sh','waypoint_runtime.py')]+[original_render]
        candidate=self.candidate(extra_sources=extra);value=json.loads(candidate.read_text());receipts=value['build_receipts']
        snapshot=self.write('source_snapshot.json',dict(synthetic_protocol_only=True));topics=self.write('required_topics.json',['/fixture/performance'])
        stage=dict(id='performance',kind='spawn',roles=[],cwd=str(self.root),env={'ROS_DOMAIN_ID':'185'},
            argv=[str(self.binary),str(self.stage),'{run_dir}/performance/ready.json','0'],files=[inputs.pin(self.stage)],
            finite_owner_and_exact_cleanup=True,ready={'path':'{run_dir}/performance/ready.json','equals':{'ok':True}})
        analysis=dict(id='analysis',kind='command',roles=[],cwd=str(self.root),env={},argv=[str(self.binary),str(self.stage),'{run_dir}/video_inputs.json','0'],
            files=[inputs.pin(self.stage)],success_files=[{'path':'{run_dir}/video_inputs.json'}])
        acceptance=dict(analysis,id='acceptance',roles=['acceptance'],argv=[str(self.binary),str(self.stage),'{run_dir}/acceptance.json','0'],
            success_files=[{'path':'{run_dir}/acceptance.json'}])
        binding=dict(schema='terra.matrix.bindings.v1',variant='official',env={'ROS_DOMAIN_ID':'185','PATH':'/usr/bin:/bin'},
            cwd=str(HERE),python=str(self.binary),workflow_run_id='BUILDER',commit=value['commit'],build_receipts=receipts,
            nav=dict(launcher=str(HERE/'matrix_navigation.py'),source_snapshot=str(snapshot),build_receipt=receipts[0]['path'],
                message_install=str(self.root/'install_A'),recovery_install=str(self.root/'install_A'),recovery_build_receipt=receipts[1]['path'],
                selected_install=str(self.root/'install_A'),waypoint_install=str(self.root/'install_A'),waypoint_build_receipt=receipts[2]['path']),
            mission=dict(client=str(HERE/'goal_pilot_source/mission_resume.py'),route=str(self.route),policy=str(self.policy),identity=str(self.map),mode='new'),
            map_identity=str(self.map),map_dir=str(self.root),map_reserve_bytes=0,required_topics=str(topics),performance_topic='/fixture/performance',
            planner_observer=str(self.stage),renderer=str(self.stage),phone_qa=str(self.stage),main_acceptance=str(candidate),
            tools={name:str(self.stage) for name in ('verify_source25_map.py','planner_observer_entry.py','verify_recording.py')},
            environment_stages=[],localization_stages=[],source_cleanup_stages=[],performance_stage=stage,
            acceptance_stage=acceptance)
        path=self.write('bindings.json',binding);template=self.root/'builder_template.json'
        result=subprocess.run([str(self.binary),'-B',str(WORKFLOW/'build_matrix_plan.py'),'--bindings',str(path),'--output',str(template)],
            env=self.environment,capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr);self.assertEqual(json.loads(result.stdout)['started_processes'],0)
        run=self.prepare(candidate,source,template,'BUILDER');self.cli('dry-run','--run-dir',run)
        plan=json.loads((run/'execution_plan.json').read_text());self.assertEqual(plan['stages'][0]['id'],'source_admission')
        for stage_id,script,role in [('analyze_closed_recording','analyze_run.py','mcap'),
                ('prepare_video_inputs','prepare_video_inputs.py','video')]:
            selected=next(x for x in plan['stages'] if x['id']==stage_id)
            self.assertEqual(selected['argv'][2],str(WORKFLOW/script));self.assertEqual(selected['roles'],[role])
            pins={x['path']:x['sha256'] for x in selected['files']}
            for path in (WORKFLOW/script,WORKFLOW/'mission_evidence.py'):
                self.assertEqual(pins[str(path)],inputs.file_sha(path))
        legacy=dict(binding,analysis_and_video_input_stages=[analysis]);legacy_path=self.write('obsolete_bindings.json',legacy)
        rejected=self.root/'obsolete_template.json'
        result=subprocess.run([str(self.binary),'-B',str(WORKFLOW/'build_matrix_plan.py'),'--bindings',str(legacy_path),'--output',str(rejected)],
            env=self.environment,capture_output=True,text=True,timeout=10)
        self.assertNotEqual(result.returncode,0);self.assertIn('obsolete',result.stderr);self.assertFalse(rejected.exists())
        closure=next(row for row in plan['stages'] if row['id']=='close_navigation');self.assertTrue(closure['exact_owned_cleanup'])
        self.assertEqual(closure['argv'][2],str(WORKFLOW/'close_recording.py'))
        target=self.root/'inert_close.json'
        command=[str(self.binary),'-B',str(WORKFLOW/'close_recording.py'),'--workflow-dir',str(run),
            '--runtime-stage','navigation_recording','--observer-stage','planner_observer','--client-stage','mission_execute',
            '--nav-dir',str(self.root/'not_started_nav'),'--observer-summary',str(self.root/'not_created.json'),
            '--marker-dir',str(self.root/'markers'),'--main-acceptance',str(candidate),'--output',str(target)]
        result=subprocess.run(command,env=self.environment,capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stderr);self.assertEqual(json.loads(result.stdout)['signals_sent'],0)
        self.assertFalse(target.exists());self.assertTrue(all(process.poll() is None for process in self.processes))

    def test_performance_source_role_binding_stops_before_ros(self):
        import runtime_performance
        import rclpy
        source=self.source();native=json.loads(source.read_text())['native']['process'];nav=self.root/'nav';nav.mkdir()
        deadline=dict(run_id='synthetic_perf',launcher=native,deadline_boottime_sec=time.clock_gettime(time.CLOCK_BOOTTIME)+5.)
        self.write('nav/runtime_manifest.json',{'recording_deadline':deadline});self.write('nav/processes.json',{})
        output=self.root/'performance';old={number:signal.getsignal(number) for number in (signal.SIGINT,signal.SIGTERM)}
        try:
            with patch.object(sys,'argv',['runtime_performance.py','--nav-dir',str(nav),'--source-manifest',str(source),
                    '--output',str(output),'--run-id','synthetic_perf']), \
                    patch.object(rclpy,'init',side_effect=RuntimeError('fixture stops before ROS initialization')):
                with self.assertRaisesRegex(RuntimeError,'fixture stops before ROS'):runtime_performance.main()
        finally:
            for number,handler in old.items():signal.signal(number,handler)
        identities=json.loads((output/'identity.json').read_text())['identities']
        self.assertEqual({key for key in identities if key.startswith('source_')},
            {'source_sensor','source_localization','source_floor','source_command'})
        self.assertTrue(all(value['pid']>0 and value['birth']>0 for value in identities.values()))


if __name__=='__main__':unittest.main()
