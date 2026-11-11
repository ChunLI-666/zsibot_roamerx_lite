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
            analysis_and_video_input_stages=[analysis],acceptance_stage=acceptance)
        path=self.write('bindings.json',binding);template=self.root/'builder_template.json'
        result=subprocess.run([str(self.binary),'-B',str(WORKFLOW/'build_matrix_plan.py'),'--bindings',str(path),'--output',str(template)],
            env=self.environment,capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr);self.assertEqual(json.loads(result.stdout)['started_processes'],0)
        run=self.prepare(candidate,source,template,'BUILDER');self.cli('dry-run','--run-dir',run)
        plan=json.loads((run/'execution_plan.json').read_text());self.assertEqual(plan['stages'][0]['id'],'source_admission')
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
