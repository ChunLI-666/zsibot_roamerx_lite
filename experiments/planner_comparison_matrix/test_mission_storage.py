"""Real SQLite durability, fake executor responsiveness, owner/crash boundaries."""
import ast,copy,json,math,os,signal,sqlite3,subprocess,sys,tempfile,threading,time,unittest,uuid
from pathlib import Path
from collections import deque
from types import SimpleNamespace,MethodType
from unittest.mock import patch
from test_mission_state import fixture
from mission_state import MissionStore,Hold,atomic_json
from mission_storage import StorageWorker
from mission_resume import Session

HERE=Path(__file__).resolve().parent;SOURCE=HERE/'goal_pilot_source'
def method(name,ns):
    tree=ast.parse((SOURCE/'waypoint_batch.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='WaypointPilot')
    node=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(SOURCE/'waypoint_batch.py'),'exec'),ns);return ns[name]

def signal_before_send_batch(node,options,route,trace):
    """Actual batch/interrupt methods inside the normal CLI, with fake ROS I/O."""
    from waypoint_contract import OwnedGoalIdentity
    class Pose:
        def __init__(self):self.header=SimpleNamespace();self.pose=SimpleNamespace(position=SimpleNamespace(),orientation=SimpleNamespace())
    class Client:
        def __init__(self,*args):pass
        def wait_for_server(self,**kwargs):return True
        def send_goal_async(self,request,**kwargs):
            trace['goals_sent']+=1
            handle=SimpleNamespace(accepted=False,goal_id=kwargs['goal_uuid'])
            return SimpleNamespace(done=lambda:True,result=lambda:handle)
    ns=dict(time=time,math=math,uuid=uuid,Path=Path,ACTION='/follow_waypoints',ActionClient=Client,
        FollowWaypoints=SimpleNamespace(Goal=lambda:SimpleNamespace(poses=[])),PoseStamped=Pose,
        UUID=lambda **kwargs:SimpleNamespace(**kwargs),OwnedGoalIdentity=OwnedGoalIdentity,
        initial_leg_limit=lambda *a:8.,mission_budget=lambda *a:{},frozen=SimpleNamespace(stamp_ns=lambda x:x))
    node.args=options;node.route=route;node.finalizing=False;node.goal_sent_ns=None;node.latest_ready_sha='fixture'
    node.event_pub=SimpleNamespace(get_subscription_count=lambda:1)
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=time.time_ns))
    node.native_feedback=lambda m:None
    snapshot={'last':{'pose':{'x':0.,'y':0.,'stamp_ns':1}}}
    node.execution_ready=lambda **kwargs:(True,[],snapshot)
    node.wait_stable_preflight=lambda *args:(True,[],snapshot)
    node.check_interruption=MethodType(method('check_interruption',ns),node)
    node.post_intent_preflight=MethodType(method('post_intent_preflight',ns),node)
    def emit(kind,**fields):
        if kind=='goal_send':
            trace['goal_send_events']=trace.get('goal_send_events',0)+1
            trace['constructed_pose_count']=len(fields['pose_header_stamps_ns'])
            os.kill(os.getpid(),signal.SIGINT)
    node.emit=emit
    return method('run_batch',ns)(node)

class StorageTests(unittest.TestCase):
    def test_health_guard_uses_one_consistent_snapshot_and_live_boot_deadline(self):
        from mission_continuation import CLIENT_SOURCES
        from mission_state import digest,boot_clock
        state=self.worker.state();boot,now=boot_clock()
        ready=dict(variant=state['variant'],mission_identity=dict(state['identity'],verified=True),
            recovery_runtime=dict(verified=True),waypoint_contract=dict(mission_resume=dict(
                schema='matrix.mission.resume_capability.v1',bt_retries=state['policy']['bt_retries'],
                client_sources_sha256={name:digest((SOURCE/name).read_bytes()) for name in CLIENT_SOURCES})))
        session=Session(self.worker,str(uuid.uuid4()),self.root/'health')
        with patch.object(self.worker,'state',wraps=self.worker.state) as reads:
            session.validate_ready(ready);self.assertEqual(reads.call_count,1)
        state['budgets'].update(deadline_boot_id=boot,deadline_boottime_sec=now+100)
        with patch.object(self.worker,'state',return_value=state):
            with patch('mission_state.boot_clock',return_value=(boot,now+101)):
                with self.assertRaisesRegex(Hold,'FAILED_BUDGET'):session.validate_ready(ready)
            with patch('mission_state.boot_clock',return_value=('changed-boot',now)):
                with self.assertRaisesRegex(Hold,'CLOCK_UNTRUSTED'):session.validate_ready(ready)
        before=self.worker.state()
        with patch('mission_storage.copy.deepcopy',side_effect=AssertionError('unnecessary copy')):
            self.assertGreater(self.worker.remaining(),0)
            self.worker.check_identity(before['identity'])
        self.assertEqual(self.worker.state(),before)

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.owner_ids=[];self.delays={};self.phases=[];self.connections=[];self.settings=[];outer=self
        class Audited(MissionStore):
            def __init__(self,path,resource):
                super().__init__(path,resource);outer.connections.append(self.db);db=self.db
                outer.settings.append((db.execute('PRAGMA synchronous').fetchone()[0],db.execute('PRAGMA journal_mode').fetchone()[0]))
                outer.owner_ids.append(threading.get_ident())
                class DB:
                    def execute(inner,sql,*args):
                        outer.owner_ids.append(threading.get_ident())
                        result=db.execute(sql,*args)
                        if sql=='COMMIT':
                            outer.phases.append(('COMMIT',time.monotonic()));time.sleep(outer.delays.get('COMMIT',0))
                        return result
                    def __getattr__(inner,name):return getattr(db,name)
                self.db=DB()
            def close(self):outer.owner_ids.append(threading.get_ident());super().close()
        self.worker=StorageWorker(self.root/'mission.db',self.root/'owner.lock',factory=Audited)
        raw,identity,policy,_=fixture();self.worker.create(raw,identity,policy)
    def tearDown(self):self.worker.close();self.assertFalse(self.worker._thread.is_alive());self.temp.cleanup()
    def export_delay(self,path,value):
        atomic_json(path,value);self.phases.append((path.name,time.monotonic()));time.sleep(self.delays.get(path.name,0))
    def test_a16_sized_commit_and_both_fsync_delays_keep_original_input_pump_advancing(self):
        self.delays.update(COMMIT=.9,**{'state.json':.45,'result.json':.13});ticks=[];samples=[]
        def pump():
            time.sleep(.001)  # Model a bounded spin when no callback backlog exists.
            # Original source stamp and callback time are separately sampled.
            source=time.time_ns();received=time.monotonic_ns();ticks.append(received);samples.append((source,received))
        self.worker.pump=pump;start=time.monotonic();session=Session(self.worker,str(uuid.uuid4()),self.root/'attempt')
        with patch('mission_state.atomic_json',side_effect=self.export_delay):ident=session.reserve()
        ended=time.monotonic();self.assertGreaterEqual(ended-start,1.48)
        self.assertGreater(len(ticks),50);self.assertLess(max(b-a for a,b in zip(ticks,ticks[1:])),100_000_000)
        self.assertTrue(all(b[0]>a[0] for a,b in zip(samples,samples[1:])))
        self.assertLess(time.time_ns()-samples[-1][0],100_000_000)
        self.assertEqual(str(ident),self.worker.state()['attempt']['parent_action_uuid'])
        self.assertEqual(self.worker.state()['budgets']['mission_dispatches_reserved'],1)
        self.assertEqual([p[0] for p in self.phases[-3:]],['COMMIT','state.json','result.json'])
        with self.assertRaises(sqlite3.ProgrammingError):self.connections[0].execute('SELECT 1')
        self.assertEqual(len(set(self.owner_ids)),1);self.assertNotEqual(self.owner_ids[0],threading.get_ident())
        self.assertEqual(self.settings,[(2,'delete')])
        with sqlite3.connect(self.root/'mission.db') as db:
            self.assertEqual(db.execute('PRAGMA journal_mode').fetchone()[0],'delete')
        # Default FULL is additionally checked on the actual owner connection below.
    def test_owner_lock_and_cached_state_never_reenter_during_delayed_commit(self):
        self.delays['COMMIT']=.08;checked=[]
        def pump():
            self.worker.state();self.worker.remaining()
            if checked:return
            with self.assertRaises(BlockingIOError):MissionStore(self.root/'other.db',self.root/'owner.lock')
            with self.assertRaisesRegex(Hold,'reentrant'):self.worker.export(self.root)
            checked.append(True)
        self.worker.pump=pump;self.worker.reserve(str(uuid.uuid4()),self.root/'attempt')
        self.assertEqual(checked,[True])
    def test_export_failure_retains_committed_uuid_and_never_retries_reservation(self):
        session=Session(self.worker,str(uuid.uuid4()),self.root/'attempt')
        with patch('mission_state.atomic_json',side_effect=OSError('injected fsync failure')):
            with self.assertRaises(OSError):session.reserve()
        self.assertEqual(str(session.action_uuid),self.worker.state()['attempt']['parent_action_uuid'])
        self.assertFalse(session.send_invoked)
        with self.assertRaisesRegex(Hold,'UNKNOWN_DISPATCH'):self.worker.reserve(str(uuid.uuid4()),self.root/'next')
        self.assertEqual(self.worker.state()['budgets']['mission_dispatches_reserved'],1)
    def test_pump_exception_waits_for_real_commit_then_preserves_intent(self):
        self.delays['COMMIT']=.08;calls=[]
        def broken():calls.append(True);raise InterruptedError('signal')
        self.worker.pump=broken
        start=time.monotonic();session=Session(self.worker,str(uuid.uuid4()),self.root/'attempt')
        with self.assertRaises(InterruptedError):session.reserve()
        self.assertGreaterEqual(time.monotonic()-start,.08);self.assertIsNotNone(session.action_uuid)
        self.assertLess(len(calls),100)  # A failing pump must not busy-spin.
        self.worker.pump=None;self.assertEqual(self.worker.state()['budgets']['mission_dispatches_reserved'],1)
    def test_callback_backlog_drains_during_commit_without_per_callback_sleep(self):
        self.delays['COMMIT']=.15;pending=deque(range(600));received=[];idle=threading.Event()
        def pump():
            if pending:received.append(pending.popleft())
            else:idle.wait(.002)
        self.worker.pump=pump;self.worker.reserve(str(uuid.uuid4()),self.root/'attempt')
        self.assertEqual(received,list(range(600)));self.assertFalse(pending)
    def test_wait_without_pump_does_not_busy_spin(self):
        self.delays['COMMIT']=.15;started=time.thread_time()
        self.worker.reserve(str(uuid.uuid4()),self.root/'attempt')
        self.assertLess(time.thread_time()-started,.075)
    def test_truthful_not_sent_has_null_native_status_no_refund_and_reopen_cannot_claim_it(self):
        session=Session(self.worker,str(uuid.uuid4()),self.root/'attempt');ident=session.reserve();before=self.worker.state()
        result=dict(action_uuid=str(ident),status=None,goals_sent=0,send_goal_async_invoked=False,strict_scan_timing_status='FAIL')
        p=self.root/'not_sent.json';atomic_json(p,result)
        with self.assertRaises(Hold):self.worker.record_not_sent(str(ident),result_path=p,send_invoked=True)
        self.worker.record_not_sent(str(ident),result_path=p,send_invoked=False);state=self.worker.state()
        self.assertEqual(state['attempt']['dispatch_state'],'NOT_SENT_AFTER_INTENT')
        self.assertIsNone(state['attempt']['native_status']);self.assertFalse(state['attempt']['terminal_confirmed'])
        self.assertEqual(state['budgets']['mission_dispatches_reserved'],before['budgets']['mission_dispatches_reserved'])
        self.assertEqual(state['progress'],before['progress'])
        self.assertEqual(state['result_flags']['strict_timing_status'],'FAIL')
        self.worker.close()
        with self.assertRaisesRegex(Hold,'proof'):
            s=MissionStore(self.root/'mission.db')
            try:s.record_not_sent(str(ident),result_path=p,send_invoked=False)
            finally:s.close()
    def test_visit_queue_is_ordered_bounded_and_drained_without_nested_storage(self):
        session=Session(self.worker,str(uuid.uuid4()),self.root/'attempt');session.reserve()
        route=self.worker.state()['route'];ordered=session.ordered(route);ordered.feedback(0,1,1)
        self.delays['COMMIT']=.05;injected=[]
        def pump():
            if injected:return
            injected.append(True);later=copy.deepcopy(ordered);later.active=1
            session.observe(later,defer=True)
        self.worker.pump=pump;session.observe(ordered)
        self.assertFalse(session.pending_visits);self.assertEqual(self.worker.state()['attempt']['highest_feedback_index'],1)
        self.worker.pump=None
        for i in range(64):
            queued=copy.deepcopy(ordered);queued.active=i;session.observe(queued,defer=True)
        with self.assertRaisesRegex(Hold,'overflow'):
            queued.active=65;session.observe(queued,defer=True)
        self.assertEqual(len(session.pending_visits),64)
    def test_failed_visit_does_not_silently_drop_or_advance_pending_checkpoint(self):
        session=Session(self.worker,str(uuid.uuid4()),self.root/'attempt');session.reserve()
        ordered=session.ordered(self.worker.state()['route']);ordered.feedback(0,1,1)
        with patch('mission_state.atomic_json',side_effect=OSError('export failed')):
            with self.assertRaises(OSError):session.observe(ordered)
        self.assertEqual(len(session.pending_visits),1)
        with self.assertRaisesRegex(Hold,'Prior storage'):session.drain()
    def test_real_sigint_during_visit_commit_cancels_before_commit_returns_and_owner_finishes(self):
        session=Session(self.worker,str(uuid.uuid4()),self.root/'attempt');session.reserve();session.send_invoked=True
        ordered=session.ordered(self.worker.state()['route']);ordered.feedback(0,1,1)
        node,spins,cancels=PumpTests().node();node.mission=session;sent=[]
        node.cancel_own=lambda reason:cancels.append((reason,time.monotonic())) if not cancels else None
        old=signal.signal(signal.SIGINT,lambda n,f:node.args.stop_signals.append(n))
        started=time.monotonic();self.delays['COMMIT']=.2
        def pump():
            if not sent and time.monotonic()-started>.03:
                sent.append(True);os.kill(os.getpid(),signal.SIGINT)
            node.pump()
        self.worker.pump=pump
        try:session.observe(ordered)
        finally:signal.signal(signal.SIGINT,old)
        ended=time.monotonic();self.assertEqual(cancels[0][0],'pilot_interrupted')
        self.assertLess(cancels[0][1]-started,.15);self.assertGreater(ended-cancels[0][1],.05)
        self.assertGreater(len(spins),10);self.assertFalse(session.pending_visits)
    def test_health_loss_and_deadline_during_ack_are_seen_before_durability_finishes(self):
        attempt=self.worker.reserve(str(uuid.uuid4()),self.root/'attempt')
        for mode in ['health','deadline']:
            self.delays['COMMIT']=.15;node,spins,cancels=PumpTests().node();started=time.monotonic()
            node.cancel_own=lambda reason:cancels.append((reason,time.monotonic())) if not cancels else None
            if mode=='deadline':node.args.timeout=.03
            else:node.execution_ready=lambda **kw:(time.monotonic()-started<.03,['scan:invalid_header_age'],{})
            self.worker.pump=node.pump
            self.worker.acknowledge(attempt['parent_action_uuid'],True)
            self.assertTrue(cancels);self.assertLess(cancels[0][1]-started,.12);self.assertGreater(len(spins),5)
            self.worker.pump=None
    def test_process_crash_after_intent_keeps_unknown_dispatch_and_releases_lock(self):
        db=self.root/'crash.db';lock=self.root/'crash_resource.lock'
        script="""import os,sys,uuid
from pathlib import Path
from mission_storage import StorageWorker
from test_mission_state import fixture
w=StorageWorker(sys.argv[1],sys.argv[2]);r,i,p,_=fixture();w.create(r,i,p)
w.reserve(str(uuid.uuid4()),Path(sys.argv[1]).parent/'crash_attempt');os._exit(19)
"""
        env=dict(os.environ,PYTHONPATH=os.pathsep.join([str(SOURCE),str(HERE)]))
        p=subprocess.run([sys.executable,'-c',script,str(db),str(lock)],env=env,timeout=10)
        self.assertEqual(p.returncode,19);s=MissionStore(db,lock)
        try:
            self.assertEqual(s.db.execute('PRAGMA synchronous').fetchone()[0],2)
            self.assertEqual(s.db.execute('PRAGMA journal_mode').fetchone()[0],'delete')
            with self.assertRaisesRegex(Hold,'UNKNOWN_DISPATCH'):s.reserve(str(uuid.uuid4()),self.root)
        finally:s.close()

class PumpTests(unittest.TestCase):
    def node(self,reason=None):
        calls=[];cancels=[];snap={'last':{'pose':{'x':0.,'y':0.}}}
        ros=SimpleNamespace(ok=lambda:True,spin_once=lambda n,timeout_sec:calls.append(timeout_sec))
        node=SimpleNamespace(goal_sent_ns=1,goal_sent_steady=time.monotonic(),args=SimpleNamespace(stop_signals=[],timeout=100.,cancel_after=None),
            ordered=SimpleNamespace(violation=None),feedback_overflow=0,mission=SimpleNamespace(store=SimpleNamespace(remaining=lambda:50.)),
            execution_ready=lambda **kw:(reason is None,[] if reason is None else [reason],snap),invalidate_artifact=lambda issues:None,
            observe_ordered=lambda *args,**kw:None,cancel_own=cancels.append)
        node.pump=MethodType(method('pump_storage_wait',dict(rclpy=ros,time=time)),node)
        return node,calls,cancels
    def test_health_signal_deadline_and_queue_failure_cancel_without_waiting_for_io(self):
        for case in ['health','signal','deadline','queue']:
            node,calls,cancels=self.node('scan:invalid_header_age' if case=='health' else None)
            if case=='signal':node.args.stop_signals=[2]
            if case=='deadline':node.args.timeout=0.
            if case=='queue':
                node.observe_ordered=lambda *a,**k:(_ for _ in ()).throw(Hold('queue overflow'))
                with self.assertRaises(Hold):node.pump()
            else:node.pump()
            self.assertEqual(calls,[.002]);self.assertEqual(len(cancels),1)
            if case=='health':self.assertTrue(cancels[0].startswith('input_became_invalid:'))
    def test_unsent_wait_observes_health_but_never_cancels_another_goal(self):
        node,_,cancels=self.node('odom:invalid_header_age');node.goal_sent_ns=None;node.pump()
        self.assertTrue(node.storage_preflight_gap);self.assertFalse(cancels)
    def test_post_intent_uses_original_stable_window_and_fresh_snapshot_same_uuid(self):
        clock=SimpleNamespace(monotonic=lambda:10.);calls=[];snapshot={'last':{'pose':{'x':0.,'y':0.}}}
        ns=dict(time=clock,math=math,initial_leg_limit=lambda *args:8.)
        run=method('post_intent_preflight',ns)
        ident=uuid.uuid4();node=SimpleNamespace(mission=SimpleNamespace(action_uuid=ident,start_index=0,store=SimpleNamespace(remaining=lambda:25.)),
            args=SimpleNamespace(timeout=100.,stable_preflight_seconds=10.,simulation_first_leg_max_distance_m=8.,simulation_scan_max_age_sec=.3),
            latest_ready={},route=[{'pose':[1.,0.,0.]}],emit=lambda *a,**k:calls.append(k),
            wait_stable_preflight=lambda d,s:(calls.append((d,s)) or (True,[],snapshot)),execution_ready=lambda **k:(True,[],snapshot))
        valid,_,returned=run(node,40.,1.)
        self.assertTrue(valid);self.assertIs(returned,snapshot);self.assertEqual(calls[0],(35.,1.));self.assertEqual(node.args.timeout,25.)
        self.assertEqual(calls[-1]['action_uuid'],str(ident));self.assertEqual(calls[-1]['original_stable_sec'],10.)
        node.execution_ready=lambda **k:(False,['stale'],snapshot)
        self.assertFalse(run(node,40.,1.)[0])

class DispatchTests(unittest.TestCase):
    def test_real_batch_reserves_once_rechecks_after_durability_and_sends_same_uuid_once(self):
        from waypoint_contract import OwnedGoalIdentity
        for ready_after in [True,False]:
            with self.subTest(ready_after=ready_after),tempfile.TemporaryDirectory() as d:
                root=Path(d);worker=StorageWorker(root/'state.db');raw,identity,policy,_=fixture();worker.create(raw,identity,policy)
                session=Session(worker,str(uuid.uuid4()),root/'attempt');events=[];sent=[];stable=[];snapshots=[]
                class Pose:
                    def __init__(self):self.header=SimpleNamespace();self.pose=SimpleNamespace(position=SimpleNamespace(),orientation=SimpleNamespace())
                class Client:
                    def __init__(self,*args):pass
                    def wait_for_server(self,**kwargs):return True
                    def send_goal_async(self,request,**kwargs):
                        state=worker.state();self_uuid=uuid.UUID(bytes=bytes(kwargs['goal_uuid'].uuid))
                        assert str(self_uuid)==state['attempt']['parent_action_uuid']
                        assert state['attempt']['dispatch_state']=='INTENT_COMMITTED'
                        assert (root/'state.json').is_file() and (root/'result.json').is_file()
                        sent.append(self_uuid);handle=SimpleNamespace(accepted=False,goal_id=kwargs['goal_uuid'])
                        return SimpleNamespace(done=lambda:True,result=lambda:handle)
                ns=dict(time=time,math=math,uuid=uuid,Path=Path,ACTION='/follow_waypoints',ActionClient=Client,
                    FollowWaypoints=SimpleNamespace(Goal=lambda:SimpleNamespace(poses=[])),PoseStamped=Pose,
                    UUID=lambda **kwargs:SimpleNamespace(**kwargs),OwnedGoalIdentity=OwnedGoalIdentity,
                    initial_leg_limit=lambda *a:8.,mission_budget=lambda *a:{},frozen=SimpleNamespace(stamp_ns=lambda x:x))
                node=SimpleNamespace(mission=session,route=worker.state()['route'],latest_ready={},latest_ready_sha='x',goal_sent_ns=None,
                    storage_preflight_gap=True,check_interruption=lambda:None,
                    args=SimpleNamespace(preflight_seconds=40.,stable_preflight_seconds=10.,timeout=100.,simulation_first_leg_max_distance_m=8.,simulation_scan_max_age_sec=.3),
                    event_pub=SimpleNamespace(get_subscription_count=lambda:1),emit=lambda kind,**kwargs:events.append((kind,kwargs)),
                    get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:time.time_ns())),native_feedback=lambda m:None)
                def check(**kwargs):
                    snapshot={'last':{'pose':{'x':0.,'y':0.,'stamp_ns':len(snapshots)}}};snapshots.append(snapshot)
                    return True,[],snapshot
                def wait(deadline,started):
                    stable.append((deadline,started));ok=ready_after or session.action_uuid is None
                    return ok,[] if ok else ['input_stale'],check()[2]
                node.execution_ready=check;node.wait_stable_preflight=wait;node.spin_until=lambda predicate,timeout:predicate()
                node.post_intent_preflight=MethodType(method('post_intent_preflight',ns),node)
                try:
                    result=method('run_batch',ns)(node)
                    self.assertEqual(worker.state()['budgets']['mission_dispatches_reserved'],1)
                    self.assertEqual(len(stable),2)
                    self.assertLessEqual(stable[1][0],stable[0][0]);self.assertEqual(stable[1][1],stable[0][1])
                    self.assertEqual(len(sent),int(ready_after));self.assertEqual(session.send_invoked,ready_after)
                    if ready_after:
                        self.assertFalse(next(v for k,v in events if k=='post_intent_preflight_passed')['persistence_health_gap'])
                        send_event=next(v for k,v in events if k=='goal_send')
                        self.assertEqual(send_event['start_snapshot'],snapshots[-1]);self.assertEqual(str(sent[0]),str(session.action_uuid))
                        self.assertEqual(result['classification'],'goal_rejected')
                    else:self.assertEqual(result['classification'],'invalid_post_intent_preflight_not_sent')
                finally:worker.close()
    def test_normal_cli_signal_from_goal_send_event_is_not_sent_after_intent(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);raw,identity,policy,_=fixture();route=json.loads(raw)
            route['waypoints']=[dict(id='F%02d'%i,pose=[0.,0.,0.],require_stopped=True) for i in range(41)]
            atomic_json(root/'route.json',route);atomic_json(root/'identity.json',identity);atomic_json(root/'policy.json',policy)
            transport=root/'transport.json';atomic_json(transport,dict(fixture_signal_before_send=True,owned_goal_shutdown={'stop':{'observed':False}}))
            cmd=[sys.executable,str(HERE/'test_mission_reconcile_evidence.py'),'--cli-fixture',str(transport),
                'start','--execute','--store',str(root/'mission.db'),'--resource-lock',str(root/'resource.lock'),
                '--route',str(root/'route.json'),'--identity',str(root/'identity.json'),'--policy',str(root/'policy.json'),
                '--request-id',str(uuid.uuid4()),'--ready',str(root/'unused.json'),'--output',str(root/'operation')]
            p=subprocess.run(cmd,env=dict(os.environ,ROS_DOMAIN_ID='185'),capture_output=True,text=True,timeout=15)
            self.assertEqual(p.returncode,10,p.stdout+p.stderr)
            trace=json.loads(transport.with_suffix('.trace.json').read_text())
            self.assertEqual(trace['goal_send_events'],1);self.assertEqual(trace['constructed_pose_count'],41)
            self.assertEqual(trace['goals_sent'],0)
            state=MissionStore.inspect(root/'mission.db');native=json.loads((root/'operation/attempt_00/native_result.json').read_text())
            self.assertEqual(state['attempt']['dispatch_state'],'NOT_SENT_AFTER_INTENT')
            self.assertEqual(state['budgets']['mission_dispatches_reserved'],1);self.assertFalse(state['attempt']['terminal_confirmed'])
            self.assertEqual(native['exception_type'],'InterruptedError');self.assertEqual(native['stop_signals'],[signal.SIGINT])
            self.assertEqual(native['goals_sent'],0);self.assertFalse(native['send_goal_async_invoked'])
            self.assertIsNone(native['status']);self.assertIsNone(state['attempt']['native_status'])
    def test_normal_cli_persists_truthful_unsent_and_next_process_never_redispatches(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);raw,identity,policy,_=fixture();(root/'route.json').write_bytes(raw)
            atomic_json(root/'identity.json',identity);atomic_json(root/'policy.json',policy)
            transport=root/'transport.json';atomic_json(transport,dict(fixture_unsent_start=True,owned_goal_shutdown={'stop':{'observed':False}}))
            base=[sys.executable,str(HERE/'test_mission_reconcile_evidence.py'),'--cli-fixture',str(transport)]
            cmd=base+['start','--execute','--store',str(root/'mission.db'),'--resource-lock',str(root/'resource.lock'),
                '--route',str(root/'route.json'),'--identity',str(root/'identity.json'),'--policy',str(root/'policy.json'),
                '--request-id',str(uuid.uuid4()),'--ready',str(root/'unused.json'),'--output',str(root/'operation')]
            env=dict(os.environ,ROS_DOMAIN_ID='185');p=subprocess.run(cmd,env=env,capture_output=True,text=True,timeout=15)
            self.assertEqual(p.returncode,10,p.stdout+p.stderr);state=MissionStore.inspect(root/'mission.db')
            self.assertEqual(state['attempt']['dispatch_state'],'NOT_SENT_AFTER_INTENT')
            self.assertIsNone(state['attempt']['native_status']);self.assertEqual(state['budgets']['mission_dispatches_reserved'],1)
            native=json.loads((root/'operation/attempt_00/native_result.json').read_text())
            self.assertIsNone(native['status']);self.assertFalse(native['send_goal_async_invoked']);self.assertEqual(native['goals_sent'],0)
            trace=json.loads(transport.with_suffix('.trace.json').read_text());self.assertEqual(trace['goals_sent'],0)
            cmd=base+['resume','--execute','--store',str(root/'mission.db'),'--resource-lock',str(root/'resource.lock'),
                '--mission-id',state['mission_id'],'--expect-revision',str(state['revision']),
                '--request-id',str(uuid.uuid4()),'--ready',str(root/'unused.json'),'--output',str(root/'resume')]
            p=subprocess.run(cmd,env=env,capture_output=True,text=True,timeout=15)
            self.assertEqual(p.returncode,10,p.stdout+p.stderr);self.assertIn('no automatic cross-operation',p.stdout)
            self.assertEqual(MissionStore.inspect(root/'mission.db'),state)

if __name__=='__main__':unittest.main()
