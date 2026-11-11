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
        def __init__(self):
            self.header=SimpleNamespace();self.pose=SimpleNamespace(position=SimpleNamespace(),orientation=SimpleNamespace())
            trace['constructed_pose_count']=trace.get('constructed_pose_count',0)+1
            if trace['constructed_pose_count']==len(route):os.kill(os.getpid(),signal.SIGINT)
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
    node.decision_ready=lambda *args,**kwargs:(True,[],snapshot)
    node.wait_stable_preflight=lambda *args:(True,[],snapshot)
    node.check_interruption=MethodType(method('check_interruption',ns),node)
    node.post_intent_preflight=MethodType(method('post_intent_preflight',ns),node)
    def emit(kind,**fields):
        if kind=='goal_send':
            trace['goal_send_events']=trace.get('goal_send_events',0)+1
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
        with patch.object(self.worker,'guard_state',wraps=self.worker.guard_state) as reads, \
                patch.object(self.worker,'state',side_effect=AssertionError('full history on hot guard')):
            session.validate_ready(ready);self.assertEqual(reads.call_count,1)
        state['budgets'].update(deadline_boot_id=boot,deadline_boottime_sec=now+100)
        with patch.object(self.worker,'guard_state',return_value=state):
            with patch('mission_state.boot_clock',return_value=(boot,now+101)):
                with self.assertRaisesRegex(Hold,'FAILED_BUDGET'):session.validate_ready(ready)
            with patch('mission_state.boot_clock',return_value=('changed-boot',now)):
                with self.assertRaisesRegex(Hold,'CLOCK_UNTRUSTED'):session.validate_ready(ready)
        before=self.worker.state()
        with patch('mission_storage.copy.deepcopy',side_effect=AssertionError('unnecessary copy')):
            self.assertGreater(self.worker.remaining(),0)
            self.worker.check_identity(before['identity'])
        self.assertEqual(self.worker.state(),before)

    def test_guard_is_bounded_for_41_points_and_does_not_copy_history(self):
        from mission_state import encoded,mission_guard_state
        raw,identity,policy,_=fixture();document=json.loads(raw)
        document['waypoints']=[dict(id='G%02d'%i,pose=[float(i),0.,0.],require_stopped=True) for i in range(41)]
        changes={}
        class HistoryStore(MissionStore):
            def flush_state_read_timing(inner,checkpoint):
                if checkpoint=='fixture_history':
                    with inner.transaction('FIXTURE_HISTORY') as state:
                        state['evidence']['fixture_history']=['x'*4096]*changes['rows']
                return super().flush_state_read_timing(checkpoint)
        worker=StorageWorker(self.root/'large.db',factory=HistoryStore)
        try:
            worker.create(json.dumps(document).encode(),identity,policy)
            before=worker.guard_state();size=len(encoded(before));real_copy=copy.deepcopy
            for rows in (1,250):
                changes['rows']=rows;worker.flush_state_read_timing('fixture_history')
                state=worker.state();view=worker.guard_state()
                self.assertEqual(view,mission_guard_state(state))
                self.assertEqual(len(state['route']),41)
                self.assertLessEqual(len(encoded(view)),size+4)
                self.assertNotIn('route',view);self.assertNotIn('evidence',view)
                def bounded_copy(value,*args,**kwargs):
                    if isinstance(value,dict) and ('route' in value or 'evidence' in value):
                        raise AssertionError('hot guard copied full state/history')
                    return real_copy(value,*args,**kwargs)
                with patch('mission_storage.copy.deepcopy',side_effect=bounded_copy):
                    self.assertEqual(worker.guard_state()['revision'],state['revision'])
                    self.assertGreater(worker.remaining(),0.)
                view['identity']['source_sha']='c'*40
                self.assertEqual(worker.guard_state()['identity']['source_sha'],identity['source_sha'])
        finally:worker.close()

    def test_guard_and_full_state_publish_one_revision_after_commit_or_rollback(self):
        from mission_state import mission_guard_state
        changes={};observed=[]
        class TransactionStore(MissionStore):
            def flush_state_read_timing(inner,checkpoint):
                if checkpoint=='fixture_transaction':
                    with inner.transaction('FIXTURE_GUARD') as state:
                        state['progress']['next_index']=1;state['current_target']=state['route'][1]
                        if changes.get('rollback'):raise RuntimeError('fixture rollback before commit')
                        time.sleep(.05)
                return super().flush_state_read_timing(checkpoint)
        worker=StorageWorker(self.root/'transaction.db',factory=TransactionStore)
        try:
            raw,identity,policy,_=fixture();worker.create(raw,identity,policy);before=worker.state()
            def pump():
                state=worker.state();guard=worker.guard_state()
                observed.append((state['revision'],guard['revision'],guard['progress']['next_index']))
                self.assertEqual(guard,mission_guard_state(state));time.sleep(.001)
            worker.pump=pump;worker.flush_state_read_timing('fixture_transaction')
            self.assertTrue(observed)
            self.assertEqual(set(observed),{(before['revision'],before['revision'],0)})
            after=worker.state();self.assertEqual(after['revision'],before['revision']+1)
            self.assertEqual(worker.guard_state(),mission_guard_state(after))
            worker.pump=None;changes['rollback']=True
            with self.assertRaisesRegex(RuntimeError,'rollback'):
                worker.flush_state_read_timing('fixture_transaction')
            self.assertEqual(worker.state(),after);self.assertEqual(worker.guard_state(),mission_guard_state(after))
        finally:worker.close()

    def test_remaining_rechecks_boottime_without_file_io_and_reopen_rejects_other_boot(self):
        from mission_state import boot_clock
        session=Session(self.worker,str(uuid.uuid4()),self.root/'budget');session.reserve()
        state=self.worker.state();deadline=state['budgets']['deadline_boottime_sec']
        with patch.object(Path,'read_text',side_effect=AssertionError('hot boot file I/O')), \
                patch('mission_storage.time.clock_gettime',return_value=deadline+.1):
            self.assertEqual(self.worker.remaining(),0.)
        self.worker.close();boot,_=boot_clock()
        with patch('mission_storage.boot_clock',return_value=('fixture-different-boot',0.)):
            reopened=StorageWorker(self.root/'mission.db',self.root/'owner.lock')
        try:
            self.assertNotEqual(boot,'fixture-different-boot')
            with self.assertRaisesRegex(Hold,'CLOCK_UNTRUSTED'):reopened.remaining()
            self.assertEqual(reopened.guard_state()['revision'],state['revision'])
        finally:reopened.close()

    def test_settle_validation_preserves_identity_when_motion_budget_is_exhausted(self):
        from mission_continuation import CLIENT_SOURCES
        from mission_state import digest,boot_clock
        state=self.worker.guard_state();boot,now=boot_clock()
        state['budgets'].update(deadline_boot_id=boot,deadline_boottime_sec=now-1)
        ready=dict(variant=state['variant'],mission_identity=dict(state['identity'],verified=True),
            recovery_runtime=dict(verified=True),waypoint_contract=dict(mission_resume=dict(
                schema='matrix.mission.resume_capability.v1',bt_retries=state['policy']['bt_retries'],
                client_sources_sha256={name:digest((SOURCE/name).read_bytes()) for name in CLIENT_SOURCES})))
        session=Session(self.worker,str(uuid.uuid4()),self.root/'settle')
        with patch.object(self.worker,'guard_state',return_value=state):
            with self.assertRaisesRegex(Hold,'FAILED_BUDGET'):session.validate_ready(ready)
            session.validate_ready(ready,require_budget=False)
            ready['mission_identity']['source_sha']='c'*40
            with self.assertRaisesRegex(Hold,'IDENTITY_MISMATCH'):
                session.validate_ready(ready,require_budget=False)

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
                def check(*args,**kwargs):
                    snapshot={'last':{'pose':{'x':0.,'y':0.,'stamp_ns':len(snapshots)}}};snapshots.append(snapshot)
                    return True,[],snapshot
                def wait(deadline,started):
                    stable.append((deadline,started));ok=ready_after or session.action_uuid is None
                    return ok,[] if ok else ['input_stale'],check()[2]
                node.execution_ready=check;node.decision_ready=check;node.wait_stable_preflight=wait;node.spin_until=lambda predicate,timeout:predicate()
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
    def test_normal_cli_signal_during_request_construction_is_not_sent_after_intent(self):
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
            self.assertEqual(trace.get('goal_send_events',0),0);self.assertEqual(trace['constructed_pose_count'],41)
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

class RetryControlTests(unittest.TestCase):
    """Durable original-attempt authority, independent of the CLI's loop."""
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.worker=StorageWorker(self.root/'mission.db',self.root/'owner.lock')
        raw,identity,policy,self.example=fixture()
        policy.update(max_parent_dispatches=10,max_parent_dispatches_per_index=3)
        self.worker.create(raw,identity,policy);self.observations=[]

    def tearDown(self):
        self.worker.close();self.temp.cleanup()

    def reserve(self,request=None):
        request=request or str(uuid.uuid4())
        attempt=self.worker.reserve(request,self.root/('attempt_'+request))
        self.worker.acknowledge(attempt['parent_action_uuid'],True)
        return attempt

    def finish(self,attempt,code=60002,index=0,**fields):
        result=dict(action_uuid=attempt['parent_action_uuid'],status=6,error_code=0,
            missed_waypoints=[dict(index=index,error_code=code)],children_quiescent=True,
            final_measured_and_commanded_stop=True,created_ns=attempt['created_ns'],finished_ns=time.time_ns(),
            owned_goal_shutdown={'stop':{'observed':True,'strict_stop_issues':[]}})
        result.update(fields);path=self.root/('observation_'+str(uuid.uuid4())+'.json')
        atomic_json(path,result);self.observations.append((path,path.read_bytes()))
        self.worker.finish_attempt(result,result['children_quiescent'],result_path=path)
        return result

    def paused_twice(self):
        first=self.reserve();self.finish(first)
        self.assertEqual(self.worker.state()['lifecycle'],'PAUSED_INPUT_HEALTH')
        second=self.reserve();result=self.finish(second)
        state=self.worker.state();control=state['retry_control']
        self.assertEqual(control['schema'],'matrix.mission.retry_control.v1')
        self.assertEqual(control['consecutive_failures'],2);self.assertTrue(control['paused'])
        self.assertEqual(control['last_counted_attempt_id'],second['attempt_id'])
        self.assertEqual(state['lifecycle'],'NEEDS_TASK_DECISION')
        self.assertEqual(state['hold_reason'],'REPEATED_FAILURE_REVIEW_REQUIRED')
        return second,result

    def assert_budget_not_refunded(self,after,before):
        self.assertEqual({k:v for k,v in after.items() if k!='remaining_sec'},
            {k:v for k,v in before.items() if k!='remaining_sec'})
        self.assertLessEqual(after['remaining_sec'],before['remaining_sec'])

    def test_same_failure_stops_third_intent_across_reopen_and_new_request(self):
        self.paused_twice();before=self.worker.state()
        for reopen in (False,True):
            if reopen:
                self.worker.close();self.worker=StorageWorker(self.root/'mission.db',self.root/'owner.lock')
            with self.assertRaisesRegex(Hold,'REPEATED_FAILURE_REVIEW_REQUIRED'):
                self.worker.reserve(str(uuid.uuid4()),self.root/'forbidden')
            self.assertEqual(self.worker.state(),before)
        self.assertEqual(before['budgets']['mission_dispatches_reserved'],2)
        self.assertEqual(before['budgets']['per_index_parent_dispatches'][0],2)
        self.assertEqual(before['budgets']['per_index_reserved'][0],4)
        self.assertEqual(before['progress']['accepted_prefix_count'],0)

    def test_review_permit_is_request_bound_consumed_once_and_never_refunds_budget(self):
        self.paused_twice();before=self.worker.state();request=str(uuid.uuid4())
        with self.assertRaises(Hold):self.worker.review_retry(request,'  ')
        self.worker.review_retry(request,'PM reviewed the frozen original failure; one diagnostic retry')
        granted=self.worker.state();self.assertTrue(granted['retry_control']['review_permit'])
        self.assert_budget_not_refunded(granted['budgets'],before['budgets'])
        with self.assertRaises(Hold):self.worker.reserve(str(uuid.uuid4()),self.root/'wrong_request')
        third=self.reserve(request);self.assertNotEqual(third['parent_action_uuid'],before['attempt']['parent_action_uuid'])
        self.assertNotEqual(self.worker.state()['retry_control']['review_permit'],granted['retry_control']['review_permit'])
        self.finish(third);after=self.worker.state()
        self.assertEqual(after['retry_control']['consecutive_failures'],3);self.assertTrue(after['retry_control']['paused'])
        self.assertEqual(after['budgets']['per_index_parent_dispatches'][0],3)
        self.assertEqual(after['budgets']['per_index_reserved'][0],6)
        fourth=str(uuid.uuid4())
        with self.assertRaisesRegex(Hold,'FAILED_BUDGET'):
            self.worker.review_retry(fourth,'A review cannot create dispatch allowance')
            self.worker.reserve(fourth,self.root/'over_budget')
        self.assert_budget_not_refunded(self.worker.state()['budgets'],after['budgets'])
        self.assertEqual(self.worker.state()['attempt']['parent_action_uuid'],third['parent_action_uuid'])

    def test_same_attempt_reconciliation_is_idempotent_and_keeps_original_evidence(self):
        attempt,result=self.paused_twice();before=self.worker.state()
        for _ in range(2):
            self.finish(attempt,**{k:v for k,v in result.items() if k not in ('action_uuid',)})
        after=self.worker.state()
        self.assertEqual(after['retry_control'],before['retry_control'])
        self.assert_budget_not_refunded(after['budgets'],before['budgets'])
        for path,raw in self.observations:self.assertEqual(path.read_bytes(),raw)
        self.assertGreaterEqual(len(after['attempt']['native_result_history']),3)

    def test_different_code_or_real_accepted_index_progress_changes_fingerprint(self):
        first=self.reserve();self.finish(first);old=copy.deepcopy(self.worker.state()['retry_control'])
        second=self.reserve();self.finish(second,code=60001)
        changed=self.worker.state()['retry_control'];self.assertNotEqual(changed['fingerprint'],old['fingerprint'])
        self.assertEqual(changed['consecutive_failures'],1);self.assertFalse(changed['paused'])
        third=self.reserve();state=self.worker.state()
        visits=copy.deepcopy(self.example['ordered']['visits'])
        for visit in visits[1:]:visit['passed']=None
        self.worker.observe(third['parent_action_uuid'],SimpleNamespace(active=1,visits=visits))
        self.assertEqual(self.worker.state()['progress']['accepted_prefix_count'],1)
        self.finish(third,code=60001,index=1)
        progressed=self.worker.state()['retry_control']
        self.assertNotEqual(progressed['fingerprint'],changed['fingerprint'])
        self.assertEqual(progressed['consecutive_failures'],1);self.assertFalse(progressed['paused'])

    def test_review_cannot_override_unknown_canceled_or_other_task_decision(self):
        for kind in ('unknown','canceled','task_decision'):
            if kind!='unknown':
                self.worker.close();self.worker=StorageWorker(self.root/(kind+'.db'),self.root/'owner.lock')
                raw,identity,policy,_=fixture();self.worker.create(raw,identity,policy)
            attempt=self.reserve()
            if kind=='canceled':self.finish(attempt,status=5,missed_waypoints=[],stop_signals=[signal.SIGINT])
            elif kind=='task_decision':self.finish(attempt,code=208)
            before=self.worker.state()
            with self.subTest(kind=kind),self.assertRaises(Hold):
                self.worker.review_retry(str(uuid.uuid4()),'Cannot turn this state into reviewed repeated failure')
            with self.subTest(kind=kind),self.assertRaises(Hold):
                self.worker.reserve(str(uuid.uuid4()),self.root/'forbidden')
            self.assertEqual(self.worker.state(),before)

    def test_explicit_task_cancel_after_terminal_revokes_review_and_never_changes_native_status(self):
        attempt,result=self.paused_twice();review=str(uuid.uuid4())
        self.worker.review_retry(review,'A permit exists but has not been consumed')
        before=self.worker.state();cancel=str(uuid.uuid4())
        self.worker.request_cancel(cancel,'PM ends the task after reviewing its repeated native failure')
        state=self.worker.state();self.assertEqual(state['lifecycle'],'CANCELED')
        self.assertIsNone(state['retry_control']['review_permit'])
        self.assertEqual(state['attempt']['native_status'],6)
        self.assertEqual(state['attempt']['parent_action_uuid'],attempt['parent_action_uuid'])
        self.assertEqual(state['attempt']['native_result'],before['attempt']['native_result'])
        self.assert_budget_not_refunded(state['budgets'],before['budgets'])
        self.finish(attempt,**{k:v for k,v in result.items() if k!='action_uuid'})
        self.worker.request_cancel(cancel,'PM ends the task after reviewing its repeated native failure')
        after=self.worker.state();self.assertEqual(after['lifecycle'],'CANCELED')
        self.assertEqual(after['attempt']['native_status'],6)
        self.assertEqual(after['cancel_intent'],state['cancel_intent'])
        with self.assertRaisesRegex(Hold,'cancell'):
            self.worker.reserve(review,self.root/'canceled_permit_cannot_send')
        with self.assertRaisesRegex(Hold,'cancell'):
            self.worker.review_retry(str(uuid.uuid4()),'Cannot review an explicitly canceled task back into motion')
        for path,raw in self.observations:self.assertEqual(path.read_bytes(),raw)

    def test_task_cancel_with_unknown_parent_holds_until_original_terminal_and_stop_proof(self):
        attempt=self.reserve();before=self.worker.state()
        self.worker.request_cancel(str(uuid.uuid4()),'Cancel this original UUID; its outcome is not known yet')
        self.assertEqual(self.worker.state()['lifecycle'],'HOLD')
        with self.assertRaises(Hold):self.worker.reserve(str(uuid.uuid4()),self.root/'unknown_cancel')
        self.finish(attempt,code=60002)
        state=self.worker.state();self.assertEqual(state['lifecycle'],'CANCELED')
        self.assertEqual(state['attempt']['native_status'],6)
        self.assertEqual(state['attempt']['parent_action_uuid'],before['attempt']['parent_action_uuid'])
        self.assertEqual(state['budgets']['mission_dispatches_reserved'],1)

class FinalResidualStorageTests(unittest.TestCase):
    """Public StorageWorker transactions; no synthetic source/plant acceptance."""
    def setUp(self):
        from test_mission_state import residual_protocol_fixture
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.worker=StorageWorker(self.root/'state.db',self.root/'resource.lock')
        self.source,self.result,self.original_path,self.visits=residual_protocol_fixture(self.worker,self.root)
        self.original_bytes=self.original_path.read_bytes();self.initial=self.worker.state()
        self.assertEqual(self.initial['hold_reason'],'FINAL_GOAL_RESIDUAL_ERROR')

    def tearDown(self):self.worker.close();self.temp.cleanup()

    def reopen(self):
        self.worker.close();self.worker=StorageWorker(self.root/'state.db',self.root/'resource.lock')

    def reserve_correction(self,ack=True):
        a=self.worker.reserve(str(uuid.uuid4()),self.root/'correction')
        if ack:self.worker.acknowledge(a['parent_action_uuid'],True)
        return a

    def finish_correction(self,attempt,status=6,code=60002,**extra):
        result=dict(action_uuid=attempt['parent_action_uuid'],status=status,error_code=0,error_msg='',
            missed_waypoints=[dict(index=4,error_code=code)] if status==6 else [],
            functional_status='FAIL',children_quiescent=True,final_measured_and_commanded_stop=True,
            owned_goal_shutdown=copy.deepcopy(self.result['owned_goal_shutdown']),
            evidence_writer=dict(evidence_complete=True),strict_scan_timing_status='NO_STRICT_SCAN_FAULT_RECORDED',
            finished_ns=time.time_ns())
        result.update(extra);path=self.root/('correction_result_'+str(uuid.uuid4())+'.json');atomic_json(path,result)
        self.worker.finish_attempt(result,result['children_quiescent'],result_path=path);return result,path

    def no_more(self):
        before=self.worker.state()
        with self.assertRaises(Hold):self.worker.reserve(str(uuid.uuid4()),self.root/'forbidden')
        with self.assertRaises(Hold):self.worker.review_retry(str(uuid.uuid4()),'Review cannot manufacture another final correction')
        after=self.worker.state();self.assertEqual(after['attempt']['parent_action_uuid'],before['attempt']['parent_action_uuid'])
        self.assertEqual({k:v for k,v in after['budgets'].items() if k!='remaining_sec'},
            {k:v for k,v in before['budgets'].items() if k!='remaining_sec'})
        self.assertLessEqual(after['budgets']['remaining_sec'],before['budgets']['remaining_sec'])
        self.assertEqual(self.original_path.read_bytes(),self.original_bytes)

    def test_one_correction_reserve_preserves_original_prefix_index_uuid_binding_and_budget_across_reopen(self):
        authority=copy.deepcopy(self.initial['attempt']['terminal_authority']);self.reopen();a=self.reserve_correction()
        s=self.worker.state();c=s['final_goal_correction']
        self.assertEqual(a['start_index'],4);self.assertEqual(a['final_goal_correction_for'],self.source['attempt_id'])
        self.assertEqual(c['consumed_attempt_id'],a['attempt_id']);self.assertEqual(c['consumed_action_uuid'],a['parent_action_uuid'])
        self.assertEqual(s['progress']['accepted_receipts'],self.initial['progress']['accepted_receipts'])
        self.assertEqual(s['budgets']['deadline_boottime_sec'],self.initial['budgets']['deadline_boottime_sec'])
        self.assertEqual(s['budgets']['per_index_reserved'],[2,2,2,2,4]);self.assertEqual(s['budgets']['mission_dispatches_reserved'],2)
        self.assertEqual(s['attempts'][0]['terminal_authority'],authority)
        self.reopen();self.no_more()

    def test_repeated_source_reconciliation_does_not_regenerate_permission_or_rewrite_original_bytes(self):
        before=self.worker.state()['final_goal_correction']
        for _ in range(2):
            p=self.root/('reconcile_'+str(uuid.uuid4())+'.json');atomic_json(p,self.result)
            self.worker.finish_attempt(self.result,True,result_path=p)
            self.assertEqual(self.worker.state()['final_goal_correction'],before)
        self.reserve_correction();self.reopen();self.no_more()

    def test_successful_correction_promotes_only_the_last_original_index_and_keeps_strict_failure(self):
        a=self.reserve_correction();visits=copy.deepcopy(self.visits)
        visits[4]['passed']['snapshot']['last']['pose']['stamp_ns']=2_000_000_000
        self.worker.observe(a['parent_action_uuid'],SimpleNamespace(active=4,visits=visits))
        self.finish_correction(a,status=4,functional_status='PASS')
        s=self.worker.state();self.assertTrue(s['lifecycle'].startswith('COMPLETED'))
        self.assertEqual(s['progress']['accepted_prefix_count'],5);self.assertIsNone(s['progress']['next_index'])
        self.assertEqual(s['progress']['accepted_receipts'][:4],self.initial['progress']['accepted_receipts'])
        self.assertEqual(s['progress']['accepted_receipts'][4]['source_action_uuid'],a['parent_action_uuid'])
        self.assertEqual(s['final_goal_correction']['outcome'],'COMPLETE')
        self.assertEqual(s['result_flags']['strict_timing_status'],'FAIL');self.no_more()

    def test_different_known_failure_cannot_create_a_third_dispatch(self):
        a=self.reserve_correction();self.finish_correction(a,code=60002);s=self.worker.state()
        self.assertEqual((s['lifecycle'],s['hold_reason']),('NEEDS_TASK_DECISION','FINAL_GOAL_CORRECTION_FAILED'))
        self.assertEqual(s['attempt']['native_status'],6);self.assertEqual(s['attempt']['missed_waypoints'][0]['error_code'],60002)
        self.reopen();self.no_more()

    def test_unknown_60003_holds_its_original_reason_and_cancel_keeps_cancel_precedence(self):
        a=self.reserve_correction();result,path=self.finish_correction(a,code=60003)
        s=self.worker.state();self.assertEqual((s['lifecycle'],s['hold_reason']),('HOLD','RECOVERY_OWNERSHIP_RECONCILIATION_REQUIRED'))
        self.assertEqual(s['final_goal_correction']['outcome'],'HELD');self.no_more()
        # Operator intent acts on the same original UUID; native status6 remains
        # immutable even though task cancellation now takes precedence.
        self.worker.request_cancel(str(uuid.uuid4()),'Operator cancels this held correction')
        s=self.worker.state();self.assertEqual(s['lifecycle'],'CANCELED')
        self.assertEqual(s['attempt']['native_status'],6);self.assertEqual(s['attempt']['parent_action_uuid'],a['parent_action_uuid'])
        self.no_more()

    def test_not_sent_reservation_is_never_refunded_after_reopen(self):
        a=self.reserve_correction(ack=False);p=self.root/'not_sent.json'
        atomic_json(p,dict(action_uuid=a['parent_action_uuid'],status=None,send_goal_async_invoked=False,goals_sent=0))
        self.worker.record_not_sent(a['parent_action_uuid'],result_path=p,send_invoked=False)
        s=self.worker.state();self.assertEqual(s['final_goal_correction']['outcome'],'NOT_SENT_AFTER_RESERVATION')
        self.assertEqual(s['hold_reason'],'NOT_SENT_AFTER_INTENT_NO_AUTOMATIC_REDISPATCH')
        self.reopen();self.no_more();self.assertEqual(self.worker.state()['budgets']['mission_dispatches_reserved'],2)

    def test_budget_failure_before_reserve_leaves_the_available_permission_unconsumed(self):
        b=self.worker.state()['budgets']
        with patch('mission_state.boot_clock',return_value=(b['deadline_boot_id'],b['deadline_boottime_sec']+1.)):
            with self.assertRaisesRegex(Hold,'FAILED_BUDGET'):self.reserve_correction()
        s=self.worker.state();self.assertIsNone(s['final_goal_correction']['consumed_attempt_id'])
        self.assertEqual(s['budgets']['mission_dispatches_reserved'],1)

    def test_unknown_correction_terminal_retains_unknown_uuid_and_used_allowance(self):
        a=self.reserve_correction();self.finish_correction(a,status=0)
        s=self.worker.state();self.assertEqual((s['lifecycle'],s['hold_reason']),('HOLD','UNKNOWN_DISPATCH'))
        self.assertEqual(s['final_goal_correction']['outcome'],'HELD')
        self.reopen();self.no_more()

    def test_correction_native_cancel_keeps_original_cancel_status_and_used_allowance(self):
        a=self.reserve_correction();self.finish_correction(a,status=5,stop_signals=[signal.SIGINT])
        s=self.worker.state();self.assertEqual((s['lifecycle'],s['hold_reason']),('CANCELED','OPERATOR_CANCEL'))
        self.assertEqual(s['attempt']['native_status'],5);self.no_more()


if __name__=='__main__':unittest.main()
