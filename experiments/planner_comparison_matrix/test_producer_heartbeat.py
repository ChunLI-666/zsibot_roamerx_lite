"""Pure worker/cadence/write contracts. No ROS imports, nodes, contexts or goals."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

HERE=Path(__file__).resolve().parent
SOURCE=HERE/'matrix_navigation.py'
EXPECTED_UNCHANGED_AST={'source_health': '398b8d1ae30452c19d2bc7f83f3f7efc855a9a55ee486c75382c33807c7c7b0a', 'native_bt_self_client_valid': '4ad3f6a6e93ce2d5503bc6ede0eeb0f91e0fd26040b06a2b7f0b785cf1fd0be2', 'waypoint_service_ownership': '8d9a68456a0c7d956fd46889afbfec7f5c0d47a5301c6d855c7265824dde9e32'}
names={'write','phase_mark','next_heartbeat_deadline','OwnershipAuditWorker'}
root=ast.parse(SOURCE.read_text());nodes=[n for n in root.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names]
ns=dict(time=time,threading=threading,pathlib=__import__('pathlib'),json=json)
exec(compile(ast.Module(body=nodes,type_ignores=[]),str(SOURCE),'exec'),ns)
Worker=ns['OwnershipAuditWorker'];next_deadline=ns['next_heartbeat_deadline']

class ProducerHeartbeatTests(unittest.TestCase):
    def seed(self):
        return dict(checked_unix_ns=time.time_ns(),exclusive_nav_client=True,status_publishers_match=True,
          follow_waypoints_active=[dict(uuid='a'*32,status=2)],navigate_to_pose_active=[],proof=dict(immutable=True))
    def blocked(self,seed):
        entered=threading.Event();release=threading.Event()
        def job():entered.set();release.wait(2.);return seed
        worker=Worker(job,seed);self.assertTrue(entered.wait(.5))
        def close():worker.request_stop();release.set();self.assertTrue(worker.join())
        self.addCleanup(close);return worker,release
    def await_sequence(self,worker,minimum=1):
        deadline=time.monotonic()+.5
        while worker.state()['sequence']<minimum and time.monotonic()<deadline:time.sleep(.001)
        self.assertGreaterEqual(worker.state()['sequence'],minimum)
    def test_worker_cannot_block_heartbeat_or_restamp_seed(self):
        seed=self.seed();worker,_=self.blocked(seed);start=time.monotonic();frames=[]
        for age in [0,50_000_000,100_000_000,200_000_000,450_000_000]:
            result,sequence=worker.snapshot(seed['checked_unix_ns']+age)
            frames.append(result['checked_unix_ns']);self.assertEqual(sequence,0)
        self.assertLess(time.monotonic()-start,.1);self.assertEqual(frames,[seed['checked_unix_ns']]*5)
        self.assertTrue(worker.state()['running'])
    def test_same_original_500ms_and_50ms_age_bounds(self):
        seed=self.seed();worker,_=self.blocked(seed)
        for age,allowed in [(500_000_000,True),(500_000_001,False),(-50_000_000,True),(-50_000_001,False)]:
            with self.subTest(age=age):
                if allowed:self.assertIs(worker.snapshot(seed['checked_unix_ns']+age)[0],seed)
                else:
                    with self.assertRaisesRegex(RuntimeError,'stale or future'):worker.snapshot(seed['checked_unix_ns']+age)
    def test_READY_never_outlives_original_ownership_expiry(self):
        seed=self.seed();worker,_=self.blocked(seed)
        for age in [0,50_000_000,300_000_000,499_999_999]:
            issued=seed['checked_unix_ns']+age;result,_=worker.snapshot(issued)
            expiry=min(issued+500_000_000,result['checked_unix_ns']+500_000_000)
            self.assertEqual(expiry,seed['checked_unix_ns']+500_000_000)
        with self.assertRaises(RuntimeError):worker.snapshot(seed['checked_unix_ns']+500_000_001)
    def test_audit_error_is_explicit_even_if_previous_result_was_fresh(self):
        seed=self.seed()
        def fail():raise RuntimeError('foreign graph audit')
        worker=Worker(fail,seed);self.addCleanup(lambda:(worker.request_stop(),worker.join()))
        self.await_sequence(worker)
        with self.assertRaisesRegex(RuntimeError,'Ownership audit failed: RuntimeError: foreign graph audit'):
            worker.snapshot(time.time_ns())
    def test_latest_full_result_preserves_original_action_uuid_and_proof(self):
        seed=self.seed();fresh=copy.deepcopy(seed);fresh['checked_unix_ns']+=1;fresh['follow_waypoints_active'][0]['uuid']='f'*32
        worker=Worker(lambda:fresh,seed);self.addCleanup(lambda:(worker.request_stop(),worker.join()))
        self.await_sequence(worker);result,sequence=worker.snapshot(time.time_ns())
        self.assertIs(result,fresh);self.assertEqual(result['follow_waypoints_active'][0]['uuid'],'f'*32)
        self.assertEqual(result['proof'],seed['proof']);self.assertGreaterEqual(sequence,1)
    def test_one_worker_has_no_parallel_audit_or_unbounded_queue(self):
        seed=self.seed();active=0;maximum=0;guard=threading.Lock()
        def job():
            nonlocal active,maximum
            with guard:active+=1;maximum=max(maximum,active)
            time.sleep(.01)
            with guard:active-=1
            return dict(seed,checked_unix_ns=time.time_ns())
        worker=Worker(job,seed,period=.015);self.addCleanup(lambda:(worker.request_stop(),worker.join()))
        self.await_sequence(worker,3);self.assertEqual(maximum,1);self.assertFalse(hasattr(worker,'queue'))
    def test_wall_and_thread_cpu_audit_costs_are_distinct(self):
        seed=self.seed()
        def job():time.sleep(.04);return dict(seed,checked_unix_ns=time.time_ns())
        worker=Worker(job,seed);self.addCleanup(lambda:(worker.request_stop(),worker.join()))
        self.await_sequence(worker);cost=worker.state()['timing']['full_audit']
        self.assertGreaterEqual(cost['wall_ns'],30_000_000);self.assertLess(cost['thread_cpu_ns'],cost['wall_ns'])
    def test_absolute_deadline_skips_missed_slots_without_adding_work_duration(self):
        self.assertAlmostEqual(next_deadline(100.,100.007),100.05)
        self.assertAlmostEqual(next_deadline(100.05,100.44),100.45)
        self.assertAlmostEqual(next_deadline(100.,100.05),100.10)
    def test_atomic_compact_READY_preserves_all_values(self):
        document=dict(ready=True,waypoint_ownership=self.seed(),graph=['x'*100 for _ in range(100)])
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'ready.json';ns['write'](path,document,compact=True)
            self.assertEqual(json.loads(path.read_bytes()),document)
            self.assertLess(path.stat().st_size,len(json.dumps(document,indent=2))+1)
            self.assertFalse(path.with_suffix('.json.tmp').exists())
    def test_source_health_only_adds_the_explicit_scan_profile(self):
        def find(path,name):return next(n for n in ast.walk(ast.parse(path.read_text())) if isinstance(n,ast.FunctionDef) and n.name==name)
        # Restore only the newly reviewed scan limit to its original default.
        # Strip only the named TF diagnostic writes/clock probes. The original
        # health predicates must still match the historical AST fingerprint.
        class RemoveTFDiagnostics(ast.NodeTransformer):
            removed=0
            def visit_Expr(self,node):
                if ast.unparse(node)=="health_tf_lookups.clear()":
                    self.removed+=1;return None
                return self.generic_visit(node)
            def visit_Assign(self,node):
                target=node.targets[0] if len(node.targets)==1 else None
                clock_probe=(isinstance(target,ast.Name) and target.id in ('began','ended')
                    and ast.unparse(node.value)=='node.get_clock().now().nanoseconds')
                diagnostic=(isinstance(target,ast.Subscript) and isinstance(target.value,ast.Name)
                    and target.value.id=='health_tf_lookups')
                if clock_probe or diagnostic:
                    self.removed+=1;return None
                return self.generic_visit(node)
        class StrictScanDefault(ast.NodeTransformer):
            count=0
            def visit_Call(self,node):
                if ast.unparse(node)=="getattr(args, 'simulation_scan_max_age_sec', 0.3)":
                    self.count+=1
                    return ast.Constant(value=.3)
                return self.generic_visit(node)
        normalize=StrictScanDefault()
        diagnostics=RemoveTFDiagnostics()
        without_diagnostics=diagnostics.visit(copy.deepcopy(find(SOURCE,'source_health')))
        self.assertEqual(diagnostics.removed,4)
        original_default=normalize.visit(without_diagnostics)
        self.assertEqual(normalize.count,1)
        # The approved recording-only camera split is the sole predicate removal.
        sources=next(n for n in ast.walk(original_default) if isinstance(n,ast.For)
                     and ast.unparse(n.target)=='(topic, limit)')
        self.assertEqual(len(sources.iter.elts),4)
        sources.iter.elts.append(ast.parse('(args.camera_topic,.5)',mode='eval').body)
        self.assertEqual(hashlib.sha256(ast.dump(original_default).encode()).hexdigest(),EXPECTED_UNCHANGED_AST['source_health'])
        self.assertEqual(hashlib.sha256(ast.dump(find(SOURCE,'native_bt_self_client_valid')).encode()).hexdigest(),EXPECTED_UNCHANGED_AST['native_bt_self_client_valid'])
        self.assertEqual(hashlib.sha256(ast.dump(find(SOURCE,'waypoint_service_ownership')).encode()).hexdigest(),EXPECTED_UNCHANGED_AST['waypoint_service_ownership'])
    def test_every_overrun_loop_still_checks_deadline_and_startup_rechecks_health(self):
        run=next(n for n in root.body if isinstance(n,ast.FunctionDef) and n.name=='run')
        loop=next(n for n in ast.walk(run) if isinstance(n,ast.While) and ast.unparse(n.test)=='not stopped')
        self.assertIsInstance(loop.body[0],ast.If)
        self.assertIn('deadline_guard.check_live()',ast.unparse(loop.body[0]))
        text=SOURCE.read_text();self.assertIn('pump(.02);source_health()',text)
        self.assertIn("ownership_audit_period_sec=.2",text)
        self.assertIn("node.destroy_subscription(subscription)",text)
        self.assertIn("recorder_continuous_costmap_topics_unchanged=True",text)
    def test_stop_event_releases_worker_without_post_shutdown_new_audit(self):
        seed=self.seed();worker,release=self.blocked(seed);worker.request_stop();release.set()
        self.assertTrue(worker.join());sequence=worker.state()['sequence'];time.sleep(.025)
        self.assertEqual(worker.state()['sequence'],sequence)

if __name__=='__main__':unittest.main(verbosity=2)
