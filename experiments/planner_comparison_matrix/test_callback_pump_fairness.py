"""Installed Jazzy ready-iterator + actual pump methods, pure fake clock. No ROS init."""
import ast
import copy
from collections import Counter
import hashlib
from pathlib import Path
from types import MethodType,SimpleNamespace
import unittest

HERE=Path(__file__).resolve().parent
MATRIX=HERE/'matrix_navigation.py'
BATCH=HERE/'goal_pilot_source/waypoint_batch.py'
PRIMARY=Path('/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/executors.py')

def method(path,name,cls=None):
    tree=ast.parse(path.read_text())
    if cls is not None:tree=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==cls)
    return copy.deepcopy(next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name))
def compiled(node,namespace):
    node=copy.deepcopy(node);node.returns=None
    for a in [*node.args.posonlyargs,*node.args.args,*node.args.kwonlyargs]:a.annotation=None
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),'<actual-source-AST>','exec'),namespace)
    return namespace[node.name]
class Clock:
    def __init__(self):self.ns=100_000_000_000
    def monotonic(self):return self.ns/1e9
    def monotonic_ns(self):return self.ns
    def thread_time_ns(self):return self.ns//10
class Ready:
    def __init__(self,wait):
        self._cb_iter=self._last_args=self._last_kwargs=None;self.rebuilds=0;self.counts=Counter();self.wait=wait
    def _wait_for_ready_callbacks(self,*args,**kwargs):
        self.rebuilds+=1
        for topic in ['high_rate_odom','scan','status','camera']:
            yield (lambda key=topic:self.counts.update([key])),topic,None
    def spin(self,timeout):
        callback,_,_=self.wait(self,timeout,None,WAIT_CONDITION);callback()
WAIT_CONDITION=lambda:False

@unittest.skipUnless(PRIMARY.exists(),'Installed Jazzy Python source unavailable')
class FairnessTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.wait=staticmethod(compiled(method(PRIMARY,'wait_for_ready_callbacks'),{}))
    def state(self,idle=False):
        clock=Clock();ready=Ready(self.wait);timeouts=[];deadline=[]
        def spin(node,timeout_sec):
            timeouts.append(timeout_sec)
            if not idle:ready.spin(timeout_sec);node.counts.update(ready.counts-ready.previous) if hasattr(ready,'previous') else node.counts.update(ready.counts);ready.previous=ready.counts.copy()
            clock.ns+=int(timeout_sec*1e9) if idle else 500_000
        ros=SimpleNamespace(ok=lambda:True,spin_once=spin)
        node=SimpleNamespace(counts=Counter())
        namespace=dict(time=clock,rclpy=ros,node=node,deadline_guard=SimpleNamespace(check_live=lambda:deadline.append(clock.ns)),stopped=False,shutdown_started=False)
        return clock,ready,timeouts,deadline,node,namespace
    def test_installed_source_identity_and_changing_timeout_restarts_at_odom(self):
        self.assertEqual(hashlib.sha256(PRIMARY.read_bytes()).hexdigest(),'46f052cf43e1fa1040af235f3508a0a8c00d98a2ab59b8dcaed9aceb3921a452')
        ready=Ready(self.wait)
        for timeout in [.020,.019,.018,.017]:ready.spin(timeout)
        self.assertEqual(ready.counts,dict(high_rate_odom=4));self.assertEqual(ready.rebuilds,4)
        ready=Ready(self.wait)
        for timeout in [.002]*4:ready.spin(timeout)
        self.assertEqual(ready.counts,dict(high_rate_odom=1,scan=1,status=1,camera=1));self.assertEqual(ready.rebuilds,1)
    def test_actual_producer_pump_services_later_sources_old_tail_expression_starves(self):
        for old in [False,True]:
            clock,ready,timeouts,deadline,node,namespace=self.state();pump=method(MATRIX,'pump')
            if old:
                call=next(n for n in ast.walk(pump) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='spin_once')
                next(k for k in call.keywords if k.arg=='timeout_sec').value=ast.parse('min(.02,max(0.,end-time.monotonic()))',mode='eval').body
            compiled(pump,namespace)(.02)
            self.assertEqual(len(deadline),len(timeouts))
            if old:self.assertEqual(set(ready.counts),{'high_rate_odom'})
            else:
                self.assertEqual(set(ready.counts),{'high_rate_odom','scan','status','camera'})
                self.assertEqual(set(timeouts),{.002});self.assertLess(ready.rebuilds,len(timeouts))
    def test_producer_idle_wait_exceeds_window_by_at_most_one_two_ms_wait(self):
        clock,ready,timeouts,deadline,node,namespace=self.state(idle=True);start=clock.ns
        compiled(method(MATRIX,'pump'),namespace)(.0201)
        self.assertGreaterEqual(clock.ns-start,20_100_000);self.assertLess(clock.ns-start,22_100_000)
        self.assertEqual(set(timeouts),{.002});self.assertEqual(len(deadline),len(timeouts))
    def test_actual_runtime_pump_has_constant_timeout_and_advances_ready_iterator(self):
        clock,ready,timeouts,deadline,node,namespace=self.state()
        node.args=SimpleNamespace(timeout=100.,cancel_after=None);node.cancel_requested=False;node.goal_sent_steady=clock.monotonic();node.check_interruption=lambda:None
        pump=compiled(method(BATCH,'pump_runtime_inputs','WaypointPilot'),namespace)
        result=pump(node,SimpleNamespace(done=lambda:False))
        self.assertEqual(set(timeouts),{.002});self.assertEqual(set(ready.counts),{'high_rate_odom','scan','status','camera'})
        self.assertGreater(result['observed_count_deltas']['scan'],0)
    def test_runtime_deadline_checked_before_each_fixed_wait_bounded_idle_tail(self):
        clock,ready,timeouts,deadline,node,namespace=self.state(idle=True)
        node.args=SimpleNamespace(timeout=100.,cancel_after=.001);node.cancel_requested=False;node.goal_sent_steady=clock.monotonic();node.check_interruption=lambda:None
        result=compiled(method(BATCH,'pump_runtime_inputs','WaypointPilot'),namespace)(node,SimpleNamespace(done=lambda:False))
        self.assertEqual(timeouts,[.002]);self.assertEqual(result['stop_reason'],'existing_deadline');self.assertEqual(result['wall_ns'],2_000_000)
    def test_overrun_branch_and_confirmation_use_fixed_literal_timeouts(self):
        run=method(MATRIX,'run');loop=next(n for n in ast.walk(run) if isinstance(n,ast.While) and ast.unparse(n.test)=='not stopped')
        branch=next(n for n in loop.body if isinstance(n,ast.If) and ast.unparse(n.test)=='remaining > 0.0')
        self.assertEqual(ast.unparse(branch.orelse[0]),'rclpy.spin_once(node, timeout_sec=0.002)')
        confirmation=method(BATCH,'confirm_scan_aging','WaypointPilot')
        calls=[n for n in ast.walk(confirmation) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='spin_once']
        self.assertEqual(len(calls),1);self.assertEqual(ast.literal_eval(next(k.value for k in calls[0].keywords if k.arg=='timeout_sec')),.005)
    def test_confirmation_preserves_original_outcome_deadline(self):
        confirmation=method(BATCH,'confirm_scan_aging','WaypointPilot')
        call=next(n for n in ast.walk(confirmation) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='spin_once')
        next(k for k in call.keywords if k.arg=='timeout_sec').value=ast.parse('min(.005, remaining_ns/1e9)',mode='eval').body
        text=ast.unparse(confirmation)
        self.assertIn('scan_aging_confirmation_outcome(current_issues, current_snapshot, original_stamp',text)
        self.assertIn('checked_ns - started_ns, budget_ns',text)
        self.assertIn('if remaining_ns <= 0',text)

if __name__=='__main__':unittest.main(verbosity=2)
