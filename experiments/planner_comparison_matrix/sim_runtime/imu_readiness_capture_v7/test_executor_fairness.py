"""Execute actual installed Executor iterator caching without ROS context/Node."""
import ast
import io
from pathlib import Path
from types import SimpleNamespace as N
import unittest
from rclpy.executors import Executor
from readiness_observer import Observer

class FakeIteratorOwner:
    def __init__(self):self._cb_iter=None;self._last_args=None;self._last_kwargs=None;self.generations=0
    def _wait_for_ready_callbacks(self,*args,**kwargs):
        self.generations+=1
        for topic in ['raw_imu','derived_imu','camera','camera_info']:yield(topic,None,None)
class ExecutorFairness(unittest.TestCase):
    def test_actual_Jazzy_changed_timeout_restarts_earlier_subscription(self):
        owner=FakeIteratorOwner();cond=object()
        topics=[Executor.wait_for_ready_callbacks(owner,t,None,cond)[0] for t in [.010,.009,.008,.007]]
        self.assertEqual(topics,['raw_imu']*4);self.assertEqual(owner.generations,4)
    def test_actual_Jazzy_fixed_timeout_retains_all_ready_subscriptions(self):
        owner=FakeIteratorOwner();cond=object()
        topics=[Executor.wait_for_ready_callbacks(owner,0.,None,cond)[0] for _ in range(4)]
        self.assertEqual(topics,['raw_imu','derived_imu','camera','camera_info']);self.assertEqual(owner.generations,1)
    def test_constructor_adds_only_own_node_once_and_spin_uses_fixed_zero(self):
        tree=ast.parse((Path(__file__).parent/'readiness_observer.py').read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        init=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
        spin=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='spin')
        adds=[n for n in ast.walk(init) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='add_node']
        self.assertEqual(len(adds),1);self.assertEqual(ast.unparse(adds[0].args[0]),'self.node')
        calls=[n for n in ast.walk(spin) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='spin_once']
        self.assertEqual(len(calls),1);self.assertEqual(ast.unparse(calls[0].func),'self.executor.spin_once');self.assertEqual(calls[0].keywords[0].value.value,0.)
    def close_fixture(self,stopped):
        events=[];obj=object.__new__(Observer);obj.stream=io.StringIO();obj.graph_stream=io.StringIO()
        obj.node=N(destroy_node=lambda:events.append('destroy_own_node'))
        obj.executor=N(remove_node=lambda n:events.append(('remove',n)),shutdown=lambda timeout_sec:events.append(('shutdown',timeout_sec)) or stopped)
        obj.rclpy=N(shutdown=lambda:events.append('shutdown_context'))
        return obj,events
    def test_only_own_executor_and_node_are_closed(self):
        obj,events=self.close_fixture(True);obj.close()
        self.assertEqual(events,[('remove',obj.node),('shutdown',0.),'destroy_own_node','shutdown_context'])
        self.assertTrue(obj.stream.closed);self.assertTrue(obj.graph_stream.closed)
    def test_executor_shutdown_failure_still_closes_node_and_context(self):
        obj,events=self.close_fixture(False)
        with self.assertRaisesRegex(RuntimeError,'did not stop'):obj.close()
        self.assertEqual(events[-2:],['destroy_own_node','shutdown_context'])
if __name__=='__main__':unittest.main(verbosity=2)
