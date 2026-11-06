"""No-data iterator transition must not cap high-rate input at100Hz outer rate."""
import ast
from pathlib import Path
from types import SimpleNamespace as N
import unittest
from unittest import mock
import readiness_observer as module
from test_bounded_spin import Clock,bare
class IteratorBoundary(unittest.TestCase):
    def test_alternating_handler_and_no_handler_keeps_draining_to32(self):
        def run(method):
            clock=Clock();obj=bare(None,clock);calls=[]
            def dispatch(node,timeout_sec):
                calls.append(timeout_sec);clock.now+=100_000
                if len(calls)%2:obj.validator.received['fixture']+=1
            obj.rclpy=N(spin_once=dispatch)
            with mock.patch.object(module,'time',clock):method(obj)
            return obj.validator.received['fixture'],clock.now,calls
        old=Path(__file__).resolve().parents[1]/'sim_imu_adapter_readiness_v7/review_source_v7/readiness_observer.py'
        fn=next(n for n in ast.walk(ast.parse(old.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='spin')
        ns={'time':None};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<exact oldv7>','exec'),ns)
        def old_method(obj):
            ns['time']=module.time;ns['spin'](obj)
        previous=run(old_method);current=run(module.Observer.spin)
        self.assertEqual(previous[0],1);self.assertEqual(current[0],32)
        self.assertLessEqual(current[1],10_000_000);self.assertEqual(set(current[2]),{0.})
if __name__=='__main__':unittest.main(verbosity=2)
