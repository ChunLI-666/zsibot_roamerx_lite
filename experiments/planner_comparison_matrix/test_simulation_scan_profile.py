"""Pure checks for explicit simulated scan latency; other source guards stay strict."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

SOURCE=Path(__file__).with_name('matrix_navigation.py')
tree=ast.parse(SOURCE.read_text())
profile=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='simulation_scan_latency_profile')
health=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='source_health')
def stamp(age):
    value=1_000_000_000-round(age*1e9)
    return NS(header=NS(stamp=NS(sec=value//1_000_000_000,nanosec=value%1_000_000_000)))

class SimulationScanProfileTests(unittest.TestCase):
    def environment(self,limit=.3,scan_age=.4,odom_age=.01,tf_age=.01,status_receive_age=.01):
        latest={topic:(.99,stamp(age)) for topic,age in [('odom',odom_age),('scan',scan_age),('camera',.1)]}
        latest.update({'/lightning/loc_status':(1.-status_receive_age,NS(data=1)),
                       '/lightning/pose_valid':(.99,NS(data=True))})
        ns=dict(args=NS(odom_topic='odom',scan_topic='scan',camera_topic='camera',simulation_scan_max_age_sec=limit),
                latest=latest,time=NS(monotonic=lambda:1.),node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000))),
                tf=NS(lookup_transform=lambda *args:stamp(tf_age)),Time=lambda:None,health_tf_lookups={})
        exec(compile(ast.Module(body=[profile,health],type_ignores=[]),str(SOURCE),'exec'),ns)
        return ns
    def test_default_profile_and_invalid_limits(self):
        f=self.environment()['simulation_scan_latency_profile']
        self.assertEqual(f()['operational_max_age_sec'],.3)
        self.assertEqual(f(.5)['strict_max_age_sec'],.3)
        self.assertEqual(f(.5)['mode'],'simulation_500ms')
        for value in [float('nan'),float('inf'),.299,.501,1.]:
            with self.assertRaises(ValueError):f(value)
    def test_strict_default_rejects_400ms_scan(self):
        with self.assertRaisesRegex(RuntimeError,'Stale/future stamp scan'):self.environment()['source_health']()
    def test_optin_accepts_499ms_scan(self):self.environment(.5,.499)['source_health']()
    def test_optin_rejects_500ms_plus_one_ns(self):
        with self.assertRaisesRegex(RuntimeError,'Stale/future stamp scan'):self.environment(.5,.500000001)['source_health']()
    def test_future_guard_is_not_extended(self):
        with self.assertRaisesRegex(RuntimeError,'Stale/future stamp scan'):self.environment(.5,-.050000001)['source_health']()
    def test_odom_guard_remains_300ms(self):
        with self.assertRaisesRegex(RuntimeError,'Stale/future stamp odom'):self.environment(.5,.4,.301)['source_health']()
    def test_actual_tf_guard_remains_300ms(self):
        with self.assertRaisesRegex(RuntimeError,'Stale/future actual TF'):self.environment(.5,.4,tf_age=.301)['source_health']()
    def test_localization_receive_guard_remains_200ms(self):
        with self.assertRaisesRegex(RuntimeError,'Stale receive /lightning/loc_status'):self.environment(.5,.4,status_receive_age=.201)['source_health']()

if __name__=='__main__':unittest.main()
