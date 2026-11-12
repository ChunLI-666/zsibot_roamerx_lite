"""Actual nested control functions, fake time/truth only; no ROS/process/socket."""
import math,unittest
from types import SimpleNamespace as N
import full_scene_mapping_contract as c
from test_startup_v6 import actual_function,install
from test_v4_core import Clock,model,truth


def fixture(profile=lambda elapsed:0.,step=.04,command=(0.,0.,0.),age=.005,fail=None):
    clk=Clock();begin=clk.monotonic();rows=[]
    ns=dict(time=clk,contract=c,model=model(),relay=N(command=command),active_lateral='retained')
    def tick(*args,**kwargs):
        rows.append((clk.monotonic(),args,kwargs));clk.sleep(step)
        if fail:raise c.Hold(fail)
        yaw=profile(clk.monotonic()-begin)
        return truth(clk,age=age,yaw=yaw),yaw
    ns['tick']=tick;install(actual_function('stable'),ns)
    return ns,clk,begin,rows


class JointHeadingStop(unittest.TestCase):
    def test_overshoot_then_recovery_requires_new_joint_continuous_stop(self):
        ns,k,start,rows=fixture(lambda t:.014 if t<.12 else .021 if t<.48 else .014)
        sample,yaw=ns['stable'](.3,8.,heading=(0.,.0155),deadline_steady=start+8.)
        self.assertGreaterEqual(k.monotonic()-start,.80)
        self.assertLess(k.monotonic()-start,.9)
        self.assertLessEqual(abs(yaw),.0155);self.assertIsNone(ns['active_lateral'])
        self.assertTrue(all(args==() and kwargs=={} for _,args,kwargs in rows))
    def test_short_inband_windows_never_accumulate_across_excursions(self):
        ns,k,start,rows=fixture(lambda t:0. if (t%.24)<.16 else .03)
        with self.assertRaises(c.Hold):ns['stable'](.3,.85,heading=(0.,.0155),deadline_steady=start+.85)
        self.assertEqual(ns['active_lateral'],'retained');self.assertLess(k.monotonic()-start,.90)
    def test_never_inband_stops_at_original_window_without_positive_or_retry(self):
        ns,k,start,rows=fixture(lambda t:.021)
        with self.assertRaises(c.Hold):ns['stable'](.3,.4,heading=(0.,.0155),deadline_steady=start+.4)
        self.assertLessEqual(k.monotonic()-start,.44);self.assertEqual(ns['active_lateral'],'retained')
        self.assertTrue(all(args==() and kwargs=={} for _,args,kwargs in rows))
    def test_inband_alone_does_not_bypass_original_measured_stop_or_freshness(self):
        for opts in [dict(command=(.4,0.,0.)),dict(age=.1)]:
            ns,k,start,rows=fixture(**opts)
            with self.assertRaises(c.Hold):ns['stable'](.3,.4,heading=(0.,.0155),deadline_steady=start+.4)
            self.assertEqual(ns['active_lateral'],'retained')
    def test_short_absolute_deadline_and_delayed_tick_cannot_report_success(self):
        for step,timeout,absolute in [(.04,8.,.2),(.5,.4,.4)]:
            ns,k,start,rows=fixture(step=step)
            with self.assertRaises(c.Hold):ns['stable'](.3,timeout,heading=(0.,.0155),deadline_steady=start+absolute)
            self.assertEqual(ns['active_lateral'],'retained')
    def test_health_fault_is_immediate_and_not_reclassified_as_heading_wait(self):
        ns,k,start,rows=fixture(fail='foreign UDP source')
        with self.assertRaisesRegex(c.Hold,'foreign UDP source'):ns['stable'](.3,8.,heading=(0.,.0155),deadline_steady=start+8.)
        self.assertEqual(len(rows),1)
    def test_ordinary_stop_without_heading_keeps_original_predicate(self):
        ns,k,start,rows=fixture(lambda t:.3)
        sample,yaw=ns['stable'](.3,8.)
        self.assertGreaterEqual(k.monotonic()-start,.3);self.assertLess(k.monotonic()-start,.4)
        self.assertEqual(yaw,.3);self.assertIsNone(ns['active_lateral'])
    def test_rotate_uses_same_tolerance_and_single_original8s_or_shorter_deadline(self):
        for shorter in [None,.7]:
            ns,k,start,rows=fixture();seen=[]
            def stable(seconds,timeout,**kw):
                seen.append((seconds,timeout,kw));return truth(k),0.
            ns.update(math=math,health=lambda:(truth(k),0.),stable=stable)
            rotate=install(actual_function('rotate_to'),ns)
            rotate(0.,.0155,deadline_steady=None if shorter is None else start+shorter)
            self.assertEqual(len(seen),1);seconds,timeout,kw=seen[0]
            self.assertEqual(seconds,c.STOP_STABLE);self.assertLessEqual(timeout,8. if shorter is None else shorter)
            self.assertEqual(kw['heading'],(0.,.0155));self.assertEqual(kw['deadline_steady'],start+(8. if shorter is None else shorter))
            self.assertFalse(rows)
    def test_rotate_positive_phase_keeps_original_command_and_terminal_binding(self):
        ns,k,start,rows=fixture();calls=[];state={'yaw':-.1}
        def tick(vx=0.,wz=0.,**kw):calls.append((vx,wz,kw));state['yaw']=0.;k.sleep(.04);return truth(k),0.
        ns.update(math=math,health=lambda:(truth(k,yaw=state['yaw']),state['yaw']),tick=tick,stable=lambda *a,**kw:(truth(k),0.))
        install(actual_function('rotate_to'),ns)(0.,.0155,deadline_steady=start+2.,terminal_deadline_steady=start+2.)
        self.assertEqual(calls,[(0.,c.MAX_WZ,{'terminal_deadline_steady':start+2.})])

if __name__=='__main__':unittest.main(verbosity=2)
