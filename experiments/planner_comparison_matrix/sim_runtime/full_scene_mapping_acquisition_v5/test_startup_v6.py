"""Pure delayed-READY/fatal-cleanup regressions; all child/FD APIs mocked."""
import ast,copy
from pathlib import Path
from types import SimpleNamespace as N
import unittest
from unittest import mock
import full_scene_mapping as owner
import full_scene_mapping_contract as c
from test_v4_core import Clock,model,truth

HERE=Path(__file__).resolve().parent
def actual_function(name):
    fn=copy.deepcopy(next(n for n in ast.walk(ast.parse((HERE/'full_scene_mapping.py').read_text())) if isinstance(n,ast.FunctionDef) and n.name==name))
    fn.body=[ast.copy_location(ast.Global(names=n.names),n) if isinstance(n,ast.Nonlocal) else n for n in fn.body]
    return fn
def install(fn,namespace):
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'<actual v6 function>','exec'),namespace)
    return namespace[fn.name]

class Startup(unittest.TestCase):
    def start_fixture(self,pump=True,ready_after=.30,partial=False,pump_delay=.01,interrupt=False,exit_after=None):
        clock=Clock();start=clock.monotonic();pumps=[];reads=[0];events=[]
        process=N(pid=17,stdin=N(fileno=lambda:10),stdout=N(fileno=lambda:11),poll=lambda:0 if exit_after is not None and clock.monotonic()-start>=exit_after else None)
        relay=owner.OwnedRelay.__new__(owner.OwnedRelay);relay.argv=['fixture'];relay.run_id='1'*32;relay.output=Path('/fixture');relay.log=object();relay.close=mock.Mock();relay.frozen=Path('/fixture/relay.py');relay.source_sha256='a'*64
        def dispatch():
            pumps.append(clock.monotonic());clock.sleep(pump_delay)
            if interrupt:raise InterruptedError('fixture startup stop requested')
        def select_(r,w,e,timeout):
            events.append(('select',timeout))
            if timeout:clock.sleep(timeout)
            available=clock.monotonic()-start>=ready_after+(0.1 if partial and reads[0] else 0.)
            return (r if available else []),[],[]
        def read_(fd,count):
            reads[0]+=1
            if partial:return b'MAPPING_LATERAL_RELAY_' if reads[0]==1 else b'RECT_V5_READY 127.0.0.1:7447\n'
            return b'MAPPING_LATERAL_RELAY_RECT_V5_READY 127.0.0.1:7447\n'
        with mock.patch.object(owner,'time',clock),mock.patch.object(owner.subprocess,'Popen',return_value=process),\
            mock.patch.object(owner.os,'pidfd_open',side_effect=lambda pid:events.append(('pidfd',pid)) or 19),\
            mock.patch.object(owner.os,'set_blocking'),mock.patch.object(owner.os,'read',side_effect=read_),\
            mock.patch.object(owner.select,'select',side_effect=select_),mock.patch.object(c,'process_identity',return_value={'argv':['fixture']}),mock.patch.object(c,'sha',return_value='a'*64):
            error=None
            try:relay.start(dispatch if pump else None)
            except (c.Hold,InterruptedError) as exc:error=exc
        return clock.monotonic()-start,pumps,events,relay,error
    def test_delayed_and_partial_READY_keep_callback_service_within_existing_gap(self):
        for partial in [False,True]:
            elapsed,pumps,events,relay,error=self.start_fixture(partial=partial)
            self.assertIsNone(error);self.assertGreaterEqual(len(pumps),29);self.assertGreaterEqual(elapsed,.30)
            self.assertLessEqual(max(b-a for a,b in zip(pumps,pumps[1:])),.010000001)
            self.assertEqual(events[0],('pidfd',17));self.assertFalse(relay.close.called)
        elapsed,pumps,events,relay,error=self.start_fixture(pump=False)
        self.assertIsNone(error);self.assertFalse(pumps);self.assertGreaterEqual(elapsed,.30)
        # The old synchronous wait leaves a >.1s accepted-input gap; no gate threshold changes here.
    def test_three_second_deadline_closes_owned_child_without_new_retry(self):
        elapsed,pumps,events,relay,error=self.start_fixture(ready_after=10.)
        self.assertIsInstance(error,c.Hold);relay.close.assert_called_once();self.assertLessEqual(elapsed,3.010000001)
    def test_late_READY_slow_callback_early_exit_and_interrupt_close_without_accept(self):
        for options in [dict(ready_after=3.),dict(pump=False,ready_after=3.),dict(ready_after=0.,pump_delay=3.1),dict(ready_after=1.,exit_after=.2),dict(interrupt=True)]:
            elapsed,pumps,events,relay,error=self.start_fixture(**options)
            self.assertIsNotNone(error);relay.close.assert_called_once()

class FatalCleanup(unittest.TestCase):
    def callback_namespace(self,clock):
        events=[];state=dict(imu_gate_fault=None,time=clock,raw_imu_gate=N(inspect=lambda *a:dict(accepted=False,fatal=True,reason='accepted_header_gap_exceeded')),
            emit=lambda *a,**k:events.append(k),contract=c,deserialize_message=mock.Mock(side_effect=AssertionError('fatal raw must not decode/remember')),Imu=object,imu=mock.Mock())
        callback=install(actual_function('raw_imu'),state)
        return callback,state,events
    def test_fatal_raw_latches_without_throw_or_refresh_and_cleanup_still_observes_GT(self):
        clock=Clock();callback,state,events=self.callback_namespace(clock);callback(b'original');first=state['imu_gate_fault'];callback(b'original2')
        self.assertIs(state['imu_gate_fault'],first);self.assertEqual(len(events),2);self.assertFalse(state['imu'].called)
        latest={};m=model();m['max_truth_age_sec']=.02
        def spin():clock.sleep(.02);callback(b'original');latest['truth']=truth(clock,age=.01)
        namespace=dict(node=object(),contract=c,model=m,time=clock,rclpy=N(ok=lambda:True),callbacks=N(spin=spin),
            relay=N(send=lambda:None,command=(0.,0.,0.)),latest=latest,active_lateral=None,stop_observation={})
        self.assertTrue(install(actual_function('observe_stop'),namespace)());self.assertIs(state['imu_gate_fault'],first)
        self.assertTrue(namespace['stop_observation']);self.assertGreaterEqual(len(events),17)
    def test_health_and_warm_retry_remain_immediate_HOLD_on_latched_fault(self):
        clock=Clock();namespace=dict(contract=c,time=clock,clock_pair=None,route={'max_runtime_sec':180.},started=clock.monotonic(),
            writer=N(summary=lambda:dict(dropped=0,worker_error=None)),audit=dict(valid=True,first_violation=None,checked_steady=clock.monotonic()),
            imu_gate_fault={'reason':'accepted_header_gap_exceeded'},cloud_cache=None)
        with self.assertRaisesRegex(c.Hold,'latched raw IMU'):install(actual_function('health'),namespace)()
        tree=ast.parse((HERE/'full_scene_mapping.py').read_text())
        loop=copy.deepcopy(next(n for n in ast.walk(tree) if isinstance(n,ast.While) and any(isinstance(x,ast.Try) and any(isinstance(y,ast.ExceptHandler) and any(isinstance(z,ast.If) and ast.unparse(z.test)=='imu_gate_fault is not None' for z in y.body) for y in x.handlers) for x in n.body)))
        send=mock.Mock(side_effect=AssertionError('latched warm fault cannot renew/retry'))
        ns=dict(contract=c,tick=lambda:(_ for _ in ()).throw(c.Hold('latched fixture fault')),imu_gate_fault={'reason':'fatal'},relay=N(send=send))
        with self.assertRaisesRegex(c.Hold,'latched fixture fault'):exec(compile(ast.fix_missing_locations(ast.Module(body=[loop],type_ignores=[])),'<actual warm retry>','exec'),ns)
        send.assert_not_called()
    def test_writer_rejection_cannot_prevent_first_fatal_latch(self):
        clock=Clock();callback,state,events=self.callback_namespace(clock)
        state['emit']=lambda *a,**k:(_ for _ in ()).throw(c.Hold('fixture writer rejected'))
        with self.assertRaisesRegex(c.Hold,'writer rejected'):callback(b'original')
        self.assertEqual(state['imu_gate_fault']['reason'],'accepted_header_gap_exceeded');self.assertFalse(state['imu'].called)
    def test_fatal_cleanup_stale_GT_never_claims_stop(self):
        clock=Clock();callback,state,events=self.callback_namespace(clock);latest={};m=model();m['max_truth_age_sec']=.02
        def spin():clock.sleep(.02);callback(b'original');latest['truth']=truth(clock,age=.04)
        namespace=dict(node=object(),contract=c,model=m,time=clock,rclpy=N(ok=lambda:True),callbacks=N(spin=spin),
            relay=N(send=lambda:None,command=(0.,0.,0.)),latest=latest,active_lateral=None,stop_observation={})
        self.assertFalse(install(actual_function('observe_stop'),namespace)());self.assertFalse(namespace['stop_observation'])

if __name__=='__main__':unittest.main(verbosity=2)
