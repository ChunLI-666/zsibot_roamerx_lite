"""Pure fake socket/time/signal/proof fixtures. No actual socket or ROS."""
import contextlib
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.dont_write_bytecode = True
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('semantic_sustained_backend_fixture',HERE/'semantic_tool_timed.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)


class FakeClock:
    def __init__(self):self.now=0;self.on_sleep=None
    def monotonic_ns(self):return self.now
    def time_ns(self):return 1_800_000_000_000_000_000+self.now
    def clock_gettime(self,unused):return self.now/1e9
    def sleep(self,seconds):
        if seconds<0:raise AssertionError('negative sleep')
        self.now+=round(seconds*1e9)
        if self.on_sleep:self.on_sleep()


class FakeSignals:
    SIGINT=2;SIGTERM=15
    def __init__(self):self.original={2:'prior_int',15:'prior_term'};self.handlers=dict(self.original)
    def signal(self,number,handler):
        old=self.handlers[number];self.handlers[number]=handler;return old


class FakeSocket:
    def __init__(self,clock):
        self.clock=clock;self.calls=[];self.closed=False;self.timeout=None;self.fail=None;self.delay_ns=0;self.close_error=False
    def settimeout(self,seconds):self.timeout=seconds
    def sendto(self,payload,target):
        command=json.loads(payload);self.calls.append((command,target,self.clock.now))
        if self.fail and self.fail(command,len(self.calls)):raise OSError('fixture send failed')
        if command['lateral']:self.clock.now+=self.delay_ns
        return len(payload)
    def close(self):
        self.closed=True
        if self.close_error:raise OSError('fixture close failed')

class SustainedBackend(unittest.TestCase):
    def fixture(self,prove=None):
        clock=FakeClock();signals=FakeSignals();sock=FakeSocket(clock);proof=[];stream=io.StringIO()
        def check():
            proof.append(clock.now)
            if prove:prove(clock,signals,len(proof))
        return clock,signals,sock,proof,stream,check
    def run_fake(self,fixture,sign=.2,duration=4.):
        clock,signals,sock,proof,stream,check=fixture
        result=b.run_probe(sign,duration,lambda:sock,check,stream,clock,signals)
        self.assertEqual(signals.handlers,signals.original)
        return result,[json.loads(line) for line in stream.getvalue().splitlines()]
    def test_fixed4s80_motion_and5_zero_each_sign(self):
        for sign in [.2,-.2]:
            f=self.fixture();r,rows=self.run_fake(f,sign)
            self.assertTrue(r['completed']);self.assertEqual(r['maximum_motion_packets'],80)
            self.assertEqual(r['motion_packets_sent'],80);self.assertEqual(r['per_motion_proof_checks'],80)
            self.assertEqual([row['command']['lateral'] for row in rows],[sign]*80+[0.]*5)
            self.assertEqual(rows[80]['send_before_steady_ns'],4_000_000_000)
            self.assertTrue(all(row['send_before_steady_ns']<4_000_000_000 for row in rows[:80]))
            self.assertEqual(r['final_zero_attempts'],5);self.assertTrue(f[2].closed)
    def test_internal_duration_cap4_and_no_invalid_socket(self):
        for value in [0.,-1.,4.000001,float('nan'),True]:
            with self.subTest(value=value),self.assertRaises(b.CanaryHold):b.run_probe(.2,value,lambda:(_ for _ in ()).throw(AssertionError('factory')),lambda:None,io.StringIO(),FakeClock(),FakeSignals())
        r,rows=self.run_fake(self.fixture(),duration=1.)
        self.assertEqual(r['motion_packets_sent'],20);self.assertEqual(len(rows),25)
    def test_proof_failure_stops_and_always_neutral(self):
        def prove(clock,signals,count):
            if count==4:raise b.CanaryHold('fresh proof lost')
        r,rows=self.run_fake(self.fixture(prove))
        self.assertFalse(r['completed']);self.assertEqual(r['motion_packets_sent'],2)
        self.assertEqual(r['final_zero_attempts'],5);self.assertEqual([x['command']['lateral'] for x in rows[-5:]],[0.]*5)
    def test_slow_proof_never_starts_nonzero_past4s(self):
        def prove(clock,signals,count):
            if count==2:clock.now+=5_000_000_000
        r,rows=self.run_fake(self.fixture(prove))
        self.assertEqual(r['motion_packets_sent'],0);self.assertEqual(len(rows),5)
    def test_send_crossing4s_is_explicit_failure_and_cleanup(self):
        f=self.fixture();f[2].delay_ns=4_100_000_000;r,rows=self.run_fake(f)
        self.assertFalse(r['completed']);self.assertEqual(r['motion_packets_sent'],1);self.assertEqual(r['final_zero_attempts'],5)
        self.assertIn('crossed active deadline',r['errors'][0]['message'])
    def test_sigint_and_repeated_cleanup_signal_cannot_skip_neutral(self):
        f=self.fixture();clock,signals=f[:2];count=[0]
        def on_sleep():
            count[0]+=1
            signals.handlers[signals.SIGINT](signals.SIGINT,None)
        clock.on_sleep=on_sleep;r,rows=self.run_fake(f)
        self.assertFalse(r['completed']);self.assertEqual(r['final_zero_attempts'],5);self.assertTrue(f[2].closed)
    def test_slow_proof_cannot_make_catchup_bursts(self):
        def prove(clock,signals,count):clock.now+=20_000_000
        r,rows=self.run_fake(self.fixture(prove));moving=[row for row in rows if row['command']['lateral']]
        self.assertLessEqual(len(moving),80)
        self.assertTrue(all(b['send_before_steady_ns']-a['send_after_steady_ns']>=50_000_000 for a,b in zip(moving,moving[1:])))

if __name__=='__main__':unittest.main(verbosity=2)


