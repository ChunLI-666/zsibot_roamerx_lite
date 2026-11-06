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
spec=importlib.util.spec_from_file_location('semantic_canary_backend_fixture',HERE/'semantic_tool_timed.py')
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


class BackendFixtures(unittest.TestCase):
    def fixture(self,sign=.2,prove=None,duration=1.,stream=None):
        clock=FakeClock();signals=FakeSignals();sock=FakeSocket(clock);calls=[];proof=[]
        def check():
            proof.append(clock.now)
            if prove:prove(clock,signals,len(proof))
        def factory():calls.append('created');return sock
        stream=io.StringIO() if stream is None else stream
        return clock,signals,sock,calls,proof,check,factory,stream

    def execute(self,fixture,sign=.2,duration=1.):
        clock,signals,sock,calls,proof,check,factory,stream=fixture
        result=b.run_probe(sign,duration,factory,check,stream,clock,signals)
        self.assertEqual(signals.handlers,signals.original)
        return result

    def test_default_hold_without_contract_socket_or_output(self):
        before=set(sys.modules)
        with contextlib.redirect_stdout(io.StringIO()) as stream:self.assertEqual(b.main([]),0)
        self.assertTrue(json.loads(stream.getvalue())['no_socket_created'])
        self.assertNotIn('lateral_contract',set(sys.modules)-before)
        self.assertNotIn('recording_deadline',set(sys.modules)-before)

    def test_each_sign_one_socket_twenty_packets_five_zeros_and_hashes(self):
        for sign in (.2,-.2):
            with self.subTest(sign=sign):
                f=self.fixture();result=self.execute(f,sign)
                clock,signals,sock,calls,proof,check,factory,stream=f
                self.assertTrue(result['completed']);self.assertEqual(calls,['created'])
                self.assertEqual(sock.timeout,.05);self.assertTrue(sock.closed)
                self.assertEqual(result['motion_packets_sent'],20);self.assertEqual(result['per_motion_proof_checks'],20)
                self.assertEqual(len(proof),21);self.assertEqual(result['final_zero_logged_successes'],5)
                rows=[json.loads(x) for x in stream.getvalue().splitlines()];self.assertEqual(len(rows),25)
                self.assertEqual([r['command']['lateral'] for r in rows], [sign]*20+[0.]*5)
                self.assertEqual(rows[20]['send_before_steady_ns'],1_000_000_000)
                for row in rows:
                    data=json.dumps(row['command'],separators=(',',':')).encode()
                    self.assertEqual(row['payload_sha256'],hashlib.sha256(data).hexdigest())
                    self.assertEqual(row['bytes'],len(data));self.assertEqual(row['target'],['127.0.0.1',7447])

    def test_whitelist_rejects_axis_other_sign_extra_and_nonfinite(self):
        changes=[dict(forward=.01),dict(yaw=.01),dict(lateral=-.2),dict(version=True),dict(action='stand'),dict(LX=-.2),dict(lateral=float('nan'))]
        for change in changes:
            command=b.frame(.2);command.update(change)
            with self.subTest(change=change),self.assertRaises(b.CanaryHold):b.permitted(command,.2)
        for sign in (0,.25,True,float('nan')):
            with self.subTest(sign=sign),self.assertRaises(b.CanaryHold):b.frame(sign)

    def test_invalid_duration_rejected_before_factory(self):
        for duration in (0.,-1.,1.01,float('nan'),True,1e-15):
            f=self.fixture()
            with self.subTest(duration=duration),self.assertRaises(b.CanaryHold):self.execute(f,duration=duration)
            self.assertEqual(f[3],[])

    def test_initial_proof_failure_never_creates_socket(self):
        def fail(*args):raise b.CanaryHold('initial stale proof')
        f=self.fixture(prove=fail);result=self.execute(f)
        self.assertFalse(result['completed']);self.assertEqual(f[3],[])
        self.assertEqual(result['final_zero_attempts'],0)

    def test_every_packet_proof_loss_stops_and_always_five_zeros(self):
        def fail(clock,signals,count):
            if count==4:raise b.CanaryHold('recorder identity no longer bound')
        f=self.fixture(prove=fail);result=self.execute(f)
        self.assertFalse(result['completed']);self.assertEqual(result['motion_packets_sent'],2)
        self.assertEqual(result['final_zero_logged_successes'],5);self.assertTrue(f[2].closed)

    def test_sigint_and_repeated_cleanup_signal_do_not_interrupt_five_zeros(self):
        f=self.fixture();clock,signals=f[:2];triggered=[]
        def interrupt():
            if not triggered:
                triggered.append(True);signals.handlers[signals.SIGINT](signals.SIGINT,None)
            elif len(triggered)==1:
                triggered.append(True);signals.handlers[signals.SIGTERM](signals.SIGTERM,None)
        clock.on_sleep=interrupt
        result=self.execute(f)
        self.assertFalse(result['completed']);self.assertEqual(result['signals_observed'],[2,15])
        self.assertEqual(result['final_zero_logged_successes'],5);self.assertTrue(f[2].closed)

    def test_keyboard_interrupt_exception_still_sends_five_zeros(self):
        def fail(clock,signals,count):
            if count==3:raise KeyboardInterrupt('fixture')
        f=self.fixture(prove=fail);result=self.execute(f)
        self.assertFalse(result['completed']);self.assertEqual(result['final_zero_logged_successes'],5)

    def test_motion_send_exception_cannot_skip_zero_cleanup(self):
        f=self.fixture();f[2].fail=lambda command,count:bool(command['lateral']) and count==3
        result=self.execute(f);self.assertFalse(result['completed'])
        self.assertEqual(result['motion_packets_sent'],2);self.assertEqual(result['final_zero_attempts'],5)
        self.assertEqual(result['final_zero_logged_successes'],5);self.assertTrue(f[2].closed)

    def test_failed_first_zero_still_attempts_remaining_zeros_and_close(self):
        f=self.fixture();f[2].fail=lambda command,count:command['lateral']==0 and count==21
        result=self.execute(f);self.assertFalse(result['completed'])
        self.assertEqual(result['final_zero_attempts'],5);self.assertEqual(result['final_zero_logged_successes'],4)
        self.assertTrue(f[2].closed)

    def test_logging_failure_still_attempts_zero_sends(self):
        class BrokenStream:
            def write(self,text):raise OSError('fixture disk failure')
            def flush(self):pass
        f=self.fixture(stream=BrokenStream());result=self.execute(f)
        self.assertFalse(result['completed']);self.assertEqual(result['final_zero_attempts'],5)
        self.assertEqual([x[0]['lateral'] for x in f[2].calls],[.2]+[0.]*5)
        self.assertTrue(f[2].closed)

    def test_close_error_never_reports_completed(self):
        f=self.fixture();f[2].close_error=True;result=self.execute(f)
        self.assertFalse(result['completed']);self.assertFalse(result['close_returned'])
        self.assertEqual(result['final_zero_logged_successes'],5)

    def test_slow_proof_never_sends_nonzero_after_deadline(self):
        def slow(clock,signals,count):
            if count>1:clock.now+=1_100_000_000
        f=self.fixture(prove=slow);result=self.execute(f)
        self.assertFalse(result['completed']);self.assertEqual(result['motion_packets_sent'],0)
        self.assertEqual(result['final_zero_logged_successes'],5)

    def test_send_syscall_crossing_deadline_is_explicit_failure(self):
        f=self.fixture();f[2].delay_ns=1_100_000_000;result=self.execute(f)
        self.assertFalse(result['completed']);self.assertEqual(result['motion_packets_sent'],1)
        self.assertTrue(any('crossed active deadline' in e['message'] for e in result['errors']))
        self.assertEqual(result['final_zero_logged_successes'],5)

    def test_slow_guard_does_not_create_catchup_bursts(self):
        def slow(clock,signals,count):clock.now+=80_000_000
        f=self.fixture(prove=slow);result=self.execute(f)
        sent=[x[2] for x in f[2].calls if x[0]['lateral']]
        self.assertTrue(all(b-a>=50_000_000 for a,b in zip(sent,sent[1:])))
        self.assertTrue(all(t<result['active_deadline_steady_ns'] for t in sent))

    def parent_fixture(self):
        clock=FakeClock();clock.now=1_000_000_000
        identities={n:dict(pid=n,identity='exact') for n in (11,12,13)}
        ready=dict(parent_identity=identities[11],recorder_identity=identities[12],helper_identity=identities[13],deadline_boottime_sec=60.)
        lease=dict(release_id='r',parent_identity=identities[11],written_steady_ns=clock.now,
                   anchor=[0.,0.,0.],samples={'fixture':'accepted latest only'})
        calls=[]
        contract=SimpleNamespace(host_proof=lambda *a,**kw:calls.append(('host',kw)),
            read=lambda p:lease,check_live=lambda *a:calls.append(('check_live',a)))
        deadline=SimpleNamespace(process_identity=lambda pid:identities[pid])
        return clock,ready,lease,calls,contract,deadline,identities

    def test_parent_proof_delegates_unchanged_header_receipt_gate(self):
        clock,ready,lease,calls,contract,deadline,identities=self.parent_fixture()
        b.prove_parent({},dict(release_id='r'),ready,Path('/fixture'),contract,deadline,clock)
        self.assertEqual(calls[-1],('check_live',(lease['samples'],lease['anchor'],clock.time_ns(),clock.monotonic_ns())))

    def test_parent_lease_150ms_identity_and_deadline_fail_closed(self):
        for kind in ('stale','future','recorder','helper','parent','release','deadline'):
            clock,ready,lease,calls,contract,deadline,identities=self.parent_fixture()
            if kind=='stale':lease['written_steady_ns']-=150_000_001
            if kind=='future':lease['written_steady_ns']+=1
            if kind in ('recorder','helper','parent'):
                pid=ready[kind+'_identity']['pid'];identities[pid]=dict(pid=pid,identity='changed')
            if kind=='release':lease['release_id']='other'
            if kind=='deadline':ready['deadline_boottime_sec']=11.
            with self.subTest(kind=kind),self.assertRaises(b.CanaryHold):
                b.prove_parent({},dict(release_id='r'),ready,Path('/fixture'),contract,deadline,clock)


if __name__=='__main__':unittest.main()
