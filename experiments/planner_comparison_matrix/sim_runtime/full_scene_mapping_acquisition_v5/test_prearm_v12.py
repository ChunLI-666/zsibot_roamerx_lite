"""Pure unchanged CDR Gate, fixed-epoch lease, real StableStop/tick guard tests."""
import ast,copy,math,struct,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest import mock
import full_scene_mapping as o
import full_scene_mapping_contract as c
from test_v4_core import Clock,model,truth,tick_fixture
from test_startup_v6 import install,actual_function


def cdr(ns,frame='map',nan=False):
    frame=frame.encode()+b'\0';b=b'\x00\x01\x00\x00'+struct.pack('<iII',ns//10**9,ns%10**9,len(frame))+frame
    aligned=4+((len(b)-4+7)//8)*8;b+=b'\0'*(aligned-len(b));v=[0.]*37;v[3]=1.
    if nan:v[4]=float('nan')
    return b+struct.pack('<37d',*v)

def fixture():
    clock=Clock();context=dict(command=[0.,0.,0.],request=[0.,0.,0.],latest_inputs={'truth':{'stamp_ns':123}})
    q=o.PrearmImuQualification(lambda:context,clock)
    return q,clock,context

def feed(q,clock,dt=0.,stamp=None,**kw):
    clock.sleep(dt);return q.inspect(cdr(clock.time_ns() if stamp is None else stamp,**kw),clock.time_ns(),clock.monotonic_ns())

def span10(q,clock):
    feed(q,clock)
    for _ in range(100):feed(q,clock,.1)

class Qualification(unittest.TestCase):
    def test_flag_strict_and_until10seconds_no_nonzero(self):
        self.assertFalse(o.prearm_requalification_enabled({}));self.assertFalse(o.prearm_requalification_enabled({'allow_prearm_imu_gap_requalification':False}))
        self.assertTrue(o.prearm_requalification_enabled({'allow_prearm_imu_gap_requalification':True}))
        for bad in [None,0,1,'true',{},[]]:
            with self.assertRaises(c.Hold):o.prearm_requalification_enabled({'allow_prearm_imu_gap_requalification':bad})
        q,k,ctx=fixture();feed(q,k)
        for _ in range(99):feed(q,k,.1)
        with self.assertRaises(c.Hold):q.arm(0,True,True)
        feed(q,k,.1);q.arm(0,True,True);q.require_command(.4,0.,0.);self.assertTrue(q.armed)
        q,k,ctx=fixture()
        with self.assertRaises(c.Hold):q.require_command(.4,0.,0.)
        self.assertTrue(q.ever_nonzero_requested);self.assertFalse(q.armed)
    def test_gap_at9point9_resets_both_spans_and_rejects_trigger_replay(self):
        q,k,ctx=fixture();feed(q,k)
        for _ in range(99):feed(q,k,.1)
        gap=feed(q,k,.11);self.assertTrue(gap['prearm_epoch_reopened']);self.assertEqual(q.epoch,1)
        self.assertIn('original_cdr_base64',gap);trigger=gap['header_ns']
        replay=feed(q,k,.002,stamp=trigger);self.assertFalse(replay['accepted']);self.assertEqual(replay['reason'],'prearm_epoch_header_not_above_high_water')
        self.assertIsNone(q.gate.last_accepted_ns);self.assertEqual(q.spans()['header_span_ns'],0)
        # The discarded candidate retained full original decode result and bytes.
        self.assertTrue(replay['original_gate_candidate_event']['accepted']);self.assertIn('original_cdr_base64',replay)
        feed(q,k,.002)
        for _ in range(99):feed(q,k,.1)
        with self.assertRaises(c.Hold):q.arm(1,True,True)
        feed(q,k,.1);q.arm(1,True,True);self.assertEqual(q.arm_receipt['epoch'],1)
        d=q.diagnostics(k.time_ns(),k.monotonic_ns());self.assertEqual(d['rejected'],2);self.assertEqual(d['active_epoch'],1);self.assertEqual(len(d['warm_invalid_snapshots']),1)
    def test_armed_gap_and_other_fatals_never_requalify(self):
        q,k,ctx=fixture();span10(q,k);q.arm(0,True,True);event=feed(q,k,.11)
        self.assertFalse(event.get('prearm_epoch_reopened',False));self.assertEqual(q.epoch,0);self.assertIsNotNone(q.fault)
        with self.assertRaises(c.Hold):q.require_command(.4,0.,0.)
        for kw in [dict(frame='other'),dict(nan=True)]:
            q,k,ctx=fixture();feed(q,k);event=feed(q,k,.11,**kw)
            self.assertTrue(event['fatal']);self.assertFalse(event.get('prearm_epoch_reopened',False));self.assertEqual(q.epoch,0)
        for raw in [b'bad',cdr(1_060_000_000)]:
            q,k,ctx=fixture();event=q.inspect(raw,k.time_ns(),k.monotonic_ns());self.assertTrue(event['fatal']);self.assertEqual(q.epoch,0)
        # Stateless validation still runs before high-water filtering.
        q,k,ctx=fixture();feed(q,k);gap=feed(q,k,.11)
        event=feed(q,k,.002,stamp=gap['header_ns'],nan=True);self.assertEqual(event['reason'],'nonfinite_imu');self.assertIsNotNone(q.fault)
    def test_three_reopens40sec_limit_and_lifetime_preserved(self):
        q,k,ctx=fixture();feed(q,k);events=[]
        for _ in range(4):feed(q,k,.002);events.append(feed(q,k,.11))
        d=q.diagnostics(k.time_ns(),k.monotonic_ns());self.assertEqual(q.epoch,3);self.assertEqual(d['raw_received'],9);self.assertEqual(d['accepted'],5);self.assertEqual(d['rejected'],4)
        self.assertEqual(d['reject_reasons']['accepted_header_gap_exceeded'],4);self.assertEqual(len(d['epochs']),4);self.assertEqual(len(d['warm_invalid_snapshots']),3)
        self.assertEqual(d['permanent_fault']['reason'],'accepted_header_gap_exceeded');self.assertTrue(all('original_cdr_base64' in e for e in events))
        q,k,ctx=fixture();k.sleep(40.)
        with self.assertRaisesRegex(c.Hold,'40s'):o.qualify_prearm_zero(q,lambda:(_ for _ in ()).throw(AssertionError('afterdeadline')),N(command=(0.,0.,0.)),model(),lambda *a,**v:None,lambda e:True,k)
        self.assertFalse(q.armed)
    def test_full_qualify_loop_restarts_GT_stop_on_epoch_change(self):
        q,k,ctx=fixture();begin=k.monotonic();gap=[False];rows=[];relay=N(command=(0.,0.,0.),send=lambda:None)
        def tick():
            dt=.05
            if k.monotonic()-begin>=9.9 and not gap[0]:dt=.15;gap[0]=True
            event=feed(q,k,dt)
            if not event['accepted']:raise c.Hold('no fresh accepted original IMU progress')
            return truth(k),0.
        result=o.qualify_prearm_zero(q,tick,relay,model(),lambda *a,**kw:rows.append(kw),lambda exc:True,k,arming_allowed=lambda:True)
        self.assertTrue(q.armed);self.assertGreaterEqual(k.monotonic()-begin,20.);self.assertEqual(q.epoch,1);self.assertTrue(rows)
        self.assertGreaterEqual(min(q.spans().values()),10_000_000_000)
    def test_zero_reanchor_actual_tick_only_before_arm_never_positive(self):
        for armed in [False,True]:
            ns,p,rows=tick_fixture();k=ns['time'];context=dict(command=[0.,0.,0.],request=[0.,0.,0.],latest_inputs={})
            q=o.PrearmImuQualification(lambda:context,k);q.armed=q.ever_armed=armed;ns['relay'].prearm_qualification=q;ns['emit']=lambda *a,**kw:None
            old=k.monotonic()-.2;ns['last_send']=old
            with mock.patch.object(c,'guard_hits',return_value={}):
                if armed:
                    with self.assertRaisesRegex(c.Hold,'control-loop gap'):ns['tick']()
                    self.assertEqual(ns['last_send'],old);self.assertFalse(rows)
                else:
                    ns['tick']();self.assertEqual(ns['last_send'],rows[0]['final_command_guard']['issued_steady_sec']);self.assertGreater(ns['last_send'],old)
                    self.assertEqual(q.invalid_snapshots[0]['old_issued_steady_sec'],old)
        q,k,ctx=fixture();old=k.monotonic()-.2
        self.assertIsNone(q.reanchor_zero(old,(.4,0.,0.),(0.,0.,0.)));self.assertIsNone(q.reanchor_zero(old,(0.,0.,0.),(.4,0.,0.)))
    def test_actual_wait_predicate_ownership_evidence_source_bad_cannot_arm(self):
        k=Clock();state=dict(writer=N(summary=lambda:dict(dropped=0,worker_error=None)),imu_gate_fault=None,
            audit=dict(valid=True,first_violation=None,checked_steady=k.monotonic()),time=k)
        allowed=install(actual_function('prearm_waiting_allowed'),state)
        self.assertTrue(allowed(c.Hold('missing input')))
        for reason in ['wall/source clock step relative to steady clock','invalid original GT frames/values','original/derived publisher ownership changed: /odom']:
            self.assertFalse(allowed(c.Hold(reason)))
        state['writer'].summary=lambda:dict(dropped=1,worker_error=None);self.assertFalse(allowed(c.Hold('missing input')))
        state['writer'].summary=lambda:dict(dropped=0,worker_error=None);state['audit']['first_violation']={'reason':'changed'};self.assertFalse(allowed(c.Hold('missing input')))
        q,k,ctx=fixture()
        with self.assertRaises(c.Hold):o.qualify_prearm_zero(q,lambda:(_ for _ in ()).throw(c.Hold('ownership bad')),N(command=(0.,0.,0.),send=lambda:None),model(),lambda *a,**kw:None,lambda e:False,k)
        self.assertFalse(q.armed)
        q,k,ctx=fixture();relay=N(command=(0.,0.,0.),send=lambda:None)
        def tick():feed(q,k,.05);return truth(k),0.
        with self.assertRaises(c.Hold):o.qualify_prearm_zero(q,tick,relay,model(),lambda *a,**kw:None,lambda e:True,k,arming_allowed=lambda:False)
        self.assertFalse(q.armed)

if __name__=='__main__':unittest.main(verbosity=2)
