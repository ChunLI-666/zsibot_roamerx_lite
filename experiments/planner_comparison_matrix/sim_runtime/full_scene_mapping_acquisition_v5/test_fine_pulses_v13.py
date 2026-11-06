"""Pure bounded plain-primitive pulses and mandatory measured-stop order."""
import copy,math,unittest
from types import SimpleNamespace as N
from unittest import mock
import full_scene_mapping as o
import full_scene_mapping_contract as c
from test_forward_terminal_v9 import policy,Plant
from test_forward_along_v8 import route
from test_fine_forward_v10 import release
from test_v4_core import tick_fixture

def pulse_policy(ticks=1,limit=32):
    p=policy();p['forward_terminal'].update(fine_pulse_ticks=ticks,max_fine_pulses=limit);return p

class PulsePlant(Plant):
    def __init__(self,ticks=1,limit=32,step=.01,**kwargs):
        super().__init__(position=.2,**kwargs);self.p=pulse_policy(ticks,limit);self.bridge=(self.start,self.target,self.p)
        self.step=step;self.phase='coarse';self.ready_for_pulse=False;self.pulse_tick_count=0;self.order=[];self.block_stop=False
    def tick(self,vx,wz,bridge,terminal_deadline_steady=None,fine_phase=False):
        assert vx==c.MAX_VX and wz==0. and bridge==self.bridge and fine_phase is False
        self.clock.sleep(self.cost)
        if terminal_deadline_steady is not None and self.clock.monotonic()>=terminal_deadline_steady:raise c.Hold('fake original final deadline')
        self.position=next(self.steps,self.position+self.step);self.packets.append(c.primitive_packet(vx,wz,self.clock.time_ns()))
        self.order.append('tick');self.pulse_tick_count+=1
        return self.health()
    def stable(self,seconds,timeout):
        self.order.append('stable_enter')
        if self.block_stop and self.phase=='pulse':raise c.Hold('no actual stable stop')
        result=super().stable(seconds,timeout);self.ready_for_pulse=True;self.order.append('stable_complete');return result
    def emit(self,name,**fields):
        if name=='fine_pulse_started':
            assert self.ready_for_pulse,'next positive burst before measured stable stop'
            self.ready_for_pulse=False;self.phase='pulse';self.pulse_tick_count=0;self.order.append('burst_started')
        if name=='fine_pulse_stopped':
            assert self.ready_for_pulse and self.pulse_tick_count<=self.p['forward_terminal']['fine_pulse_ticks']
            self.order.append('burst_stopped')
        self.events.append((name,fields))
    def run(self):
        return o.forward_terminal_leg(self.start,self.target,self.entry,self.p,self.health,self.tick,self.stable,self.rotate,self.emit,self.clock)

class FinePulses(unittest.TestCase):
    def test_types_mutual_exclusion_and_disabled_low_profile(self):
        for ticks in [1,2,3]:
            p=pulse_policy(ticks);c.validate_route(route(p));self.assertIsNone(c.validate_fine_route({},route(p)))
            with self.assertRaises(c.Hold):c.validate_fine_route(release(.2),route(p))
        for key,values in [('fine_pulse_ticks',[None,True,False,0,4,1.,'1']),('max_fine_pulses',[None,True,0,33,32.,'32'])]:
            for bad in values:
                p=pulse_policy();p['forward_terminal'][key]=bad
                with self.subTest(key=key,bad=bad),self.assertRaises(c.Hold):c.validate_route(route(p))
        for key in ['fine_pulse_ticks','max_fine_pulses']:
            p=pulse_policy();p['forward_terminal'].pop(key)
            with self.assertRaises(c.Hold):c.validate_route(route(p))
        p=pulse_policy();p['forward_terminal']['fine_forward_mps']=.2
        with self.assertRaises(c.Hold):c.validate_route(route(p))
    def test_one_two_three_tick_bursts_plain_amplitude_and_actual_stop_order(self):
        for ticks in [1,2,3]:
            f=PulsePlant(ticks=ticks,cost=.05);f.run()
            issued=[v for n,v in f.events if n=='fine_pulse_issued'];stopped=[v for n,v in f.events if n=='fine_pulse_stopped']
            counts={}
            for v in issued:counts[v['index']]=counts.get(v['index'],0)+1
            self.assertTrue(counts);self.assertTrue(all(1<=v<=ticks for v in counts.values()));self.assertEqual(len(stopped),len(counts))
            self.assertEqual([v['issued_ticks'] for v in stopped],list(counts.values()))
            self.assertTrue(all(p['forward']==.2 and p['lateral']==p['yaw']==0. and 'primitive_phase' not in p for p in f.packets))
            self.assertEqual([v['index'] for v in stopped],list(range(1,len(stopped)+1)))
    def test_failed_measured_stop_blocks_every_next_positive(self):
        f=PulsePlant(ticks=1,cost=.05);f.block_stop=True
        with self.assertRaisesRegex(c.Hold,'actual stable'):f.run()
        self.assertEqual(len(f.packets),1);self.assertEqual(len([n for n,v in f.events if n=='fine_pulse_started']),1);self.assertFalse(any(n=='fine_pulse_stopped' for n,v in f.events))
    def test_shared_pulse_count_and_existing_correction_limit_not_renewed(self):
        f=PulsePlant(ticks=1,limit=2,steps=[.28,.28,.28],stops=[.2,.274,.274,.274],cost=.05)
        with self.assertRaisesRegex(c.Hold,'pulse budget'):f.run()
        self.assertEqual(len(f.packets),2);self.assertEqual([v['index'] for n,v in f.events if n=='fine_pulse_started'],[1,2])
        f=PulsePlant(ticks=1,limit=32,steps=[.28,.28,.28],stops=[.2,.274,.274,.274],cost=.05)
        with self.assertRaisesRegex(c.Hold,'correction count'):f.run()
        self.assertEqual(len(f.packets),3)
    def test_same_absolute20s_deadline_and_overshoot_no_reverse(self):
        f=PulsePlant(ticks=3,cost=20.)
        with self.assertRaisesRegex(c.Hold,'deadline'):f.run()
        self.assertFalse(f.packets)
        f=PulsePlant(ticks=3,steps=[.34],cost=.05)
        with self.assertRaisesRegex(c.Hold,'overshoot'):f.run()
        self.assertEqual(len(f.packets),1);self.assertEqual(f.packets[0]['forward'],.2)
        self.assertFalse(any(n=='fine_pulse_started' and v['index']>1 for n,v in f.events))
    def test_actual_tick_armed_deadline_and_original_issued_anchor(self):
        for before_arm in [True,False]:
            ns,unused,rows=tick_fixture();q=o.PrearmImuQualification(lambda:dict(command=[0.,0.,0.],request=[0.,0.,0.],latest_inputs={}),ns['time'])
            q.armed=q.ever_armed=not before_arm;ns['relay'].prearm_qualification=q;ns['emit']=lambda *a,**kw:None
            p=pulse_policy();deadline=ns['time'].monotonic()+1.
            with mock.patch.object(c,'guard_hits',return_value={}):
                if before_arm:
                    with self.assertRaises(c.Hold):ns['tick'](.4,0.,bridge=((0,0,0),(.3,0,0),p),terminal_deadline_steady=deadline)
                    self.assertFalse(rows)
                else:
                    ns['tick'](.4,0.,bridge=((0,0,0),(.3,0,0),p),terminal_deadline_steady=deadline)
                    self.assertEqual(ns['last_send'],rows[0]['final_command_guard']['issued_steady_sec']);self.assertEqual(rows[0]['policy_vx'],.4);self.assertFalse(rows[0]['fine_forward_phase'])
            if not before_arm:
                ns,unused,rows=tick_fixture()
                with mock.patch.object(c,'guard_hits',return_value={}),self.assertRaisesRegex(c.Hold,'terminal deadline'):ns['tick'](.4,0.,bridge=((0,0,0),(.3,0,0),p),terminal_deadline_steady=ns['time'].monotonic()+.02)
                self.assertFalse(rows)

if __name__=='__main__':unittest.main(verbosity=2)
