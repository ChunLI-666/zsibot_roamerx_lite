"""Pure root-binding/fine-only wire/actual trace/deadline/cleanup evidence tests."""
import ast,copy,io,json,math,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest import mock
import full_scene_mapping as o
import full_scene_mapping_contract as c
import lateral_capability as p
import mapping_lateral_relay as r
from test_v4_core import Clock,Socket,tick_fixture
from test_forward_along_v8 import route
from test_forward_terminal_v9 import policy,Plant


def release(value=.2):return dict(allow_fine_forward_profile=True,selected_fine_forward_mps=value,fine_forward_subcapability=p.FINE_FORWARD_SUBCAPABILITY)
def selected(value=.2):return p.fine_profile_from_release(release(value))
def leg(value=.2):
    v=policy();v['forward_terminal']['fine_forward_mps']=value;return v

class FineForward(unittest.TestCase):
    def test_root_release_and_private_admission_exact_selection(self):
        self.assertIsNone(p.fine_profile_from_release({}));self.assertIsNone(p.fine_admission_profile({},{}))
        for value in [.2,.3]:self.assertEqual(p.fine_admission_profile(dict(fine_forward_profile=selected(value)),release(value)),selected(value))
        for value in [None,True,False,.1,.4,float('nan'),float('inf'),'0.2']:
            with self.subTest(value=value),self.assertRaises(p.AdmissionError):p.fine_profile_from_release(release(value))
        for v in [dict(allow_fine_forward_profile=False,selected_fine_forward_mps=.2),dict(allow_fine_forward_profile=1),dict(release(),fine_forward_subcapability='other')]:
            with self.assertRaises(p.AdmissionError):p.fine_profile_from_release(v)
        with self.assertRaises(p.AdmissionError):p.fine_admission_profile(dict(fine_forward_profile=selected(.3)),release(.2))
        with self.assertRaises(p.AdmissionError):p.fine_admission_profile({},release(.2))
    def test_route_and_factory_require_root_selected_phase_value(self):
        for value in [.2,.3]:
            q=leg(value);v=route(q);c.validate_route(v)
            self.assertEqual(c.validate_fine_route(release(value),v),selected(value))
            with self.assertRaises(c.Hold):c.validate_fine_route({},v)
            with self.assertRaises(c.Hold):c.validate_fine_route(release(.3 if value==.2 else .2),v)
            with self.assertRaises(c.Hold):c.primitive_packet(value,0.,10)
            packet=c.primitive_packet(value,0.,10,fine_forward_profile=selected(value),fine_phase=True)
            self.assertEqual(packet['forward'],value/2.);self.assertEqual(packet['primitive_phase'],'forward_terminal_fine')
            for args in [(-value,0.,0.),(value,.3,0.),(value,0.,.4),(.4,0.,0.)]:
                with self.assertRaises(c.Hold):c.primitive_packet(*args[:2],10,args[2],fine_forward_profile=selected(value),fine_phase=True)
        for value in [None,True,0.,.1,.4,float('nan')]:
            with self.assertRaises(c.Hold):c.validate_route(route(leg(value)))
    def test_child_exact_phase_selection_age_and_wire_cleanup(self):
        for value in [.2,.3]:
            clock=Clock();packet=c.primitive_packet(value,0.,clock.time_ns(),fine_forward_profile=selected(value),fine_phase=True)
            self.assertEqual(r.validated_axes(packet,clock.time_ns(),selected(value))['forward'],value/2.)
            for profile in [None,selected(.3 if value==.2 else .2)]:
                with self.assertRaises(ValueError):r.validated_axes(packet,clock.time_ns(),profile)
            for changes in [dict(primitive_phase='coarse'),dict(fine_forward_subcapability='other'),dict(forward=.11),dict(lateral=.4),dict(yaw=.3),dict(issued_unix_ns=clock.time_ns()+1)]:
                with self.assertRaises(ValueError):r.validated_axes(dict(packet,**changes),clock.time_ns(),selected(value))
            with self.assertRaises(ValueError):r.validated_axes(packet,clock.time_ns()+100_000_001,selected(value))
            sock=Socket(clock);stream=io.StringIO(json.dumps(packet)+'\n')
            def select_(a,b,e,timeout):return a,[],[]
            result=r.run_loop(sock,stream,lambda:None,clock,select_,fine_forward_profile=selected(value))
            self.assertEqual(result['accepted'],1);self.assertEqual(sock.rows[0][0]['forward'],value/2.);self.assertTrue(all(x==r.ZERO for x,t in sock.rows[-5:]));self.assertTrue(sock.closed)
    def test_owned_relay_checks_profile_and_returns_actual_selected_packet(self):
        relay=o.OwnedRelay.__new__(o.OwnedRelay);relay.process=N(pid=42,poll=lambda:None,stdin=N(fileno=lambda:88));relay.identity={'pid':42};relay.command=(0.,0.,0.);relay.fine_forward_profile=selected(.2)
        rows=[]
        with mock.patch.object(c,'process_identity',return_value=relay.identity),mock.patch.object(o.os,'write',side_effect=lambda fd,data:rows.append(json.loads(data)) or len(data)):
            with self.assertRaises(c.Hold):relay.send(.2,0.,preflight=lambda *a:None)
            with self.assertRaises(c.Hold):relay.send(.3,0.,preflight=lambda *a:None,fine_phase=True)
            packet=relay.send(.2,0.,preflight=lambda *a:None,fine_phase=True)
        self.assertEqual(rows,[packet]);self.assertEqual(packet['forward'],.1);self.assertEqual(relay.command,(.2,0.,0.))
    def test_actual_tick_applied_trace_and_final_deadline(self):
        for value in [.2,.3]:
            for expires in [False,True]:
                ns,unused,rows=tick_fixture();ns['release']=release(value);profile=selected(value);q=leg(value);clock=ns['time']
                def send(vx,wz,preflight,lateral_raw,fine_phase=False):
                    preflight(clock.time_ns(),clock.monotonic());return c.primitive_packet(vx,wz,clock.time_ns(),lateral_raw,fine_forward_profile=profile,fine_phase=fine_phase)
                ns['relay'].send=send
                with mock.patch.object(c,'guard_hits',return_value={}):
                    if expires:
                        with self.assertRaisesRegex(c.Hold,'terminal deadline'):ns['tick'](value,0.,bridge=((0,0,0),(.3,0,0),q),terminal_deadline_steady=clock.monotonic()+.02,fine_phase=True)
                        self.assertFalse(rows)
                    else:
                        ns['tick'](value,0.,bridge=((0,0,0),(.3,0,0),q),terminal_deadline_steady=clock.monotonic()+1.,fine_phase=True)
                        self.assertEqual(rows[0]['policy_vx'],value);self.assertEqual(rows[0]['applied_forward_policy_mps'],value);self.assertEqual(rows[0]['applied_command']['forward'],value/2.);self.assertTrue(rows[0]['fine_forward_phase'])
    def test_helper_only_fine_uses_selected_value_default_coarse_unchanged(self):
        for value in [.2,.3]:
            f=Plant(steps=[.1,.2,.28],stops=[.22,.28]);f.p=leg(value);f.bridge=(f.start,f.target,f.p);phases=[]
            def tick(vx,wz,bridge,terminal_deadline_steady=None,fine_phase=False):
                self.assertEqual(bridge,f.bridge);self.assertEqual(vx,value if fine_phase else .4);self.assertEqual(wz,0.);phases.append(fine_phase)
                f.clock.sleep(.1);f.position=next(f.steps);f.packets.append(c.primitive_packet(vx,wz,f.clock.time_ns(),fine_forward_profile=selected(value),fine_phase=fine_phase))
            f.tick=tick;f.run();self.assertEqual(phases,[False,False,True]);self.assertEqual(f.packets[-1]['forward'],value/2.)
    def test_cleanup_receipt_exact_returned_packet_missing_on_failed_send(self):
        for fails in [False,True]:
            order=[];packet=c.primitive_packet(0.,0.,123456789)
            def send():
                order.append('zero')
                if fails:raise OSError('original send failed')
                return packet
            relay=N(send=send,close=lambda:order.append('close'),close_errors=[],process=N(poll=lambda:0))
            result=o.finalize_owned_relay(relay,lambda:order.append('observe') or True)
            self.assertEqual(order,['zero','observe','close']);self.assertTrue(result['stop_verified']);self.assertTrue(result['relay_closed'])
            if fails:self.assertIsNone(result['initial_cleanup_zero_receipt']);self.assertEqual(result['zero_error'],'original send failed')
            else:self.assertEqual(result['initial_cleanup_zero_receipt']['packet'],packet);self.assertIn('parent_pipe',result['initial_cleanup_zero_receipt']['basis'])
        tree=ast.parse(Path(o.__file__).read_text());execute=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute');code=ast.unparse(execute)
        self.assertLess(code.index('closed = finalize_owned_relay'),code.index("emit('initial_cleanup_zero_receipt'"));self.assertLess(code.index("emit('initial_cleanup_zero_receipt'"),code.index('writer.close('))

if __name__=='__main__':unittest.main(verbosity=2)
