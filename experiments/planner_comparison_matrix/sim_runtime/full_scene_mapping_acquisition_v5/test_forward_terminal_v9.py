"""Pure measured-stop and guarded-tick terminal controller regressions."""
import ast,copy,math,unittest
from types import SimpleNamespace as N
from unittest import mock
import full_scene_mapping as owner
import full_scene_mapping_contract as c
from test_v4_core import Clock,model,tick_fixture,target
from test_forward_along_v8 import policy as old_policy,route

def policy():
    p=old_policy();p['forward_terminal']=dict(kind='coarse_stop_then_fine_v1',coarse_stop_remaining_m=.10,max_corrections=2,max_extra_work_sec=20.)
    return p

class Plant:
    def __init__(self,start=0.,position=None,steps=(),stops=(),cross=0.,cost=.1):
        self.clock=Clock();self.start=(start,0.,0.);self.target=(start+.3,0.,0.);self.entry=self.start
        self.p=policy();self.position=start if position is None else position;self.steps=iter(steps);self.stops=iter(stops)
        self.cross=cross;self.cost=cost;self.packets=[];self.events=[];self.turn_entries=[];self.stop_count=0;self.bridge=(self.start,self.target,self.p)
    def health(self):return dict(xyz=[self.position,self.cross,.33]),0.
    def tick(self,vx,wz,bridge,terminal_deadline_steady=None):
        assert bridge==self.bridge and vx==c.MAX_VX and wz==0.
        self.clock.sleep(self.cost)
        if terminal_deadline_steady is not None and self.clock.monotonic()>=terminal_deadline_steady:raise c.Hold('fake original final guard deadline')
        self.packets.append(c.primitive_packet(vx,wz,self.clock.time_ns()))
        self.position=next(self.steps,self.position+.02)
    def stable(self,seconds,timeout):
        assert seconds==c.STOP_STABLE and 0<timeout<=8.
        self.clock.sleep(seconds);self.stop_count+=1;self.position=next(self.stops,self.position)
        return self.health()
    def rotate(self,heading,deadline_steady,terminal_deadline_steady):
        assert heading==self.target[2] and terminal_deadline_steady==deadline_steady;self.turn_entries.append(self.position);self.clock.sleep(.3)
        if self.clock.monotonic()>=deadline_steady:raise c.Hold('fake bounded rotate deadline')
        return self.health()
    def run(self):
        return owner.forward_terminal_leg(self.start,self.target,self.entry,self.p,self.health,self.tick,self.stable,self.rotate,
            lambda name,**fields:self.events.append((name,fields)),self.clock)

class ForwardTerminal(unittest.TestCase):
    def test_policy_strict_optin_route_and_audit_pregeometry(self):
        p=old_policy();self.assertIsNone(c.forward_terminal_policy((0,0,0),(.3,0,0),p));c.validate_route(route(p))
        p=policy();c.validate_route(route(p));grid=N(padded_radius=1.,checked_sweep=lambda *a:[(0,0,0)])
        c.audit_route_geometry(grid,route(p),model())
        for field,bads in [('coarse_stop_remaining_m',[None,True,float('nan'),.049,.151,.025]),('max_extra_work_sec',[None,True,0.,20.001,float('inf')]),('max_corrections',[True,0,3,2.0])]:
            for value in bads:
                q=copy.deepcopy(p);q['forward_terminal'][field]=value
                with self.subTest(field=field,value=value),self.assertRaises(c.Hold):c.validate_route(route(q))
        for modifier in [lambda q:q.pop('along_endpoint_tolerance_m'),lambda q:q.update(kind='lateral_fixed_body_heading'),lambda q:q.update(forward_terminal=None)]:
            q=policy();modifier(q)
            with self.assertRaises(c.Hold):c.forward_terminal_policy((0,0,0),(.3,0,0),q)
    def test_actual_25mm_stop_fallback_requires_sameleg_forward_correction(self):
        target=.6985224;f=Plant(start=target-.3,position=target-.1,steps=[.6736375,.6785],stops=[target-.08,.6729908,.6785])
        truth,yaw=f.run();self.assertEqual(len(f.packets),2);self.assertEqual(f.stop_count,3)
        self.assertEqual(f.turn_entries,[.6785])  # Rebound undershoot is corrected before any turn.
        self.assertAlmostEqual(truth['xyz'][0],.6785);self.assertIn('correction_started',[e[0] for e in f.events])
        self.assertEqual([e[1]['correction'] for e in f.events if e[0]=='completed'],[1])
        self.assertTrue(all(p['forward']==.2 and p['yaw']==0. and p['lateral']==0. for p in f.packets))
    def test_coarse_then_fine_and_original_radial_with_cross_offset(self):
        f=Plant(steps=[.10,.20,.27,.29],stops=[.22,.28],cross=.04);truth,yaw=f.run()
        self.assertEqual([n for n,d in f.events],['coarse_stop_requested','coarse_stopped','fine_started','fine_stopped','completed'])
        self.assertTrue(c.forward_endpoint_state(f.start,f.target,(*truth['xyz'][:2],yaw),f.p)['reached'])
        self.assertEqual(f.stop_count,2)
        f=Plant(position=.2,cross=.074,steps=[.28],stops=[.28])
        with self.assertRaisesRegex(c.Hold,'radial miss'):f.run()
    def test_coast_overshoot_and_cross_yaw_unknown_hold_no_retry(self):
        f=Plant(position=.2,stops=[.34])
        with self.assertRaisesRegex(c.Hold,'overshoot'):f.run()
        self.assertEqual(f.packets,[])
        f=Plant(position=.2,cross=.076)
        with self.assertRaises(c.Hold):f.run()
        f=Plant(position=.2);f.health=lambda:({'xyz':[.2,0.,.33]},.061)
        with self.assertRaises(c.Hold):f.run()
        f=Plant(position=.2);f.health=lambda:(_ for _ in ()).throw(c.Hold('original source/age unknown'))
        with self.assertRaisesRegex(c.Hold,'source/age'):f.run()
        self.assertFalse(f.packets)
    def test_two_corrections_max_and_twenty_seconds_not_extended(self):
        f=Plant(position=.2,steps=[.28,.28,.28],stops=[.22,.274,.274,.274])
        with self.assertRaisesRegex(c.Hold,'count exhausted'):f.run()
        self.assertEqual(len(f.packets),3);self.assertEqual(len([n for n,d in f.events if n=='correction_started']),2)
        f=Plant(position=.2,cost=20.,stops=[.22])
        with self.assertRaisesRegex(c.Hold,'deadline'):f.run()
        self.assertEqual(f.packets,[])
    def test_real_tick_final_deadline_preserves_guard_and_no_packet(self):
        state,p,rows=tick_fixture()
        with self.assertRaisesRegex(c.Hold,'terminal deadline'),mock.patch.object(c,'guard_hits',return_value={}):
            state['tick'](bridge=((0,0,0),target(p),p),lateral_raw=.4,terminal_deadline_steady=state['time'].monotonic()+.02)
        self.assertFalse(rows)
    def test_measured_stop_can_rebound_and_no_turn_precedes_final_endpoint(self):
        goal=.6985224;peak=.705753446;settled=.6729908
        self.assertAlmostEqual(peak-settled,.032762646)
        self.assertTrue(c.forward_endpoint_state((goal-.3,0,0),(goal,0,0),(peak,0,0),policy())['reached'])
        self.assertFalse(c.forward_endpoint_state((goal-.3,0,0),(goal,0,0),(settled,0,0),policy())['reached'])
        f=Plant(start=goal-.3,position=goal-.1,steps=[.6736375,.6785],stops=[goal-.08,settled,.6785])
        f.run();self.assertEqual(f.turn_entries,[.6785]);self.assertEqual(f.stop_count,3)
    def test_real_final_guard_after_pump_refuses_terminal_overshoot_only(self):
        for selected in [False,True]:
            state,unused,rows=tick_fixture();p=policy() if selected else old_policy()
            original_spin=state['callbacks'].spin
            state['callbacks'].spin=lambda:(original_spin(),state['latest']['truth']['xyz'].__setitem__(0,.34))
            with mock.patch.object(c,'guard_hits',return_value={}):
                if selected:
                    with self.assertRaisesRegex(c.Hold,'overshoot at final'):state['tick'](c.MAX_VX,0.,bridge=((0,0,0),(.3,0,0),p))
                    self.assertFalse(rows)
                else:
                    state['tick'](c.MAX_VX,0.,bridge=((0,0,0),(.3,0,0),p));self.assertTrue(rows)
    def test_actual_legal_terminal_yaw_crossing_deadline_cannot_send(self):
        from pathlib import Path
        tree=ast.parse((Path(__file__).parent/'full_scene_mapping.py').read_text())
        fn=copy.deepcopy(next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='rotate_to'))
        call=next(n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='rotate_to' and any(k.arg=='terminal_deadline_steady' for k in n.keywords))
        self.assertEqual(ast.unparse(next(k.value for k in call.keywords if k.arg=='terminal_deadline_steady')),'deadline')
        p=policy();p['max_yaw_error_rad']=.08
        c.forward_terminal_policy((0,0,0),(.3,0,.005),p)
        c.straight_leg_state((0,0,0),(.3,0,.005),(.28,0,-.08),p)
        self.assertTrue(c.forward_endpoint_state((0,0,0),(.3,0,.005),(.28,0,-.08),p)['reached'])
        for propagate in [False,True]:
            state,unused,rows=tick_fixture();sample=state['latest']['truth'];original_spin=state['callbacks'].spin
            def spin():
                original_spin();sample.update(xyz=[.28,0.,.4],q=[0.,0.,math.sin(-.04),math.cos(-.04)])
            spin();state['callbacks'].spin=spin
            previous=state['health'];state['health']=lambda:(previous()[0],-.08,*previous()[2:])
            state['stable']=mock.Mock(side_effect=AssertionError('deadline crossing must abort before stable'))
            exec(compile(ast.fix_missing_locations(ast.Module(body=[copy.deepcopy(fn)],type_ignores=[])),'<actual rotate_to>','exec'),state)
            deadline=state['time'].monotonic()+.04
            with mock.patch.object(c,'guard_hits',return_value={}),self.assertRaises(c.Hold):
                state['rotate_to'](.005,deadline_steady=deadline,terminal_deadline_steady=deadline if propagate else None)
            # Source22 also binds a call-only deadline to the final command guard.
            self.assertEqual(len(rows),0)
            state['stable'].assert_not_called()
    def test_old_route_branch_and_all_other_runtime_files_preserved(self):
        from pathlib import Path
        from test_v4_core import BASE as root
        base=root/'full_scene_mapping_lateral_owner_v8/review_source_v8';here=Path(__file__).resolve().parent
        # Source10 explicitly changes relay/proof for fine subcapability; three unchanged runtime files remain exact.
        for n in ['imu_gate.py','dependencies/waypoint_contract.py','dependencies/map_coverage.py']:
            self.assertEqual((here/n).read_bytes(),(base/n).read_bytes())
        self.assertEqual(c.SCOPE,'MAPPING_ACQUISITION_LATERAL_RECT_V5_ONLY')
        self.assertEqual(c.translation_endpoint_tolerance((0,0,0),(.3,0,0)),.075)
        self.assertEqual(c.MAX_VX,.4)

if __name__=='__main__':unittest.main(verbosity=2)
