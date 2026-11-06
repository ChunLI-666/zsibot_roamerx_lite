"""Pure optional F-along endpoint fixtures. No runtime I/O or pose rewriting."""
import ast,copy,math
from pathlib import Path
from types import SimpleNamespace as N
import unittest
import full_scene_mapping_contract as c
from test_v4_core import model

HERE=Path(__file__).resolve().parent
def policy(heading=0.,value=.025):return dict(kind='straight_fixed_heading',travel_yaw_rad=heading,max_yaw_error_rad=.06,max_cross_track_m=.075,along_endpoint_tolerance_m=value)
def pose(heading,along,cross=0.):return (along*math.cos(heading)-cross*math.sin(heading),along*math.sin(heading)+cross*math.cos(heading),heading)
def route(p):return dict(schema_version=1,scope=c.SCOPE,mode=c.MODE,executable=False,lateral_capability=c.CAPABILITY,
    scene_instance_id=dict(pid=1,starttime_ticks=2,cmdline_sha256='a'*64,boot_id='fixture'),truth_frame_id='map',truth_child_frame_id='BASE_LINK',max_runtime_sec=180.,
    start_anchor_truth=[0.,0.,0.],waypoints=[dict(id='F',pose_truth=[.3,0.,0.],require_stopped=True,leg_policy=p)])

class ForwardAlong(unittest.TestCase):
    def test_typed_bounds_and_original_radial_default_unchanged(self):
        for length in [0.,.3,.6,2.]:
            default=c.translation_endpoint_tolerance((0,0,0),(length,0,0))
            for cross in [0.,.04,.08]:
                target=(length,0,0);current=(length-.02,cross,0)
                self.assertEqual(c.forward_endpoint_state((0,0,0),target,current)['reached'],math.dist(target[:2],current[:2])<=default)
                p=policy();p.pop('along_endpoint_tolerance_m')
                self.assertEqual(c.forward_endpoint_state((0,0,0),target,current,p)['radial_tolerance_m'],default)
        for bad in [None,True,False,0.,-.1,.0751,float('nan'),float('inf'),float('-inf'),'0.025']:
            with self.subTest(value=bad),self.assertRaises(c.Hold):c.forward_endpoint_state((0,0,0),(.3,0,0),(0,0,0),policy(value=bad))
        for good in [1e-12,.025,.075]:self.assertEqual(c.forward_endpoint_state((0,0,0),(.3,0,0),(0,0,0),policy(value=good))['along_endpoint_tolerance_m'],good)
        with self.assertRaises(c.Hold):c.forward_endpoint_state((0,0,0),(0,0,1),(0,0,0),policy())
        p=policy();p.pop('along_endpoint_tolerance_m');p['endpoint_tolerance_m']=.001
        self.assertTrue(c.forward_endpoint_state((0,0,0),(.3,0,0),(.25,0,0),p)['reached'])  # Old radial is not overridden.
    def test_four_headings_cross_point04_with_along_error_point02_passes(self):
        for heading in [0.,math.pi/2,-math.pi/2,math.pi]:
            start=(0.,0.,heading);target=pose(heading,.3);p=policy(heading)
            state=c.forward_endpoint_state(start,target,pose(heading,.28,.04),p)
            self.assertTrue(state['reached']);self.assertAlmostEqual(state['absolute_along_error_m'],.02);self.assertAlmostEqual(state['radial_tolerance_m'],.075)
            self.assertFalse(c.forward_endpoint_state(start,target,pose(heading,.27),p)['reached'])
            self.assertFalse(c.forward_endpoint_state(start,target,pose(heading,.326),p)['reached'])
            self.assertFalse(c.forward_endpoint_state(start,target,pose(heading,.28,.08),p)['reached'])
    def test_route_and_nominal_audit_validate_field_before_geometry(self):
        grid=N(padded_radius=math.hypot(.797,.68),checked_sweep=lambda *a:[(0,0,0)])
        valid=route(policy());c.validate_route(valid)
        audit=c.audit_route_geometry(grid,valid,model());self.assertEqual(audit['legs'][0]['along_endpoint_tolerance_m'],.025);self.assertEqual(audit['legs'][0]['radial_endpoint_tolerance_m'],.075)
        for bad in [None,True,float('nan'),.076]:
            value=route(policy(value=bad))
            with self.assertRaises(c.Hold):c.validate_route(value)
            no_geometry=N(padded_radius=1.,checked_sweep=lambda *a:(_ for _ in ()).throw(AssertionError('invalid endpoint reached geometry')))
            with self.assertRaises(c.Hold):c.audit_route_geometry(no_geometry,value,model())
    def test_real_while_break_rejects_original49mm_along_but_accepts_cross_offset(self):
        tree=ast.parse((HERE/'full_scene_mapping.py').read_text());branch=copy.deepcopy(next(n for n in ast.walk(tree) if isinstance(n,ast.If) and isinstance(n.test,ast.Subscript) and 'forward_endpoint_state' in ast.unparse(n.test)))
        def while_decision(current,p,entry):
            state=dict(contract=c,leg_start=(.1410984,.007272,0.),x=.4410984,y=.007272,heading=0.,truth=dict(xyz=[*current[:2],.33]),yaw=0.,policy=p,
                leg_entry=entry,tolerance=.075,would_continue=False)
            loop=ast.While(test=ast.Constant(True),body=[copy.deepcopy(branch),ast.Assign(targets=[ast.Name(id='would_continue',ctx=ast.Store())],value=ast.Constant(True)),ast.Break()],orelse=[])
            exec(compile(ast.fix_missing_locations(ast.Module(body=[loop],type_ignores=[])),'<actual F break>','exec'),state);return state['would_continue']
        actual=(.3913046,.0114054,0.);p=policy()
        self.assertTrue(while_decision(actual,p,(.1410984,.007272,0.)))
        old=copy.deepcopy(p);old.pop('along_endpoint_tolerance_m');self.assertFalse(while_decision(actual,old,(.1410984,.007272,0.)))
        self.assertFalse(while_decision((.4210984,.047272,0.),p,(.1410984,.007272,0.)))
        with self.assertRaisesRegex(c.Hold,'without actual leg progress'):while_decision((.4410984,.007272,0.),p,(.4360984,.007272,0.))
    def test_actual_post_stop_and_next_lateral_hard_tube_still_reject(self):
        tree=ast.parse((HERE/'full_scene_mapping.py').read_text());branch=copy.deepcopy(next(n for n in ast.walk(tree) if isinstance(n,ast.If) and isinstance(n.test,ast.IfExp) and 'forward_endpoint_state' in ast.unparse(n.test)))
        state=dict(contract=c,math=math,lateral=False,leg_start=(0,0,0),x=.3,y=0.,heading=0.,yaw=0.,policy=policy(),tolerance=.075,truth=dict(xyz=[.274,0.,.33]))
        with self.assertRaisesRegex(c.Hold,'no retry'):exec(compile(ast.fix_missing_locations(ast.Module(body=[branch],type_ignores=[])),'<actual F poststop>','exec'),state)
        lp=dict(kind='lateral_fixed_body_heading',body_yaw_rad=0.,world_travel_yaw_rad=math.pi/2,lateral_sign=1,max_yaw_error_rad=.12,max_cross_track_m=.03,endpoint_tolerance_m=.075)
        with self.assertRaises(c.Hold):c.lateral_leg_state((.4410984,.007272,0.),(.4410984,.307272,0.),(.3913046,.0114054,0.),lp)
        self.assertEqual(lp['endpoint_tolerance_m'],.075)

if __name__=='__main__':unittest.main(verbosity=2)
