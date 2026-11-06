"""Synthetic actual rotate/stable function tests; no ROS, SDK or plant claim."""
import math,unittest
import full_scene_mapping_contract as c
from test_startup_v6 import actual_function,install
from test_joint_heading_stop_v21 import fixture
from test_v4_core import truth

def rotating(yaw,step=.02):
    ns,k,start,_=fixture();state={'yaw':yaw};rows=[]
    def tick(vx=0.,wz=0.,**kw):
        rows.append((vx,wz,kw));k.sleep(.04)
        if wz:state['yaw']=c.wrap(state['yaw']+math.copysign(step,wz))
        ns['relay'].command=(vx,wz,0.)
        return truth(k,yaw=state['yaw']),state['yaw']
    ns.update(math=math,health=lambda:(truth(k,yaw=state['yaw']),state['yaw']),tick=tick)
    rotate=install(actual_function('rotate_to'),ns)
    return ns,k,start,rows,state,rotate

class HeadingCenter(unittest.TestCase):
    def test_inside_outer_but_outside_inner_actually_turns_toward_center_both_signs(self):
        for initial in [-.055,.055]:
            ns,k,start,rows,state,rotate=rotating(initial)
            sample,yaw=rotate(0.,.06)
            positive=[r for r in rows if r[1]]
            self.assertEqual(len(positive),2)
            self.assertTrue(all(r[1]==math.copysign(c.MAX_WZ,-initial) for r in positive))
            self.assertLessEqual(abs(yaw),.03);self.assertGreaterEqual(len(rows)-2,8)
    def test_already_inside_inner_needs_only_original_joint_zero_stop(self):
        ns,k,start,rows,state,rotate=rotating(.02)
        sample,yaw=rotate(0.,.06)
        self.assertFalse(any(r[1] for r in rows));self.assertEqual(yaw,.02)
    def test_quantized_step_crossing_target_zeros_then_uses_original_outer_tolerance(self):
        ns,k,start,rows,state,rotate=rotating(-.009,step=.018)
        sample,yaw=rotate(0.,.0155)
        self.assertEqual(sum(bool(r[1])for r in rows),1)
        self.assertGreater(abs(yaw),.0155/2.);self.assertLessEqual(abs(yaw),.0155)
        self.assertTrue(all(r[1]==0. for r in rows[1:]))
    def test_crossing_target_does_not_accept_settled_pose_outside_outer_band(self):
        ns,k,start,rows,state,rotate=rotating(-.009,step=.032)
        with self.assertRaises(c.Hold):rotate(0.,.0155)
        self.assertEqual(sum(bool(r[1])for r in rows),1)
        self.assertTrue(all(r[1]==0. for r in rows[1:]));self.assertLess(k.monotonic()-start,8.09)
    def test_pi_branch_cut_change_is_not_near_target_crossing(self):
        ns,k,start,rows,state,rotate=rotating(-math.pi+.02)
        sequence=[math.pi-.02,0.]
        def tick(vx=0.,wz=0.,**kw):
            rows.append((vx,wz,kw));k.sleep(.04)
            if wz:state['yaw']=sequence.pop(0)
            ns['relay'].command=(vx,wz,0.)
            return truth(k,yaw=state['yaw']),state['yaw']
        ns['tick']=tick
        sample,yaw=rotate(0.,.08)
        self.assertEqual(sum(bool(r[1])for r in rows),2);self.assertEqual(yaw,0.)
    def test_existing_call_deadline_and_terminal_binding_prevent_extra_positive(self):
        ns,k,start,rows,state,rotate=rotating(-.055)
        with self.assertRaises(c.Hold):rotate(0.,.06,deadline_steady=start+.04,terminal_deadline_steady=start+.04)
        self.assertEqual(len(rows),1);self.assertEqual(rows[0],(0.,c.MAX_WZ,{'terminal_deadline_steady':start+.04}))


    def test_call_only_and_unequal_deadlines_bind_the_earliest_to_original_tick(self):
        for call_seconds,terminal_seconds,expected in [(1.,None,1.),(1.,2.,1.),(2.,1.,1.),(None,1.,1.)]:
            ns,k,start,rows,state,rotate=rotating(-.055)
            rotate(0.,.06,deadline_steady=None if call_seconds is None else start+call_seconds,
                   terminal_deadline_steady=None if terminal_seconds is None else start+terminal_seconds)
            positives=[row for row in rows if row[1]]
            self.assertTrue(positives)
            self.assertTrue(all(row[2]['terminal_deadline_steady']==start+expected for row in positives))

if __name__=='__main__':unittest.main(verbosity=2)
