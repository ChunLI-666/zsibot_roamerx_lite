"""Regression checks for independent scene/lineage acceptance, not ROS substitutes."""
import copy
from types import SimpleNamespace
import unittest
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'epoch_navigation_validation'))
import numpy as np
from recovery_scenarios import mark_rear_wall,audit_bt_recovery,audit_dynamic_geometry


class Scenarios(unittest.TestCase):
    def test_wall_rounding_preserves_physical_clearance(self):
        scene=SimpleNamespace(resolution=.05,origin=np.array([-5.,-5.]),free=np.ones((200,200),dtype=bool),occupancy=np.zeros((200,200),dtype=int))
        detail=mark_rear_wall(scene,[-.153,0.,0.],[[-.435,-.225],[-.435,.225],[.435,.225],[.435,-.225]])
        self.assertGreaterEqual(detail['clearance_m'],.03)
        self.assertLess(detail['clearance_m'],.08)
        for col,row in detail['cells']:self.assertEqual(scene.occupancy[row,col],1);self.assertFalse(scene.free[row,col])

    def test_dynamic_world_does_not_retroactively_collide_with_old_trajectory(self):
        before=SimpleNamespace(resolution=.05,origin=np.array([-2.,-2.]),free=np.ones((100,100),dtype=bool),occupancy=np.zeros((100,100),dtype=int))
        after=copy.deepcopy(before);after.occupancy[40,40]=1;after.free[40,40]=False
        polygon=np.array([[-.1,-.1],[-.1,.1],[.1,.1],[.1,-.1]])
        def step(start,end,a,b):return dict(event='plant_step',monotonic_ns=end,data=dict(interval_start_ns=start,interval_end_ns=end,before=a,after=b,executed=[(b[0]-a[0])/((end-start)*1e-9),0.,0.],dt=(end-start)*1e-9))
        rows=[dict(event='fixture_start',data=dict(pose=[0.,0.,0.])),step(1,1_000_000_001,[0.,0.,0.],[1.,0.,0.]),step(1_000_000_001,2_000_000_001,[1.,0.,0.],[1.,0.,0.])]
        self.assertTrue(audit_dynamic_geometry(rows,before,after,polygon,dict(effective_ns=1_000_000_001,pose=[1.,0.,0.]))['geometry_pass'])
        with self.assertRaisesRegex(AssertionError,'crosses'):
            audit_dynamic_geometry(rows,before,after,polygon,dict(effective_ns=1_000_000_000,pose=[1.,0.,0.]))

    def chain(self):
        def state(at,kind,seq,active=True,reason='installed'):
            return dict(monotonic_ns=at,event='/nav_epoch/execution',data=dict(active=active,reason=reason,token=dict(navigation_session_id='same',task_sequence=1,plan_sequence=seq,execution_kind=kind)))
        return [state(1,0,1),state(2,0,1,False,'Failed to make progress'),state(3,1,2),state(4,1,2,False,'backup_finished'),dict(monotonic_ns=5,event='/back_up_epoch/_action/status',data=dict(status_list=[dict(status=4)])),state(6,0,3)]

    def test_complete_real_action_lineage(self):
        self.assertTrue(audit_bt_recovery(self.chain())['automatic_recovery'])

    def test_direct_backup_without_progress_failure_is_not_auto_recovery(self):
        rows=self.chain();del rows[1]
        with self.assertRaisesRegex(AssertionError,'progress-checker'):audit_bt_recovery(rows)

    def test_new_mission_does_not_count_as_recovery_of_same_goal(self):
        rows=self.chain();rows[-1]['data']['token']['task_sequence']=2
        with self.assertRaisesRegex(AssertionError,'mission identity'):audit_bt_recovery(rows)

    def test_canceled_backup_does_not_count_as_success(self):
        rows=self.chain();rows[-2]['data']['status_list'][0]['status']=5
        with self.assertRaisesRegex(AssertionError,'SUCCEEDED'):audit_bt_recovery(rows)

if __name__=='__main__':unittest.main()
