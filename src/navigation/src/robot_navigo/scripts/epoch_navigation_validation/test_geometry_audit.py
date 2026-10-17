import unittest
import math
from types import SimpleNamespace
import numpy as np
from geometry_audit import Oracle, audit_geometry, twist_pose
class GeometryTests(unittest.TestCase):
 def setUp(self):
  self.scene=SimpleNamespace(resolution=.05,origin=np.array([0.,0.]),free=np.ones((100,100),bool),occupancy=np.zeros((100,100),np.uint8))
  self.oracle=Oracle(self.scene,np.array([[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]]))
 def test_obstacle_inside_polygon_not_only_edges(self):
  self.scene.occupancy[20,20]=1
  self.assertEqual(self.oracle.at([1.025,1.025,0]),'occupied')
 def test_unknown_and_outside_distinguished(self):
  self.scene.occupancy[20,20]=2
  self.assertEqual(self.oracle.at([1.025,1.025,0]),'unknown');self.assertEqual(self.oracle.at([0.,0.,0]),'outside_map')
 def test_sweep_detects_between_endpoint_collision(self):
  self.scene.occupancy[20,40]=1
  self.assertEqual(self.oracle.at([1,1,0]),'free');self.assertEqual(self.oracle.at([3,1,0]),'free')
  self.assertEqual(self.oracle.sweep([1,1,0],[3,1,0])['status'],'collision')
 def test_rotation_sweep_detects_corner(self):
  oracle=Oracle(self.scene,np.array([[-.4,-.1],[.4,-.1],[.4,.1],[-.4,.1]]))
  self.scene.occupancy[45,45]=1
  self.assertEqual(oracle.at([2,2,0]),'free');self.assertEqual(oracle.at([2,2,1.57079632679]),'free')
  self.assertEqual(oracle.sweep([2,2,0],[2,2,1.57079632679])['status'],'collision')
 def test_empty_trajectory_is_not_pass(self):
  result=audit_geometry([dict(event='fixture_start',data=dict(pose=[1,1,0]))],self.scene,self.oracle.polygon)
  self.assertFalse(result['geometry_pass'])
 def test_arc_collision_is_not_hidden_by_free_chord(self):
  oracle=Oracle(self.scene,np.array([[-.01,-.01],[.01,-.01],[.01,.01],[-.01,.01]]))
  self.scene.occupancy[23,29]=1
  start=[1,1,0];command=[1,0,math.pi/2];end=[1+2/math.pi,1+2/math.pi,math.pi/2]
  self.assertEqual(oracle.sweep(start,end)['status'],'free')
  result=oracle.sweep_twist(start,command,1.)
  self.assertEqual(result['status'],'collision')
  self.assertEqual(result['model'],'constant_body_twist')
 def test_full_revolution_is_not_lost_by_wrapped_endpoint(self):
  oracle=Oracle(self.scene,np.array([[-.4,-.1],[.4,-.1],[.4,.1],[-.4,.1]]))
  self.scene.occupancy[45,45]=1
  self.assertEqual(oracle.sweep([2,2,0],[2,2,0])['status'],'free')
  self.assertEqual(oracle.sweep_twist([2,2,0],[0,0,2*math.pi],1.)['status'],'collision')
 def test_arc_speed_bound_and_exact_path_length(self):
  command=[.1,.03,.2];dt=.2;start=[1,1,0];end=twist_pose(start,command,dt)
  result=self.oracle.sweep_twist(start,command,dt)
  self.assertAlmostEqual(result['vertex_speed_bound_mps'],math.hypot(.1,.03)+self.oracle.radius*.2)
  self.assertLessEqual(result['bound_m'],.001)
  rows=[dict(event='fixture_start',data=dict(pose=start)),dict(event='plant_step',monotonic_ns=1,
   data=dict(before=start,after=end,executed=command,dt=dt,interval_start_ns=0,interval_end_ns=200_000_000))]
  audit=audit_geometry(rows,self.scene,self.oracle.polygon)
  self.assertTrue(audit['geometry_pass'])
  self.assertAlmostEqual(audit['path_length_m'],math.hypot(.1,.03)*dt)
  self.assertEqual(audit['trajectory_models'],{'constant_body_twist':1,'legacy_chord':0})
 def test_endpoint_mismatch_cannot_be_certified(self):
  rows=[dict(event='fixture_start',data=dict(pose=[1,1,0])),dict(event='plant_step',monotonic_ns=1,
   data=dict(before=[1,1,0],after=[1,1,0],executed=[.1,0,0],dt=.02,interval_start_ns=0,interval_end_ns=20_000_000))]
  result=audit_geometry(rows,self.scene,self.oracle.polygon)
  self.assertFalse(result['geometry_pass']);self.assertEqual(result['trajectory_validation_failure_count'],1)
 def test_legacy_without_velocity_keeps_chord_and_marks_metric_incomplete(self):
  rows=[dict(event='fixture_start',data=dict(pose=[1,1,0])),dict(event='plant_step',monotonic_ns=1,
   data=dict(before=[1,1,0],after=[1.1,1,0]))]
  result=audit_geometry(rows,self.scene,self.oracle.polygon)
  self.assertTrue(result['geometry_pass'])
  self.assertEqual(result['trajectory_models']['legacy_chord'],1)
  self.assertFalse(result['velocity_integral_metrics_complete'])
 def test_old_executed_metadata_without_wall_interval_stays_legacy(self):
  rows=[dict(event='fixture_start',data=dict(pose=[1,1,0])),dict(event='plant_step',monotonic_ns=1,
   data=dict(before=[1,1,0],after=[1.1,1,0],executed=[.1,0,0],dt=1.))]
  result=audit_geometry(rows,self.scene,self.oracle.polygon)
  self.assertTrue(result['geometry_pass']);self.assertEqual(result['trajectory_models']['legacy_chord'],1)
 def test_dt_mismatch_and_nonfinite_twist_are_rejected(self):
  for command,duration in (([.1,0,0],.01),([math.nan,0,0],.02)):
   rows=[dict(event='fixture_start',data=dict(pose=[1,1,0])),dict(event='plant_step',monotonic_ns=1,
    data=dict(before=[1,1,0],after=[1,1,0],executed=command,dt=duration,interval_start_ns=0,interval_end_ns=20_000_000))]
   with self.subTest(command=command,duration=duration):
    self.assertFalse(audit_geometry(rows,self.scene,self.oracle.polygon)['geometry_pass'])
if __name__=='__main__':unittest.main()
