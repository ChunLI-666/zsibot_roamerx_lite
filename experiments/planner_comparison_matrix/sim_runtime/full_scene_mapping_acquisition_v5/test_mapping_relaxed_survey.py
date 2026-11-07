"""Offline tests only: no ROS context, subprocess, socket, or SDK."""
import copy
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).parent/'full_scene_mapping_acquisition_v5'))
import full_scene_mapping as owner
import full_scene_mapping_contract as contract


class Clock:
    def __init__(self):self.now=0.
    def monotonic(self):return self.now
    def sleep(self,seconds):self.now+=seconds


def truth(x=0.,y=0.,speed=0.):
    return dict(xyz=[x,y,.5],twist=[speed,0.,0.,0.,0.,0.])


def route(*poses):
    return dict(waypoints=[dict(id=str(i),pose_truth=list(p)) for i,p in enumerate(poses)])


class SurveyTests(unittest.TestCase):
    def test_default_flag_and_mapping_purpose(self):
        self.assertFalse(contract.mapping_survey_enabled({}))
        for value in [1,'true',None]:
            with self.assertRaises(contract.Hold):contract.mapping_survey_enabled(dict(mapping_only_relaxed_survey=value))
        with self.assertRaises(contract.Hold):contract.mapping_survey_enabled(dict(mapping_only_relaxed_survey=True))
        self.assertTrue(contract.mapping_survey_enabled(dict(mapping_only_relaxed_survey=True,purpose='mapping_only_relaxed_survey',formal_navigation_acceptance=False)))

    def test_coarse_arrival_ignores_strict_heading_and_fine_endpoint(self):
        clock=Clock();events=[];ticks=[];zeros=[]
        result=owner.mapping_survey(route((.34,0.,math.pi)),lambda:(truth(),0.),lambda *a,**k:ticks.append(a),zeros.append,lambda kind,**k:events.append((kind,k)),clock)
        self.assertEqual(result[2]['reached_points'],1)
        self.assertFalse(ticks)
        self.assertTrue(zeros)
        self.assertFalse(result[2]['formal_navigation_acceptance'])

    def test_required_input_wait_zero_then_resume(self):
        clock=Clock();position=truth();zeros=[];calls=0
        def health():
            nonlocal calls
            calls+=1
            if calls==1:raise contract.InputUnavailable('stale GT')
            return position,0.
        def tick(vx,wz,lateral_raw=0.):position['xyz'][0]+=.4;clock.sleep(.05)
        result=owner.mapping_survey(route((1.,0.,0.)),health,tick,zeros.append,lambda *a,**k:None,clock)
        self.assertEqual(result[2]['reached_points'],1)
        self.assertIn('required_input_wait: stale GT',zeros)

    def test_collision_rejected_zero_skip_and_next_point(self):
        clock=Clock();zeros=[];events=[];requests=[]
        def blocked(*a,**k):requests.append((a,k));clock.sleep(.05);raise contract.CollisionBlocked('occupied')
        result=owner.mapping_survey(route((2.,0.,0.),(.1,0.,math.pi)),lambda:(truth(),0.),blocked,zeros.append,lambda kind,**k:events.append(kind),clock)
        self.assertEqual(result[2]['skipped_points'],1)
        self.assertEqual(result[2]['reached_points'],1)
        self.assertIn('collision_rejected',zeros)
        self.assertIn('point_skipped',events)
        self.assertTrue(any(k.get('lateral_raw') for a,k in requests))

    def test_owner_recorder_writer_unknown_faults_are_fatal(self):
        for reason in ['owner changed','recorder lost','bounded writer evidence gap','unknown fault']:
            def fatal(*a,**k):raise contract.Hold(reason)
            with self.assertRaisesRegex(contract.Hold,reason):
                owner.mapping_survey(route((2.,0.,0.)),lambda:(truth(),0.),fatal,lambda *a:None,lambda *a,**k:None,Clock())

    def test_direction_switch_coasts_then_progresses(self):
        clock=Clock();position=truth();zeros=[];commands=[]
        def tick(vx,wz,lateral_raw=0.):
            commands.append((vx,wz,lateral_raw));clock.sleep(.05)
            position['xyz'][0]+=vx
            position['xyz'][1]+=lateral_raw
        result=owner.mapping_survey(route((1.,0.,0.),(.8,1.,2.)),lambda:(position,0.),tick,zeros.append,lambda *a,**k:None,clock)
        self.assertEqual(result[2]['reached_points'],2)
        self.assertIn('primitive_direction_switch',zeros)
        self.assertTrue(any(c[2] for c in commands))

    def test_goal_behind_turns_and_lateral_corrects_current_body_error(self):
        self.assertNotEqual(owner.mapping_command(truth(),0.,(-1.,0.,0.))[1],0.)
        self.assertGreater(owner.mapping_command(truth(),0.,(.1,1.,0.))[2],0.)
        self.assertLess(owner.mapping_command(truth(),math.pi/2.,(1.,.1,0.))[2],0.)

    def test_speed_excess_expands_guard_model_without_mutating_static_model(self):
        original=dict(max_truth_speed_mps=.8,max_truth_yaw_rate_rps=.5,lateral_models={name:dict(max_truth_speed_mps=.45,max_yaw_rate_rps=.15,stop_distance_reserve_m=.1) for name in ['positive','negative']})
        frozen=copy.deepcopy(original);sample=truth(speed=1.4);sample['twist'][5]=.9
        effective=owner.mapping_motion_model(original,sample)
        self.assertGreater(effective['max_truth_speed_mps'],1.4)
        self.assertGreater(effective['max_truth_yaw_rate_rps'],.9)
        self.assertGreater(effective['lateral_models']['negative']['max_truth_speed_mps'],1.4)
        self.assertEqual(original,frozen)

    def test_no_progress_skips_after20s_and_advances(self):
        clock=Clock();events=[]
        result=owner.mapping_survey(route((2.,0.,0.),(.1,0.,0.)),lambda:(truth(),0.),lambda *a,**k:clock.sleep(.05),lambda *a:None,lambda kind,**k:events.append((kind,k)),clock)
        self.assertEqual((result[2]['reached_points'],result[2]['skipped_points']),(1,1))
        self.assertTrue(any(kind=='point_skipped' and value['reason']=='no_distance_improvement_for_20s' for kind,value in events))

    def test_turn_hysteresis_and_lateral_negative_x_do_not_fight_hint(self):
        target=(2.*math.cos(.3),2.*math.sin(.3),0.)
        self.assertEqual(owner.mapping_command(truth(),0.,target)[0],.4)
        self.assertEqual(owner.mapping_command(truth(),0.,target,previous=(0.,.3,0.))[1],.3)
        target=(2.*math.cos(.14),2.*math.sin(.14),0.)
        self.assertEqual(owner.mapping_command(truth(),0.,target,previous=(0.,.3,0.))[0],.4)
        policy=dict(kind='lateral_fixed_body_heading',body_yaw_rad=2.)
        self.assertEqual(owner.mapping_command(truth(),0.,(-.2,2.,0.),policy),(0.,0.,.4))

    def test_physical_fullbody_and_raw_hit_protection(self):
        grid=contract.OracleGrid(np.ones((120,120),dtype=np.uint8),.05,(-3.,-3.,0.),(.74,.35),.15)
        grid.cells[60,85]=0
        with self.assertRaises(contract.CollisionBlocked):grid.checked_sweep((0.,0.,0.),(1.,0.,0.))
        model=dict(floor_z_m=0.,floor_hit_tolerance_m=.03,obstacle_top_m=2.)
        with self.assertRaises(contract.CollisionBlocked):contract.guard_hits(np.array([[.8,0.,.5]]),[(0.,0.,0.)],0.,model,grid)


if __name__=='__main__':unittest.main()
