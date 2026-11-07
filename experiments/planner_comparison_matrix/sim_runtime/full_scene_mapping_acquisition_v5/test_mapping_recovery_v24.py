"""Actual nested tick recovery regressions; fake clock, no ROS/SDK/network."""
from pathlib import Path
import sys
import unittest
from unittest import mock

HERE=Path(__file__).resolve().parent
RUNTIME=HERE if (HERE/'full_scene_mapping.py').is_file() else HERE/'full_scene_mapping_acquisition_v5'
CANONICAL=HERE.parents[2]/'src/zsibot/zsibot_roamerx_lite/experiments/planner_comparison_matrix/sim_runtime/full_scene_mapping_acquisition_v5'
sys.path[:0]=[str(HERE),str(RUNTIME),str(CANONICAL)]
import full_scene_mapping as owner
import full_scene_mapping_contract as contract
import test_v4_core as core


def fixture(mapping=True):
    previous=core.HERE;core.HERE=RUNTIME
    try:namespace,policy,rows=core.tick_fixture()
    finally:core.HERE=previous
    namespace.update(mapping_motion_model=owner.mapping_motion_model,
        MAPPING_CONTROL_PERIOD=owner.MAPPING_CONTROL_PERIOD,MAPPING_INPUT_AGE=owner.MAPPING_INPUT_AGE,
        MAPPING_SWEEP_COMPUTE_MARGIN=owner.MAPPING_SWEEP_COMPUTE_MARGIN,MAPPING_AUDIT_MAX_AGE=owner.MAPPING_AUDIT_MAX_AGE)
    namespace['relay'].mapping_only_relaxed_survey=mapping
    return namespace,rows


class Recovery(unittest.TestCase):
    def test_zero_coast_over1s_then_fresh_tick_sends(self):
        ns,rows=fixture();clock=ns['time'];ns['last_send']=clock.monotonic()
        ns['relay'].command=(0.,0.,0.);clock.sleep(1.2);ns['audit']['checked_steady']=clock.monotonic()
        ns['tick'](.4)
        self.assertEqual(ns['relay'].command,(.4,0.,0.))
        self.assertEqual(len(rows),1)

    def test_strict_keeps_historical_gap_hold(self):
        ns,rows=fixture(False);ns['last_send']=ns['time'].monotonic()-1.2
        with self.assertRaisesRegex(contract.Hold,'control-loop gap'):ns['tick'](.4)
        self.assertFalse(rows);self.assertEqual(ns['relay'].command,(0.,0.,0.))

    def test_150ms_guard_uses_expanded_sweep_age_instead_of_100ms_hardlock(self):
        ns,rows=fixture();original=contract.stopping_sweep
        def slow(*args,**kwargs):
            result=original(*args,**kwargs);ns['time'].sleep(.15);return result
        with mock.patch.object(contract,'stopping_sweep',slow):ns['tick'](.4)
        self.assertEqual(len(rows),1)
        self.assertGreater(rows[0]['final_command_guard']['pose']['guard_elapsed_s'],.1)
        self.assertGreaterEqual(rows[0]['final_command_guard']['pose']['swept_pose_age_upper_s'],.2)

    def test_stale_pose_collision_and_actual_owner_fault_never_send_positive(self):
        for failure in ['stale','collision','owner']:
            ns,rows=fixture()
            if failure=='stale':
                original=ns['health']
                def stale():
                    sample,yaw,cloud,age=original();sample['stamp_ns']-=1_000_000_000;return sample,yaw,cloud,1.
                ns['health']=stale
            elif failure=='collision':ns['oracle'].cells[:]=0
            else:ns['audit']['first_violation']={'reason':'foreign owner'}
            with self.assertRaises(contract.Hold):ns['tick'](.4)
            self.assertFalse(rows);self.assertEqual(ns['relay'].command,(0.,0.,0.))


if __name__=='__main__':unittest.main()
