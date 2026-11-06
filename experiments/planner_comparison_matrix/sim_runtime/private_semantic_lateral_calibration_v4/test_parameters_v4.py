"""Finite case/sign binding only; fake parent/socket/time, no runtime actions."""
import copy
import json
import math
from pathlib import Path
import unittest
import lateral_contract as c
import test_sustained_backend as backend_fixtures
import test_sustained_parent as parent_fixtures
import semantic_tool_timed as backend

class ParameterCases(unittest.TestCase):
    def case(self,magnitude=.4,duration=1.5):return dict(magnitude=magnitude,probes=[magnitude,-magnitude],duration_sec=duration)
    def test_only_allowed_finite_magnitude_duration_pairs(self):
        for magnitude in [.2,.4,.6]:
            for duration in [1.5,2.7,4.]:self.assertEqual(c.probe_parameters(self.case(magnitude,duration)),(magnitude,duration))
        for value in [0.,-.2,.1,.3,.8,1.,True,float('nan'),float('inf'),'0.4']:
            case=self.case();case['magnitude']=value
            with self.subTest(magnitude=value),self.assertRaises(c.Hold):c.probe_parameters(case)
        for value in [0.,1.,1.499999,4.000001,True,float('nan'),float('inf'),'1.5']:
            case=self.case();case['duration_sec']=value
            with self.subTest(duration=value),self.assertRaises(c.Hold):c.probe_parameters(case)
    def test_case_requires_exact_plus_minus_selected_pair(self):
        for probes in [[.2,-.2],[-.4,.4],[.4,-.6],[.4,.4],[.4,-.4,.4],None]:
            case=self.case();case['probes']=probes
            with self.subTest(probes=probes),self.assertRaises(c.Hold):c.probe_parameters(case)
    def test_payload_can_only_use_selected_sign_or_neutral_no_mixed_axis(self):
        for sign in [.2,-.2,.4,-.4,.6,-.6]:
            frame=backend.frame(sign);c.permitted_payload(frame,sign);backend.permitted(frame,sign)
            zero=backend.frame(sign,neutral=True);c.permitted_payload(zero,sign)
            bads=[dict(frame,lateral=-sign),dict(frame,forward=.01),dict(frame,yaw=.01),dict(frame,LX=sign)]
            if abs(sign)!=.2:bads.append(dict(frame,lateral=math.copysign(.2,sign)))
            for bad in bads:
                with self.subTest(sign=sign,bad=bad):
                    with self.assertRaises(c.Hold):c.permitted_payload(bad,sign)
                    with self.assertRaises(backend.CanaryHold):backend.permitted(bad,sign)
    def test_backend_every_enum_sign_and_short_case_remain_bounded(self):
        fixture=backend_fixtures.SustainedBackend()
        for sign in [.2,-.2,.4,-.4,.6,-.6]:
            result,rows=fixture.run_fake(fixture.fixture(),sign=sign,duration=1.5)
            self.assertEqual(result['motion_packets_sent'],30);self.assertEqual(result['final_zero_attempts'],5)
            self.assertEqual([r['command']['lateral'] for r in rows],[sign]*30+[0.]*5)
        for sign in [0.,.3,-.8,True,float('inf')]:
            with self.assertRaises(backend.CanaryHold):backend.frame(sign)
    def test_parameterized_parent_binds_actual_case_sign_and_duration(self):
        fixture=parent_fixtures.CombinedFixtures()
        for magnitude,duration in [(.4,1.5),(.6,4.)]:
            result,tools,plans,events=fixture.run_parent(magnitude=magnitude,duration=duration)
            self.assertEqual(tools,[0,1]);self.assertTrue(result['diagnostic_passed'])
            self.assertEqual([p['policy_lateral'] for p in result['probes']],[magnitude,-magnitude])
    def test_default_candidate_is_HOLD_point4_for1point5_with_all_original_bounds(self):
        case=c.read(Path(__file__).parent/'case.HOLD.json')
        self.assertFalse(case['executable']);self.assertEqual(c.probe_parameters(case),(.4,1.5))
        self.assertEqual(case['probes'],[.4,-.4]);self.assertEqual(case['max_runtime_sec'],60.)
        for key,value in [('lateral_abort_m',.8),('forward_drift_abort_m',.1),('yaw_abort_rad',.08),('speed_abort_mps',1.),('stop_reserve_m',1.)]:self.assertEqual(case[key],value)

if __name__=='__main__':unittest.main(verbosity=2)
