"""Targeted v5 pure fixtures; never initializes ROS or real command I/O."""
import ast
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as N
import unittest
from unittest import mock
import numpy as np
import full_scene_mapping as owner
import full_scene_mapping_contract as c
import lateral_capability as proof
from test_v4_core import BASE,HERE,Clock,model,policy,target,truth,proof_fixture,tick_fixture


def candidate():
    return c.read_json(BASE/'full_scene_actual_engineering_model_v1/motion_model.v5.CANDIDATE_HOLD.json')


def rect_manifest():
    files={'manifest':BASE/'model_envelope_rectangular_audit_v1/manifest.json',
        'report':BASE/'model_envelope_rectangular_audit_v1/unrestricted_axis_proof.json',
        'independent_review':BASE/'model_envelope_rectangular_independent_review_solmax_v1/review.json'}
    refs={key:dict(path=str(path),sha256=c.sha(path)) for key,path in files.items()}
    return dict(footprint_mode='certified_full_body_rectangle',body_half_xy_m=[.647,.530],padding_m=.15,
        footprint_inflation_in_grid_m=0.,body_envelope_proof=refs,
        artifacts_sha256={str(path):c.sha(path) for path in files.values()},motion_model=dict(max_roll_pitch_rad=.20))


class RectGeometry(unittest.TestCase):
    def test_pinned_whole_model_proof_and_receipt_and_bad_selections(self):
        value=rect_manifest();out=c.footprint_spec(value,HERE/'synthetic_oracle.json')
        self.assertEqual(out['body_half_xy_m'],[.647,.530]);self.assertEqual(out['certified_roll_pitch_cap_rad'],.25)
        for update in [dict(body_half_xy_m=[.646,.530]),dict(padding_m=.149),dict(motion_model=dict(max_roll_pitch_rad=.251)),
            dict(footprint_mode='arbitrary_shrink')]:
            bad=copy.deepcopy(value);bad.update(update)
            with self.assertRaises(c.Hold):c.footprint_spec(bad,HERE/'synthetic_oracle.json')
        bad=copy.deepcopy(value);bad['artifacts_sha256'].pop(bad['body_envelope_proof']['independent_review']['path'])
        with self.assertRaises(c.Hold):c.footprint_spec(bad,HERE/'synthetic_oracle.json')
        bad=copy.deepcopy(value);bad['body_envelope_proof']['report']['sha256']='a'*64
        with self.assertRaises(c.Hold):c.footprint_spec(bad,HERE/'synthetic_oracle.json')
        value['motion_model']['body_half_xy_m']=[.74,.74]
        with self.assertRaisesRegex(c.Hold,'disagree'):c.footprint_spec(value,HERE/'synthetic_oracle.json')

    def test_legacy_default_exact_and_rectangle_filled_cell_point_guard_radius(self):
        legacy=dict(body_half=[.74,.74],padding=.15,footprint_inflation_in_grid_m=0.)
        self.assertEqual(c.footprint_spec(legacy,HERE/'unused')['mode'],'legacy_square')
        with self.assertRaises(c.Hold):c.footprint_spec(dict(legacy,body_half=[.647,.530]),HERE/'unused')
        old_path=BASE/'full_scene_mapping_lateral_owner_v4/review_source_v4/full_scene_mapping_contract.py'
        spec=importlib.util.spec_from_file_location('frozen_v4_contract_delta',old_path);old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old
        exec(compile(old_path.read_text(),str(old_path),'exec'),old.__dict__)
        cells=np.ones((160,160),dtype=np.uint8);cells[81,98]=2
        a=c.OracleGrid(cells,.05,(-4.,-4.,0.));b=old.OracleGrid(cells,.05,(-4.,-4.,0.))
        for x in [-.2,0.,.2]:
            for yaw in [0.,.23,math.pi/2]:self.assertEqual(a.footprint_free(x,0,yaw),b.footprint_free(x,0,yaw))
        self.assertEqual(a.sweep((0,0,0),(.4,.2,.6)),b.sweep((0,0,0),(.4,.2,.6)))
        rect=c.OracleGrid(np.ones((160,160),dtype=np.uint8),.05,(-4.,-4.,0.),(.647,.530))
        self.assertEqual(rect.padded_half_xy,(.797,.68));self.assertAlmostEqual(rect.padded_radius,math.hypot(.797,.68))
        rect.cells[80,80]=2;self.assertFalse(rect.footprint_free(0,0,.4))
        rect.cells[80,80]=1;rect.cells[96,80]=2
        self.assertTrue(rect.footprint_free(0,0,0.));self.assertFalse(rect.footprint_free(0,0,math.pi/2))
        with self.assertRaises(c.Hold):c.guard_hits(np.array([[0.,.7,.5]]),[(0,0,0)],0.,model(),rect)
        c.guard_hits(np.array([[0.,.8,.5]]),[(0,0,0)],0.,model(),rect)
        # Corner-motion bound is computed from both padded axes, without shrinking r*delta.
        points=rect.sweep((0,0,0),(0,0,.4));self.assertGreaterEqual(len(points)-1,math.ceil(rect.padded_radius*.4/.025))

    def test_only_numeric_length_and_accuracy_comparison_guard(self):
        m=candidate();p=policy(length=.3);p.update(max_yaw_error_rad=.12,max_cross_track_m=.03,endpoint_tolerance_m=.075-1e-14)
        grid=N(padded_radius=math.hypot(.797,.68),checked_sweep=lambda *args:[(0,0,0)])
        def route(length):return dict(start_anchor_truth=[0,0,0],waypoints=[dict(id='numeric',pose_truth=[0,length,0],leg_policy=p)])
        self.assertTrue(c.audit_route_geometry(grid,route(.3-1e-14),m)['checked'])
        with self.assertRaises(c.Hold):c.audit_route_geometry(grid,route(.299),m)
        bad=copy.deepcopy(p);bad['endpoint_tolerance_m']=.074
        r=route(.3);r['waypoints'][0]['leg_policy']=bad
        with self.assertRaises(c.Hold):c.audit_route_geometry(grid,r,m)


class Compensation(unittest.TestCase):
    def test_target_single_axis_cancels_forward_bias_and_signs_feedback(self):
        m=candidate()
        for body in [0.,math.pi/2,-math.pi/2,math.pi]:
            for sign in [-1,1]:
                p=policy(body,sign);p.update(max_yaw_error_rad=.12,max_cross_track_m=.03)
                for forward_error in [-.01,0.,.01]:
                    cross=-sign*forward_error;state=dict(cross_track_m=cross)
                    out=c.lateral_heading_target(state,(0,0,body),p,m);delta=out['target_delta_from_cardinal_rad']
                    vx=.02*math.cos(delta)-sign*.30*math.sin(delta)
                    self.assertAlmostEqual(vx,-.4*forward_error*math.cos(delta),places=12)
                    aligned=(0,0,out['target_body_yaw_rad'])
                    self.assertFalse(c.lateral_correction_state(state,aligned,p,4,m)['stop_then_recheck'])
        p=policy();p['max_yaw_error_rad']=.1201
        with self.assertRaises(c.Hold):c.lateral_leg_state((0,0,0),target(p),(0,0,0),p)
        p['max_yaw_error_rad']=.12;c.lateral_leg_state((0,0,0),target(p),(0,0,0),p)
        p['max_cross_track_m']=.03
        bad=copy.deepcopy(m);bad['lateral_models']['positive']['bias_vx_mps']=.10
        with self.assertRaisesRegex(c.Hold,'no silent clamp'):c.lateral_heading_target(dict(cross_track_m=0.),(0,0,0),p,bad)
        with self.assertRaisesRegex(c.Hold,'hard-tube margin'):c.lateral_heading_target(dict(cross_track_m=.029),(0,0,0),p,m)

    def test_cross_window_allows_resume_then_requires_bounded_improvement(self):
        obs=dict(started_steady=10.,start_along_m=.1,start_abs_cross_m=.02)
        self.assertIs(c.feedback_window(obs,dict(cross_track_m=.02),10.99,.2),obs)
        for steady,along in [(11.,.2),(10.5,.31)]:
            with self.assertRaises(c.Hold):c.feedback_window(obs,dict(cross_track_m=.02),steady,along)
        self.assertIsNone(c.feedback_window(obs,dict(cross_track_m=.018),11.,.2))
        m=candidate();p=policy();p.update(max_yaw_error_rad=.12,max_cross_track_m=.03)
        state=dict(cross_track_m=-.018);out=c.lateral_heading_target(state,(0,0,0),p,m)
        decision=c.lateral_correction_state(state,(0,0,out['target_body_yaw_rad']),p,4,m)
        self.assertFalse(decision['stop_then_recheck']);self.assertTrue(decision['observe_cross'])
        with self.assertRaisesRegex(c.Hold,'exhausted'):c.lateral_correction_state(state,(0,0,0),p,4,m)

    def test_directional_original_header_and_twist_yaw_caps_lateral_and_zero(self):
        clk=Clock();m=candidate();s=truth(clk)
        for key in ['twist','original_header_yaw_rate_rps']:
            bad=copy.deepcopy(s)
            if key=='twist':bad[key][5]=.151
            else:bad[key]=.151
            with self.assertRaises(c.Hold):c.directional_yaw_guard(bad,m,1)
        s['original_header_yaw_rate_rps']=.15;c.directional_yaw_guard(s,m,-1)
        self.assertEqual(m['max_truth_yaw_rate_rps'],.5)
        ns,p,rows=tick_fixture();ns['model']['lateral_models']['positive']['max_yaw_rate_rps']=.15
        ns['tick'](bridge=((0,0,0),target(p),p),lateral_raw=.4)
        spin=ns['callbacks'].spin
        def drift():spin();ns['latest']['truth']['original_header_yaw_rate_rps']=.151
        ns['callbacks'].spin=drift
        with self.assertRaisesRegex(c.Hold,'directional cap'):ns['tick']()
        self.assertEqual(len(rows),1)
        tree=ast.parse((HERE/'full_scene_mapping.py').read_text());fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='observe_stop')
        self.assertIn('directional_yaw_guard',ast.unparse(fn))

    def test_root_candidate_declarations_validate_only_in_private_fixture_copy(self):
        original=candidate();self.assertFalse(original['lateral_model_reviewed']);self.assertFalse(original['independent_future_errors_reviewed'])
        m=copy.deepcopy(original);m.update(lateral_model_reviewed=True,independent_future_errors_reviewed=True)
        c.validate_model(m)
        for key,value in [('max_yaw_rate_rps',.501),('nominal_vy_mps',0.),('k_cross_per_sec',-.1),('bias_vx_mps',float('nan'))]:
            bad=copy.deepcopy(m);bad['lateral_models']['positive'][key]=value
            with self.assertRaises(c.Hold):c.validate_model(bad)
        self.assertEqual(candidate(),original)


class ProvenanceAndCleanup(unittest.TestCase):
    def test_original_actual_precheck_and_current5237_real_proof(self):
        path=BASE/'full_scene_actual_engineering_model_v1/lateral_actual_pair_admission.root_reviewed.json';value=proof.read(path)
        admitted=proof.validate_proof(dict(path=str(path),sha256=proof.sha(path)),value['scene_instance_id'],value['sensor_config_sha256'],value['adapter_process_identity'])
        self.assertFalse(admitted['calibration_diagnostic_passed']);self.assertEqual(value['adapter_process_identity']['pid'],5237)
        m=candidate();m.update(lateral_model_reviewed=True,independent_future_errors_reviewed=True);proof.validate_model_evidence(m,admitted)

    def test_synthetic_complete_precheck_origin_and_stale4744_rejected(self):
        for stale in [False,True]:
            with tempfile.TemporaryDirectory() as d:
                path,value,scene,sensor,adapter,objects=proof_fixture(Path(d));art=value['artifacts'];case=objects['case']
                case['source_identities']=dict(new_adapter=adapter,bridge24={'pid':24},projector61={'pid':61})
                precheck=dict(state='PASS',sources=copy.deepcopy(case['source_identities']))
                if stale:precheck['sources']['new_adapter']['pid']=4744
                for key,data in [('case',case),('adapter_identity',precheck)]:
                    file=Path(art[key]['path']);file.write_text(json.dumps(data));art[key]['sha256']=proof.sha(file)
                # Source hash membership remains mandatory even for a new recognized envelope.
                compact=objects['model_measurements'];compact['actual_case_artifact_sha256'].update({k:art[k]['sha256'] for k in ['case','adapter_identity']})
                file=Path(art['model_measurements']['path']);file.write_text(json.dumps(compact));art['model_measurements']['sha256']=proof.sha(file)
                manifest=objects['measurement_manifest'];manifest['files']['model_measurements.json']['sha256']=art['model_measurements']['sha256']
                file=Path(art['measurement_manifest']['path']);file.write_text(json.dumps(manifest));art['measurement_manifest']['sha256']=proof.sha(file)
                path.write_text(json.dumps(value));ref=dict(path=str(path),sha256=proof.sha(path))
                if stale:
                    with self.assertRaises(proof.AdmissionError):proof.validate_proof(ref,scene,sensor,adapter)
                else:proof.validate_proof(ref,scene,sensor,adapter)

    def test_cleanup_reset_keeps_model_cap_and_rectangle_attitude(self):
        tree=ast.parse((HERE/'full_scene_mapping.py').read_text());fn=copy.deepcopy(next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='observe_stop'))
        for age,rp,expected in [(.04,0.,False),(.01,0.,True),(.01,.21,False)]:
            clk=Clock();m=model();m['max_truth_age_sec']=.02;latest={};calls=[0]
            class Relay:
                command=(0.,0.,0.)
                def send(self):
                    calls[0]+=1
                    if calls[0]==1:raise OSError('fixture first send fault')
            def spin():
                clk.sleep(.02);s=truth(clk,age);s['q']=[math.sin(rp/2),0.,0.,math.cos(rp/2)];latest['truth']=s
            ns=dict(node=object(),contract=c,model=m,time=clk,rclpy=N(ok=lambda:True),callbacks=N(spin=spin),
                relay=Relay(),latest=latest,active_lateral=None,stop_observation={})
            exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'<real cleanup cap regression>','exec'),ns)
            self.assertEqual(ns['observe_stop'](),expected);self.assertGreater(calls[0],1)


if __name__=='__main__':unittest.main(verbosity=2)
