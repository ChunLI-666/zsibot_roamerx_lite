"""Pure bounded-SIM v5 delta fixtures. No ROS, child, signal or UDP."""
import ast,contextlib,copy,hashlib,io,json,math
from pathlib import Path
from types import SimpleNamespace as N
import tempfile,unittest
from unittest import mock
import numpy as np
import full_scene_mapping as owner
import full_scene_mapping_contract as c
import mapping_lateral_relay as relay
import lateral_capability as proof

HERE=Path(__file__).resolve().parent
BASE=next(p for p in HERE.parents if p.name=='planner_comparison_20261105')

class Clock:
    def __init__(self):self.ns=10_000_000_000;self.sleeps=[]
    def monotonic(self):return self.ns/1e9
    def monotonic_ns(self):return self.ns
    def time_ns(self):return 1_000_000_000+self.ns-10_000_000_000
    def sleep(self,seconds):self.sleeps.append(seconds);self.ns+=round(seconds*1e9)

class Socket:
    def __init__(self,clock):self.clock=clock;self.rows=[];self.closed=False
    def sendto(self,data,target):self.rows.append((json.loads(data),self.clock.ns));return len(data)
    def close(self):self.closed=True

def model():
    value=dict(max_truth_age_sec=.03,max_truth_speed_mps=.6,max_truth_yaw_rate_rps=.5,stop_distance_reserve_m=.15,
        stop_angle_reserve_rad=.15,forward_drift_reserve_m=.05,turn_drift_reserve_m=.1,forward_yaw_reserve_rad=.05,
        max_roll_pitch_rad=.2,body_z_min_m=.2,body_z_max_m=.8,floor_z_m=0.,floor_hit_tolerance_m=.02,
        obstacle_top_m=1.5,basis='root_reviewed_engineering_bound_sim_only',lateral_capability=c.CAPABILITY,
        lateral_model_reviewed=True,lateral_calibration_proof_sha256='a'*64,
        future_error_reference=c.ACTUAL_GT_STOP_REFERENCE,independent_future_errors_reviewed=True)
    value['lateral_models']={name:dict(raw_lateral=sign*.4,max_truth_speed_mps=.5,max_yaw_rate_rps=.5,stop_distance_reserve_m=.2,
        drift_reserve_m=.02,yaw_reserve_rad=.02,min_controllable_leg_m=.02,endpoint_accuracy_bound_m=.01,
        bias_vx_mps=0.,nominal_vy_mps=.3,k_cross_per_sec=0.)
        for name,sign in [('positive',1),('negative',-1)]}
    return value
def grid():return c.OracleGrid(np.ones((160,160),dtype=np.uint8),.05,(-4.,-4.,0.))
def policy(body=0.,sign=1,length=1.):
    return dict(kind='lateral_fixed_body_heading',body_yaw_rad=body,world_travel_yaw_rad=c.wrap(body+sign*math.pi/2),
        lateral_sign=sign,max_yaw_error_rad=.08,max_cross_track_m=.15,endpoint_tolerance_m=min(.15,length/4),root_tracking_approved=False)
def target(p,length=1.):return (length*math.cos(p['world_travel_yaw_rad']),length*math.sin(p['world_travel_yaw_rad']),p['body_yaw_rad'])
def truth(clock,age=.005,yaw=0.):
    return dict(stamp_ns=clock.time_ns()-round(age*1e9),received_steady=clock.monotonic(),frame_id='map',child_frame_id='BASE_LINK',
        xyz=[0.,0.,.4],q=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)],twist=[0.]*6,stamp_regressed=False,original_header_yaw_rate_rps=0.)

def tick_fixture(future=False):
    tree=ast.parse((HERE/'full_scene_mapping.py').read_text());fn=copy.deepcopy(next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='tick'))
    fn.body=[ast.copy_location(ast.Global(names=n.names),n) if isinstance(n,ast.Nonlocal) else n for n in fn.body]
    clk=Clock();sample=truth(clk);cloud=dict(stamp_ns=sample['stamp_ns'],raw_received_steady=clk.monotonic(),points=np.empty((0,3)),pose={})
    def spin():
        clk.sleep(.01);sample.update(truth(clk,age=-.02 if future else .005));cloud.update(stamp_ns=sample['stamp_ns'],raw_received_steady=clk.monotonic())
    rows=[]
    class R:
        command=(0.,0.,0.);delay=0.
        def send(self,vx,wz,preflight,lateral_raw):
            preflight(clk.time_ns(),clk.monotonic());self.command=(vx,wz,lateral_raw)
            packet=c.primitive_packet(vx,wz,clk.time_ns(),lateral_raw);clk.sleep(self.delay);return packet
    ns=dict(contract=c,math=math,time=clk,stopped=[],last_send=None,last_request=None,active_lateral=None,
        callbacks=N(spin=spin),health=lambda:(sample,0.,cloud,(clk.time_ns()-sample['stamp_ns'])/1e9),
        latest=dict(truth=sample),oracle=grid(),model=model(),clock_pair=None,relay=R(),writer=N(submit=lambda k,v:rows.append(v) or True),
        audit=dict(valid=True,first_violation=None,checked_steady=clk.monotonic()))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'<actual v4 tick>','exec'),ns)
    return ns,policy(),rows

class ProtocolAndTime(unittest.TestCase):
    def test_new_scope_magnitude_single_axes_and_packet_bounds(self):
        for vx,wz,lat in [(0,0,.4),(0,0,-.4),(.4,0,0),(0,.3,0),(0,-.3,0),(0,0,0)]:
            p=c.primitive_packet(vx,wz,1_000_000_000,lat);q=relay.validated_axes(p,1_000_000_000)
            self.assertEqual((q['forward'],q['lateral'],q['yaw']),(vx/2,lat,wz))
        for values in [(0,0,.2),(0,0,.6),(-.4,0,0),(.4,0,.4),(0,.3,.4),(0,True,0)]:
            with self.assertRaises(c.Hold):c.primitive_packet(*values[:2],1_000_000_000,values[2])
        p=c.primitive_packet(0,0,1_000_000_000,.4)
        relay.validated_axes(p,1_100_000_000)
        for now in [999_999_999,1_100_000_001]:
            with self.assertRaises(ValueError):relay.validated_axes(p,now)
    def test_expiry_deadline_removes_selector_phase_and_keeps_zero_cleanup(self):
        clk=Clock();sock=Socket(clk);stream=io.StringIO(json.dumps(c.primitive_packet(0,0,clk.time_ns(),.4))+'\n');turn=[0];timeouts=[]
        def select(r,w,e,timeout):
            i=turn[0];turn[0]+=1;timeouts.append(timeout)
            if i in (0,3):return r,[],[]
            clk.sleep(timeout);return [],[],[]
        result=relay.run_loop(sock,stream,lambda:None,clk,select)
        first_zero=next(stamp for value,stamp in sock.rows if value==relay.ZERO)
        self.assertEqual(first_zero-10_000_000_000,100_000_000)
        self.assertEqual(result['expired_before_send'],1);self.assertTrue(sock.closed)
        self.assertTrue(all(v==relay.ZERO for v,t in sock.rows[-5:]));self.assertTrue(all(0<=t<=.05 for t in timeouts))
    def test_accept_then_dispatch_pause_cannot_replay_expired_positive(self):
        clk=Clock();original=clk.monotonic;count=[0]
        def monotonic():
            count[0]+=1;value=original()
            if count[0]==2:clk.sleep(.11)
            return value
        clk.monotonic=monotonic;sock=Socket(clk);stream=io.StringIO(json.dumps(c.primitive_packet(0,0,clk.time_ns(),.4))+'\n')
        result=relay.run_loop(sock,stream,lambda:None,clk,lambda r,w,e,t:(r,[],[]))
        self.assertEqual(result['accepted'],1);self.assertEqual(result['expired_before_send'],1)
        self.assertTrue(all(v==relay.ZERO for v,t in sock.rows))
    def test_oversleep_is_not_a_hardcap_but_no_stale_resend(self):
        clk=Clock();sock=Socket(clk);stream=io.StringIO(json.dumps(c.primitive_packet(0,0,clk.time_ns(),.4))+'\n');count=[0]
        def select(r,w,e,t):
            count[0]+=1
            if count[0]==2:clk.sleep(.3);return [],[],[]
            return r,[],[]
        relay.run_loop(sock,stream,lambda:None,clk,select)
        self.assertEqual(sock.rows[0][0]['lateral'],.4)
        self.assertTrue(all(v==relay.ZERO for v,t in sock.rows[1:]))
    def test_reject_mixed_and_parent_failure_always_close_owned_socket(self):
        clk=Clock();sock=Socket(clk);packet=c.primitive_packet(0,0,clk.time_ns(),.4);packet['yaw']=.3
        relay.run_loop(sock,io.StringIO(json.dumps(packet)+'\n'),lambda:None,clk,lambda r,w,e,t:(r,[],[]))
        self.assertTrue(all(v==relay.ZERO for v,t in sock.rows));self.assertTrue(sock.closed)
        sock=Socket(clk)
        with self.assertRaises(ValueError):relay.run_loop(sock,io.StringIO(),lambda:None,clk,parent_live=lambda:False)
        self.assertEqual(len(sock.rows),5);self.assertTrue(sock.closed)

class GeometryAndGT(unittest.TestCase):
    def test_four_cardinal_headings_both_signs_and_short_progress(self):
        for body in [0.,math.pi/2,-math.pi/2,math.pi]:
            for sign in [-1,1]:
                p=policy(body,sign,.05);end=target(p,.05)
                self.assertFalse(c.lateral_leg_state((0,0,body),end,(0,0,body),p)['reached'])
                self.assertTrue(c.lateral_leg_state((0,0,body),end,end,p)['reached'])
                self.assertTrue(c.require_translation_progress((0,0,body),end,(0,0,body),end,p['endpoint_tolerance_m'])['required'])
                with self.assertRaises(c.Hold):c.require_translation_progress((0,0,body),end,end,end,p['endpoint_tolerance_m'])
        with self.assertRaises(c.Hold):c.lateral_leg_state((0,0,.1),target(policy(.1)),(0,0,.1),policy(.1))
        self.assertEqual(c.translation_endpoint_tolerance((0,0,0),(.05,0,0)),.0125)
        self.assertFalse(c.require_translation_progress((0,0,0),(0,0,1),(0,0,0),(0,0,1),.15)['required'])
    def test_model_GT_cap_is_HOLD_and_other_sensor_point3_unchanged(self):
        for cap in [None,0.,.0500001,True,float('nan')]:
            with self.assertRaises(c.Hold):c.truth_age_limit(dict(max_truth_age_sec=cap))
        self.assertEqual(c.truth_age_limit(dict(max_truth_age_sec=.025)),.025)
        clk=Clock();s=truth(clk,age=.031)
        c.fresh(s,clk.time_ns(),clk.monotonic())
        with self.assertRaises(c.Hold):c.command_time_pose_guard(s,clk.time_ns(),clk.monotonic(),clk.monotonic(),.03,.03)
        s=truth(clk,age=.029);self.assertAlmostEqual(c.command_time_pose_guard(s,clk.time_ns(),clk.monotonic(),clk.monotonic(),.03,.03)['header_age_s'],.029)
    def test_actual_reference_same_cap_full_padding_and_future_errors(self):
        clk=Clock();m=model();m['max_truth_age_sec']=.025;p=policy();s=truth(clk,age=.01)
        sweep,inf,ref=c.actual_gt_lateral_stopping_sweep(grid(),s,((0,0,0),target(p),p),m,clk.time_ns(),clk.monotonic())
        bare,bareinf=c.stopping_sweep(grid(),(0,0,0),'lateral',1,m,.025,policy=None)
        self.assertEqual(sweep,bare);self.assertEqual(inf,bareinf);self.assertEqual(ref['max_truth_age_sec'],.025)
        self.assertEqual(ref['input_age_upper_s'],.025);self.assertEqual(ref['nominal_body_half_m'],.89)
        self.assertTrue(ref['independent_future_errors_retained']);self.assertFalse(ref['padding_consumed_as_error_budget'])
        self.assertAlmostEqual(ref['signed_stop_tail_sec'],.35)
        g=grid();g.cells[80,97]=2
        self.assertFalse(g.footprint_free(0,0,0))
    def test_prethreshold_yaw_and_cross_trigger_bounded_correction(self):
        p=policy();state=dict(cross_track_m=.01)
        self.assertTrue(c.lateral_correction_state(state,(0,.2,.041),p,0,model())['stop_then_recheck'])
        self.assertFalse(c.lateral_correction_state(dict(cross_track_m=.076),(0,.2,0),p,0,model())['stop_then_recheck'])
        self.assertFalse(c.lateral_correction_state(state,(0,.2,.01),p,0,model())['stop_then_recheck'])
        with self.assertRaises(c.Hold):c.lateral_correction_state(state,(0,.2,.041),p,4,model())
    def test_real_tick_keeps_signed_stop_reference_on_zero(self):
        ns,p,rows=tick_fixture();bridge=((0,0,0),target(p),p)
        ns['tick'](bridge=bridge,lateral_raw=.4);ns['tick']()
        self.assertEqual([v['primitive'] for v in rows],['lateral','lateral'])
        self.assertEqual(rows[-1]['actual_GT_stop_reference']['max_truth_age_sec'],.03)
        self.assertEqual(rows[-1]['policy_lateral_raw'],0.)
        with self.assertRaises(c.Hold):ns['tick'](bridge=((0,0,0),target(policy(sign=-1)),policy(sign=-1)),lateral_raw=-.4)
    def test_final_guard_rechecks_cap_and_renewal(self):
        ns,p,rows=tick_fixture()
        def pause(*args):ns['time'].sleep(.031);return {}
        with mock.patch.object(c,'guard_hits',side_effect=pause),self.assertRaises(c.Hold):ns['tick'](bridge=((0,0,0),target(p),p),lateral_raw=.4)
        self.assertFalse(rows)
        ns,p,rows=tick_fixture(future=True);ns['last_send']=ns['time'].monotonic()-.03
        def pause2(*args):ns['time'].sleep(.03);return {}
        with mock.patch.object(c,'guard_hits',side_effect=pause2),self.assertRaisesRegex(c.Hold,'renewal'):ns['tick'](bridge=((0,0,0),target(p),p),lateral_raw=.4)
    def test_write_pause_does_not_refresh_renewal_anchor(self):
        ns,p,rows=tick_fixture();ns['relay'].delay=.11
        with self.assertRaisesRegex(c.Hold,'write/dispatch'):ns['tick'](bridge=((0,0,0),target(p),p),lateral_raw=.4)
        self.assertAlmostEqual(ns['time'].monotonic()-ns['last_send'],.11);self.assertFalse(rows)

class Batch(unittest.TestCase):
    def test_all_callbacks_and_empty_cycles_keep_fixed0_and32_cap(self):
        clk=Clock();calls=[];batch=None
        def once(timeout_sec):
            calls.append(timeout_sec);clk.sleep(.0001)
            if len(calls)%2:batch.wrap(lambda:None)()
        batch=owner.BoundedCallbacks(N(spin_once=once),clk);batch.spin()
        self.assertEqual(batch.dispatched,32);self.assertEqual(len(calls),63);self.assertEqual(set(calls),{0.})
    def test_slow_callback_or_error_is_not_swallowed_or_followed(self):
        clk=Clock();calls=[];batch=None
        def once(timeout_sec):calls.append(timeout_sec);batch.wrap(lambda:clk.sleep(.12))()
        batch=owner.BoundedCallbacks(N(spin_once=once),clk);batch.spin();self.assertEqual(len(calls),1)
        batch=owner.BoundedCallbacks(N(spin_once=lambda **kw:(_ for _ in ()).throw(ValueError('callback'))),clk)
        with self.assertRaisesRegex(ValueError,'callback'):batch.spin()

def proof_fixture(root):
    """Synthetic metadata/tiny bytes only; actual reviewed source bytes reused."""
    artifacts={};objects={}
    def file(key,value,raw=False):
        path=root/(key.replace('/','_')+'.json')
        path.write_bytes(value if raw else (json.dumps(value)+'\n').encode())
        artifacts[key]=dict(path=str(path),sha256=proof.sha(path));objects[key]=value
        return path
    scene=dict(pid=7,starttime_ticks=8,cmdline_sha256='f'*64,boot_id='fixture')
    adapter=dict(pid=70,starttime_ticks=80,cmdline_sha256='e'*64,argv=['fixture_adapter'],boot_id='fixture')
    config=file('sensor_config',b'{}',True);sensor_sha=proof.sha(config);file('adapter_identity',adapter)
    file('case',dict(scope=proof.CAL_SCOPE,magnitude=.4,probes=[.4,-.4],duration_sec=1.5,max_runtime_sec=60.,scene_instance_id=scene,sensor_config_artifact=dict(sha256=sensor_sha)))
    cal=BASE/'sim_semantic_lateral_bounded_amplitude_v4/review_source'
    file('source_manifest',(cal/'source_manifest_v4.json').read_bytes(),True)
    for name in proof.CAL_SOURCES:file('source_'+name,(cal/name).read_bytes(),True)
    file('bag_metadata',b'fixture metadata',True);bag=root/'fixture.mcap';bag.write_bytes(b'pure fake bag bytes; no recorder')
    bag_sha=proof.sha(bag);measurements=[];acceptances=[]
    for index,sign in [(0,.4),(1,-.4)]:
        start=10_000_000_000+index*3_000_000_000;rows=[]
        for i in range(28):
            moving=i<23;command=dict(version=1,action='walk',forward=0.,lateral=sign if moving else 0.,yaw=0.)
            payload=json.dumps(command,separators=(',',':')).encode();stamp=start+i*50_000_000
            rows.append(dict(event='actual_socket_sendto_interval_not_actuation',send_success=True,target=['127.0.0.1',7447],bytes=len(payload),payload_sha256=hashlib.sha256(payload).hexdigest(),command=command,send_before_steady_ns=stamp,send_after_steady_ns=stamp+1,send_before_unix_ns=stamp,send_after_unix_ns=stamp+1))
        file(f'probe{index}_trace',(''.join(json.dumps(v)+'\n' for v in rows)).encode(),True)
        errors=[] if index==0 else [dict(stage='active',type='Hold',message='stale/future/missing/empty original sensor: /front_camera/image/compressed')]
        file(f'probe{index}_backend',dict(sign=sign,requested_active_sec=1.5,motion_packets_sent=23,final_zero_attempts=5,final_zero_logged_successes=5,close_returned=True,completed=index==0,errors=errors))
        acceptance=dict(index=index,state='MEASURED_SUSTAINED_LATERAL_PROBE',policy_lateral=sign,
            sustained=dict(final_active1s_complete=True,strict_stopped_at_cutoff=True,signed_net_stopped_body_y_m=.2,signed_final_active1s_body_vy_median_mps=.2))
        file(f'probe{index}_acceptance',acceptance);acceptances.append(acceptance)
        measurements.append(dict(index=index,raw_lateral=sign,strict_stop_confirmed=True,retained_signed_net_m=.2,
            nonzero_packets=23,neutral_packets=5,backend_completed=index==0,backend_errors=errors,
            max_original_speed3d_mps=.37,max_original_header_yaw_derivative_rps=.14,neutral_to_stop_start_path_xy_m=.05,
            future_excursions_by_phase=dict(whole_postneutral=dict(max_signed_along_m=.03,max_abs_cross_m=.008,max_abs_yaw_rad=.021))))
    file('probe1_plan',dict(prior_positive_acceptance_sha256=artifacts['probe0_acceptance']['sha256']))
    file('result',dict(scope=proof.CAL_SCOPE,diagnostic_passed=False,state='HOLD_DIAGNOSTIC_FAILED',failure_stop_and_tail_verified=True,
        recorder_exited=True,source_signals_sent=0,host_signals_sent=0,raw_cdr_reencoded=False,source_package_sha256=proof.CAL_SOURCE_SHA,
        observed_topic_counts={t:1 for t in proof.RAW_TOPICS},probes=acceptances,
        bag_audit=dict(complete=True,files=[dict(sha256=bag_sha)],metadata_sha256=artifacts['bag_metadata']['sha256'])))
    compact=dict(schema='locnav.root_lateral_model_measurement_input.v1',original_case_state='HOLD_DIAGNOSTIC_FAILED',diagnostic_passed=False,
        source_package_sha256=proof.CAL_SOURCE_SHA,measured_raw_signs=[.4,-.4],bag=dict(sha256=bag_sha),root_engineering_model=None,runtime_approved=False,
        probes=measurements,actual_case_artifact_sha256={k:v['sha256'] for k,v in artifacts.items()})
    file('model_measurements',compact)
    file('coverage_correction',dict(all_motion_stop_and_measurement_headers_common=True,original_movement_stop_future_values_changed=False,motion_range_headers=100,common_exact_position_quaternion_linear_angular=110))
    file('measurement_manifest',dict(runtime_approved=False,files={'model_measurements.json':dict(sha256=artifacts['model_measurements']['sha256']),'coverage_correction_v2.json':dict(sha256=artifacts['coverage_correction']['sha256'])}))
    value=dict(scope=proof.CAL_SCOPE,capability=proof.CAPABILITY,admission_kind='root_reviewed_auxiliary_camera_interrupted_pair',
        basis='actual_semantic_closed_bag_measurement',root_measurement_reviewed=True,root_limited_probe_approved=True,hard_plant_guarantee=False,
        scene_instance_id=scene,sensor_config_sha256=sensor_sha,adapter_process_identity=adapter,artifacts=artifacts,
        source_package_sha256=proof.CAL_SOURCE_SHA,mcap_files=[dict(path=str(bag),sha256=bag_sha)])
    path=root/'synthetic_proof.json';path.write_text(json.dumps(value))
    return path,value,scene,sensor_sha,adapter,objects

class LimitedAdmission(unittest.TestCase):
    def test_limited_HOLD_pair_admits_evidence_without_relabeling_result(self):
        with tempfile.TemporaryDirectory() as d:
            path,value,scene,sensor,adapter,objects=proof_fixture(Path(d))
            admitted=proof.validate_proof(dict(path=str(path),sha256=proof.sha(path)),scene,sensor,adapter)
            self.assertFalse(admitted['calibration_diagnostic_passed']);self.assertTrue(admitted['actual_semantic_two_sign_admitted'])
            self.assertEqual(objects['result']['state'],'HOLD_DIAGNOSTIC_FAILED')
            bounded=model();bounded['lateral_calibration_proof_sha256']=admitted['proof_sha256']
            for value in bounded['lateral_models'].values():value['yaw_reserve_rad']=.025
            proof.validate_model_evidence(bounded,admitted)
            bad=copy.deepcopy(bounded);bad['lateral_models']['positive']['drift_reserve_m']=.001
            with self.assertRaises(proof.AdmissionError):proof.validate_model_evidence(bad,admitted)
    def test_missing_root_limited_approval_or_wrong_complete_label_rejected(self):
        for change in [dict(root_limited_probe_approved=False),dict(admission_kind='complete_sustained_pair'),dict(capability='semantic_lateral_axes_v1')]:
            with tempfile.TemporaryDirectory() as d:
                path,value,scene,sensor,adapter,objects=proof_fixture(Path(d));value.update(change);path.write_text(json.dumps(value))
                with self.assertRaises(proof.AdmissionError):proof.validate_proof(dict(path=str(path),sha256=proof.sha(path)),scene,sensor,adapter)
    def test_changed_source_or_case_or_provenance_bytes_rejected(self):
        for key in ['case','source_calibration_observer.py','coverage_correction','probe1_backend']:
            with tempfile.TemporaryDirectory() as d:
                path,value,scene,sensor,adapter,objects=proof_fixture(Path(d));Path(value['artifacts'][key]['path']).write_bytes(b'changed')
                with self.assertRaises(proof.AdmissionError):proof.validate_proof(dict(path=str(path),sha256=proof.sha(path)),scene,sensor,adapter)
    def test_actual_measurements_remain_unapproved_with_correct_coverage(self):
        audit=proof.read(BASE/'sim_semantic_lateral_case3_motion_audit_v1/model_measurements.json')
        coverage=proof.read(BASE/'sim_semantic_lateral_case3_motion_audit_v1/coverage_correction_v2.json')
        self.assertEqual(audit['measured_raw_signs'],[.4,-.4]);self.assertFalse(audit['root_limited_probe_approved'])
        self.assertIsNone(audit['root_engineering_model']);self.assertFalse(audit['diagnostic_passed'])
        self.assertTrue(coverage['all_motion_stop_and_measurement_headers_common'])

class CorrectionFlow(unittest.TestCase):
    def run_branch(self,cross=.01,yaw=.041,turn_blocked=False,correction_used=0.,turn_seconds=0.):
        tree=ast.parse((HERE/'full_scene_mapping.py').read_text())
        branch=copy.deepcopy(next(n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test)=="decision['stop_then_recheck']"))
        p=policy();events=[];sample=dict(xyz=[-cross,.2,.4]);decision=c.lateral_correction_state(dict(cross_track_m=cross),(0,.2,yaw),p,0,model());clk=Clock()
        def stable(seconds,timeout):events.append('measured_zero_stop');return sample,yaw
        def rotate(target,tolerance,deadline):
            self.assertEqual(events,['measured_zero_stop']);events.append('SAT_guarded_target_turn')
            if turn_blocked:raise c.Hold('fixture filled-SAT blocked')
            clk.sleep(turn_seconds)
            return sample,target
        def emit(*args,**kwargs):events.append('same_leg_resume');decision['stop_then_recheck']=False
        ns=dict(contract=c,math=math,decision=decision,truth=sample,yaw=yaw,stable=stable,rotate_to=rotate,
            leg_start=(0,0,0),x=0.,y=1.,heading=0.,policy=p,corrections=0,correction_used=correction_used,model=model(),time=clk,emit=emit,index=0)
        loop=ast.While(test=ast.Constant(True),body=[branch,ast.Break()],orelse=[])
        try:exec(compile(ast.fix_missing_locations(ast.Module(body=[loop],type_ignores=[])),'<actual correction branch>','exec'),ns)
        except c.Hold as exc:return events,str(exc)
        return events,None
    def test_measured_stop_then_target_turn_then_same_leg_resume(self):
        events,error=self.run_branch();self.assertIsNone(error)
        self.assertEqual(events,['measured_zero_stop','SAT_guarded_target_turn','same_leg_resume'])
    def test_soft_cross_is_not_immediately_rejected_but_SAT_block_never_resumes(self):
        events,error=self.run_branch(cross=.08,yaw=.041)
        self.assertIsNone(error);self.assertEqual(events[-1],'same_leg_resume')
        events,error=self.run_branch(turn_blocked=True)
        self.assertEqual(events,['measured_zero_stop','SAT_guarded_target_turn']);self.assertIn('SAT blocked',error)
    def test_correction_work_budget_is_cumulative_and_never_resumes_after_exhaustion(self):
        events,error=self.run_branch(correction_used=20.)
        self.assertFalse(events);self.assertIn('budget exhausted',error)
        events,error=self.run_branch(correction_used=19.9,turn_seconds=.2)
        self.assertEqual(events,['measured_zero_stop','SAT_guarded_target_turn']);self.assertIn('exceeded20s',error)

class DefaultHold(unittest.TestCase):
    def test_real_model_and_default_owner_do_not_authorize_or_start(self):
        with self.assertRaises(c.Hold):c.validate_model(c.read_json(HERE/'motion_model.ROOT_HOLD.json'))
        with tempfile.TemporaryDirectory() as out,mock.patch.object(owner.sys,'argv',['owner','--output',str(Path(out)/'hold')]),\
            mock.patch.object(owner.subprocess,'Popen',side_effect=AssertionError('child forbidden')),\
            mock.patch('socket.socket',side_effect=AssertionError('socket forbidden')),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(owner.main(),0)
            result=json.loads((Path(out)/'hold/result.json').read_text())
            self.assertEqual(result['classification'],'HOLD_no_execution_or_incomplete')

if __name__=='__main__':unittest.main(verbosity=2)
