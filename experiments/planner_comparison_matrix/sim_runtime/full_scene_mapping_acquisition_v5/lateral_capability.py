"""Read-only admission of actual semantic two-sign measurements; no transport."""
import hashlib
import json
import math
from pathlib import Path

CAPABILITY='semantic_lateral_mag04_rect_v5'
ACTUAL_GT_STOP_REFERENCE='original_GT_actual_pose_future_errors_v1'
SCOPE='MAPPING_ACQUISITION_LATERAL_RECT_V5_ONLY'
CAL_SCOPE='MAPPING_SEMANTIC_LATERAL_BOUNDED_AMPLITUDE_SUSTAINED_CALIBRATION_ONLY'
CAL_SOURCES={'lateral_once.py','lateral_contract.py','semantic_tool_timed.py','host_preflight.py',
             'calibration_observer.py','imu_gate.py','recording_deadline.py','bag_summary.py'}
# Exact independently reviewed combined canary. A changed diagnostic needs review.
CAL_SOURCE_SHA={'lateral_once.py': 'd630eb255c96c922d3fdcd91e23532d2253fdb03c08737ac1a800fadbdb98a5f', 'lateral_contract.py': '910ce9e739854a5f073fdef8564eff198ce406cd620d26dd3df3b4cdde6ec7d3', 'semantic_tool_timed.py': 'c7c0281553bb1a457f99b344cd0d06a95b714a732dca3485956d223d6dfd051a', 'host_preflight.py': '749da4569db74798f69a47b8cbe31af1d656f99013dea23be8911cdd1ac3dfb7', 'calibration_observer.py': '2d8b36b459b74b85fab36557515bb4bad7528bcda7434735fba0e919132a2515', 'imu_gate.py': '0b451821b94eb8182ff2f8df8d5e7faf359a24486455c90b629478f944023f66', 'recording_deadline.py': 'e850bc78bfe46d0678cfd1984995c688bf4f7ad76c1b806ccfb36c4bc944b80c', 'bag_summary.py': '1293667a9720e1538d67cc1966a8cbb0753854f94129997e11a78bb680f5e792'}
RAW_TOPICS={'/odom','/front_lidar','/front_lidar/imu','/front_camera/image/compressed',
            '/camera/front/image/compressed','/camera/front/camera_info'}

class AdmissionError(ValueError):pass

def require(value,reason):
    if not value:raise AdmissionError(reason)

def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()

def read(path):
    def unique(items):
        value={}
        for k,v in items:
            require(k not in value,'duplicate proof JSON key');value[k]=v
        return value
    return json.loads(Path(path).read_text(),object_pairs_hook=unique,
        parse_constant=lambda x:(_ for _ in ()).throw(AdmissionError('nonfinite proof JSON')))

def finite(values):return all(type(v) in (int,float) and math.isfinite(v) for v in values)

def semantic_trace(path,sign,duration_sec=4.):
    require(sign in (-.4,.4) and finite([duration_sec]) and 1.5<=duration_sec<=4.,
            'only actual measured magnitude0.4 case in1.5..4s')
    records=[json.loads(line) for line in Path(path).read_text().splitlines()]
    require(bool(records),'empty actual semantic send trace')
    moving=[];previous=None
    for item in records:
        require(item.get('event')=='actual_socket_sendto_interval_not_actuation' and item.get('send_success') is True,
                'failed/short/nonactual semantic send trace')
        command=item.get('command',{})
        require(set(command)=={'version','action','forward','lateral','yaw'} and type(command['version']) is int
            and command['version']==1 and command['action']=='walk' and finite([command['forward'],command['lateral'],command['yaw']])
            and command['forward']==command['yaw']==0. and command['lateral'] in (0.,sign),
            'only actual semantic single-axis measured sign may establish capability; MC/reverse/mixed rejected')
        require(item.get('target')==['127.0.0.1',7447],'semantic target differs')
        before,after=item['send_before_steady_ns'],item['send_after_steady_ns']
        require(type(before) is int and type(after) is int and before<=after and (previous is None or before>=previous),'send syscall order invalid')
        require(all(type(item.get(k)) is int and item[k]>=0 for k in ['send_before_unix_ns','send_after_unix_ns'])
            and item['send_before_unix_ns']<=item['send_after_unix_ns'],'send wall-clock interval invalid')
        previous=after
        payload=json.dumps(command,separators=(',',':'),allow_nan=False).encode()
        require(item.get('bytes')==len(payload) and hashlib.sha256(payload).hexdigest()==item['payload_sha256'],
                'trace command differs from actual payload digest')
        if command['lateral']:moving.append(item)
    require(len(moving)>=2 and 1.<=(moving[-1]['send_after_steady_ns']-moving[0]['send_before_steady_ns'])/1e9<=duration_sec+.05,'not a bounded measured lateral probe with a full active1s')
    require(any(not r['command']['lateral'] and r['send_before_steady_ns']>moving[-1]['send_after_steady_ns'] for r in records),'no actual semantic neutral after probe')
    return moving


def validate_proof(reference,scene,sensor_sha,adapter_identity):
    require(isinstance(adapter_identity,dict) and set(adapter_identity)=={'pid','starttime_ticks','cmdline_sha256','argv','boot_id'}
        and type(adapter_identity['pid']) is int and adapter_identity['pid']>0 and type(adapter_identity['starttime_ticks']) is int
        and adapter_identity['starttime_ticks']>0 and bool(adapter_identity['argv']) and len(adapter_identity['cmdline_sha256'])==64
        and bool(adapter_identity['boot_id']),'complete actual replacement adapter identity required')
    path=Path(reference.get('path',''))
    require(path.is_absolute() and sha(path)==reference.get('sha256'),'actual semantic calibration proof missing/changed')
    proof=read(path);kind=proof.get('admission_kind')
    require(proof.get('capability')==CAPABILITY and proof.get('scope')==CAL_SCOPE
        and proof.get('basis')=='actual_semantic_closed_bag_measurement'
        and proof.get('root_measurement_reviewed') is True and proof.get('hard_plant_guarantee') is False,
        'root-reviewed actual two-sign evidence required; no hard plant guarantee')
    require(kind in ('complete_sustained_pair','root_reviewed_auxiliary_camera_interrupted_pair'),
        'explicit complete or limited auxiliary-camera admission kind required')
    if kind=='root_reviewed_auxiliary_camera_interrupted_pair':
        require(proof.get('root_limited_probe_approved') is True,'limited actual pair remains root HOLD')
    require(proof.get('scene_instance_id')==scene and proof.get('sensor_config_sha256')==sensor_sha
        and proof.get('adapter_process_identity')==adapter_identity,'calibration scene/config/actual new adapter differs')
    artifacts=proof.get('artifacts',{})
    required={'case','result','probe0_trace','probe1_trace','probe0_backend','probe1_backend',
        'probe0_acceptance','probe1_acceptance','probe1_plan','bag_metadata','model_measurements',
        'source_manifest','sensor_config','adapter_identity','measurement_manifest','coverage_correction'}
    required|={'source_'+name for name in CAL_SOURCES}
    require(required<=set(artifacts),'complete actual calibration provenance missing')
    files={}
    for name,item in artifacts.items():
        value=Path(item['path']);value=value if value.is_absolute() else path.parent/value
        require(value.is_file() and sha(value)==item['sha256'],'calibration artifact changed: '+name);files[name]=value
    case=read(files['case'])
    identity_record=read(files['adapter_identity'])
    if 'sources' in identity_record:
        require(identity_record.get('state')=='PASS' and isinstance(case.get('source_identities'),dict)
            and set(identity_record['sources'])=={'new_adapter','bridge24','projector61'}
            and identity_record['sources']==case['source_identities'],
            'original actual precheck PASS/full source identities must equal frozen case')
        identity_record=identity_record['sources']['new_adapter']
    else:
        identity_record=identity_record.get('new_source_container_identity',identity_record)
    identity_five={key:identity_record.get(key) for key in ['pid','starttime_ticks','cmdline_sha256','argv','boot_id']}
    require(artifacts['sensor_config']['sha256']==sensor_sha and identity_five==adapter_identity,
        'actual sensor/container-adapter evidence differs')
    duration=case.get('duration_sec')
    require(case.get('scope')==CAL_SCOPE and case.get('magnitude')==.4 and case.get('probes')==[.4,-.4]
        and finite([duration]) and 1.5<=duration<=4. and case.get('max_runtime_sec')==60.,'only exact actual0.4 paired case admitted')
    require(case.get('scene_instance_id')==scene and case.get('sensor_config_artifact',{}).get('sha256')==sensor_sha,
        'actual case scene/config differs from mapping proof')
    result=read(files['result']);sources=result.get('source_package_sha256',{})
    require(result.get('scope')==CAL_SCOPE and result.get('recorder_exited') is True
        and result.get('source_signals_sent')==result.get('host_signals_sent')==0 and result.get('raw_cdr_reencoded') is False,
        'closed raw actual case with no source/host signal required')
    if kind=='complete_sustained_pair':
        require(result.get('diagnostic_passed') is True and result.get('state')=='COMPLETE_PROBES_STOPPED_PLUS5S_RAW_TAIL',
            'complete admission must retain the actual complete case result')
    else:
        require(result.get('diagnostic_passed') is False and result.get('state')=='HOLD_DIAGNOSTIC_FAILED'
            and result.get('failure_stop_and_tail_verified') is True,'limited admission must preserve original HOLD and measured stop+tail')
    require(sources==CAL_SOURCE_SHA and proof.get('source_package_sha256')==sources,
        'exact8 independently reviewed bounded calibration source SHAs required')
    require(all(artifacts['source_'+name]['sha256']==digest for name,digest in CAL_SOURCE_SHA.items()),
        'actual closed calibration runtime source bytes differ from reviewed8')
    require(read(files['source_manifest']).get('runtime_package_sha256')==sources,'actual source manifest must bind8 sources')
    require(all(result.get('observed_topic_counts',{}).get(t,0)>0 for t in RAW_TOPICS),'raw/GT/IMU/camera observation evidence missing')
    bag=result.get('bag_audit',{});require(bag.get('complete') is True and bool(bag.get('files')),'complete actual closed MCAP required')
    mcap_refs=proof.get('mcap_files',[])
    require({v['sha256'] for v in mcap_refs}=={v['sha256'] for v in bag['files']},'actual complete bag SHA binding differs')
    for item in mcap_refs:
        value=Path(item['path']);value=value if value.is_absolute() else path.parent/value
        require(value.is_file() and sha(value)==item['sha256'],'actual MCAP bytes missing/changed')
    require(bag.get('metadata_sha256')==artifacts['bag_metadata']['sha256'],'bag metadata binding differs')
    measurements=read(files['model_measurements'])
    require(measurements.get('schema')=='locnav.root_lateral_model_measurement_input.v1'
        and measurements.get('original_case_state')==result['state']
        and measurements.get('diagnostic_passed')==result['diagnostic_passed']
        and measurements.get('source_package_sha256')==sources
        and measurements.get('measured_raw_signs')==[.4,-.4]
        and measurements.get('bag',{}).get('sha256') in {v['sha256'] for v in mcap_refs},
        'independent actual raw-GT measurement schema/result/source/bag binding differs')
    coverage=read(files['coverage_correction']);measurement_manifest=read(files['measurement_manifest'])
    require(coverage.get('all_motion_stop_and_measurement_headers_common') is True
        and coverage.get('original_movement_stop_future_values_changed') is False
        and type(coverage.get('motion_range_headers')) is int and coverage['motion_range_headers']>0
        and coverage.get('common_exact_position_quaternion_linear_angular',0)>=coverage['motion_range_headers'],
        'explicit motion/stop original-GT coverage correction required; startup intersection is not full coverage')
    manifest_files=measurement_manifest.get('files',{})
    require(measurement_manifest.get('runtime_approved') is False
        and manifest_files.get('model_measurements.json',{}).get('sha256')==artifacts['model_measurements']['sha256']
        and manifest_files.get('coverage_correction_v2.json',{}).get('sha256')==artifacts['coverage_correction']['sha256'],
        'measurement/corrected coverage frozen manifest mismatch')
    input_hashes=set(measurements.get('actual_case_artifact_sha256',{}).values())
    require(all(artifacts[key]['sha256'] in input_hashes for key in ['case','result','probe0_trace','probe1_trace',
        'probe0_backend','probe1_backend','probe0_acceptance','probe1_acceptance','probe1_plan','sensor_config','adapter_identity']),
        'measurement audit does not bind the actual case/result/send/backend/acceptance/config/identity inputs')
    # Measurement artifacts cannot grant runtime permission or choose I/D.
    require(measurements.get('root_engineering_model') is None and measurements.get('runtime_approved') is False,
        'measurement-only input must preserve runtime/model HOLD')
    measured=measurements.get('probes',[]);probes=result.get('probes',[])
    require(len(measured)==len(probes)==2,'both actual semantic signs required')
    normalized=[]
    allowed_camera_errors={'stale/future/missing/empty original sensor: '+topic for topic in
        ['/front_camera/image/compressed','/camera/front/image/compressed','/camera/front/camera_info']}
    for index,sign in [(0,.4),(1,-.4)]:
        accepted=read(files[f'probe{index}_acceptance']);measurement=next((v for v in probes if v.get('index')==index),None)
        raw=next((v for v in measured if v.get('index')==index),None);backend=read(files[f'probe{index}_backend'])
        require(accepted==measurement and isinstance(raw,dict) and measurement.get('state')=='MEASURED_SUSTAINED_LATERAL_PROBE'
            and measurement.get('policy_lateral')==sign and raw.get('raw_lateral')==sign,'actual sign result/audit/acceptance mismatch')
        s=measurement.get('sustained',{})
        require(s.get('final_active1s_complete') is True and s.get('strict_stopped_at_cutoff') is True
            and finite([s.get('signed_net_stopped_body_y_m'),s.get('signed_final_active1s_body_vy_median_mps')])
            and s['signed_net_stopped_body_y_m']>.05 and s['signed_final_active1s_body_vy_median_mps']>.03
            and raw.get('strict_stop_confirmed') is True and finite([raw.get('retained_signed_net_m')])
            and abs(raw['retained_signed_net_m']-s['signed_net_stopped_body_y_m'])<1e-5,
            'both original-GT sustained directions and actual strict stop required')
        moving=semantic_trace(files[f'probe{index}_trace'],sign,duration)
        trace=[json.loads(line) for line in files[f'probe{index}_trace'].read_text().splitlines()]
        zeros=[v for v in trace if not v['command']['lateral']]
        require(len(zeros)==5 and all(v['send_before_steady_ns']>moving[-1]['send_after_steady_ns'] for v in zeros),
            'five successful actual neutral packets must follow every actual sign')
        require(backend.get('sign')==sign and backend.get('requested_active_sec')==duration
            and backend.get('motion_packets_sent')==len(moving)==raw.get('nonzero_packets')
            and 2<=len(moving)<=80 and backend.get('final_zero_attempts')==backend.get('final_zero_logged_successes')==raw.get('neutral_packets')==5
            and backend.get('close_returned') is True,'actual backend packet count/neutral/close evidence differs')
        if kind=='complete_sustained_pair' or index==0:
            require(backend.get('completed') is True and backend.get('errors')==[],'actual completed positive/backend required')
        else:
            errors=backend.get('errors',[])
            require(backend.get('completed') is False and bool(errors) and all(v.get('stage')=='active' and v.get('message') in allowed_camera_errors for v in errors),
                'limited negative may only be interrupted by the explicit auxiliary-camera freshness guard')
            require(raw.get('backend_errors')==errors and raw.get('backend_completed') is False,'independent interruption reason must match raw backend')
        future=raw.get('future_excursions_by_phase',{}).get('whole_postneutral',{})
        values=[raw.get('max_original_speed3d_mps'),raw.get('max_original_header_yaw_derivative_rps'),
            raw.get('neutral_to_stop_start_path_xy_m'),future.get('max_signed_along_m'),future.get('max_abs_cross_m'),future.get('max_abs_yaw_rad')]
        require(finite(values) and values[0]>0 and min(values[1:])>=0,'finite actual max/future-error measurements required')
        normalized.append(dict(sign=1 if sign>0 else -1,max_norm_speed_mps=values[0],max_yaw_rate_rps=values[1],
            stop_distance_m=max(values[2],values[3]),cross_drift_m=values[4],yaw_drift_rad=values[5]))
    require(read(files['probe1_plan']).get('prior_positive_acceptance_sha256')==artifacts['probe0_acceptance']['sha256'],
        'negative was not bound to admitted positive')
    return dict(proof_sha256=reference['sha256'],audit=dict(probes=normalized),source_package_sha256=sources,
        actual_semantic_two_sign_admitted=True,calibration_diagnostic_passed=result['diagnostic_passed'],admission_kind=kind,
        boundary='Finite actual magnitude0.4 both-sign evidence plus root normal static-SIM engineering model; original overall result retained,not whole-route or hard plant acceptance')


def validate_model_evidence(model,admission):
    require(model.get('future_error_reference')==ACTUAL_GT_STOP_REFERENCE and model.get('independent_future_errors_reviewed') is True,
            'root independent future-error reserve semantics remain HOLD')
    require(model.get('lateral_capability')==CAPABILITY and model.get('lateral_model_reviewed') is True,
            'root directional engineering model is still HOLD')
    require(model.get('lateral_calibration_proof_sha256')==admission['proof_sha256'],
            'directional engineering model is not linked to actual semantic measurements')
    for actual in admission['audit']['probes']:
        name='positive' if actual['sign']>0 else 'negative';bound=model['lateral_models'][name]
        for engineering,measurement in [('max_truth_speed_mps','max_norm_speed_mps'),
              ('max_yaw_rate_rps','max_yaw_rate_rps'),('stop_distance_reserve_m','stop_distance_m'),('drift_reserve_m','cross_drift_m'),('yaw_reserve_rad','yaw_drift_rad')]:
            require(bound[engineering]>=actual[measurement],'root '+name+' engineering bound below actual '+measurement)
        require(model['max_truth_speed_mps']>=actual['max_norm_speed_mps']
            and model['max_truth_yaw_rate_rps']>=actual['max_yaw_rate_rps'],'global future-speed/yaw engineering bound below actual measurement')
    return admission


FINE_FORWARD_SUBCAPABILITY='sim_acquisition_forward_terminal_fine_v1'

def fine_profile(value):
    require(isinstance(value,dict) and set(value)=={'subcapability','selected_fine_forward_mps'}
        and value.get('subcapability')==FINE_FORWARD_SUBCAPABILITY
        and finite([value.get('selected_fine_forward_mps')]) and value['selected_fine_forward_mps'] in (.2,.3),
        'exact root-selected fine-forward subcapability/value required')
    return dict(value)

def fine_profile_from_release(release):
    allow=release.get('allow_fine_forward_profile',False)
    if allow is False:
        require(release.get('selected_fine_forward_mps') is None and release.get('fine_forward_subcapability') is None,
            'disabled fine profile must not select a magnitude/subcapability')
        return None
    require(allow is True,'fine profile authorization must be explicit true')
    return fine_profile(dict(subcapability=release.get('fine_forward_subcapability'),selected_fine_forward_mps=release.get('selected_fine_forward_mps')))

def fine_admission_profile(admission,release):
    value=fine_profile_from_release(release)
    require(admission.get('fine_forward_profile')==value,'relay fine-forward admission differs from exact root release')
    return value
