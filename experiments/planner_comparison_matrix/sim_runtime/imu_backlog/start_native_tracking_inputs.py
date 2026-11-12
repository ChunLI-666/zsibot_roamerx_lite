#!/usr/bin/env python3
"""Default HOLD; restart Native LM and SDK inputs from a pinned historical Native seed. Sensor source ownership remains separate."""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml

WS=Path('/home/charles/project/colcon_ws')
BASE=WS/'analysis_outputs/planner_comparison_20261105'
RUNTIME=WS/'analysis_outputs/project_smoke_20261030/install'
LM=RUNTIME/'libexec/lightning/run_loc_online'
LIBRARY=RUNTIME/'lib/liblightning.libs.so'
TEMPLATE=BASE/'samehost_runtime/pipeline_tbb4_v1/localization.yaml'
NATIVE=BASE/'native_private_endpoint_committed_root_v1/B'
SNAPSHOT=NATIVE/'source_snapshot.json'
ADAPTER_REL='experiments/planner_comparison_matrix/sim_runtime/sim_command_adapter.py'
RELAY_REL='experiments/planner_comparison_matrix/sim_runtime/sim_udp_relay.py'
LM_SHA='706ffd34d98c83a12790587885ffbc056ce94295c38058f470c791185de839a6'
LIB_SHA='7911cf3d1492a8ffd0e4ab3ec070b78d77d9a14683a4c6d7117f04abd62c6305'
SNAPSHOT_SHA='572dd7518d7979e853b452b350937be640e6d93e0123aee894a9d513eb2769c8'


def require(ok,message):
    if not ok:raise ValueError(message)


def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def read(path):return json.loads(Path(path).read_text())


def write(path,value):
    with Path(path).open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def closed_inputs(case):
    recording=read(case/'recording_result.json')
    controller_path=case/'controller/result.json'
    if not controller_path.is_file():controller_path=case/'controller_result.frozen.json'
    controller=read(controller_path)
    require(recording.get('recorder_exited') is True,'recording is not closed')
    require(controller.get('stop_verified') is True and controller.get('relay_closed') is True,'mapping stop/relay closure not verified')
    return dict(case=str(case),recording_result_sha256=sha(case/'recording_result.json'),
        controller_result=str(controller_path),controller_result_sha256=sha(controller_path),
        recorder_exited=True,stop_verified=True,relay_closed=True,
        recording_classification=recording.get('classification'),controller_classification=controller.get('classification'))


def native_seed(map_dir,provenance):
    evidence=read(provenance);index=map_dir/'index.txt';frame=map_dir/'map_frame.yaml'
    if evidence.get('schema')=='Native.last_known_full_T_map_imu.initialization_candidate.v1':
        return tracking_seed(map_dir,provenance,evidence)
    require(evidence.get('pose_semantics')=='T_map_imu; xyz + qxyzw; exact SLAM index values','seed must be native T_map_imu')
    require(evidence.get('index_sha256')==sha(index) and evidence.get('map_frame_sha256')==sha(frame),'new-map seed hashes differ')
    seed=evidence['seed'];pose=seed['pose_xyz_qxyzw']
    require(isinstance(pose,list) and len(pose)==7 and all(type(v) in (int,float) and math.isfinite(v) for v in pose),'invalid native FP pose')
    require(abs(sum(v*v for v in pose[3:])-1.)<1e-3,'invalid FP quaternion')
    lines=index.read_text().splitlines();marker=lines.index('# functional points')
    points=[(i+1,line) for i,line in enumerate(lines) if i>marker and line.strip()]
    require(bool(points),'native map has no functional point')
    number,line=points[-1];fields=line.split()
    require(number==seed['index_line_number'] and line==seed['original_line'] and len(fields)==8 and fields[0]==seed['name'] and list(map(float,fields[1:]))==pose,'provenance is not the exact last native FP')
    qx,qy,qz,qw=pose[3:];yaw=math.atan2(2.*(qw*qz+qx*qy),1.-2.*(qy*qy+qz*qz))
    return dict(provenance=str(provenance),provenance_sha256=sha(provenance),pose_xyz_qxyzw=pose,
        semantics='T_M_I from native SLAM; SE2 initialization x/y/yaw only, not accepted localization',
        init_x=pose[0],init_y=pose[1],init_yaw=yaw,index_sha256=sha(index),map_frame_sha256=sha(frame))


def tracking_seed(map_dir,provenance,evidence):
    # Initialization hypothesis only: never restamp or publish this old measurement.
    require(map_dir.resolve()==(BASE/'mapping_expand_20261106/offline_runs/warehouse_full_source25_map_v1/data/new_map').resolve(),'tracking seed requires frozen full native map')
    require({Path(x['path']).name for x in evidence['map_identity']}=={'map.yaml','map_frame.yaml','map.pgm'},'missing map identities')
    for entry in evidence['map_identity']:
        path=map_dir/Path(entry['path']).name
        require(Path(entry['path']).resolve()==path.resolve() and sha(path)==entry['sha256'],'seed map differs')
    require(yaml.safe_load((map_dir/'map_frame.yaml').read_text()).get('pose_frame')=='imu','seed is IMU, not BODY')
    raw=Path(evidence['original_CDR_path']);require(raw.stat().st_size==evidence['original_CDR_bytes'] and sha(raw)==evidence['original_CDR_sha256'],'Native debug CDR differs')
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    msg=deserialize_message(raw.read_bytes(),get_message('lightning/msg/LightningDebug'))
    require(msg.schema_version==2 and msg.global_parent_frame_id=='map' and msg.loc_status==1,'Native debug frame/state differs')
    for key in ['dr_valid','lio_valid','lio_update_valid','lidar_loc_valid','pgo_fused_valid','extrapolated_valid','smoothed_valid']:
        require(getattr(msg,key) is True,'historical Native full pose invalid: '+key)
    p=msg.smoothed_pose;values=[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
    require(values==evidence['position_xyz']+evidence['quaternion_xyzw'] and all(math.isfinite(v) for v in values),'Native seed values do not match original CDR')
    require(abs(sum(v*v for v in values[3:])-1.)<1e-3,'invalid Native seed quaternion')
    require(msg.smoothed_stamp==evidence['smoothed_ownstamp_seconds_original_double'] and msg.header.stamp.sec*10**9+msg.header.stamp.nanosec==evidence['header_stamp_ns'],'Native seed stamps differ')
    for source in evidence['source_bag_files']:
        bag=Path(source['path']);case=bag.parent.parent
        require(bag.stat().st_size==source['bytes'] and read(case/'final_audit.json')['mcap_sha256'][bag.name]==source['sha256'],'closed seed bag metadata differs')
        events=[json.loads(line) for line in (case/'events.jsonl').read_text().splitlines()]
        closed=[x for x in events if x.get('kind')=='owned_processes_closed']
        require(len(closed)==1 and len(closed[0]['returncodes'])==9 and all(x==0 for x in closed[0]['returncodes'].values()),'seed bag actors not closed')
    qx,qy,qz,qw=values[3:];yaw=math.atan2(2.*(qw*qz+qx*qy),1.-2.*(qy*qy+qz*qz))
    return dict(provenance=str(provenance),provenance_sha256=sha(provenance),pose_xyz_qxyzw=values,
        semantics='Historical original Native T_map_imu initialization hypothesis; only x/y/yaw applied, no GT or retiming. Fresh localization and stopped preflight required after startup.',
        original_measurement_stamp_seconds=msg.smoothed_stamp,initialization_age_seconds=time.time()-msg.smoothed_stamp,
        init_x=values[0],init_y=values[1],init_yaw=yaw,index_sha256=sha(map_dir/'index.txt'),map_frame_sha256=sha(map_dir/'map_frame.yaml'))


def source_binding(path):
    release=read(path);sources=release['allowed_udp_source_processes']
    sensor=release['sensor_adapter_process_identity']
    require(type(sensor['pid']) is int and sensor['pid']>0 and sensor['pid'] not in {47,97},'invalid sensor actor PID')
    require({entry['pid'] for entry in sources}=={47,97,sensor['pid']} and len(sources)==3,'expected retained bridge/projector plus explicit sensor actor')
    for entry in sources:
        require(set(entry)=={'pid','starttime_ticks','cmdline_sha256','argv','boot_id'} and
            type(entry['starttime_ticks']) is int and entry['starttime_ticks']>0 and
            isinstance(entry['argv'],list) and bool(entry['argv']) and
            len(entry['cmdline_sha256'])==64 and entry['boot_id']==release['boot_id'],'invalid static source identity')
    require(release.get('sensor_adapter_process_identity') in sources,'source adapter not bound')
    require(release['sim_command_endpoint']['ipv4']=='192.168.1.219' and release['sim_command_endpoint']['port']==7447,'wrong private SIM endpoint')
    return dict(path=str(path),sha256=sha(path),boot_id=release['boot_id'],network_namespace=release['network_namespace'],
        controller_pid_namespace=release['controller_pid_namespace'],sources=sources,
        scope='Static source-release identity only. Root attests current state before execute; no sensor/bridge/projector/UE is started or signalled.')


def artifacts():
    require(sha(LM)==LM_SHA and sha(LIBRARY)==LIB_SHA,'frozen LM executable/DSO differs')
    require(sha(SNAPSHOT)==SNAPSHOT_SHA,'committed Native endpoint snapshot differs')
    snapshot=read(SNAPSHOT)
    result={str(LM):LM_SHA,str(LIBRARY):LIB_SHA,str(SNAPSHOT):SNAPSHOT_SHA}
    for relative in [ADAPTER_REL,RELAY_REL]:
        path=NATIVE/relative;require(sha(path)==snapshot['files_sha256'][relative],'committed Native endpoint source differs')
        result[str(path)]=sha(path)
    return result


def lio_wake_diagnostic_profile(data,mode='template'):
    require(mode in ('template','false'),'unsupported LIO wake diagnostic mode')
    previous=data.get('performance',{}).get('lio_imu_ready_wake')
    if mode=='false':
        require(type(previous)is bool,'diagnostic requires an explicit boolean template wake option')
        data['performance']['lio_imu_ready_wake']=False
    return dict(mode=mode,template_value=previous,
        effective_value=data.get('performance',{}).get('lio_imu_ready_wake'),
        diagnostic_only=mode!='template',
        boundary='Existing compiled configuration only. Disabling ready wake changes LIO scheduling; consumption, tail, frame freshness and accuracy must be evaluated separately. No source stamp or freshness predicate is changed.')


def prepare(args):
    case=args.closed_case_dir.resolve();map_dir=args.map_dir.resolve()
    provenance=(args.seed_provenance or map_dir.parent.parent/'new_map_seed_provenance.json').resolve()
    closure=closed_inputs(case);seed=native_seed(map_dir,provenance);binding=source_binding(args.source_release.resolve())
    data=yaml.safe_load(TEMPLATE.read_text());original=copy.deepcopy(data)
    data['system'].update(map_path=str(map_dir),use_fp_init=False,init_x=seed['init_x'],init_y=seed['init_y'],init_yaw=seed['init_yaw'])
    data['lidar_loc']['init_with_fp']=False
    wake_profile=lio_wake_diagnostic_profile(data,getattr(args,'lio_imu_ready_wake','template'))
    config=args.output/'localization.yaml'
    with config.open('x') as stream:yaml.safe_dump(data,stream,sort_keys=False)
    # Preserve every other template value, including TBB4, geometry and input timing.
    for key in ['map_path','use_fp_init','init_x','init_y','init_yaw']:original['system'][key]=data['system'][key]
    original['lidar_loc']['init_with_fp']=False
    if wake_profile['diagnostic_only']:original['performance']['lio_imu_ready_wake']=False
    require(original==data,'unexpected template semantic change')
    return dict(state='HOLD_PREPARED_NO_PROCESS',closure=closure,seed=seed,source_binding=binding,
        lio_wake_diagnostic_profile=wake_profile,
        template=dict(path=str(TEMPLATE),sha256=sha(TEMPLATE)),config=dict(path=str(config),sha256=sha(config)),artifacts=artifacts(),
        argv=dict(localization=[str(LM),'--config',str(config)],adapter=['/usr/bin/python3','-u',str(NATIVE/ADAPTER_REL),'--relay-mode','local','--sim-host','192.168.1.219','--max-policy-vx','0.4','--max-policy-wz','0.3']),
        boundary='Inputs only; no Nav/goal/source3/UE launch or signal. Seed does not imply accepted Native pose.')


def environment(role,output,parent=None):
    env=dict(os.environ if parent is None else parent)
    env.update(PYTHONDONTWRITEBYTECODE='1',ROS_LOG_DIR=str(output/'ros_logs'))
    if role=='localization':
        env.update(OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1',LD_LIBRARY_PATH=':'.join([
            '/opt/frozen_runtime/lib',str(RUNTIME/'lib'),str(WS/'src/lightning-lm/thirdparty/Pangolin-0.9.3/build'),
            '/opt/ros/jazzy/lib','/opt/ros/jazzy/lib/x86_64-linux-gnu']))
    return env


def execution_context(binding):
    require(os.environ.get('ROS_DOMAIN_ID')=='185','execute requires inherited private domain185')
    require(Path('/proc/sys/kernel/random/boot_id').read_text().strip()==binding['boot_id'],'private seed boot differs')
    require(os.readlink('/proc/self/ns/net')==binding['network_namespace'] and os.readlink('/proc/self/ns/pid')==binding['controller_pid_namespace'],'actor private PID/net namespace differs from source-release')
    require(hasattr(os,'pidfd_open') and hasattr(signal,'pidfd_send_signal'),'pidfd support required')


def process_identity(pid):
    root=Path('/proc')/str(pid);stat=(root/'stat').read_text();argv=(root/'cmdline').read_bytes()
    return dict(pid=pid,starttime_ticks=int(stat[stat.rfind(')')+2:].split()[19]),
        argv=[s.decode() for s in argv.rstrip(b'\0').split(b'\0')],cmdline_sha256=hashlib.sha256(argv).hexdigest(),
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),executable_sha256=sha(root/'exe'))


def cleanup(children):
    results=[]
    # Reverse launch order: adapter handles its owned relay's zero/close, then LM.
    for child in reversed(children):
        proc=child['process'];record=child['record'];errors=[];sent=[]
        try:
            for number,timeout in [(signal.SIGINT,10.),(signal.SIGTERM,3.),(signal.SIGKILL,3.)]:
                if proc.poll() is not None:break
                if child['pidfd'] is None:raise RuntimeError('no retained child pidfd; no PID/name-based signal attempted')
                try:signal.pidfd_send_signal(child['pidfd'],number);sent.append(int(number))
                except ProcessLookupError:pass
                try:proc.wait(timeout=timeout)
                except subprocess.TimeoutExpired:continue
            record.update(returncode=proc.poll(),closed=proc.poll() is not None,signals_sent=sent)
        except (OSError,RuntimeError) as exc:errors.append(str(exc))
        finally:
            if child['pidfd'] is not None:os.close(child['pidfd'])
            child['log'].close();record['cleanup_errors']=errors;results.append(record)
    return results


def execute(plan,output,max_runtime):
    execution_context(plan['source_binding']);require(artifacts()==plan['artifacts'],'artifacts changed after preparation')
    require(sha(plan['config']['path'])==plan['config']['sha256'],'localization config changed after preparation')
    children=[];stops=[];result=dict(state='HOLD_STARTUP_OR_RUNTIME_ERROR',formal_navigation_acceptance=False)
    previous={number:signal.signal(number,lambda number,frame:stops.append(number)) for number in [signal.SIGINT,signal.SIGTERM]}
    started=time.monotonic()
    try:
        for role in ['localization','adapter']:
            if stops:raise InterruptedError('Root stopped bootstrap during startup')
            argv=plan['argv'][role];env=environment(role,output);log=(output/(role+'.log')).open('xb')
            try:proc=subprocess.Popen(argv,env=env,cwd=str(output),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            except BaseException:log.close();raise
            record=dict(role=role,pid=proc.pid,argv=argv,environment={key:env.get(key) for key in ['ROS_DOMAIN_ID','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','LD_LIBRARY_PATH','OMP_NUM_THREADS','OPENBLAS_NUM_THREADS']})
            child=dict(process=proc,pidfd=None,record=record,log=log);children.append(child)
            child['pidfd']=os.pidfd_open(proc.pid);record['identity']=process_identity(proc.pid)
            require(record['identity']['argv']==argv,'actual child argv differs')
            if role=='localization':require(record['identity']['executable_sha256']==LM_SHA,'actual LM executable differs')
            write(output/(role+'.started.json'),record)
        write(output/'inputs_started.json',dict(state='TWO_INPUT_CHILDREN_STARTED_NOT_LOCALIZATION_ACCEPTANCE',children=[child['record'] for child in children],artifacts=plan['artifacts']))
        while not stops and time.monotonic()-started<max_runtime and all(child['process'].poll() is None for child in children):time.sleep(.2)
        result['state']='ROOT_STOP_INPUTS_CLOSED' if stops else 'DEADLINE_INPUTS_CLOSED' if time.monotonic()-started>=max_runtime else 'CHILD_EXIT_INPUTS_CLOSED'
    except BaseException as exc:result.update(error_type=type(exc).__name__,error=str(exc))
    finally:
        result.update(children=cleanup(children),signals=stops,elapsed_seconds=time.monotonic()-started)
        for number,handler in previous.items():signal.signal(number,handler)
        result['all_owned_children_closed']=all(row.get('closed') is True and not row['cleanup_errors'] for row in result['children'])
        write(output/'result.json',result)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--closed-case-dir',required=True,type=Path)
    parser.add_argument('--map-dir',required=True,type=Path);parser.add_argument('--seed-provenance',type=Path)
    parser.add_argument('--source-release',required=True,type=Path);parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--max-runtime-sec',type=float,default=14400.)
    parser.add_argument('--lio-imu-ready-wake',choices=['template','false'],default='template',
        help='Source-only diagnostic of existing Native LIO scheduling; false requires separate timing/accuracy evaluation')
    args=parser.parse_args(argv);args.output=args.output.resolve()
    if not math.isfinite(args.max_runtime_sec) or not 0<args.max_runtime_sec<=14400.:parser.error('runtime must be finite in(0,14400]')
    args.output.mkdir(parents=True,exist_ok=False)
    try:
        plan=prepare(args);plan['max_runtime_sec']=args.max_runtime_sec;write(args.output/'preparation.json',plan)
        if args.execute:result=execute(plan,args.output,args.max_runtime_sec)
        else:result=dict(state='HOLD_PREPARED_NO_PROCESS',formal_navigation_acceptance=False);write(args.output/'result.json',result)
    except BaseException as exc:
        result=dict(state='HOLD_INPUTS_OR_ADMISSION',error_type=type(exc).__name__,error=str(exc),formal_navigation_acceptance=False)
        if not (args.output/'result.json').exists():write(args.output/'result.json',result)
    print(json.dumps(dict(output=str(args.output),state=result['state'])))
    expected_stop=result['state'] in ('ROOT_STOP_INPUTS_CLOSED','DEADLINE_INPUTS_CLOSED')
    return 0 if result['state']=='HOLD_PREPARED_NO_PROCESS' or (expected_stop and result.get('all_owned_children_closed') and 'error' not in result) else 2


if __name__=='__main__':raise SystemExit(main())
