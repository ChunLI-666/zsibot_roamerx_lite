#!/usr/bin/env python3
"""Default-HOLD controlled static SIM raw-data acquisition owner.

Preparation creates no ROS context, subprocess or socket. Execution requires a
new root release, reviewed physical oracle/route and retired-writer proof. GT is
used only for acquisition feedback; no Nav TF/pose/status/seed is published.
"""
import argparse
from collections import deque
import fcntl
import json
import math
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import threading
import time

import full_scene_mapping_contract as contract
import lateral_capability
from imu_gate import ImuMonotonicGate

HERE=Path(__file__).resolve().parent
REPO=HERE
REQUIRED_RETIRED_ROLES={'old_sim_command_adapter','old_sim_udp_relay','old_navigation'}
RAW_RECORD_TOPICS={'/front_lidar','/front_lidar/imu','/odom','/front_camera/image/compressed',
  '/livox/lidar','/livox/imu','/laser_scan','/camera/front/image/compressed','/camera/front/camera_info','/tf_static','/clock'}
GUARDED_SOURCES={'/odom','/front_lidar','/laser_scan','/front_lidar/imu','/livox/imu'}


def source_files():
    return [HERE/name for name in ['full_scene_mapping.py','full_scene_mapping_contract.py',
      'mapping_lateral_relay.py','lateral_capability.py','imu_gate.py',
      'dependencies/waypoint_contract.py','dependencies/map_coverage.py']]


class BoundedCallbacks:
    """Private persistent executor dispatch budget; never restamps sensor data."""
    def __init__(self,executor,clock=time):self.executor=executor;self.clock=clock;self.dispatched=0
    def wrap(self,callback):
        def dispatch(*args,**kwargs):
            self.dispatched+=1
            return callback(*args,**kwargs)
        return dispatch
    def spin(self):
        deadline=self.clock.monotonic_ns()+10_000_000;count=0;empty=0
        while count<32:
            if self.clock.monotonic_ns()>=deadline:break
            before=self.dispatched;self.executor.spin_once(timeout_sec=0.);after=self.dispatched
            if after<=before:
                empty+=1
                if empty>=2:
                    remaining=deadline-self.clock.monotonic_ns()
                    if remaining>0:self.clock.sleep(min(.001,remaining/1e9))
                continue
            count+=after-before;empty=0


def validate_release(release,route,route_path,oracle_path,output):
    if release.get('scope')!=contract.SCOPE or release.get('mode')!=contract.MODE or release.get('allow_execute') is not True:
        raise contract.Hold('root release absent or not executable')
    rid=release.get('release_id','')
    if len(rid)!=32 or any(c not in '0123456789abcdef' for c in rid):raise contract.Hold('unique root release UUID required')
    if release.get('route_sha256')!=contract.sha(route_path) or release.get('oracle_sha256')!=contract.sha(oracle_path):raise contract.Hold('root route/oracle hash mismatch')
    if release.get('output_directory')!=str(output) or release.get('max_runtime_sec')!=route['max_runtime_sec']:raise contract.Hold('output/budget differs from root release')
    expected={str(p.relative_to(REPO)):contract.sha(p) for p in source_files()}
    if release.get('sources_sha256')!=expected:raise contract.Hold('root release does not bind all actual owner/relay/geometry/evidence sources')
    if release.get('boot_id')!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():raise contract.Hold('root boot binding differs')
    if release.get('network_namespace')!=os.readlink('/proc/self/ns/net'):raise contract.Hold('root network namespace differs')
    if release.get('controller_pid_namespace')!=os.readlink('/proc/self/ns/pid'):raise contract.Hold('root controller PID namespace differs')
    proof=release.get('scene_identity_proof',{})
    path=Path(proof.get('evidence_path',''))
    if proof.get('mode')!='root_external_host_attestation' or not path.is_absolute() or not path.is_file() or contract.sha(path)!=proof.get('evidence_sha256') or contract.read_json(path)!=route['scene_instance_id']:raise contract.Hold('root external host scene identity proof missing/changed')
    oracle_manifest=contract.read_json(oracle_path)
    artifacts=oracle_manifest.get('artifacts_sha256',{})
    if oracle_manifest.get('scene_instance_id')!=route['scene_instance_id'] or not any((Path(oracle_path).parent/key).resolve()==path and digest==proof['evidence_sha256'] for key,digest in artifacts.items()):raise contract.Hold('root host identity proof is not bound to this physical oracle')
    if not proof.get('host_pid_namespace') or proof['host_pid_namespace']!=route['scene_instance_id'].get('pid_namespace'):raise contract.Hold('external host scene PID namespace differs')
    if set(release.get('retired_writer_roles',{}))!=REQUIRED_RETIRED_ROLES:raise contract.Hold('exact old adapter, relay and Nav retirement identities required')
    if not all(release['retired_writer_roles'][role] for role in REQUIRED_RETIRED_ROLES):raise contract.Hold('empty retired-writer identity set')
    if not release.get('allowed_udp_source_processes') or not release.get('recorder_process_identity'):raise contract.Hold('root-reviewed UDP inventory/recorder identity missing')
    identities=[item for values in release['retired_writer_roles'].values() for item in values]+release['allowed_udp_source_processes']+[release['recorder_process_identity']]
    if not all(contract.valid_local_identity(item) for item in identities):raise contract.Hold('full local PID/start/argv/hash/boot identities required; partial identity cannot prove retirement')
    deadline=release.get('recorder_deadline_steady_ns')
    if type(deadline) is not int or (deadline-time.monotonic_ns())/1e9<route['max_runtime_sec']+5.:raise contract.Hold('raw recorder deadline must outlast the entire finite owner budget plus 5s')
    if not RAW_RECORD_TOPICS<=set(release.get('required_recorded_topics',[])):raise contract.Hold('complete original/derived raw recording topics required')
    if not release.get('recorder_node_name') or release.get('recorder_node_namespace')!='/':raise contract.Hold('exact recorder graph identity missing')
    ready=Path(release.get('recorder_ready_path',''))
    if not ready.is_absolute() or not ready.is_file() or contract.sha(ready)!=release.get('recorder_ready_sha256'):raise contract.Hold('exact raw recorder ready receipt missing/changed')
    receipt=contract.read_json(ready)
    if not isinstance(release.get('case_id'),str) or not release['case_id'] or not isinstance(release.get('recording_run_id'),str) or not release['recording_run_id']:raise contract.Hold('recording case/run binding missing')
    if receipt.get('scope')!='RAW_RECORDING_ONLY' or any(receipt.get(key)!=release.get(key) for key in ['case_id','recording_run_id','recorder_process_identity','recorder_deadline_steady_ns']):raise contract.Hold('raw recorder receipt identity/deadline binding differs')
    if receipt.get('node_name')!=release['recorder_node_name'] or receipt.get('node_namespace')!=release['recorder_node_namespace'] or not set(release['required_recorded_topics'])<=set(receipt.get('required_recorded_topics',[])):raise contract.Hold('raw recorder receipt graph/topic binding differs')
    if set(release.get('expected_sensor_publishers',{}))!=GUARDED_SOURCES or not all(len(value)==1 for value in release['expected_sensor_publishers'].values()):raise contract.Hold('exact unique original/derived sensor publisher binding required')
    if release.get('scene_instance_id')!=route.get('scene_instance_id'):raise contract.Hold('scene instance not bound')
    if release.get('lateral_capability')!=contract.CAPABILITY:raise contract.Hold('explicit root lateral capability release missing')
    if release.get('stop_envelope_reference')!=contract.ACTUAL_GT_STOP_REFERENCE or release.get('independent_future_errors_reviewed') is not True:
        raise contract.Hold('root actual-GT future-error stop-reference approval missing')
    config_key=oracle_manifest['sensor_config_json'];config_sha=oracle_manifest['artifacts_sha256'][config_key]
    if release.get('sensor_config_sha256')!=config_sha:raise contract.Hold('calibrated actual sensor configuration differs')
    adapter=release.get('sensor_adapter_process_identity')
    if not contract.valid_local_identity(adapter) or contract.process_identity(adapter['pid'])!=adapter:raise contract.Hold('exact calibrated replacement adapter absent/changed')
    admission=lateral_capability.validate_proof(release.get('lateral_calibration_proof',{}),route['scene_instance_id'],config_sha,adapter)
    lateral_capability.validate_model_evidence(oracle_manifest['motion_model'],admission)
    return admission


def atomic_json(path,value):
    temporary=path.with_name(path.name+'.tmp.'+str(os.getpid()))
    contract.claim_once(temporary,value);os.replace(temporary,path)


def template():
    return dict(schema_version=1,scope=contract.SCOPE,mode=contract.MODE,executable=False,
      stop_envelope_reference=contract.ACTUAL_GT_STOP_REFERENCE,independent_future_errors_reviewed=False,
      lateral_capability=contract.CAPABILITY,lateral_magnitude=contract.MAX_LATERAL_RAW,scene_instance_id=None,truth_frame_id=contract.TRUTH_FRAME,truth_child_frame_id=contract.TRUTH_CHILD,max_runtime_sec=600.,
      model_truth_age_cap_field='max_truth_age_sec',timing_bounds=dict(max_truth_age_sec=.05,other_input_age_sec=.3,renewal_sec=.10,relay_packet_age_sec=.10,relay_watchdog_sec=.15),
      start_anchor_truth=None,waypoints=[],required_root_evidence=['complete physical XML/mesh dependencies',
       'drivable floor/trinary acquisition oracle in original simulation truth coordinates',
       'raw-scan coordinate correspondence and actual T_body_lidar',
       'root-reviewed SIM engineering stop/drift/attitude/height bounds','static scene with no dynamic spawn or mutation',
       'retired adapter/relay/Nav identities and exact UDP ownership inventory','actual both-sign semantic closed-bag calibration and root directional engineering model','new once-only root release'],
      boundary='No route is invented. map_coverage.py may generate points only after root supplies the verified whole-scene physical floor oracle. This oracle/GT never becomes a Nav map, TF, localization seed or planner acceptance.')


class OwnedRelay:
    def __init__(self,output,run_id,root_release,model,owner_identity):
        self.process=None;self.pidfd=None;self.identity=None;self.output=output;self.command=(0.,0.,0.);self.run_id=run_id;self.close_errors=[]
        source=HERE/'mapping_lateral_relay.py';self.frozen=output/'mapping_lateral_relay_frozen.py';shutil.copy2(source,self.frozen)
        shutil.copy2(HERE/'lateral_capability.py',output/'lateral_capability.py')
        self.admission_path=output/'relay_admission.json'
        contract.claim_once(self.admission_path,dict(root_release_path=str(root_release.resolve()),root_release_sha256=contract.sha(root_release),
            owner_identity=owner_identity,motion_model=model))
        self.argv=[sys.executable,'-I',str(self.frozen),'--execute','--admission',str(self.admission_path),'--admission-sha256',contract.sha(self.admission_path)]
        self.source_sha256=contract.sha(self.frozen)
        self.log=(output/'relay.stderr').open('w')
    def start(self,pump=None):
        env=dict(os.environ,MAPPING_OWNER_RUN_ID=self.run_id,MAPPING_OWNER_OUTPUT=str(self.output))
        try:
            self.process=subprocess.Popen(self.argv,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
              stderr=self.log,text=True,bufsize=1,env=env,start_new_session=True)
            self.pidfd=os.pidfd_open(self.process.pid)
            os.set_blocking(self.process.stdin.fileno(),False)
            os.set_blocking(self.process.stdout.fileno(),False)
            until=time.monotonic()+3.;line=b''
            while True:
                if time.monotonic()>=until:raise contract.Hold('owned relay did not acknowledge readiness')
                if pump is not None:pump()
                remaining=until-time.monotonic()
                if remaining<=0 or self.process.poll() is not None:raise contract.Hold('owned relay did not acknowledge readiness')
                readable=select.select([self.process.stdout],[],[],0. if pump is not None else min(.01,remaining))[0]
                if time.monotonic()>=until:raise contract.Hold('owned relay did not acknowledge readiness')
                if not readable:continue
                chunk=os.read(self.process.stdout.fileno(),4096)
                if not chunk:raise contract.Hold('owned relay closed before readiness')
                line+=chunk
                if len(line)>4096:raise contract.Hold('owned relay readiness line exceeded bound')
                if b'\n' in line:
                    if line.split(b'\n',1)[0].strip()!=b'MAPPING_LATERAL_RELAY_RECT_V5_READY 127.0.0.1:7447':raise contract.Hold('owned relay did not acknowledge readiness')
                    break
            # Immediately retained pidfd binds the Popen child. Wait for the
            # immutable child's post-exec READY before sampling argv/identity.
            self.identity=contract.process_identity(self.process.pid)
            expected=self.argv
            if self.identity['argv']!=expected or contract.sha(self.frozen)!=self.source_sha256:raise contract.Hold('owned relay identity differs')
        except BaseException:
            self.close();raise

    def send(self,vx=0.,wz=0.,preflight=None,lateral_raw=0.):
        if self.process.poll() is not None or contract.process_identity(self.process.pid)!=self.identity:raise contract.Hold('owned relay exited/changed')
        if (vx or wz or lateral_raw) and preflight is None:raise contract.Hold('positive relay command requires final input guard')
        issued_ns,steady=time.time_ns(),time.monotonic()
        if preflight is not None:preflight(issued_ns,steady)
        packet=contract.primitive_packet(vx,wz,issued_ns,lateral_raw)
        payload=(json.dumps(packet)+'\n').encode()
        if os.write(self.process.stdin.fileno(),payload)!=len(payload):raise contract.Hold('incomplete nonblocking owned-relay command write')
        self.command=(vx,wz,lateral_raw)
        return packet

    def close(self):
        if self.process is None:self.log.close();return
        try:
            if self.identity is not None and self.process.poll() is None:self.send()
        except (OSError,ValueError,ProcessLookupError):pass
        try:self.process.stdin.close()
        except (OSError,ValueError):pass
        try:self.process.wait(timeout=3.)
        except subprocess.TimeoutExpired:
            try:
                if self.pidfd is not None and (self.identity is None or contract.process_identity(self.process.pid)==self.identity):
                    # Before READY, the immediately retained Popen pidfd is the
                    # exact owned child identity; argv has not been admitted.
                    signal.pidfd_send_signal(self.pidfd,signal.SIGINT)
            except (OSError,ValueError,ProcessLookupError) as exc:self.close_errors.append(str(exc))
            try:self.process.wait(timeout=3.)
            except subprocess.TimeoutExpired:pass
        if not self.log.closed:self.log.close()
        if self.pidfd is not None:os.close(self.pidfd);self.pidfd=None


def finalize_owned_relay(relay,observe_stop):
    """Zero first, then observe; any observation failure still closes only ours."""
    result=dict(stop_verified=False,relay_closed=False,relay_close_errors=[])
    try:
        try:relay.send()
        except BaseException as exc:result['zero_error']=str(exc)
        try:result['stop_verified']=bool(observe_stop())
        except BaseException as exc:result['stop_observation_error']=str(exc)
    finally:
        try:relay.close()
        except BaseException as exc:result['relay_close_errors'].append(str(exc))
        result['relay_close_errors']+=relay.close_errors
        result['relay_closed']=relay.process is None or relay.process.poll() is not None
    return result


def execute(args,route,oracle_manifest,release,output,lateral_admission):
    """The only branch that may create ROS/relay, called after explicit release."""
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2,Imu,LaserScan
    from std_msgs.msg import String
    sys.path.insert(0,str(HERE/'dependencies'))
    from waypoint_contract import AsyncEvidenceWriter,evidence_json

    if os.environ.get('ROS_DOMAIN_ID')!='185':raise contract.Hold('isolated same-host SIM domain185 only')
    if os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':raise contract.Hold('root-reviewed same-host FastDDS environment required')
    if not hasattr(os,'pidfd_open') or not hasattr(signal,'pidfd_send_signal'):raise contract.Hold('pidfd support required')
    if os.environ.get('MAPPING_OWNER_RUN_ID')!=release['release_id'] or os.environ.get('MAPPING_OWNER_OUTPUT')!=str(output):raise contract.Hold('root controller environment binding differs')
    started=time.monotonic();release_sha256=contract.sha(args.root_release)
    oracle=contract.OracleGrid.load(oracle_manifest,args.oracle);model=oracle_manifest['motion_model']
    if oracle_manifest.get('scene_instance_id')!=release['scene_instance_id']:raise contract.Hold('physical oracle belongs to another scene')
    retired=[item for identities in release['retired_writer_roles'].values() for item in identities]
    contract.verify_retired(retired)
    own=contract.process_identity(os.getpid());recorded=release['recorder_process_identity']
    if contract.process_identity(recorded['pid'])!=recorded:raise contract.Hold('exact preregistered raw recorder is not alive')
    allowed=release['allowed_udp_source_processes']+[recorded,own]
    contract.verify_udp_inventory(contract.udp_inventory(),allowed,release.get('allowed_external_udp_sockets',[]))
    lock=(Path('/tmp')/('matrix_full_scene_mapping_'+str(os.stat('/proc/self/ns/net').st_ino)+'.lock')).open('a+')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    stopped=[];previous={number:signal.signal(number,lambda n,f:stopped.append(n)) for number in [signal.SIGINT,signal.SIGTERM]}
    node=None;executor=None;relay=None;writer=None;stop_verified=False;result={};fault_snapshot=None;last_request=None;stop_observation={}
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node=Node('full_scene_mapping_owner_'+release['release_id'],enable_rosout=False,start_parameter_services=False)
        executor=SingleThreadedExecutor(context=node.context);executor.add_node(node);callbacks=BoundedCallbacks(executor)
        if node.get_parameter('use_sim_time').value is not False:raise contract.Hold('wall/source clock mode must remain false')
        raw_imu_gate=ImuMonotonicGate();active_lateral=None;imu_gate_fault=None
        latest={};history=deque(maxlen=1000);clouds=deque(maxlen=5);scans=deque(maxlen=5);subs=[];cloud_cache=None;last_send=None;clock_pair=None
        event_pub=node.create_publisher(String,'/experiment/full_scene_mapping/events',50)
        writer=AsyncEvidenceWriter(output,publish_event=lambda line:event_pub.publish(String(data=line)))
        def emit(kind,**fields):
            if not writer.submit('events',dict(kind=kind,scope=contract.SCOPE,run_id=release['release_id'],unix_ns=time.time_ns(),steady_ns=time.monotonic_ns(),**fields)):raise contract.Hold('bounded event writer rejected evidence')
        def stamp(message):return message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        def remember(key,message,value):
            item=dict(value,stamp_ns=stamp(message),frame_id=message.header.frame_id,received_steady=time.monotonic())
            prior=latest.get(key);item['stamp_regressed']=bool(prior and item['stamp_ns']<prior['stamp_ns'])
            item['advanced_steady']=item['received_steady'] if prior is None or item['stamp_ns']>prior['stamp_ns'] else prior['advanced_steady']
            latest[key]=item;return item
        def odom(message):
            p,q=message.pose.pose.position,message.pose.pose.orientation;t=message.twist.twist
            item=remember('truth',message,dict(xyz=[p.x,p.y,p.z],q=[q.x,q.y,q.z,q.w],child_frame_id=message.child_frame_id,
                twist=[t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z]))
            item['original_header_yaw_rate_rps']=None
            if history:
                previous=history[-1];dt=(item['stamp_ns']-previous['stamp_ns'])/1e9
                if dt>0:item['original_header_yaw_rate_rps']=contract.wrap(contract.rpy(item['q'])[2]-contract.rpy(previous['q'])[2])/dt
            history.append(item)
        def cloud(message):clouds.append(remember('raw_lidar',message,dict(message=message,width=message.width)))
        def scan(message):scans.append(remember('scan',message,dict(ranges=len(message.ranges))))
        def imu(key,message):
            a,g=message.linear_acceleration,message.angular_velocity
            return remember(key,message,dict(acceleration=[a.x,a.y,a.z],angular_velocity=[g.x,g.y,g.z]))
        def raw_imu(raw):
            nonlocal imu_gate_fault
            receipt_ns,steady_ns=time.time_ns(),time.monotonic_ns()
            event=raw_imu_gate.inspect(bytes(raw),receipt_ns,steady_ns)
            if not event['accepted']:
                if event['fatal']:
                    if imu_gate_fault is None:imu_gate_fault=event
                emit('raw_imu_rejected',event=event)
                return
            sample=imu('raw_imu',deserialize_message(raw,Imu))
            sample['received_steady']=sample['advanced_steady']=steady_ns/1e9
        subs.append(node.create_subscription(Imu,'/front_lidar/imu',callbacks.wrap(raw_imu),qos_profile_sensor_data,raw=True))
        for kind,topic,callback in [(Odometry,'/odom',odom),(PointCloud2,'/front_lidar',cloud),(LaserScan,'/laser_scan',scan),
          (Imu,'/livox/imu',lambda m:imu('imu',m))]:
            subs.append(node.create_subscription(kind,topic,callbacks.wrap(callback),qos_profile_sensor_data))
        # No Nav publisher/client is created. Only a JSON evidence publisher is present.
        # Passive truth precheck comes before any relay process: the frozen relay
        # emits action=walk even for zero, and must not stand a lying robot.
        truth_deadline=time.monotonic()+10.
        while 'truth' not in latest and time.monotonic()<truth_deadline:callbacks.spin()
        contract.standing_before_relay(latest.get('truth'),model,time.time_ns(),time.monotonic())
        if imu_gate_fault is not None:raise contract.Hold('latched raw IMU fault: '+imu_gate_fault['reason'])
        relay=OwnedRelay(output,release['release_id'],args.root_release,model,own);relay.start(callbacks.spin);allowed.append(relay.identity)
        audit=dict(valid=False,checked_steady=0.,reason='not_checked',first_violation=None);audit_stop=threading.Event()
        def audit_loop():
            while not audit_stop.is_set():
                try:
                    contract.verify_retired(retired)
                    if contract.process_identity(recorded['pid'])!=recorded:raise contract.Hold('recorder identity changed')
                    if contract.process_identity(relay.process.pid)!=relay.identity:raise contract.Hold('own relay identity changed')
                    contract.verify_udp_inventory(contract.udp_inventory(),allowed,release.get('allowed_external_udp_sockets',[]))
                    audit.update(valid=True,checked_steady=time.monotonic(),reason=None)
                except BaseException as exc:
                    first=audit['first_violation'] or dict(steady_ns=time.monotonic_ns(),reason=str(exc))
                    audit.update(valid=False,checked_steady=time.monotonic(),reason=str(exc),first_violation=first)
                audit_stop.wait(.1)
        auditor=threading.Thread(target=audit_loop,name='mapping-owner-inventory',daemon=True);auditor.start()
        def health():
            nonlocal cloud_cache,clock_pair
            now_ns,steady=time.time_ns(),time.monotonic()
            clock_pair=contract.check_clock(clock_pair,now_ns,steady)
            if steady-started>route['max_runtime_sec']-contract.CLEANUP_RESERVE:raise contract.Hold('fixed acquisition runtime budget expired; cleanup reserve begins')
            evidence=writer.summary()
            if evidence['dropped'] or evidence['worker_error'] is not None:raise contract.Hold('bounded writer evidence gap')
            if not audit['valid'] or audit['first_violation'] is not None or steady-audit['checked_steady']>.3:raise contract.Hold('velocity-owner audit missing/stale/violated: '+str(audit['reason']))
            if imu_gate_fault is not None:raise contract.Hold('latched raw IMU fault: '+imu_gate_fault['reason'])
            if not raw_imu_gate.fresh(now_ns,round(steady*1e9)):raise contract.Hold('no fresh accepted original IMU progress')
            truth=latest.get('truth');contract.fresh(truth,now_ns,steady,limit=contract.truth_age_limit(model))
            if truth['frame_id']!=contract.TRUTH_FRAME or truth['child_frame_id']!=contract.TRUTH_CHILD or not contract.finite([*truth['xyz'],*truth['twist']]):raise contract.Hold('invalid original GT frames/values')
            roll,pitch,yaw=contract.rpy(truth['q'])
            if max(abs(roll),abs(pitch))>model['max_roll_pitch_rad'] or not model['body_z_min_m']<=truth['xyz'][2]<=model['body_z_max_m']:raise contract.Hold('body attitude/height outside reviewed SIM bounds')
            if math.sqrt(sum(v*v for v in truth['twist'][:3]))>model['max_truth_speed_mps'] or abs(truth['twist'][5])>model['max_truth_yaw_rate_rps']:raise contract.Hold('actual truth speed exceeds approved envelope')
            for key,frame in [('raw_imu','map'),('imu','imu_link')]:
                sample=latest.get(key);contract.fresh(sample,now_ns,steady)
                if sample['frame_id']!=frame or not contract.finite(sample['acceleration']+sample['angular_velocity']):raise contract.Hold('invalid original/derived IMU')
            for sample in latest.values():
                if sample['stamp_regressed'] or steady-sample['advanced_steady']>.3:raise contract.Hold('regressed or nonadvancing original/derived source stamp')
            for sample in reversed(clouds):
                paired=next((s for s in scans if s['stamp_ns']==sample['stamp_ns'] and s['frame_id']=='base_link' and s['ranges']>0),None)
                if paired is None:continue
                contract.fresh(paired,now_ns,steady)
                contract.fresh(sample,now_ns,steady)
                if sample['frame_id']!='map':raise contract.Hold('raw lidar frame differs from frozen sensor-local contract')
                if cloud_cache is None or cloud_cache['stamp_ns']!=sample['stamp_ns']:
                    try:pose=contract.pose_at(history,sample['stamp_ns'])
                    except contract.Hold:continue
                    points=contract.checked_cloud(sample['message'])
                    cloud_cache=dict(stamp_ns=sample['stamp_ns'],raw_received_steady=sample['received_steady'],pose=pose,
                        points=contract.hits_in_truth(points,pose,oracle_manifest['T_body_lidar']))
                break
            if cloud_cache is None:raise contract.Hold('no fresh original-stamp raw/derived lidar+GT association')
            contract.fresh(dict(stamp_ns=cloud_cache['stamp_ns'],received_steady=cloud_cache['raw_received_steady']),now_ns,steady)
            for topic in ['/cmd_vel_nav','/cmd_vel','/cmd_vel_safe','/simulation/command_adapter/policy_cmd']:
                if node.get_publishers_info_by_topic(topic):raise contract.Hold('forbidden concurrent Nav/adapter velocity publisher: '+topic)
            nav_actions={'/navigate_to_pose','/navigate_through_poses','/follow_waypoints','/follow_path','/compute_path_to_pose'}
            if any(name.split('/_action/')[0] in nav_actions for name,types in node.get_service_names_and_types() if '/_action/' in name):raise contract.Hold('concurrent Nav action service remains present')
            for topic,expected in release['expected_sensor_publishers'].items():
                observed=[dict(name=info.node_name,namespace=info.node_namespace,type=info.topic_type) for info in node.get_publishers_info_by_topic(topic)]
                if observed!=expected:raise contract.Hold('original/derived publisher ownership changed: '+topic)
            identities=node.get_node_names_and_namespaces()
            guarded=[(release['recorder_node_name'],release['recorder_node_namespace'])]+[(entry[0]['name'],entry[0]['namespace']) for entry in release['expected_sensor_publishers'].values()]
            if any(identities.count(identity)!=1 for identity in guarded):raise contract.Hold('missing/duplicate recorder or sensor graph identity')
            required=release.get('required_recorded_topics',[])+['/experiment/full_scene_mapping/events']
            if not required:raise contract.Hold('raw recording topic contract absent')
            for topic in required:
                if (release['recorder_node_name'],release['recorder_node_namespace']) not in [(info.node_name,info.node_namespace) for info in node.get_subscriptions_info_by_topic(topic)]:raise contract.Hold('raw recorder subscription absent: '+topic)
            return truth,yaw,cloud_cache,(now_ns-truth['stamp_ns'])/1e9
        def tick(vx=0.,wz=0.,bridge=None,lateral_raw=0.):
            nonlocal last_send,last_request,active_lateral
            contract.primitive_packet(vx,wz,time.time_ns(),lateral_raw)
            last_request=dict(vx=vx,wz=wz,lateral_raw=lateral_raw,requested_unix_ns=time.time_ns())
            until=time.monotonic()+contract.PERIOD
            while time.monotonic()<until:callbacks.spin()
            if stopped:raise InterruptedError('mapping stop requested')
            if last_send is not None and time.monotonic()-last_send>contract.CONTROL_RENEWAL_LIMIT:raise contract.Hold('control-loop gap; no stale positive command is renewed')
            guard_started=time.monotonic();truth,yaw,cloud,age=health()
            age_cap=contract.truth_age_limit(model);age_upper=contract.command_age_upper(age,age_cap)
            if lateral_raw:
                if bridge is None or bridge[2].get('kind')!='lateral_fixed_body_heading':raise contract.Hold('lateral command missing frozen leg')
                lateral_context=(1 if lateral_raw>0 else -1,bridge)
                if active_lateral is not None and lateral_context!=active_lateral:raise contract.Hold('lateral sign/leg cannot change before measured stop')
            else:lateral_context=active_lateral
            if lateral_context is not None:
                if vx or wz:raise contract.Hold('lateral must reach measured stop before another primitive')
                sign,checked_bridge=lateral_context;kind='lateral';policy=checked_bridge[2]
                if sign!=policy['lateral_sign']:raise contract.Hold('actual lateral packet sign differs from checked stop reference')
                contract.lateral_leg_state(checked_bridge[0],checked_bridge[1],(*truth['xyz'][:2],yaw),policy)
                if math.sqrt(sum(v*v for v in truth['twist'][:3]))>contract.lateral_model(model,sign)['max_truth_speed_mps']:raise contract.Hold('actual lateral truth speed exceeds directional model')
            else:
                kind='forward' if vx else 'turn' if wz else 'zero';sign=1 if wz>0 else -1 if wz<0 else 0
                checked_bridge=bridge;policy=None
            stop_reference=None
            if kind=='lateral':
                sweep,inflation,stop_reference=contract.actual_gt_lateral_stopping_sweep(
                    oracle,truth,checked_bridge,model,time.time_ns(),time.monotonic())
                age_upper=stop_reference['input_age_upper_s']
            else:sweep,inflation=contract.stopping_sweep(oracle,(*truth['xyz'][:2],yaw),kind,sign,model,age_upper,policy)

            visibility=contract.guard_hits(cloud['points'],sweep,inflation,model,oracle)
            # Geometry/graph work must not turn a previously fresh sample into a
            # stale positive command with a new packet timestamp.
            final_facts={}
            def final_guard(issued_ns,steady):
                contract.check_clock(clock_pair,issued_ns,steady)
                final_facts['issued_steady_sec']=steady
                final_facts['pose']=contract.command_time_pose_guard(truth,issued_ns,steady,guard_started,age_upper,age_cap)
                if checked_bridge is not None:
                    start,target,leg_policy=checked_bridge
                    check=contract.lateral_leg_state if leg_policy.get('kind')=='lateral_fixed_body_heading' else contract.straight_leg_state
                    final_facts['bridge']=check(start,target,(*truth['xyz'][:2],yaw),leg_policy)
                final_facts['inputs']={key:contract.fresh(value,issued_ns,steady,limit=age_cap if key=='truth' else contract.MAX_INPUT_AGE) for key,value in latest.items()}
                final_facts['cloud']=contract.fresh(dict(stamp_ns=cloud['stamp_ns'],received_steady=cloud['raw_received_steady']),issued_ns,steady)
                if last_send is not None and steady-last_send>contract.CONTROL_RENEWAL_LIMIT:raise contract.Hold('guard computation exceeded command renewal bound')
                if not audit['valid'] or audit['first_violation'] is not None or steady-audit['checked_steady']>.3:raise contract.Hold('owner audit expired during guard computation')
            packet=relay.send(vx,wz,preflight=final_guard,lateral_raw=lateral_raw);last_send=final_facts['issued_steady_sec']
            if lateral_raw:active_lateral=lateral_context
            if time.monotonic()-last_send>contract.CONTROL_RENEWAL_LIMIT:raise contract.Hold('command write/dispatch exceeded renewal bound; issued anchor retained')
            accepted=writer.submit('state_trace',dict(unix_ns=time.time_ns(),scope=contract.SCOPE,truth=truth,lidar_stamp_ns=cloud['stamp_ns'],
              lidar_gt_association=cloud['pose'],command=packet,policy_vx=vx,policy_wz=wz,policy_lateral_raw=lateral_raw,primitive=kind,
              lateral_stop_envelope_retained=bool(lateral_context),
              actual_GT_stop_reference=stop_reference,
              original_inputs={key:{k:v for k,v in value.items() if k!='message'} for key,value in latest.items()},
              final_command_guard=final_facts,input_ages_at_command=final_facts['inputs'],used_lidar_age_at_command=final_facts['cloud'],
              stop_sweep_samples=len(sweep),inflation_m=inflation,visibility=visibility,owner_audit=dict(audit)))
            if not accepted:raise contract.Hold('bounded state writer rejected evidence')
            return truth,yaw
        def stable(seconds,timeout):
            nonlocal active_lateral
            begin=time.monotonic();checker=contract.StableStop(contract.truth_age_limit(model))
            while time.monotonic()-begin<timeout:
                truth,yaw=tick()
                if checker.update(truth,relay.command,time.time_ns(),time.monotonic(),seconds):
                    active_lateral=None;return truth,yaw
            raise contract.Hold('fresh truth/zero-command stable stop not observed')
        # Warm data before requesting a continuous 10s stopped interval.
        deadline=time.monotonic()+40.
        while True:
            try:truth,yaw=tick();break
            except contract.Hold as exc:
                if imu_gate_fault is not None:raise
                relay.send()
                if 'first_warm_hold' not in locals():
                    first_warm_hold=str(exc);emit('first_warm_hold',reason=first_warm_hold)
                if audit['checked_steady'] and not audit['valid']:raise
                if time.monotonic()>=deadline:raise
        truth,yaw=stable(contract.START_STABLE,max(0.,deadline-time.monotonic()))
        anchor=route['start_anchor_truth']
        if math.hypot(truth['xyz'][0]-anchor[0],truth['xyz'][1]-anchor[1])>.1 or abs(contract.wrap(yaw-anchor[2]))>.08:raise contract.Hold('fresh truth start differs from frozen root anchor')
        atomic_json(output/'runtime_ready.json',dict(scope=contract.SCOPE,mode=contract.MODE,run_id=release['release_id'],
          owner_process_identity=own,relay_identity=relay.identity,relay_source_sha256=relay.source_sha256,
          root_release_sha256=release_sha256,route_sha256=release['route_sha256'],oracle_sha256=release['oracle_sha256'],
          sources_sha256=release['sources_sha256'],lateral_admission=lateral_admission,output_directory=str(output),truth=truth,start_stable_seconds=contract.START_STABLE,
          actual_truth_age_cap_sec=contract.truth_age_limit(model),lateral_magnitude=contract.MAX_LATERAL_RAW,
          scene_identity_basis='root external host attestation; no direct host UE monitoring',controller_pid_namespace=release['controller_pid_namespace'],scene_identity_proof=release['scene_identity_proof'],
          checked_unix_ns=time.time_ns(),checked_steady_ns=time.monotonic_ns(),boundary='Preflight evidence only; no additional motion permission.'))
        def rotate_to(target,tolerance=.08,deadline_steady=None):
            while True:
                if deadline_steady is not None and time.monotonic()>=deadline_steady:raise contract.Hold('bounded heading correction deadline expired')
                truth,yaw=health()[:2];error=contract.wrap(target-yaw)
                if abs(error)<=tolerance:break
                tick(0.,math.copysign(contract.MAX_WZ,error))
            remaining=8. if deadline_steady is None else min(8.,max(0.,deadline_steady-time.monotonic()))
            truth,yaw=stable(contract.STOP_STABLE,remaining)
            if deadline_steady is not None and time.monotonic()>deadline_steady:raise contract.Hold('heading settle exceeded correction deadline')
            if abs(contract.wrap(target-yaw))>tolerance:raise contract.Hold('settled heading outside acquisition tolerance')
            return truth,yaw
        for index,point in enumerate(route['waypoints']):
            x,y,heading=point['pose_truth'];emit('point_entered',index=index,original_truth_target=point)
            policy=point.get('leg_policy');leg_start=tuple(route['start_anchor_truth'] if index==0 else route['waypoints'][index-1]['pose_truth'])
            lateral=policy is not None and policy.get('kind')=='lateral_fixed_body_heading'
            tolerance=policy['endpoint_tolerance_m'] if lateral else contract.translation_endpoint_tolerance(leg_start,(x,y,heading))
            if policy is not None:
                stable(contract.STOP_STABLE,8.)
                rotate_to(policy['body_yaw_rad'] if lateral else policy['travel_yaw_rad'],policy['max_yaw_error_rad'])
            entry_truth,entry_yaw=health()[:2];leg_entry=(*entry_truth['xyz'][:2],entry_yaw);corrections=0;correction_used=0.;feedback=None
            while True:
                truth,yaw=health()[:2];dx,dy=x-truth['xyz'][0],y-truth['xyz'][1]
                if lateral:
                    state=contract.lateral_leg_state(leg_start,(x,y,heading),(*truth['xyz'][:2],yaw),policy)
                    if state['reached']:
                        contract.require_translation_progress(leg_start,(x,y,heading),leg_entry,(*truth['xyz'][:2],yaw),tolerance);break
                    feedback=contract.feedback_window(feedback,state,time.monotonic(),state['along_m'])
                    decision=contract.lateral_correction_state(state,(*truth['xyz'][:2],yaw),policy,corrections,model)
                    if decision['observe_cross'] and feedback is None:
                        feedback=dict(started_steady=time.monotonic(),start_along_m=state['along_m'],start_abs_cross_m=abs(state['cross_track_m']))
                    if decision['stop_then_recheck']:
                        # Zero through the signed actual-GT stop envelope first.
                        # stable() alone clears active_lateral after measured stop.
                        remaining=contract.LATERAL_CORRECTION_BUDGET_SEC-correction_used
                        if remaining<=0:raise contract.Hold('total per-leg correction budget exhausted')
                        correction_started=time.monotonic();correction_deadline=correction_started+remaining
                        truth,yaw=stable(contract.STOP_STABLE,min(8.,remaining))
                        state=contract.lateral_leg_state(leg_start,(x,y,heading),(*truth['xyz'][:2],yaw),policy)
                        desired=contract.lateral_heading_target(state,(*truth['xyz'][:2],yaw),policy,model)
                        fine=desired['settled_yaw_tolerance_rad'];turned=desired['yaw_error_to_target_rad']>fine
                        if turned:
                            truth,yaw=rotate_to(desired['target_body_yaw_rad'],fine,correction_deadline)
                            corrections+=1
                        correction_used+=time.monotonic()-correction_started
                        if correction_used>contract.LATERAL_CORRECTION_BUDGET_SEC:raise contract.Hold('total heading correction work exceeded20s')
                        state=contract.lateral_leg_state(leg_start,(x,y,heading),(*truth['xyz'][:2],yaw),policy)
                        aligned=contract.lateral_heading_target(state,(*truth['xyz'][:2],yaw),policy,model)
                        if aligned['yaw_error_to_target_rad']>aligned['settled_yaw_tolerance_rad']:raise contract.Hold('settled compensation target changed beyond attained tolerance')
                        feedback=dict(started_steady=time.monotonic(),start_along_m=state['along_m'],start_abs_cross_m=abs(state['cross_track_m'])) if abs(state['cross_track_m'])>=decision['cross_trigger_m'] else None
                        emit('same_leg_stopped_heading_compensation',index=index,correction=corrections,correction_work_sec=correction_used,turned=turned,truth=truth,decision=aligned)
                        continue  # Same start/target/entry/sign, no new route or axis.
                    tick(bridge=(leg_start,(x,y,heading),policy),lateral_raw=policy['lateral_sign']*contract.MAX_LATERAL_RAW)
                    continue
                if policy is not None:contract.straight_leg_state(leg_start,(x,y,heading),(*truth['xyz'][:2],yaw),policy)
                if math.hypot(dx,dy)<=tolerance:
                    contract.require_translation_progress(leg_start,(x,y,heading),leg_entry,(*truth['xyz'][:2],yaw),tolerance);break
                bearing=math.atan2(dy,dx)
                if policy is not None:tick(contract.MAX_VX,0.,bridge=(leg_start,(x,y,heading),policy))
                elif abs(contract.wrap(bearing-yaw))>.12:
                    stable(contract.STOP_STABLE,8.);rotate_to(bearing)
                else:tick(contract.MAX_VX,0.)
            stable(contract.STOP_STABLE,8.);truth,yaw=rotate_to(heading)
            if math.hypot(x-truth['xyz'][0],y-truth['xyz'][1])>tolerance:raise contract.Hold('settled acquisition XY target missed; no retry')
            progress=contract.require_translation_progress(leg_start,(x,y,heading),leg_entry,(*truth['xyz'][:2],yaw),tolerance)
            emit('point_visited',index=index,truth=truth,target=point,measured_and_commanded_stop=True,actual_leg_progress=progress)
        truth,yaw=stable(contract.END_STABLE,28.);stop_verified=True
        result=dict(classification='controlled_static_sim_raw_acquisition_completed',points=len(route['waypoints']),
          final_truth=truth,start_stable_seconds=contract.START_STABLE,end_stable_seconds=contract.END_STABLE)
    except BaseException as exc:
        if 'latest' in locals():fault_snapshot={key:dict(value) for key,value in latest.items()}
        result=dict(classification='HOLD_or_interrupted_acquisition_incomplete',error_type=type(exc).__name__,error=str(exc))
    finally:
        # No evidence I/O precedes zeroing the owned relay on this exit path.
        if relay is not None:
            def observe_stop():
                if node is None:return False
                checker=contract.StableStop(contract.truth_age_limit(model));until=time.monotonic()+8.
                while rclpy.ok() and time.monotonic()<until:
                    callbacks.spin()
                    try:
                        relay.send();sample=latest.get('truth')
                        if active_lateral is not None:contract.directional_yaw_guard(sample,model,active_lateral[0])
                        contract.standing_before_relay(sample,model,time.time_ns(),time.monotonic())
                        if checker.update(sample,relay.command,time.time_ns(),time.monotonic(),contract.STOP_STABLE):
                            stop_observation.update(truth=sample,command=list(relay.command),checked_unix_ns=time.time_ns(),checked_steady_ns=time.monotonic_ns(),continuous_seconds=contract.STOP_STABLE)
                            return True
                    except BaseException:checker=contract.StableStop(contract.truth_age_limit(model))
                return False
            closed=finalize_owned_relay(relay,observe_stop);result.update(closed);stop_verified=closed['stop_verified']
            result.update(relay_identity=relay.identity,relay_source_sha256=relay.source_sha256,relay_returncode=relay.process.poll() if relay.process is not None else None)
            if not result['relay_closed']:result['classification']='HOLD_owned_relay_not_closed'
            elif result['relay_close_errors']:result['classification']='HOLD_owned_relay_close_error'
        if 'audit_stop' in locals():audit_stop.set();auditor.join(timeout=.5)
        result.update(lateral_admission=lateral_admission,imu_gate=raw_imu_gate.diagnostics(time.time_ns(),time.monotonic_ns()) if 'raw_imu_gate' in locals() else None,
                      scope=contract.SCOPE,mode=contract.MODE,run_id=release['release_id'],stop_verified=stop_verified,
                      final_stop_observation=stop_observation,owner_process_identity=own,
                      controller_pid_namespace=release['controller_pid_namespace'],scene_identity_basis='root external host attestation; no direct host UE monitoring',scene_identity_proof=release['scene_identity_proof'],
                      runtime_binding=dict(root_release_sha256=release_sha256,route_sha256=release['route_sha256'],oracle_sha256=release['oracle_sha256'],sources_sha256=release['sources_sha256'],
                        output_directory=str(output),case_id=release.get('case_id'),recording_run_id=release.get('recording_run_id'),recorder_ready_sha256=release['recorder_ready_sha256']),
                      signals=stopped,elapsed_seconds=time.monotonic()-started,formal_navigation_acceptance=False)
        if fault_snapshot is not None:
            summary={}
            for key,value in fault_snapshot.items():
                message=value.pop('message',None)
                if message is not None:value['original_point_data_sha256']=__import__('hashlib').sha256(message.data).hexdigest()
                summary[key]=value
            result['first_fault_original_inputs']=summary;result['last_requested_command']=last_request
        if result.get('classification')=='controlled_static_sim_raw_acquisition_completed' and not stop_verified:
            result['classification']='HOLD_final_measured_stop_unobserved'
        if writer is not None:
            result['writer']=writer.close()
            if not result['writer']['evidence_complete']:result['classification']='HOLD_evidence_gap'
        if executor is not None:
            try:
                executor.remove_node(node)
                if not executor.shutdown(timeout_sec=0.):raise contract.Hold('owned mapping executor did not stop')
            except BaseException as exc:result.update(classification='HOLD_executor_close_error',executor_close_error=str(exc))
        if node is not None:
            try:node.destroy_node()
            except BaseException as exc:result.update(classification='HOLD_node_close_error',node_close_error=str(exc))
        if rclpy.ok():
            try:rclpy.shutdown()
            except BaseException as exc:result.update(classification='HOLD_context_close_error',context_close_error=str(exc))
        (output/'result.json').write_text(evidence_json(result,indent=2)+'\n')
        for number,handler in previous.items():signal.signal(number,handler)
        lock.close()
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--route',type=Path)
    parser.add_argument('--oracle',type=Path);parser.add_argument('--root-release',type=Path)
    parser.add_argument('--output',required=True,type=Path);args=parser.parse_args()
    output=args.output.resolve()
    if output.exists():parser.error('unique output required; never overwrite an attempt')
    output.mkdir(parents=True)
    manifest=dict(schema_version=1,scope=contract.SCOPE,mode=contract.MODE,execution_requested=args.execute,
      sources_sha256={str(p.relative_to(REPO)):contract.sha(p) for p in source_files()},template=template())
    (output/'preparation.json').write_text(json.dumps(manifest,indent=2)+'\n')
    try:
        if args.route is None or args.oracle is None:raise contract.Hold('root physical oracle/route pending; executable:false template only')
        route=contract.validate_route(contract.read_json(args.route),args.execute)
        oracle=contract.validate_oracle(contract.read_json(args.oracle),args.oracle)
        grid=contract.OracleGrid.load(oracle,args.oracle)
        geometry=contract.audit_route_geometry(grid,route,oracle['motion_model'])
        (output/'route_geometry_audit.json').write_text(json.dumps(geometry,indent=2)+'\n')
        if not args.execute:result=dict(classification='PREPARED_ONLY_NO_ROS_NO_MOTION',route_sha256=contract.sha(args.route),oracle_sha256=contract.sha(args.oracle))
        else:
            if args.root_release is None:raise contract.Hold('explicit once-only root release required')
            release=contract.read_json(args.root_release);admission=validate_release(release,route,args.route,args.oracle,output)
            marker=args.root_release.resolve().parent/(release['release_id']+'.mapping_once.json')
            contract.claim_once(marker,dict(scope=contract.SCOPE,release_sha256=contract.sha(args.root_release),
              output_directory=str(output),owner_identity=contract.process_identity(os.getpid()),claimed_unix_ns=time.time_ns()))
            result=execute(args,route,oracle,release,output,admission)
    except BaseException as exc:
        result=dict(classification='HOLD_no_execution_or_incomplete',error_type=type(exc).__name__,error=str(exc),formal_navigation_acceptance=False)
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    if not (output/'result.json').exists():(output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(output=str(output),classification=result['classification'])))
    return 0 if not args.execute else 0 if result['classification']=='controlled_static_sim_raw_acquisition_completed' else 2


if __name__=='__main__':raise SystemExit(main())
