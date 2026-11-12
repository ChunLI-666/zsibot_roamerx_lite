#!/usr/bin/python3
"""ROOT_HOLD by default. Exactly two short lateral probes in one raw MCAP."""
import argparse
import json
import math
import os
from pathlib import Path
import select
import shutil
import signal
import statistics
import subprocess
import sys
import time

import bag_summary
import lateral_contract as c
import recording_deadline as deadline


class Observer:
    def __init__(self,output):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.serialization import deserialize_message
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2,Imu,CompressedImage,CameraInfo
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO);self.rclpy=rclpy;self.decode=deserialize_message
        self.node=Node('lateral_calibration_observer',enable_rosout=False,start_parameter_services=False)
        self.latest={};self.history=[];self.errors=[];self.counts={t:0 for t in c.TOPICS};self.stop=c.StableStop();self.stream=(output/'observations.jsonl').open('x')
        types=[Odometry,PointCloud2,Imu,CompressedImage,CompressedImage,CameraInfo];self.subscriptions=[]
        for topic,typ in zip(c.TOPICS,types):
            self.subscriptions.append(self.node.create_subscription(typ,topic,
                lambda raw,info,t=topic,cls=typ:self.capture(t,cls,raw),qos_profile_sensor_data,raw=True))
    def capture(self,topic,typ,raw):
        now,steady=time.time_ns(),time.monotonic_ns()
        try:
            message=self.decode(raw,typ);h=message.header
            stamp=int(h.stamp.sec)*1_000_000_000+int(h.stamp.nanosec)
            if topic in self.latest and stamp<=self.latest[topic]['stamp_ns']:raise c.Hold('nonadvancing original header: '+topic)
            sample=dict(topic=topic,stamp_ns=stamp,received_unix_ns=now,received_steady_ns=steady,frame=h.frame_id,
                raw_cdr_bytes=len(raw),payload_valid=True)
            if topic=='/odom':
                p=message.pose.pose;t=message.twist.twist
                sample.update(child=message.child_frame_id,position=[p.position.x,p.position.y,p.position.z],
                    quaternion=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],
                    linear=[t.linear.x,t.linear.y,t.linear.z],angular=[t.angular.x,t.angular.y,t.angular.z])
                if self.history:
                    previous=self.history[-1];dt=(stamp-previous['stamp_ns'])/1e9
                    sample['position_speed_mps']=math.dist(sample['position'],previous['position'])/dt
                self.stop.update(sample);self.history.append(sample)
            elif topic=='/front_lidar':sample['payload_valid']=message.width>0 and len(message.data)==message.row_step*message.height
            elif topic.endswith('/compressed'):sample['payload_valid']=len(message.data)>100
            elif topic.endswith('/camera_info'):sample['payload_valid']=message.width>0 and message.height>0
            else:sample['payload_valid']=c.finite([message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z,
                message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z])
            self.latest[topic]=sample;self.counts[topic]+=1;self.stream.write(json.dumps(sample)+'\n')
        except Exception as exc:self.errors.append(str(exc))
    def spin(self):self.rclpy.spin_once(self.node,timeout_sec=.02)
    def close(self):self.stream.flush();self.stream.close();self.node.destroy_node();self.rclpy.shutdown()


def stat_birth(pid):
    fields=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=pid,starttime_ticks=int(fields[19]),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())


def child(argv,log,env,check=None):
    with log.open('x') as stream:
        process=subprocess.Popen(argv,cwd='/',stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,env=env,close_fds=True)
    descriptor=os.pidfd_open(process.pid)
    owned=dict(process=process,fd=descriptor,birth=None,birth3=None,argv_ready=False,signalled=False)
    try:
        # Stable across exec even when /proc/cmdline is momentarily empty.
        # Retain this before any watch callback; complete argv is a separate gate.
        owned['birth3']=stat_birth(process.pid)
        until=time.monotonic()+2.
        while time.monotonic()<until:
            if select.select([descriptor],[],[],0)[0]:return owned
            try:
                actual=deadline.process_identity(process.pid)
                if any(actual[k]!=owned['birth3'][k] for k in ['pid','starttime_ticks','boot_id']) or actual['argv'] not in [list(argv),deadline.command_argv(argv)]:raise c.Hold('unexpected owned child exec identity')
                owned['birth']=actual;owned['argv_ready']=True
            except ProcessLookupError:pass
            if check is not None:check()
            if owned['argv_ready']:return owned
            time.sleep(.005)
        raise c.Hold('owned child identity unavailable')
    except BaseException as original:
        # A guard failure during spawn/exec must not lose the newly owned tool.
        cleanup=dict(birth3=owned['birth3'],argv_ready=owned['argv_ready'],original_error=str(original))
        try:
            interrupt(owned,[list(argv),deadline.command_argv(argv)])
        except BaseException as exc:cleanup['signal_error']=str(exc)
        finally:
            try:process.wait(timeout=3.)
            except BaseException as exc:cleanup['wait_error']=str(exc)
            cleanup.update(sigint_sent=owned['signalled'],exited=process.poll() is not None)
            try:deadline.atomic_json(log.with_suffix('.spawn_cleanup.json'),cleanup)
            finally:os.close(descriptor)
        raise


def interrupt(owned,allowed_argv):
    if owned is None or owned['signalled'] or select.select([owned['fd']],[],[],0)[0]:return
    birth3=owned['birth3'];actual3=stat_birth(owned['process'].pid)
    if birth3 is None or actual3!=birth3:raise c.Hold('owned-child stat birth changed; no signal')
    try:actual=deadline.process_identity(owned['process'].pid)
    except ProcessLookupError:
        if owned['argv_ready']:raise c.Hold('admitted owned-child argv unavailable; no signal')
        # This is only the immediately retained Popen pidfd, before admission.
        # An exec-empty cmdline cannot turn it into another process instance.
    else:
        if any(actual[k]!=birth3[k] for k in ['pid','starttime_ticks','boot_id']) or actual['argv'] not in allowed_argv:raise c.Hold('owned-child argv changed; no signal')
    signal.pidfd_send_signal(owned['fd'],signal.SIGINT);owned['signalled']=True


def metrics(history,trace_path):
    sends=[json.loads(line) for line in trace_path.read_text().splitlines()]
    moving=[s for s in sends if s['command']['LX']]
    if not moving:return dict(state='FAILED_NO_MOTION_SEND',hard_plant_guarantee=False)
    first,last=moving[0],moving[-1]
    neutral=next((s for s in sends if not s['command']['LX'] and s['send_before_steady_ns']>last['send_after_steady_ns']),None)
    records=[s for s in history if first['send_before_unix_ns']<=s['stamp_ns']<=last['send_after_unix_ns']]
    velocities=[]
    for a,b in zip(records,records[1:]):
        dt=(b['stamp_ns']-a['stamp_ns'])/1e9;y0=c.rpy(a['quaternion'])[2];y1=c.rpy(b['quaternion'])[2]
        vx,vy=c.body_xy((b['position'][0]-a['position'][0])/dt,(b['position'][1]-a['position'][1])/dt,y0+c.wrap(y1-y0)/2)
        velocities.append((vx,vy))
    delta=None
    if len(records)>=2:
        delta=list(c.body_xy(records[-1]['position'][0]-records[0]['position'][0],records[-1]['position'][1]-records[0]['position'][1],c.rpy(records[0]['quaternion'])[2]))
    stop_start=None;stop_continuity_error=None
    if neutral:
        tracker=None;previous_stamp=None
        for row in history:
            if row['stamp_ns']<neutral['send_before_unix_ns']:continue
            if previous_stamp is not None:
                gap=row['stamp_ns']-previous_stamp
                if gap<=0:stop_continuity_error='nonadvancing original GT';break
                if gap>100_000_000:tracker=None
            previous_stamp=row['stamp_ns']
            zero=math.sqrt(sum(v*v for v in row['linear']))<=.03 and math.sqrt(sum(v*v for v in row['angular']))<=.05 and row.get('position_speed_mps',0.)<=.03
            if not zero:tracker=None;continue
            if tracker is None:tracker=row
            if math.dist(row['position'][:2],tracker['position'][:2])>.02 or abs(c.wrap(c.rpy(row['quaternion'])[2]-c.rpy(tracker['quaternion'])[2]))>.03:tracker=row
            if row['stamp_ns']-tracker['stamp_ns']>=300_000_000:stop_start=tracker['stamp_ns'];break
    percentile=lambda values,p:sorted(values)[min(len(values)-1,int((len(values)-1)*p))] if values else None
    sign=-float(first['command']['LX']);motion_ok=delta is not None and math.copysign(1.,sign)*delta[1]>.01
    return dict(state='MEASURED_SHORT_PROBE' if motion_ok and stop_start else 'FAILED_NO_MEASURABLE_LATERAL_OR_STOP',
        gt_original_stamp_samples=len(records),policy_lateral=sign,body_displacement_xy_m=delta,
        body_vy_median_mps=statistics.median([v[1] for v in velocities]) if velocities else None,
        body_vy_p10_mps=percentile([v[1] for v in velocities],.1),body_vy_p90_mps=percentile([v[1] for v in velocities],.9),
        first_motion_send_interval_ns=[first['send_before_unix_ns'],first['send_after_unix_ns']],
        first_neutral_send_interval_ns=[neutral['send_before_unix_ns'],neutral['send_after_unix_ns']] if neutral else None,
        gt_first_stopped_header_ns=stop_start,
        gt_stop_continuity_error=stop_continuity_error,
        gt_stopped_after_neutral_send_sec_interval=[(stop_start-neutral['send_after_unix_ns'])/1e9,(stop_start-neutral['send_before_unix_ns'])/1e9] if stop_start and neutral and motion_ok else None,
        timing_basis='Original GT header deltas and midpoint original body yaw. Send syscall interval is not simulator actuation time.',
        stop_definition='linear3Dnorm<=.03m/s, angular3Dnorm<=.05rad/s, stable position/yaw and advancing original GT >=.3s',
        hard_plant_guarantee=False,scope='Only these two one-second probes; no general motion model inferred.')


def run(case_path,release_path,output):
    case,release=c.read(case_path),c.read(release_path);c.validate(case,release,case_path,output);patch=c.patch_check(case)
    output.mkdir(parents=True,exist_ok=False)
    c.claim(release_path.parent/(release['release_id']+'.lateral_once.json'),dict(release_sha256=c.sha(release_path),output_directory=str(output),claimed_unix_ns=time.time_ns()))
    for name in c.PACKAGE:shutil.copyfile(c.HERE/name,output/name)
    shutil.copyfile(case_path,output/'case.frozen.json');shutil.copyfile(release_path,output/'root_release.frozen.json')
    deadline.atomic_json(output/'patch_check.json',patch)
    release_sha,case_sha=c.sha(release_path),c.sha(case_path)
    result=dict(scope=c.SCOPE,state='HOLD_PREPARATION',source_signals_sent=0,host_signals_sent=0,probes=[],raw_cdr_reencoded=False)
    recorder=tool=guard=observer=None;anchor=None;stopped=[];tool_argv=None;tool_logs=[]
    previous={n:signal.signal(n,lambda n,f:stopped.append(n)) for n in [signal.SIGINT,signal.SIGTERM]}
    parent_identity=deadline.process_identity(os.getpid());lease_written_ns=0
    def watch(require_sensor=True,force_lease=False):
        nonlocal lease_written_ns
        if stopped:raise c.Hold('parent interrupted')
        guard.check_live()
        if recorder and recorder['process'].poll() is not None:raise c.Hold('owned recorder exited')
        if c.sha(release_path)!=release_sha or c.sha(case_path)!=case_sha:raise c.Hold('frozen release/case changed')
        if require_sensor:
            observer.spin()
            if observer.errors:raise c.Hold('observer error: '+observer.errors[0])
            checked=c.check_live(observer.latest,anchor,time.time_ns(),time.monotonic_ns())
            if force_lease or time.monotonic_ns()-lease_written_ns>=20_000_000:
                lease_written_ns=time.monotonic_ns()
                deadline.atomic_json(output/'observation_lease.json',dict(release_id=release['release_id'],parent_identity=parent_identity,
                    written_steady_ns=lease_written_ns,samples=observer.latest,anchor=list(anchor) if anchor else None))
            return checked
    def wait_stop(seconds=0.,limit=6.):
        until=time.monotonic()+limit;stable_since=None
        while time.monotonic()<until:
            watch()
            if observer.stop.ready():
                if stable_since is None:stable_since=time.monotonic()
                if time.monotonic()-stable_since>=seconds:return
            else:stable_since=None
        raise c.Hold('fresh measured stop/tail gate timed out')
    try:
        guard=deadline.arm(output,60.)
        argv=['/opt/ros/jazzy/bin/ros2','bag','record','-a','--include-hidden-topics','--node-name','lateral_raw_'+release['release_id'],
              '-s','mcap','--storage-preset-profile','zstd_fast','--max-cache-size','67108864','-o',str(output/'bag')]
        command,env=guard.owned_command('recorder',argv,dict(os.environ));recorder=child(command,output/'recorder.log',env)
        guard.wait_for_child('recorder',recorder['process'].pid)
        registration=c.read(output/'recording_deadline_recorder_registration.json');recorder_identity=registration['identity']
        if deadline.process_identity(recorder['process'].pid)!=recorder_identity:raise c.Hold('recorder exec identity differs')
        end=time.monotonic()+6.;magic=[]
        while time.monotonic()<end:
            watch(False)
            magic=[str(p) for p in (output/'bag').glob('*.mcap') if p.open('rb').read(8)==bag_summary.MAGIC]
            if magic:break
            time.sleep(.02)
        if not magic:raise c.Hold('MCAP magic not opened before probes')
        ready=dict(release_id=release['release_id'],parent_identity=parent_identity,recorder_identity=recorder_identity,helper_identity=c.read(output/'recording_deadline_armed.json')['watchdog'],
            deadline_ack_verified=True,mcap_magic_verified=True,mcap_paths=magic,deadline_boottime_sec=guard.manifest['deadline_boottime_sec'])
        deadline.atomic_json(output/'recorder_ready.json',ready)
        observer=Observer(output);until=time.monotonic()+8.
        while set(observer.latest)!=set(c.TOPICS) and time.monotonic()<until:watch(False);observer.spin()
        wait_stop()
        gt,angles=watch();ref=case['initial_truth_reference']
        if math.dist(gt['position'][:2],ref[:2])>case['initial_xy_tolerance_m'] or abs(c.wrap(angles[2]-ref[2]))>.02:raise c.Hold('initial original GT anchor differs from reviewed patch')
        anchor=(*gt['position'][:2],angles[2]);result['original_truth_anchor']=list(anchor)
        for index in [0,1]:
            wait_stop();c.host_proof(release,case)
            if c.package_hashes()!=release['package_sha256']:raise c.Hold('source changed before official tool')
            if guard.manifest['deadline_boottime_sec']-time.clock_gettime(time.CLOCK_BOOTTIME)<15.:raise c.Hold('insufficient capped recording reserve')
            c.claim(output/f'probe_{index}.planned.json',dict(release_id=release['release_id'],probe=index,original_gt_stamp_ns=observer.latest['/odom']['stamp_ns']))
            watch(force_lease=True)  # Original stamps; refreshed lease, never restamped data.
            tool_argv=[sys.executable,'-u',str(c.HERE/'official_tool_timed.py'),'--execute','--case',str(case_path),
                '--root-release',str(release_path),'--ready',str(output/'recorder_ready.json'),'--ready-sha256',c.sha(output/'recorder_ready.json'),'--probe-index',str(index)]
            tool=child(tool_argv,output/f'probe_{index}_stdout.receipt.log',dict(os.environ),check=watch);tool_logs.append(index)
            until=time.monotonic()+3.
            while tool['process'].poll() is None:
                watch()
                if time.monotonic()>until:raise c.Hold('official tool exceeded normal duration/finally budget')
            if tool['process'].returncode!=0:raise c.Hold('official tool failed/interrupted')
            os.close(tool['fd']);tool=None
            wait_stop(seconds=3. if index==0 else 0.)
        wait_stop(seconds=5.,limit=8.)
        result['state']='COMPLETE_PROBES_STOPPED_PLUS5S_RAW_TAIL'
    except BaseException as exc:
        result.update(state='HOLD_DIAGNOSTIC_FAILED',error_type=type(exc).__name__,error=str(exc))
    finally:
        if tool is not None:
            try:interrupt(tool,[tool_argv])
            except Exception as exc:result['tool_close_error']=str(exc)
            finally:
                try:tool['process'].wait(timeout=3.)
                except BaseException as exc:result['tool_wait_error']=str(exc)
            result['tool_sigint_sent']=tool['signalled'];result['tool_exited']=tool['process'].poll() is not None
            os.close(tool['fd'])
        if observer is not None:
            # On a failure, keep recording while attempting a bounded fresh stop
            # and5s tail. Missing/stale GT remains an honest unverified failure.
            if result['state']=='HOLD_DIAGNOSTIC_FAILED':
                try:wait_stop(seconds=5.,limit=8.);result['failure_stop_and_tail_verified']=True
                except BaseException as exc:result['failure_stop_and_tail_verified']=False;result['stop_tail_error']=str(exc)
            for index in tool_logs:
                path=output/f'probe_{index}_send.jsonl'
                if path.exists():
                    try:result['probes'].append(dict(index=index,**metrics(observer.history,path)))
                    except BaseException as exc:result['probes'].append(dict(index=index,state='FAILED_METRICS',error=str(exc)))
            result['observed_topic_counts']=observer.counts
            try:observer.close()
            except BaseException as exc:result['observer_close_error']=str(exc)
        if guard:
            try:guard.begin_shutdown(result['state'])
            except BaseException as exc:result['helper_shutdown_error']=str(exc)
        if recorder is not None:
            try:
                plan=guard.children['recorder'];interrupt(recorder,[plan['wrapper_argv'],plan['executed_argv']])
            except BaseException as exc:result['recorder_close_error']=str(exc)
            finally:
                try:recorder['process'].wait(timeout=15.)
                except BaseException as exc:result['recorder_wait_error']=str(exc)
            result['recorder_exited']=recorder['process'].poll() is not None;result['recorder_sigint_sent']=recorder['signalled'];os.close(recorder['fd'])
        if guard and (recorder is None or recorder['process'].poll() is not None):
            guard.recording_closed(result['state'])
            try:guard.process.wait(timeout=3.)
            except subprocess.TimeoutExpired:result['helper_exit_pending']=True
        if recorder and recorder['process'].poll() is not None:
            try:result['bag_audit']=bag_summary.audit(output/'bag',c.TOPICS)
            except BaseException as exc:result['bag_audit']=dict(complete=False,error=str(exc))
        result['diagnostic_passed']=bool(result['state']=='COMPLETE_PROBES_STOPPED_PLUS5S_RAW_TAIL' and len(result['probes'])==2
            and all(p['state']=='MEASURED_SHORT_PROBE' for p in result['probes']) and result.get('bag_audit',{}).get('complete'))
        result.update(finished_unix_ns=time.time_ns(),scene_identity_basis='root external host-preflight, not continuous host monitoring',
            source_package_sha256=release['package_sha256'],no_motion_model_generalization=True)
        deadline.atomic_json(output/'result.json',result)
        for n,h in previous.items():signal.signal(n,h)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',type=Path,default=c.HERE/'case.HOLD.json')
    p.add_argument('--output',type=Path);p.add_argument('--root-release',type=Path);p.add_argument('--execute',action='store_true');args=p.parse_args()
    if not args.execute:
        print(json.dumps(dict(state='ROOT_HOLD',no_ROS_or_Popen_or_socket_or_signal=True,package_sha256=c.package_hashes())));return 0
    if args.output is None or args.root_release is None:p.error('explicit root release and new unique output required')
    result=run(args.case.resolve(),args.root_release.resolve(),args.output.resolve());print(json.dumps(dict(state=result['state'],diagnostic_passed=result['diagnostic_passed'])));return 0 if result['diagnostic_passed'] else 2


if __name__=='__main__':raise SystemExit(main())
