#!/usr/bin/python3
"""ROOT_HOLD by default. Two conditional bounded-amplitude sustained lateral probes in one raw MCAP."""
import argparse
import hashlib
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
from calibration_observer import Observer


def stat_birth(pid):
    fields=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=pid,starttime_ticks=int(fields[19]),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())


def child_snapshot(owned):
    """Evidence only; a late snapshot can never authorize an unknown birth."""
    value=dict(pid=owned['process'].pid,birth3=None,proc_state=None,observed_identity=None)
    try:
        value['birth3']=stat_birth(value['pid'])
        value['proc_state']=(Path('/proc')/str(value['pid'])/'stat').read_text().rsplit(')',1)[1].split()[0]
    except BaseException as exc:value['stat_error']=dict(error_type=type(exc).__name__,error=str(exc))
    try:
        actual=deadline.process_identity(value['pid'])
        if value['birth3'] is None or any(actual[k]!=value['birth3'][k] for k in ['pid','starttime_ticks','boot_id']):raise c.Hold('snapshot argv/birth differs')
        value['observed_identity']=actual
    except BaseException as exc:value['identity_error']=dict(error_type=type(exc).__name__,error=str(exc))
    return value


def acquire_child_pidfd(owned,purpose):
    attempt=dict(purpose=purpose);owned['pidfd_attempts'].append(attempt);descriptor=None
    try:
        if owned['process'].poll() is not None:raise c.Hold('owned child already exited; no pidfd reopen')
        if owned['birth3'] is None:raise c.Hold('initial owned birth unknown; no later bootstrap')
        attempt['birth_before']=stat_birth(owned['process'].pid)
        if attempt['birth_before']!=owned['birth3']:raise c.Hold('owned birth changed before pidfd open')
        descriptor=os.pidfd_open(owned['process'].pid)
        attempt['birth_after']=stat_birth(owned['process'].pid)
        if attempt['birth_after']!=owned['birth3']:raise c.Hold('owned birth changed after pidfd open')
        attempt['same_birth_verified']=True;return descriptor
    except BaseException as exc:
        attempt.update(same_birth_verified=False,error_type=type(exc).__name__,error=str(exc))
        if descriptor is not None:os.close(descriptor)
        raise


def failed_child_cleanup(owned,original):
    """Only this exact Popen child; wait even when no safe signal is possible."""
    cleanup=dict(pid=owned['process'].pid,expected_argv=owned['expected_argv'],birth3=owned['birth3'],
        argv_ready=owned['argv_ready'],observed_identity=owned['birth'],initial_snapshot=owned.get('initial_snapshot'),
        original_error_type=type(original).__name__,original_error=str(original),pidfd_attempts=owned['pidfd_attempts'],
        wait_attempted=False,exited=False,still_live=None,returncode=None,errors=[])
    try:
        if owned['process'].poll() is None:
            if owned['fd'] is None:
                try:owned['fd']=acquire_child_pidfd(owned,'cleanup_recovery_once')
                except BaseException as exc:cleanup['errors'].append(dict(stage='pidfd_recovery',error_type=type(exc).__name__,error=str(exc)))
            if owned['fd'] is not None:interrupt(owned,owned['allowed_argv'])
    except BaseException as exc:cleanup['errors'].append(dict(stage='signal_or_poll',error_type=type(exc).__name__,error=str(exc)))
    finally:
        cleanup['wait_attempted']=True
        try:cleanup['wait_returncode']=owned['process'].wait(timeout=3.)
        except BaseException as exc:cleanup['errors'].append(dict(stage='wait',error_type=type(exc).__name__,error=str(exc)))
        try:
            cleanup['returncode']=owned['process'].poll();cleanup['exited']=cleanup['returncode'] is not None
            cleanup['still_live']=not cleanup['exited']
        except BaseException as exc:cleanup['errors'].append(dict(stage='final_poll',error_type=type(exc).__name__,error=str(exc)))
        cleanup['sigint_sent']=owned['signalled'];cleanup['final_snapshot']=child_snapshot(owned)
        cleanup['state']='OWNED_CHILD_EXITED' if cleanup['exited'] else 'HOLD_OWNED_CHILD_OUTSTANDING'
        if owned['fd'] is not None:
            try:os.close(owned['fd']);cleanup['pidfd_closed']=True
            except BaseException as exc:cleanup['errors'].append(dict(stage='close_pidfd',error_type=type(exc).__name__,error=str(exc)))
            finally:owned['fd']=None
    return cleanup


def child(argv,log,env,check=None):
    process=owned=None
    try:
        with log.open('x') as stream:
            process=subprocess.Popen(argv,cwd='/',stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,env=env,close_fds=True)
            # Retain before any /proc, pidfd, watch callback, or log close.
            owned=dict(process=process,fd=None,birth=None,birth3=None,argv_ready=False,signalled=False,
                expected_argv=list(argv),allowed_argv=[list(argv)],pidfd_attempts=[])
            owned['birth3']=stat_birth(process.pid)
            owned['initial_snapshot']=dict(pid=process.pid,birth3=owned['birth3'],observed_argv=None)
            owned['allowed_argv'].append(deadline.command_argv(argv))
            owned['fd']=acquire_child_pidfd(owned,'initial')
        # Stable across exec even when /proc/cmdline is momentarily empty.
        # Retain this before any watch callback; complete argv is a separate gate.
        until=time.monotonic()+2.
        while time.monotonic()<until:
            if select.select([owned['fd']],[],[],0)[0]:return owned
            try:
                actual=deadline.process_identity(process.pid)
                if any(actual[k]!=owned['birth3'][k] for k in ['pid','starttime_ticks','boot_id']) or actual['argv'] not in owned['allowed_argv']:raise c.Hold('unexpected owned child exec identity')
                owned['birth']=actual;owned['argv_ready']=True
            except ProcessLookupError:pass
            if check is not None:check()
            if owned['argv_ready']:return owned
            time.sleep(.005)
        raise c.Hold('owned child identity unavailable')
    except BaseException as original:
        if process is None:raise
        # An interruption after Popen but before retention has no trusted birth.
        if owned is None:owned=dict(process=process,fd=None,birth=None,birth3=None,argv_ready=False,signalled=False,
            expected_argv=list(argv),allowed_argv=[list(argv)],pidfd_attempts=[])
        cleanup=failed_child_cleanup(owned,original)
        setattr(original,'owned_spawn_cleanup',cleanup)
        try:deadline.atomic_json(log.with_suffix('.spawn_cleanup.json'),cleanup)
        except BaseException as exc:cleanup['evidence_write_error']=dict(error_type=type(exc).__name__,error=str(exc))
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


def semantic_sends(trace_path,sign,target=c.LEGACY_ENDPOINT):
    target=c.normalized_endpoint(target)
    sends=[json.loads(line) for line in trace_path.read_text().splitlines()]
    previous=None;neutral_seen=False
    for item in sends:
        c.permitted_payload(item['command'],sign)
        payload=json.dumps(item['command'],separators=(',',':'),allow_nan=False).encode()
        clocks=[item[k] for k in ['send_before_unix_ns','send_after_unix_ns','send_before_steady_ns','send_after_steady_ns']]
        if (item.get('event')!='actual_socket_sendto_interval_not_actuation' or item.get('send_success') is not True
            or item.get('target')!=list(target) or item.get('bytes')!=len(payload)
            or item.get('payload_sha256')!=hashlib.sha256(payload).hexdigest()
            or any(type(v) is not int or v<0 for v in clocks)
            or clocks[0]>clocks[1] or clocks[2]>clocks[3]
            or previous is not None and clocks[2]<=previous):raise c.Hold('invalid/failed semantic send evidence')
        if item['command']['lateral'] and neutral_seen:raise c.Hold('nonzero semantic send after cleanup neutral')
        if not item['command']['lateral']:neutral_seen=True
        previous=clocks[3]
    return sends


def metrics(history,trace_path,sign,target=c.LEGACY_ENDPOINT):
    sends=semantic_sends(trace_path,sign,target)
    moving=[s for s in sends if s['command']['lateral']]
    if not moving:return dict(state='FAILED_NO_MOTION_SEND',hard_plant_guarantee=False)
    first,last=moving[0],moving[-1]
    neutral=next((s for s in sends if not s['command']['lateral'] and s['send_before_steady_ns']>last['send_after_steady_ns']),None)
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
    motion_ok=delta is not None and math.copysign(1.,sign)*delta[1]>.01
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
        hard_plant_guarantee=False,scope='Only this bounded-amplitude/duration pair; no general motion model inferred.')


def sustained_classification(history,trace_path,sign,legacy,cutoff_stamp_ns,target=c.LEGACY_ENDPOINT):
    """Original GT only; command-time window is not simulator actuation time."""
    sends=semantic_sends(trace_path,sign,target);moving=[s for s in sends if s['command']['lateral']]
    failed=dict(state='FAILED_NOT_SUSTAINED_LATERAL',measurement_cutoff_gt_stamp_ns=cutoff_stamp_ns,
        signed_net_stopped_body_y_m=None,signed_final_active1s_body_vy_median_mps=None,
        final_active1s_complete=False,strict_stopped_at_cutoff=False,hard_plant_guarantee=False,
        basis='Original GT headers/probe-initial body yaw; last1s ends at last nonzero send-after time, not actuation. Parent freshness/stop gates remain required.')
    if not moving:return failed
    first,last=moving[0],moving[-1]
    neutral=next((s for s in sends if not s['command']['lateral'] and s['send_before_steady_ns']>last['send_after_steady_ns']),None)
    if neutral is None:return failed
    end=last['send_after_unix_ns'];start=end-1_000_000_000
    initial=next((s for s in reversed(history) if s['stamp_ns']<=first['send_before_unix_ns']),None)
    if initial is None or first['send_before_unix_ns']-initial['stamp_ns']>300_000_000:return failed
    rows=[s for s in history if initial['stamp_ns']<=s['stamp_ns']<=cutoff_stamp_ns]
    if len(rows)<2:return failed
    if rows[-1]['stamp_ns']!=cutoff_stamp_ns:return failed
    if any(not c.finite([*r['position'],*r['linear'],*r['angular']]) for r in rows):raise c.Hold('nonfinite sustained GT')
    if any(b['stamp_ns']<=a['stamp_ns'] for a,b in zip(rows,rows[1:])):raise c.Hold('nonadvancing sustained GT')
    direction=math.copysign(1.,sign);current=rows[-1];initial_yaw=c.rpy(initial['quaternion'])[2]
    net=c.body_xy(current['position'][0]-initial['position'][0],current['position'][1]-initial['position'][1],initial_yaw)
    tracker=c.StableStop()
    for row in rows:
        if row['stamp_ns']>=neutral['send_before_unix_ns']:tracker.update(row)
    stopped=tracker.start is not None and tracker.stamp-tracker.start[0]>=300_000_000
    covered=start;velocities=[];complete=True
    for a,b in zip(rows,rows[1:]):
        lo,hi=a['stamp_ns'],b['stamp_ns']
        if hi<=start or lo>=end:continue
        if lo>covered or hi-lo>100_000_000:complete=False;break
        dt=(hi-lo)/1e9
        vx,vy=c.body_xy((b['position'][0]-a['position'][0])/dt,(b['position'][1]-a['position'][1])/dt,initial_yaw)
        velocities.append(direction*vy);covered=max(covered,min(hi,end))
    complete=complete and covered>=end and bool(velocities)
    median=statistics.median(velocities) if complete else None
    passed=legacy.get('state')=='MEASURED_SHORT_PROBE' and stopped and direction*net[1]>.05 and median is not None and median>.03
    failed.update(state='MEASURED_SUSTAINED_LATERAL_PROBE' if passed else 'FAILED_NOT_SUSTAINED_LATERAL',
        signed_net_stopped_body_y_m=direction*net[1],signed_final_active1s_body_vy_median_mps=median,
        final_active1s_complete=complete,strict_stopped_at_cutoff=stopped,final_active1s_original_header_window_ns=[start,end],
        initial_gt_stamp_ns=initial['stamp_ns'],stopped_gt_stamp_ns=current['stamp_ns'],body_displacement_xy_m=list(net),
        final_active1s_velocity_intervals=len(velocities),short_probe_state=legacy.get('state'))
    return failed


def run(case_path,release_path,output):
    case,release=c.read(case_path),c.read(release_path);c.validate(case,release,case_path,output);patch=c.patch_check(case)
    selected_target=c.command_endpoint(case,release)
    output.mkdir(parents=True,exist_ok=False)
    c.claim(release_path.parent/(release['release_id']+'.lateral_once.json'),dict(release_sha256=c.sha(release_path),output_directory=str(output),claimed_unix_ns=time.time_ns()))
    for name in c.PACKAGE:shutil.copyfile(c.HERE/name,output/name)
    shutil.copyfile(case_path,output/'case.frozen.json');shutil.copyfile(release_path,output/'root_release.frozen.json')
    deadline.atomic_json(output/'patch_check.json',patch)
    release_sha,case_sha=c.sha(release_path),c.sha(case_path)
    result=dict(scope=c.SCOPE,state='HOLD_PREPARATION',source_signals_sent=0,host_signals_sent=0,probes=[],raw_cdr_reencoded=False)
    recorder=tool=guard=observer=None;anchor=None;stopped=[];tool_argv=None;tool_logs=[];probe_results={}
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
    def record_probe(index):
        try:
            legacy=metrics(observer.history,output/f'probe_{index}_send.jsonl',case['probes'][index],selected_target)
            sustained=sustained_classification(observer.history,output/f'probe_{index}_send.jsonl',case['probes'][index],legacy,observer.latest['/odom']['stamp_ns'],selected_target)
            item=dict(index=index,**legacy);item.update(short_probe_state=item['state'],state=sustained['state'],sustained=sustained)
        except BaseException as exc:item=dict(index=index,state='FAILED_METRICS',error=str(exc))
        probe_results[index]=item
        try:deadline.atomic_json(output/f'probe_{index}.acceptance.json',item)
        except BaseException as exc:
            item=dict(index=index,state='FAILED_METRICS',error='acceptance evidence write failed: '+str(exc),measurement=item)
            probe_results[index]=item
        return item
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
            if c.package_hashes()!=release['package_sha256']:raise c.Hold('source changed before semantic tool')
            if guard.manifest['deadline_boottime_sec']-time.clock_gettime(time.CLOCK_BOOTTIME)<15.:raise c.Hold('insufficient capped recording reserve')
            if index==1 and record_probe(0)['state']!='MEASURED_SUSTAINED_LATERAL_PROBE':raise c.Hold('positive sustained probe must still pass after between-stop before negative plan')
            c.claim(output/f'probe_{index}.planned.json',dict(release_id=release['release_id'],probe=index,original_gt_stamp_ns=observer.latest['/odom']['stamp_ns'],
                prior_positive_acceptance_sha256=c.sha(output/'probe_0.acceptance.json') if index==1 else None))
            watch(force_lease=True)  # Original stamps; refreshed lease, never restamped data.
            tool_argv=[sys.executable,'-u',str(c.HERE/c.TOOL_FILENAME),'--execute','--case',str(case_path),
                '--root-release',str(release_path),'--ready',str(output/'recorder_ready.json'),'--ready-sha256',c.sha(output/'recorder_ready.json'),'--probe-index',str(index)]
            tool=child(tool_argv,output/f'probe_{index}_stdout.receipt.log',dict(os.environ),check=watch);tool_logs.append(index)
            until=time.monotonic()+6.
            while tool['process'].poll() is None:
                watch()
                if time.monotonic()>until:raise c.Hold('semantic tool exceeded normal duration/finally budget')
            if tool['process'].returncode!=0:raise c.Hold('semantic tool failed/interrupted')
            os.close(tool['fd']);tool=None
            wait_stop()
            if record_probe(index)['state']!='MEASURED_SUSTAINED_LATERAL_PROBE':raise c.Hold('semantic probe lacks sustained signed tail velocity and net stopped displacement')
            if index==0:wait_stop(seconds=3.)
        wait_stop(seconds=5.,limit=8.)
        if record_probe(1)['state']!='MEASURED_SUSTAINED_LATERAL_PROBE':raise c.Hold('negative sustained net displacement did not survive final stopped tail')
        result['state']='COMPLETE_PROBES_STOPPED_PLUS5S_RAW_TAIL'
    except BaseException as exc:
        result.update(state='HOLD_DIAGNOSTIC_FAILED',error_type=type(exc).__name__,error=str(exc))
        if hasattr(exc,'owned_spawn_cleanup'):result['spawn_failed_child']=exc.owned_spawn_cleanup
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
                if path.exists() and index not in probe_results:record_probe(index)
            result['probes']=[probe_results[index] for index in sorted(probe_results)]
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
        if guard and (recorder is None or recorder['process'].poll() is not None) and result.get('spawn_failed_child',{}).get('exited',True):
            guard.recording_closed(result['state'])
            try:guard.process.wait(timeout=3.)
            except subprocess.TimeoutExpired:result['helper_exit_pending']=True
        if recorder and recorder['process'].poll() is not None:
            try:result['bag_audit']=bag_summary.audit(output/'bag',c.TOPICS)
            except BaseException as exc:result['bag_audit']=dict(complete=False,error=str(exc))
        result['diagnostic_passed']=bool(result['state']=='COMPLETE_PROBES_STOPPED_PLUS5S_RAW_TAIL' and len(result['probes'])==2
            and all(p['state']=='MEASURED_SUSTAINED_LATERAL_PROBE' for p in result['probes']) and result.get('bag_audit',{}).get('complete'))
        result.update(finished_unix_ns=time.time_ns(),scene_identity_basis='root external host-preflight, not continuous host monitoring',
            source_package_sha256=release['package_sha256'],no_motion_model_generalization=True)
        if 'spawn_failed_child' in result:result['all_spawn_failed_children_closed']=result['spawn_failed_child']['exited']
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
