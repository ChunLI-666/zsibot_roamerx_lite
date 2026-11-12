#!/usr/bin/python3
"""Default-HOLD read-only 10s sensor capture; sole owned runtime child is recorder."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import bag_summary
import readiness_contract as c
import recording_deadline as deadline
from owned_recorder import child,interrupt
from readiness_observer import Observer

def bounded_discovery(observer,watch):
    """Use the original3s discovery budget fully; admit only current end state."""
    until=time.monotonic()+3.
    while time.monotonic()<until:
        watch()
        try:
            graph=observer.graph();observer.validator.graph(graph)
            observer.validator.check(time.time_ns(),time.monotonic_ns())
        except c.Hold:
            pass  # No admission; original watch still raises all capture faults.
    try:
        graph=observer.graph();observer.validator.graph(graph)
        observer.validator.check(time.time_ns(),time.monotonic_ns())
        observer.validator.graph(graph,pin=True)
    except c.Hold as exc:raise c.Hold('3s bounded discovery ended without fresh complete inputs: '+str(exc)) from exc

def run(config_path,release_path,output):
    config,release=c.read(config_path),c.read(release_path);c.validate(config,release,config_path,output)
    output.mkdir(parents=True,exist_ok=False)
    c.claim(release_path.parent/(release['release_id']+'.readiness_once.json'),dict(scope=c.SCOPE,release_sha256=c.sha(release_path),output_directory=str(output)))
    for name in c.PACKAGE:shutil.copyfile(c.HERE/name,output/name)
    shutil.copyfile(config_path,output/'config.frozen.json');shutil.copyfile(release_path,output/'root_release.frozen.json')
    result=dict(scope=c.SCOPE,state='HOLD',ready_for_motion=False,no_motion_authority=True,source_signals_sent=0,scene_signals_sent=0,bag_CDR_reencoded=False,
        expected_IMU_CDR_serialized_for_comparison_only=True,launcher_deadline_sec=45.,owned_recorder_fallback_SIGINT_by_sec=60.,hard_IO_exit_guarantee=False)
    guard=recorder=observer=source_stream=None;stopped=[];last_source=last_artifacts=last_graph=None;last_data_check=None
    fixed=(c.sha(config_path),c.sha(release_path));previous={n:signal.signal(n,lambda n,f:stopped.append(n)) for n in (signal.SIGINT,signal.SIGTERM)}
    def watch(data=False):
        nonlocal last_source,last_artifacts,last_data_check
        if stopped:raise c.Hold('read-only capture parent interrupted')
        guard.check_live()
        if recorder is not None and recorder['process'].poll() is not None:raise c.Hold('owned recorder exited before capture close')
        now=time.monotonic_ns()
        if last_source is None or now-last_source>=20_000_000:
            identities=c.source_live(config);source_stream.write(json.dumps(dict(received_unix_ns=time.time_ns(),received_steady_ns=now,identities=identities))+'\n');source_stream.flush();last_source=now
        if last_artifacts is None or now-last_artifacts>=1_000_000_000:
            if (c.sha(config_path),c.sha(release_path))!=fixed or c.package_hashes()!=release['package_sha256']:raise c.Hold('frozen capture inputs changed')
            c.artifacts(config);c.host_proof(release,config);last_artifacts=now
        if observer is not None:
            observer.spin()
            if observer.validator.errors:raise c.Hold(observer.validator.errors[0])
            if data:
                steady=time.monotonic_ns()
                if last_data_check is not None and steady-last_data_check>100_000_000:raise c.Hold('readiness watch gap exceeded100ms')
                observer.validator.check(time.time_ns(),steady);last_data_check=steady
    try:
        source_stream=(output/'source_identity_polls.jsonl').open('x')
        guard=deadline.arm(output,45.)  # Same e850 bytes: launcher45s +15s grace => recorder fallback SIGINT by60s.
        argv=['/opt/ros/jazzy/bin/ros2','bag','record','-a','--include-hidden-topics','--node-name','readiness_raw_'+release['release_id'],
            '-s','mcap','--storage-preset-profile','zstd_fast','--max-cache-size','67108864','-o',str(output/'bag')]
        command,env=guard.owned_command('recorder',argv,dict(os.environ));recorder=child(command,output/'recorder.log',env,check=watch)
        guard.wait_for_child('recorder',recorder['process'].pid)
        registration=c.read(output/'recording_deadline_recorder_registration.json')
        if deadline.process_identity(recorder['process'].pid)!=registration['identity']:raise c.Hold('owned recorder registration/exec identity differs')
        until=time.monotonic()+6.;opened=[]
        while time.monotonic()<until:
            watch()
            for path in (output/'bag').glob('*.mcap'):
                with path.open('rb') as stream:
                    if stream.read(8)==bag_summary.MAGIC:opened.append(str(path))
            if opened:break
            time.sleep(.02)
        if not opened:raise c.Hold('MCAP magic not opened before observer')
        deadline.atomic_json(output/'recorder_ready.json',dict(registration=registration,mcap_paths=opened,deadline_ack_verified=True))
        observer=Observer(output,c.read(Path(config['runtime_config_artifact']['path'])),release['release_id'])
        bounded_discovery(observer,watch)
        observer.validator.measure_start=time.monotonic_ns();last_data_check=None;last_graph=time.monotonic_ns()
        result['window_start_unix_ns']=time.time_ns()
        while time.monotonic_ns()-observer.validator.measure_start<10_000_000_000:
            watch(True)
            if time.monotonic_ns()-last_graph>=250_000_000:
                observer.validator.graph(observer.graph());last_graph=time.monotonic_ns()
        observer.validator.measure_end=observer.validator.measure_start+10_000_000_000
        drain_until=time.monotonic_ns()+350_000_000
        while time.monotonic_ns()<drain_until:watch(True)
        observer.validator.graph(observer.graph());c.source_live(config);c.artifacts(config)
        result['validation']=observer.validator.finish(time.time_ns(),time.monotonic_ns())
        if not result['validation']['complete']:raise c.Hold('core-window header/payload correspondences incomplete')
        result.update(state='READ_ONLY_DATA_WINDOW_VALIDATED_PENDING_BAG_CLOSE',window_end_unix_ns=time.time_ns(),continuous_observed_sec=10.)
    except BaseException as exc:result.update(state='HOLD_READINESS_FAILED',error_type=type(exc).__name__,error=str(exc))
    finally:
        if observer is not None:
            if 'validation' not in result:
                try:result['validation']=observer.validator.finish(time.time_ns(),time.monotonic_ns())
                except Exception as exc:result['validation_error']=str(exc)
            try:observer.close()
            except Exception as exc:result['observer_close_error']=str(exc)
        if source_stream is not None:source_stream.flush();source_stream.close()
        if guard is not None:
            try:guard.begin_shutdown(result['state'])
            except Exception as exc:result['helper_shutdown_error']=str(exc)
        if recorder is not None:
            try:
                plan=guard.children['recorder'];interrupt(recorder,[plan['wrapper_argv'],plan['executed_argv']])
            except Exception as exc:result['recorder_close_error']=str(exc)
            finally:
                try:recorder['process'].wait(timeout=15.)
                except BaseException as exc:result['recorder_wait_error']=str(exc)
            result.update(recorder_sigint_sent=recorder['signalled'],recorder_exited=recorder['process'].poll() is not None)
            os.close(recorder['fd'])
        if guard is not None and (recorder is None or recorder['process'].poll() is not None):
            try:
                guard.recording_closed(result['state']);guard.process.wait(timeout=3.)
            except subprocess.TimeoutExpired:result['helper_exit_pending']=True
            except Exception as exc:result['helper_close_error']=str(exc)
        if recorder is not None and recorder['process'].poll() is not None:
            try:
                from readiness_validator import FRESH
                result['bag_audit']=bag_summary.audit(output/'bag',FRESH+('/tf_static',))
            except Exception as exc:result['bag_audit']=dict(complete=False,error=str(exc))
        try:result['source_identities_at_close']=c.source_live(config);result['actual_artifacts_at_close']=c.artifacts(config)
        except Exception as exc:result['source_at_close_error']=str(exc)
        passed=(result['state']=='READ_ONLY_DATA_WINDOW_VALIDATED_PENDING_BAG_CLOSE' and result.get('validation',{}).get('complete')
            and result.get('recorder_exited') and result.get('bag_audit',{}).get('complete')
            and not any(k in result for k in ['observer_close_error','helper_shutdown_error','recorder_close_error','recorder_wait_error','helper_exit_pending','helper_close_error','source_at_close_error']))
        result.update(state='READ_ONLY_READY_OBSERVED_NO_MOTION_AUTHORITY' if passed else 'HOLD_READINESS_FAILED',readiness_observed=bool(passed),finished_unix_ns=time.time_ns(),
            source_package_sha256=release['package_sha256'],graph_process_binding_basis='Root expected source/node naming review plus co-observed exact process identities/node names/graph endpoint GIDs; ROS graph itself does not expose OS PIDs.',
            per_message_publisher_GID_available=False,static_edge_per_message_GID_attribution=False,
            publisher_evidence_basis='Actual Jazzy7.1.12 callback timestamp/sequence dict + matched publisher counts + pinned graph endpoint GIDs. Static two-writer edge ownership is source/config based, not direct callback GID attribution.')
        deadline.atomic_json(output/'readiness_report.json',result)
        for n,handler in previous.items():signal.signal(n,handler)
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execute',action='store_true')
    parser.add_argument('--config',type=Path,default=c.HERE/'config.HOLD.json');parser.add_argument('--root-release',type=Path);parser.add_argument('--output',type=Path);args=parser.parse_args()
    if not args.execute:
        print(json.dumps(dict(state='ROOT_HOLD',scope=c.SCOPE,no_ROS_Node_Popen_signal=True,ready_for_motion=False,package_sha256=c.package_hashes())));return 0
    if args.root_release is None or args.output is None:parser.error('explicit root read-only release and unique output required')
    return 0 if run(args.config,args.root_release,args.output)['readiness_observed'] else 2

if __name__=='__main__':raise SystemExit(main())
