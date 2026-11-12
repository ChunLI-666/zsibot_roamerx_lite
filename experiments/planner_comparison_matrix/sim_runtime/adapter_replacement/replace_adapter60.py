#!/usr/bin/python3
"""Default-HOLD exact old60 SIGINT then one new source start; no ROS client."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import time

HERE=Path(__file__).resolve().parent
SCOPE='SIM_IMU_ADAPTER60_REPLACEMENT_ONLY'
KEYS=('pid','starttime_ticks','cmdline_sha256','argv','boot_id')
SOURCE_NAMES={'sim_imu_adapter.py','adapter_with_gate.py','imu_gate.py','matrix_sensor_contract.py'}

class Hold(ValueError):pass

def read(path):return json.loads(Path(path).read_text())
def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()
def five(value):return {k:value[k] for k in KEYS}
def claim(path,value):
    fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'w') as stream:stream.write(json.dumps(value)+'\n');stream.flush();os.fsync(stream.fileno())
def write(path,value):
    temporary=path.with_suffix(path.suffix+'.tmp');temporary.write_text(json.dumps(value,indent=2)+'\n');os.replace(temporary,path)

def identity(pid):
    p=Path('/proc')/str(pid);fields=(p/'stat').read_text().rsplit(')',1)[1].split();raw=(p/'cmdline').read_bytes()
    status=(p/'status').read_text().splitlines()
    def values(key):return [int(v) for v in next(x for x in status if x.startswith(key+':')).split()[1:]]
    if fields[0]=='Z' or not raw:raise ProcessLookupError('retired')
    return dict(pid=pid,starttime_ticks=int(fields[19]),cmdline_sha256=hashlib.sha256(raw).hexdigest(),
        argv=[v.decode(errors='backslashreplace') for v in raw.rstrip(b'\0').split(b'\0')],
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),uid=values('Uid'),gid=values('Gid'),
        namespace_pids=values('NSpid'),pid_namespace=os.readlink(p/'ns/pid'),mount_namespace=os.readlink(p/'ns/mnt'),
        executable=os.readlink(p/'exe'),cwd=os.readlink(p/'cwd'),cgroup_sha256=hashlib.sha256((p/'cgroup').read_bytes()).hexdigest())

def exact(expected):
    actual=identity(expected['pid'])
    if any(actual.get(k)!=v for k,v in expected.items()):raise Hold('complete process identity differs: '+str(expected['pid']))
    return actual

def retirement(expected):
    p=Path('/proc')/str(expected['pid'])
    try:fields=(p/'stat').read_text().rsplit(')',1)[1].split()
    except FileNotFoundError:return 'ABSENT'
    if int(fields[19])!=expected['starttime_ticks']:raise Hold('old numeric PID reused; do not authorize it')
    return 'Z' if fields[0]=='Z' else 'LIVE'

def retained(plan):
    for key in ['24','61']:exact(plan['retained_sources'][key])

def host_proof(plan,release):
    spec=release['host_preflight'];path=Path(spec['path'])
    if spec.get('mode')!='root_external_host_attestation' or not path.is_absolute() or sha(path)!=spec.get('sha256'):raise Hold('host proof binding differs')
    proof=read(path);age=(time.time_ns()-proof['completed_unix_ns'])/1e9
    if not 0<spec['max_age_sec']<=30 or not -.05<=age<=spec['max_age_sec']:raise Hold('host proof too old/future')
    if (proof.get('state')!='PASS' or proof['scene_identity']!=plan['scene_instance_id']
        or proof['boot_id']!=plan['boot_id'] or proof['uid']!=1000 or proof['container_id']!=plan['container_id']
        or proof['container_pid_namespace']!=plan['pid_namespace'] or proof['network_namespace']!=plan['network_namespace']
        or proof.get('shared_host_network') is not True or proof.get('port7447_exact_scene_owner') is not True
        or proof.get('forbidden_live_processes')!=[]):raise Hold('exact newUE/container/7447 host proof incomplete')

def validate(plan,release,plan_path,output):
    if plan.get('scope')!=SCOPE or plan.get('executable') is not True or release.get('scope')!=SCOPE or release.get('allow_execute') is not True:raise Hold('ROOT_HOLD')
    rid=release.get('release_id','')
    if len(rid)!=32 or any(c not in '0123456789abcdef' for c in rid):raise Hold('unique replacement release required')
    if (release.get('plan_sha256')!=sha(plan_path) or release.get('driver_sha256')!=sha(__file__)
        or release.get('output_directory')!=str(output)):raise Hold('driver/plan/output binding differs')
    if not release.get('sole_adapter_replacement_root_approved') or not release.get('no_Nav_native_command_recorder_active_root_verified'):raise Hold('root sole-source/quiescent plan review missing')
    if (os.getuid()!=1000 or plan['uid']!=1000 or plan['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        or plan['pid_namespace']!=os.readlink('/proc/self/ns/pid') or plan['network_namespace']!=os.readlink('/proc/self/ns/net')
        or any(os.environ.get(k)!=v for k,v in plan['expected_environment'].items())):raise Hold('same Jazzy container185 identity/env differs')
    old=plan['old_adapter']
    if old['pid']!=60 or old['starttime_ticks']!=4460360 or old['cmdline_sha256']!='2182d0da6ae7cd136dfddd47ae1a80e8d514795d2561cd08a796d2eea46705e9':raise Hold('only original exact60 eligible')
    if set(plan['retained_sources'])!={'24','61'} or plan['scene_instance_id']['pid']!=1172665 or plan['scene_instance_id']['starttime_ticks']!=9427026:raise Hold('retained sources/newUE differs')
    if plan['old_exit_timeout_sec']!=15. or plan['new_identity_timeout_sec']!=5.:raise Hold('fixed bounded operation required')
    if sha(plan['runtime_config'])!=plan['runtime_config_sha256'] or plan['runtime_config_sha256']!='38aab54b3a6ca49570feaaa8e983f4f9bc9ee197060066b186478a70f92ea023':raise Hold('original config changed')
    for name,digest in plan['old_sources_sha256'].items():
        if sha(Path(old['argv'][1]).parent/name)!=digest:raise Hold('old rollback source bytes changed')
    if set(plan['new_sources_sha256'])!=SOURCE_NAMES:raise Hold('exact four new source files required')
    for name,digest in plan['new_sources_sha256'].items():
        if sha(Path(plan['source_directory'])/name)!=digest:raise Hold('reviewed new source differs')
    review=Path(release['source_review_path'])
    if not review.is_absolute() or sha(review)!=release['source_review_sha256']:raise Hold('independent source review receipt missing/changed')
    inner=Path(release['adapter_release_path'])
    if not inner.is_absolute() or sha(inner)!=release['adapter_release_sha256']:raise Hold('inner source release differs')
    r=read(inner)
    expected=dict(scope='SIM_ONLY_IMU_MONOTONIC_GATE',allow_execute=True,release_id=release['release_id'],uid=1000,
        boot_id=plan['boot_id'],pid_namespace=plan['pid_namespace'],network_namespace=plan['network_namespace'],
        sources_sha256=plan['new_sources_sha256'],runtime_config_sha256=plan['runtime_config_sha256'],old_adapter60_retired=True,
        bridge24_projector61_preserved=True,old_adapter_identity=five(old),retained_source_identities=[five(plan['retained_sources'][k]) for k in ['24','61']])
    if any(r.get(k)!=v for k,v in expected.items()):raise Hold('inner release does not bind this planned replacement')
    host_proof(plan,release);retained(plan)

def run(plan_path,release_path,output):
    plan,release=read(plan_path),read(release_path);validate(plan,release,plan_path,output)
    output.mkdir(parents=True,exist_ok=False);oldfd=newfd=None;process=None
    result=dict(scope=SCOPE,state='HOLD_PREPARATION',old_sigint_sent=False,new_start_attempts=0,ready_for_motion=False,
        signals_to24_61_UE_new_source=0,automatic_rollback_or_retry=False)
    try:
        oldfd=os.pidfd_open(plan['old_adapter']['pid'])
        exact(plan['old_adapter']);retained(plan);host_proof(plan,release)
        claim(release_path.parent/(release['release_id']+'.replace_adapter60_once.json'),dict(plan_sha256=sha(plan_path),release_sha256=sha(release_path),claimed_unix_ns=time.time_ns()))
        write(output/'operation_inputs.json',dict(plan=plan,release=release))
        # Sole signal in this program: retained exact original60 kernel object.
        if select.select([oldfd],[],[],0)[0]:raise Hold('old60 exited before normal signal; fresh root plan required')
        exact(plan['old_adapter']);signal.pidfd_send_signal(oldfd,signal.SIGINT);result['old_sigint_sent']=True
        until=time.monotonic()+15.
        while not select.select([oldfd],[],[],0)[0]:
            retained(plan)
            if time.monotonic()>=until:raise Hold('old60 did not exit after one normal SIGINT; no escalation/start')
            time.sleep(.02)
        state=retirement(plan['old_adapter'])
        if state not in ('Z','ABSENT'):raise Hold('old60 not confirmed Z/absent')
        result['old_retired_state']=state;retained(plan);host_proof(plan,release)
        if sha(plan['runtime_config'])!=plan['runtime_config_sha256']:raise Hold('config changed during replacement')
        for name,digest in plan['new_sources_sha256'].items():
            if sha(Path(plan['source_directory'])/name)!=digest:raise Hold('new source changed before start')
        argv=['/usr/bin/python3','-u',str(Path(plan['source_directory'])/'sim_imu_adapter.py'),'--execute',
              '--runtime-config',plan['runtime_config'],'--root-release',release['adapter_release_path']]
        claim(output/'new_start_once.json',dict(argv=argv,old_retired_state=state,time_ns=time.time_ns()))
        result['new_start_attempts']=1
        with (output/'new_adapter.log').open('x') as stream:
            process=subprocess.Popen(argv,cwd=plan['old_adapter']['cwd'],env=dict(os.environ),stdin=subprocess.DEVNULL,
                stdout=stream,stderr=subprocess.STDOUT,start_new_session=True,close_fds=True)
        result.update(new_source_pid=process.pid,new_source_expected_argv=argv)
        write(output/'new_start_return.json',dict(pid=process.pid,expected_argv=argv,time_ns=time.time_ns()))
        newfd=os.pidfd_open(process.pid);until=time.monotonic()+5.;observed=None
        while time.monotonic()<until:
            retained(plan)
            if process.poll() is not None:raise Hold('new source exited; do not retry or rollback automatically')
            try:observed=identity(process.pid)
            except (FileNotFoundError,ProcessLookupError):observed=None
            if observed and observed['argv']==argv and observed['uid']==[1000]*4:break
            time.sleep(.02)
        if not observed or observed['argv']!=argv:raise Hold('new source full exec identity not ready')
        result.update(state='SOURCE_STARTED_PENDING_ROOT_READINESS',new_source_identity=observed,
            source_process_live=True,readiness_basis='Process identity only; no ROS graph/data/TF readiness claim')
        write(output/'new_source_identity.json',observed)
    except BaseException as exc:result.update(state='HOLD_OPERATION_INCOMPLETE',error_type=type(exc).__name__,error=str(exc))
    finally:
        if process is not None:result['new_returncode']=process.poll()
        for fd in [newfd,oldfd]:
            if fd is not None:os.close(fd)
        # Never stop/restart an actor on failure. Root owns any next exact step.
        result['finished_unix_ns']=time.time_ns();write(output/'result.json',result)
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true')
    p.add_argument('--plan',type=Path,default=HERE/'replacement_plan.HOLD.json');p.add_argument('--root-release',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
    if not a.execute:print(json.dumps(dict(state='ROOT_HOLD',scope=SCOPE,no_signal_Popen_Node_ROS=True)));return 0
    if a.root_release is None or a.output is None:p.error('root release and unique output required')
    r=run(a.plan.resolve(),a.root_release.resolve(),a.output.resolve());print(json.dumps(r));return 0 if r['state']=='SOURCE_STARTED_PENDING_ROOT_READINESS' else 2

if __name__=='__main__':raise SystemExit(main())
