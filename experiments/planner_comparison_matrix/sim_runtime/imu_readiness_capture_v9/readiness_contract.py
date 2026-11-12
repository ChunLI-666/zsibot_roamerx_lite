"""Read-only SIM readiness release and original age/stop guards."""
import hashlib
import json
import math
import os
from pathlib import Path
import time

HERE=Path(__file__).resolve().parent
SCOPE="READ_ONLY_SIM_IMU_ADAPTER_READINESS_ONLY"
HELPER_SHA="e850bc78bfe46d0678cfd1984995c688bf4f7ad76c1b806ccfb36c4bc944b80c"
PACKAGE=("readiness_capture.py","readiness_contract.py","readiness_observer.py","readiness_validator.py","owned_recorder.py","imu_converter.py","imu_gate.py","recording_deadline.py","bag_summary.py")
class Hold(ValueError):pass

def sha(path):
    with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def read(path):
    if Path(path).stat().st_size>16*1024*1024:raise Hold('JSON too large')
    def unique(pairs):
        out={}
        for key,value in pairs:
            if key in out:raise Hold('duplicate key')
            out[key]=value
        return out
    return json.loads(Path(path).read_text(),object_pairs_hook=unique,
        parse_constant=lambda x:(_ for _ in ()).throw(Hold('nonfinite JSON')))

def finite(values):return all(type(v) in (int,float) and math.isfinite(v) for v in values)

def wrap(x):return math.atan2(math.sin(x),math.cos(x))

def rpy(q):
    if len(q)!=4 or not finite(q) or abs(sum(v*v for v in q)-1)>.01:raise Hold('invalid GT quaternion')
    x,y,z,w=q
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(max(-1.,min(1.,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))

def claim(path,value):
    fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'w') as f:f.write(json.dumps(value)+'\n');f.flush();os.fsync(f.fileno())

def host_proof(release,case,now_ns=None):
    item=release['host_preflight'];path=Path(item['path'])
    if item.get('mode')!='root_external_host_attestation' or not path.is_absolute() or sha(path)!=item.get('sha256'):raise Hold('external host proof hash/path differs')
    proof=read(path);now_ns=time.time_ns() if now_ns is None else now_ns
    age=(now_ns-proof['completed_unix_ns'])/1e9
    if not finite([item.get('max_age_sec')]) or not 0<item['max_age_sec']<=30. or not -.05<=age<=item['max_age_sec']:raise Hold('external host proof too old/future')
    if (proof.get('state')!='PASS' or proof['scene_identity']!=case['scene_instance_id']
        or proof['boot_id']!=release['boot_id'] or proof['uid']!=1000
        or proof['network_namespace']!=release['network_namespace']
        or proof['container_pid_namespace']!=release['controller_pid_namespace']
        or proof['container_id']!=case['container_id'] or proof.get('shared_host_network') is not True
        or proof.get('port7447_exact_scene_owner') is not True or proof.get('retired_workers_verified') is not True
        or proof.get('forbidden_live_processes')!=[] or not proof.get('artifacts_verified')):
        raise Hold('external host scene/port/retirement/artifact binding incomplete')
    return proof

class StableStop:
    def __init__(self):self.start=None;self.stamp=None;self.origin=None
    def update(self,sample):
        stamp=sample['stamp_ns'];steady=sample['received_steady_ns'];yaw=rpy(sample['quaternion'])[2]
        zero=math.sqrt(sum(v*v for v in sample['linear']))<=.03 and math.sqrt(sum(v*v for v in sample['angular']))<=.05 and sample.get('position_speed_mps',0.)<=.03
        if self.stamp is not None and stamp<=self.stamp:raise Hold('nonadvancing original GT stamp')
        if self.stamp is not None and stamp-self.stamp>100_000_000:self.start=None;self.origin=None
        self.stamp=stamp
        if self.origin and (math.dist(sample['position'][:2],self.origin[:2])>.02 or abs(wrap(yaw-self.origin[2]))>.03):zero=False
        if not zero:self.start=None;self.origin=None;return
        if self.start is None:self.start=(stamp,steady);self.origin=(*sample['position'][:2],yaw)
    def ready(self):return self.start is not None and self.stamp-self.start[0]>=300_000_000 and time.monotonic_ns()-self.start[1]>=300_000_000


def package_hashes():return {name:sha(HERE/name) for name in PACKAGE}

def source_live(config):
    import recording_deadline as deadline
    identities=config['source_identities']
    if set(identities)!={'new_adapter','bridge24','projector61'}:raise Hold('exact new adapter/24/61 identities required')
    observed={}
    for role,expected in identities.items():
        if (not isinstance(expected,dict) or set(expected)!={'pid','starttime_ticks','cmdline_sha256','argv','boot_id'}
            or type(expected['pid']) is not int or expected['pid']<=0 or type(expected['starttime_ticks']) is not int
            or not isinstance(expected['argv'],list) or not expected['argv']):raise Hold('complete five-field source identity required')
        observed[role]=deadline.process_identity(expected['pid'])
        if observed[role]!=expected:raise Hold('continuous source identity changed: '+role)
    if len({item['pid'] for item in observed.values()})!=3:raise Hold('source identities must be distinct')
    return observed

def artifacts(config):
    expected={'sim_imu_adapter.py':'8b21f321ed7fe79042d28202ebf1fb4ec848ceacb720a4066fd21dbbba52edc2',
        'adapter_with_gate.py':'be6c4863a0616e2d2b861a9e31e559f0d60f4cde18fa85093042b2198a5df53c',
        'imu_gate.py':'0b451821b94eb8182ff2f8df8d5e7faf359a24486455c90b629478f944023f66',
        'matrix_sensor_contract.py':'3159610e54ae74e50312979103cb37e6634b24a6c47dcf74a32685d130e9dc7a'}
    if set(config['new_adapter_artifacts'])!=set(expected):raise Hold('four reviewed adapter source artifacts required')
    items=[]
    for name,value in expected.items():
        item=config['new_adapter_artifacts'][name]
        if not Path(item['path']).is_absolute() or Path(item['path']).name!=name or item['sha256']!=value or sha(item['path'])!=value:raise Hold('actual new adapter source bytes differ: '+name)
        items.append(item)
    item=config['runtime_config_artifact']
    if not Path(item['path']).is_absolute() or item['sha256']!='38aab54b3a6ca49570feaaa8e983f4f9bc9ee197060066b186478a70f92ea023' or sha(item['path'])!=item['sha256']:raise Hold('actual config38a differs')
    return items+[item]

def validate(config,release,config_path,output):
    if (config.get('scope')!=SCOPE or release.get('scope')!=SCOPE or config.get('executable') is not True
        or release.get('allow_execute') is not True or release.get('root_read_only_capture_approved') is not True):raise Hold('ROOT_HOLD: read-only capture release absent')
    rid=release.get('release_id','')
    if len(rid)!=32 or any(v not in '0123456789abcdef' for v in rid):raise Hold('unique read-only release UUID required')
    if release.get('config_sha256')!=sha(config_path) or release.get('output_directory')!=str(output):raise Hold('config/output binding differs')
    if release.get('package_sha256')!=package_hashes() or sha(HERE/'recording_deadline.py')!=HELPER_SHA:raise Hold('readiness source freeze differs')
    if (os.getuid()!=1000 or release.get('uid')!=1000 or release.get('boot_id')!=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        or release.get('network_namespace')!=os.readlink('/proc/self/ns/net') or release.get('controller_pid_namespace')!=os.readlink('/proc/self/ns/pid')):raise Hold('same-container UID/boot/namespaces differ')
    if any(os.environ.get(k)!=v for k,v in config['expected_environment'].items()):raise Hold('ROS source environment differs')
    if (config.get('duration_sec')!=10. or config.get('discovery_sec')!=10. or config.get('max_runtime_sec')!=60. or config.get('launcher_deadline_sec')!=45.
        or config.get('root_expected_source_node_names_reviewed') is not True):raise Hold('fixed read-only timing/root expected source/node naming review required')
    host_proof(release,config);source_live(config);artifacts(config)
