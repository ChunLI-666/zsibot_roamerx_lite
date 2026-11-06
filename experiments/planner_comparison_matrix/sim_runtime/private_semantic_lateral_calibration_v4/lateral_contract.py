"""Pure guards for the two fixed, one-second SIM lateral probes only."""
import hashlib
import json
import math
import os
from pathlib import Path
import time

HERE=Path(__file__).resolve().parent
SCOPE='MAPPING_SEMANTIC_LATERAL_BOUNDED_AMPLITUDE_SUSTAINED_CALIBRATION_ONLY'
TOOL_FILENAME='semantic_tool_timed.py'
HELPER_SHA='e850bc78bfe46d0678cfd1984995c688bf4f7ad76c1b806ccfb36c4bc944b80c'
PACKAGE=('lateral_once.py','lateral_contract.py',TOOL_FILENAME,'host_preflight.py',
         'calibration_observer.py','imu_gate.py','recording_deadline.py','bag_summary.py')
TOPICS=('/odom','/front_lidar','/front_lidar/imu','/front_camera/image/compressed',
        '/camera/front/image/compressed','/camera/front/camera_info')
ALLOWED_MAGNITUDES=(.2,.4,.6)
LEGACY_ENDPOINT=('127.0.0.1',7447)
PRIVATE_ENDPOINT=('192.168.1.219',7447)
PRIVATE_IMAGE='sha256:1e05a178bb34bb6e391150104b1d2924bb733153b6c9e9d9bae1e55ae882917e'

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
def body_xy(dx,dy,yaw):return (math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy)

def rpy(q):
    if len(q)!=4 or not finite(q) or abs(sum(v*v for v in q)-1)>.01:raise Hold('invalid GT quaternion')
    x,y,z,w=q
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(max(-1.,min(1.,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))

def claim(path,value):
    fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'w') as f:f.write(json.dumps(value)+'\n');f.flush();os.fsync(f.fileno())

def package_hashes():return {name:sha(HERE/name) for name in PACKAGE}

def probe_parameters(case):
    magnitude,duration=case.get('magnitude'),case.get('duration_sec')
    if (type(magnitude) not in (int,float) or not math.isfinite(magnitude) or magnitude not in ALLOWED_MAGNITUDES
        or type(duration) not in (int,float) or not math.isfinite(duration) or not 1.5<=duration<=4.
        or case.get('probes')!=[magnitude,-magnitude]):raise Hold('only paired magnitude0.2/0.4/0.6 and finite duration1.5..4s')
    return magnitude,duration

def private_network_mode(value):
    flag=value.get('private_network_endpoint',False)
    if type(flag) is not bool:raise Hold('private endpoint flag must be strict boolean')
    return flag

def normalized_endpoint(value):
    if (not isinstance(value,(list,tuple)) or len(value)!=2 or type(value[0]) is not str
        or type(value[1]) is not int or tuple(value) not in (LEGACY_ENDPOINT,PRIVATE_ENDPOINT)):
        raise Hold('only exact reviewed IPv4:7447 endpoint')
    return tuple(value)

def command_endpoint(case,release=None):
    private=private_network_mode(case);expected=PRIVATE_ENDPOINT if private else LEGACY_ENDPOINT
    target=normalized_endpoint(case.get('command_endpoint',expected))
    if target!=expected or private and 'command_endpoint' not in case:raise Hold('case endpoint/topology differs')
    if release is not None:
        allow=release.get('allow_private_network_endpoint',False)
        if type(allow) is not bool or allow!=private:raise Hold('Root private endpoint authorization differs')
        if normalized_endpoint(release.get('command_endpoint',expected))!=target or private and 'command_endpoint' not in release:
            raise Hold('Root selected endpoint differs from case')
    return target

def container_attestation(value,container_id=None):
    item=value['container_attestation_artifact'];path=Path(item['path'])
    if not path.is_absolute() or sha(path)!=item['sha256']:raise Hold('private container attestation hash/path differs')
    a=read(path);cid=value.get('container_id') if container_id is None else container_id
    if (not isinstance(cid,str) or len(cid)!=64 or any(x not in '0123456789abcdef' for x in cid)
        or a.get('Id')!=cid or a.get('Image')!=PRIVATE_IMAGE or not a.get('pidns') or not a.get('netns')):
        raise Hold('private container ID/image/namespace attestation differs')
    return a,item['sha256']

def core_identity(value):
    keys={'pid','starttime_ticks','cmdline_sha256','argv','boot_id'}
    return (isinstance(value,dict) and keys<=set(value) and type(value['pid']) is int and value['pid']>0
        and type(value['starttime_ticks']) is int and value['starttime_ticks']>0
        and isinstance(value['cmdline_sha256'],str) and len(value['cmdline_sha256'])==64
        and all(x in '0123456789abcdef' for x in value['cmdline_sha256'])
        and isinstance(value['argv'],list) and bool(value['argv']) and all(isinstance(x,str) for x in value['argv'])
        and isinstance(value['boot_id'],str) and bool(value['boot_id']))

def host_proof(release,case,now_ns=None):
    target=command_endpoint(case,release)
    item=release['host_preflight'];path=Path(item['path'])
    if item.get('mode')!='root_external_host_attestation' or not path.is_absolute() or sha(path)!=item.get('sha256'):raise Hold('external host proof hash/path differs')
    proof=read(path);now_ns=time.time_ns() if now_ns is None else now_ns
    age=(now_ns-proof['completed_unix_ns'])/1e9
    if not finite([item.get('max_age_sec')]) or not 0<item['max_age_sec']<=30. or not -.05<=age<=item['max_age_sec']:raise Hold('external host proof too old/future')
    if private_network_mode(case):
        a,digest=container_attestation(case)
        sources=case.get('source_identities',{})
        if (not core_identity(case['scene_instance_id']) or set(sources)!={'new_adapter','bridge','projector'}
            or not all(core_identity(v) and set(v)=={'pid','starttime_ticks','cmdline_sha256','argv','boot_id'} for v in sources.values())
            or proof.get('collector_sha256')!=sha(HERE/'host_preflight.py')
            or proof.get('state')!='PASS' or proof.get('scene_identity')!=case['scene_instance_id']
            or proof.get('boot_id')!=release['boot_id'] or proof.get('uid')!=1000
            or proof.get('container_id')!=case['container_id'] or proof.get('container_image')!=PRIVATE_IMAGE
            or proof.get('container_pid_namespace')!=release['controller_pid_namespace'] or a['pidns']!=release['controller_pid_namespace']
            or proof.get('network_namespace')!=release['network_namespace'] or proof.get('container_network_namespace')!=release['network_namespace']
            or a['netns']!=release['network_namespace'] or not proof.get('host_network_namespace')
            or proof['host_network_namespace']==proof['container_network_namespace'] or proof.get('shared_host_network') is not False
            or proof.get('private_bridge_verified') is not True or proof.get('command_endpoint')!=list(target)
            or proof.get('container_attestation_sha256')!=digest or proof.get('container_sources')!=sources
            or proof.get('port7447_exact_scene_owner') is not True or proof.get('retired_workers_verified') is not True
            or proof.get('forbidden_live_processes')!=[] or not proof.get('artifacts_verified')):
            raise Hold('external private host/container/endpoint/source binding incomplete')
        return proof
    if (proof.get('state')!='PASS' or proof['scene_identity']!=case['scene_instance_id']
        or proof['boot_id']!=release['boot_id'] or proof['uid']!=1000
        or proof['network_namespace']!=release['network_namespace']
        or proof['container_pid_namespace']!=release['controller_pid_namespace']
        or proof['container_id']!=case['container_id'] or proof.get('shared_host_network') is not True
        or proof.get('port7447_exact_scene_owner') is not True or proof.get('retired_workers_verified') is not True
        or proof.get('forbidden_live_processes')!=[] or not proof.get('artifacts_verified')):
        raise Hold('external host scene/port/retirement/artifact binding incomplete')
    return proof

def validate(case,release,case_path,output):
    if case.get('scope')!=SCOPE or case.get('executable') is not True or release.get('scope')!=SCOPE or release.get('allow_execute') is not True:raise Hold('ROOT_HOLD: no executable diagnostic release')
    rid=release.get('release_id','')
    if len(rid)!=32 or any(x not in '0123456789abcdef' for x in rid):raise Hold('unique release UUID required')
    if release.get('case_sha256')!=sha(case_path) or release.get('output_directory')!=str(output):raise Hold('case/output binding differs')
    if release.get('package_sha256')!=package_hashes() or sha(HERE/'recording_deadline.py')!=HELPER_SHA:raise Hold('frozen source differs')
    if (release.get('uid')!=os.getuid() or os.getuid()!=1000 or release.get('boot_id')!=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        or release.get('network_namespace')!=os.readlink('/proc/self/ns/net')
        or release.get('controller_pid_namespace')!=os.readlink('/proc/self/ns/pid')):raise Hold('same-host/container identity differs')
    if not release.get('root_unique_velocity_owner_approved') or not release.get('root_static_patch_and_engineering_stop_reserve_approved'):raise Hold('root unique-owner/patch engineering approval missing')
    if not case.get('root_patch_approved') or not case.get('coordinate_verified_by_root'):raise Hold('patch/coordinate root audit pending')
    probe_parameters(case)
    if any(case.get(k)!=v for k,v in [('max_runtime_sec',60.),('between_sec',3.),('tail_sec',5.),('body_half_m',.74),('padding_m',.15),('cell_guard_m',.05),('lateral_abort_m',.8),('forward_drift_abort_m',.1),('yaw_abort_rad',.08),('speed_abort_mps',1.),('stop_reserve_m',1.)]):raise Hold('fixed once-probe bounds were changed')
    if case['scene_instance_id']['pid']!=1172665 or case['scene_instance_id']['starttime_ticks']!=9427026:raise Hold('this case is only for the frozen new UE instance')
    if any(os.environ.get(k)!=v for k,v in case['expected_environment'].items()):raise Hold('ROS source environment differs')
    for key in ['sensor_config_artifact','coordinate_evidence_artifact']:
        item=case[key]
        if not item.get('path') or not Path(item['path']).is_absolute() or sha(item['path'])!=item.get('sha256'):raise Hold('sensor/coordinate datafreeze missing')
    host_proof(release,case)

def patch_check(case):
    import numpy as np
    path=Path(case['oracle_path'])
    if sha(path)!=case['oracle_sha256']:raise Hold('original raw oracle differs')
    data=np.load(path,allow_pickle=False);cells=data['cells'];origin=data['origin_xy_m'];res=float(data['resolution_m'])
    if res!=.05 or cells.dtype!=np.uint8 or cells.ndim!=2:raise Hold('unexpected raw grid')
    bb=case['patch_xyxy'];lo=np.floor((np.array(bb[:2])-origin)/res).astype(int);hi=np.ceil((np.array(bb[2:])-origin)/res).astype(int)
    if lo.min()<0 or hi[0]>cells.shape[1] or hi[1]>cells.shape[0] or not (cells[lo[1]:hi[1],lo[0]:hi[0]]==1).all():raise Hold('approved complete patch intersects unknown/occupied/outside')
    x,y,yaw=case['initial_truth_reference'];h=.74+.15
    # Entire global +/-0.8m excursion +1.0m engineering stop reserve;
    # +/-0.1m cross drift, initial anchor uncertainty, yaw envelope and SAT cell guard.
    extent=max(h*(abs(math.cos(a))+abs(math.sin(a))) for a in [yaw-.10,yaw+.10])+.075
    ux,uy=math.cos(yaw),math.sin(yaw);vx,vy=-uy,ux
    ex=abs(ux)*.1+abs(vx)*1.8+extent+case['initial_xy_tolerance_m']
    ey=abs(uy)*.1+abs(vy)*1.8+extent+case['initial_xy_tolerance_m']
    envelope=[x-ex,y-ey,x+ex,y+ey]
    if any([envelope[0]<bb[0],envelope[1]<bb[1],envelope[2]>bb[2],envelope[3]>bb[3]]):raise Hold('complete lateral+stop filled-body envelope exceeds approved patch')
    return dict(complete_patch_cells=int((hi[0]-lo[0])*(hi[1]-lo[1])),all_original_class1=True,
        patch_xyxy=bb,engineering_swept_envelope_xyxy=envelope,body_half_m=.74,padding_m=.15,cell_guard_m=.05,
        no_grid_erosion=True,stop_reserve_m=1.,hard_plant_guarantee=False)

def check_live(samples,anchor,now_ns,steady_ns):
    for topic in TOPICS:
        s=samples.get(topic)
        if not s or not -.05<=(now_ns-s['stamp_ns'])/1e9<=.3 or not 0<=(steady_ns-s['received_steady_ns'])/1e9<=.3 or not s['payload_valid']:raise Hold('stale/future/missing/empty original sensor: '+topic)
    gt=samples['/odom'];p=gt['position'];roll,pitch,yaw=rpy(gt['quaternion'])
    if gt['frame']!='map' or gt['child']!='BASE_LINK' or not finite([*p,*gt['linear'],*gt['angular']]) or p[2]<=.2 or abs(roll)>.25 or abs(pitch)>.25:raise Hold('GT frame/standing/attitude abnormal')
    speed=max(math.sqrt(sum(v*v for v in gt['linear'])),gt.get('position_speed_mps',0.))
    if speed>1.:raise Hold('measured speed exceeds1m/s')
    if anchor is not None:
        dx,dy=body_xy(p[0]-anchor[0],p[1]-anchor[1],anchor[2])
        if abs(dx)>.1 or abs(dy)>.8 or abs(wrap(yaw-anchor[2]))>.08:raise Hold('lateral/cross/yaw excursion limit')
    return gt,(roll,pitch,yaw)

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

def permitted_payload(command,sign):
    if (type(sign) not in (int,float) or not math.isfinite(sign) or abs(sign) not in ALLOWED_MAGNITUDES
        or not isinstance(command,dict) or set(command)!={'version','action','forward','lateral','yaw'}
        or type(command['version']) is not int or command['version']!=1 or command['action']!='walk'
        or not finite([command['forward'],command['lateral'],command['yaw']])
        or command['forward']!=0 or command['yaw']!=0 or command['lateral'] not in (0.,sign)):
        raise Hold('only exact semantic version1/walk forward0/yaw0 and selected lateral sign or zero')
