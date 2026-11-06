"""Pure guards for the two fixed, one-second SIM lateral probes only."""
import hashlib
import json
import math
import os
from pathlib import Path
import time

HERE=Path(__file__).resolve().parent
SCOPE='MAPPING_LATERAL_DIAGNOSTIC_ONLY'
TOOL_SHA='5a31cd800317f9d280a453424143312584332caf7f38da5144b0730e25284eb3'
HELPER_SHA='e850bc78bfe46d0678cfd1984995c688bf4f7ad76c1b806ccfb36c4bc944b80c'
PACKAGE=('lateral_once.py','lateral_contract.py','official_tool_timed.py','host_preflight.py',
         'matrix_mc_udp_test.py','recording_deadline.py','bag_summary.py')
TOPICS=('/odom','/front_lidar','/front_lidar/imu','/front_camera/image/compressed',
        '/camera/front/image/compressed','/camera/front/camera_info')

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

def validate(case,release,case_path,output):
    if case.get('scope')!=SCOPE or case.get('executable') is not True or release.get('scope')!=SCOPE or release.get('allow_execute') is not True:raise Hold('ROOT_HOLD: no executable diagnostic release')
    rid=release.get('release_id','')
    if len(rid)!=32 or any(x not in '0123456789abcdef' for x in rid):raise Hold('unique release UUID required')
    if release.get('case_sha256')!=sha(case_path) or release.get('output_directory')!=str(output):raise Hold('case/output binding differs')
    if release.get('package_sha256')!=package_hashes() or sha(HERE/'matrix_mc_udp_test.py')!=TOOL_SHA or sha(HERE/'recording_deadline.py')!=HELPER_SHA:raise Hold('frozen source differs')
    if (release.get('uid')!=os.getuid() or os.getuid()!=1000 or release.get('boot_id')!=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        or release.get('network_namespace')!=os.readlink('/proc/self/ns/net')
        or release.get('controller_pid_namespace')!=os.readlink('/proc/self/ns/pid')):raise Hold('same-host/container identity differs')
    if not release.get('root_unique_velocity_owner_approved') or not release.get('root_static_patch_and_engineering_stop_reserve_approved'):raise Hold('root unique-owner/patch engineering approval missing')
    if not case.get('root_patch_approved') or not case.get('coordinate_verified_by_root'):raise Hold('patch/coordinate root audit pending')
    if case.get('probes')!=[.2,-.2] or any(case.get(k)!=v for k,v in [('max_runtime_sec',60.),('duration_sec',1.),('between_sec',3.),('tail_sec',5.),('body_half_m',.74),('padding_m',.15),('cell_guard_m',.05),('lateral_abort_m',.8),('forward_drift_abort_m',.1),('yaw_abort_rad',.08),('speed_abort_mps',1.),('stop_reserve_m',1.)]):raise Hold('fixed once-probe case was changed')
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
    ordinary={'LX','LY','RX','RY','LT','RT','A','B','X','Y','LB','RB','VIEW','MENU','GUIDE','LS','RS'}
    if set(command)!=ordinary or not finite(command.values()):raise Hold('unexpected UDP fields')
    if any(command[k]!=0 for k in ordinary-{'LX','LB','B'}):raise Hold('forward/yaw/stand/other control forbidden')
    if command['LX'] not in (0.,-sign) or (command['LB'],command['B']) not in ((0,0),(1,1)) or command['LX'] and (command['LB'] or command['B']):raise Hold('only this lateral sign, neutral and official balance allowed')
