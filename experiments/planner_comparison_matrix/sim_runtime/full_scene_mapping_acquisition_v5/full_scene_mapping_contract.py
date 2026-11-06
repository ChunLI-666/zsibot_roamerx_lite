"""Pure contracts for controlled static SIM raw-data acquisition; never Nav input.

Filled-cell SAT and sweep spacing follow robot_feasible_navigation/footprint_sweep.hpp.
Point hits are occupied observations only. The root-reviewed physical static
oracle supplies free evidence; hit-cloud gaps never do.
"""
from dataclasses import dataclass
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys

MODE='controlled_static_sim_scene'
SCOPE='MAPPING_ACQUISITION_LATERAL_RECT_V5_ONLY'
HALF_BODY=.74
PADDING=.15
MAX_VX=.4
MAX_WZ=.3
PERIOD=.05
CONTROL_RENEWAL_LIMIT=.10
GUARD_COMPUTE_BUDGET=.10
PACKET_AGE=.10
RELAY_WATCHDOG=.15
MAX_INPUT_AGE=.3
MAX_TRUTH_AGE=.05
FUTURE_ALLOWANCE=.05
START_STABLE=10.
END_STABLE=20.
STOP_STABLE=.3
MAX_BUDGET=3600.
CLEANUP_RESERVE=25.
TRUTH_FRAME='map'
TRUTH_CHILD='BASE_LINK'  # Original wire label; never normalized into Nav base_link.
CAPABILITY='semantic_lateral_mag04_rect_v5'
MAX_LATERAL_RAW=.4
ACTUAL_GT_STOP_REFERENCE='original_GT_actual_pose_future_errors_v1'
RECTANGLE_PROOF_MANIFEST_SHA='ebfb6900d7c424b46e9e1e191f774a57427a1c2bcf76d625b31937963a239cc2'
RECTANGLE_PROOF_REPORT_SHA='e18db218e4b4183bafe564530a571b1c939690707f709e9dc8efe81cc787b981'
RECTANGLE_REVIEW_SHA='669efe13a5f038b6d73adc7b9041e88d1c87c8b3336840ed607108acd0879111'
MAX_LATERAL_YAW_BAND=.12
MAX_LATERAL_CORRECTIONS=4
LATERAL_CORRECTION_BUDGET_SEC=20.
FEEDBACK_WINDOW_SEC=1.
FEEDBACK_WINDOW_PROGRESS_M=.20
MIN_CROSS_IMPROVEMENT_M=.001
LENGTH_COMPARISON_EPS_M=1e-9

POINT_FIELDS=[('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('tag',16,2),('line',17,2),('timestamp',18,8)]


class Hold(ValueError):
    pass


def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def read_json(path):
    def unique(pairs):
        result={}
        for key,value in pairs:
            if key in result:raise Hold('duplicate JSON key: '+key)
            result[key]=value
        return result
    def constant(value):raise Hold('nonfinite JSON constant: '+value)
    return json.loads(Path(path).read_text(),object_pairs_hook=unique,parse_constant=constant)


def finite(values):
    return all(type(value) in (int,float) and math.isfinite(value) for value in values)


def wrap(value):return math.atan2(math.sin(value),math.cos(value))


def checked_quaternion(value):
    if len(value)!=4 or not finite(value):raise Hold('nonfinite quaternion')
    if abs(sum(x*x for x in value)-1.)>1e-3:raise Hold('nonunit quaternion; not normalized by this tool')
    return value


def rpy(value):
    x,y,z,w=checked_quaternion(value)
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(max(-1.,min(1.,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))


def rotate(q,p):
    x,y,z,w=checked_quaternion(q);a,b,c=p
    return ((1-2*(y*y+z*z))*a+2*(x*y-z*w)*b+2*(x*z+y*w)*c,
            2*(x*y+z*w)*a+(1-2*(x*x+z*z))*b+2*(y*z-x*w)*c,
            2*(x*z-y*w)*a+2*(y*z+x*w)*b+(1-2*(x*x+y*y))*c)


def mount_quaternion(rpy_rad):
    if len(rpy_rad)!=3 or not finite(rpy_rad):raise Hold('invalid T_body_lidar rotation')
    r,p,y=(value/2 for value in rpy_rad);cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return (sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy)


def pose_at(history,stamp_ns,max_gap_ns=50_000_000):
    """Interpolate bracketing original GT samples, never extrapolate to now()."""
    def checked(value):
        if value.get('frame_id')!=TRUTH_FRAME or value.get('child_frame_id')!=TRUTH_CHILD or len(value.get('xyz',[]))!=3 or not finite(value['xyz']):raise Hold('invalid original GT association frame/position')
        checked_quaternion(value.get('q',[]));return value
    ordered=sorted(history,key=lambda value:value['stamp_ns'])
    exact=next((value for value in ordered if value['stamp_ns']==stamp_ns),None)
    if exact is not None:return dict(checked(exact),association='exact original GT stamp')
    before=[value for value in ordered if value['stamp_ns']<stamp_ns]
    after=[value for value in ordered if value['stamp_ns']>stamp_ns]
    if not before or not after:raise Hold('lidar stamp lacks bracketing original GT poses')
    a,b=checked(before[-1]),checked(after[0]);gap=b['stamp_ns']-a['stamp_ns']
    if not 0<gap<=max_gap_ns:raise Hold('GT interpolation gap too large')
    t=(stamp_ns-a['stamp_ns'])/gap;qa=list(checked_quaternion(a['q']));qb=list(checked_quaternion(b['q']))
    dot=sum(x*y for x,y in zip(qa,qb))
    if dot<0:qb=[-x for x in qb];dot=-dot
    if dot>.9995:
        q=[x+t*(y-x) for x,y in zip(qa,qb)];length=math.sqrt(sum(x*x for x in q));q=[x/length for x in q]
    else:
        angle=math.acos(max(-1.,min(1.,dot)));den=math.sin(angle)
        q=[(math.sin((1-t)*angle)*x+math.sin(t*angle)*y)/den for x,y in zip(qa,qb)]
    xyz=[x+t*(y-x) for x,y in zip(a['xyz'],b['xyz'])]
    return dict(stamp_ns=stamp_ns,xyz=xyz,q=q,frame_id=a['frame_id'],child_frame_id=a['child_frame_id'],
                association='interpolated GT at original lidar stamp',bracket_stamp_ns=[a['stamp_ns'],b['stamp_ns']])


def fresh(sample,now_ns,steady,limit=MAX_INPUT_AGE):
    if not sample:raise Hold('missing input')
    age=(now_ns-sample['stamp_ns'])/1e9;received=steady-sample['received_steady']
    if not finite([age,received]) or not -FUTURE_ALLOWANCE<=age<=limit or not 0<=received<=limit:
        raise Hold('stale/future input header or callback receipt')
    return dict(header_age_s=age,callback_age_s=received)


def check_clock(previous,unix_ns,steady):
    if previous is not None and abs((unix_ns-previous[0])/1e9-(steady-previous[1]))>.05:
        raise Hold('wall/source clock step relative to steady clock')
    return unix_ns,steady


def truth_age_limit(model):
    value=model.get('max_truth_age_sec')
    if not finite([value]) or not 0<value<=MAX_TRUTH_AGE:raise Hold('root GT age cap must be finite in(0,.05]')
    return value


def command_age_upper(header_age_s,truth_age_cap=MAX_TRUTH_AGE):
    if not finite([header_age_s,truth_age_cap]) or not 0<truth_age_cap<=MAX_TRUTH_AGE or not -FUTURE_ALLOWANCE<=header_age_s<=truth_age_cap:raise Hold('invalid initial command pose age/cap')
    return min(truth_age_cap,header_age_s+GUARD_COMPUTE_BUDGET)


def command_time_pose_guard(truth,issued_ns,steady,guard_started,age_upper,truth_age_cap=MAX_TRUTH_AGE):
    if not finite([truth_age_cap]) or not 0<truth_age_cap<=MAX_TRUTH_AGE or age_upper>truth_age_cap:raise Hold('command age cap differs from swept truth cap')
    age=fresh(truth,issued_ns,steady,limit=truth_age_cap)
    if not finite([guard_started,age_upper]) or not 0<=steady-guard_started<=GUARD_COMPUTE_BUDGET:raise Hold('command guard computation exceeded its swept age budget')
    if age['header_age_s']>age_upper+1e-9:raise Hold('command-time pose age exceeds swept uncertainty')
    return dict(**age,guard_elapsed_s=steady-guard_started,swept_pose_age_upper_s=age_upper)


def valid_scene_instance(value):
    return isinstance(value,dict) and type(value.get('pid')) is int and value['pid']>0 and type(value.get('starttime_ticks')) is int and value['starttime_ticks']>0 and isinstance(value.get('cmdline_sha256'),str) and len(value['cmdline_sha256'])==64 and bool(value.get('boot_id'))


def valid_local_identity(value):
    return valid_scene_instance(value) and set(value)=={'pid','starttime_ticks','cmdline_sha256','argv','boot_id'} and isinstance(value['argv'],list) and bool(value['argv']) and all(isinstance(item,str) for item in value['argv'])


def validate_route(value,execute=False):
    if type(value.get('schema_version')) is not int or value.get('schema_version')!=1 or value.get('scope')!=SCOPE or value.get('mode')!=MODE:raise Hold('wrong mapping-only route scope/mode')
    if not valid_scene_instance(value.get('scene_instance_id')):raise Hold('root scene process identity binding required')
    if execute and value.get('executable') is not True:raise Hold('route remains non-executable')
    budget=value.get('max_runtime_sec')
    if not finite([budget]) or not START_STABLE+END_STABLE+CLEANUP_RESERVE<budget<=MAX_BUDGET:raise Hold('finite runtime budget in (55,3600] including cleanup is required')
    if value.get('truth_frame_id')!=TRUTH_FRAME or value.get('truth_child_frame_id')!=TRUTH_CHILD:raise Hold('unsupported original GT frames; wire child is BASE_LINK')
    start=value.get('start_anchor_truth')
    if not isinstance(start,list) or len(start)!=3 or not finite(start):raise Hold('root must freeze original truth start anchor')
    entries=value.get('waypoints')
    if not isinstance(entries,list) or not 1<=len(entries)<=4096:raise Hold('root-reviewed route required; no invented exploration route')
    if value.get('lateral_capability')!=CAPABILITY:raise Hold('explicit lateral mapping capability required')
    ids=set()
    for entry in entries:
        pose=entry.get('pose_truth');name=entry.get('id')
        if not isinstance(name,str) or not name or name in ids:raise Hold('route IDs must be unique')
        if not isinstance(pose,list) or len(pose)!=3 or not finite(pose):raise Hold('nonfinite truth route pose')
        if entry.get('require_stopped') is not True:raise Hold('all acquisition points require a measured stop')
        policy=entry.get('leg_policy')
        if policy is not None:
            forward_terminal_policy(start if not ids else entries[len(ids)-1]['pose_truth'],pose,policy)
            if isinstance(policy,dict) and policy.get('kind')=='lateral_fixed_body_heading':
                previous=start if not ids else entries[len(ids)-1]['pose_truth']
                lateral_leg_state(previous,pose,(*previous[:2],policy['body_yaw_rad']),policy)
                if execute and policy.get('root_tracking_approved') is not True:raise Hold('lateral endpoint tracking/minimum-leg approval pending')
            else:
                if not isinstance(policy,dict) or policy.get('kind')!='straight_fixed_heading':raise Hold('unsupported acquisition leg policy')
                if not finite([policy.get(key) for key in ['travel_yaw_rad','max_yaw_error_rad','max_cross_track_m']]):raise Hold('finite fixed-heading/tube bounds required')
                if not 0<policy['max_yaw_error_rad']<=.08 or not 0<policy['max_cross_track_m']<=.15:raise Hold('fixed-heading/tube bounds exceed supported acquisition tolerance')
                previous=start if not ids else entries[len(ids)-1]['pose_truth']
                forward_endpoint_state(previous,pose,previous,policy)
        ids.add(name)
    return value


def validate_model(value):
    truth_age_limit(value)
    fields=['max_truth_speed_mps','max_truth_yaw_rate_rps','stop_distance_reserve_m','stop_angle_reserve_rad',
            'forward_drift_reserve_m','turn_drift_reserve_m','forward_yaw_reserve_rad',
            'max_roll_pitch_rad','body_z_min_m','body_z_max_m','floor_z_m','floor_hit_tolerance_m','obstacle_top_m']
    if not finite([value.get(key) for key in fields]):raise Hold('root must freeze finite engineering motion/height bounds')
    if not MAX_VX<=value['max_truth_speed_mps']<=1. or not MAX_WZ<=value['max_truth_yaw_rate_rps']<=1.:raise Hold('truth speed bound unsupported')
    if not .1<=value['stop_distance_reserve_m']<=5. or not .05<=value['stop_angle_reserve_rad']<=math.pi:raise Hold('stop reserve unsupported')
    if not all(0<=value[key]<=.5 for key in ['forward_drift_reserve_m','turn_drift_reserve_m','forward_yaw_reserve_rad']):raise Hold('drift reserve unsupported')
    if not 0<value['max_roll_pitch_rad']<=.3 or value['body_z_max_m']<=value['body_z_min_m']:raise Hold('invalid attitude/height bounds')
    if not 0<value['floor_hit_tolerance_m']<=.05 or value['obstacle_top_m']<=value['floor_z_m']:raise Hold('invalid flat-floor/obstacle height interval')
    if value.get('basis')!='root_reviewed_engineering_bound_sim_only':raise Hold('bounds must be explicitly simulation engineering assumptions')
    validate_lateral_models(value)
    if value.get('future_error_reference')!=ACTUAL_GT_STOP_REFERENCE or value.get('independent_future_errors_reviewed') is not True:
        raise Hold('root independent future-error reference/model approval pending')
    return value


def footprint_spec(value,path):
    """Runtime shape selection is stricter than generic geometry construction."""
    mode=value.get('footprint_mode','legacy_square');half=value.get('body_half',value.get('body_half_xy_m'))
    if value.get('padding',value.get('padding_m'))!=PADDING or value.get('footprint_inflation_in_grid_m')!=0.:
        raise Hold('independent .15 padding and un-eroded oracle required')
    model=value.get('motion_model',{})
    if ('footprint_mode' in model and model['footprint_mode']!=mode
        or 'body_half_xy_m' in model and model['body_half_xy_m']!=half
        or 'padding_m' in model and model['padding_m']!=PADDING):raise Hold('motion model and physical oracle footprint disagree')
    if mode=='legacy_square':
        if half!=[HALF_BODY,HALF_BODY]:raise Hold('legacy mode cannot shrink the .74square')
        return dict(mode=mode,body_half_xy_m=half,padding_m=PADDING,proof_sha256=None,certified_roll_pitch_cap_rad=None)
    if mode!='certified_full_body_rectangle' or not isinstance(half,list) or len(half)!=2 or not finite(half):raise Hold('new rectangle half widths/proof remain ROOT HOLD')
    evidence=value.get('body_envelope_proof',{});artifacts=value.get('artifacts_sha256',{});root=Path(path).parent
    def bound_file(key,expected=None):
        ref=evidence.get(key,{})
        file=Path(ref.get('path',''));file=file if file.is_absolute() else root/file
        digest=ref.get('sha256')
        if not file.is_file() or sha(file)!=digest or expected is not None and digest!=expected:raise Hold('full-body proof artifact absent/changed: '+key)
        if not any((Path(k) if Path(k).is_absolute() else root/k).resolve()==file.resolve() and v==digest for k,v in artifacts.items()):raise Hold('full-body proof artifact missing from complete oracle inventory: '+key)
        return read_json(file),digest,file
    manifest,manifest_sha,manifest_path=bound_file('manifest',RECTANGLE_PROOF_MANIFEST_SHA)
    report,report_sha,report_path=bound_file('report',RECTANGLE_PROOF_REPORT_SHA)
    review,review_sha,review_path=bound_file('independent_review',RECTANGLE_REVIEW_SHA)
    if (review.get('schema')!='locnav.full_body_rectangle.independent_review.v1'
        or review.get('decision')!='MATH_SOURCE_GO_CONDITIONAL_FULL_BODY_RECTANGLE_BOUND' or review.get('blocking_findings')!=[]
        or review.get('reviewed_manifest_sha256')!=manifest_sha or review.get('proof_sha256')!=report_sha
        or review.get('runtime_release_authorized') is not False):raise Hold('independent full-geometry SOURCE_GO receipt is missing or unbound')
    if manifest.get('authoritative_files',{}).get('unrestricted_axis_proof.json',{}).get('sha256')!=report_sha:raise Hold('full geometry manifest/report mismatch')
    if (report.get('schema')!='locnav.full_xml_arbitrary_hinge_axis_enclosure.v1'
        or report.get('includes_all_17_meshes_all_30_geoms') is not True
        or report.get('all_hinge_angles_unrestricted') is not True or report.get('joint_limits_or_margin_used_as_hard_bounds') is not False
        or report.get('body_roll_pitch_condition_rad')!=[.25,.25] or report.get('padding_consumed_by_bound') is not False):
        raise Hold('only the independently reviewed whole-geometry arbitrary-hinge/RP proof may shrink the body')
    for dep in report.get('dependencies_sha256',{}).values():
        asset=Path(dep['path'])
        if not asset.is_file() or sha(asset)!=dep['sha256']:raise Hold('certified full-body model asset missing/changed')
    certified=review['accepted_half_extents_m']
    if certified!=report['strict_upward_millimetre_ceil_half_xy_m'] or review.get('padding_separate') is not True:raise Hold('reviewed rectangle/padding mismatch')
    if any(h<b for h,b in zip(half,certified)) or min(half)<=0:raise Hold('rectangle smaller than certified outward body half bound')
    rp=value.get('motion_model',{}).get('max_roll_pitch_rad')
    if not finite([rp]) or not 0<rp<=.25:raise Hold('rectangular model must bound entire motion/stop RP<=.25')
    return dict(mode=mode,body_half_xy_m=half,padding_m=PADDING,proof_sha256=report_sha,
        proof_manifest_sha256=manifest_sha,independent_review_sha256=review_sha,certified_roll_pitch_cap_rad=.25)


def validate_oracle(value,path):
    if value.get('scope')!=SCOPE or value.get('mode')!=MODE:raise Hold('missing controlled static physical oracle')
    audit=value.get('root_geometry_audit',{})
    if not isinstance(audit,dict) or not all(audit.get(key) is True for key in ['complete','coordinate_verified','static_scene_verified','root_domain_approved']):raise Hold('root physical/floor/coordinate/static audit is pending')
    if value.get('oracle_kind')!='physical_collision_floor_projection_not_slam_map':raise Hold('old/SLAM navigation maps are not physical scene oracles')
    if not valid_scene_instance(value.get('scene_instance_id')):raise Hold('physical oracle scene instance missing')
    height=value.get('floor_height_model',{})
    if value.get('truth_frame_id')!=TRUTH_FRAME or not isinstance(height,dict) or height.get('type')!='flat':raise Hold('unsupported truth/floor model')
    footprint_spec(value,path)
    alias=value.get('frame_alias',{})
    if not isinstance(alias,dict) or alias.get('verified') is not True or alias.get('observed_gt_frame')!=TRUTH_FRAME or alias.get('observed_gt_child_frame')!=TRUTH_CHILD or alias.get('numeric_transform')!='identity_verified_against_raw_scan':raise Hold('frame naming alone is not numeric coordinate alignment')
    if value.get('point_coordinates')!='sensor_local_flu_despite_raw_frame_id_map':raise Hold('raw frame label is not a coordinate transform')
    mount=value.get('T_body_lidar',{})
    if not isinstance(mount.get('xyz'),list) or len(mount['xyz'])!=3 or not finite(mount['xyz']):raise Hold('missing true T_body_lidar translation')
    mount_quaternion(mount.get('rpy_rad',[]))
    if value.get('self_return_policy')!='root_verified_simulator_ignore_mounting_actor_no_geometric_footprint_mask':raise Hold('self exclusion provenance required; no blanket footprint mask')
    artifacts=value.get('artifacts_sha256')
    if not isinstance(artifacts,dict) or not artifacts:raise Hold('complete XML/mesh/floor audit artifact hashes required')
    for filename,digest in artifacts.items():
        artifact=(Path(path).parent/filename).resolve() if not Path(filename).is_absolute() else Path(filename)
        if not isinstance(digest,str) or len(digest)!=64 or not artifact.is_file() or sha(artifact)!=digest:raise Hold('physical dependency missing or changed: '+filename)
    if alias.get('evidence_file') not in artifacts or alias.get('evidence_sha256')!=artifacts[alias['evidence_file']]:raise Hold('numeric alignment evidence not bound')
    config_key=value.get('sensor_config_json')
    if config_key not in artifacts:raise Hold('actual frozen sensor configuration must be an audited dependency')
    config_path=(Path(path).parent/config_key).resolve();sensors=read_json(config_path)['robot']['sensors']
    lidar,odom=sensors['lidar'],sensors['Odom']
    actual_xyz=[lidar['position'][axis] for axis in ('x','y','z')]
    actual_rpy=[math.radians(lidar['rotation'][axis]) for axis in ('roll','pitch','yaw')]
    if actual_xyz!=mount['xyz'] or any(abs(a-b)>1e-12 for a,b in zip(actual_rpy,mount['rpy_rad'])):raise Hold('T_body_lidar differs from frozen actual sensor config')
    if lidar['frame_id']!='map' or lidar['sensor_attach']!='base_link' or lidar['max_distance']!=100 or lidar.get('simulate_motion_distortion') is not False or lidar.get('ignore_mounting_actor') is not True:raise Hold('unsupported actual raw lidar contract')
    if odom['frame_id']!='map' or odom['child_frame_id'] not in ('base_link','BASE_LINK') or odom['sensor_attach']!='base_link' or any(odom['position'][axis]!=0 for axis in ('x','y','z')) or any(odom['rotation'][axis]!=0 for axis in ('roll','pitch','yaw')):raise Hold('original GT must refer to body origin, not a displaced sensor')
    model=validate_model(value.get('motion_model',{}))
    if value.get('motion_model_reviewed') is not True:raise Hold('root engineering motion model review is pending')
    if height.get('floor_z_m')!=model['floor_z_m']:raise Hold('live hit floor differs from audited truth floor')
    return value


def load_coverage_module():
    location=Path(__file__).resolve().parent/'dependencies/map_coverage.py'
    spec=importlib.util.spec_from_file_location('mapping_acquisition_existing_map_coverage',location)
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    return module


@dataclass
class OracleGrid:
    cells: object
    resolution: float
    origin: tuple
    body_half_xy: tuple=(HALF_BODY,HALF_BODY)
    padding: float=PADDING
    footprint_evidence: object=None

    def __post_init__(self):
        if len(self.body_half_xy)!=2 or not finite([*self.body_half_xy,self.padding]) or min(self.body_half_xy)<=0 or self.padding<0:raise Hold('invalid geometric body half widths/padding')

    @property
    def padded_half_xy(self):return tuple(h+self.padding for h in self.body_half_xy)

    @property
    def padded_radius(self):return math.hypot(*self.padded_half_xy)

    @classmethod
    def load(cls,manifest,path):
        coverage=load_coverage_module();yaml=(Path(path).parent/manifest['grid_yaml']).resolve()
        grid=coverage.load_grid_map(yaml)
        if not finite([grid.resolution,*grid.origin]) or not 0<grid.resolution<=.05:raise Hold('oracle grid must be finite at <=5cm')
        expected=manifest['artifacts_sha256']
        for artifact in [grid.source_yaml,grid.source_image]:
            if not any((Path(path).parent/key).resolve()==artifact and sha(artifact)==digest for key,digest in expected.items()):
                raise Hold('oracle grid YAML/image not bound to physical audit')
        body=footprint_spec(manifest,path)
        return cls(grid.cells,grid.resolution,grid.origin,tuple(body['body_half_xy_m']),PADDING,body)

    def local(self,x,y,yaw=0.):
        ox,oy,angle=self.origin;c,s=math.cos(angle),math.sin(angle);dx,dy=x-ox,y-oy
        return c*dx+s*dy,-s*dx+c*dy,wrap(yaw-angle)

    def state(self,x,y):
        x,y,_=self.local(x,y);ix,iy=math.floor(x/self.resolution),math.floor(y/self.resolution)
        if not 0<=iy<len(self.cells) or not 0<=ix<len(self.cells[0]):return 0
        return int(self.cells[iy][ix])

    def footprint_free(self,x,y,yaw,inflation=0.):
        if not finite([x,y,yaw,inflation]) or inflation<0:raise Hold('invalid footprint query')
        x,y,yaw=self.local(x,y,yaw);hx,hy=(h+inflation for h in self.padded_half_xy);r=self.resolution
        c,s=math.cos(yaw),math.sin(yaw);extent_x=hx*abs(c)+hy*abs(s);extent_y=hx*abs(s)+hy*abs(c);guard=r
        lo_x,hi_x=math.floor((x-extent_x-guard)/r),math.floor((x+extent_x+guard)/r)
        lo_y,hi_y=math.floor((y-extent_y-guard)/r),math.floor((y+extent_y+guard)/r)
        if lo_x<0 or lo_y<0 or hi_y>=len(self.cells) or hi_x>=len(self.cells[0]):return False
        if hasattr(self.cells,'shape') and bool((self.cells[lo_y:hi_y+1,lo_x:hi_x+1]==1).all()):return True
        # Same filled-cell SAT as the retained C++ checker, including its extra
        # whole-cell guard. Unknown cells are blocked, including interior cells.
        half_cell=.5*r+guard
        for iy in range(lo_y,hi_y+1):
            for ix in range(lo_x,hi_x+1):
                if int(self.cells[iy][ix])==1:continue
                dx,dy=(ix+.5)*r-x,(iy+.5)*r-y
                if abs(dx)>extent_x+half_cell or abs(dy)>extent_y+half_cell:continue
                projected=half_cell*(abs(c)+abs(s))
                if abs(c*dx+s*dy)>hx+projected or abs(-s*dx+c*dy)>hy+projected:continue
                return False
        return True

    def sweep(self,a,b,inflation=0.):
        dyaw=wrap(b[2]-a[2]);radius=math.hypot(*(h+inflation for h in self.padded_half_xy))
        steps=max(1,math.ceil((math.hypot(b[0]-a[0],b[1]-a[1])+radius*abs(dyaw))/(.5*self.resolution)))
        if steps>100000:raise Hold('unbounded sweep')
        return [(a[0]+(b[0]-a[0])*i/steps,a[1]+(b[1]-a[1])*i/steps,a[2]+dyaw*i/steps) for i in range(steps+1)]

    def checked_sweep(self,a,b,inflation=0.):
        samples=self.sweep(a,b,inflation)
        if not all(self.footprint_free(*pose,inflation) for pose in samples):raise Hold('filled-body sweep intersects occupied/unknown/outside physical oracle')
        return samples


def stopping_sweep(oracle,pose,kind,sign,model,input_age_s,policy=None):
    """Engineering SIM envelope: original age + packet/watchdog tail + reserve."""
    if kind not in ('forward','turn','zero','lateral') or sign not in (-1,0,1):raise Hold('unsupported acquisition primitive')
    if not finite([*pose,input_age_s]) or not -FUTURE_ALLOWANCE<=input_age_s<=MAX_INPUT_AGE:raise Hold('invalid sweep age/pose')
    radius=oracle.padded_radius;age=max(0.,input_age_s)
    age_yaw_rate=lateral_model(model,sign)['max_yaw_rate_rps'] if kind=='lateral' else model['max_truth_yaw_rate_rps']
    uncertainty=(model['max_truth_speed_mps']+radius*age_yaw_rate)*age
    tail=CONTROL_RENEWAL_LIMIT+PACKET_AGE+RELAY_WATCHDOG
    if kind=='forward':
        distance=model['max_truth_speed_mps']*tail+model['stop_distance_reserve_m']
        inflation=uncertainty+model['forward_drift_reserve_m']+radius*model['forward_yaw_reserve_rad']
        end=(pose[0]+distance*math.cos(pose[2]),pose[1]+distance*math.sin(pose[2]),pose[2])
    elif kind=='lateral':
        if sign not in (-1,1):raise Hold('lateral sign missing')
        lateral=lateral_model(model,sign)
        distance=lateral['max_truth_speed_mps']*tail+lateral['stop_distance_reserve_m']
        inflation=uncertainty+lateral['drift_reserve_m']+radius*lateral['yaw_reserve_rad']
        if policy is not None:inflation+=policy['max_cross_track_m']+radius*policy['max_yaw_error_rad']
        end=(pose[0]-sign*distance*math.sin(pose[2]),pose[1]+sign*distance*math.cos(pose[2]),pose[2])
    elif kind=='turn':
        angle=model['max_truth_yaw_rate_rps']*tail+model['stop_angle_reserve_rad']
        # The command horizon is < pi. Larger requested goals are rechecked each tick.
        if angle>=math.pi:raise Hold('rotation stop envelope unbounded')
        inflation=uncertainty+model['turn_drift_reserve_m'];end=(pose[0],pose[1],pose[2]+sign*angle)
    else:inflation=uncertainty;end=pose
    return oracle.checked_sweep(pose,end,inflation),inflation


def checked_cloud(message):
    fields=[(field.name,field.offset,field.datatype) for field in message.fields]
    if message.is_bigendian or message.height!=1 or message.point_step!=26 or fields!=POINT_FIELDS or any(field.count!=1 for field in message.fields):raise Hold('unexpected original Matrix PointCloud2 layout')
    if message.width<=0 or message.row_step!=26*message.width or len(message.data)!=message.row_step:raise Hold('empty/truncated raw hit cloud')
    import numpy as np
    dtype=np.dtype({'names':['x','y','z','intensity','tag','line','timestamp'],
      'formats':['<f4','<f4','<f4','<f4','u1','u1','<f8'],'offsets':[0,4,8,12,16,17,18],'itemsize':26})
    raw=np.frombuffer(message.data,dtype=dtype,count=message.width)
    points=np.column_stack([raw[key] for key in ['x','y','z']]).astype(float)
    intensity=raw['intensity'];stamp=raw['timestamp']
    if not np.isfinite(points).all() or not np.isfinite(intensity).all() or not (stamp==0.).all():raise Hold('nonfinite/invalid raw hit point')
    if not ((intensity>=0)&(intensity<=255)&(intensity==np.floor(intensity))).all():raise Hold('invalid raw intensity')
    distance=np.linalg.norm(points,axis=1)
    if (distance>100.01).any():raise Hold('raw hit exceeds frozen 100m Matrix sensor contract')
    # The simulator is root-verified to ignore its mounting actor. Do not hide
    # positive near returns or points inside the body. Exact origin is ambiguous
    # (no-return sentinel versus a hit) and fails closed instead of clearing space.
    if (distance==0.).any():raise Hold('ambiguous exact-origin raw hit; never treated as clear')
    return points


def hits_in_truth(points,pose,mount):
    import numpy as np
    q=mount_quaternion(mount['rpy_rad']);translation=mount['xyz']
    if not finite([*translation,*pose['xyz']]):raise Hold('nonfinite mount/GT translation')
    basis=[(1.,0.,0.),(0.,1.,0.),(0.,0.,1.)]
    body=np.asarray(points)@np.column_stack([rotate(q,p) for p in basis]).T+translation
    result=body@np.column_stack([rotate(pose['q'],p) for p in basis]).T+pose['xyz']
    if not np.isfinite(result).all():raise Hold('nonfinite transformed raw hits')
    return result


def guard_hits(points,sweep,inflation,model,oracle):
    """No sector/no-hit clearing and no blanket self-footprint mask."""
    floor=model['floor_z_m']+model['floor_hit_tolerance_m'];top=model['obstacle_top_m']
    import numpy as np
    points=np.asarray(points);kept=points[(points[:,2]>floor)&(points[:,2]<=top)]
    # Cover motion between neighboring sweep samples, whose corner motion is
    # bounded by half a cell. The oracle SAT has its own full-cell guard.
    hx,hy=(h+inflation+.5*oracle.resolution for h in oracle.padded_half_xy)
    extent=math.hypot(hx,hy);poses=np.asarray(sweep)
    nearby=kept[(kept[:,0]>=poses[:,0].min()-extent)&(kept[:,0]<=poses[:,0].max()+extent)&
                (kept[:,1]>=poses[:,1].min()-extent)&(kept[:,1]<=poses[:,1].max()+extent)]
    for x,y,yaw in sweep:
        c,s=math.cos(yaw),math.sin(yaw)
        dx,dy=nearby[:,0]-x,nearby[:,1]-y
        if ((np.abs(c*dx+s*dy)<=hx)&(np.abs(-s*dx+c*dy)<=hy)).any():raise Hold('fresh raw hit contradicts body/stop swept corridor')
    bins=np.unique(((np.arctan2(kept[:,1]-sweep[0][1],kept[:,0]-sweep[0][0])+math.pi)/(2*math.pi)*72).astype(int)%72)
    return dict(raw_hits=len(points),height_relevant_hits=len(kept),azimuth_bins_with_hits=len(bins),azimuth_bins_without_hits=72-len(bins),
                coverage_semantics='Hit/visibility statistics only; gaps/occlusion do not prove free. Physical static oracle is the free evidence.',
                geometric_self_mask_applied=False,scope=SCOPE)


def measured_stop(odom,command,now_ns,steady,truth_age_cap=MAX_TRUTH_AGE):
    fresh(odom,now_ns,steady,limit=truth_age_cap)
    if odom.get('frame_id')!=TRUTH_FRAME or odom.get('child_frame_id')!=TRUTH_CHILD or not finite(odom.get('xyz',[])) or len(odom.get('xyz',[]))!=3:return False
    try:checked_quaternion(odom.get('q',[]))
    except Hold:return False
    values=odom.get('twist',[])
    if len(values)!=6 or not finite(values):return False
    return math.sqrt(sum(x*x for x in values[:3]))<=.03 and math.sqrt(sum(x*x for x in values[3:]))<=.05 and command in ((0.,0.),(0.,0.,0.))


class StableStop:
    def __init__(self,truth_age_cap=MAX_TRUTH_AGE):self.since=None;self.stamp=None;self.anchor=None;self.truth_age_cap=truth_age_cap
    def update(self,odom,command,now_ns,steady,seconds):
        try:valid=measured_stop(odom,command,now_ns,steady,self.truth_age_cap)
        except Hold:valid=False
        if not valid:self.since=None;return False
        yaw=rpy(odom['q'])[2]
        if self.since is None:self.since=steady;self.stamp=odom['stamp_ns'];self.anchor=odom['xyz'][:2]+[yaw]
        if math.hypot(odom['xyz'][0]-self.anchor[0],odom['xyz'][1]-self.anchor[1])>.05 or abs(wrap(yaw-self.anchor[2]))>.05:
            self.since=None;return False
        return steady-self.since>=seconds and odom['stamp_ns']>self.stamp


def standing_before_relay(odom,model,now_ns,steady):
    """Even a zero 'walk' packet must not be used as an implicit stand command."""
    if not measured_stop(odom,(0.,0.),now_ns,steady,truth_age_limit(model)):raise Hold('body must already be measured stationary before relay starts')
    roll,pitch,_=rpy(odom['q'])
    if not model['body_z_min_m']<=odom['xyz'][2]<=model['body_z_max_m'] or max(abs(roll),abs(pitch))>model['max_roll_pitch_rad']:
        raise Hold('body is not in the root-reviewed standing height/attitude; no zero walk packet sent')


def straight_leg_state(start,target,pose,policy):
    """No corrective turns inside a frozen bridge; violation means zero + HOLD."""
    heading=policy['travel_yaw_rad'];dx,dy=target[0]-start[0],target[1]-start[1]
    length=math.hypot(dx,dy)
    if length<.15 or abs(wrap(math.atan2(dy,dx)-heading))>.005:raise Hold('fixed-heading bridge does not match its forward segment')
    along=(pose[0]-start[0])*math.cos(heading)+(pose[1]-start[1])*math.sin(heading)
    cross=-(pose[0]-start[0])*math.sin(heading)+(pose[1]-start[1])*math.cos(heading)
    if abs(wrap(pose[2]-heading))>policy['max_yaw_error_rad'] or abs(cross)>policy['max_cross_track_m'] or not -.15<=along<=length+.15:
        raise Hold('fixed-heading bridge yaw/tube/endpoint bound violated; no in-leg turn')
    return dict(along_m=along,cross_track_m=cross,length_m=length)


def audit_route_geometry(oracle,route,model):
    """Nominal full-route SAT proof; fresh age/stop envelopes are rechecked live.

    This is geometric feasibility under the root's SIM bounds, not a promise of
    successful control. A live age/drift/stop envelope may still require HOLD.
    """
    pose=tuple(route['start_anchor_truth']);samples=0;legs=[];radius=oracle.padded_radius
    for point in route['waypoints']:
        x,y,yaw=point['pose_truth'];length=math.hypot(x-pose[0],y-pose[1]);policy=point.get('leg_policy')
        forward_terminal_policy(pose,(x,y,yaw),policy)
        if policy is not None and policy.get('kind')=='lateral_fixed_body_heading':
            sign=policy['lateral_sign'];body=policy['body_yaw_rad'];lat=lateral_model(model,sign)
            if length+LENGTH_COMPARISON_EPS_M<lat['min_controllable_leg_m'] or policy['endpoint_tolerance_m']+LENGTH_COMPARISON_EPS_M<lat['endpoint_accuracy_bound_m']:
                raise Hold('lateral short leg/endpoint tracking unsupported by root measured engineering model')
            lateral_leg_state(pose,(x,y,yaw),(*pose[:2],body),policy)
            samples+=len(oracle.checked_sweep(pose,(*pose[:2],body),model['turn_drift_reserve_m']))
            inflation=lat['drift_reserve_m']+radius*lat['yaw_reserve_rad']+policy['max_cross_track_m']+radius*policy['max_yaw_error_rad']
            samples+=len(oracle.checked_sweep((*pose[:2],body),(x,y,body),inflation))
            samples+=len(oracle.checked_sweep((x,y,body),(x,y,yaw),model['turn_drift_reserve_m']))
            legs.append(dict(id=point['id'],length_m=length,lateral_sign=sign,body_yaw_rad=body,world_travel_yaw_rad=policy['world_travel_yaw_rad']))
            pose=(x,y,yaw);continue
        endpoint=forward_endpoint_state(pose,(x,y,yaw),pose,policy)
        heading=policy['travel_yaw_rad'] if policy else math.atan2(y-pose[1],x-pose[0]) if length>.15 else pose[2]
        if policy:straight_leg_state(pose,(x,y,yaw),(pose[0],pose[1],heading),policy)
        samples+=len(oracle.checked_sweep(pose,(*pose[:2],heading),model['turn_drift_reserve_m']))
        inflation=model['forward_drift_reserve_m']+radius*model['forward_yaw_reserve_rad']
        if policy:inflation+=policy['max_cross_track_m']+radius*policy['max_yaw_error_rad']
        samples+=len(oracle.checked_sweep((*pose[:2],heading),(x,y,heading),inflation))
        samples+=len(oracle.checked_sweep((x,y,heading),(x,y,yaw),model['turn_drift_reserve_m']))
        legs.append(dict(id=point['id'],length_m=length,travel_yaw_rad=heading,fixed_heading=policy is not None,
            radial_endpoint_tolerance_m=endpoint['radial_tolerance_m'],along_endpoint_tolerance_m=endpoint['along_endpoint_tolerance_m']))
        pose=(x,y,yaw)
    return dict(checked=True,legs=legs,sweep_samples=samples,semantics='Nominal filled-body translation/rotation and fixed-heading tube; original-age stopping reserve is checked before every live packet.')


def primitive_packet(vx,wz,now_ns,lateral_raw=0.,fine_forward_profile=None,fine_phase=False):
    if fine_phase is not False:
        import lateral_capability as proof
        if fine_phase is not True:raise Hold('fine phase must be exact boolean')
        try:profile=proof.fine_profile(fine_forward_profile)
        except ValueError as exc:raise Hold(str(exc)) from exc
        if not finite([vx,wz,lateral_raw]) or vx!=profile['selected_fine_forward_mps'] or wz!=0. or lateral_raw!=0.:raise Hold('only root-selected exclusive fine forward may use this phase')
        return dict(scope=SCOPE,capability=CAPABILITY,issued_unix_ns=now_ns,forward=vx/2.,lateral=0.,yaw=0.,
            primitive_phase='forward_terminal_fine',fine_forward_subcapability=profile['subcapability'])
    if (not finite([vx,wz,lateral_raw]) or vx not in (0.,MAX_VX) or wz not in (-MAX_WZ,0.,MAX_WZ)
        or lateral_raw not in (-MAX_LATERAL_RAW,0.,MAX_LATERAL_RAW) or sum(v!=0 for v in [vx,wz,lateral_raw])>1):
        raise Hold('only forward, inplace turn or pure measured lateral; no reverse/mixed/micro axes')
    return dict(scope=SCOPE,capability=CAPABILITY,issued_unix_ns=now_ns,forward=vx/2.,lateral=lateral_raw,yaw=wz)


def lateral_model(model,sign):
    if sign not in (-1,1):raise Hold('lateral sign must be explicit')
    return model['lateral_models']['positive' if sign>0 else 'negative']


def validate_lateral_models(model):
    if model.get('lateral_capability')!=CAPABILITY or model.get('lateral_model_reviewed') is not True:
        raise Hold('root actual semantic calibration and directional model pending')
    digest=model.get('lateral_calibration_proof_sha256')
    if not isinstance(digest,str) or len(digest)!=64:raise Hold('directional model must bind actual calibration proof SHA')
    values=model.get('lateral_models',{})
    if set(values)!={'positive','negative'}:raise Hold('both root directional lateral models required')
    for name,sign in [('positive',1),('negative',-1)]:
        value=values[name]
        keys=['max_truth_speed_mps','max_yaw_rate_rps','stop_distance_reserve_m','drift_reserve_m','yaw_reserve_rad','min_controllable_leg_m','endpoint_accuracy_bound_m',
            'bias_vx_mps','nominal_vy_mps','k_cross_per_sec']
        if value.get('raw_lateral')!=sign*MAX_LATERAL_RAW or not finite([value.get(k) for k in keys]):raise Hold('finite measured semantic single-axis model required')
        if (not 0<value['max_truth_speed_mps']<=model['max_truth_speed_mps'] or not 0<value['max_yaw_rate_rps']<=model['max_truth_yaw_rate_rps'] or not .1<=value['stop_distance_reserve_m']<=5.
            or not 0<=value['drift_reserve_m']<=.5 or not 0<=value['yaw_reserve_rad']<=.5
            or not 0<value['min_controllable_leg_m']<=100. or not 0<value['endpoint_accuracy_bound_m']<=.15):raise Hold('directional root model outside supported engineering bounds')
        if (value['nominal_vy_mps']==0 or math.hypot(value['bias_vx_mps'],value['nominal_vy_mps'])>value['max_truth_speed_mps']
            or value['k_cross_per_sec']<0):raise Hold('explicit finite directional bias/nominal/gain engineering model required')
    return values


def lateral_leg_state(start,target,pose,policy):
    if policy.get('kind')!='lateral_fixed_body_heading':raise Hold('not a fixed-body lateral leg')
    keys=['body_yaw_rad','world_travel_yaw_rad','max_yaw_error_rad','max_cross_track_m','endpoint_tolerance_m']
    if not finite([*start,*target,*pose,*[policy.get(k) for k in keys]]):raise Hold('nonfinite lateral leg')
    sign=policy.get('lateral_sign');body=policy['body_yaw_rad'];travel=policy['world_travel_yaw_rad']
    if type(sign) is not int or sign not in (-1,1) or min(abs(wrap(body-h)) for h in (0.,math.pi/2,-math.pi/2,math.pi))>1e-8:raise Hold('only measured lateral sign with fixed cardinal body yaw0/+-pi/2/pi')
    dx,dy=target[0]-start[0],target[1]-start[1];length=math.hypot(dx,dy)
    if length<=0 or abs(wrap(math.atan2(dy,dx)-travel))>.005 or abs(wrap(travel-body-sign*math.pi/2))>.005:
        raise Hold('incoming leg is not pure signed body-Y translation')
    if (not 0<policy['max_yaw_error_rad']<=MAX_LATERAL_YAW_BAND or not 0<policy['max_cross_track_m']<=.15
        or not 0<policy['endpoint_tolerance_m'] or policy['endpoint_tolerance_m']>min(.15,length/4)+LENGTH_COMPARISON_EPS_M):
        raise Hold('lateral yaw/tube/endpoint tolerance outside bounds; short legs cannot be skipped at.15')
    along=(pose[0]-start[0])*math.cos(travel)+(pose[1]-start[1])*math.sin(travel)
    cross=-(pose[0]-start[0])*math.sin(travel)+(pose[1]-start[1])*math.cos(travel)
    tolerance=policy['endpoint_tolerance_m']
    if abs(wrap(pose[2]-body))>policy['max_yaw_error_rad'] or abs(cross)>policy['max_cross_track_m'] or not -tolerance<=along<=length+tolerance:
        raise Hold('lateral yaw/tube/endpoint overrun; zero+HOLD, no in-leg correction')
    return dict(along_m=along,cross_track_m=cross,length_m=length,remaining_m=length-along,
                reached=math.hypot(target[0]-pose[0],target[1]-pose[1])<=tolerance)


def translation_endpoint_tolerance(start,target):
    length=math.hypot(target[0]-start[0],target[1]-start[1])
    return .15 if length<=1e-9 else min(.15,length/4)


def forward_terminal_policy(start,target,policy):
    """Optional same-bridge forward terminal work; no tolerance or axis change."""
    if not isinstance(policy,dict) or 'forward_terminal' not in policy:return None
    value=policy['forward_terminal'];length=math.hypot(target[0]-start[0],target[1]-start[1])
    keys={'kind','coarse_stop_remaining_m','max_corrections','max_extra_work_sec'}
    along=policy.get('along_endpoint_tolerance_m');heading=policy.get('travel_yaw_rad')
    if (policy.get('kind')!='straight_fixed_heading' or not isinstance(value,dict) or set(value) not in (keys,keys|{'fine_forward_mps'},keys|{'fine_pulse_ticks','max_fine_pulses'})
        or value.get('kind')!='coarse_stop_then_fine_v1' or not finite([*start,*target,along,heading])
        or not 0<along<=min(.15,length/4) or abs(wrap(target[2]-heading))>.005):
        raise Hold('forward terminal requires explicit valid fixed-heading along tolerance and unchanged target heading')
    if 'fine_forward_mps' in value and (not finite([value['fine_forward_mps']]) or value['fine_forward_mps'] not in (.2,.3)):raise Hold('fine forward allows only explicit internal.2/.3; absent retainsMAX_VX')
    if 'fine_pulse_ticks' in value and (type(value['fine_pulse_ticks']) is not int or value['fine_pulse_ticks'] not in (1,2,3)
        or type(value['max_fine_pulses']) is not int or not 1<=value['max_fine_pulses']<=32):raise Hold('fine pulse ticks exactint1/2/3 and total max_fine_pulses exactint1..32 required')
    coarse=value['coarse_stop_remaining_m'];budget=value['max_extra_work_sec'];count=value['max_corrections']
    if (not finite([coarse,budget]) or not .05<=coarse<=min(.15,length/2) or coarse<=along
        or not 0<budget<=20. or type(count) is not int or not 1<=count<=2):
        raise Hold('forward terminal bounds: coarse[.05,min(.15,L/2)]>along; corrections1..2; work(0,20]')
    return dict(value)


def forward_endpoint_state(start,target,pose,policy=None):
    """Old radial test plus an optional frozen-heading along test; no correction."""
    forward_terminal_policy(start,target,policy)
    radial=translation_endpoint_tolerance(start,target)
    distance=math.hypot(target[0]-pose[0],target[1]-pose[1]);along_tolerance=None;along=None;error=None
    if policy is not None and 'along_endpoint_tolerance_m' in policy:
        value=policy['along_endpoint_tolerance_m'];heading=policy.get('travel_yaw_rad')
        length=math.hypot(target[0]-start[0],target[1]-start[1])
        if policy.get('kind')!='straight_fixed_heading' or not finite([value,heading,*start,*target,*pose]) or not 0<value<=min(.15,length/4):
            raise Hold('forward along tolerance must be finite nonbool in(0,min(.15,L/4)]')
        along_tolerance=value;along=(pose[0]-start[0])*math.cos(heading)+(pose[1]-start[1])*math.sin(heading)
        error=abs(length-along)
    return dict(radial_distance_m=distance,radial_tolerance_m=radial,along_endpoint_tolerance_m=along_tolerance,
        actual_along_m=along,absolute_along_error_m=error,reached=distance<=radial and (along_tolerance is None or error<=along_tolerance))


def require_translation_progress(start,target,entry,pose,tolerance):
    """A stationary/turn-only point is distinct from a nonzero acquisition leg."""
    dx,dy=target[0]-start[0],target[1]-start[1];length=math.hypot(dx,dy)
    if length<=1e-9:return dict(required=False,actual_along_m=0.,minimum_progress_m=0.)
    if not finite([*entry,*pose,tolerance]) or not 0<tolerance or tolerance>min(.15,length/4)+LENGTH_COMPARISON_EPS_M:raise Hold('invalid short-leg completion tolerance')
    along=((pose[0]-entry[0])*dx+(pose[1]-entry[1])*dy)/length
    minimum=min(tolerance,length/4)
    if along+1e-9<minimum:raise Hold('endpoint reached without actual leg progress; no skipped short translation')
    return dict(required=True,actual_along_m=along,minimum_progress_m=minimum)


def lateral_heading_target(state,pose,policy,model):
    """Fixed-cardinal body-forward error is -sign times the travel-left error."""
    sign=policy['lateral_sign'];directional=lateral_model(model,sign)
    if not finite([*pose,state.get('cross_track_m'),directional.get('bias_vx_mps'),directional.get('nominal_vy_mps'),directional.get('k_cross_per_sec')]):raise Hold('nonfinite heading compensation model/state')
    bias,vy,gain=directional['bias_vx_mps'],abs(directional['nominal_vy_mps']),directional['k_cross_per_sec']
    if vy<=0 or gain<0:raise Hold('invalid nominal lateral velocity/cross gain')
    error=-sign*state['cross_track_m'];delta=sign*math.atan2(bias+gain*error,vy)
    band=policy['max_yaw_error_rad'];margin=band-abs(delta)
    if not 0<band<=MAX_LATERAL_YAW_BAND or margin<=1e-6:raise Hold('heading compensation has no policy yaw-band margin; no silent clamp')
    cross_margin=policy['max_cross_track_m']-abs(state['cross_track_m'])
    minimum_margin=min(.005,policy['max_cross_track_m']/10)
    if cross_margin<=minimum_margin:raise Hold('cross tracking has no hard-tube margin')
    fine=min(.02,margin/2)
    return dict(target_body_yaw_rad=wrap(policy['body_yaw_rad']+delta),target_delta_from_cardinal_rad=delta,
        cardinal_body_yaw_rad=policy['body_yaw_rad'],body_forward_cross_error_m=error,
        settled_yaw_tolerance_rad=fine,yaw_band_margin_rad=margin,cross_hard_margin_m=cross_margin,
        yaw_error_to_target_rad=abs(wrap(pose[2]-policy['body_yaw_rad']-delta)))


def lateral_correction_state(state,pose,policy,attempts,model):
    """Target-error recovery; legal non-cardinal compensation is not a fault."""
    if type(attempts) is not int or attempts<0:raise Hold('invalid correction count')
    if not finite([*pose,state.get('cross_track_m'),policy.get('body_yaw_rad'),policy.get('max_yaw_error_rad'),policy.get('max_cross_track_m')]):raise Hold('nonfinite correction state')
    target=lateral_heading_target(state,pose,policy,model)
    yaw_error=target['yaw_error_to_target_rad'];fine=target['settled_yaw_tolerance_rad']
    cross=abs(state['cross_track_m']);cross_trigger=policy['max_cross_track_m']/2
    yaw_trigger=max(fine,min(.04,target['yaw_band_margin_rad']/2))
    requested=yaw_error>yaw_trigger or cross>=cross_trigger and yaw_error>fine
    if requested and attempts>=MAX_LATERAL_CORRECTIONS:raise Hold('bounded same-leg heading corrections exhausted')
    return dict(target,stop_then_recheck=requested,yaw_error_rad=yaw_error,cross_track_m=cross,
        yaw_trigger_rad=yaw_trigger,cross_trigger_m=cross_trigger,observe_cross=cross>=cross_trigger)


def feedback_window(observation,state,steady,along):
    if observation is None:return None
    if steady-observation['started_steady']<FEEDBACK_WINDOW_SEC and along-observation['start_along_m']<FEEDBACK_WINDOW_PROGRESS_M:return observation
    if abs(state['cross_track_m'])>=observation['start_abs_cross_m']-MIN_CROSS_IMPROVEMENT_M:
        raise Hold('bounded compensated progress/time window showed no cross improvement')
    return None


def directional_yaw_guard(truth,model,sign):
    rate=truth.get('original_header_yaw_rate_rps');twist=truth.get('twist',[])
    if len(twist)!=6 or not finite([rate,*twist]):raise Hold('original lateral yaw-rate evidence missing/nonfinite')
    maximum=lateral_model(model,sign)['max_yaw_rate_rps']
    if max(abs(twist[5]),abs(rate))>maximum:raise Hold('actual lateral yaw exceeds directional cap')
    return dict(original_header_yaw_rate_rps=rate,twist_yaw_rate_rps=twist[5],max_yaw_rate_rps=maximum)


def actual_gt_lateral_stopping_sweep(oracle,truth,bridge,model,now_ns,steady):
    """Keep the selected padded body+future errors; policy error is in GT pose.

    This entry constructs its pose from the original accepted GT snapshot. It
    cannot be used for a nominal route pose. Generic stopping_sweep and nominal
    audit keep nominal policy-envelope semantics with the selected body radius.
    """
    if (model.get('future_error_reference')!=ACTUAL_GT_STOP_REFERENCE
        or model.get('independent_future_errors_reviewed') is not True):
        raise Hold('independent future error reserves not explicitly approved')
    cap=truth_age_limit(model);fresh(truth,now_ns,steady,limit=cap)
    if (truth.get('frame_id')!=TRUTH_FRAME or truth.get('child_frame_id')!=TRUTH_CHILD
        or truth.get('stamp_regressed') is True or len(truth.get('xyz',[]))!=3
        or len(truth.get('twist',[]))!=6 or not finite(truth['xyz']+truth['twist'])):
        raise Hold('actual original GT reference invalid')
    roll,pitch,yaw=rpy(truth.get('q',[]));pose=(*truth['xyz'][:2],yaw)
    if (max(abs(roll),abs(pitch))>model['max_roll_pitch_rad']
        or not model['body_z_min_m']<=truth['xyz'][2]<=model['body_z_max_m']):
        raise Hold('actual GT reference height/attitude invalid')
    start,target,policy=bridge;state=lateral_leg_state(start,target,pose,policy)
    sign=policy['lateral_sign'];directional=lateral_model(model,sign)
    yaw_rate=directional_yaw_guard(truth,model,sign)
    if math.sqrt(sum(v*v for v in truth['twist'][:3]))>directional['max_truth_speed_mps']:raise Hold('actual lateral GT speed exceeds directional future model')
    age_upper=command_age_upper((now_ns-truth['stamp_ns'])/1e9,cap)
    # New scoped timers; independent future drift/yaw and D are not consumed.
    samples,inflation=stopping_sweep(oracle,pose,'lateral',sign,model,age_upper,policy=None)
    reference=dict(basis=ACTUAL_GT_STOP_REFERENCE,original_GT_header_ns=truth['stamp_ns'],frame_id=truth['frame_id'],
        child_frame_id=truth['child_frame_id'],actual_pose_truth=list(pose),input_age_upper_s=age_upper,max_truth_age_sec=cap,
        observed_policy_error=state,observed_yaw_error_rad=wrap(yaw-policy['body_yaw_rad']),
        observed_policy_envelope_readded=False,independent_future_errors_retained=True,
        nominal_body_half_m=oracle.padded_half_xy[0] if oracle.padded_half_xy[0]==oracle.padded_half_xy[1] else None,
        nominal_body_half_xy_m=list(oracle.padded_half_xy),footprint_radius_m=oracle.padded_radius,
        footprint_evidence=oracle.footprint_evidence,padding_consumed_as_error_budget=False,
        signed_stop_tail_sec=CONTROL_RENEWAL_LIMIT+PACKET_AGE+RELAY_WATCHDOG,directional_yaw_rate_guard=yaw_rate)
    return samples,inflation,reference


def process_identity(pid):
    path=Path('/proc')/str(pid);parts=(path/'stat').read_text().rsplit(')',1)[1].split();argv=(path/'cmdline').read_bytes()
    if not argv or parts[0]=='Z':raise ProcessLookupError('process is not live')
    return dict(pid=int(pid),starttime_ticks=int(parts[19]),cmdline_sha256=hashlib.sha256(argv).hexdigest(),
                argv=[part.decode(errors='backslashreplace') for part in argv.rstrip(b'\0').split(b'\0')],boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())


def verify_retired(expected,reader=process_identity):
    for entry in expected:
        try:current=reader(entry['pid'])
        except (FileNotFoundError,ProcessLookupError):continue
        if current==entry:raise Hold('retired velocity/Nav writer is still live: '+str(entry['pid']))
        # A reused PID is not declared to be the old process; the socket inventory
        # still has to authorize the new process's complete identity independently.


def claim_once(path,record):
    descriptor=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(descriptor,'w') as stream:
        stream.write(json.dumps(record,indent=2)+'\n');stream.flush();os.fsync(stream.fileno())


def udp_inventory(proc=Path('/proc')):
    """Inventory every UDP socket in this network namespace; unknown owners HOLD."""
    sockets={}
    for protocol in ['udp','udp6']:
        for line in (proc/'self/net'/protocol).read_text().splitlines()[1:]:
            fields=line.split();sockets[fields[9]]=dict(protocol=protocol,local=fields[1],remote=fields[2],inode=fields[9],owners=[])
    errors=[]
    for directory in proc.iterdir():
        if not directory.name.isdecimal():continue
        try:
            for fd in (directory/'fd').iterdir():
                try:target=os.readlink(fd)
                except FileNotFoundError:continue
                if target.startswith('socket:[') and target[8:-1] in sockets:sockets[target[8:-1]]['owners'].append(int(directory.name))
        except (FileNotFoundError,ProcessLookupError):continue
        except PermissionError:errors.append(str(directory))
    if errors:raise Hold('cannot prove UDP ownership; unreadable process fd tables: '+repr(errors[:5]))
    return list(sockets.values())


def verify_udp_inventory(inventory,allowed_processes,allowed_external_sockets,reader=process_identity):
    known={value['pid']:value for value in allowed_processes}
    external={(value['protocol'],value['local'],value['remote'],str(value['inode'])) for value in allowed_external_sockets}
    for item in inventory:
        key=(item['protocol'],item['local'],item['remote'],str(item['inode']))
        if not item['owners']:
            if key not in external:raise Hold('unattributed UDP socket; no exclusivity proof')
            continue
        for pid in item['owners']:
            if pid not in known or reader(pid)!=known[pid]:raise Hold('unknown or changed UDP writer identity: '+str(pid))
    return True


def fine_command_profile(release,bridge,vx):
    import lateral_capability as proof
    try:profile=proof.fine_profile_from_release(release)
    except ValueError as exc:raise Hold(str(exc)) from exc
    if bridge is None or profile is None:raise Hold('fine forward requires explicit root release and frozen leg')
    config=forward_terminal_policy(*bridge)
    if config is None or config.get('fine_forward_mps')!=profile['selected_fine_forward_mps'] or vx!=profile['selected_fine_forward_mps']:
        raise Hold('fine forward route/value differs from root-selected subcapability')
    return profile


def validate_fine_route(release,route):
    import lateral_capability as proof
    try:profile=proof.fine_profile_from_release(release)
    except ValueError as exc:raise Hold(str(exc)) from exc
    start=route['start_anchor_truth']
    for point in route['waypoints']:
        target=point['pose_truth'];policy=point.get('leg_policy');config=forward_terminal_policy(start,target,policy)
        if config is not None and 'fine_pulse_ticks' in config and profile is not None:raise Hold('plain fine pulses require unused low-forward profile disabled/unselected')
        if config is not None and 'fine_forward_mps' in config:fine_command_profile(release,(start,target,policy),config['fine_forward_mps'])
        start=target
    return profile


SIM_ENDPOINT_SCHEMA='locnav.exact_sim_command_endpoint.v1'
SIM_UE_ID_KEYS=('pid','starttime_ticks','cmdline_sha256','argv','boot_id')

def sim_endpoint_from_release(release):
    allow=release.get('allow_sim_host_endpoint',False);value=release.get('sim_command_endpoint')
    if type(allow) is not bool:raise Hold('SIM endpoint flag must be exact bool')
    if not allow:
        if value is not None:raise Hold('nondefault SIM endpoint requires explicit root flag')
        return dict(schema=SIM_ENDPOINT_SCHEMA,ipv4='127.0.0.1',port=7447,scene_identity_proof_sha256=None,ue_process_identity=None)
    keys={'schema','ipv4','port','scene_identity_proof_sha256','ue_process_identity'}
    if (not isinstance(value,dict) or set(value)!=keys or value['schema']!=SIM_ENDPOINT_SCHEMA
        or value['ipv4']!='192.168.1.219' or type(value['port']) is not int or value['port']!=7447):
        raise Hold('only exact Root-attested SIM IPv4 192.168.1.219:7447 is supported')
    scene=release.get('scene_instance_id');proof=release.get('scene_identity_proof',{});identity=value['ue_process_identity'];digest=value['scene_identity_proof_sha256']
    if (not valid_local_identity(identity) or not isinstance(identity['boot_id'],str) or not identity['boot_id']
        or not isinstance(scene,dict) or identity!={key:scene.get(key) for key in SIM_UE_ID_KEYS}
        or not isinstance(digest,str) or len(digest)!=64 or any(ch not in '0123456789abcdef' for ch in digest)
        or proof.get('mode')!='root_external_host_attestation' or proof.get('evidence_sha256')!=digest
        or not proof.get('host_pid_namespace') or proof.get('host_pid_namespace')!=scene.get('pid_namespace')):
        raise Hold('SIM endpoint must bind same Root host-attestation SHA and exact UE5 identity')
    path=Path(proof.get('evidence_path',''))
    if not path.is_absolute() or not path.is_file() or sha(path)!=digest or read_json(path)!=scene:
        raise Hold('SIM endpoint host-attestation file missing/changed or wrong UE scene')
    return dict(value,ue_process_identity=dict(identity))


def sim_endpoint_from_admission(admission,release):
    expected=sim_endpoint_from_release(release)
    if admission.get('sim_command_endpoint')!=expected:raise Hold('relay SIM endpoint admission differs from same Root release')
    return expected
