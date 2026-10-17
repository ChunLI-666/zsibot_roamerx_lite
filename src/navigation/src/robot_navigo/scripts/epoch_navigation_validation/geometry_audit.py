"""Independent occupied-cell SAT oracle and conservative continuous SE2 sweep.

The polygon/cell axes follow controller_feedback/footprint_oracle.hpp. No
production controller collision predicate is imported or called. The sweep
inflates support projections by a proven vertex displacement bound; an overlap
of inflated projections alone is uncertainty, not a confirmed collision.
"""
import hashlib
import math
from pathlib import Path

import numpy as np
import yaml


def load_footprint(path):
    path=Path(path);data=yaml.safe_load(path.read_text());raw=np.asarray(data['footprint'],dtype=float)
    if data['frame_id']!='base_link':raise ValueError('Expected base_link geometry')
    if raw.ndim!=2 or raw.shape[1]!=2 or len(raw)<3 or not np.isfinite(raw).all():raise ValueError('Invalid polygon')
    edges=np.roll(raw,-1,axis=0)-raw
    cross=np.cross(edges,np.roll(edges,-1,axis=0))
    if not (np.all(cross>1e-10) or np.all(cross<-1e-10)):raise ValueError('Strict convex polygon required')
    padding=float(data['footprint_padding'])
    if not math.isfinite(padding) or padding<0:raise ValueError('Invalid padding')
    # Same documented Nav2 padFootprint convention, independently calculated.
    polygon=raw+np.sign(raw)*padding
    if 'padded_footprint' in data and not np.allclose(polygon,data['padded_footprint'],atol=1e-10,rtol=0):
        raise ValueError('Authority padded vertices disagree with independent calculation')
    return polygon,dict(source=str(path.resolve()),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        raw_polygon=raw.tolist(),padded_polygon=polygon.tolist(),padding=padding,
        frame_id=data['frame_id'],inflation_radius=data.get('inflation_radius'),envelope_kind=data.get('envelope_kind'))


def transform(polygon,pose):
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return polygon@np.array([[c,s],[-s,c]])+pose[:2]


def twist_pose(start,command,elapsed):
    """Independent SE(2) exponential; does not import the plant integrator."""
    x,y,yaw=start;vx,vy,wz=command;angle=wz*elapsed
    sinc=math.sin(angle)/angle if abs(angle)>1e-8 else 1.-angle*angle/6.
    cosc=2.*math.sin(angle/2.)**2/angle if angle else 0.
    dx=elapsed*(sinc*vx-cosc*vy);dy=elapsed*(cosc*vx+sinc*vy)
    c,s=math.cos(yaw),math.sin(yaw)
    return [x+c*dx-s*dy,y+s*dx+c*dy,math.atan2(math.sin(yaw+angle),math.cos(yaw+angle))]


class Oracle:
    def __init__(self,scene,polygon):
        self.scene=scene;self.polygon=np.asarray(polygon);self.radius=float(np.linalg.norm(polygon,axis=1).max())

    def at(self,pose,margin=0.):
        if not np.isfinite(pose).all():return 'nonfinite'
        world=transform(self.polygon,pose);epsilon=max(1e-9,self.scene.resolution*1e-8)
        lower=world.min(axis=0)-margin-epsilon;upper=world.max(axis=0)+margin+epsilon
        low=np.floor((lower-self.scene.origin)/self.scene.resolution).astype(int)
        high=np.floor((upper-self.scene.origin)/self.scene.resolution).astype(int)
        height,width=self.scene.free.shape
        if min(low)<0 or high[0]>=width or high[1]>=height:return 'outside_map'
        region=self.scene.occupancy[low[1]:high[1]+1,low[0]:high[0]+1]
        iy,ix=np.nonzero(region!=0)
        if not len(ix):return 'free'
        centers=np.column_stack((ix+low[0]+.5,iy+low[1]+.5))*self.scene.resolution+self.scene.origin
        edges=np.roll(world,-1,axis=0)-world
        axes=np.vstack((np.eye(2),np.column_stack((-edges[:,1],edges[:,0]))))
        projected=world@axes.T;center=centers@axes.T
        support=self.scene.resolution*.5*np.abs(axes).sum(axis=1)+margin*np.linalg.norm(axes,axis=1)
        overlaps=np.all((center+support>=projected.min(axis=0)-1e-10)&(center-support<=projected.max(axis=0)+1e-10),axis=1)
        if not overlaps.any():return 'free'
        return 'occupied' if np.any(region[iy[overlaps],ix[overlaps]]==1) else 'unknown'

    def _sample_sweep(self,pose_at,displacement,max_vertex_step,model):
        if not math.isfinite(displacement) or displacement<0 or not math.isfinite(max_vertex_step) or max_vertex_step<=0:
            raise ValueError('Invalid continuous sweep bound')
        count=max(1,math.ceil(displacement/max_vertex_step))
        # Every instant is within half a subdivision of a sampled pose. The
        # supplied bound covers the entire vertex path, not just its endpoints.
        margin=displacement/(2*count)
        possible=None
        for fraction in np.linspace(0,1,count+1):
            pose=pose_at(float(fraction))
            exact=self.at(pose)
            if exact!='free':return dict(status='collision',reason=exact,fraction=float(fraction),subdivisions=count,bound_m=margin,model=model)
            expanded=self.at(pose,margin)
            if expanded!='free':possible=expanded
        return dict(status='uncertain' if possible else 'free',reason=possible or 'free',subdivisions=count,bound_m=margin,model=model)

    def sweep(self,start,end,max_vertex_step=.002):
        """Legacy straight-center chord, shortest-yaw interpolation only."""
        delta=np.asarray(end,dtype=float)-start
        delta[2]=math.atan2(math.sin(delta[2]),math.cos(delta[2]))
        displacement=float(np.linalg.norm(delta[:2])+self.radius*abs(delta[2]))
        return self._sample_sweep(lambda fraction:np.asarray(start)+fraction*delta,
            displacement,max_vertex_step,'legacy_chord')

    def sweep_twist(self,start,command,dt,max_vertex_step=.002):
        if len(start)!=3 or len(command)!=3 or not all(math.isfinite(v) for v in [*start,*command,dt]) or dt<0:
            raise ValueError('Invalid constant-twist segment')
        speed=math.hypot(command[0],command[1])+self.radius*abs(command[2])
        # For any footprint vertex |p|<=radius, |dp/dt| <= |v|+radius*|w|.
        # Sampling the arc itself avoids an unaccounted chord sagitta term and
        # retains revolutions even when the endpoint yaw wraps to its start.
        result=self._sample_sweep(lambda fraction:twist_pose(start,command,dt*fraction),
            speed*dt,max_vertex_step,'constant_body_twist')
        result['vertex_speed_bound_mps']=speed
        return result


def audit_geometry(rows,scene,polygon):
    oracle=Oracle(scene,polygon);steps=[r for r in rows if r['event']=='plant_step']
    start=next(r['data']['pose'] for r in rows if r['event']=='fixture_start');initial=oracle.at(start)
    violations=[];uncertain=[];invalid=[];reverse_distance=0.;lateral_distance=0.;path_length=0.;maximum_bound=0.
    models={'constant_body_twist':0,'legacy_chord':0};missing_velocity=0;maximum_position_error=maximum_yaw_error=0.
    for row in steps:
        data=row['data']
        exact=all(key in data for key in ('executed','dt','interval_start_ns','interval_end_ns'))
        model='constant_body_twist' if exact else 'legacy_chord';models[model]+=1
        try:
            if exact:
                duration=(data['interval_end_ns']-data['interval_start_ns'])/1e9
                if duration<0 or not math.isfinite(data['dt']) or abs(duration-data['dt'])>1e-12:
                    raise ValueError('Recorded wall interval disagrees with dt')
                if len(data['after'])!=3 or not all(math.isfinite(v) for v in data['after']):
                    raise ValueError('Recorded endpoint is not a finite pose')
                sweep=oracle.sweep_twist(data['before'],data['executed'],data['dt'])
                expected=twist_pose(data['before'],data['executed'],data['dt'])
                position_error=math.dist(expected[:2],data['after'][:2])
                yaw_error=abs(math.atan2(math.sin(expected[2]-data['after'][2]),math.cos(expected[2]-data['after'][2])))
                maximum_position_error=max(maximum_position_error,position_error);maximum_yaw_error=max(maximum_yaw_error,yaw_error)
                if position_error>1e-9 or yaw_error>1e-9:
                    raise ValueError('Recorded endpoint disagrees with constant body twist')
                path_length+=math.hypot(*data['executed'][:2])*data['dt']
            else:
                sweep=oracle.sweep(data['before'],data['after'])
                path_length+=math.dist(data['before'][:2],data['after'][:2])
        except (ValueError,OverflowError) as error:
            invalid.append(dict(at=row['monotonic_ns'],reason=str(error),model=model));continue
        maximum_bound=max(maximum_bound,sweep['bound_m'])
        if sweep['status']=='collision':violations.append(dict(at=row['monotonic_ns'],**sweep,pose=data['after']))
        elif sweep['status']=='uncertain':uncertain.append(dict(at=row['monotonic_ns'],**sweep))
        if 'executed' in data and 'dt' in data:
            reverse_distance+=max(0.,-data['executed'][0])*data['dt'];lateral_distance+=abs(data['executed'][1])*data['dt']
        else:missing_velocity+=1
    return dict(initial_classification=initial,steps=len(steps),initial_overlap=initial in ('occupied','unknown'),
        initial_outside_map=initial=='outside_map',trajectory_confirmed_collisions=len(violations),sweep_uncertainties=len(uncertain),
        violations=violations[:30],uncertainties=uncertain[:30],reverse_distance_m=reverse_distance,lateral_distance_m=lateral_distance,
        path_length_m=path_length,maximum_sweep_bound_m=maximum_bound,trajectory_models=models,
        trajectory_validation_failures=invalid[:30],trajectory_validation_failure_count=len(invalid),
        maximum_endpoint_position_error_m=maximum_position_error,maximum_endpoint_yaw_error_rad=maximum_yaw_error,
        velocity_integral_metrics_complete=missing_velocity==0,
        geometry_pass=bool(steps) and initial=='free' and not violations and not uncertain and not invalid,
        note='Independent SAT; new wall-interval segments use exact constant body twist and vertex-speed bounds. Segments missing interval/velocity metadata retain legacy chord semantics, not an arc certificate.')
