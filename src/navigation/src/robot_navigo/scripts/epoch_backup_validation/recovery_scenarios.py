"""Deterministic recovery injections and independent event-sequence checks."""
import copy
import math
import time


def mark_rear_wall(scene, pose, polygon, clearance=.03):
    """Occupy cells beyond the actual rear polygon, never inside initial geometry.

    This scenario is explicitly straight and axis aligned. Rounding moves the
    wall farther away; the returned physical front clearance is the authority.
    """
    if abs(pose[2]) > 1e-6:
        raise ValueError('Rear-wall injection requires axis-aligned backup')
    rear=pose[0]+min(float(point[0]) for point in polygon)
    resolution=scene.resolution
    col=math.floor((rear-clearance-scene.origin[0])/resolution)-1
    lo=math.floor((pose[1]-.6-scene.origin[1])/resolution)
    hi=math.floor((pose[1]+.6-scene.origin[1])/resolution)
    height,width=scene.free.shape
    if not 0<=col<width or lo<0 or hi>=height:
        raise ValueError('Dynamic wall would leave the scene')
    cells=[]
    for row in range(lo,hi+1):
        scene.occupancy[row,col]=1;scene.free[row,col]=False
        cells.append([col,row])
    front=scene.origin[0]+(col+1)*resolution
    return dict(cells=cells,world_front_x=front,rear_x=rear,clearance_m=rear-front,
                requested_clearance_m=clearance,resolution=resolution,pose=list(pose),
                world_cell_centers=[[float(scene.origin[0]+(col+.5)*resolution),float(scene.origin[1]+(row+.5)*resolution)] for col,row in cells])


def token_key(token):
    return (token['navigation_session_id'],token['task_sequence'],token['plan_sequence'],token['execution_kind'])


def audit_bt_recovery(rows):
    states=[r for r in rows if r['event']=='/nav_epoch/execution']
    backup=[r for r in states if r['data']['active'] and r['data']['token']['execution_kind']==1]
    if not backup:raise AssertionError('Real BT never installed BackUpEpoch')
    first=backup[0];token=first['data']['token'];key=token_key(token)
    previous=[r for r in states if r['monotonic_ns']<first['monotonic_ns'] and r['data']['active'] and r['data']['token']['execution_kind']==0]
    if not previous:raise AssertionError('No ordinary TRACK execution before automatic recovery')
    failure=[r for r in states if r['monotonic_ns']<first['monotonic_ns'] and 'progress' in r['data'].get('reason','').lower()]
    if not failure:raise AssertionError('No real progress-checker failure before BackUp')
    finished=[r for r in states if not r['data']['active'] and r['data'].get('reason')=='backup_finished' and token_key(r['data']['token'])==key]
    if not finished:raise AssertionError('BackUp authority was not sealed')
    terminal=[r for r in rows if r['event']=='/back_up_epoch/_action/status' and any(s['status']==4 for s in r['data']['status_list'])]
    if not terminal:raise AssertionError('Real BackUp action did not reach SUCCEEDED')
    after=[r for r in states if r['monotonic_ns']>finished[0]['monotonic_ns'] and r['data']['active'] and r['data']['token']['execution_kind']==0]
    if not after:raise AssertionError('No new TRACK installed after BackUp')
    next_token=after[0]['data']['token']
    if next_token['navigation_session_id']!=token['navigation_session_id'] or next_token['task_sequence']!=token['task_sequence'] or next_token['plan_sequence']<=token['plan_sequence']:
        raise AssertionError('Automatic recovery lost mission identity or reused request sequence')
    return dict(first_track_token=previous[0]['data']['token'],progress_failure=failure[-1],
                backup_token=token,backup_finished_ns=finished[0]['monotonic_ns'],resumed_track_token=next_token,
                automatic_recovery=True)


class ScanTiming:
    """Wrap scan computation without editing the shared fixture/scene modules."""
    def __init__(self,scene):
        self.scene=scene;self.original=scene.scan;self.samples=[]
    def scan(self,pose):
        before=time.monotonic_ns();result=self.original(pose);after=time.monotonic_ns()
        self.samples.append(dict(start_ns=before,end_ns=after,duration_ns=after-before))
        return result


def audit_dynamic_geometry(rows,before_scene,after_scene,polygon,injection):
    """Score each full plant interval against the world valid during that interval."""
    from geometry_audit import audit_geometry
    boundary=injection['effective_ns'];before=[];after=[]
    for row in rows:
        if row['event']!='plant_step':continue
        data=row['data'];start=data['interval_start_ns'];end=data['interval_end_ns']
        if end<=boundary:before.append(row)
        elif start>=boundary:after.append(row)
        else:raise AssertionError('Plant interval crosses dynamic-world mutation without a split')
    start=next(row for row in rows if row['event']=='fixture_start')
    pre=audit_geometry([start]+before,before_scene,polygon)
    post_start=dict(event='fixture_start',data=dict(pose=injection['pose']),monotonic_ns=boundary)
    post=audit_geometry([post_start]+after,after_scene,polygon)
    return dict(geometry_pass=pre['geometry_pass'] and post['geometry_pass'],before_injection=pre,after_injection=post,
                trajectory_confirmed_collisions=pre['trajectory_confirmed_collisions']+post['trajectory_confirmed_collisions'],
                sweep_uncertainties=pre['sweep_uncertainties']+post['sweep_uncertainties'],
                path_length_m=pre['path_length_m']+post['path_length_m'],effective_obstacle_ns=boundary,
                note='Each interval uses its time-valid occupancy; no final-map retrospective substitution.')


class CostmapVisibility:
    """Observe real local costmap occupancy, retaining query-time uncertainty."""
    def __init__(self,fixture,injection):
        from nav2_msgs.srv import GetCostmap
        self.fixture=fixture;self.injection=injection;self.request_type=GetCostmap.Request
        self.client=fixture.create_client(GetCostmap,'/local_costmap/get_costmap')
        self.pending=None;self.request_ns=None;self.samples=[];self.last_absent_ns=injection['effective_ns']
        self.timer=fixture.create_timer(.02,self.poll)
        self.poll()
    def poll(self):
        if self.pending is not None and self.pending.done():
            response=self.pending.result();self.pending=None
            if response is not None:
                self.fixture.flush_to_now();at=time.monotonic_ns();grid=response.map;meta=grid.metadata
                values=[]
                for x,y in self.injection['world_cell_centers']:
                    col=math.floor((x-meta.origin.position.x)/meta.resolution)
                    row=math.floor((y-meta.origin.position.y)/meta.resolution)
                    if 0<=col<meta.size_x and 0<=row<meta.size_y:values.append(int(grid.data[row*meta.size_x+col]))
                visible=any(cost==254 for cost in values)
                sample=dict(request_ns=self.request_ns,response_ns=at,visible=visible,costs=values,
                    costmap_update_stamp_ns=meta.update_time.sec*1_000_000_000+meta.update_time.nanosec,
                    actual_velocity=self.fixture._executed_velocity.copy(),pose=self.fixture.pose.copy(),
                    visibility_window_ns=[self.last_absent_ns,at])
                self.samples.append(sample);self.fixture.record('dynamic_obstacle_costmap',sample)
                if not visible:self.last_absent_ns=self.request_ns
        if self.pending is None and self.client.service_is_ready():
            self.request_ns=time.monotonic_ns();self.pending=self.client.call_async(self.request_type())
    def close(self):
        self.timer.cancel()
        first=next((s for s in self.samples if s['visible']),None)
        moving=next((s for s in self.samples if s['visible'] and s['actual_velocity'][0]<-.005),None)
        return dict(samples=self.samples,first_confirmed_visible=first,visible_while_reversing=moving is not None)
