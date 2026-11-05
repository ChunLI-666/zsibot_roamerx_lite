#!/usr/bin/env python3
"""Offline guard probes only: creates no ROS node and sends no goal."""
import copy,json,pathlib,tempfile,time,types
import pilot
cases=[]
def check(name,fn):
 fn();cases.append({'case':name,'passed':True})
with tempfile.TemporaryDirectory(dir=pilot.HERE) as tmp:
 p=pathlib.Path(tmp)/'ready.json'
 def ready():
  now=time.time_ns();return dict(ready=True,time_ns=now,valid_until_ns=now+500_000_000,variant='official',action_server='/navigate_to_pose',goal_tolerances=dict(xy=.15,yaw=.08,stateful=False),params_sha256='a'*64,build_receipt_sha256='b'*64)
 def accept():
  d=ready();p.write_text(json.dumps(d));actual,binding,h=pilot.read_ready(p,'official');assert actual['goal_tolerances']['xy']==.15 and len(h)==64
 check('current_500ms_lease_and_runtime_tolerance',accept)
 def reject(change,expected=None):
  d=ready();change(d);p.write_text(json.dumps(d))
  try:pilot.read_ready(p,'official',expected)
  except ValueError:return
  raise AssertionError('bad lease accepted')
 for name,change in [('expired',lambda d:d.update(valid_until_ns=time.time_ns()-1)),('false',lambda d:d.update(ready=False)),('future',lambda d:d.update(time_ns=time.time_ns()+1_000_000_000,valid_until_ns=time.time_ns()+1_500_000_000)),('variant',lambda d:d.update(variant='segmented')),('invalid_tolerance',lambda d:d['goal_tolerances'].update(xy=1.)),('missing_lease',lambda d:d.pop('valid_until_ns'))]:check(name,lambda c=change:reject(c))
 p.write_text(json.dumps(ready()));_,binding,_=pilot.read_ready(p,'official')
 check('binding_change',lambda:reject(lambda d:d.update(params_sha256='c'*64),binding))
 sample=dict(counts={},last={
  'pose':dict(receive_age_s=0.,header_age_s=0.,frame_id='map',x=1.,y=1.,yaw=0.),
  'odom':dict(receive_age_s=0.,header_age_s=0.,frame_id='odom',child_frame_id='base_link',vx=0.,vy=0.,wz=0.),
  'loc_status':dict(receive_age_s=0.,value=1),'pose_valid':dict(receive_age_s=0.,value=True),
  'scan':dict(receive_age_s=0.,header_age_s=0.,frame_id='base_link',ranges=100),
  'camera':dict(receive_age_s=0.,header_age_s=0.,decoded_shape=[360,640,3]),
  'camera_info':dict(receive_age_s=0.,width=640,height=360,fx=300.,fy=300.),
  '/cmd_vel_safe':dict(receive_age_s=0.,vx=0.,vy=0.,wz=0.)},
  tf={k:dict(regressed=False,header_age_s=0.,advance_age_s=0.) for k in ['map->odom','odom->base_link']})
 def input_check(change=None,expected=False):
  s=copy.deepcopy(sample)
  if change:change(s)
  ok,issues,_=pilot.Pilot.input_ready(types.SimpleNamespace(current_summary=lambda:s),require_stopped=True)
  assert ok==expected,(ok,issues)
 check('all_live_sources_valid',lambda:input_check(expected=True))
 for name,change in [('camera_header_stale',lambda s:s['last']['camera'].update(header_age_s=.6)),('camera_receipt_stale',lambda s:s['last']['camera'].update(receive_age_s=.6)),('invalid_pose_valid',lambda s:s['last']['pose_valid'].update(value=False)),('stale_scan',lambda s:s['last']['scan'].update(header_age_s=.4)),('stale_odom',lambda s:s['last']['odom'].update(header_age_s=.4)),('frozen_tf',lambda s:s['tf']['map->odom'].update(advance_age_s=.4))]:check(name,lambda c=change:input_check(c))
out=pilot.HERE/'guard_validation.json';out.write_text(json.dumps(dict(scope='Offline pure-function probes, no ROS node or goal',cases=cases),indent=2)+'\n');print(json.dumps({'passed':len(cases),'path':str(out)}))
