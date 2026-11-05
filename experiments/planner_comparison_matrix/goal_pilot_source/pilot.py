#!/usr/bin/env python3
"""Independent, single-case Matrix navigation pilot.

Default is observation only. --execute requires a true launcher ready.json,
domain 185, fresh real Lightning inputs, a stopped command chain and a single
explicit map-frame goal. It never starts navigation, mapping, a recorder or a
robot command adapter. Only this client's own action may be canceled.
"""
import argparse
from collections import Counter, deque
import hashlib
import json
import math
import os
from pathlib import Path
import time
import uuid

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Log
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from std_msgs.msg import String, UInt8, Bool
from tf2_msgs.msg import TFMessage
from unique_identifier_msgs.msg import UUID
from action_msgs.srv import CancelGoal

HERE = Path(__file__).resolve().parent


def read_ready(path, variant, binding=None):
    """Read the atomic launcher lease afresh; runtime identity cannot change mid-case."""
    data = path.read_bytes()
    ready = json.loads(data)
    now = time.time_ns()
    issued, expiry = ready.get('time_ns', 0), ready.get('valid_until_ns', 0)
    if ready.get('ready') is not True:
        raise ValueError('launcher_not_ready:'+str(ready.get('reason')))
    if not isinstance(issued, int) or not isinstance(expiry, int) or not 0 < expiry-now <= 550_000_000 or not -50_000_000 <= now-issued <= 500_000_000:
        raise ValueError('launcher_lease_missing_expired_or_future')
    if ready.get('variant') != variant or ready.get('action_server') != '/navigate_to_pose':
        raise ValueError('launcher_variant_or_action_mismatch')
    tolerance = ready.get('goal_tolerances', {})
    for key in ['xy', 'yaw']:
        value = tolerance.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= .25:
            raise ValueError('invalid_launcher_goal_tolerance:'+key)
    if tolerance.get('stateful') is not False:
        raise ValueError('launcher_final_goal_checker_must_be_nonstateful')
    immutable = {k:ready.get(k) for k in ['variant', 'action_server', 'goal_tolerances', 'params_sha256', 'build_receipt_sha256']}
    for key in ['params_sha256', 'build_receipt_sha256']:
        value = immutable[key]
        if not isinstance(value, str) or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
            raise ValueError('missing_runtime_binding:'+key)
    if binding is not None and binding != immutable:
        raise ValueError('launcher_runtime_binding_changed')
    return ready, immutable, hashlib.sha256(data).hexdigest()


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def stamp_ns(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


def error(pose, goal):
    if pose is None:
        return None
    return {'xy_m': math.hypot(pose['x']-goal[0], pose['y']-goal[1]),
            'yaw_rad': abs(math.atan2(math.sin(pose['yaw']-goal[2]), math.cos(pose['yaw']-goal[2]))),
            'pose': pose}


class Pilot(Node):
    def __init__(self, args, output):
        super().__init__('matrix_independent_goal_pilot')
        self.args, self.output = args, output
        self.case_uuid = str(uuid.uuid4())
        self.events = (output/'events.jsonl').open('a', buffering=1)
        self.counts = Counter()
        self.last = {}
        self.transforms = {}
        self.odom_history = deque(maxlen=10000)
        self.goal_sent_steady = None
        self.goal_sent_ns = None
        self.goal_uuid = None
        self.action_goal_handle = None
        self.action_result = None
        self.first_target_matching_plan = None
        self.runtime_binding = None
        self.artifact_invalid_reasons = []
        self.event_pub = self.create_publisher(String, '/experiment/goal_pilot/events', 50) if args.execute else None
        self.subscriptions_kept = []
        self.sub( PoseStamped, '/lightning/pose', self.pose_cb)
        self.sub( Odometry, args.odom_topic, self.odom_cb)
        self.sub( UInt8, '/lightning/loc_status', lambda m: self.remember('loc_status', {'value': int(m.data)}))
        self.sub(Bool, '/lightning/pose_valid', lambda m: self.remember('pose_valid', {'value': bool(m.data)}))
        self.sub(LaserScan, args.scan_topic, lambda m: self.remember('scan', {
            'stamp_ns':stamp_ns(m.header.stamp), 'frame_id':m.header.frame_id, 'ranges':len(m.ranges)}))
        for topic in ['/cmd_vel_nav', '/cmd_vel', '/cmd_vel_safe']:
            self.sub(Twist, topic, lambda m,t=topic: self.remember(t, {'vx':m.linear.x,'vy':m.linear.y,'wz':m.angular.z}))
        self.sub(TFMessage, '/tf', self.tf_cb)
        self.sub(CompressedImage, args.camera_topic, self.camera_cb)
        self.sub(CameraInfo, args.camera_info_topic, self.info_cb)
        self.sub(NavPath, '/plan', self.plan_cb)
        self.sub(Log, '/rosout', self.log_cb)

    def sub(self, cls, topic, callback):
        self.subscriptions_kept.append(self.create_subscription(cls, topic, callback, qos_profile_sensor_data))

    def remember(self, key, value):
        self.counts[key] += 1
        value = dict(value, received_steady=time.monotonic(), received_unix_ns=time.time_ns())
        self.last[key] = value
        return value

    def pose_cb(self, msg):
        p = msg.pose
        self.remember('pose', {'frame_id':msg.header.frame_id, 'stamp_ns':stamp_ns(msg.header.stamp),
                              'x':p.position.x,'y':p.position.y,'yaw':yaw(p.orientation)})

    def odom_cb(self, msg):
        v = msg.twist.twist
        value = self.remember('odom', {'frame_id':msg.header.frame_id,'child_frame_id':msg.child_frame_id,
            'stamp_ns':stamp_ns(msg.header.stamp),'vx':v.linear.x,'vy':v.linear.y,'wz':v.angular.z})
        self.odom_history.append(value)

    def tf_cb(self, msg):
        now = time.monotonic()
        for t in msg.transforms:
            key = (t.header.frame_id, t.child_frame_id)
            if key not in [('map','odom'),('odom','base_link')]:
                continue
            ns = stamp_ns(t.header.stamp)
            prior = self.transforms.get(key)
            advanced = now if prior is None or ns > prior['stamp_ns'] else prior['advanced_steady']
            self.transforms[key] = {'stamp_ns':ns,'received_steady':now,'advanced_steady':advanced,
                                    'regressed':bool(prior and ns < prior['stamp_ns'])}
            self.counts['tf:'+key[0]+'->'+key[1]] += 1

    def camera_cb(self, msg):
        value = {'stamp_ns':stamp_ns(msg.header.stamp),'frame_id':msg.header.frame_id,
                 'format':msg.format,'bytes':len(msg.data)}
        image = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        value['decoded_shape'] = None if image is None else list(image.shape)
        self.remember('camera', value)

    def info_cb(self, msg):
        self.remember('camera_info', {'stamp_ns':stamp_ns(msg.header.stamp),'frame_id':msg.header.frame_id,
            'width':msg.width,'height':msg.height,'fx':msg.k[0],'fy':msg.k[4]})

    def plan_cb(self, msg):
        self.counts['plan'] += 1
        if not self.args.execute or self.goal_sent_ns is None or not msg.poses:
            return
        p = msg.poses[-1].pose
        end = {'x':p.position.x,'y':p.position.y,'yaw':yaw(p.orientation)}
        e = error(end, self.args.goal)
        matches = msg.header.frame_id == 'map' and e['xy_m'] <= .02 and e['yaw_rad'] <= .10
        item = {'frame_id':msg.header.frame_id,'stamp_ns':stamp_ns(msg.header.stamp),
                'poses':len(msg.poses),'endpoint':end,'target_match_inferred':matches,
                'association':'Geometry/time inference only; /plan has no Action UUID.'}
        self.emit('plan_observation', **item)
        if matches and self.first_target_matching_plan is None:
            self.first_target_matching_plan = dict(item, latency_s=time.monotonic()-self.goal_sent_steady)

    def log_cb(self, msg):
        if not self.args.execute or self.goal_sent_ns is None:
            return
        if msg.level >= 30 and any(x in msg.name for x in ['planner','controller','navigator','BehaviorTree']):
            self.emit('runtime_log', source_node=msg.name, level=int(msg.level),
                      source_stamp_ns=stamp_ns(msg.stamp), text=msg.msg,
                      association='Temporal only; retain explicit target/error text for offline attribution.')

    def emit(self, kind, **fields):
        event = {'kind':kind,'case_uuid':self.case_uuid,'case_id':self.args.case_id,
                 'variant':self.args.variant,'unix_ns':time.time_ns(),'steady_ns':time.monotonic_ns(),**fields}
        line = json.dumps(event, ensure_ascii=False, allow_nan=False)
        self.events.write(line+'\n')
        if self.event_pub:
            msg = String();msg.data = line;self.event_pub.publish(msg)
        return event

    def current_summary(self):
        now_ns = self.get_clock().now().nanoseconds
        now = time.monotonic()
        snapshot = {k:dict(v) for k,v in self.last.items()}
        for value in snapshot.values():
            value['receive_age_s'] = now-value['received_steady']
            if 'stamp_ns' in value:
                value['header_age_s'] = (now_ns-value['stamp_ns'])/1e9
        return {'counts':dict(self.counts),'last':snapshot,'tf':{a+'->'+b:dict(v,header_age_s=(now_ns-v['stamp_ns'])/1e9,
                advance_age_s=now-v['advanced_steady']) for (a,b),v in self.transforms.items()}}

    def input_ready(self, require_camera=True, require_stopped=False):
        s = self.current_summary();issues=[]
        for key in ['pose','odom','loc_status','pose_valid','scan']:
            value=s['last'].get(key)
            if value is None or value['receive_age_s'] > .3:
                issues.append(key+':missing_or_stale_receive');continue
            if 'header_age_s' in value and not -.05 <= value['header_age_s'] <= .3:
                issues.append(key+':invalid_header_age')
        if s['last'].get('loc_status',{}).get('value') != 1:issues.append('localization_not_NORMAL')
        if s['last'].get('pose_valid',{}).get('value') is not True:issues.append('localization_pose_invalid')
        if s['last'].get('pose',{}).get('frame_id') != 'map':issues.append('pose_not_map')
        if s['last'].get('odom',{}).get('frame_id') != 'odom':issues.append('odom_frame_not_odom')
        if s['last'].get('odom',{}).get('child_frame_id') != 'base_link':issues.append('odom_child_not_base_link')
        if s['last'].get('scan',{}).get('frame_id') != 'base_link' or s['last'].get('scan',{}).get('ranges',0)==0:issues.append('scan_frame_or_payload_invalid')
        for key, fields in [('pose',['x','y','yaw']),('odom',['vx','vy','wz'])]:
            v=s['last'].get(key,{})
            if not all(math.isfinite(v.get(f,math.nan)) for f in fields):issues.append(key+':nonfinite_value')
        for key in ['map->odom','odom->base_link']:
            t=s['tf'].get(key)
            if t is None or t['regressed'] or not -.05 <= t['header_age_s'] <= .3 or t['advance_age_s'] > .3:
                issues.append(key+':missing_stale_future_or_regressed')
        if require_camera:
            c=s['last'].get('camera',{});info=s['last'].get('camera_info',{})
            if c.get('receive_age_s',math.inf)>.5 or not -.05<=c.get('header_age_s',math.inf)<=.5 or not c.get('decoded_shape'):
                issues.append('camera_missing_stale_or_decode_failed')
            if not info.get('width') or not info.get('height') or info.get('fx',0)<=0 or info.get('fy',0)<=0:
                issues.append('camera_info_missing_or_invalid')
            elif c.get('decoded_shape') and c['decoded_shape'][:2] != [info['height'],info['width']]:
                issues.append('camera_shape_mismatch')
        if require_stopped:
            for key, linear_limit, angular_limit in [('odom',.03,.05),('/cmd_vel_safe',.01,.01)]:
                v=s['last'].get(key,{})
                if v.get('receive_age_s',math.inf)>.3 or math.hypot(v.get('vx',math.inf),v.get('vy',math.inf))>linear_limit or abs(v.get('wz',math.inf))>angular_limit:
                    issues.append(key+':not_fresh_stopped')
        return not issues,issues,s

    def execution_ready(self, require_stopped=False):
        valid, issues, snapshot = self.input_ready(require_camera=True, require_stopped=require_stopped)
        if self.args.execute:
            try:
                self.latest_ready, binding, self.latest_ready_sha = read_ready(self.args.ready, self.args.variant, self.runtime_binding)
                if self.runtime_binding is None:self.runtime_binding = binding
            except Exception as exc:issues.append('readiness:'+str(exc))
            if self.event_pub.get_subscription_count()==0:issues.append('goal_event_subscriber_disappeared')
        return not issues, issues, snapshot

    def invalidate_artifact(self, issues):
        new = [reason for reason in issues if reason not in self.artifact_invalid_reasons]
        if new:
            self.artifact_invalid_reasons.extend(new)
            self.emit('artifact_invalid', issues=new, scope='Input/recording/lease failure; do not attribute this attempt to planner capability alone.')

    def spin_until(self, predicate, timeout):
        end=time.monotonic()+timeout
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.02)
            if predicate():return True
        return False

    def wait_settled(self, seconds):
        stable_since=None;end=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.02)
            healthy,health_issues,_=self.execution_ready()
            if not healthy:self.invalidate_artifact(health_issues)
            valid,issues,s=self.execution_ready(require_stopped=True)
            if valid:
                if stable_since is None:stable_since=time.monotonic()
                if time.monotonic()-stable_since>=.3:return True,s
            else:stable_since=None
        return False,self.current_summary()

    def run_case(self):
        self.emit('case_created', role=self.args.kind,requested_goal_map=self.args.goal,
                  requested_goal_relative=self.args.relative_goal,
                  evidence_boundary='Real simulation inputs required; no ground-truth TF or pose substituted.')
        if not self.spin_until(lambda:self.event_pub.get_subscription_count()>0,5.):
            self.emit('preflight_failed',issues=['goal_event_topic_has_no_recorder_subscriber'])
            return {'classification':'invalid_event_recording_not_ready_goal_not_sent'}
        if not self.spin_until(lambda:self.execution_ready(require_stopped=True)[0],self.args.preflight_seconds):
            valid,issues,s=self.execution_ready(require_stopped=True)
            self.emit('preflight_failed',issues=issues,snapshot=s)
            return {'classification':'invalid_preflight_not_sent','issues':issues,'snapshot':s}
        valid,issues,start_snapshot=self.execution_ready(require_stopped=True)
        start_pose=start_snapshot['last']['pose']
        if self.args.relative_goal is None and math.hypot(self.args.goal[0]-start_pose['x'],self.args.goal[1]-start_pose['y'])>=8.0:
            return {'classification':'invalid_goal_outside_reviewed_rolling_window','snapshot':start_snapshot}
        client=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        if not client.wait_for_server(timeout_sec=5.):
            return {'classification':'invalid_action_server_unavailable','snapshot':start_snapshot}
        # Action discovery can block. Pump and re-read the lease and live sources
        # immediately before creating the request instead of using the earlier snapshot.
        self.spin_until(lambda:self.execution_ready(require_stopped=True)[0],1.)
        valid,issues,start_snapshot=self.execution_ready(require_stopped=True)
        if not valid:
            self.emit('preflight_failed',issues=issues,snapshot=start_snapshot)
            return {'classification':'invalid_final_preflight_not_sent','issues':issues,'snapshot':start_snapshot}
        if self.args.relative_goal is not None:
            start_pose=start_snapshot['last']['pose'];dx,dy,dyaw=self.args.relative_goal;c=math.cos(start_pose['yaw']);s=math.sin(start_pose['yaw'])
            self.args.goal=[start_pose['x']+c*dx-s*dy,start_pose['y']+s*dx+c*dy,
                math.atan2(math.sin(start_pose['yaw']+dyaw),math.cos(start_pose['yaw']+dyaw))]
        if math.hypot(self.args.goal[0]-start_snapshot['last']['pose']['x'],self.args.goal[1]-start_snapshot['last']['pose']['y'])>=8.0:
            return {'classification':'invalid_goal_outside_reviewed_rolling_window','snapshot':start_snapshot}
        ready=self.latest_ready
        self.emit('case_recording_ready',role=self.args.kind,requested_goal_map=self.args.goal,
                  readiness_manifest=ready,readiness_manifest_sha256=self.latest_ready_sha,
                  goal_tolerances=self.runtime_binding['goal_tolerances'],event_subscriber_count=self.event_pub.get_subscription_count())
        request=NavigateToPose.Goal();request.pose=PoseStamped();request.pose.header.frame_id='map'
        request.pose.header.stamp=self.get_clock().now().to_msg()
        request.pose.pose.position.x=self.args.goal[0]
        request.pose.pose.position.y=self.args.goal[1]
        request.pose.pose.orientation.z=math.sin(self.args.goal[2]/2.)
        request.pose.pose.orientation.w=math.cos(self.args.goal[2]/2.)
        requested_uuid=uuid.uuid4();ros_uuid=UUID(uuid=list(requested_uuid.bytes))
        self.goal_uuid=ros_uuid
        self.goal_sent_steady=time.monotonic();self.goal_sent_ns=time.time_ns()
        self.emit('goal_send',action_uuid=str(requested_uuid),pose_header_stamp_ns=stamp_ns(request.pose.header.stamp),
                  requested_goal_map=self.args.goal,start_snapshot=start_snapshot)
        sent=client.send_goal_async(request,goal_uuid=ros_uuid,
            feedback_callback=lambda message:self.emit('action_feedback',distance_remaining=float(message.feedback.distance_remaining),
                navigation_time_s=message.feedback.navigation_time.sec+message.feedback.navigation_time.nanosec/1e9))
        if not self.spin_until(sent.done,10.):
            self.emit('goal_response_timeout',action_uuid=str(requested_uuid))
            # Target ONLY the UUID sent by this pilot, even if its goal response
            # was lost. Never cancel all actions owned by other clients.
            service=self.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
            cancel_request=CancelGoal.Request();cancel_request.goal_info.goal_id=ros_uuid
            if service.wait_for_service(timeout_sec=2.):
                canceled=service.call_async(cancel_request)
                observed=self.spin_until(canceled.done,3.)
                self.emit('cancel_unacknowledged_goal',action_uuid=str(requested_uuid),response_observed=observed,
                    return_code=int(canceled.result().return_code) if observed else None)
            return {'classification':'goal_response_timeout_acceptance_unknown','action_uuid':str(requested_uuid)}
        self.action_goal_handle=sent.result()
        self.emit('goal_response',accepted=bool(self.action_goal_handle.accepted),action_uuid=str(requested_uuid))
        if not self.action_goal_handle.accepted:
            return {'classification':'goal_rejected','action_uuid':str(requested_uuid)}
        result_future=self.action_goal_handle.get_result_async();cancel_reason=None;cancel_future=None
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self,timeout_sec=.02)
            elapsed=time.monotonic()-self.goal_sent_steady
            valid,issues,_=self.execution_ready()
            if not valid:self.invalidate_artifact(issues)
            reason = 'input_became_invalid:'+','.join(issues) if not valid else None
            if self.args.cancel_after is not None and elapsed>=self.args.cancel_after:reason='scheduled_cancel'
            if elapsed>=self.args.timeout:reason='pilot_navigation_deadline'
            if reason and cancel_future is None:
                cancel_reason=reason;self.emit('cancel_send',action_uuid=str(requested_uuid),reason=reason)
                cancel_future=self.action_goal_handle.cancel_goal_async()
            if cancel_future is not None and cancel_future.done() and not getattr(self,'cancel_logged',False):
                self.cancel_logged=True
                response=cancel_future.result()
                self.emit('cancel_response',return_code=int(response.return_code),goals_canceling=[bytes(g.goal_id.uuid).hex() for g in response.goals_canceling])
            if elapsed>=self.args.timeout+10.:
                self.emit('terminal_result_timeout',action_uuid=str(requested_uuid));break
        terminal_snapshot=self.current_summary();terminal_pose=terminal_snapshot['last'].get('pose')
        valid,issues,_=self.execution_ready()
        if not valid:self.invalidate_artifact(issues)
        if result_future.done():
            result=result_future.result();self.action_result=result
            result_fields={'status':int(result.status),'error_code':getattr(result.result,'error_code',None),
                           'error_msg':getattr(result.result,'error_msg',None)}
        else:result_fields={'status':None,'error_code':None,'error_msg':'No terminal response observed'}
        self.emit('action_result',action_uuid=str(requested_uuid),**result_fields,terminal_snapshot=terminal_snapshot)
        settled,settled_snapshot=self.wait_settled(self.args.settle_seconds)
        settled_pose=settled_snapshot['last'].get('pose')
        terminal_error=error(terminal_pose,self.args.goal);settled_error=error(settled_pose,self.args.goal)
        tolerance=self.runtime_binding['goal_tolerances']
        within=lambda e:e is not None and e['xy_m']<=tolerance['xy'] and e['yaw_rad']<=tolerance['yaw'] and e['pose'].get('receive_age_s',math.inf)<=.3 and -.05<=e['pose'].get('header_age_s',math.inf)<=.3
        outcome={'case_uuid':self.case_uuid,'case_id':self.args.case_id,'variant':self.args.variant,'kind':self.args.kind,
            'action_uuid':str(requested_uuid),'requested_goal_map':self.args.goal,**result_fields,
            'cancel_reason':cancel_reason,'terminal_original_goal_error':terminal_error,
            'goal_tolerances':tolerance,'artifact_valid':not self.artifact_invalid_reasons,'artifact_invalid_reasons':self.artifact_invalid_reasons,
            'settled_original_goal_error':settled_error,'settled_measured_and_commanded_stop':settled,
            'terminal_within_original_tolerance':within(terminal_error),'settled_within_original_tolerance':within(settled_error),
            'goal_to_first_target_matching_plan':self.first_target_matching_plan,
            'planning_measurement_boundary':'Observed /plan latency includes BT/action transport and is not planner CPU/search duration. /plan association is geometric, not UUID proof.',
            'classification':'invalid_runtime_evidence' if self.artifact_invalid_reasons else 'completed_action_success_and_original_goal_pass' if result_fields['status']==4 and settled and within(terminal_error) and within(settled_error)
                else 'action_success_but_independent_acceptance_failed' if result_fields['status']==4 else 'canceled_by_pilot' if cancel_reason else 'completed_action_failure_or_missing_result',
            'final_snapshot':settled_snapshot,'runtime_binding':ready}
        self.emit('case_finished',outcome=outcome)
        self.spin_until(lambda:False,.2)
        return outcome


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--ready',type=Path)
    goal_arguments=parser.add_mutually_exclusive_group()
    goal_arguments.add_argument('--goal',type=float,nargs=3)
    goal_arguments.add_argument('--relative-goal',type=float,nargs=3,metavar=('DX','DY','DYAW'),
        help='Body-frame translation and relative yaw resolved from the final live preflight pose, then frozen as an explicit map goal')
    parser.add_argument('--case-id',default='observation')
    parser.add_argument('--variant',choices=['official','segmented','hybrid'],default='official')
    parser.add_argument('--kind',choices=['benchmark','setup','fault'],default='benchmark')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--observe-seconds',type=float,default=8.)
    parser.add_argument('--preflight-seconds',type=float,default=10.)
    parser.add_argument('--timeout',type=float,default=120.)
    parser.add_argument('--settle-seconds',type=float,default=8.)
    parser.add_argument('--cancel-after',type=float)
    parser.add_argument('--odom-topic',default='/odom/current_pose')
    parser.add_argument('--scan-topic',default='/laser_scan')
    parser.add_argument('--camera-topic',default='/camera/front/image/compressed')
    parser.add_argument('--camera-info-topic',default='/camera/front/camera_info')
    args=parser.parse_args()
    args.output=args.output.resolve()
    if HERE not in args.output.parents:parser.error('Output must be inside this pilot directory')
    if args.output.exists():parser.error('Output already exists; do not overwrite an attempt')
    args.output.mkdir(parents=True)
    created_ns=time.time_ns();attempt_uuid=str(uuid.uuid4())
    source=Path(__file__).read_bytes();(args.output/'pilot_snapshot.py').write_bytes(source)
    invocation={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()}
    (args.output/'invocation.json').write_text(json.dumps(dict(created_ns=created_ns,attempt_uuid=attempt_uuid,args=invocation,
        pilot_sha256=hashlib.sha256(source).hexdigest()),indent=2)+'\n')
    node=None;exit_code=0
    try:
        if os.environ.get('ROS_DOMAIN_ID')!='185':raise ValueError('Only isolated Matrix domain185 is supported')
        if args.execute and (args.ready is None or args.goal is None and args.relative_goal is None):raise ValueError('--execute requires --ready and --goal or --relative-goal')
        if args.goal is not None and not all(math.isfinite(v) for v in args.goal):raise ValueError('Goal must be finite map SE2')
        if args.relative_goal is not None and not all(math.isfinite(v) for v in args.relative_goal):raise ValueError('Relative goal must be finite SE2')
        rclpy.init();node=Pilot(args,args.output)
        if args.execute:
            result=node.run_case()
        else:
            node.spin_until(lambda:False,args.observe_seconds)
            valid,issues,snapshot=node.input_ready(require_stopped=True)
            result={'mode':'observe_only','goals_sent':0,'ready_for_new_goal':valid,'issues':issues,'snapshot':snapshot}
    except BaseException as exc:
        exit_code=2
        result={'classification':'pilot_exception_or_invalid_invocation','exception_type':type(exc).__name__,'exception_message':str(exc)}
        if node is not None:
            node.emit('pilot_exception',type=type(exc).__name__,message=str(exc))
            if node.action_goal_handle is not None and node.action_goal_handle.accepted and node.action_result is None:
                cancel=node.action_goal_handle.cancel_goal_async();result['exception_cancel_response_observed']=node.spin_until(cancel.done,3.)
            elif node.goal_uuid is not None and node.action_result is None:
                service=node.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
                request=CancelGoal.Request();request.goal_info.goal_id=node.goal_uuid
                if service.wait_for_service(timeout_sec=2.):
                    cancel=service.call_async(request);result['exception_uuid_cancel_response_observed']=node.spin_until(cancel.done,3.)
    finally:
        result.update(attempt_uuid=attempt_uuid,case_id=args.case_id,variant=args.variant,kind=args.kind,
            pilot_sha256=hashlib.sha256(source).hexdigest(),goals_sent=int(node is not None and node.goal_sent_ns is not None),created_ns=created_ns,finished_ns=time.time_ns())
        (args.output/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
        index={k:result.get(k) for k in ['attempt_uuid','case_id','variant','kind','classification','status','goals_sent','created_ns','finished_ns']}
        index['result_file']=str(args.output/'result.json')
        with (HERE/'attempt_index.jsonl').open('a') as stream:stream.write(json.dumps(index)+'\n')
        if node is not None:node.events.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    print(json.dumps({'result_file':str(args.output/'result.json'),'classification':result.get('classification'),
        'mode':result.get('mode'),'issues':result.get('issues'),'status':result.get('status'),
        'settled':result.get('settled_measured_and_commanded_stop'),
        'terminal_original_goal_error':result.get('terminal_original_goal_error'),
        'settled_original_goal_error':result.get('settled_original_goal_error')},ensure_ascii=False,allow_nan=False))
    return exit_code


if __name__=='__main__':raise SystemExit(main())
