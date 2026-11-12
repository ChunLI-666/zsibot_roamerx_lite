#!/usr/bin/env python3
"""Normal client decision/dispatch/active regressions on isolated localhost DDS.

Captured B41 bytes stay read-only. Generated inputs and the FollowWaypoints
server are explicit protocol fixtures, without a plant, simulator or robot.
"""
import argparse
from collections import deque
import copy
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch
import uuid


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--client-dir',type=Path,required=True)
    parser.add_argument('--evidence-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',required=True,choices=['normal','validation_odom','validation_tf',
        'validation_ready','presend','presend_fresh','active_pulse','active_health','active_validation'])
    args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='190'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(args.client_dir.resolve()))
    import cv2
    import numpy as np
    import rclpy
    from rclpy.action import ActionServer,CancelResponse,GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped,TransformStamped,Twist
    from nav_msgs.msg import Odometry,Path as NavPath
    from nav2_msgs.action import FollowWaypoints
    from sensor_msgs.msg import LaserScan,CompressedImage,CameraInfo
    from std_msgs.msg import String,UInt8,Bool
    from tf2_msgs.msg import TFMessage
    import mission_resume
    import waypoint_batch as batch
    from mission_resume import Session
    from mission_state import MissionStore,encoded,digest,boot_clock,atomic_json
    from mission_storage import StorageWorker
    from mission_continuation import CLIENT_SOURCES
    from waypoint_contract import simulation_scan_latency_profile

    root=args.evidence_root.resolve()
    profile=json.loads((root/'solmax_b41_stop_review_v1/PROFILE_RESULT.json').read_text())
    original_state_path=root/profile['profile_state']['path']
    captured=json.loads(original_state_path.read_bytes())
    event_path=root/profile['ready_capture']['path']
    for number,line in enumerate(event_path.open(),1):
        if number==profile['ready_capture']['line']:
            ready=copy.deepcopy(json.loads(line)['latest_ready']);break
    else:raise AssertionError('captured READY not found')
    (out/'captured_state.json').write_bytes(original_state_path.read_bytes())
    (out/'captured_READY.json').write_text(encoded(ready)+'\n')
    fresh=args.case.startswith(('presend','active_'))
    seed=MissionStore(out/'fixture.db')
    boot,now=boot_clock()
    if fresh:
        document=dict(schema_version=1,frame_id='map',mission_budget_sec=captured['policy']['mission_budget_sec'],
            waypoints=[{key:value for key,value in row.items() if key in ('id','pose','tags','require_stopped')}
                for row in captured['route']])
        seed.create(json.dumps(document).encode(),captured['identity'],captured['policy'])
    else:
        local=copy.deepcopy(captured)
        local['budgets'].update(deadline_boot_id=boot,deadline_boottime_sec=now+120)
        seed.db.execute('INSERT INTO mission VALUES (1,?,?)',(local['revision'],encoded(local)))
    seed.close()
    worker=StorageWorker(out/'fixture.db',out/'resource.lock');before=worker.state()
    session=Session(worker,str(uuid.uuid4()),out/'attempt')
    manifest=batch.freeze_sources(out,.3)
    ready['waypoint_contract']['client_sources_sha256']={
        'experiments/planner_comparison_matrix/goal_pilot_source/'+name:value for name,value in manifest['sources_sha256'].items()}
    ready['waypoint_contract']['mission_resume']['client_sources_sha256']={
        name:digest((args.client_dir/name).read_bytes()) for name in CLIENT_SOURCES}
    ready['simulation_scan_latency_profile']=simulation_scan_latency_profile(.3)
    ready['mission_identity']=dict(before['identity'],verified=True)
    ready['mission_deadline'].update(launcher_boot_id=boot,deadline_boottime_sec=now+120)
    ready['waypoint_ownership'].update(navigate_to_pose_active=[],follow_waypoints_active=[],follow_path_active=[])
    ready_path=out/'synthetic_READY.json'
    options=SimpleNamespace(execute=True,ready=ready_path,variant=before['variant'],case_id='decision_'+args.case,
        scan_aging_confirm_sec=0.,simulation_scan_max_age_sec=.3,simulation_first_leg_max_distance_m=8.,
        odom_topic='/odom/current_pose',scan_topic='/laser_scan',camera_topic='/test/camera/compressed',
        camera_info_topic='/test/camera/info',stop_signals=[],timeout=3.,settle_seconds=.7,
        preflight_seconds=6.,stable_preflight_seconds=.1,cancel_after=None,goal=None,relative_goal=None)
    rclpy.init();producer=rclpy.create_node('synthetic_execution_inputs')
    node=batch.WaypointPilot(options,out,before['route'],manifest,mission=session)
    group=ReentrantCallbackGroup();executor=MultiThreadedExecutor(num_threads=3);executor.add_node(producer)
    subscriptions=[producer.create_subscription(String,'/experiment/goal_pilot/events',lambda _:None,50)]
    specs=[('pose',PoseStamped,'/lightning/pose'),('odom',Odometry,options.odom_topic),
        ('loc',UInt8,'/lightning/loc_status'),('valid',Bool,'/lightning/pose_valid'),('scan',LaserScan,options.scan_topic),
        ('nav',Twist,'/cmd_vel_nav'),('cmd',Twist,'/cmd_vel'),('safe',Twist,'/cmd_vel_safe'),('tf',TFMessage,'/tf'),
        ('camera',CompressedImage,options.camera_topic),('info',CameraInfo,options.camera_info_topic),('plan',NavPath,'/plan')]
    pubs={key:producer.create_publisher(cls,topic,qos_profile_sensor_data) for key,cls,topic in specs}
    jpeg=cv2.imencode('.jpg',np.zeros((64,64,3),dtype=np.uint8))[1].tobytes()
    stop=threading.Event();failures=[];requests=[];observations=[];callbacks=[];work=[]
    shared=dict(inject=False,pulse_requested=False,pulse_received=None,pose_index=session.start_index)
    publish_ring=deque(maxlen=512);callback_ring=deque(maxlen=512)
    original_remember=node.remember
    def remember(key,value):
        result=original_remember(key,value)
        if key in ('odom','pose','pose_valid','loc_status','scan'):
            callback_ring.append(dict(key=key,steady_ns=time.monotonic_ns(),
                stamp_ns=result.get('stamp_ns'),value=result.get('value'),vx=result.get('vx')))
        bad=(key=='odom' and abs(result.get('vx',0.))>.03) or (key=='pose_valid' and not result.get('value',True))
        if bad and shared['pulse_requested'] and shared['pulse_received'] is None:
            shared['pulse_received']=time.monotonic_ns()
            callbacks.append(dict(key=key,received_steady_ns=shared['pulse_received'],sample=dict(result),
                active=node.ordered.active,dwell_after_callback=node.ordered.dwell_start,
                advancing_first_after_callback=node.ordered.advancing_stop.first_stamp))
        return result
    node.remember=remember
    original_observe=node.observe_ordered
    def observe(healthy,snapshot,**kwargs):
        result=original_observe(healthy,snapshot,**kwargs)
        if node.ordered.active==1:
            observations.append(dict(steady_ns=time.monotonic_ns(),healthy=healthy,
                vx=snapshot['last']['odom']['vx'],pose_valid=snapshot['last']['pose_valid']['value'],
                passed=node.ordered.visits[1]['passed'] is not None))
        return result
    node.observe_ordered=observe
    original_pump=node.pump_runtime_inputs
    def pump(terminal):
        if args.case in ('active_pulse','active_health') and node.ordered.active==1 and not shared['pulse_requested']:
            dwell=node.ordered.dwell_start
            if dwell is not None and time.monotonic_ns()-dwell>=140_000_000:shared['pulse_requested']=True
        return original_pump(terminal)
    node.pump_runtime_inputs=pump

    def publish():
        tick=0
        try:
            while not stop.is_set():
                tick+=1;stamp=producer.get_clock().now().to_msg();wall=time.time_ns();steady=time.monotonic_ns()
                publish_ring.append(dict(tick=tick,steady_ns=steady,stamp_ns=stamp.sec*1_000_000_000+stamp.nanosec,
                    full_batch=tick%5==0,pulse_requested=shared['pulse_requested']))
                index=shared['pose_index'];x,y,yaw=before['route'][index]['pose']
                pulse=shared['pulse_requested'] and shared['pulse_received'] is None
                o=Odometry();o.header.frame_id='odom';o.child_frame_id='base_link';o.header.stamp=stamp
                if shared['inject'] and args.case in ('validation_odom','active_validation'):
                    o.header.stamp=(producer.get_clock().now()-Duration(seconds=.2)).to_msg()
                o.pose.pose.position.x=x;o.pose.pose.position.y=y
                o.pose.pose.orientation.z=math.sin(yaw/2);o.pose.pose.orientation.w=math.cos(yaw/2)
                o.twist.twist.linear.x=.08 if pulse and args.case=='active_pulse' else 0.;pubs['odom'].publish(o)
                if tick%5==0:
                    p=PoseStamped();p.header.frame_id='map';p.header.stamp=stamp;p.pose=o.pose.pose;pubs['pose'].publish(p)
                    pubs['loc'].publish(UInt8(data=1));pubs['valid'].publish(Bool(data=not(pulse and args.case=='active_health')))
                    scan=LaserScan();scan.header.frame_id='base_link';scan.header.stamp=stamp;scan.ranges=[10.]*360;pubs['scan'].publish(scan)
                    for key in ('nav','cmd','safe'):pubs[key].publish(Twist())
                    transforms=[]
                    for parent,child in [('map','odom'),('odom','base_link')]:
                        transform=TransformStamped();transform.header.frame_id=parent;transform.child_frame_id=child
                        transform.header.stamp=(producer.get_clock().now()-Duration(seconds=.2)).to_msg() if shared['inject'] and args.case=='validation_tf' else stamp
                        transform.transform.rotation.w=1.
                        if child=='base_link':
                            transform.transform.translation.x=x;transform.transform.translation.y=y
                            transform.transform.rotation.z=math.sin(yaw/2);transform.transform.rotation.w=math.cos(yaw/2)
                        transforms.append(transform)
                    pubs['tf'].publish(TFMessage(transforms=transforms))
                    c=CompressedImage();c.header.stamp=stamp;c.format='jpeg';c.data=jpeg;pubs['camera'].publish(c)
                    info=CameraInfo();info.header.stamp=stamp;info.width=info.height=64;info.k=[100.,0.,32.,0.,100.,32.,0.,0.,1.];pubs['info'].publish(info)
                    path=NavPath();path.header.frame_id='map';path.header.stamp=stamp;path.poses=[p];pubs['plan'].publish(path)
                    ready.update(ready=True,time_ns=wall,valid_until_ns=wall+(100_000_000 if shared['inject'] and args.case=='validation_ready' else 500_000_000))
                    ready['waypoint_ownership']['checked_unix_ns']=wall
                    temp=ready_path.with_suffix('.tmp');temp.write_text(json.dumps(ready));temp.replace(ready_path)
                stop.wait(.002)
        except BaseException as error:failures.append(repr(error));stop.set()

    def accept(goal):
        requests.append(dict(index=int(goal.goal_index),poses=len(goal.poses),received_steady_ns=time.monotonic_ns()))
        return GoalResponse.REJECT if args.case.startswith('presend') else GoalResponse.ACCEPT
    def execute(handle):
        end=time.monotonic()+5.
        while not stop.is_set() and time.monotonic()<end:
            if handle.is_cancel_requested:handle.canceled();return FollowWaypoints.Result()
            index=1 if node.ordered.visits[0]['passed'] is not None else 0
            shared['pose_index']=index
            feedback=FollowWaypoints.Feedback();feedback.current_waypoint=index;handle.publish_feedback(feedback)
            if args.case=='active_validation' and node.ordered.active==1:shared['inject']=True
            if node.ordered.visits[1]['passed'] is not None:handle.abort();return FollowWaypoints.Result()
            time.sleep(.01)
        handle.abort();return FollowWaypoints.Result()
    server=ActionServer(producer,FollowWaypoints,'/follow_waypoints',execute,goal_callback=accept,
        cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group)
    spinning=threading.Thread(target=executor.spin);thread=threading.Thread(target=publish)
    spinning.start();thread.start();record={}
    try:
        end=time.monotonic()+2.
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.002)
        assert node.execution_ready()[0],node.execution_ready()[1]
        real_validate=session.validate_ready;real_digest=mission_resume.digest
        def validate(lease,**kwargs):
            delayed=False
            def digest_work(value):
                nonlocal delayed
                inject=shared['inject'] and (args.case.startswith('validation_') or args.case=='active_validation')
                if inject and not delayed:
                    delayed=True;start=time.monotonic_ns();source=node.current_summary()
                    time.sleep(.15 if args.case=='validation_ready' else .2)
                    work.append(dict(stage='actual_integrity_digest_in_Session.validate_ready',
                        start_steady_ns=start,end_steady_ns=time.monotonic_ns(),original=source))
                return real_digest(value)
            with patch.object(mission_resume,'digest',side_effect=digest_work):return real_validate(lease,**kwargs)
        session.validate_ready=validate
        if args.case.startswith(('presend','active_')):
            worker.pump=node.pump_storage_wait
            original_budget=batch.mission_budget;prepared=[]
            def prepare_budget(*values,**kwargs):
                result=original_budget(*values,**kwargs)
                if args.case=='presend' and not prepared:
                    start=time.monotonic_ns();time.sleep(.35);prepared.append(True)
                    work.append(dict(stage='actual_presend_mission_budget_preparation',start_steady_ns=start,end_steady_ns=time.monotonic_ns()))
                return result
            with patch.object(batch,'mission_budget',side_effect=prepare_budget):result=node.run_batch()
            worker.pump=None
            if args.case=='presend':
                assert result['classification']=='invalid_dispatch_decision_not_sent',result
                assert not requests and not session.send_invoked and node.goal_sent_ns is None,result
                path=out/'not_sent_result.json';atomic_json(path,dict(result,action_uuid=str(session.action_uuid),
                    status=None,goals_sent=0,send_goal_async_invoked=False))
                worker.record_not_sent(str(session.action_uuid),result_path=path,send_invoked=False)
                assert worker.state()['attempt']['dispatch_state']=='NOT_SENT_AFTER_INTENT'
            elif args.case=='presend_fresh':
                assert len(requests)==1 and session.send_invoked and result['classification']=='goal_rejected',result
            elif args.case=='active_validation':
                assert work and node.cancel_requested and node.ordered.visits[0]['passed'],result
                assert node.ordered.visits[1]['passed'] is None,result
            elif args.case=='active_health' and node.cancel_requested:
                # A delivered invalid sample visible to a main decision follows
                # the existing own-UUID cancellation policy. The Odom pulse
                # case separately requires complete between-poll delivery.
                assert callbacks and callbacks[0]['key']=='pose_valid'
                assert callbacks[0]['dwell_after_callback'] is None and callbacks[0]['advancing_first_after_callback'] is None
                assert 'localization_pose_invalid' in node.cancel_reason,node.cancel_reason
                assert node.ordered.visits[1]['passed'] is None
                assert worker.state()['progress']['accepted_prefix_count']==1
                assert worker.state()['progress']['accepted_receipts'][0]['visit']['passed']==node.ordered.visits[0]['passed']
            else:
                passed=node.ordered.visits[1]['passed'];assert callbacks and passed,result
                assert callbacks[0]['dwell_after_callback'] is None and callbacks[0]['advancing_first_after_callback'] is None,callbacks
                assert passed['steady_ns']-callbacks[0]['received_steady_ns']>=300_000_000,passed
                assert all(row['vx']==0. and row['pose_valid'] for row in observations),observations
                state=worker.state()
                # Native index1 has passed independently, but only a later
                # feedback/terminal may promote it beyond the existing prefix.
                assert state['progress']['accepted_prefix_count']==1,state['progress']
                assert state['progress']['accepted_receipts'][0]['visit']['passed']==node.ordered.visits[0]['passed']
                assert state['candidates']['1']['passed']==passed
            state=worker.state()
            assert state['budgets']['mission_dispatches_reserved']==1,state['budgets']
            assert str(session.action_uuid)==state['attempt']['parent_action_uuid']
            record.update(result=result,after_state=state,requests=requests,callbacks=callbacks,observations=observations)
        else:
            shared['inject']=args.case!='normal'
            end=time.monotonic()+.15
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.002)
            valid,issues,snapshot=node.execution_ready()
            expected={'validation_odom':'odom:invalid_header_age','validation_tf':'map->odom:','validation_ready':'launcher_lease'}
            if args.case=='normal':assert valid,issues
            else:
                assert not valid and any(expected[args.case] in issue for issue in issues),issues
                assert work and work[-1]['original']['last']['odom']['header_age_s']<.3,work
                shared['inject']=False
                end=time.monotonic()+.3
                while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.002)
                assert node.execution_ready()[0],node.execution_ready()[1]
            assert worker.state()==before
            record.update(valid=valid,issues=issues,snapshot=snapshot,state_unchanged=True)
        assert not failures,failures
        record.update(schema='matrix.execution_decision.dds_fixture.v1',case=args.case,passed=True,
            work=work,boundary=__doc__,captured_state_sha256=digest(original_state_path.read_bytes()),
            loaded_client_sha256={name:digest((args.client_dir/name).read_bytes()) for name in CLIENT_SOURCES},
            fresh_synthetic_mission=fresh,simulation_acceptance=False,physical_acceptance=False)
        (out/'RESULT.json').write_text(json.dumps(record,indent=2)+'\n')
        print(json.dumps(dict(case=args.case,passed=True,requests=len(requests),callback_pulses=len(callbacks))))
    finally:
        attempt=dict(case=args.case,requests=requests,callbacks=callbacks,observations=observations,work=work,
            publisher_ring=list(publish_ring),callback_ring=list(callback_ring),publisher_failures=failures,
            last_counts=dict(node.counts),pulse_requested=shared['pulse_requested'],pulse_received=shared['pulse_received'])
        (out/'ATTEMPT.json').write_text(batch.evidence_json(attempt,indent=2)+'\n')
        worker.pump=None;stop.set();thread.join(timeout=3.)
        if node.action_goal_handle is not None and node.action_result is None:
            node.finalize_own_goal()
        executor.shutdown(timeout_sec=3.);spinning.join(timeout=3.)
        node.finish_evidence();node.destroy_node();server.destroy();producer.destroy_node();rclpy.shutdown();worker.close()


if __name__=='__main__':main()
