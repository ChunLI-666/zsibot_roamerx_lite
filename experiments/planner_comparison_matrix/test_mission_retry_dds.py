#!/usr/bin/env python3
"""Normal mission CLI plus real localhost DDS; all runtime inputs are fixtures.

Exercises the production CLI main/run_attempt/WaypointPilot/Session/SQLite path.
The FollowWaypoints protocol fixture returns original-index 60002; this is not
another Nav2/plant parent-chain or actual-source acceptance run. A child adapter
forces every real rclpy context to test domain190 while satisfying the historical
CLI's domain185 admission check. No production health/stop/decision code is mocked.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import runpy
import subprocess
import sys
import threading
import time
import uuid


def child_cli():
    source=Path(sys.argv[2]).resolve();context_path=Path(sys.argv[3])
    assert os.environ.get('ROS_DOMAIN_ID')=='190'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    sys.path.insert(0,str(source))
    import rclpy
    real_init=rclpy.init
    def isolated_init(*args,**kwargs):
        kwargs['domain_id']=190
        value=real_init(*args,**kwargs)
        domain=rclpy.get_default_context().get_domain_id();assert domain==190
        with context_path.open('a') as stream:
            stream.write(json.dumps(dict(actual_rclpy_domain=domain,pid=os.getpid(),unix_ns=time.time_ns()))+'\n')
        return value
    rclpy.init=isolated_init
    # Only the fixed domain admission is adapted; DDS uses explicit domain190.
    os.environ['ROS_DOMAIN_ID']='185'
    sys.argv=[str(source/'mission_resume.py'),*sys.argv[4:]]
    runpy.run_path(str(source/'mission_resume.py'),run_name='__main__')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--client-dir',type=Path,required=True)
    parser.add_argument('--evidence-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='190'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    source=args.client_dir.resolve();sys.path.insert(0,str(source))
    from mission_state import MissionStore,atomic_json,boot_clock,digest
    from mission_continuation import CLIENT_SOURCES
    from waypoint_contract import simulation_scan_latency_profile
    import cv2
    import numpy as np
    import rclpy
    from rclpy.action import ActionServer,CancelResponse,GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped,TransformStamped,Twist
    from nav_msgs.msg import Odometry,Path as NavPath
    from nav2_msgs.action import FollowWaypoints
    from nav2_msgs.msg import MissedWaypoint
    from sensor_msgs.msg import CameraInfo,CompressedImage,LaserScan
    from std_msgs.msg import Bool,String,UInt8
    from tf2_msgs.msg import TFMessage

    root=args.evidence_root.resolve()
    profile=json.loads((root/'solmax_b41_stop_review_v1/PROFILE_RESULT.json').read_text())
    for number,line in enumerate((root/profile['ready_capture']['path']).open(),1):
        if number==profile['ready_capture']['line']:
            ready=copy.deepcopy(json.loads(line)['latest_ready']);break
    else:raise AssertionError('captured READY shape unavailable')
    document=dict(schema_version=1,frame_id='map',mission_budget_sec=300.,waypoints=[
        dict(id='ORIGINAL_0',pose=[0.,0.,0.],require_stopped=True),
        dict(id='ORIGINAL_1',pose=[1.,0.,0.],require_stopped=True)])
    identity=dict(variant='official',source_sha='f'*40,**{key:'b'*64 for key in
        ('map_contract_sha256','footprint_sha256','acceptance_policy_sha256','effective_config_sha256')})
    policy=dict(mission_budget_sec=300.,max_parent_dispatches=10,max_parent_dispatches_per_index=3,
        max_attempts_per_index=6,bt_retries=1)
    atomic_json(out/'route.json',document);atomic_json(out/'identity.json',identity);atomic_json(out/'policy.json',policy)
    seed=MissionStore(out/'identity_fixture.db')
    seed.create((out/'route.json').read_bytes(),identity,policy);expected_identity=seed.state()['identity'];seed.close()
    ready['variant']='official';ready['mission_identity']=dict(expected_identity,verified=True)
    ready['simulation_scan_latency_profile']=simulation_scan_latency_profile(.3)
    ready['waypoint_contract']['client_sources_sha256']={
        'experiments/planner_comparison_matrix/goal_pilot_source/'+name:digest((source/name).read_bytes()) for name in
        ('waypoint_batch.py','pilot_scan_timing.py','message_info_contract.py','waypoint_contract.py')}
    ready['waypoint_contract']['mission_resume']['client_sources_sha256']={
        name:digest((source/name).read_bytes()) for name in CLIENT_SOURCES}
    ready['waypoint_ownership'].update(navigate_to_pose_active=[],follow_waypoints_active=[],follow_path_active=[])
    # Recorder/source lease includes motion, shutdown reserve and full preflight;
    # the mission's independent motion budget remains exactly 300 seconds.
    boot,now=boot_clock();ready['mission_deadline'].update(launcher_boot_id=boot,deadline_boottime_sec=now+600.)
    rclpy.init(domain_id=190);producer=rclpy.create_node('retry_cli_protocol_fixture')
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(producer)
    group=ReentrantCallbackGroup();requests=[];events=[];errors=[];stop=threading.Event();commands=[]
    subscription=producer.create_subscription(String,'/experiment/goal_pilot/events',lambda message:events.append(message.data),100)
    specs=[('pose',PoseStamped,'/lightning/pose'),('odom',Odometry,'/odom/current_pose'),
        ('loc',UInt8,'/lightning/loc_status'),('valid',Bool,'/lightning/pose_valid'),
        ('scan',LaserScan,'/laser_scan'),('nav',Twist,'/cmd_vel_nav'),('cmd',Twist,'/cmd_vel'),
        ('safe',Twist,'/cmd_vel_safe'),('tf',TFMessage,'/tf'),('camera',CompressedImage,'/retry/camera/compressed'),
        ('info',CameraInfo,'/retry/camera/info'),('plan',NavPath,'/plan')]
    pubs={name:producer.create_publisher(cls,topic,qos_profile_sensor_data) for name,cls,topic in specs}
    jpeg=cv2.imencode('.jpg',np.zeros((64,64,3),dtype=np.uint8))[1].tobytes()
    def publish():
        count=0
        try:
            while not stop.is_set():
                count+=1;stamp=producer.get_clock().now().to_msg();wall=time.time_ns()
                odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.header.stamp=stamp
                odom.pose.pose.orientation.w=1.;pubs['odom'].publish(odom)
                pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=stamp;pose.pose=odom.pose.pose
                pubs['pose'].publish(pose)
                transforms=[]
                for parent,child in [('map','odom'),('odom','base_link')]:
                    transform=TransformStamped();transform.header.frame_id=parent;transform.child_frame_id=child
                    transform.header.stamp=stamp;transform.transform.rotation.w=1.;transforms.append(transform)
                pubs['tf'].publish(TFMessage(transforms=transforms))
                if count%25==0:
                    pubs['loc'].publish(UInt8(data=1));pubs['valid'].publish(Bool(data=True))
                    scan=LaserScan();scan.header.frame_id='base_link';scan.header.stamp=stamp;scan.ranges=[10.]*360
                    pubs['scan'].publish(scan)
                    for key in ('nav','cmd','safe'):pubs[key].publish(Twist())
                    camera=CompressedImage();camera.header.stamp=stamp;camera.format='jpeg';camera.data=jpeg;pubs['camera'].publish(camera)
                    info=CameraInfo();info.header.stamp=stamp;info.width=info.height=64
                    info.k=[100.,0.,32.,0.,100.,32.,0.,0.,1.];pubs['info'].publish(info)
                    path=NavPath();path.header.frame_id='map';path.header.stamp=stamp;path.poses=[pose];pubs['plan'].publish(path)
                    ready.update(ready=True,time_ns=wall,valid_until_ns=wall+500_000_000)
                    ready['waypoint_ownership']['checked_unix_ns']=wall
                    temp=out/'READY.tmp';temp.write_text(json.dumps(ready));temp.replace(out/'READY.json')
                stop.wait(.002)
        except BaseException as error:errors.append(repr(error));stop.set()
    def accept(request):
        requests.append(dict(index=int(request.goal_index),poses=len(request.poses),unix_ns=time.time_ns()))
        return GoalResponse.ACCEPT
    def execute(handle):
        requests[-1]['action_uuid']=str(uuid.UUID(bytes=bytes(handle.goal_id.uuid)))
        until=time.monotonic()+.5
        while time.monotonic()<until and not stop.is_set():
            if handle.is_cancel_requested:handle.canceled();return FollowWaypoints.Result()
            feedback=FollowWaypoints.Feedback();feedback.current_waypoint=0;handle.publish_feedback(feedback);time.sleep(.02)
        result=FollowWaypoints.Result();missed=MissedWaypoint();missed.index=0;missed.error_code=60002
        missed.goal=handle.request.poses[0];result.missed_waypoints=[missed]
        handle.abort();return result
    server=ActionServer(producer,FollowWaypoints,'/follow_waypoints',execute,goal_callback=accept,
        cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group)
    spinning=threading.Thread(target=executor.spin);publishing=threading.Thread(target=publish)
    spinning.start();publishing.start()
    contexts=out/'actual_DDS_contexts.jsonl';store_path=out/'mission.db'
    environment=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',ROS_DOMAIN_ID='190',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    def invoke(operation,label,reason=None,expected=(0,)):
        request=str(uuid.uuid4());directory=out/label
        command=[sys.executable,'-B',str(Path(__file__).resolve()),'--normal-cli',str(source),str(contexts),operation,
            '--execute','--store',str(store_path),'--resource-lock',str(out/'resource.lock'),'--request-id',request,
            '--ready',str(out/'READY.json'),'--output',str(directory),'--camera-topic','/retry/camera/compressed',
            '--camera-info-topic','/retry/camera/info']
        if operation=='start':
            command.extend(['--route',str(out/'route.json'),'--identity',str(out/'identity.json'),'--policy',str(out/'policy.json')])
        else:
            state=MissionStore.inspect(store_path)
            command.extend(['--mission-id',state['mission_id'],'--expect-revision',str(state['revision'])])
        if reason is not None:command.extend(['--reason',reason])
        result=subprocess.run(command,env=environment,capture_output=True,text=True,timeout=110)
        record=dict(label=label,command=command,returncode=result.returncode,stdout=result.stdout,stderr=result.stderr)
        commands.append(record);atomic_json(out/('CLI_'+label+'.json'),record)
        assert result.returncode in expected,record
        return MissionStore.inspect(store_path),record
    try:
        end=time.monotonic()+3.
        while not (out/'READY.json').exists() and time.monotonic()<end:time.sleep(.02)
        assert (out/'READY.json').exists()
        state,_=invoke('start','start_two_failures',expected=(14,))
        assert len(requests)==2 and len(state['attempts'])==2,state
        assert state['hold_reason']=='REPEATED_FAILURE_REVIEW_REQUIRED'
        assert state['retry_control']['paused'] and state['retry_control']['consecutive_failures']==2
        assert state['progress']['accepted_prefix_count']==0
        assert state['budgets']['mission_dispatches_reserved']==2
        original_uuid=state['attempt']['parent_action_uuid'];control=copy.deepcopy(state['retry_control'])
        state,_=invoke('resume','resume_without_explicit_reason',expected=(10,14))
        assert len(requests)==2 and len(state['attempts'])==2 and state['attempt']['parent_action_uuid']==original_uuid
        assert state['retry_control']==control
        state,_=invoke('reconcile','reconcile_same_attempt',expected=(11,))
        assert len(requests)==2 and state['retry_control']==control
        state,_=invoke('resume','reviewed_one_retry',reason='PM reviewed original failure evidence; permit one bounded diagnostic attempt',expected=(14,))
        assert len(requests)==3 and len(state['attempts'])==3
        assert state['retry_control']['paused'] and state['retry_control']['consecutive_failures']==3
        assert state['budgets']['per_index_parent_dispatches'][0]==3
        assert state['budgets']['per_index_reserved'][0]==6
        assert state['progress']['accepted_prefix_count']==0
        state,record=invoke('resume','review_cannot_reset_budget',reason='Review cannot bypass the original allowance',expected=(10,12,14))
        assert len(requests)==3 and len(state['attempts'])==3
        assert 'FAILED_BUDGET' in record['stdout'],record
        assert [x['action_uuid'] for x in requests]==[x['parent_action_uuid'] for x in state['attempts']]
        main_state=copy.deepcopy(state)
        # A separate task still has one parent allowance left when explicitly
        # canceled; budget exhaustion cannot mask an ineffective cancel gate.
        store_path=out/'cancel_mission.db'
        canceled,_=invoke('start','cancel_task_two_failures',expected=(14,))
        assert len(requests)==5 and len(canceled['attempts'])==2
        original=copy.deepcopy(canceled['attempt'])
        canceled,_=invoke('cancel','cancel_already_terminal_task',reason='PM stops this still-budgeted task',expected=(11,))
        assert canceled['lifecycle']=='CANCELED' and canceled['cancel_intent']
        assert canceled['attempt']['native_status']==6
        assert canceled['attempt']['parent_action_uuid']==original['parent_action_uuid']
        assert canceled['attempt']['native_result']==original['native_result']
        for label,reason in [('canceled_resume_default',None),('canceled_resume_review','Review cannot reverse task cancellation')]:
            canceled,_=invoke('resume',label,reason=reason,expected=(10,14))
            assert canceled['lifecycle']=='CANCELED' and len(canceled['attempts'])==2 and len(requests)==5
        assert canceled['budgets']['per_index_parent_dispatches'][0]==2
        assert canceled['budgets']['per_index_reserved'][0]==4
        assert [x['action_uuid'] for x in requests[3:]]==[x['parent_action_uuid'] for x in canceled['attempts']]
        assert not errors,errors
        atomic_json(out/'RESULT.json',dict(schema='matrix.mission.retry.normal_cli_dds.v1',passed=True,
            requests=requests,final_state=main_state,canceled_task_state=canceled,
            actual_contexts=[json.loads(x) for x in contexts.read_text().splitlines()],
            loaded_client_sha256={name:digest((source/name).read_bytes()) for name in CLIENT_SOURCES},
            boundary=__doc__,simulation_acceptance=False,physical_acceptance=False))
        print(json.dumps(dict(passed=True,actual_server_requests=len(requests),accepted_prefix_count=0)))
    finally:
        stop.set();publishing.join(timeout=3.);executor.shutdown(timeout_sec=3.);spinning.join(timeout=3.)
        server.destroy();producer.destroy_node();rclpy.shutdown()
        atomic_json(out/'ATTEMPT.json',dict(requests=requests,commands=commands,publisher_errors=errors,
            event_count=len(events),simulation_acceptance=False,physical_acceptance=False))


if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--normal-cli':child_cli()
    else:main()
