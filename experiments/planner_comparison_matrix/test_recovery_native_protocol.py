#!/usr/bin/env python3
"""Actual BT navigator -> FollowWaypoints -> classifier, isolated DDS only.

The fake planner/follower and generated sensor inputs are protocol fixtures,
not a simulator, plant model, physical navigation or waypoint acceptance.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose, FollowPath, FollowWaypoints,NavigateToPose
from nav2_msgs.srv import GetCostmap
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, UInt8
from tf2_ros import TransformBroadcaster
import yaml

import recovery_navigation


def wait(future, timeout=8.):
    deadline=time.monotonic()+timeout
    while not future.done():
        if time.monotonic()>=deadline:raise TimeoutError('DDS future deadline')
        time.sleep(.002)
    return future.result()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix',type=Path,required=True)
    parser.add_argument('--waypoint-prefix',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cases',nargs='+',choices=['a17','one_drift','invalid_path','stale_health','blocked_validate','unknown_release'],
        default=['a17','one_drift','invalid_path','stale_health','blocked_validate','unknown_release'])
    args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='190'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    prefix=args.prefix.resolve();waypoint=args.waypoint_prefix.resolve()
    sys.path.insert(0,str(Path(__file__).parent/'goal_pilot_source'))
    from mission_state import failure_decision

    config=dict(run_id='native_typed_recovery',footprint=[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]],
        padding=0.,odom_topic='/odom/current_pose',scan_topic='/laser_scan',scan_max_age=.3,
        stop_timeout_sec=30.,health_stable_sec=1.,bt_retries=1)
    xml=out/'navigate.xml';xml.write_text(recovery_navigation.tree(config))
    params={'bt_navigator':{'ros__parameters':dict(use_sim_time=False,global_frame='map',robot_base_frame='base_link',
        odom_topic='/odom/current_pose',default_nav_to_pose_bt_xml=str(xml),navigators=['navigate_to_pose'],
        plugin_lib_names=['matrix_recovery_nodes'],error_code_names=recovery_navigation.ERROR_CODE_NAMES,
        bond_heartbeat_period=0.,default_server_timeout=1000)}}
    params['bt_navigator']['ros__parameters']['navigate_to_pose.plugin']='nav2_bt_navigator::NavigateToPoseNavigator'
    param_file=out/'params.yaml';param_file.write_text(yaml.safe_dump(params))
    rclpy.init();node=rclpy.create_node('controller_server');group=ReentrantCallbackGroup()
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    state=dict(mode='a17',plans=0,follows=0,shift=0.,normal=True)
    child_status=[];gate_process=None;gate_suspended=False
    lock=threading.Lock();tf=TransformBroadcaster(node)
    odom=node.create_publisher(Odometry,'/odom/current_pose',qos_profile_sensor_data)
    scan=node.create_publisher(LaserScan,'/laser_scan',qos_profile_sensor_data)
    loc=node.create_publisher(UInt8,'/lightning/loc_status',10)
    valid=node.create_publisher(Bool,'/lightning/pose_valid',10)
    safe=node.create_publisher(Twist,'/cmd_vel_safe',10)
    raw=node.create_publisher(Twist,'/matrix/native_typed/raw',10)
    def statuses(message):
        with lock:child_status.append(dict(steady_ns=time.monotonic_ns(),statuses=[
            dict(uuid=bytes(row.goal_info.goal_id.uuid).hex(),status=int(row.status)) for row in message.status_list]))
    child_subscription=node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',statuses,
        QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL),callback_group=group)
    ticks=0
    def inputs():
        nonlocal ticks
        ticks+=1;now=node.get_clock().now().to_msg()
        with lock:shift=state['shift'];normal=state['normal']
        o=Odometry();o.header.frame_id='odom';o.child_frame_id='base_link';o.header.stamp=now
        o.pose.pose.orientation.w=1.;odom.publish(o)
        a=TransformStamped();a.header.stamp=now;a.header.frame_id='map';a.child_frame_id='odom'
        a.transform.translation.x=shift;a.transform.rotation.w=1.
        b=TransformStamped();b.header.stamp=now;b.header.frame_id='odom';b.child_frame_id='base_link';b.transform.rotation.w=1.
        tf.sendTransform([a,b])
        if ticks%10:return
        safe.publish(Twist());raw.publish(Twist());loc.publish(UInt8(data=1 if normal else 0));valid.publish(Bool(data=True))
        s=LaserScan();s.header.stamp=now;s.header.frame_id='base_link';s.range_min=.1;s.range_max=20.;s.ranges=[10.]*360;scan.publish(s)
    timer=node.create_timer(.005,inputs,callback_group=MutuallyExclusiveCallbackGroup())
    def costmap(request,response):
        m=response.map;m.header.frame_id='odom';m.header.stamp=node.get_clock().now().to_msg()
        m.metadata.size_x=m.metadata.size_y=150;m.metadata.resolution=.02
        with lock:blocked=state['mode']=='blocked_validate' and state['plans']>0
        m.metadata.origin.position.x=m.metadata.origin.position.y=-1.5;m.metadata.origin.orientation.w=1.;m.data=[254 if blocked else 0]*22500
        return response
    map_service=node.create_service(GetCostmap,'/local_costmap/get_costmap',costmap,callback_group=group)
    def plan(handle):
        nonlocal gate_suspended
        with lock:
            state['plans']+=1;number=state['plans'];mode=state['mode'];start=state['shift']
        result=ComputePathToPose.Result();result.path.header.frame_id='map';result.path.header.stamp=node.get_clock().now().to_msg()
        p=PoseStamped();p.header=result.path.header;p.pose.position.x=start;p.pose.orientation.w=1.
        result.path.poses=[] if mode=='invalid_path' else [p,handle.request.goal]
        if (mode=='a17' and number==2) or (mode=='one_drift' and number==1):
            with lock:state['shift']+=.06
            time.sleep(.15)
        if mode=='blocked_validate':time.sleep(.2)  # current RPC sees the real blocked map before Validate
        if mode=='unknown_release':
            # HOLD/ACQUIRE already succeeded. Only this owned gate stops here;
            # native Validate sends a real RELEASE whose answer remains unknown.
            assert gate_process is not None
            os.killpg(gate_process.pid,signal.SIGSTOP);gate_suspended=True
        handle.succeed();return result
    def follow(handle):
        with lock:state['follows']+=1;mode=state['mode'];number=state['follows']
        result=FollowPath.Result()
        if mode=='a17' and number==1:
            result.error_code=106;handle.abort()
        else:handle.succeed()
        return result
    planner=ActionServer(node,ComputePathToPose,'/compute_path_to_pose',plan,callback_group=group,
        goal_callback=lambda _:GoalResponse.ACCEPT,cancel_callback=lambda _:CancelResponse.ACCEPT)
    follower=ActionServer(node,FollowPath,'/follow_path',follow,callback_group=group,
        goal_callback=lambda _:GoalResponse.ACCEPT,cancel_callback=lambda _:CancelResponse.ACCEPT)
    thread=threading.Thread(target=executor.spin);thread.start()
    env=os.environ.copy();env.update(AMENT_PREFIX_PATH=f'{prefix}:{waypoint}:/opt/ros/jazzy',
        LD_LIBRARY_PATH=f'{prefix}/lib:{waypoint}/lib:/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu',
        ROS_LOG_DIR=str(out/'ros_logs'))
    processes=[];logs=[]
    def launch(name,command):
        log=(out/(name+'.log')).open('w');logs.append(log)
        p=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p);return p
    def activate(name):
        service=node.create_client(ChangeState,'/'+name+'/change_state',callback_group=group)
        assert service.wait_for_service(timeout_sec=10),name
        for value in (1,3):
            request=ChangeState.Request();request.transition.id=value
            assert wait(service.call_async(request),15).success,name
    cases=[]
    try:
        gate_process=launch('gate',[str(prefix/'lib/matrix_recovery_bt/zero_handoff_gate'),'--ros-args',
            '-p','run_id:=native_typed_recovery','-p','raw_topic:=/matrix/native_typed/raw'])
        navigator=launch('navigator',['/opt/ros/jazzy/lib/nav2_bt_navigator/bt_navigator','--ros-args','--params-file',str(param_file)])
        activate('bt_navigator')
        launch('waypoint',[str(waypoint/'lib/matrix_waypoint_follower/waypoint_follower'),'--ros-args',
            '-p','wait_at_waypoint.enabled:=false','-p','bond_heartbeat_period:=0.0'])
        activate('waypoint_follower')
        readback=node.create_client(GetParameters,'/bt_navigator/get_parameters',callback_group=group)
        assert readback.wait_for_service(timeout_sec=5)
        request=GetParameters.Request();request.names=['error_code_names','default_nav_to_pose_bt_xml']
        values=wait(readback.call_async(request)).values
        assert list(values[0].string_array_value)==recovery_navigation.ERROR_CODE_NAMES
        assert values[1].string_value==str(xml)
        (out/'error_parameter_readback.json').write_text(json.dumps(dict(error_code_names=list(values[0].string_array_value),
            xml_path=values[1].string_value,xml_sha256=hashlib.sha256(xml.read_bytes()).hexdigest()),indent=2)+'\n')
        client=ActionClient(node,FollowWaypoints,'/follow_waypoints',callback_group=group)
        assert client.wait_for_server(timeout_sec=5)
        child=ActionClient(node,NavigateToPose,'/navigate_to_pose',callback_group=group)
        assert child.wait_for_server(timeout_sec=5)
        time.sleep(1.)  # allow the newly-created C++ child client to discover it too
        for mode,expected_code,expected_class,expected_plans,expected_follows in [
            ('a17',60001,'RECOVERABLE_FAILURE',2,1),
            ('one_drift',0,'SUCCESS',2,1),
            ('invalid_path',60004,'NEEDS_TASK_DECISION',1,0),
            ('stale_health',60002,'PAUSED_INPUT_HEALTH',0,0),
            ('blocked_validate',60005,'RECOVERABLE_FAILURE',1,0),
            ('unknown_release',60003,'HOLD',1,0)]:
            if mode not in args.cases:continue
            with lock:status_start=len(child_status)
            case_started=time.monotonic_ns()
            with lock:state.update(mode=mode,plans=0,follows=0,normal=mode!='stale_health')
            goal=FollowWaypoints.Goal();goal.goal_index=1;goal.number_of_loops=0
            for x in [0.,.5]:
                p=PoseStamped();p.header.frame_id='map';p.pose.position.x=x;p.pose.orientation.w=1.;goal.poses.append(p)
            accepted=wait(client.send_goal_async(goal));assert accepted.accepted
            result=wait(accepted.get_result_async(),45.)
            if gate_suspended:
                os.killpg(gate_process.pid,signal.SIGCONT);gate_suspended=False
            missed=[dict(index=m.index,error_code=m.error_code) for m in result.result.missed_waypoints]
            if mode=='one_drift':
                assert result.status==4 and not missed
                classification='SUCCESS'
            else:
                assert result.status==6 and missed==[dict(index=1,error_code=expected_code)],missed
                classification=failure_decision(dict(status=result.status,missed_waypoints=missed),True,True,True)[0]
            assert classification==expected_class,(mode,classification)
            with lock:observed=dict(state)
            assert observed['plans']==expected_plans and observed['follows']==expected_follows,observed
            end=time.monotonic()+3.
            while time.monotonic()<end:
                with lock:messages=child_status[status_start:]
                if messages and all(row['status']>=4 for row in messages[-1]['statuses']):break
                time.sleep(.01)
            assert messages and messages[-1]['statuses'],(mode,'missing native NavigateToPose UUID/status')
            assert all(row['status']>=4 for row in messages[-1]['statuses']),(mode,messages[-1])
            if mode=='blocked_validate':
                assert time.monotonic_ns()-case_started>=29_000_000_000
                assert failure_decision(dict(status=result.status,missed_waypoints=missed),True,True,True)[1]=='BOUNDED_OBSTACLE_WAIT'
            if mode=='unknown_release':
                assert not observed['follows'] and expected_class=='HOLD'
            cases.append(dict(mode=mode,parent_uuid=bytes(accepted.goal_id.uuid).hex(),native_status=result.status,
                missed_waypoints=missed,classification=classification,plans=observed['plans'],follows=observed['follows'],
                child_status_events=messages,wall_ns=time.monotonic_ns()-case_started))
            (out/'cases.json').write_text(json.dumps(cases,indent=2)+'\n')
        text=(out/'navigator.log').read_text()
        if 'a17' in args.cases:assert 'prior_controller=106' in text and 'plan_start_invalidated' in text
        if 'blocked_validate' in args.cases:assert 'WAIT_PLAN_START_CERTIFICATE' in text and '60005' in text
        if 'unknown_release' in args.cases:assert 'WAIT_RELEASE_RESPONSE' in text and '60003' in text
        mapping=(Path('/proc')/str(navigator.pid)/'maps').read_text();(out/'navigator.maps').write_text(mapping)
        assert str(prefix/'lib/libmatrix_recovery_nodes.so') in mapping
        (out/'loaded_artifacts.json').write_text(json.dumps(dict(recovery_plugin_sha256=hashlib.sha256(
            (prefix/'lib/libmatrix_recovery_nodes.so').read_bytes()).hexdigest(),waypoint_core_sha256=hashlib.sha256(
            (waypoint/'lib/libmatrix_waypoint_follower_core.so').read_bytes()).hexdigest(),
            boundary=__doc__,simulation_acceptance=False,physical_acceptance=False),indent=2)+'\n')
        assert failure_decision(dict(status=6,missed_waypoints=[dict(index=1,error_code=0)]),True,True,True)[0]=='NEEDS_TASK_DECISION'
        print(json.dumps(dict(cases=len(cases),result='PASS_NATIVE_BT_PARENT_CLASSIFIER',physical_acceptance=False)))
    finally:
        if gate_suspended and gate_process is not None and gate_process.poll() is None:
            os.killpg(gate_process.pid,signal.SIGCONT)
        for p in reversed(processes):
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in reversed(processes):
            try:p.wait(timeout=6)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        timer.cancel();executor.shutdown();thread.join();planner.destroy();follower.destroy();node.destroy_node();rclpy.shutdown()
        for log in logs:log.close()


if __name__=='__main__':main()
