#!/usr/bin/env python3
"""Normal WaypointPilot/Session/StorageWorker with captured large B41 state.

All topics and READY leases are explicitly synthetic localhost fixtures. The
captured historical bytes are never changed. No Action goals, plant simulation,
robot connections or physical acceptance are involved. Use a separate process
for each case to preserve the original single-threaded ROS executor.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import statistics
import sys
import threading
import time
from types import SimpleNamespace
import uuid


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--client-dir',type=Path,required=True)
    parser.add_argument('--evidence-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',choices=['terminal','cancel','moving','pulse','cached','stale','deadline','overrun'],required=True)
    parser.add_argument('--baseline',action='store_true')
    args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='190'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(args.client_dir.resolve()))
    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped,TransformStamped,Twist
    from nav_msgs.msg import Odometry,Path as NavPath
    from sensor_msgs.msg import LaserScan,CompressedImage,CameraInfo
    from std_msgs.msg import String,UInt8,Bool
    from tf2_msgs.msg import TFMessage
    from waypoint_batch import WaypointPilot,freeze_sources
    from waypoint_contract import simulation_scan_latency_profile,OwnedGoalIdentity
    from mission_resume import Session
    from mission_storage import StorageWorker
    from mission_state import MissionStore,encoded,digest,boot_clock,Hold
    from mission_continuation import CLIENT_SOURCES

    D=args.evidence_root.resolve()
    profile=json.loads((D/'solmax_b41_stop_review_v1/PROFILE_RESULT.json').read_text())
    state_path=D/profile['profile_state']['path'];state=json.loads(state_path.read_bytes())
    event_path=D/profile['ready_capture']['path'];event=None
    for number,line in enumerate(event_path.open(),1):
        if number==profile['ready_capture']['line']:event=json.loads(line);break
    ready=copy.deepcopy(event['latest_ready']);original_ready=encoded(ready).encode()
    (out/'captured_ready_structured.json').write_bytes(original_ready)
    (out/'captured_state.json').write_bytes(state_path.read_bytes())
    boot,now=boot_clock()
    # Only the isolated fixture DB receives a new local clock. No historical DB
    # or evidence path is opened for writing; acceptance/progress remain intact.
    original_budgets=copy.deepcopy(state['budgets'])
    state['budgets'].update(deadline_boot_id=boot,deadline_boottime_sec=now+120)
    database=out/'fixture.db';seed=MissionStore(database)
    seed.db.execute('INSERT INTO mission VALUES (1,?,?)',(state['revision'],encoded(state)));seed.close()
    worker=StorageWorker(database,out/'resource.lock')
    session=Session(worker,str(uuid.uuid4()),out)
    manifest=freeze_sources(out)
    ready['waypoint_contract']['client_sources_sha256']={
        'experiments/planner_comparison_matrix/goal_pilot_source/'+name:value for name,value in manifest['sources_sha256'].items()}
    ready['waypoint_contract']['mission_resume']['client_sources_sha256']={
        name:digest((args.client_dir/name).read_bytes()) for name in CLIENT_SOURCES}
    ready['simulation_scan_latency_profile']=simulation_scan_latency_profile(.3)
    ready['mission_deadline'].update(launcher_boot_id=boot,deadline_boottime_sec=now+120)
    ready['waypoint_ownership'].update(navigate_to_pose_active=[],follow_waypoints_active=[],follow_path_active=[])
    ready_path=out/'synthetic_READY.json'
    audit=dict(boundary=__doc__,source_state_path=str(state_path),source_state_sha256=digest(state_path.read_bytes()),
        source_event_path=str(event_path),source_event_line=profile['ready_capture']['line'],
        original_event_ready_sha256=event['latest_ready_sha256'],structured_ready_sha256=digest(original_ready),
        structured_ready_bytes=len(original_ready),state_bytes=state_path.stat().st_size,
        fixture_changes=['local boot/deadline in isolated DB and READY','current loaded client source bindings',
            'strict .3 scan profile','fresh synthetic READY lease and ownership timestamp','empty synthetic active goals'],
        original_budgets=original_budgets,physical_acceptance=False)
    (out/'fixture_manifest.json').write_text(json.dumps(audit,indent=2)+'\n')
    measurements=[]
    for _ in range(100):
        start=time.perf_counter_ns();session.validate_ready(ready);measurements.append(time.perf_counter_ns()-start)
    before=worker.state()
    options=SimpleNamespace(execute=True,ready=ready_path,variant=state['variant'],case_id='dds_stop_'+args.case,
        scan_aging_confirm_sec=0.,simulation_scan_max_age_sec=.3,odom_topic='/odom/current_pose',scan_topic='/laser_scan',
        camera_topic='/test/camera/compressed',camera_info_topic='/test/camera/info',stop_signals=[],
        timeout=10.,settle_seconds=1.3,cancel_after=None,goal=None,relative_goal=None)
    rclpy.init();producer=rclpy.create_node('synthetic_b41_stop_inputs')
    node=WaypointPilot(options,out,state['route'],manifest,mission=session)
    subscriptions=[producer.create_subscription(String,'/experiment/goal_pilot/events',lambda _:None,50)]
    specs=[('pose',PoseStamped,'/lightning/pose'),('odom',Odometry,options.odom_topic),
        ('loc',UInt8,'/lightning/loc_status'),('valid',Bool,'/lightning/pose_valid'),('scan',LaserScan,options.scan_topic),
        ('nav',Twist,'/cmd_vel_nav'),('cmd',Twist,'/cmd_vel'),('safe',Twist,'/cmd_vel_safe'),('tf',TFMessage,'/tf'),
        ('camera',CompressedImage,options.camera_topic),('info',CameraInfo,options.camera_info_topic),('plan',NavPath,'/plan')]
    pubs={key:producer.create_publisher(cls,topic,qos_profile_sensor_data) for key,cls,topic in specs}
    jpeg=cv2.imencode('.jpg',np.zeros((64,64,3),dtype=np.uint8))[1].tobytes()
    stopped=threading.Event();shared=dict(start=None,source=None,last_nonzero_ns=None);errors=[]
    def publish():
        tick=0
        try:
            while not stopped.is_set():
                tick+=1;stamp=producer.get_clock().now().to_msg();wall=time.time_ns();steady=time.monotonic_ns()
                elapsed=0. if shared['start'] is None else (steady-shared['start'])/1e9
                active=shared['start'] is not None
                speed=.08 if active and ((args.case=='moving' and elapsed<.2) or (args.case=='pulse' and .22<elapsed<.235)) else 0.
                if speed:shared['last_nonzero_ns']=steady
                o=Odometry();o.header.frame_id='odom';o.child_frame_id='base_link';o.header.stamp=stamp;o.pose.pose.orientation.w=1.
                o.twist.twist.linear.x=speed
                if active and args.case=='cached':
                    if shared['source'] is None:shared['source']=stamp
                    o.header.stamp=shared['source']
                if not (active and args.case=='stale'):pubs['odom'].publish(o)
                if tick%10==0:
                    p=PoseStamped();p.header.frame_id='map';p.header.stamp=stamp;p.pose.orientation.w=1.;pubs['pose'].publish(p)
                    pubs['loc'].publish(UInt8(data=1));pubs['valid'].publish(Bool(data=True))
                    scan=LaserScan();scan.header.frame_id='base_link';scan.header.stamp=stamp;scan.ranges=[10.]*360;pubs['scan'].publish(scan)
                    for key in ('nav','cmd','safe'):pubs[key].publish(Twist())
                    transforms=[]
                    for parent,child in [('map','odom'),('odom','base_link')]:
                        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=stamp;t.transform.rotation.w=1.;transforms.append(t)
                    pubs['tf'].publish(TFMessage(transforms=transforms))
                    c=CompressedImage();c.header.stamp=stamp;c.format='jpeg';c.data=jpeg;pubs['camera'].publish(c)
                    info=CameraInfo();info.header.stamp=stamp;info.width=info.height=64;info.k=[100.,0.,32.,0.,100.,32.,0.,0.,1.];pubs['info'].publish(info)
                    path=NavPath();path.header.frame_id='map';path.header.stamp=stamp;path.poses=[p];pubs['plan'].publish(path)
                    ready.update(ready=True,time_ns=wall,valid_until_ns=wall+500_000_000)
                    ready['waypoint_ownership']['checked_unix_ns']=wall
                    temp=ready_path.with_suffix('.tmp');temp.write_text(json.dumps(ready));temp.replace(ready_path)
                stopped.wait(.002)
        except BaseException as error:errors.append(repr(error));stopped.set()
    thread=threading.Thread(target=publish);thread.start()
    result={}
    try:
        warm=time.monotonic()+2.
        while time.monotonic()<warm:rclpy.spin_once(node,timeout_sec=.002)
        valid,issues,_=node.execution_ready();assert valid,issues
        node.owned_identity=OwnedGoalIdentity(uuid.uuid4().bytes)
        node.goal_sent_ns=time.time_ns();node.goal_sent_steady=time.monotonic()
        node.action_result=SimpleNamespace(status=5 if args.case=='cancel' else 4)
        node.cancel_requested=args.case=='cancel';node.finalizing=True
        if args.case=='cancel':options.stop_signals=[2]  # finally must still drain after SIGINT
        if args.case=='overrun':
            def slow_callback():
                delay.cancel();time.sleep(.15)  # an individual callback is not preemptible
            delay=node.create_timer(.31,slow_callback)
        counts=dict(node.counts);start=time.monotonic_ns();shared['start']=start
        seconds=.15 if args.case=='deadline' else .4 if args.case=='overrun' else 1.3
        observed,snapshot=node.wait_settled(seconds);end=time.monotonic_ns()
        result=dict(case=args.case,baseline=args.baseline,observed=observed,elapsed_ns=end-start,
            deadline_ns=start+int(seconds*1e9),finished_steady_ns=end,
            callback_deltas={k:node.counts[k]-counts.get(k,0) for k in node.counts},
            artifact_invalid_reasons=node.artifact_invalid_reasons,
            last_nonzero_steady_ns=shared['last_nonzero_ns'],observation=getattr(node,'stop_wait_observation',None),
            full_guard_median_ns=statistics.median(measurements),full_guard_max_ns=max(measurements),
            state_unchanged=worker.state()==before,physical_acceptance=False)
        (out/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n')
        if not args.baseline:
            assert observed==(args.case not in ('cached','stale','deadline','overrun')),result
            if args.case in ('terminal','cancel','moving','pulse'):
                assert not node.artifact_invalid_reasons,result
            if args.case in ('moving','pulse'):
                assert shared['last_nonzero_ns'] and end-shared['last_nonzero_ns']>=300_000_000,result
                assert result['observation']['callback_invalidations']>0,result
            if args.case=='deadline':assert end-start<250_000_000,result
            if args.case=='overrun':assert 400_000_000<end-start<650_000_000,result
        assert worker.state()==before
        assert not errors,errors
        print(json.dumps({k:result[k] for k in ('case','baseline','observed','elapsed_ns','full_guard_median_ns','state_unchanged')}))
    finally:
        stopped.set();thread.join();node.finish_evidence();node.destroy_node();producer.destroy_node();rclpy.shutdown();worker.close()


if __name__=='__main__':main()
