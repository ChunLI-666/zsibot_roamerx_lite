#!/usr/bin/env python3
"""Normal mission CLI and real DDS protocol fixtures; no plant or source PASS.

All production health/decision/stop/Session/SQLite code runs normally. The only
domain adapter is the existing read-only retry test helper. A separate optional
checkpoint injection delays a real completed Store.observe return, receives a
bad raw Pose through the registered subscription, then lets the unchanged final
decision run. SQLite timeout/journal/durability behavior is unchanged.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid


def fixture_scan(stamp_ns):
    from sensor_msgs.msg import LaserScan
    scan=LaserScan();scan.header.frame_id='base_link'
    scan.header.stamp.sec=stamp_ns//1_000_000_000;scan.header.stamp.nanosec=stamp_ns%1_000_000_000
    scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360;scan.angle_max=math.pi-scan.angle_increment
    scan.time_increment=0.;scan.scan_time=.05;scan.range_min=.05;scan.range_max=20.;scan.ranges=[10.]*360
    return scan


def child_cli():
    source=Path(sys.argv[2]).resolve();sys.path.insert(0,str(source))
    flag=os.environ.get('P3_CHECKPOINT_PAYLOAD_FLAG')
    if flag:
        from mission_state import MissionStore
        import waypoint_batch
        from waypoint_contract import evidence_json
        cls=waypoint_batch.WaypointPilot;observe=cls.observe_ordered;run=cls.run_batch
        flag=Path(flag);receipt=flag.with_suffix('.receipt.json')
        main_thread=threading.get_ident();bad_received=threading.Event()
        checkpoint=dict(armed=False,injected=False);store_observe=MissionStore.observe
        def delayed_observe(self,*args,**kwargs):
            result=store_observe(self,*args,**kwargs)
            if checkpoint['armed'] and not checkpoint['injected']:
                assert threading.get_ident()!=main_thread
                checkpoint.update(injected=True,worker_thread_id=threading.get_ident(),
                    original_observe_result=result,original_observe_finished_steady_ns=time.monotonic_ns())
                flag.write_text('raw_pose_z_nan\n')
                checkpoint['callback_observed_before_worker_return']=bad_received.wait(.20)
                checkpoint['worker_return_steady_ns']=time.monotonic_ns()
            return result
        MissionStore.observe=delayed_observe
        def inject(self,*args,**kwargs):
            value=observe(self,*args,**kwargs)
            if (getattr(self,'_p3_checkpoint_injected',False) or self.action_result is None
                    or not (self.stop_evidence or {}).get('observed')):return value
            self._p3_checkpoint_injected=True
            store=self.mission.store;before=self.current_summary();original_pump=store.pump
            seen=None;checkpoint['armed']=True
            def pump():
                nonlocal seen
                original_pump()
                sample=self.current_summary()
                if sample['last'].get('pose',{}).get('raw_pose_values_finite') is False:
                    seen=sample;bad_received.set()
            try:
                store.pump=pump
                # A real observe checkpoint runs in the normal worker. Only its
                # return is delayed, so the original main-thread ROS pump can
                # receive this deliberate original-payload fault in that window.
                store.observe(str(self.mission.action_uuid),self.ordered)
            finally:
                store.pump=original_pump
                checkpoint['armed']=False
            assert seen is not None and checkpoint['callback_observed_before_worker_return'],checkpoint
            receipt.write_text(evidence_json(dict(injected=True,original_callbacks_executed=True,
                before_pose_count=before['counts'].get('pose'),after_pose_count=seen['counts'].get('pose'),
                bad_original_pose=seen['last']['pose'],guard_methods_patched=False,
                sqlite_behavior_changed=False,main_thread_id=main_thread,event_timeout_sec=.20,
                checkpoint=checkpoint))+'\n')
            return value
        def run_then_restore(self):
            result=run(self)
            if getattr(self,'_p3_checkpoint_injected',False):
                flag.unlink(missing_ok=True);deadline=time.monotonic()+.5
                while time.monotonic()<deadline:
                    self.pump_stop_inputs(time.monotonic_ns()+20_000_000)
                    if self.current_summary()['last'].get('pose',{}).get('raw_pose_values_finite') is True:break
                assert self.current_summary()['last']['pose']['raw_pose_values_finite'] is True
            return result
        cls.observe_ordered=inject;cls.run_batch=run_then_restore
    # Reuses only the domain adapter. It calls the real production CLI main.
    from test_mission_retry_dds import child_cli as normal_child
    try:normal_child()
    finally:
        if flag:
            MissionStore.observe=store_observe
            cls.observe_ordered=observe;cls.run_batch=run


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--client-dir',type=Path,required=True)
    parser.add_argument('--evidence-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',choices=['all','success','different_failure','checkpoint'],default='all')
    args=parser.parse_args();assert os.environ.get('ROS_DOMAIN_ID')=='190'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    source=args.client_dir.resolve();sys.path.insert(0,str(source))
    from mission_state import MissionStore,atomic_json,boot_clock,digest
    from mission_continuation import CLIENT_SOURCES
    from waypoint_contract import simulation_scan_latency_profile,scan_confirmation_payload_valid
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
    from std_msgs.msg import Bool,UInt8,String
    from tf2_msgs.msg import TFMessage
    profile=json.loads((args.evidence_root/'solmax_b41_stop_review_v1/PROFILE_RESULT.json').read_text())
    for number,line in enumerate((args.evidence_root/profile['ready_capture']['path']).open(),1):
        if number==profile['ready_capture']['line']:ready=copy.deepcopy(json.loads(line)['latest_ready']);break
    else:raise AssertionError('Captured real READY shape missing')
    document=dict(schema_version=1,frame_id='map',mission_budget_sec=300.,waypoints=[
        dict(id='ORIGINAL_0',pose=[0.,0.,0.],require_stopped=True),
        dict(id='ORIGINAL_FINAL_1',pose=[0.,0.,0.],require_stopped=True)])
    identity=dict(variant='official',source_sha='f'*40,**{k:'b'*64 for k in
        ['map_contract_sha256','footprint_sha256','acceptance_policy_sha256','effective_config_sha256']})
    policy=dict(mission_budget_sec=300.,max_parent_dispatches=10,max_parent_dispatches_per_index=3,
        max_attempts_per_index=6,bt_retries=1)
    for name,value in [('route',document),('identity',identity),('policy',policy)]:atomic_json(out/(name+'.json'),value)
    seed=MissionStore(out/'identity_fixture.db');seed.create((out/'route.json').read_bytes(),identity,policy)
    expected_identity=seed.state()['identity'];seed.close()
    ready['variant']='official';ready['mission_identity']=dict(expected_identity,verified=True)
    ready['simulation_scan_latency_profile']=simulation_scan_latency_profile(.3)
    ready['waypoint_contract']['client_sources_sha256']={
        'experiments/planner_comparison_matrix/goal_pilot_source/'+n:digest((source/n).read_bytes()) for n in
        ['waypoint_batch.py','pilot_scan_timing.py','message_info_contract.py','waypoint_contract.py']}
    ready['waypoint_contract']['mission_resume']['client_sources_sha256']={n:digest((source/n).read_bytes()) for n in CLIENT_SOURCES}
    ready['waypoint_ownership'].update(navigate_to_pose_active=[],follow_waypoints_active=[],follow_path_active=[])
    boot,now=boot_clock();ready['mission_deadline'].update(launcher_boot_id=boot,deadline_boottime_sec=now+600.)
    assert scan_confirmation_payload_valid(fixture_scan(time.time_ns()))
    rclpy.init(domain_id=190);producer=rclpy.create_node('residual_cli_protocol_fixture')
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(producer)
    group=ReentrantCallbackGroup();stop=threading.Event();requests=[];commands=[];errors=[]
    events=[]
    event_recorder=producer.create_subscription(String,'/experiment/goal_pilot/events',lambda value:events.append(value.data),100)
    current=dict(label=None,mode=None,yaw=0.,store=None);bad_flag=out/'bad_payload.flag'
    memory=dict(scan=None,bad_pose=False);scan_publications=[];ready_writes=[]
    specs=[('pose',PoseStamped,'/lightning/pose'),('odom',Odometry,'/odom/current_pose'),
        ('loc',UInt8,'/lightning/loc_status'),('valid',Bool,'/lightning/pose_valid'),
        ('scan',LaserScan,'/laser_scan'),('nav',Twist,'/cmd_vel_nav'),('cmd',Twist,'/cmd_vel'),
        ('safe',Twist,'/cmd_vel_safe'),('tf',TFMessage,'/tf'),
        ('camera',CompressedImage,'/residual/camera/compressed'),('info',CameraInfo,'/residual/camera/info'),('plan',NavPath,'/plan')]
    pubs={name:producer.create_publisher(cls,topic,qos_profile_sensor_data) for name,cls,topic in specs}
    jpeg=cv2.imencode('.jpg',np.zeros((64,64,3),dtype=np.uint8))[1].tobytes()
    def publish():
        count=0
        try:
            while not stop.is_set():
                count+=1;stamp=producer.get_clock().now().to_msg();wall=time.time_ns();steady=time.monotonic_ns()
                odom=Odometry();odom.header.frame_id='odom';odom.header.stamp=stamp;odom.child_frame_id='base_link'
                odom.pose.pose.orientation.z=math.sin(current['yaw']/2);odom.pose.pose.orientation.w=math.cos(current['yaw']/2)
                pubs['odom'].publish(odom)
                pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=stamp;pose.pose=copy.deepcopy(odom.pose.pose)
                if memory['bad_pose']:pose.pose.position.z=math.nan
                pubs['pose'].publish(pose)
                transforms=[]
                for parent,child in [('map','odom'),('odom','base_link')]:
                    t=TransformStamped();t.header.frame_id=parent;t.header.stamp=stamp;t.child_frame_id=child
                    t.transform.rotation.w=1.
                    if child=='base_link':t.transform.rotation=odom.pose.pose.orientation
                    transforms.append(t)
                pubs['tf'].publish(TFMessage(transforms=transforms))
                if count%25==0:
                    pubs['loc'].publish(UInt8(data=1));pubs['valid'].publish(Bool(data=True))
                    ns=stamp.sec*1_000_000_000+stamp.nanosec;pubs['scan'].publish(fixture_scan(ns))
                    for key in ('nav','cmd','safe'):pubs[key].publish(Twist())
                    camera=CompressedImage();camera.header.stamp=stamp;camera.format='jpeg';camera.data=jpeg;pubs['camera'].publish(camera)
                    info=CameraInfo();info.header.stamp=stamp;info.width=info.height=64
                    info.k=[100.,0.,32.,0.,100.,32.,0.,0.,1.];pubs['info'].publish(info)
                    path=NavPath();path.header.frame_id='map';path.header.stamp=stamp;path.poses=[pose];pubs['plan'].publish(path)
                    memory['scan']=dict(topic='/laser_scan',header_ns=ns,frame_id='base_link',ranges=360,
                        receive_steady_seconds=steady/1e9,validation_clock_begin_ns=wall,validation_clock_end_ns=wall,
                        validation_steady_begin_ns=steady,validation_steady_end_ns=steady)
                    scan_publications.append(dict(header_ns=ns,publish_group_finished_unix_ns=time.time_ns(),
                        publish_group_finished_steady_ns=time.monotonic_ns()))
                stop.wait(.002)
        except BaseException as error:errors.append(repr(error));stop.set()
    def write_ready():
        # Source publishing above has no filesystem access. Only this separate
        # writer serializes/replaces READY and observes the test control flag.
        try:
            while not stop.is_set():
                entered=time.monotonic_ns();memory['bad_pose']=bad_flag.exists();flag_done=time.monotonic_ns()
                observation=memory['scan']
                if observation is None:stop.wait(.005);continue
                document=copy.deepcopy(ready);wall=time.time_ns()
                document['producer_scan_observation']=observation
                document.update(ready=True,time_ns=wall,valid_until_ns=wall+500_000_000)
                document['waypoint_ownership']['checked_unix_ns']=wall
                serial_started=time.monotonic_ns();encoded=json.dumps(document);serial_done=time.monotonic_ns()
                temp=out/'READY.tmp';temp.write_text(encoded);write_done=time.monotonic_ns();temp.replace(out/'READY.json')
                ready_writes.append(dict(started_steady_ns=entered,flag_read_ns=flag_done-entered,
                    serialization_ns=serial_done-serial_started,write_ns=write_done-serial_done,
                    replace_ns=time.monotonic_ns()-write_done,original_scan_header_ns=observation['header_ns']))
                stop.wait(.05)
        except BaseException as error:errors.append('ready_writer:'+repr(error));stop.set()
    def accept(request):
        previous=[r for r in requests if r['task']==current['label']]
        row=dict(task=current['label'],ordinal=len(previous),index=int(request.goal_index),poses=len(request.poses),unix_ns=time.time_ns())
        state=MissionStore.inspect(current['store']);row['store_at_accept']=dict(mission_id=state['mission_id'],
            accepted_prefix=state['progress']['accepted_prefix_count'],next_index=state['progress']['next_index'],
            budget=state['budgets'],correction=state.get('final_goal_correction'))
        requests.append(row);return GoalResponse.ACCEPT
    def execute(handle):
        row=requests[-1];row['action_uuid']=str(uuid.UUID(bytes=bytes(handle.goal_id.uuid)))
        current['yaw']=0.;start=int(handle.request.goal_index)
        for index in range(start,len(handle.request.poses)):
            end=time.monotonic()+.75
            while time.monotonic()<end and not stop.is_set():
                if handle.is_cancel_requested:handle.canceled();return FollowWaypoints.Result()
                feedback=FollowWaypoints.Feedback();feedback.current_waypoint=index;handle.publish_feedback(feedback);time.sleep(.02)
        result=FollowWaypoints.Result()
        if row['ordinal']==0:
            current['yaw']=.12;time.sleep(.2)
            if handle.is_cancel_requested:handle.canceled();return FollowWaypoints.Result()
            handle.succeed()
        elif current['mode']=='different_failure':
            missed=MissedWaypoint();missed.index=1;missed.error_code=60002;missed.goal=handle.request.poses[1]
            result.missed_waypoints=[missed];handle.abort()
        else:handle.succeed()
        return result
    server=ActionServer(producer,FollowWaypoints,'/follow_waypoints',execute,goal_callback=accept,
        cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group)
    spinning=threading.Thread(target=executor.spin);publishing=threading.Thread(target=publish)
    writer=threading.Thread(target=write_ready,name='fixture_ready_writer',daemon=True)
    spinning.start();publishing.start();writer.start();contexts=out/'actual_DDS_contexts.jsonl';states={}
    def invoke(operation,label,reason=None,inject=False):
        command=[sys.executable,'-B',str(Path(__file__).resolve()),'--normal-cli',str(source),str(contexts),operation,
            '--execute','--store',str(current['store']),'--resource-lock',str(out/'robot_domain.lock'),
            '--request-id',str(uuid.uuid4()),'--ready',str(out/'READY.json'),'--output',str(out/label),
            '--camera-topic','/residual/camera/compressed','--camera-info-topic','/residual/camera/info','--simulation-scan-max-age-sec','.3']
        if operation=='start':
            for name in ['route','identity','policy']:command.extend(['--'+name,str(out/(name+'.json'))])
        else:
            state=MissionStore.inspect(current['store']);command.extend(['--mission-id',state['mission_id'],'--expect-revision',str(state['revision'])])
        if reason:command.extend(['--reason',reason])
        env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',ROS_DOMAIN_ID='190',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        if inject:env['P3_CHECKPOINT_PAYLOAD_FLAG']=str(bad_flag)
        completed=subprocess.run(command,env=env,capture_output=True,text=True,timeout=110)
        record=dict(command=command,returncode=completed.returncode,stdout=completed.stdout,stderr=completed.stderr)
        atomic_json(out/('CLI_'+label+'.json'),record);commands.append(record)
        assert current['store'].exists(),record
        return MissionStore.inspect(current['store']),record
    try:
        end=time.monotonic()+3.
        while not (out/'READY.json').exists() and time.monotonic()<end:time.sleep(.02)
        assert (out/'READY.json').exists()
        # This bounded source-only check precedes any CLI invocation/goal. It
        # does not rewrite stamps, relax .3s, or hide a writer/GIL source pause.
        start_index=len(scan_publications);end=time.monotonic()+3.
        while time.monotonic()<end and not stop.is_set():stop.wait(.02)
        observed=scan_publications[start_index:]
        intervals=[(b['header_ns']-a['header_ns'])/1e9 for a,b in zip(observed,observed[1:])]
        check=dict(schema='matrix.residual.protocol_source_health.v1',source_only_seconds=3.,
            original_scan_samples=len(observed),max_original_header_gap_sec=max(intervals,default=math.inf),
            minimum_original_header_gap_sec=min(intervals,default=0.),
            max_ready_serialization_ns=max((r['serialization_ns'] for r in ready_writes),default=None),
            max_ready_write_ns=max((r['write_ns'] for r in ready_writes),default=None),
            passed=len(observed)>=20 and bool(intervals) and min(intervals)>0. and max(intervals)<=.3,
            scan_original_headers_unchanged=True,publisher_filesystem_io=False,publisher_sleep_sec=.002)
        atomic_json(out/'SOURCE_HEALTH_SELF_CHECK.json',check)
        assert check['passed'] and not errors,check
        assert rclpy.get_default_context().get_domain_id()==190
        atomic_json(out/'SETUP_AUDIT.json',dict(schema='matrix.residual.dds_setup_audit.v1',
            actual_domain=190,source_client_sha256={n:digest((source/n).read_bytes()) for n in CLIENT_SOURCES},
            actual_publishers=[dict(role=n,topic=t,type=c.__name__) for n,c,t in specs],
            actual_event_subscription='/experiment/goal_pilot/events',action_server='/follow_waypoints',
            scan_payload_valid=scan_confirmation_payload_valid(fixture_scan(time.time_ns())),
            intentional_scan_header_age_sec=0.,operational_scan_max_sec=.3,strict_scan_max_sec=.3,
            database_lock_rule='each DB own .lock; distinct robot_domain.lock',
            recorder_lease_sec=600.,mission_budget_sec=300.,normal_cancel_handling=True,
            boundary='Protocol fixture setup, not actual Source/candidate admission'))
        cases=['success','different_failure','checkpoint'] if args.case=='all' else [args.case]
        for mode in cases:
            current.update(label=mode,mode=mode,yaw=0.,store=out/(mode+'.db'))
            state,cli=invoke('start',mode+'_start',inject=mode=='checkpoint')
            own=[r for r in requests if r['task']==mode];states[mode]=state
            if mode=='checkpoint':
                assert len(own)==1 and len(state['attempts'])==1,(own,state,cli)
                native=json.loads(Path(state['attempt']['native_result']['path']).read_text())
                assert native['status']==4 and native['final_goal_validation']['predicates']['execution_healthy'] is False,native
                assert native['current_final_goal_observation']['healthy'] is True,native
                assert state.get('final_goal_correction') is None and state['progress']['accepted_prefix_count']==1,state
                injection=json.loads(bad_flag.with_suffix('.receipt.json').read_text())
                assert injection['injected'] and injection['after_pose_count']>injection['before_pose_count'],injection
                assert injection['checkpoint']['injected'] and injection['sqlite_behavior_changed'] is False,injection
                assert injection['checkpoint']['worker_thread_id']!=injection['main_thread_id'],injection
            else:
                assert len(own)==2 and len(state['attempts'])==2,(own,state,cli)
                assert [r['index'] for r in own]==[0,1] and [r['poses'] for r in own]==[2,2],own
                assert [r['action_uuid'] for r in own]==[a['parent_action_uuid'] for a in state['attempts']],state
                c=state['final_goal_correction'];assert c['consumed_attempt_id']==state['attempts'][1]['attempt_id'],state
                assert own[1]['store_at_accept']['accepted_prefix']==1,own
                b0,b1=own[0]['store_at_accept']['budget'],own[1]['store_at_accept']['budget']
                assert b0['deadline_boottime_sec']==b1['deadline_boottime_sec'],(b0,b1)
                assert state['budgets']['mission_dispatches_reserved']==2 and state['budgets']['per_index_reserved']==[2,4],state
                original=json.loads(Path(c['source_evidence']['path']).read_text())
                assert original['status']==4 and original['functional_status']=='FAIL' and original['missed_waypoints']==[],original
                # Quality faults are intentionally absent here. The separate
                # real Store transaction tests prove a prior strict FAIL stays
                # locked across correction; this protocol run does not inject it.
                assert original['strict_scan_timing_status']!='FAIL',original
                assert set(k for k,v in original['final_goal_validation']['predicates'].items() if not v)=={'final_yaw_within'},original
                for result in [original]:
                    stop_proof=result['owned_goal_shutdown']['stop']['callback_observation'];d=stop_proof['advancing_odom_dwell']
                    assert type(stop_proof['callback_invalidations']) is int and d['last_stamp']-d['first_stamp']>=300_000_000
                    assert d['last_steady']-d['first_steady']>=300_000_000
                if mode=='success':
                    assert cli['returncode']==0 and state['progress']['accepted_prefix_count']==2 and state['lifecycle'].startswith('COMPLETED'),(state,cli)
                else:
                    assert state['hold_reason']=='FINAL_GOAL_CORRECTION_FAILED' and state['lifecycle']=='NEEDS_TASK_DECISION',(state,cli)
                    assert state['attempt']['missed_waypoints'][0]['error_code']==60002,state
                    for label,reason in [('default',None),('review','A review cannot create a second final-goal correction')]:
                        again,_=invoke('resume',mode+'_resume_'+label,reason=reason)
                        assert len([r for r in requests if r['task']==mode])==2 and len(again['attempts'])==2,again
            assert not errors,errors
        atomic_json(out/'RESULT.json',dict(schema='matrix.mission.residual.normal_cli_dds.v1',passed=True,
            cases=cases,requests=requests,states=states,actual_contexts=[json.loads(x) for x in contexts.read_text().splitlines()],
            client_source_sha256={n:digest((source/n).read_bytes()) for n in CLIENT_SOURCES},
            boundary=__doc__,simulation_acceptance=False,physical_acceptance=False))
        print(json.dumps(dict(passed=True,cases=cases,actual_server_requests=len(requests))))
    finally:
        bad_flag.unlink(missing_ok=True);stop.set();publishing.join(timeout=3.);writer.join(timeout=3.)
        executor.shutdown(timeout_sec=3.);spinning.join(timeout=3.);server.destroy();producer.destroy_node();rclpy.shutdown()
        atomic_json(out/'PUBLISHER_TELEMETRY.json',dict(schema='matrix.residual.protocol_publisher_timing.v1',
            scans=scan_publications,ready_writes=ready_writes,writer_alive=writer.is_alive(),publisher_alive=publishing.is_alive(),
            original_header_gap_sec=[(b['header_ns']-a['header_ns'])/1e9 for a,b in zip(scan_publications,scan_publications[1:])],
            boundary='Fixture source timing only; no Scene/SDK latency model'))
        atomic_json(out/'ATTEMPT.json',dict(requests=requests,commands=commands,publisher_errors=errors,states=states,
            event_recording_samples=len(events),simulation_acceptance=False,physical_acceptance=False))


if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--normal-cli':child_cli()
    else:main()
