#!/usr/bin/env python3
"""Bounded BackUp action against real controller/smoother/gate, isolated test plant.

No hardware launch/transport is used. Synthetic rear-wall geometry is intentionally
separate from recorded-bag navigation accuracy. ROS domain is restricted to 190.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid
import yaml

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'epoch_navigation_validation'))
from run_stack import prepare,Trial,runtime_manifest


def test_map(output,blocked):
    side=200;pixels=bytearray([254])*(side*side)
    for row in range(side):
        for col in range(side):
            if row in (0,side-1) or col in (0,side-1):pixels[row*side+col]=0
            # Rear wall starts outside the robot footprint, but blocks a 0.2 m retreat.
            if blocked and col==89 and 85<=row<=115:pixels[row*side+col]=0
    image=output/'map.pgm';image.write_bytes(b'P5\n200 200\n255\n'+pixels)
    path=output/'map.yaml';path.write_text(yaml.safe_dump(dict(image=str(image),resolution=.05,origin=[-5.,-5.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
    return path


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scenario',choices=['free','blocked','cancel','loss','epoch','deadline','odom_freeze','moving_feedback','delayed_odom','small_budget','no_effective','invalid','replan','slow_controller','slow_relay','slow_gate','rear_during_braking','recorded_stall'],required=True)
    parser.add_argument('--map',type=Path)
    parser.add_argument('--initial',type=float,nargs=3,default=[0.,0.,0.])
    args=parser.parse_args();args.workspace=args.workspace.resolve();args.output=args.output.resolve()
    args.output.mkdir(parents=True,exist_ok=False);(args.output/'COLCON_IGNORE').touch()
    args.map=args.map.resolve() if args.map else test_map(args.output,args.scenario=='blocked')
    args.footprint=args.workspace/'src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/params/zsl1_model_envelope.yaml'
    args.geometry=True;args.domain=190
    params=prepare(args.workspace,args.output)
    config=yaml.safe_load(params.read_text());config['controller_server']['ros__parameters']['enable_epoch_backup']=True
    # BackUp has an independent action loop; ordinary TRACK keeps its original rate.
    config['controller_server']['ros__parameters']['epoch_backup_control_frequency']=20.
    config['controller_server']['ros__parameters']['epoch_backup_minimum_speed']=config['velocity_smoother']['ros__parameters']['deadband_velocity'][0]
    if args.scenario=='slow_controller':config['controller_server']['ros__parameters']['epoch_backup_control_frequency']=10.
    if args.scenario=='slow_relay':config['velocity_smoother']['ros__parameters']['smoothing_frequency']=10.
    if args.scenario=='no_effective':config['controller_server']['ros__parameters']['epoch_backup_minimum_speed']=.08
    if args.scenario in ('delayed_odom','rear_during_braking'):config['velocity_smoother']['ros__parameters']['max_decel'][0]=-.2
    params.write_text(yaml.safe_dump(config))
    inputs=json.loads((args.output/'inputs.json').read_text())
    inputs['immutable_files']=[str(path) for path in HERE.glob('*.py')]
    inputs['params_sha256']=hashlib.sha256(params.read_bytes()).hexdigest()
    (args.output/'inputs.json').write_text(json.dumps(inputs,indent=2))
    os.environ.update(ROS_DOMAIN_ID='190',ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    import rclpy
    from rclpy.action import ActionClient
    from navigo_epoch_msgs.action import BackUpEpoch
    from navigo_epoch_msgs.msg import NavigationIntent,LocalizationEpoch
    from fixture import Fixture,stamp,fill_pose
    from geometry_audit import load_footprint,audit_geometry
    from scene import Scene
    from recovery_scenarios import ScanTiming,mark_rear_wall,CostmapVisibility
    from sensor_msgs.msg import LaserScan
    before=runtime_manifest(args.workspace,args.output)
    rclpy.init();fixture=Fixture(args.map,args.initial,args.output,geometry_enabled=True)
    scan_timing=ScanTiming(fixture.scene);fixture.scene.scan=scan_timing.scan
    before_scene=Scene(args.map);dynamic_injection=None;visibility=None
    started=None;initial=None;backup_measurement_end_ns=None;goal=None
    trial=Trial(args,fixture);summary=dict(scenario=args.scenario,success=False,hardware_io=False)
    intent=NavigationIntent();intent.boot_id=fixture.boot_id
    publisher=fixture.create_publisher(NavigationIntent,'/nav_epoch/intent',10)
    def renew():
        intent.heartbeat_sequence+=1;intent.source_steady_time_ns=time.monotonic_ns();publisher.publish(intent)
    timer=fixture.create_timer(.05,renew)
    try:
        if not hasattr(fixture,'flush_to_now'):raise RuntimeError('Formal follow-up requires full wall-time plant')
        trial.start('navigation',['ros2','launch','robot_navigo','bringup_launch.py','use_composition:=false','use_sim_time:=true','use_respawn:=false','map:='+str(args.map),'params_file:='+str(params)])
        # HERE.parent is scripts; resolve directly rather than launching any robot adapter.
        gate=HERE.parent/'nav_safety_gate.py'
        trial.start('gate',[sys.executable,str(gate),'--ros-args','-p','use_sim_time:=true','-p','enable_epoch_contract:=true','-p','stop_publish_period_ms:='+str(100 if args.scenario=='slow_gate' else 50)])
        trial.lifecycle_ready();trial.verify_loaded_footprints();trial.spin_for(.5)
        client=ActionClient(fixture,BackUpEpoch,'/back_up_epoch')
        trial.spin_until(client.server_is_ready,10,'BackUpEpoch action availability')
        trial.spin_until(lambda:'/nav_epoch/gate_session' in fixture.latest_messages,5,'Gate session')
        token=intent.token
        token.localization.process_session_id=fixture.localization_session;token.localization.map_loaded_instance=fixture.map_instance
        token.localization.epoch=fixture.epoch;token.localization.commits=fixture.commits
        token.navigation_session_id=str(uuid.uuid4());token.task_sequence=token.plan_sequence=1
        token.gate_session_id=fixture.latest_messages['/nav_epoch/gate_session'].data
        token.execution_kind=token.BACKUP;token.backup_max_speed=.08
        token.execution_deadline_ns=time.monotonic_ns()+int((.8 if args.scenario=='deadline' else 5.)*1e9)
        intent.active=True;renew()
        goal=BackUpEpoch.Goal();goal.token=copy.deepcopy(token);goal.distance=.2;goal.speed=.08
        if args.scenario=='invalid':goal.distance=.31
        if args.scenario in ('small_budget','no_effective'):goal.distance=.03
        goal.start.header.frame_id='map';goal.start.header.stamp=stamp(fixture.sim_time);fill_pose(goal.start.pose,fixture.current_map_pose())
        goal.localization_heartbeat_sequence=fixture.heartbeat
        started=fixture.flush_to_now();initial=fixture.pose.copy()
        future=client.send_goal_async(goal)
        trial.spin_until(future.done,5,'Backup acknowledgement');handle=future.result()
        if not handle.accepted:raise AssertionError('Action admission rejected instead of bounded result')
        result=handle.get_result_async()
        if args.scenario in ('cancel','loss','epoch','odom_freeze','moving_feedback','delayed_odom','recorded_stall'):
            trial.spin_until(lambda:fixture.safe[0]<-.001 or result.done(),4,'Actual reverse output')
            if result.done():
                early=result.result();summary.update(action_status=early.status,error_code=early.result.error_code,reason=early.result.error_msg)
                raise AssertionError('Action ended before injection')
            trial.spin_for(.2);injection=time.monotonic_ns()
            if args.scenario=='cancel':
                canceled=handle.cancel_goal_async();trial.spin_until(canceled.done,3,'Cancel acknowledgement')
            elif args.scenario=='loss':fixture.change_health(LocalizationEpoch.LOST)
            elif args.scenario=='epoch':fixture.change_health(LocalizationEpoch.NORMAL,recover=True)
            elif args.scenario=='odom_freeze':
                fixture.odom_pub.publish=lambda msg:None
            elif args.scenario=='recorded_stall':
                trial.spin_until(lambda:fixture.safe[0]<-.005 or result.done(),1.,'Actual reverse immediately before executor stall')
                fixture.flush_to_now()
                if result.done() or fixture._executed_velocity[0]>=-.005:
                    raise AssertionError('Stall precondition uncovered: plant is not executing reverse')
                fixture.record('sensor_executor_stall',dict(duration_sec=.35,actual_velocity=fixture._executed_velocity.copy(),reason='deterministic reproduction of stale-source interval'))
                time.sleep(.35)
                fixture.flush_to_now()
                fixture.record('sensor_executor_resumed',{})
            elif args.scenario=='moving_feedback':
                original_publish=fixture.odom_pub.publish
                def moving_feedback(message):
                    message.twist.twist.linear.x=-.03
                    original_publish(message)
                fixture.odom_pub.publish=moving_feedback
            else:
                original_publish=fixture.odom_pub.publish;delayed=[]
                def delayed_odom(message):
                    current=time.monotonic();delayed.append((current,copy.deepcopy(message)))
                    while delayed and current-delayed[0][0]>=.25:
                        original_publish(delayed.pop(0)[1])
                fixture.odom_pub.publish=delayed_odom
            summary['injection_ns']=injection
        if args.scenario=='rear_during_braking':
            def braking_observed():
                raw=fixture.latest_messages.get('/cmd_vel_epoch_raw')
                odom=fixture.latest.get('odometry_publication')
                return bool(raw and raw.token.execution_kind==1 and raw.source_motion_phase==raw.PHASE_HOLD and odom and odom['data']['twist']['twist']['linear']['x']<-.005)
            trial.spin_until(lambda:braking_observed() or result.done(),5,'First source HOLD with measured rearward motion')
            if result.done():raise AssertionError('BackUp stopped before braking obstacle could be injected')
            if not hasattr(fixture,'flush_to_now'):raise RuntimeError('Dynamic obstacle requires full wall-time plant')
            effective_ns=fixture.flush_to_now()
            polygon,_=load_footprint(args.footprint)
            measured_vx=fixture.latest['odometry_publication']['data']['twist']['twist']['linear']['x']
            minimum_clearance=measured_vx*measured_vx/(2.*.2)+.01
            dynamic_injection=mark_rear_wall(fixture.scene,fixture.pose,polygon,clearance=minimum_clearance)
            dynamic_injection.update(effective_ns=effective_ns,mutation_completed_ns=time.monotonic_ns(),
                measured_vx=measured_vx,ideal_deceleration_only_stop_distance_m=measured_vx*measured_vx/(2.*.2),
                odometry=fixture.latest['odometry_publication']['data'],source_hold_sequence=fixture.latest_messages['/cmd_vel_epoch_raw'].command_sequence)
            fixture.record('dynamic_rear_obstacle',dynamic_injection)
            scan=LaserScan();scan.header.stamp=stamp(fixture.sim_time);scan.header.frame_id='base_link'
            scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360;scan.angle_max=scan.angle_min+359*scan.angle_increment
            scan.range_min=.01;scan.range_max=8.;scan.scan_time=.04;scan.ranges=fixture.scene.scan(fixture.pose)
            fixture.scan_pub.publish(scan)
            summary['obstacle_injection']=dynamic_injection
            visibility=CostmapVisibility(fixture,dynamic_injection)
        trial.spin_until(result.done,8,'Backup terminal result')
        wrapped=result.result()
        if visibility:
            summary['costmap_visibility']=visibility.close()
        fixture.flush_to_now()
        summary['measured_velocity_at_result']=fixture._executed_velocity.copy()
        if wrapped.result.error_code==0 and any(abs(v)>.005 for v in fixture._executed_velocity):
            raise AssertionError('Success before measured plant stopped')
        summary.update(action_status=wrapped.status,error_code=wrapped.result.error_code,reason=wrapped.result.error_msg,
            reported_distance_m=wrapped.result.distance_traveled,reported_path_length_m=wrapped.result.path_length,
            configured_minimum_speed=wrapped.result.configured_minimum_speed,duration_sec=(time.monotonic_ns()-started)*1e-9)
        intent.active=False;renew();trial.assert_stopped(time.monotonic_ns(),settle_sec=.15,duration_sec=.6)
        rows=[r for r in fixture.events if r['event']=='/cmd_vel_safe' and r['monotonic_ns']>=started]
        nonzero=[r for r in rows if any(abs(v)>1e-7 for v in r['data']['velocity'])]
        if any(not(-.0800001<=r['data']['velocity'][0]<=0) or any(abs(v)>1e-9 for v in r['data']['velocity'][1:]) for r in rows):
            raise AssertionError('Safe output violated reverse-only speed/axis bound')
        distance=math.hypot(fixture.pose[0]-initial[0],fixture.pose[1]-initial[1]);summary['actual_distance_m']=distance
        steps=[r['data'] for r in fixture.events if r['event']=='plant_step' and r['monotonic_ns']>=started]
        actual_path=sum(math.hypot(row['after'][0]-row['before'][0],row['after'][1]-row['before'][1]) for row in steps)
        maximum_excursion=max([math.hypot(row['after'][0]-initial[0],row['after'][1]-initial[1]) for row in steps]+[0.])
        summary.update(distance_budget_m=goal.distance,actual_path_length_m=actual_path,maximum_excursion_m=maximum_excursion)
        if max(distance,actual_path,maximum_excursion,wrapped.result.path_length,wrapped.result.distance_traveled)>goal.distance+1e-9:
            raise AssertionError('Backup exceeded requested maximum distance budget')
        if args.scenario=='rear_during_braking':
            if not summary['costmap_visibility']['visible_while_reversing']:
                raise AssertionError('Dynamic obstacle coverage missing: no costmap-visible obstacle during actual reverse')
            if wrapped.result.error_msg!='Backup stopping footprint sweep blocked':
                raise AssertionError('Dynamic obstacle was not detected in the STOPPING footprint sweep')
        elif args.scenario in ('free','delayed_odom','replan'):
            if wrapped.result.error_code or distance<.01:raise AssertionError('Free backup did not reach target')
        elif args.scenario=='small_budget':
            if not wrapped.result.error_code and distance<.01:raise AssertionError('Small budget falsely reported effective retreat')
        elif not wrapped.result.error_code:raise AssertionError('Failure scenario falsely succeeded')
        if args.scenario in ('blocked','invalid','no_effective','slow_controller','slow_relay','slow_gate') and nonzero:raise AssertionError('Invalid/blocked backup emitted motion')
        if 'injection_ns' in summary and args.scenario not in ('moving_feedback','delayed_odom'):
            trial.assert_stopped(summary['injection_ns'],settle_sec=.4,duration_sec=.1)
        if args.scenario=='replan':
            backup_measurement_end_ns=fixture.flush_to_now()
            timer.cancel()
            nav_handle,nav_result=trial.goal([args.initial[0]+.6,args.initial[1],args.initial[2]])
            trial.spin_until(nav_result.done,25,'Fresh TRACK planning after BackUp')
            if nav_result.result().status!=4:raise AssertionError('Replanning after BackUp failed')
            execution=fixture.latest_messages.get('/nav_epoch/execution')
            if not execution or execution.token.execution_kind!=0 or execution.token.navigation_session_id==token.navigation_session_id:
                raise AssertionError('New TRACK action reused BackUp authority')
            summary['replan_action_status']=nav_result.result().status
            summary['replan_final_pose']=fixture.pose.copy()
            trial.assert_stopped(time.monotonic_ns(),settle_sec=.15,duration_sec=.3)
        summary['success']=True
    except Exception as error:
        summary['error']=repr(error);fixture.record('trial_error',summary)
    finally:
        if visibility:visibility.close()
        trial.close()
        # Include the full last held command/watchdog interval before scoring,
        # even when an assertion/action failed. New TRACK has a separate window.
        polygon,_=load_footprint(args.footprint)
        try:
            if dynamic_injection:
                from recovery_scenarios import audit_dynamic_geometry
                geometry=audit_dynamic_geometry(fixture.events,before_scene,fixture.scene,polygon,dynamic_injection)
            else:geometry=audit_geometry(fixture.events,fixture.scene,polygon)
            summary['geometry']=geometry
            (args.output/'geometry_audit.json').write_text(json.dumps(geometry,indent=2))
            if not geometry['geometry_pass']:
                summary['success']=False;summary.setdefault('final_audit_errors',[]).append('Final full-tail geometry audit failed')
            if started is not None:
                end=backup_measurement_end_ns or fixture._plant_time_ns
                steps=[row['data'] for row in fixture.events if row['event']=='plant_step' and row['data']['interval_start_ns']>=started and row['data']['interval_end_ns']<=end]
                final_pose=steps[-1]['after'] if steps else initial
                path=sum(math.dist(step['before'][:2],step['after'][:2]) for step in steps)
                excursion=max([math.dist(step['after'][:2],initial[:2]) for step in steps]+[0.])
                summary.update(actual_distance_m=math.dist(final_pose[:2],initial[:2]),actual_path_length_m=path,
                    maximum_excursion_m=excursion,measurement_end_ns=end,measurement_after_process_close=True)
                if goal and max(path,excursion)>goal.distance+1e-9:
                    summary['success']=False;summary.setdefault('final_audit_errors',[]).append('Final held-command tail exceeded BackUp distance budget')
        except Exception as error:
            summary['success']=False;summary.setdefault('final_audit_errors',[]).append(repr(error))
        after=runtime_manifest(args.workspace,args.output,'runtime_manifest_after')
        summary['runtime_inputs_unchanged']=before['files']==after['files']
        summary['source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        if not summary['runtime_inputs_unchanged']:summary['success']=False
        summary['scan_timing']=dict(samples=len(scan_timing.samples),maximum_ms=max((sample['duration_ns']/1e6 for sample in scan_timing.samples),default=0.))
        (args.output/'scan_timing.json').write_text(json.dumps(scan_timing.samples,indent=2))
        (args.output/'result.json').write_text(json.dumps(summary,indent=2))
        fixture.close();rclpy.shutdown()
    print(json.dumps(summary,indent=2));return 0 if summary['success'] else 1

if __name__=='__main__':sys.exit(main())
