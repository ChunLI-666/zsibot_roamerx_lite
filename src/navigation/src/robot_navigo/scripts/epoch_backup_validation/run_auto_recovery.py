#!/usr/bin/env python3
"""Exercise the production recovery BT with a real progress-checker failure.

Only NavigateToPose is sent. BackUpEpoch is issued exclusively by the BT plugin.
The localhost SE2 plant represents a temporary traction constraint; no SDK/LCM.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import yaml

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'epoch_navigation_validation'))
from run_stack import prepare,Trial,runtime_manifest
from run_backup_stack import test_map
from recovery_scenarios import ScanTiming,audit_bt_recovery


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.workspace=args.workspace.resolve();args.output=args.output.resolve()
    args.output.mkdir(parents=True,exist_ok=False);(args.output/'COLCON_IGNORE').touch()
    args.map=test_map(args.output,False);args.domain=190;args.initial=[0.,0.,0.];args.geometry=True
    nav=args.workspace/'src/zsibot/zsibot_roamerx_lite/src/navigation/src'
    args.footprint=nav/'robot_navigo/params/zsl1_model_envelope.yaml'
    params=prepare(args.workspace,args.output);config=yaml.safe_load(params.read_text())
    controller=config['controller_server']['ros__parameters']
    controller['enable_epoch_backup']=True;controller['epoch_backup_control_frequency']=20.
    controller['epoch_backup_minimum_speed']=config['velocity_smoother']['ros__parameters']['deadband_velocity'][0]
    bt_path=nav/'navigo_bt_navigator/behavior_trees/navigate_to_pose_with_epoch_backup.xml'
    config['bt_navigator']['ros__parameters']['enable_epoch_backup_recovery']=True
    config['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(bt_path)
    params.write_text(yaml.safe_dump(config))
    inputs=json.loads((args.output/'inputs.json').read_text());inputs['immutable_files']=[str(bt_path)]+[str(path) for path in HERE.glob('*.py')]
    inputs['params_sha256']=hashlib.sha256(params.read_bytes()).hexdigest()
    (args.output/'inputs.json').write_text(json.dumps(inputs,indent=2))
    os.environ.update(ROS_DOMAIN_ID='190',ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from nav2_msgs.msg import BehaviorTreeLog
    from rcl_interfaces.srv import GetParameters
    from rosidl_runtime_py.convert import message_to_ordereddict
    from fixture import Fixture
    from geometry_audit import audit_geometry,load_footprint
    before=runtime_manifest(args.workspace,args.output)
    rclpy.init();fixture=Fixture(args.map,args.initial,args.output,geometry_enabled=True)
    fixture.create_subscription(BehaviorTreeLog,'/behavior_tree_log',fixture.observer('/behavior_tree_log'),100)
    fixture.create_subscription(GoalStatusArray,'/back_up_epoch/_action/status',fixture.observer('/back_up_epoch/_action/status'),20)
    scan=ScanTiming(fixture.scene);fixture.scene.scan=scan.scan
    trial=Trial(args,fixture);summary=dict(scenario='automatic_bt_recovery',success=False,hardware_io=False,
        progress_timeout_s=controller['progress_checker']['movement_time_allowance'],bt_xml=str(bt_path))
    try:
        if not hasattr(fixture,'flush_to_now'):raise RuntimeError('Requires the full wall-time plant before formal execution')
        trial.start('navigation',['ros2','launch','robot_navigo','bringup_launch.py','use_composition:=false','use_sim_time:=true','use_respawn:=false','map:='+str(args.map),'params_file:='+str(params)])
        trial.start('gate',[sys.executable,str(HERE.parent/'nav_safety_gate.py'),'--ros-args','-p','use_sim_time:=true','-p','enable_epoch_contract:=true'])
        trial.lifecycle_ready();trial.verify_loaded_footprints();trial.spin_for(.5)
        parameter_client=fixture.create_client(GetParameters,'/bt_navigator/get_parameters')
        request=GetParameters.Request();request.names=['enable_epoch_contract','enable_epoch_backup_recovery','default_nav_to_pose_bt_xml']
        parameter_future=parameter_client.call_async(request)
        trial.spin_until(parameter_future.done,3,'Read actual navigator tree-selection parameters')
        summary['loaded_parameters']=message_to_ordereddict(parameter_future.result())
        fixture.record('loaded_navigator_parameters',dict(names=request.names,**summary['loaded_parameters']))
        fixture.freeze_motion=True
        fixture.record('traction_constraint',dict(active=True,reason='deterministic progress-checker trigger'))
        handle,result=trial.goal([.8,0.,0.])
        def backup_installed():
            return any(r['event']=='/nav_epoch/execution' and r['data']['active'] and r['data']['token']['execution_kind']==1 for r in fixture.events)
        trial.spin_until(lambda:backup_installed() or result.done(),30,'Automatic BT BackUp after real progress failure')
        if result.done():raise AssertionError('NavigateToPose ended before automatic BackUp')
        fixture.freeze_motion=False
        fixture.record('traction_constraint',dict(active=False,reason='release on real BACKUP execution installation'))
        trial.spin_until(result.done,40,'Same mission replans and reaches the original goal')
        wrapped=result.result();summary['navigation_action_status']=wrapped.status
        if wrapped.status!=4:raise AssertionError('Automatic recovery mission did not succeed')
        trial.assert_stopped(time.monotonic_ns(),settle_sec=.15,duration_sec=.5)
        summary['lineage']=audit_bt_recovery(fixture.events)
        summary['final_pose']=fixture.pose.copy()
        import math
        checker=controller['general_goal_checker']
        summary['goal_tolerance']=dict(xy_m=checker['xy_goal_tolerance'],yaw_rad=checker['yaw_goal_tolerance'])
        if (math.hypot(fixture.pose[0]-.8,fixture.pose[1])>checker['xy_goal_tolerance'] or
                abs(math.atan2(math.sin(fixture.pose[2]),math.cos(fixture.pose[2])))>checker['yaw_goal_tolerance']):
            raise AssertionError('Final mission pose outside frozen production XY/yaw goal tolerance')
        summary['success']=True
    except Exception as error:
        summary['error']=repr(error);fixture.record('trial_error',summary)
    finally:
        trial.close()
        polygon,_=load_footprint(args.footprint);geometry=audit_geometry(fixture.events,fixture.scene,polygon)
        summary.update(geometry=geometry,actual_distance_m=geometry['path_length_m'],measurement_after_process_close=True,final_pose=fixture.pose.copy())
        (args.output/'geometry_audit.json').write_text(json.dumps(geometry,indent=2))
        if not geometry['geometry_pass']:
            summary['success']=False;summary.setdefault('final_audit_errors',[]).append('Final full-tail geometry audit failed')
        after=runtime_manifest(args.workspace,args.output,'runtime_manifest_after')
        summary['runtime_inputs_unchanged']=before['files']==after['files']
        summary['source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        summary['scan_timing']=dict(samples=len(scan.samples),maximum_ms=max((s['duration_ns']/1e6 for s in scan.samples),default=0.))
        (args.output/'scan_timing.json').write_text(json.dumps(scan.samples,indent=2))
        if not summary['runtime_inputs_unchanged']:summary['success']=False
        (args.output/'result.json').write_text(json.dumps(summary,indent=2));fixture.close();rclpy.shutdown()
    print(json.dumps(summary,indent=2));return 0 if summary['success'] else 1

if __name__=='__main__':sys.exit(main())
