#!/usr/bin/env python3
"""Launch real Nav2 plus a sensor-only test world in an isolated ROS domain."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml


def merge(base, patch):
    for key,value in patch.items():
        if isinstance(value,dict) and isinstance(base.get(key),dict): merge(base[key],value)
        else: base[key]=copy.deepcopy(value)


def prepare(workspace, output, scenario="normal"):
    repo=workspace/'src/zsibot/zsibot_roamerx_lite/src/navigation/src'
    robot=repo/'robot_navigo'
    inputs=[robot/'params'/name for name in ('navigo_params.yaml','forward_alignment_experiment.yaml','zsl1_model_footprint_overlay.yaml','epoch_navigation_experiment.yaml')]
    config={}
    for path in inputs:
        if not path.exists(): raise RuntimeError(f'Production input is not ready: {path}')
        merge(config,yaml.safe_load(path.read_text()))
    bt=config['bt_navigator']['ros__parameters']
    for label in ('pose','through_poses'):
        filename='navigate_to_pose_with_epoch.xml' if label=='pose' else 'navigate_through_poses_with_epoch.xml'
        path=repo/'navigo_bt_navigator/behavior_trees'/filename
        if not path.exists(): raise RuntimeError(f'Production BT is not ready: {path}')
        bt['default_nav_to_pose_bt_xml' if label=='pose' else 'default_nav_through_poses_bt_xml']=str(path)
    def simulation_time(value):
        if isinstance(value,dict):
            for key,child in value.items():
                if key=='use_sim_time':value[key]=True
                else:simulation_time(child)
    simulation_time(config)
    # No display transport is needed in an automated action integration trial.
    config['controller_server']['ros__parameters']['FollowPath']['visualize']=False
    config['controller_server']['ros__parameters']['FollowPath']['regenerate_noises']=False
    path=output/'params.yaml';path.write_text(yaml.safe_dump(config,sort_keys=False))
    manifest=dict(inputs=[dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in inputs],
                  params_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),hardware_io=False)
    (output/'inputs.json').write_text(json.dumps(manifest,indent=2))
    return path


def runtime_manifest(workspace, output, label="runtime_manifest"):
    from ament_index_python.packages import get_package_prefix
    files=[];dependencies={}
    for package,relative in [('navigo_path_controller','lib/navigo_path_controller/controller_server'),
                             ('navigo_path_planner','lib/navigo_path_planner/planner_server'),
                             ('navigo_bt_navigator','lib/navigo_bt_navigator/bt_navigator'),
                             ('navigo_velocity_optimizer','lib/navigo_velocity_optimizer/velocity_smoother'),
                             ('navigo_mppi_controller','lib/libmppi_controller.so'),
                             ('navigo_mppi_controller','lib/libmppi_critics.so'),
                             ('navigo_behavior_tree','lib/libnavigo_epoch_navigation_bt_node.so')]:
        path=Path(get_package_prefix(package))/relative
        if not path.exists():raise RuntimeError('Required production binary not installed: '+str(path))
        files.append(dict(path=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        listing=subprocess.check_output(['ldd',str(path)],text=True)
        dependencies[str(path.resolve())]=listing
        if 'not found' in listing:raise RuntimeError('Unresolved production shared library: '+str(path))
        for line in listing.splitlines():
            if '=>' in line:
                target=Path(line.split('=>',1)[1].strip().split(' ',1)[0])
                if target.is_file() and str(workspace) in str(target.resolve()):
                    files.append(dict(path=str(target.resolve()),sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    repo=workspace/'src/zsibot/zsibot_roamerx_lite'
    navigation=repo/'src/navigation/src'
    sources=[navigation/name for name in (
        'robot_navigo/scripts/nav_safety_gate.py',
        'navigo_core/include/navigo_core/epoch_contract.hpp',
        'navigo_core/include/navigo_core/controller.hpp',
        'navigo_path_controller/src/controller_server.cpp',
        'navigo_path_controller/include/navigo_path_controller/controller_server.hpp',
        'navigo_velocity_optimizer/src/velocity_smoother.cpp',
        'navigo_velocity_optimizer/include/navigo_velocity_optimizer/velocity_smoother.hpp',
        'navigo_behavior_tree/plugins/action/epoch_navigation.cpp',
        'navigo_bt_navigator/src/navigators/navigate_to_pose.cpp',
        'navigo_bt_navigator/src/navigators/navigate_through_poses.cpp',
        'navigo_util/include/navigo_util/simple_action_server.hpp')]
    sources+=list((navigation/'navigo_epoch_msgs/msg').glob('*.msg'))+list((navigation/'navigo_epoch_msgs/action').glob('*.action'))
    sources+=list(Path(__file__).parent.glob('*.py'))
    for package in ('navigo_path_controller','navigo_velocity_optimizer','navigo_behavior_tree','navigo_bt_navigator','navigo_core','navigo_util'):
        sources+=list((navigation/package/'include').rglob('*.hpp'))
    sources+=list((navigation/'navigo_bt_navigator/behavior_trees').glob('*with_epoch.xml'))
    sources+=list((Path(get_package_prefix('navigo_bt_navigator'))/'share/navigo_bt_navigator/behavior_trees').glob('*with_epoch.xml'))
    sources+=[navigation/'robot_navigo/params'/name for name in ('navigo_params.yaml','forward_alignment_experiment.yaml','zsl1_model_footprint_overlay.yaml','epoch_navigation_experiment.yaml')]
    for source in sources:
        files.append(dict(path=str(source.resolve()),sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
    files=sorted({item['path']:item for item in files}.values(),key=lambda item:item['path'])
    result=dict(files=files,ldd=dependencies,repository_head=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(),
                ros_distro=os.environ.get('ROS_DISTRO'),ros_domain_id=os.environ['ROS_DOMAIN_ID'])
    (output/(label+'.json')).write_text(json.dumps(result,indent=2))
    return result


class Trial:
    def __init__(self, args, fixture):
        self.args=args;self.fixture=fixture;self.processes=[];self.streams=[];self.suspended=[]

    def start(self, name, command, env=None):
        stream=(self.args.output/(name+'.log')).open('w');self.streams.append(stream)
        process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True,env=env)
        self.processes.append(process);self.fixture.record('process_start',dict(name=name,pid=process.pid,command=command))
        return process

    def spin_until(self, predicate, seconds, label):
        import rclpy
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(self.fixture,timeout_sec=.01)
            if predicate(): return
            failed=[(p.pid,p.returncode) for p in self.processes if p.poll() is not None]
            if failed: raise RuntimeError(f'Production process exited while {label}: {failed}')
        raise TimeoutError(label)

    def spin_for(self, seconds):
        start=time.monotonic()
        self.spin_until(lambda:time.monotonic()-start>=seconds,seconds+1,'waiting for observation window')

    def lifecycle_ready(self):
        from lifecycle_msgs.srv import GetState
        clients={name:self.fixture.create_client(GetState,'/'+name+'/get_state') for name in
                 ('map_server','planner_server','controller_server','smoother_server','bt_navigator','velocity_smoother')}
        deadline=time.monotonic()+60
        pending={};active=set()
        while time.monotonic()<deadline:
            for name,client in clients.items():
                if name in active: continue
                if name in pending:
                    if pending[name].done():
                        response=pending.pop(name).result()
                        if response and response.current_state.id==3:active.add(name)
                elif client.service_is_ready(): pending[name]=client.call_async(GetState.Request())
            if len(active)==len(clients) and self.fixture.navigator.server_is_ready():
                self.fixture.record('stack_ready',dict(nodes=sorted(active)));return
            self.spin_for(.1)
        raise TimeoutError('Lifecycle readiness failed: '+str(set(clients)-active))

    def goal(self, position):
        self.last_goal=position
        from nav2_msgs.action import NavigateToPose
        from fixture import stamp,fill_pose
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=stamp(self.fixture.sim_time)
        fill_pose(goal.pose.pose,position)
        future=self.fixture.navigator.send_goal_async(goal)
        self.spin_until(future.done,10,'NavigateToPose goal acknowledgement')
        handle=future.result()
        if not handle.accepted:raise RuntimeError('Real NavigateToPose rejected the test goal')
        self.fixture.record('goal_accepted',dict(uuid=[int(v) for v in handle.goal_id.uuid],pose=position))
        return handle,handle.get_result_async()

    def through_goal(self, positions):
        from nav2_msgs.action import NavigateThroughPoses
        from geometry_msgs.msg import PoseStamped
        from fixture import stamp,fill_pose
        self.spin_until(self.fixture.through_navigator.server_is_ready,10,'NavigateThroughPoses readiness')
        goal=NavigateThroughPoses.Goal();self.last_goal=positions[-1]
        for position in positions:
            pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=stamp(self.fixture.sim_time)
            fill_pose(pose.pose,position);goal.poses.append(pose)
        future=self.fixture.through_navigator.send_goal_async(goal,feedback_callback=lambda msg:self.fixture.record('through_feedback',dict(remaining=msg.feedback.number_of_poses_remaining,distance_remaining=msg.feedback.distance_remaining)))
        self.spin_until(future.done,10,'NavigateThroughPoses acknowledgement')
        handle=future.result()
        if not handle.accepted:raise AssertionError('Real NavigateThroughPoses rejected goal')
        self.fixture.record('goal_accepted',dict(uuid=[int(v) for v in handle.goal_id.uuid],poses=positions,action='NavigateThroughPoses'))
        return handle,handle.get_result_async()

    def moving(self):
        return any(abs(v)>1e-5 for v in self.fixture.safe)

    def assert_stopped(self, after_ns, settle_sec=.30, duration_sec=.8):
        self.spin_for(settle_sec+duration_sec)
        rows=[row for row in self.fixture.events if row['event']=='/cmd_vel_safe' and row['monotonic_ns']>=after_ns+int(settle_sec*1e9)]
        if not rows:raise AssertionError('No safe output during the stop observation window')
        violating=[r for r in rows if any(abs(v)>1e-5 for v in r['data']['velocity'])]
        if violating:raise AssertionError(f'Nonzero safe commands after stop deadline: {len(violating)}')
        self.fixture.record('stop_assertion_pass',dict(after_ns=after_ns,settle_sec=settle_sec,samples=len(rows)))

    def cancel(self, handle, result):
        self.fixture.record('cancel_requested',{})
        after=time.monotonic_ns();future=handle.cancel_goal_async()
        self.spin_until(future.done,5,'Cancel acknowledgement')
        self.spin_until(result.done,10,'Cancelled navigation result')
        status=result.result().status
        self.fixture.record('action_terminal',dict(status=status))
        if status!=5:raise AssertionError(f'Expected real CANCELED action status 5, got {status}')
        self.assert_stopped(after)

    def find_owned_node(self, launch, fragment):
        # Walk only descendants of our own launch process; never use global pkill.
        lines=subprocess.check_output(['ps','-eo','pid=,ppid=,args='],text=True).splitlines()
        processes=[line.strip().split(None,2) for line in lines]
        owned={launch.pid}
        for _ in range(10):
            owned.update(int(pid) for pid,parent,_ in processes if int(parent) in owned)
        candidates=[int(pid) for pid,_,command in processes if int(pid) in owned and fragment in command]
        if len(candidates)!=1:raise RuntimeError(f'Expected one owned {fragment} process, found {candidates}')
        return candidates[0]

    def bare_recovery(self, handle, result):
        from nav2_msgs.action import Spin,BackUp
        from rclpy.action import ActionClient
        from navigo_epoch_msgs.msg import LocalizationEpoch
        self.fixture.change_health(LocalizationEpoch.LOST);self.spin_for(.5)
        for action,name in [(Spin,'/spin'),(BackUp,'/backup')]:
            client=ActionClient(self.fixture,action,name)
            self.spin_until(client.server_is_ready,5,'Real behavior action readiness '+name)
            goal=action.Goal();goal.time_allowance.sec=10
            if name=='/spin':goal.target_yaw=.5
            else:goal.target.x=-.3;goal.speed=.05
            future=client.send_goal_async(goal);self.spin_until(future.done,5,'Behavior action acknowledgement')
            behavior=future.result()
            if not behavior.accepted:raise AssertionError('Actual behavior goal rejected before bypass test')
            after=time.monotonic_ns();self.fixture.record('bare_behavior_started',dict(action=name))
            self.assert_stopped(after,settle_sec=.2,duration_sec=.8)
            raw=[row for row in self.fixture.events if row['event']=='/cmd_vel_nav' and row['monotonic_ns']>=after]
            if not any(abs(row['data']['linear']['x'])+abs(row['data']['angular']['z'])>1e-5 for row in raw):
                raise AssertionError('Behavior did not actually emit a legacy motion command: '+name)
            cancel=behavior.cancel_goal_async();self.spin_until(cancel.done,5,'Behavior cancellation acknowledgement')
            self.fixture.record('bare_behavior_blocked',dict(action=name,raw_samples=len(raw)))
            self.spin_for(.2)
        self.cancel(handle,result)

    def start_proxy_stack(self, params):
        # Test-only process orchestration: unchanged production executables and
        # strict installed BT. Only the chosen action transport is intercepted.
        config=yaml.safe_load(params.read_text())
        config['map_server']['ros__parameters']['yaml_filename']=str(self.args.map)
        nodes=[('navigo_map_server','map_server','map_server'),
               ('navigo_path_controller','controller_server','controller_server'),
               ('navigo_path_planner','planner_server','planner_server'),
               ('navigo_behaviors','behavior_server','behavior_server'),
               ('navigo_velocity_optimizer','velocity_smoother','velocity_smoother'),
               ('nav2_smoother','smoother_server','smoother_server'),
               ('navigo_bt_navigator','bt_navigator','bt_navigator'),
               ('navigo_waypoint_follower','waypoint_follower','waypoint_follower')]
        config['lifecycle_manager_navigation']={'ros__parameters':{'autostart':True,'bond_timeout':4.,
            'node_names':['map_server','controller_server','smoother_server','planner_server','behavior_server','velocity_smoother','bt_navigator','waypoint_follower']}}
        process_params=self.args.output/'process_params.yaml';process_params.write_text(yaml.safe_dump(config))
        for package,executable,name in nodes:
            command=['ros2','run',package,executable,'--ros-args','--params-file',str(process_params),'-r','__node:='+name]
            if name in ('controller_server','behavior_server'):command+=['-r','cmd_vel:=cmd_vel_nav']
            if name=='velocity_smoother':command+=['-r','cmd_vel:=cmd_vel_nav','-r','cmd_vel_smoothed:=cmd_vel']
            if name=='planner_server' and self.args.scenario=='pending_planner':command+=['-r','compute_path_to_pose:=/validation/real_compute_path_to_pose']
            if name=='smoother_server' and self.args.scenario=='pending_smoother':command+=['-r','smooth_path:=/validation/real_smooth_path']
            env=None
            if name=='smoother_server':env=dict(os.environ,LD_LIBRARY_PATH='/opt/ros/'+os.environ.get('ROS_DISTRO','jazzy')+'/lib:'+os.environ.get('LD_LIBRARY_PATH',''))
            self.start(name,command,env=env)
        self.start('lifecycle',['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args','--params-file',str(process_params),'-r','__node:=lifecycle_manager_navigation'])

    def execute(self, params):
        fixture=self.fixture;args=self.args
        if args.scenario in ('pending_planner','pending_smoother'):
            self.start_proxy_stack(params);launch=None
        else:
            launch=self.start('navigation',['ros2','launch','robot_navigo','bringup_launch.py','use_composition:=false','use_sim_time:=true','use_respawn:=false','map:='+str(args.map),'params_file:='+str(params)])
        gate=args.workspace/'src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/nav_safety_gate.py'
        self.start('gate',[sys.executable,str(gate),'--ros-args','-p','use_sim_time:=true','-p','enable_epoch_contract:=true'])
        self.lifecycle_ready()
        if args.scenario in ('pending_planner','pending_smoother'):
            kind='planner' if args.scenario=='pending_planner' else 'smoother'
            self.start('action_proxy',[sys.executable,str(Path(__file__).with_name('action_delay_proxy.py')),'--kind',kind])
            self.spin_until(lambda:fixture.latest.get('proxy_event',{}).get('data',{}).get('event')=='ready',10,'Action delay proxy readiness')
        if args.scenario=='through_poses':
            handle,result=self.through_goal([[args.initial[0]+.7,args.initial[1],0.],[args.initial[0]+2.,args.initial[1],0.]])
        else:
            handle,result=self.goal([args.initial[0]+2.,args.initial[1],0.])
        if args.scenario in ('pending_planner','pending_smoother'):
            from navigo_epoch_msgs.msg import LocalizationEpoch
            from std_msgs.msg import String
            self.spin_until(lambda:any(row['event']=='proxy_event' and row['data']['event']=='held' for row in fixture.events),15,'Real backend result captured and held')
            after=fixture.change_health(LocalizationEpoch.LOST);self.assert_stopped(after)
            fixture.change_health(LocalizationEpoch.NORMAL,recover=True)
            self.spin_until(self.moving,30,'Fresh planning after held old result')
            release_after=time.monotonic_ns()
            fixture.proxy_control.publish(String(data='release'))
            fixture.record('old_backend_result_released',dict(epoch=fixture.epoch))
            self.spin_until(lambda:any(row['event']=='proxy_event' and row['data']['event']=='forwarded' and row['data']['data']['request']==1 for row in fixture.events),5,'Held old result actually delivered after new epoch motion')
            proof=[row for row in fixture.events if row['event']=='proxy_event' and row['data'].get('data',{}).get('request')==1]
            hashes={row['data']['data']['result_sha256'] for row in proof if row['data']['event'] in ('backend_result','held','forwarded')}
            forwarded=next(row for row in proof if row['data']['event']=='forwarded')
            if len(hashes)!=1 or forwarded['monotonic_ns']<release_after:raise AssertionError('Old result changed or was not actually delivered after recovery')
            fixture.record('late_result_delivery_verified',dict(result_sha256=next(iter(hashes)),release_after_ns=release_after))
        if args.scenario=='pose_mismatch':
            after=time.monotonic_ns();self.assert_stopped(after,settle_sec=.30,duration_sec=2.)
            reasons=[row['data'].get('reason','') for row in fixture.events if row['event']=='/nav_epoch/execution']
            if not any('TF' in reason or 'tf_mismatch' in reason for reason in reasons):
                raise AssertionError('No actual controller source-TF rejection observed')
            fixture.typed_pose_bias=0.;fixture.record('typed_pose_bias_removed',{})
        self.spin_until(self.moving,25,'First authorized safe motion')
        self.spin_for(.5)
        if args.scenario=='stuck_replan':
            fixture.freeze_motion=True;after=time.monotonic_ns();fixture.record('plant_motion_blocked',{})
            self.spin_until(lambda:any(row['event']=='/nav_epoch/execution' and 'Failed to make progress' in row['data'].get('reason','') and row['monotonic_ns']>=after for row in fixture.events),18,'Production progress timeout despite periodic replanning')
            revisions={row['data']['token']['plan_sequence'] for row in fixture.events if row['event']=='/nav_epoch/execution' and row['monotonic_ns']>=after}
            if len(revisions)<3:raise AssertionError('Progress timeout test did not observe repeated installed plans')
            fixture.record('stuck_timeout_verified',dict(distinct_plans=len(revisions),elapsed_sec=(time.monotonic_ns()-after)/1e9))
            self.cancel(handle,result);return
        if args.scenario=='future_tf':
            from geometry_msgs.msg import TransformStamped
            from fixture import stamp
            transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='odom'
            transform.header.stamp=stamp(fixture.sim_time+3.);transform.transform.translation.x=1.;transform.transform.rotation.w=1.
            fixture.tf.sendTransform([transform]);after=time.monotonic_ns();fixture.record('old_future_tf_inserted',dict(future_sec=3.,wrong_x=1.))
            self.spin_for(1.5)
            debug=[row['data'] for row in fixture.events if row['event']=='/controller_server/debug' and row['monotonic_ns']>=after+200_000_000]
            if len(debug)<5 or any(not row['source_pose_found'] or row['source_position_error_m']>1e-5 for row in debug):
                raise AssertionError('Controller did not use matching source-stamped typed pose under future TF contamination')
            fixture.record('source_pose_under_future_tf_verified',dict(samples=len(debug)))
        if args.scenario=='preempt':
            old_token=copy.deepcopy(fixture.latest['/cmd_vel_epoch_raw']['data']['token'])
            old_result=result
            handle,result=self.goal([args.initial[0]+1.,args.initial[1]+.5,0.])
            self.spin_until(lambda:fixture.latest.get('/cmd_vel_epoch_raw',{}).get('data',{}).get('token',{}).get('task_sequence',0)>old_token['task_sequence'],20,'New task revision for preempted business goal')
            self.spin_until(old_result.done,10,'Preempted original action terminal state')
            fixture.record('preempted_action_terminal',dict(status=old_result.result().status,old_token=old_token))
            if old_result.result().status==4:raise AssertionError('Original goal incorrectly succeeded during preemption')
        if args.scenario=='localization_session':
            from navigo_epoch_msgs.msg import LocalizationEpoch
            import uuid
            old=fixture.localization_session
            after=fixture.change_health(LocalizationEpoch.LOST);self.assert_stopped(after)
            fixture.localization_session=str(uuid.uuid4());fixture.epoch=1;fixture.commits=1;fixture.heartbeat=0
            from fixture import stamp
            fixture.commit_stamp=stamp(fixture.sim_time);fixture.health=LocalizationEpoch.NORMAL
            fixture.record('localization_session_replaced',dict(old_session=old,new_session=fixture.localization_session,map_instance_unchanged=fixture.map_instance))
            self.spin_until(lambda:self.moving() and fixture.latest.get('/cmd_vel_epoch_raw',{}).get('data',{}).get('token',{}).get('localization',{}).get('process_session_id')==fixture.localization_session,30,'Real navigation authorization for new localization session')
        if args.scenario=='gate_restart':
            old_session=fixture.latest['/nav_epoch/gate_session']['data']['data']
            gate_process=self.processes[-1]
            os.killpg(gate_process.pid,signal.SIGINT);gate_process.wait(timeout=5)
            self.processes.remove(gate_process)
            self.start('gate_restarted',[sys.executable,str(gate),'--ros-args','-p','use_sim_time:=true','-p','enable_epoch_contract:=true'])
            fixture.record('gate_restart',dict(old_session=old_session))
            self.spin_until(lambda:fixture.latest.get('/nav_epoch/gate_session',{}).get('data',{}).get('data') not in (None,old_session),10,'New gate process identity')
            new_session=fixture.latest['/nav_epoch/gate_session']['data']['data']
            self.spin_until(lambda:self.moving() and fixture.latest.get('/cmd_vel_epoch_raw',{}).get('data',{}).get('token',{}).get('gate_session_id')==new_session,20,'New plan bound to restarted gate')
        if args.scenario=='command_reorder':
            self.spin_for(.3)
            recent=fixture.command_history[-1][1]
            older=next((message for received,message in reversed(fixture.command_history[:-1])
                if message.token==recent.token and message.controller_session_id==recent.controller_session_id and
                message.command_sequence<recent.command_sequence and (message.max_age_ms*1_000_000-(time.monotonic_ns()-message.source_steady_time_ns))>150_000_000),None)
            if older is None:raise RuntimeError('No actual recent same-token older command captured')
            smoother=self.find_owned_node(launch,'/navigo_velocity_optimizer/velocity_smoother')
            os.kill(smoother,signal.SIGSTOP);self.suspended.append(smoother)
            after=time.monotonic_ns()
            fixture.record('reordered_command_injection',dict(older_sequence=older.command_sequence,accepted_sequence=recent.command_sequence,source_age_ms=(after-older.source_steady_time_ns)/1e6))
            for _ in range(10):fixture.command_injector.publish(older);self.spin_for(.02)
            self.assert_stopped(after,settle_sec=.10,duration_sec=.30)
            zeros=[row for row in fixture.events if row['event']=='/cmd_vel_safe' and row['monotonic_ns']>=after and not any(abs(v)>1e-5 for v in row['data']['velocity'])]
            if not zeros or zeros[0]['monotonic_ns']-older.source_steady_time_ns>=older.max_age_ms*1_000_000-20_000_000:
                raise AssertionError('Reorder stop was not witnessed before old command TTL expiry')
            fixture.record('fresh_reorder_stop_verified',dict(source_age_at_zero_ms=(zeros[0]['monotonic_ns']-older.source_steady_time_ns)/1e6,max_age_ms=older.max_age_ms))
            os.kill(smoother,signal.SIGCONT);self.suspended.remove(smoother)
            self.cancel(handle,result);return
        if args.scenario=='bare_recovery':
            self.bare_recovery(handle,result);return
        if args.scenario=='cancel':
            self.cancel(handle,result);return
        if args.scenario in ('clock_pause','controller_silence'):
            controller=self.find_owned_node(launch,'/navigo_path_controller/controller_server')
            fixture.pause_clock=args.scenario=='clock_pause';os.kill(controller,signal.SIGSTOP);self.suspended.append(controller)
            fixture.record('clock_paused_controller_suspended',dict(pid=controller,clock_paused=fixture.pause_clock))
            after=time.monotonic_ns()
            # Localization heartbeats continue while ROS time and raw controller input stop.
            self.assert_stopped(after,settle_sec=.50,duration_sec=1.)
            os.kill(controller,signal.SIGCONT);self.suspended.remove(controller);fixture.pause_clock=False
            fixture.record('clock_resumed_controller_resumed',dict(pid=controller))
            self.cancel(handle,result);return
        if args.scenario=='loss_recovery':
            from navigo_epoch_msgs.msg import LocalizationEpoch
            after=fixture.change_health(LocalizationEpoch.LOST)
            self.assert_stopped(after)
            fixture.change_health(LocalizationEpoch.NORMAL,recover=True)
            self.spin_until(self.moving,30,'Motion after new epoch and real replan')
            raw=fixture.latest.get('/cmd_vel_epoch_raw',{}).get('data',{})
            if raw.get('token',{}).get('localization',{}).get('epoch')!=fixture.epoch:
                raise AssertionError('Recovered motion does not reference the new localization epoch')
        self.spin_until(result.done,90,'Real NavigateToPose completion')
        status=result.result().status;fixture.record('action_terminal',dict(status=status))
        if status!=4:raise AssertionError(f'Expected SUCCEEDED action status 4, got {status}')
        if args.scenario=='through_poses':
            remaining=[row['data']['remaining'] for row in fixture.events if row['event']=='through_feedback']
            if not remaining or min(remaining)>=2:raise AssertionError('No actual through-poses pruning witnessed in action feedback')
            fixture.record('through_pruning_verified',dict(min_remaining=min(remaining),max_remaining=max(remaining)))
        import math
        target=self.last_goal;actual=fixture.current_map_pose()
        goal_params=yaml.safe_load(params.read_text())['controller_server']['ros__parameters']['general_goal_checker']
        xy=math.hypot(actual[0]-target[0],actual[1]-target[1])
        yaw=abs(math.atan2(math.sin(actual[2]-target[2]),math.cos(actual[2]-target[2])))
        if xy>goal_params['xy_goal_tolerance']+1e-6 or yaw>goal_params['yaw_goal_tolerance']+1e-6:
            raise AssertionError(f'Action succeeded outside independent goal tolerance: xy={xy}, yaw={yaw}')
        fixture.record('arrival_geometry_verified',dict(xy_error=xy,yaw_error=yaw,goal_checker=goal_params))
        self.assert_stopped(time.monotonic_ns())

    def close(self):
        for pid in self.suspended:
            try:os.kill(pid,signal.SIGCONT)
            except ProcessLookupError:pass
        for process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    try:process.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        for stream in self.streams:stream.close()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--workspace',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--map',type=Path);parser.add_argument('--domain',type=int,choices=range(188,192),default=188)
    parser.add_argument('--scenario',choices=['normal','loss_recovery','clock_pause','controller_silence','cancel','pending_planner','pending_smoother','bare_recovery','gate_restart','command_reorder','preempt','localization_session','pose_mismatch','future_tf','stuck_replan','through_poses'],default='normal')
    parser.add_argument('--initial',type=float,nargs=3,default=[2.817459926495081,.8539874986700582,0.])
    parser.add_argument('--prepare-only',action='store_true')
    args=parser.parse_args();args.workspace=args.workspace.resolve();args.output=args.output.resolve()
    args.map=(args.map or args.workspace/'artifacts/matrix_scene_terrain_wh_gt_filtered_20260827/map.yaml').resolve()
    args.output.mkdir(parents=True,exist_ok=False);(args.output/'COLCON_IGNORE').touch()
    params=prepare(args.workspace,args.output,args.scenario)
    if args.prepare_only: print(params);return
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    before_manifest=runtime_manifest(args.workspace,args.output)
    import rclpy
    from fixture import Fixture
    rclpy.init();fixture=Fixture(args.map,args.initial,args.output)
    if args.scenario in ('loss_recovery','pending_planner','pending_smoother'):
        fixture.map_offset[0]=-.30
        fixture.record('initial_localization_bias',dict(map_to_odom_x=-.30,physical_odom_continuous=True))
    if args.scenario=='pose_mismatch':fixture.typed_pose_bias=.0001
    trial=Trial(args,fixture)
    result=dict(scenario=args.scenario,domain=args.domain,success=False)
    started=time.monotonic()
    try:
        trial.execute(params);result['success']=True
    except Exception as error:
        result['error']=repr(error);fixture.record('trial_error',result)
    finally:
        trial.close();result['duration_sec']=time.monotonic()-started
        import math
        final=fixture.current_map_pose();target=getattr(trial,'last_goal',None)
        result.update(final_map_pose=final,final_odom_pose=fixture.pose,final_safe_velocity=fixture.safe,
            goal=target,xy_error_m=math.hypot(final[0]-target[0],final[1]-target[1]) if target else None,
            yaw_error_rad=abs(math.atan2(math.sin(final[2]-target[2]),math.cos(final[2]-target[2]))) if target else None,
            accepted_goals=[row['data'] for row in fixture.events if row['event']=='goal_accepted'],
            action_statuses=[row['data']['status'] for row in fixture.events if row['event']=='action_terminal'],
            injection_evidence=[row for row in fixture.events if row['event'] in (
                'localization_transition','clock_paused_controller_suspended','cancel_requested','gate_restart',
                'reordered_command_injection','bare_behavior_started','old_backend_result_released','localization_session_replaced',
                'typed_pose_bias_removed','plant_motion_blocked','old_future_tf_inserted','preempted_action_terminal')])
        after_manifest=runtime_manifest(args.workspace,args.output,'runtime_manifest_after')
        result['runtime_inputs_unchanged']=before_manifest['files']==after_manifest['files']
        if not result['runtime_inputs_unchanged']:
            result['success']=False;result['error']='Runtime code or binary changed during this trial; results invalid'
        from audit_events import audit
        wire_audit=audit(fixture.events)
        (args.output/'wire_audit.json').write_text(json.dumps(wire_audit,indent=2))
        result['wire_audit_failures']=len(wire_audit['failures'])
        if result['success'] and wire_audit['failures']:
            result['success']=False;result['error']='Independent wire audit failed; inspect wire_audit.json'
        (args.output/'result.json').write_text(json.dumps(result,indent=2))
        fixture.close();rclpy.shutdown()
    print(json.dumps(result))
    if not result['success']:raise SystemExit(1)


if __name__=='__main__':main()
