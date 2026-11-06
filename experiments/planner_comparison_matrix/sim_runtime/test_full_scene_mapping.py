#!/usr/bin/env python3
"""Pure fixtures and prepare-only CLI tests; no ROS, UDP, motion or relay child."""
import argparse
import ast
import contextlib
import copy
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import full_scene_mapping_contract as c
import full_scene_mapping as owner


def model():
    return dict(max_truth_speed_mps=.6,max_truth_yaw_rate_rps=.5,stop_distance_reserve_m=.15,
      stop_angle_reserve_rad=.15,forward_drift_reserve_m=.05,turn_drift_reserve_m=.1,forward_yaw_reserve_rad=.05,
      max_roll_pitch_rad=.2,body_z_min_m=.2,body_z_max_m=.8,floor_z_m=0.,floor_hit_tolerance_m=.02,
      obstacle_top_m=1.5,basis='root_reviewed_engineering_bound_sim_only')


def grid():return c.OracleGrid(np.ones((600,600),dtype=np.uint8),.05,(-15.,-15.,0.))


def mark(oracle,x,y,state=2):
    ix=math.floor((x-oracle.origin[0])/oracle.resolution);iy=math.floor((y-oracle.origin[1])/oracle.resolution)
    oracle.cells[iy,ix]=state


def truth(stamp=1_000_000_000,received=10.):
    return dict(stamp_ns=stamp,received_steady=received,frame_id='map',child_frame_id='BASE_LINK',
                xyz=[0.,0.,.4],q=[0.,0.,0.,1.],twist=[0.]*6)


def cloud(points):
    payload=b''.join(struct.pack('<ffffBBd',*point) for point in points)
    return SimpleNamespace(is_bigendian=False,height=1,width=len(points),point_step=26,row_step=len(payload),data=payload,
                           fields=[SimpleNamespace(name=name,offset=offset,datatype=kind,count=1) for name,offset,kind in c.POINT_FIELDS])


def route():
    return dict(schema_version=1,scope=c.SCOPE,mode=c.MODE,executable=True,max_runtime_sec=600.,
      scene_instance_id=dict(pid=1172665,starttime_ticks=9427026,cmdline_sha256='1'*64,boot_id='fixture-scene',pid_namespace='pid:[host_fixture]'),
      truth_frame_id=c.TRUTH_FRAME,truth_child_frame_id=c.TRUTH_CHILD,start_anchor_truth=[0.,0.,0.],
      waypoints=[dict(id='fixture-only',pose_truth=[1.,0.,0.],require_stopped=True)])


def oracle_fixture(directory):
    (directory/'grid.pgm').write_bytes(b'P5\n400 400\n255\n'+bytes([255])*160000)
    (directory/'grid.yaml').write_text('image: grid.pgm\nresolution: 0.05\norigin: [-10.0, -10.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    (directory/'alignment.json').write_text('{"scope":"synthetic fixture only"}')
    (directory/'scene_identity.json').write_text(json.dumps(route()['scene_instance_id']))
    config=dict(robot=dict(sensors=dict(lidar=dict(position=dict(x=0.,y=0.,z=.1),rotation=dict(roll=0.,pitch=0.,yaw=0.),
      frame_id='map',sensor_attach='base_link',max_distance=100,simulate_motion_distortion=False,ignore_mounting_actor=True),
      Odom=dict(position=dict(x=0.,y=0.,z=0.),rotation=dict(roll=0.,pitch=0.,yaw=0.),frame_id='map',child_frame_id='base_link',sensor_attach='base_link'))))
    (directory/'config.json').write_text(json.dumps(config))
    artifacts={name:c.sha(directory/name) for name in ['grid.pgm','grid.yaml','alignment.json','config.json','scene_identity.json']}
    value=dict(scope=c.SCOPE,mode=c.MODE,oracle_kind='physical_collision_floor_projection_not_slam_map',
      scene_instance_id=route()['scene_instance_id'],truth_frame_id='map',body_half=[.74,.74],padding=.15,footprint_inflation_in_grid_m=0.,
      floor_height_model=dict(type='flat',floor_z_m=0.),root_geometry_audit={key:True for key in ['complete','coordinate_verified','static_scene_verified','root_domain_approved']},
      frame_alias=dict(verified=True,observed_gt_frame='map',observed_gt_child_frame='BASE_LINK',numeric_transform='identity_verified_against_raw_scan',
        evidence_file='alignment.json',evidence_sha256=artifacts['alignment.json']),
      point_coordinates='sensor_local_flu_despite_raw_frame_id_map',T_body_lidar=dict(xyz=[0.,0.,.1],rpy_rad=[0.,0.,0.]),
      self_return_policy='root_verified_simulator_ignore_mounting_actor_no_geometric_footprint_mask',
      artifacts_sha256=artifacts,sensor_config_json='config.json',grid_yaml='grid.yaml',motion_model=model(),motion_model_reviewed=True)
    return value


def release_fixture(directory,route_path,oracle_path,output):
    identity=c.process_identity(os.getpid())
    ready=directory/'recorder_ready.json';deadline=owner.time.monotonic_ns()+700_000_000_000
    ready.write_text(json.dumps(dict(scope='RAW_RECORDING_ONLY',case_id='synthetic_fixture',recording_run_id='2'*32,
      recorder_process_identity=identity,recorder_deadline_steady_ns=deadline,node_name='fixture_recorder',node_namespace='/',required_recorded_topics=sorted(owner.RAW_RECORD_TOPICS))))
    return dict(scope=c.SCOPE,mode=c.MODE,allow_execute=True,release_id='1'*32,
      route_sha256=c.sha(route_path),oracle_sha256=c.sha(oracle_path),output_directory=str(output),max_runtime_sec=600.,
      sources_sha256={str(path.relative_to(owner.REPO)):c.sha(path) for path in owner.source_files()},
      boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),network_namespace=os.readlink('/proc/self/ns/net'),
      controller_pid_namespace=os.readlink('/proc/self/ns/pid'),
      scene_identity_proof=dict(mode='root_external_host_attestation',evidence_path=str(directory/'scene_identity.json'),evidence_sha256=c.sha(directory/'scene_identity.json'),host_pid_namespace=route()['scene_instance_id']['pid_namespace']),
      retired_writer_roles={role:[identity] for role in owner.REQUIRED_RETIRED_ROLES},allowed_udp_source_processes=[identity],
      recorder_process_identity=identity,recorder_deadline_steady_ns=deadline,case_id='synthetic_fixture',recording_run_id='2'*32,
      required_recorded_topics=sorted(owner.RAW_RECORD_TOPICS),recorder_node_name='fixture_recorder',recorder_node_namespace='/',
      recorder_ready_path=str(ready),recorder_ready_sha256=c.sha(ready),
      expected_sensor_publishers={topic:[dict(name='fixture_sensor',namespace='/',type='fixture/msg/Type')] for topic in owner.GUARDED_SOURCES},
      scene_instance_id=route()['scene_instance_id'])


class GeometryTests(unittest.TestCase):
    def test_filled_interior_and_corner_cells_cannot_hide_between_edges(self):
        for x,y in [(0.,0.),(.86,.86),(-.86,-.86)]:
            oracle=grid();mark(oracle,x,y)
            self.assertFalse(oracle.footprint_free(0.,0.,0.))

    def test_corner_only_collision_during_rotation_between_clear_endpoints(self):
        oracle=grid();mark(oracle,1.15,0.)
        self.assertTrue(oracle.footprint_free(0.,0.,0.))
        self.assertTrue(oracle.footprint_free(0.,0.,math.pi/2))
        with self.assertRaises(c.Hold):oracle.checked_sweep((0.,0.,0.),(0.,0.,math.pi/2))

    def test_full_translation_sweep_catches_thin_mid_segment_obstacle(self):
        oracle=grid();mark(oracle,2.,0.)
        self.assertTrue(oracle.footprint_free(0.,0.,0.));self.assertTrue(oracle.footprint_free(4.,0.,0.))
        with self.assertRaises(c.Hold):oracle.checked_sweep((0.,0.,0.),(4.,0.,0.))

    def test_unknown_and_domain_boundary_are_blocked(self):
        oracle=grid();mark(oracle,.5,.5,0)
        self.assertFalse(oracle.footprint_free(0.,0.,0.))
        self.assertFalse(oracle.footprint_free(14.8,0.,0.))

    def test_stop_reserve_blocks_wall_beyond_instantaneous_body(self):
        oracle=grid();mark(oracle,1.25,0.)
        self.assertTrue(oracle.footprint_free(0.,0.,0.))
        with self.assertRaises(c.Hold):c.stopping_sweep(oracle,(0.,0.,0.),'forward',0,model(),0.)

    def test_fresh_hit_occluder_in_stop_corridor_is_a_stop_even_with_free_oracle(self):
        oracle=grid();samples,inflation=c.stopping_sweep(oracle,(0.,0.,0.),'forward',0,model(),0.)
        with self.assertRaises(c.Hold):c.guard_hits([(1.,0.,.4)],samples,inflation,model(),oracle)

    def test_no_geometric_self_footprint_mask_discards_obstacle_inside_body(self):
        oracle=grid()
        with self.assertRaises(c.Hold):c.guard_hits([(.2,.2,.4)],[(0.,0.,0.)],0.,model(),oracle)

    def test_sparse_hits_do_not_claim_missing_sectors_are_free(self):
        result=c.guard_hits([(-4.,0.,.4)],[(0.,0.,0.)],0.,model(),grid())
        self.assertGreater(result['azimuth_bins_without_hits'],0)
        self.assertIn('do not prove free',result['coverage_semantics'])
        self.assertFalse(result['geometric_self_mask_applied'])

    def test_fixed_heading_bridge_refuses_yaw_tube_or_backward_endpoint_violation(self):
        policy=dict(travel_yaw_rad=0.,max_yaw_error_rad=.05,max_cross_track_m=.05)
        self.assertEqual(c.straight_leg_state((0.,0.,0.),(3.,0.,0.),(1.,.02,.01),policy)['along_m'],1.)
        for pose in [(1.,.06,0.),(1.,0.,.06),(-.2,0.,0.),(3.2,0.,0.)]:
            with self.assertRaises(c.Hold):c.straight_leg_state((0.,0.,0.),(3.,0.,0.),pose,policy)

    def test_route_audit_checks_turns_and_full_bridge_not_only_endpoints(self):
        value=route();value['waypoints'][0]['pose_truth']=[4.,0.,math.pi/2]
        value['waypoints'][0]['leg_policy']=dict(kind='straight_fixed_heading',travel_yaw_rad=0.,max_yaw_error_rad=.05,max_cross_track_m=.05)
        oracle=grid();self.assertTrue(c.audit_route_geometry(oracle,value,model())['checked'])
        mark(oracle,2.,0.)
        with self.assertRaises(c.Hold):c.audit_route_geometry(oracle,value,model())


class SensorTests(unittest.TestCase):
    def test_stale_future_and_callback_delay_rejected(self):
        for change in [dict(stamp_ns=690_000_000),dict(stamp_ns=1_060_000_000),dict(received_steady=9.69)]:
            sample=truth();sample.update(change)
            with self.assertRaises(c.Hold):c.fresh(sample,1_000_000_000,10.)

    def test_nan_empty_invalid_layout_and_nonzero_point_time_rejected(self):
        for points in [[],[(math.nan,0.,1.,10.,0,0,0.)],[(1.,0.,1.,10.,0,0,.1)],[(0.,0.,0.,10.,0,0,0.)]]:
            with self.assertRaises(c.Hold):c.checked_cloud(cloud(points))
        message=cloud([(1.,0.,1.,10.,0,0,0.)]);message.point_step=32
        with self.assertRaises(c.Hold):c.checked_cloud(message)

    def test_positive_near_return_is_never_filtered_as_self(self):
        points=c.checked_cloud(cloud([(.001,.002,.003,10.,0,0,0.)]))
        self.assertEqual(len(points),1)
        with self.assertRaises(c.Hold):c.guard_hits(c.hits_in_truth(points,truth(),dict(xyz=[0.,0.,.1],rpy_rad=[0.,0.,0.])),[(0.,0.,0.)],0.,model(),grid())

    def test_raw_map_label_is_transformed_from_sensor_local_at_original_stamp(self):
        a=truth(0);b=truth(20_000_000);a['xyz']=[0.,0.,.4];b['xyz']=[1.,0.,.4]
        b['q']=[0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)]
        original=copy.deepcopy([a,b]);pose=c.pose_at([a,b],10_000_000)
        points=c.hits_in_truth([(1.,0.,0.)],pose,dict(xyz=[0.,0.,.1],rpy_rad=[0.,0.,0.]))
        self.assertAlmostEqual(points[0][0],.5+math.sqrt(.5));self.assertAlmostEqual(points[0][1],math.sqrt(.5))
        self.assertAlmostEqual(points[0][2],.5);self.assertEqual(pose['stamp_ns'],10_000_000)
        self.assertEqual([a,b],original)

    def test_gt_pose_cannot_be_extrapolated_or_nonfinite_quaternion_normalized(self):
        with self.assertRaises(c.Hold):c.pose_at([truth(1)],2)
        with self.assertRaises(c.Hold):c.checked_quaternion([0.,0.,1.,math.inf])
        with self.assertRaises(c.Hold):c.checked_quaternion([0.,0.,0.,0.])
        sample=truth();sample['child_frame_id']='base_link'
        with self.assertRaises(c.Hold):c.pose_at([sample],sample['stamp_ns'])

    def test_clock_step_rejected(self):
        self.assertEqual(c.check_clock((1_000_000_000,10.),1_050_000_000,10.05),(1_050_000_000,10.05))
        with self.assertRaises(c.Hold):c.check_clock((1_000_000_000,10.),2_000_000_000,10.05)

    def test_stop_requires_finite_truth_and_an_advancing_sample(self):
        checker=c.StableStop();sample=truth()
        self.assertFalse(checker.update(sample,(0.,0.),1_000_000_000,10.,.3))
        self.assertFalse(checker.update(sample,(0.,0.),1_290_000_000,10.29,.3))
        sample=truth(1_310_000_000,10.31)
        self.assertTrue(checker.update(sample,(0.,0.),1_310_000_000,10.31,.3))
        sample['twist'][0]=math.nan;self.assertFalse(c.measured_stop(sample,(0.,0.),sample['stamp_ns'],10.31))

    def test_lying_truth_is_rejected_before_any_zero_walk_relay_can_start(self):
        sample=truth();sample['xyz'][2]=.07305
        with self.assertRaises(c.Hold):c.standing_before_relay(sample,model(),1_000_000_000,10.)
        source=Path(owner.__file__).read_text()
        self.assertLess(source.index('contract.standing_before_relay(latest.get'),source.index('relay=OwnedRelay(output'))


class OwnerAndReleaseTests(unittest.TestCase):
    def test_exact_retired_writer_still_live_blocks_and_reused_pid_needs_independent_udp_authorization(self):
        old=dict(pid=63,starttime_ticks=1,argv=['old_adapter'])
        with self.assertRaises(c.Hold):c.verify_retired([old],lambda pid:dict(old))
        new=dict(old,starttime_ticks=2)
        c.verify_retired([old],lambda pid:new)
        socket=dict(protocol='udp',local='0100007F:C001',remote='00000000:0000',inode='10',owners=[63])
        with self.assertRaises(c.Hold):c.verify_udp_inventory([socket],[],[],lambda pid:new)

    def test_only_exact_owned_relay_and_root_sources_pass_udp_inventory(self):
        relay=dict(pid=77,starttime_ticks=1,argv=['python3','sim_udp_relay_frozen.py'])
        socket=dict(protocol='udp',local='0100007F:C001',remote='00000000:0000',inode='10',owners=[77])
        self.assertTrue(c.verify_udp_inventory([socket],[relay],[],lambda pid:dict(relay)))
        with self.assertRaises(c.Hold):c.verify_udp_inventory([socket],[relay],[],lambda pid:dict(relay,starttime_ticks=2))
        socket['owners']=[]
        with self.assertRaises(c.Hold):c.verify_udp_inventory([socket],[],[])

    def test_once_marker_never_reexecutes(self):
        with tempfile.TemporaryDirectory() as temporary:
            marker=Path(temporary)/'once.json';c.claim_once(marker,dict(run_id='first'))
            with self.assertRaises(FileExistsError):c.claim_once(marker,dict(run_id='second'))
            self.assertEqual(json.loads(marker.read_text())['run_id'],'first')

    def test_only_fixed_calibrated_forward_or_turn_packets(self):
        self.assertEqual(c.primitive_packet(.4,0.,1)['forward'],.2)
        for vx,wz in [(-.1,0.),(.1,0.),(.4,.3),(0.,.4),(0.,math.nan)]:
            with self.assertRaises(c.Hold):c.primitive_packet(vx,wz,1)
        relay_tree=ast.parse((owner.HERE/'sim_udp_relay.py').read_bytes())
        function=next(n for n in relay_tree.body if isinstance(n,ast.FunctionDef) and n.name=='validated_axes')
        ns=dict(math=math);exec(compile(ast.Module(body=[function],type_ignores=[]),'frozen_relay_validation','exec'),ns)
        for now in [210_000_001,-60_000_001]:
            with self.assertRaises(ValueError):ns['validated_axes'](c.primitive_packet(.4,0.,0),now)

    def test_owned_relay_full_pipe_fails_promptly_without_claiming_a_new_command(self):
        read_fd,write_fd=os.pipe();os.set_blocking(write_fd,False)
        try:
            while True:
                try:os.write(write_fd,b'x'*4096)
                except BlockingIOError:break
            relay=owner.OwnedRelay.__new__(owner.OwnedRelay);relay.identity={'pid':77};relay.command=(0.,0.)
            relay.process=SimpleNamespace(pid=77,poll=lambda:None,stdin=SimpleNamespace(fileno=lambda:write_fd))
            with mock.patch.object(c,'process_identity',return_value=relay.identity),self.assertRaises(BlockingIOError):relay.send(.4,0.,preflight=lambda *args:None)
            self.assertEqual(relay.command,(0.,0.))
        finally:os.close(read_fd);os.close(write_fd)

    def test_actual_runtime_budget_branch_stops_before_any_input_or_command(self):
        tree=ast.parse(Path(owner.__file__).read_bytes());execute=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute')
        health=next(n for n in ast.walk(execute) if isinstance(n,ast.FunctionDef) and n.name=='health')
        wrapper=ast.parse('def build():\n cloud_cache=None\n clock_pair=None\n pass\n return health\n').body[0];wrapper.body[2]=health
        ns=dict(contract=c,time=SimpleNamespace(time_ns=lambda:2_000_000_000,monotonic=lambda:2.),started=0.,route={'max_runtime_sec':1.})
        exec(compile(ast.fix_missing_locations(ast.Module(body=[wrapper],type_ignores=[])),'actual_budget_guard','exec'),ns)
        with self.assertRaisesRegex(c.Hold,'budget expired'):ns['build']()()

    def test_actual_rotation_returns_post_turn_measured_stop_not_pre_turn_sample(self):
        tree=ast.parse(Path(owner.__file__).read_bytes())
        function=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='rotate_to')
        before=truth();after=truth(2_000_000_000);after['xyz'][0]=.1
        ns=dict(contract=c,math=math,health=lambda:(before,0.),stable=lambda *args:(after,.01),tick=mock.Mock())
        exec(compile(ast.Module(body=[function],type_ignores=[]),'actual_rotation','exec'),ns)
        sample,yaw=ns['rotate_to'](0.)
        self.assertIs(sample,after);self.assertEqual(yaw,.01);ns['tick'].assert_not_called()

    def test_command_age_upper_includes_computation_and_can_change_the_collision_decision(self):
        sample=truth();upper=c.command_age_upper(.01)
        result=c.command_time_pose_guard(sample,1_090_000_000,10.09,10.01,upper)
        self.assertAlmostEqual(result['header_age_s'],.09);self.assertAlmostEqual(upper,.11)
        with self.assertRaisesRegex(c.Hold,'computation exceeded'):
            c.command_time_pose_guard(sample,1_121_000_000,10.121,10.01,upper)
        oracle=grid();mark(oracle,0.,1.15)
        c.stopping_sweep(oracle,(0.,0.,0.),'forward',0,model(),.01)
        with self.assertRaises(c.Hold):c.stopping_sweep(oracle,(0.,0.,0.),'forward',0,model(),upper)

    def test_actual_tick_rechecks_post_spin_bridge_pose_and_packet_time_computation_age(self):
        tree=ast.parse(Path(owner.__file__).read_bytes());tick=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='tick')
        policy=dict(kind='straight_fixed_heading',travel_yaw_rad=0.,max_yaw_error_rad=.05,max_cross_track_m=.05)
        c.straight_leg_state((0.,0.,0.),(3.,0.,0.),(1.,0.,0.),policy)  # Outer observation was valid.
        for cross,delay,expected in [(0.,.08,None),(.06,0.,'bridge yaw/tube'),(0.,.11,'computation exceeded')]:
            clock=SimpleNamespace(steady=10.)
            clock.monotonic=lambda:clock.steady;clock.time_ns=lambda:1_000_000_000+round((clock.steady-10.)*1e9)
            def spin(node,timeout_sec):clock.steady+=timeout_sec
            latest={};sent=[];records=[]
            def health():
                sample=truth(clock.time_ns()-10_000_000,clock.steady);sample['xyz']=[1.,cross,.4];latest['truth']=sample
                cloud_value=dict(stamp_ns=sample['stamp_ns'],raw_received_steady=clock.steady,points=np.array([[8.,8.,.4]]),pose=sample)
                clock.steady+=delay
                return sample,0.,cloud_value,.01
            def send(vx,wz,preflight):
                preflight(clock.time_ns(),clock.monotonic());sent.append((vx,wz));return c.primitive_packet(vx,wz,clock.time_ns())
            ns=dict(contract=c,time=clock,rclpy=SimpleNamespace(spin_once=spin),node=object(),stopped=[],health=health,oracle=grid(),model=model(),
              relay=SimpleNamespace(send=send),writer=SimpleNamespace(submit=lambda stream,record:records.append(record) or True),latest=latest,
              audit=dict(valid=True,first_violation=None,checked_steady=10.),clock_pair=(1_000_000_000,10.))
            wrapper=ast.parse('def build():\n last_send=None\n last_request=None\n pass\n return tick\n').body[0];wrapper.body[2]=copy.deepcopy(tick)
            exec(compile(ast.fix_missing_locations(ast.Module(body=[wrapper],type_ignores=[])),'actual_tick','exec'),ns)
            actual=ns['build']()
            if expected:
                with self.assertRaisesRegex(c.Hold,expected):actual(.4,0.,bridge=((0.,0.,0.),(3.,0.,0.),policy))
                self.assertEqual(sent,[]);self.assertEqual(records,[])
            else:
                actual(.4,0.,bridge=((0.,0.,0.),(3.,0.,0.),policy))
                self.assertEqual(sent,[(.4,0.)]);self.assertAlmostEqual(records[0]['final_command_guard']['pose']['header_age_s'],.09)

    def test_relay_retains_pidfd_then_waits_for_ready_before_sampling_exec_identity(self):
        events=[];relay=owner.OwnedRelay.__new__(owner.OwnedRelay)
        relay.output=Path('/fixture');relay.run_id='1'*32;relay.frozen=Path('/fixture/relay.py');relay.source_sha256='h'*64
        relay.process=None;relay.pidfd=None;relay.identity=None;relay.log=None
        def ready():events.append('ready');return 'SIM_RELAY_READY 127.0.0.1:7447\n'
        process=SimpleNamespace(pid=77,stdin=SimpleNamespace(fileno=lambda:444),stdout=SimpleNamespace(readline=ready))
        def identity(pid):
            self.assertIn('ready',events);events.append('identity')
            return dict(pid=77,argv=[sys.executable,'-I',str(relay.frozen)])
        with mock.patch.object(owner.subprocess,'Popen',return_value=process),mock.patch.object(owner.os,'pidfd_open',side_effect=lambda pid:events.append('pidfd') or 555),mock.patch.object(owner.os,'set_blocking'),mock.patch.object(owner.select,'select',return_value=([process.stdout],[],[])),mock.patch.object(c,'process_identity',side_effect=identity),mock.patch.object(c,'sha',return_value='h'*64),mock.patch.object(relay,'close') as close:
            relay.start();close.assert_not_called()
        self.assertEqual(events,['pidfd','ready','identity'])
        with mock.patch.object(owner.subprocess,'Popen',return_value=process),mock.patch.object(owner.os,'pidfd_open',return_value=555),mock.patch.object(owner.os,'set_blocking'),mock.patch.object(owner.select,'select',return_value=([],[],[])),mock.patch.object(c,'process_identity') as identity_read,mock.patch.object(relay,'close') as close:
            with self.assertRaises(c.Hold):relay.start()
            close.assert_called_once();identity_read.assert_not_called()

    def test_default_cli_is_hold_without_ros_context_process_or_udp(self):
        with tempfile.TemporaryDirectory() as temporary:
            output=Path(temporary)/'new';before=set(sys.modules)
            with mock.patch.object(sys,'argv',['full_scene_mapping.py','--output',str(output)]),contextlib.redirect_stdout(io.StringIO()),mock.patch.object(owner.subprocess,'Popen',side_effect=AssertionError('no child permitted')) as process,mock.patch.object(owner,'execute',side_effect=AssertionError('no runtime permitted')) as runtime:
                self.assertEqual(owner.main(),0)
                process.assert_not_called();runtime.assert_not_called()
            result=json.loads((output/'result.json').read_text());self.assertTrue(result['classification'].startswith('HOLD'))
            self.assertFalse(json.loads((output/'preparation.json').read_text())['template']['executable'])
            self.assertFalse(any(name.startswith('rclpy') for name in set(sys.modules)-before))

    def test_missing_geometry_and_dynamic_mode_rejected(self):
        value=route();value['mode']='dynamic_scene'
        with self.assertRaises(c.Hold):c.validate_route(value,True)
        value['mode']=c.MODE
        for budget in [math.inf,3600.01,55.,0.,-1.]:
            value['max_runtime_sec']=budget
            with self.assertRaisesRegex(c.Hold,'runtime budget'):c.validate_route(value,True)
        with self.assertRaises(c.Hold):c.validate_oracle(dict(schema_version=1,scope=c.SCOPE,mode=c.MODE),Path('/fixture/oracle.json'))

    def test_oracle_positive_readback_and_tampered_dependency_mount_floor_or_pending_flags(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary);value=oracle_fixture(directory);path=directory/'oracle.json'
            self.assertIs(c.validate_oracle(value,path),value)
            oracle=c.OracleGrid.load(value,path);self.assertTrue(oracle.footprint_free(0.,0.,0.))
            for change in [lambda v:v['root_geometry_audit'].update(coordinate_verified=False),
                           lambda v:v['T_body_lidar'].update(xyz=[0.,0.,0.]),
                           lambda v:v['frame_alias'].update(observed_gt_child_frame='base_link'),
                           lambda v:v.update(footprint_inflation_in_grid_m=1.258),
                           lambda v:v['floor_height_model'].update(floor_z_m=.1),
                           lambda v:v.update(motion_model_reviewed=False)]:
                bad=copy.deepcopy(value);change(bad)
                with self.assertRaises(c.Hold):c.validate_oracle(bad,path)
            (directory/'alignment.json').write_text('changed')
            with self.assertRaisesRegex(c.Hold,'dependency'):c.validate_oracle(value,path)

    def test_release_binds_sources_unique_output_full_retired_identity_budget_and_raw_deadline(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary);rp=directory/'route.json';op=directory/'oracle.json';output=directory/'new'
            rp.write_text(json.dumps(route()));op.write_text(json.dumps(oracle_fixture(directory)));release=release_fixture(directory,rp,op,output)
            self.assertTrue(owner.validate_release(release,route(),rp,op,output))
            for change in [lambda v:v['sources_sha256'].update(extra='0'*64),lambda v:v.update(output_directory=str(directory/'other')),
                           lambda v:v.update(retired_writer_roles={key:[{'pid':63}] for key in owner.REQUIRED_RETIRED_ROLES}),
                           lambda v:v.update(recorder_deadline_steady_ns=owner.time.monotonic_ns()+10_000_000_000),
                           lambda v:v.update(max_runtime_sec=601.),lambda v:v.update(allow_execute=False),
                           lambda v:v.update(controller_pid_namespace='pid:[wrong]'),
                           lambda v:v['scene_identity_proof'].update(host_pid_namespace='pid:[wrong]'),
                           lambda v:v['scene_identity_proof'].update(evidence_sha256='0'*64)]:
                bad=copy.deepcopy(release);change(bad)
                with self.assertRaises(c.Hold):owner.validate_release(bad,route(),rp,op,output)

    def test_positive_prepare_only_full_contract_has_no_child_or_ros_and_unique_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary);rp=directory/'route.json';op=directory/'oracle.json';output=directory/'new'
            rp.write_text(json.dumps(route()));op.write_text(json.dumps(oracle_fixture(directory)))
            args=['mapping','--output',str(output),'--route',str(rp),'--oracle',str(op)]
            with mock.patch.object(sys,'argv',args),contextlib.redirect_stdout(io.StringIO()),mock.patch.object(owner,'execute') as runtime,mock.patch.object(owner.subprocess,'Popen') as process:
                self.assertEqual(owner.main(),0);runtime.assert_not_called();process.assert_not_called()
                self.assertEqual(json.loads((output/'result.json').read_text())['classification'],'PREPARED_ONLY_NO_ROS_NO_MOTION')
                with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):owner.main()

    def test_exit_observation_failures_always_zero_then_close_only_own_relay(self):
        for fail in ['send','observe','close',None]:
            events=[]
            class Relay:
                close_errors=[];process=SimpleNamespace(poll=lambda:0)
                def send(self):
                    events.append('zero')
                    if fail=='send':raise OSError('pipe closed')
                def close(self):
                    events.append('close')
                    if fail=='close':raise OSError('close failure')
            def observe():
                events.append('observe')
                if fail=='observe':raise RuntimeError('ROS context lost')
                return True
            result=owner.finalize_owned_relay(Relay(),observe)
            self.assertEqual(events,['zero','observe','close']);self.assertEqual(result['stop_verified'],fail!='observe')
            if fail=='close':self.assertTrue(result['relay_close_errors'])


if __name__=='__main__':unittest.main()
