#!/usr/bin/env python3
"""Offline contract/process-observer tests. No ROS context, goals or simulation."""
import ast
import argparse
import contextlib
import copy
import hashlib
import io
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import MethodType, SimpleNamespace
import unittest
from unittest import mock
import uuid


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE/'goal_pilot_source'))
from waypoint_contract import (AsyncEvidenceWriter, OrderedVisits, OwnedGoalIdentity, WAYPOINT_PARAMETERS,
                               evidence_json, exclusive_graph, finite_velocity, mission_budget, strict_stopped,
                               validate_ownership, validate_route, validate_settle_budget, validate_scan_aging_confirmation, waypoint_binding)


def route_document():
    return dict(schema_version=1, frame_id='map', mission_budget_sec=100., waypoints=[
        dict(id='first', pose=[0., 0., 0.], require_stopped=True),
        dict(id='same_xy_180', pose=[0., 0., math.pi], require_stopped=True, tags=['in_place_180'])])


def snapshot(x=0., y=0., heading=0., stamp=1):
    return dict(last=dict(pose=dict(frame_id='map', receive_age_s=0., header_age_s=0.,
                                   x=x, y=y, yaw=heading, stamp_ns=stamp),
        odom=dict(frame_id='odom', child_frame_id='base_link', receive_age_s=0., header_age_s=0., vx=0., vy=0., wz=0.),
        **{'/cmd_vel_safe': dict(receive_age_s=0., vx=0., vy=0., wz=0.)}))


def load_method(name):
    tree = ast.parse((HERE/'goal_pilot_source/waypoint_batch.py').read_bytes())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'WaypointPilot')
    method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = dict(OwnedGoalIdentity=OwnedGoalIdentity, ACTION='/follow_waypoints', json=json,
                     evidence_json=evidence_json, finite_velocity=finite_velocity, strict_stopped=strict_stopped, time=time)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(HERE/'goal_pilot_source/waypoint_batch.py'), 'exec'), namespace)
    return namespace[name], namespace


class OrderedVisitTests(unittest.TestCase):
    def tracker(self):
        return OrderedVisits(validate_route(route_document()))

    def pass_first(self, tracker):
        tracker.feedback(0, 0, 0)
        self.assertIsNone(tracker.observe(snapshot(stamp=1), 0, 0, True))
        self.assertIsNotNone(tracker.observe(snapshot(stamp=2), 310_000_000, 310_000_000, True))

    def test_duplicate_xy_keeps_heading_and_requires_independent_second_stop(self):
        tracker = self.tracker()
        self.pass_first(tracker)
        tracker.feedback(1, 400_000_000, 400_000_000)
        tracker.observe(snapshot(stamp=3), 410_000_000, 410_000_000, True)
        tracker.observe(snapshot(stamp=4), 800_000_000, 800_000_000, True)
        self.assertIsNone(tracker.visits[1]['passed'])
        tracker.observe(snapshot(heading=math.pi, stamp=5), 900_000_000, 900_000_000, True)
        tracker.observe(snapshot(heading=math.pi, stamp=6), 1_210_000_000, 1_210_000_000, True)
        self.assertTrue(tracker.summary()['all_visited'])
        self.assertAlmostEqual(tracker.visits[1]['motion']['absolute_yaw_rad'], math.pi)

    def test_feedback_skip_cannot_turn_action_success_into_ordered_visits(self):
        tracker = self.tracker()
        tracker.feedback(0, 0, 0)
        event = tracker.feedback(1, 1, 1)
        self.assertEqual(event['kind'], 'waypoint_order_violation')
        self.assertFalse(tracker.summary()['all_visited'])

    def test_missing_first_feedback_fails_even_at_last_goal(self):
        tracker = self.tracker()
        tracker.feedback(1, 0, 0)
        tracker.observe(snapshot(heading=math.pi), 1_000_000_000, 1_000_000_000, True)
        self.assertEqual(tracker.violation, 'native_feedback_skipped_first_waypoint')

    def test_stale_or_moving_command_resets_continuous_stop(self):
        tracker = self.tracker()
        tracker.feedback(0, 0, 0)
        tracker.observe(snapshot(stamp=1), 0, 0, True)
        moving = snapshot(stamp=2)
        moving['last']['/cmd_vel_safe']['wz'] = .1
        tracker.observe(moving, 290_000_000, 290_000_000, True)
        tracker.observe(snapshot(stamp=3), 310_000_000, 310_000_000, True)
        self.assertIsNone(tracker.observe(snapshot(stamp=4), 590_000_000, 590_000_000, True))
        self.assertIsNotNone(tracker.observe(snapshot(stamp=5), 620_000_000, 620_000_000, True))

    def test_frozen_pose_never_passes_dwell_despite_repeated_snapshot(self):
        tracker = self.tracker()
        tracker.feedback(0, 0, 0)
        tracker.observe(snapshot(stamp=1), 0, 0, True)
        tracker.observe(snapshot(stamp=1), 600_000_000, 600_000_000, True)
        self.assertIsNone(tracker.visits[0]['passed'])

    def test_yaw_wrap_and_native_map_frame_are_checked(self):
        route = validate_route(route_document())
        route[0]['pose'][2] = -math.pi+.02
        tracker = OrderedVisits(route)
        tracker.feedback(0, 0, 0)
        wrong = snapshot(heading=math.pi-.02)
        wrong['last']['pose']['frame_id'] = 'odom'
        tracker.observe(wrong, 0, 0, True)
        self.assertIsNone(tracker.visits[0]['first_within'])
        tracker.observe(snapshot(heading=math.pi-.02, stamp=2), 10_000_000, 10_000_000, True)
        tracker.observe(snapshot(heading=math.pi-.02, stamp=3), 320_000_000, 320_000_000, True)
        self.assertAlmostEqual(tracker.visits[0]['passed']['yaw_error_rad'], .04)

    def test_route_limit_is_each_leg_and_not_initial_global_radius(self):
        document = route_document()
        document['waypoints'] = [dict(id=str(i), pose=[7.*i, 0., 0.], require_stopped=True) for i in range(4)]
        self.assertEqual(len(validate_route(document)), 4)
        document['waypoints'][2]['pose'][0] = 16.
        with self.assertRaises(ValueError):
            validate_route(document)


class OwnershipAndBudgetTests(unittest.TestCase):
    def graph(self):
        return [dict(name='waypoint_follower', namespace='/',
                     clients=[('/navigate_to_pose', ['nav2_msgs/action/NavigateToPose'])],
                     servers=[('/follow_waypoints', ['nav2_msgs/action/FollowWaypoints'])]),
                dict(name='bt_navigator', namespace='/', clients=[],
                     servers=[('/navigate_to_pose', ['nav2_msgs/action/NavigateToPose'])])]

    def test_foreign_client_and_duplicate_node_fail_exclusivity(self):
        graph = self.graph()
        self.assertTrue(exclusive_graph(graph)['exclusive_nav_client'])
        graph.append(dict(name='other_client', namespace='/', clients=graph[0]['clients'], servers=[]))
        self.assertFalse(exclusive_graph(graph)['exclusive_nav_client'])
        self.assertFalse(exclusive_graph(self.graph()+self.graph()[:1])['exclusive_nav_client'])

    def test_active_preexisting_or_other_batch_is_rejected(self):
        ownership = dict(exclusive_nav_client=True, status_publishers_match=True, checked_unix_ns=1_000_000_000,
                         navigate_to_pose_active=[], follow_waypoints_active=[])
        validate_ownership(ownership, 1_010_000_000)
        ownership['navigate_to_pose_active'] = [dict(uuid='b'*32, accepted_stamp_ns=900_000_000)]
        with self.assertRaises(ValueError):
            validate_ownership(ownership, 1_010_000_000)
        with self.assertRaises(ValueError):
            validate_ownership(ownership, 1_010_000_000, 'a'*32, 1_000_000_000)
        ownership['navigate_to_pose_active'] = []
        ownership['follow_waypoints_active'] = [dict(uuid='c'*32)]
        with self.assertRaises(ValueError):
            validate_ownership(ownership, 1_010_000_000, 'a'*32)

    def test_remaining_fixed_deadline_includes_shutdown_reserve_and_boot(self):
        ready = dict(mission_deadline=dict(deadline_boottime_sec=150., launcher_boot_id='boot'))
        self.assertEqual(mission_budget(ready, 100., 20., 'boot')['remaining_sec'], 130.)
        for clock, boot in [(20.001, 'boot'), (20., 'another-boot')]:
            with self.assertRaises(ValueError):
                mission_budget(ready, 100., clock, boot)

    def test_cleanup_observation_cannot_exceed_fixed_shutdown_reserve(self):
        validate_settle_budget(.3)
        validate_settle_budget(8.)
        for seconds in [0., .2, 8.001, 1e9, math.nan, math.inf]:
            with self.assertRaises(ValueError):
                validate_settle_budget(seconds)

    def test_stop_policy_and_source_binding_cannot_change_mid_batch(self):
        ready = dict(waypoint_action_server='/follow_waypoints', mission_deadline=dict(run_id='run',
                     manifest_sha256='d'*64, launcher_boot_id='boot', deadline_boottime_sec=150.),
                     waypoint_contract=dict(package_version='1.3.13',parameters=copy.deepcopy(WAYPOINT_PARAMETERS), number_of_loops=0, goal_index=0,
                                            underlay_files_sha256={'node':'a'*64}, client_sources_sha256={'client':'b'*64}))
        binding = waypoint_binding(ready)
        altered = copy.deepcopy(ready)
        altered['waypoint_contract']['parameters']['stop_on_failure'] = False
        with self.assertRaises(ValueError):
            waypoint_binding(altered, binding)
        altered = copy.deepcopy(ready)
        altered['waypoint_contract']['client_sources_sha256']['client'] = 'c'*64
        with self.assertRaises(ValueError):
            waypoint_binding(altered, binding)


class RawPoseTests(unittest.TestCase):
    def test_nonfinite_quaternion_cannot_hide_behind_finite_projected_yaw(self):
        callback, namespace = load_method('pose_cb')
        base = ast.parse((HERE/'goal_pilot_source/pilot_scan_timing.py').read_bytes())
        functions = [node for node in base.body if isinstance(node, ast.FunctionDef) and node.name in ['yaw', 'stamp_ns']]
        pure = dict(math=math)
        exec(compile(ast.Module(body=functions, type_ignores=[]), 'unchanged_pilot_helpers', 'exec'), pure)
        namespace['frozen'] = SimpleNamespace(yaw=pure['yaw'], stamp_ns=pure['stamp_ns'])
        namespace['math'] = math
        for w, expected in [(1., True), (math.inf, False)]:
            with self.subTest(w=w):
                remembered = []
                message = SimpleNamespace(header=SimpleNamespace(frame_id='map', stamp=SimpleNamespace(sec=12, nanosec=34)),
                    pose=SimpleNamespace(position=SimpleNamespace(x=1., y=2., z=0.),
                        orientation=SimpleNamespace(x=0., y=0., z=1., w=w)))
                callback(SimpleNamespace(remember=lambda key, value: remembered.append((key, value))), message)
                self.assertEqual(len(remembered), 1)
                key, value = remembered[0]
                self.assertEqual((key, value['frame_id'], value['stamp_ns'], value['x'], value['y']),
                                 ('pose', 'map', 12_000_000_034, 1., 2.))
                self.assertTrue(math.isfinite(value['yaw']))
                self.assertEqual(value['raw_pose_values_finite'], expected)
                checked, check_namespace = load_method('execution_ready')
                node = SimpleNamespace(check_interruption=lambda: None,
                    input_ready=lambda **kwargs: (True, [], dict(last=dict(pose=value,**{'/cmd_vel_safe':dict(vx=0.,vy=0.,wz=0.)}))), args=SimpleNamespace(execute=False, scan_aging_confirm_sec=0.),
                    scan_timing_record_check=lambda *args: None)
                valid, issues, snapshot = checked(node)
                self.assertEqual(valid, expected)
                if not expected:
                    self.assertIn('pose:raw_position_or_quaternion_nonfinite_or_missing', issues)
                    self.assertEqual(json.loads(evidence_json(value))['raw_quaternion']['w'], dict(nonfinite_float='+Infinity'))

    def test_moving_safe_command_has_only_finite_gate_without_stop_threshold(self):
        checked, namespace = load_method('execution_ready')
        for velocity, expected in [(dict(vx=.4,vy=0.,wz=.3),True),(dict(vx=math.nan,vy=0.,wz=0.),False),
                                   (dict(vx=0.,vy=math.inf,wz=0.),False),(dict(vx=0.,vy=0.),False)]:
            sample=snapshot()
            sample['last']['pose']['raw_pose_values_finite']=True
            sample['last']['/cmd_vel_safe'].update(velocity)
            if 'wz' not in velocity:sample['last']['/cmd_vel_safe'].pop('wz')
            node=SimpleNamespace(check_interruption=lambda:None,input_ready=lambda **kwargs:(True,[],sample),
                                 args=SimpleNamespace(execute=False, scan_aging_confirm_sec=0.),scan_timing_record_check=lambda *args:None)
            self.assertEqual(checked(node,require_stopped=False)[0],expected)


class WriterAndCancellationTests(unittest.TestCase):
    def test_nonfinite_safe_command_is_not_a_final_stop_and_remains_valid_json_evidence(self):
        for field in ['vx', 'vy', 'wz']:
            sample = snapshot()
            sample['last']['/cmd_vel_safe'][field] = math.nan
            self.assertFalse(strict_stopped(sample)[0])
            encoded = json.loads(evidence_json(dict(first_invalid_snapshot=sample)))
            self.assertEqual(encoded['first_invalid_snapshot']['last']['/cmd_vel_safe'][field],
                             dict(nonfinite_float='NaN'))
            self.assertTrue(math.isnan(sample['last']['/cmd_vel_safe'][field]))

    def test_bounded_queue_counts_gap_and_drains_retained_records(self):
        with tempfile.TemporaryDirectory() as temporary:
            entered, release = threading.Event(), threading.Event()
            def open_slow(path):
                entered.set()
                release.wait(2.)
                return path.open('a')
            writer = AsyncEvidenceWriter(temporary, capacity=2, stream_factory=open_slow)
            writer.submit('scan_trace', dict(index=0))
            self.assertTrue(entered.wait(1.))
            self.assertTrue(writer.submit('scan_trace', dict(index=1)))
            self.assertTrue(writer.submit('scan_trace', dict(index=2)))
            self.assertFalse(writer.submit('scan_trace', dict(index=3)))
            release.set()
            result = writer.close()
            self.assertFalse(result['evidence_complete'])
            self.assertEqual((result['dropped'], result['persisted']), (1, 3))
            records = [json.loads(line) for line in (Path(temporary)/'scan_trace.jsonl').read_text().splitlines()]
            self.assertEqual([record['index'] for record in records], [0, 1, 2])

    def cancel_node(self, namespace, emit):
        sent = []
        own = OwnedGoalIdentity(bytes(range(16)))
        handle = SimpleNamespace(accepted=True, goal_id=SimpleNamespace(uuid=own.value),
                                 cancel_goal_async=lambda: sent.append('own_handle') or 'future')
        service = SimpleNamespace(wait_for_service=lambda timeout_sec: True,
                                  call_async=lambda request: sent.append(bytes(request.goal_info.goal_id.uuid)) or 'future')
        namespace['CancelGoal'] = SimpleNamespace(Request=lambda: SimpleNamespace(goal_info=SimpleNamespace(goal_id=None)))
        namespace['UUID'] = lambda uuid: SimpleNamespace(uuid=uuid)
        node = SimpleNamespace(owned_identity=own, action_result=None, cancel_requested=False, cancel_future=None,
                               action_goal_handle=handle, emit=emit, create_client=lambda kind, name: service)
        return node, sent

    def test_cancel_is_one_owned_uuid_even_if_goal_response_is_wrong_or_missing(self):
        cancel, namespace = load_method('cancel_own')
        for handle_kind in ['own', 'missing', 'wrong']:
            node, sent = self.cancel_node(namespace, lambda *args, **kwargs: None)
            if handle_kind == 'missing':
                node.action_goal_handle = None
            elif handle_kind == 'wrong':
                node.action_goal_handle.goal_id.uuid = bytes(reversed(range(16)))
            cancel(node, 'test')
            cancel(node, 'repeated')
            self.assertEqual(sent, ['own_handle'] if handle_kind == 'own' else [node.owned_identity.value])

    def test_writer_failure_does_not_prevent_cancel_or_final_gap_snapshot(self):
        with tempfile.TemporaryDirectory() as temporary:
            def unavailable(path):
                raise OSError('intentional write failure')
            writer = AsyncEvidenceWriter(temporary, stream_factory=unavailable)
            writer.submit('events', dict(kind='before_failure'))
            writer.thread.join(1.)
            self.assertIsNotNone(writer.summary()['worker_error'])
            cancel, namespace = load_method('cancel_own')
            node, sent = self.cancel_node(namespace, lambda kind, **fields: writer.submit('events', dict(kind=kind, **fields)))
            cancel(node, 'original_guard_input_invalid')
            self.assertEqual(sent, ['own_handle'])
            node.writer, node.output = writer, Path(temporary)
            node.scan_timing_capture_errors, node.scan_timing_last_capture_error = 0, None
            node.scan_timing_firstbad = dict(issues=['scan:invalid_header_age'], scan_ring=[dict(header_ns=10)])
            node.scan_timing_latest_check = node.scan_timing_last_good_check = None
            node.scan_timing_ring = [dict(header_ns=10)]
            finish, _ = load_method('finish_evidence')
            status = finish(node)
            self.assertFalse(status['evidence_complete'])
            persisted = json.loads((Path(temporary)/'scan_timing_evidence.json').read_text())
            self.assertEqual(persisted['first_runtime_invalid']['issues'], ['scan:invalid_header_age'])

    def test_all_zero_uuid_cannot_request_cancel_all(self):
        with self.assertRaises(ValueError):
            OwnedGoalIdentity(bytes(16))


class ExitFlowTests(unittest.TestCase):
    """Execute actual main/run_batch/finalizer AST with a harmless Action transport."""
    def exercise(self, failure, writer_failed=False):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            document = route_document()
            route_path = root/'route.json'
            route_path.write_text(json.dumps(document))
            output = root/'attempt'
            events = []
            tree = ast.parse((HERE/'goal_pilot_source/waypoint_batch.py').read_bytes())
            main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'main')
            cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'WaypointPilot')
            methods = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name in
                       ['run_batch', 'finalize_own_goal', 'cancel_own', 'record_stop', 'check_interruption','execution_ready']]
            future = SimpleNamespace(done=lambda: True,
                                     result=lambda: SimpleNamespace(return_code=0, goals_canceling=[]))
            fake_ros = SimpleNamespace(ok=lambda: True,
                init=lambda **kwargs: events.append(('init', kwargs)), shutdown=lambda: events.append(('shutdown', None)),
                spin_once=lambda *args,**kwargs:None)
            pure={"math":math}
            original=ast.parse((HERE/'goal_pilot_source/pilot_scan_timing.py').read_bytes())
            error_function=next(node for node in original.body if isinstance(node,ast.FunctionDef) and node.name=='error')
            exec(compile(ast.Module(body=[error_function],type_ignores=[]),'unchanged_goal_error','exec'),pure)
            namespace = dict(argparse=argparse, Path=Path, hashlib=hashlib, json=json, math=math, os=os, sys=sys, time=time,
                uuid=uuid, HERE=root, rclpy=fake_ros, SignalHandlerOptions=SimpleNamespace(NO='NO'),
                signal=SimpleNamespace(SIGINT=2, SIGTERM=15, signal=lambda number, handler: 'previous'),
                frozen=SimpleNamespace(validate_numeric_arguments=lambda args: None,
                                       stamp_ns=lambda stamp: stamp.sec*1_000_000_000+stamp.nanosec,error=pure['error']),
                validate_route=validate_route, validate_settle_budget=validate_settle_budget,
                validate_scan_aging_confirmation=validate_scan_aging_confirmation,
                evidence_json=evidence_json, mission_budget=mission_budget,
                strict_stopped=strict_stopped,finite_velocity=finite_velocity, OwnedGoalIdentity=OwnedGoalIdentity, ACTION='/follow_waypoints',
                freeze_sources=lambda destination: dict(sources_sha256={'waypoint_batch.py':'a'*64}),
                UUID=lambda uuid: SimpleNamespace(uuid=uuid),
                CancelGoal=SimpleNamespace(Request=lambda: SimpleNamespace(goal_info=SimpleNamespace(
                    goal_id=None, stamp=SimpleNamespace(sec=0, nanosec=0)))),
                FollowWaypoints=SimpleNamespace(Goal=lambda: SimpleNamespace(poses=[])))
            namespace['PoseStamped'] = lambda: SimpleNamespace(header=SimpleNamespace(frame_id='', stamp=None),
                pose=SimpleNamespace(position=SimpleNamespace(x=0., y=0.), orientation=SimpleNamespace(z=0., w=0.)))
            class Client:
                def __init__(self, node, *args):
                    self.node=node
                def wait_for_server(self, **kwargs):
                    return True
                def send_goal_async(self, request, **kwargs):
                    events.append(('send', dict(poses=len(request.poses), loops=request.number_of_loops, index=request.goal_index)))
                    if failure=='nonfinite_safe_runtime':
                        def cancel_handle():
                            cancellation=namespace['CancelGoal'].Request()
                            cancellation.goal_info.goal_id=kwargs['goal_uuid']
                            return self.node.create_client(namespace['CancelGoal'],'/follow_waypoints/_action/cancel_goal').call_async(cancellation)
                        terminal=SimpleNamespace(done=lambda:self.node.cancel_requested,
                            result=lambda:SimpleNamespace(status=5,result=SimpleNamespace(error_code=0,error_msg='canceled',missed_waypoints=[])))
                        handle=SimpleNamespace(accepted=True,goal_id=kwargs['goal_uuid'],
                            cancel_goal_async=cancel_handle,get_result_async=lambda:terminal)
                        return SimpleNamespace(done=lambda:True,result=lambda:handle)
                    return SimpleNamespace(done=lambda: False)
            namespace['ActionClient'] = Client
            exec(compile(ast.Module(body=methods+[main], type_ignores=[]), 'actual_waypoint_exit_flow', 'exec'), namespace)
            class FakePilot:
                def __init__(self, args, destination, route, source_manifest):
                    self.args, self.output, self.route = args, destination, route
                    self.owned_identity = None if failure in ['response_timeout','nonfinite_safe_runtime'] else OwnedGoalIdentity(bytes(range(16)))
                    self.goal_sent_ns = None if self.owned_identity is None else 1
                    self.goal_sent_steady = None
                    self.goal_uuid = None
                    self.action_goal_handle = self.action_result = self.cancel_future = None
                    self.cancel_requested = self.finalizing = False
                    self.cancel_reason = self.stop_evidence = None
                    self.artifact_invalid_reasons = []
                    self.scan_aging_episodes = []
                    self.scan_timing_capture_errors = 0
                    self.ordered = OrderedVisits(route)
                    self.first_plans={}
                    self.feedback_overflow=0
                    self.event_pub = SimpleNamespace(get_subscription_count=lambda: 1)
                    self.latest_ready = dict(mission_deadline=dict(deadline_boottime_sec=time.clock_gettime(time.CLOCK_BOOTTIME)+1000.,
                        launcher_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()))
                    self.latest_ready_sha = 'b'*64
                    self.native_feedback = lambda message: None
                    for name in ['finalize_own_goal', 'cancel_own', 'record_stop', 'check_interruption']:
                        setattr(self, name, MethodType(namespace[name], self))
                def emit(self, kind, **fields):
                    events.append((kind, fields))
                def spin_until(self, predicate, timeout):
                    return False if timeout == 10. and failure=='response_timeout' else predicate()
                def execution_ready(self, **kwargs):
                    if failure=='nonfinite_safe_runtime':
                        sample=snapshot();sample['last']['pose']['raw_pose_values_finite']=True
                        if self.goal_sent_ns is not None and not self.cancel_requested:
                            sample['last']['/cmd_vel_safe']['vx']=math.nan
                        self.input_ready=lambda **unused:(True,[],sample)
                        self.scan_timing_record_check=lambda *unused:None
                        previous=self.args.execute;self.args.execute=False
                        try:return namespace['execution_ready'](self,**kwargs)
                        finally:self.args.execute=previous
                    return True, [], snapshot()
                def invalidate_artifact(self, issues):
                    self.artifact_invalid_reasons.extend(issues)
                def observe_ordered(self,*args):
                    pass
                def wait_stable_preflight(self, *args):
                    return True, [], snapshot()
                def get_clock(self):
                    return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=int(time.time()), nanosec=0)))
                def run_batch(self):
                    if failure in ['response_timeout','nonfinite_safe_runtime']:
                        return namespace['run_batch'](self)
                    if failure == 'signal':
                        self.args.stop_signals.append(2)
                        self.check_interruption()
                    if failure == 'keyboard_interrupt':
                        raise KeyboardInterrupt('intentional harmless interruption')
                    raise RuntimeError(failure)
                def create_client(self, kind, endpoint):
                    def dispatch(request):
                        events.append(('uuid_cancel_dispatch', dict(endpoint=endpoint,
                            uuid=bytes(request.goal_info.goal_id.uuid),
                            stamp=[request.goal_info.stamp.sec, request.goal_info.stamp.nanosec])))
                        return future
                    return SimpleNamespace(wait_for_service=lambda **kwargs: True, call_async=dispatch)
                def wait_settled(self, seconds):
                    events.append(('stop_observation', seconds))
                    return True, snapshot()
                def finish_evidence(self):
                    events.append(('evidence_close', None))
                    return dict(evidence_complete=not writer_failed)
                def destroy_node(self):
                    events.append(('destroy', None))
            namespace['WaypointPilot'] = FakePilot
            arguments = ['waypoint_batch.py', '--execute', '--route', str(route_path), '--ready', str(root/'ready.json'),
                         '--output', str(output)]
            with mock.patch.object(sys, 'argv', arguments), mock.patch.dict(os.environ, {'ROS_DOMAIN_ID':'185'}), contextlib.redirect_stdout(io.StringIO()):
                namespace['main']()
            result = json.loads((output/'result.json').read_text())
            self.assertEqual(events[0], ('init', dict(signal_handler_options='NO')))
            cancel = next(index for index, event in enumerate(events) if event[0] == 'uuid_cancel_dispatch')
            stop = next(index for index, event in enumerate(events) if event[0] == 'stop_observation')
            close = next(index for index, event in enumerate(events) if event[0] == 'evidence_close')
            shutdown = next(index for index, event in enumerate(events) if event[0] == 'shutdown')
            self.assertLess(cancel, stop)
            self.assertLess(stop, close)
            self.assertLess(close, shutdown)
            target = events[cancel][1]
            self.assertEqual(target['endpoint'], '/follow_waypoints/_action/cancel_goal')
            self.assertEqual(target['stamp'], [0, 0])
            self.assertEqual(len(target['uuid']), 16)
            self.assertTrue(any(target['uuid']))
            self.assertEqual(len([event for event in events if event[0] == 'uuid_cancel_dispatch']), 1)
            self.assertTrue(result['final_measured_and_commanded_stop'])
            return result, events

    def test_goal_response_timeout_finally_cancels_then_observes_stop(self):
        result, events = self.exercise('response_timeout')
        self.assertEqual(result['classification'], 'goal_response_timeout_acceptance_unknown')
        self.assertEqual([event[1] for event in events if event[0] == 'send'], [dict(poses=2, loops=0, index=0)])

    def test_signal_context_stays_alive_through_cancel_and_stop(self):
        result, events = self.exercise('signal')
        self.assertEqual(result['exception_type'], 'InterruptedError')
        self.assertEqual(result['stop_signals'], [2])

    def test_keyboard_interrupt_and_server_result_exceptions_all_observe_stop(self):
        for failure in ['keyboard_interrupt', 'get_result_exception', 'server_exception']:
            with self.subTest(failure=failure):
                result, events = self.exercise(failure)
                self.assertEqual(result['classification'], 'pilot_exception_or_invalid_invocation')

    def test_writer_failed_finally_preserves_cancel_and_stop_then_marks_gap(self):
        result, events = self.exercise('get_result_exception', writer_failed=True)
        self.assertEqual(result['classification'], 'invalid_callback_evidence_gap')

    def test_runtime_nonfinite_safe_command_promptly_cancels_only_own_accepted_goal(self):
        result,events=self.exercise('nonfinite_safe_runtime')
        self.assertEqual(result['classification'],'invalid_runtime_evidence')
        self.assertIn('/cmd_vel_safe:nonfinite_or_missing_velocity',result['cancel_reason'])
        self.assertEqual(result['status'],5)


if __name__ == '__main__':
    unittest.main()
