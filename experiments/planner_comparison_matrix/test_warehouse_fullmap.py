"""Offline warehouse full-map admission and durable single-goal dispatch tests."""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import time
from types import MethodType, SimpleNamespace
import unittest
import uuid

from test_mission_state import fixture
from test_native_waypoints import load_method, snapshot
from mission_resume import Session
from mission_storage import StorageWorker
from warehouse_fullmap import apply_global_profile, map_geometry, terminal_extent, validate_actual_geometry
from waypoint_contract import mission_budget, validate_initial_request, validate_ownership, validate_route


class WarehouseFullMapTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(self.enterContext(tempfile.TemporaryDirectory()))
        self.image = self.root / 'map.pgm'
        self.width, self.height = 300, 200
        self.image_header = b'P5\n300 200\n255\n'
        self.write_map()
        self.map_document = dict(resolution=.1, origin=[-15., -10., 0.], width=self.width, height=self.height)
        self.geometry = map_geometry(self.map_document, self.image)
        self.geometry.update(image=str(self.image), image_sha256=self.map_hash(), free_thresh=.25,
                             footprint=[[-.6, -.5], [.6, -.5], [.6, .5], [-.6, .5]], padding=.15)
        self.metadata = dict(size_x=self.width, size_y=self.height, resolution=.1,
                             origin=dict(position=dict(x=-15., y=-10.), orientation=dict(z=0., w=1.)))
        self.ready = dict(warehouse_fullmap=self.geometry,
                          actual_costmaps=dict(global_costmap=self.metadata),
                          mission_identity=dict(verified=True, effective_config_sha256='b' * 64))
        self.start = dict(x=-10., y=-5.)
        self.goal = [10., 5., 0.]
        self.document = dict(
            schema_version=1, frame_id='map', mission_budget_sec=100., scenario_profile='warehouse_fullmap',
            waypoints=[dict(id='g', pose=self.goal, require_stopped=True)],
        )
        self.route = validate_route(self.document)

    def write_map(self, pixel=None):
        pixels = bytearray(b'\xff' * (self.width * self.height))
        if pixel is not None:
            col, row, value = pixel
            pixels[row * self.width + col] = value
        self.image.write_bytes(self.image_header + pixels)

    def map_hash(self):
        return hashlib.sha256(self.image.read_bytes()).hexdigest()

    def test_only_explicit_fullmap_route_accepts_exactly_one_goal(self):
        self.assertEqual(len(self.route), 1)
        rolling = copy.deepcopy(self.document)
        del rolling['scenario_profile']
        with self.assertRaisesRegex(ValueError, '2..256 waypoints'):
            validate_route(rolling)
        self.document['waypoints'].append(dict(id='second', pose=[11., 5., 0.], require_stopped=True))
        with self.assertRaisesRegex(ValueError, 'one explicit warehouse_fullmap goal'):
            validate_route(self.document)

    def test_map_geometry_and_bound_long_leg(self):
        self.assertEqual((self.geometry['size_x'], self.geometry['size_y']), (300, 200))
        self.assertEqual((self.geometry['origin_x'], self.geometry['origin_y']), (-15., -10.))
        self.assertEqual(self.geometry['resolution'], .1)
        validate_actual_geometry(self.metadata, self.geometry)
        self.assertTrue(validate_initial_request(8., self.ready, .3, self.start, self.route))

    def test_fullmap_profile_sets_static_global_geometry_at_warehouse_resolution(self):
        document = dict(self.map_document, resolution=.01, origin=[-1.5, -1., 0.])
        geometry = map_geometry(document, self.image)
        parameters = dict(rolling_window=True, width=20, height=20, plugins=['static_layer', 'obstacle_layer'])
        apply_global_profile(parameters, geometry)
        self.assertEqual(parameters, dict(rolling_window=False, width=3, height=2, resolution=.01,
                                          origin_x=-1.5, origin_y=-1., plugins=['static_layer', 'obstacle_layer']))
        self.assertEqual(terminal_extent(geometry), 3.)

    def test_mismatched_actual_costmap_dimensions_resolution_and_origin(self):
        for field, value in (
            ('size_x', 299), ('size_y', 199), ('resolution', math.nan),
            ('origin', dict(position=dict(x=-14., y=-10.), orientation=dict(z=0., w=1.))),
        ):
            with self.subTest(field=field):
                ready = copy.deepcopy(self.ready)
                metadata = ready['actual_costmaps']['global_costmap']
                metadata[field] = value
                with self.assertRaisesRegex(ValueError, 'geometry differs'):
                    validate_actual_geometry(metadata, self.geometry)
                with self.assertRaisesRegex(ValueError, 'Fullmap geometry mismatch'):
                    validate_initial_request(8., ready, .3, self.start, self.route)

    def test_map_image_hash_and_actual_dimensions_are_checked(self):
        self.write_map((0, 0, 254))
        with self.assertRaisesRegex(ValueError, 'image hash mismatch'):
            validate_initial_request(8., self.ready, .3, self.start, self.route)
        self.image.write_bytes(b'P5\n299 200\n255\n' + b'\xff' * (299 * 200))
        self.geometry['image_sha256'] = self.map_hash()
        with self.assertRaisesRegex(ValueError, 'dimensions differ'):
            map_geometry(self.map_document, self.image)
        with self.assertRaisesRegex(ValueError, 'image geometry invalid'):
            validate_initial_request(8., self.ready, .3, self.start, self.route)

    def test_clearance_rejects_outside_unknown_and_padded_rotating_corner(self):
        for goal in ([15., 0., 0.], [-14.9, 0., 0.]):
            with self.subTest(goal=goal):
                self.assertFalse(validate_initial_request(8., self.ready, .3, self.start, [{'pose': goal}]))
        for pixel in (
            (250, 49, 128),  # Unknown at goal.
            (61, 149, 0),    # Near the start, within axis padding plus decision margin.
        ):
            with self.subTest(pixel=pixel):
                self.write_map(pixel)
                self.geometry['image_sha256'] = self.map_hash()
                self.assertFalse(validate_initial_request(8., self.ready, .3, self.start, self.route))

    def test_missing_metadata_unverified_identity_extra_goal_and_relaxed_scan_rejected(self):
        missing = dict(self.ready, actual_costmaps={})
        unverified = dict(self.ready, mission_identity=dict(self.ready['mission_identity'], verified=False))
        for ready, age, route, reason in (
            (missing, .3, self.route, 'evidence missing or invalid'),
            (unverified, .3, self.route, 'strict bound mission identity'),
            (self.ready, .3, self.route * 2, 'one persistent goal'),
            (self.ready, .5, self.route, 'strict bound mission identity'),
        ):
            with self.subTest(reason=reason, age=age, goals=len(route)):
                with self.assertRaisesRegex(ValueError, reason):
                    validate_initial_request(8., ready, age, self.start, route)

    def test_legacy_rolling_uses_strict_eight_metre_first_leg(self):
        rolling = copy.deepcopy(self.document)
        del rolling['scenario_profile']
        rolling['waypoints'] = [
            dict(id='first', pose=[-2.01, -5., 0.], require_stopped=True),
            dict(id='second', pose=[-1., -5., 0.], require_stopped=True),
        ]
        route = validate_route(rolling)
        self.assertTrue(validate_initial_request(8., {}, .3, self.start, route))
        route[0]['pose'][0] = -2.
        self.assertFalse(validate_initial_request(8., {}, .3, self.start, route))
        self.assertFalse(validate_initial_request(8., {}, .3, self.start, self.route))

    def exercise_dispatch(self, case):
        _, identity, policy, _ = fixture()
        worker = StorageWorker(self.root / 'state.db')
        self.addCleanup(worker.close)
        worker.create(json.dumps(self.document).encode(), identity, policy)
        session = Session(worker, str(uuid.uuid4()), self.root / 'attempt')
        now = 1_000_000_000
        boottime = 1000.
        boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.ready.update(
            time_ns=now, valid_until_ns=now + 500_000_000,
            mission_deadline=dict(launcher_boot_id=boot_id, deadline_boottime_sec=boottime + 120.),
            waypoint_ownership=dict(exclusive_nav_client=True, status_publishers_match=True,
                                    checked_unix_ns=now, navigate_to_pose_active=[], follow_waypoints_active=[]),
        )
        sent, events = [], []

        class Pose:
            def __init__(self):
                self.header = SimpleNamespace()
                self.pose = SimpleNamespace(position=SimpleNamespace(), orientation=SimpleNamespace())

        class OfflineClient:
            def __init__(self, pilot, action_type, action_name):
                pass

            def wait_for_server(self, *, timeout_sec):
                return True

            def send_goal_async(self, request, *, goal_uuid, feedback_callback):
                attempt = worker.state()['attempt']
                sent.append((request, str(uuid.UUID(bytes=bytes(goal_uuid.uuid))), attempt))
                return SimpleNamespace(done=lambda: True,
                                       result=lambda: SimpleNamespace(accepted=False, goal_id=goal_uuid))

        clock = SimpleNamespace(time_ns=lambda: now, monotonic_ns=lambda: 10_000_000_000,
                                monotonic=lambda: 10., clock_gettime=lambda clock_id: boottime,
                                CLOCK_BOOTTIME=time.CLOCK_BOOTTIME)

        def sample(x=self.start['x']):
            reading = snapshot(x=x, y=self.start['y'])
            reading['last']['pose']['raw_pose_values_finite'] = True
            return True, [], reading

        def wait_stable(deadline, started):
            if session.action_uuid is not None and case == 'post_intent_geometry':
                self.metadata['size_x'] -= 1
            return sample()

        def emit(kind, **fields):
            events.append((kind, fields))
            if kind == 'post_intent_preflight_passed':
                if case == 'lease_expired':
                    self.ready['valid_until_ns'] = now - 1
                if case == 'recording_deadline':
                    self.ready['mission_deadline']['deadline_boottime_sec'] = boottime + 1.

        methods = {}
        for name in ('run_batch', 'post_intent_preflight', 'decision_ready', 'current_payload_issues'):
            method, namespace = load_method(name)
            namespace.update(time=clock, math=math, uuid=uuid, Path=Path, ActionClient=OfflineClient,
                             FollowWaypoints=SimpleNamespace(Goal=lambda: SimpleNamespace(poses=[])),
                             PoseStamped=Pose, UUID=SimpleNamespace, validate_initial_request=validate_initial_request,
                             validate_ownership=validate_ownership, mission_budget=mission_budget,
                             frozen=SimpleNamespace(stamp_ns=lambda stamp: stamp))
            methods[name] = method
        node = SimpleNamespace(
            mission=session, route=worker.state()['route'], latest_ready=self.ready, latest_ready_sha='a' * 64,
            goal_sent_ns=None, owned_identity=None, storage_preflight_gap=False, check_interruption=lambda: None,
            simulation_scan_max_age_sec=.3, simulation_scan_sequence_fault=None, scan_timing_record_check=lambda *args: None,
            args=SimpleNamespace(execute=True, scan_aging_confirm_sec=0., preflight_seconds=40.,
                                 stable_preflight_seconds=1., timeout=30., simulation_first_leg_max_distance_m=8.,
                                 simulation_scan_max_age_sec=.3),
            event_pub=SimpleNamespace(get_subscription_count=lambda: 1), emit=emit,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: now)),
            native_feedback=lambda message: None, spin_until=lambda predicate, timeout: predicate(),
            execution_ready=lambda **kwargs: sample(), wait_stable_preflight=wait_stable,
            input_ready=lambda **kwargs: sample(-9.849 if case == 'displaced' else self.start['x']),
        )
        for name, method in methods.items():
            setattr(node, name, MethodType(method, node))
        if case == 'pre_intent_geometry':
            self.metadata['size_x'] -= 1
        result = node.run_batch()
        state = worker.state()
        return result, state, session, sent, dict(events)

    def test_original_goal_and_durable_uuid_reach_offline_transport_once(self):
        result, state, session, sent, events = self.exercise_dispatch('eligible')
        self.assertEqual(result['classification'], 'goal_rejected')
        self.assertTrue(session.send_invoked)
        self.assertEqual(state['budgets']['mission_dispatches_reserved'], 1)
        self.assertEqual(state['attempt']['dispatch_state'], 'REJECTED')
        self.assertEqual(len(sent), 1)
        request, sent_uuid, attempt_at_send = sent[0]
        self.assertEqual(attempt_at_send['dispatch_state'], 'INTENT_COMMITTED')
        self.assertEqual(attempt_at_send['parent_action_uuid'], sent_uuid)
        self.assertEqual(events['post_intent_preflight_passed']['action_uuid'], sent_uuid)
        self.assertIn('fullmap_static_start_checked', events)
        self.assertEqual((request.number_of_loops, request.goal_index, len(request.poses)), (0, 0, 1))
        pose = request.poses[0]
        self.assertEqual((pose.header.frame_id, pose.pose.position.x, pose.pose.position.y,
                          pose.pose.orientation.z, pose.pose.orientation.w), ('map', 10., 5., 0., 1.))

    def test_bad_geometry_before_intent_does_not_reserve_or_send(self):
        result, state, session, sent, events = self.exercise_dispatch('pre_intent_geometry')
        self.assertEqual(result['classification'], 'invalid_initial_request_window_goal_not_sent')
        self.assertEqual(result['reason'], 'Fullmap geometry mismatch')
        self.assertEqual(state['budgets']['mission_dispatches_reserved'], 0)
        self.assertIsNone(session.action_uuid)
        self.assertFalse(sent)
        self.assertNotIn('goal_send', events)

    def test_changed_geometry_after_intent_retains_uuid_without_sending(self):
        result, state, session, sent, events = self.exercise_dispatch('post_intent_geometry')
        self.assertEqual(result['classification'], 'invalid_post_intent_preflight_not_sent')
        self.assertEqual(result['issues'], ['post_intent_geometry_invalid:Fullmap geometry mismatch'])
        self.assert_unsent_intent(state, session, sent, events)

    def assert_unsent_intent(self, state, session, sent, events):
        self.assertEqual(state['budgets']['mission_dispatches_reserved'], 1)
        self.assertEqual(state['attempt']['dispatch_state'], 'INTENT_COMMITTED')
        self.assertEqual(state['attempt']['parent_action_uuid'], str(session.action_uuid))
        self.assertFalse(session.send_invoked)
        self.assertFalse(sent)
        self.assertNotIn('goal_send', events)

    def test_pose_displacement_after_static_check_does_not_send(self):
        result, state, session, sent, events = self.exercise_dispatch('displaced')
        self.assertEqual(result['classification'], 'invalid_dispatch_decision_not_sent')
        self.assertIn('fullmap_pose_changed_beyond_checked_static_margin', result['issues'])
        self.assertIn('fullmap_static_start_checked', events)
        self.assert_unsent_intent(state, session, sent, events)

    def test_expired_readiness_lease_at_final_decision_does_not_send(self):
        result, state, session, sent, events = self.exercise_dispatch('lease_expired')
        self.assertEqual(result['classification'], 'invalid_dispatch_decision_not_sent')
        self.assertIn('readiness:launcher_lease_missing_expired_or_future', result['issues'])
        self.assert_unsent_intent(state, session, sent, events)

    def test_insufficient_recording_deadline_after_intent_does_not_send(self):
        result, state, session, sent, events = self.exercise_dispatch('recording_deadline')
        self.assertEqual(result['classification'], 'invalid_recording_deadline_budget_goal_not_sent')
        self.assertIn('Insufficient fixed recording deadline', result['reason'])
        self.assert_unsent_intent(state, session, sent, events)


if __name__ == '__main__':
    unittest.main()
