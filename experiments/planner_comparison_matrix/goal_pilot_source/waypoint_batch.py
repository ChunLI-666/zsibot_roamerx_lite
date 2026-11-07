#!/usr/bin/env python3
"""Independent native FollowWaypoints batch; observation is the default.

The immutable a63b single-goal observer supplies the original 13 subscriptions,
QoS, live-input checks and continuous stopped preflight. This client sends one
native batch and cancels only that batch UUID. It publishes no velocity, TF,
odometry, localization state or child NavigateToPose goal.
"""
import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID

import pilot_scan_timing as frozen
import message_info_contract
from waypoint_contract import (ACTION, AsyncEvidenceWriter, OrderedVisits, OwnedGoalIdentity,
                               evidence_json, finite_velocity, mission_budget, strict_stopped, validate_ownership, validate_route, waypoint_binding)
from waypoint_contract import ownership_metadata_lease_deviation, validate_settle_budget
from waypoint_contract import (scan_aging_confirmation_eligible, scan_aging_confirmation_outcome,
                               scan_confirmation_payload_valid, validate_scan_aging_confirmation)


HERE = Path(__file__).resolve().parent
FROZEN_PILOT_SHA256 = 'a63bce9490f55438e7af5d16025e4dd20d86293cec12f1560d403ecb629cbe31'


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def freeze_sources(output):
    if file_sha(frozen.__file__) != FROZEN_PILOT_SHA256:
        raise ValueError('The historical safety observer bytes differ from the reviewed a63b snapshot')
    sources = [Path(__file__), Path(frozen.__file__), Path(message_info_contract.__file__), HERE/'waypoint_contract.py']
    hashes = {}
    for path in sources:
        target = output/('source_'+path.name)
        target.write_bytes(path.read_bytes())
        hashes[path.name] = file_sha(path)
    manifest = dict(schema='matrix_native_follow_waypoints_v1', sources_sha256=hashes,
        inherited_observer_sha256=FROZEN_PILOT_SHA256, original_subscriptions=13,
        scan_ring_limit=50, trace_policy='Bounded 4096-record nonblocking queue; dedicated writer serializes and flushes at low frequency. No 2048-sample trace truncation.',
        input_limits='Original .3s old/.05s future and .5s READY lease; original JPEG/Info checks and QoS retained.',
        acceptance='One native batch, all original map points independently in order with .3s fresh measured+commanded stop. Native feedback and Action success alone are insufficient.',
        cancellation='Only this client FollowWaypoints UUID; the upstream Jazzy server internally cancels all goals on its NavigateToPose endpoint. Exclusive experiment ownership is required.',
        timing_boundary='Native source pose sample stamps are retained. /plan receipt latency is transport/BT/observer latency, not planner computation time. Child NavigateToPose UUID is unavailable to this client.')
    manifest['nonfinite_evidence_encoding']='Writer/final serialization only: {nonfinite_float: NaN/+Infinity/-Infinity}; original caches, first-invalid references and decision values remain unchanged.'
    manifest['ownership_metadata_lease']='NavigateToPose ownership metadata uses the same 500ms old/50ms future lease as READY. Each checked_unix_ns exceeding the historical 300ms metadata boundary is recorded once with original validation time and READY; this is separate metadata latency evidence and never changes physical-input classification.'
    manifest['stop_cleanup']='Every possibly sent goal reaches finally own-UUID cancellation and a fresh finite measured/commanded stop observation, including response timeout, signal and exception. Default rclpy shutdown-on-SIGINT is disabled until cleanup completes.'
    manifest['scan_aging_confirmation']='Default 0 preserves strict behavior. Opt-in <=.05s drains existing callbacks only after sole scan-header aging; a newer original stamp must restore every original guard. Every episode permanently fails strict timing acceptance; no header-age threshold changes.'
    (output/'instrumentation_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


class WaypointPilot(frozen.Pilot):
    def __init__(self, args, output, route, source_manifest):
        self.route = route
        self.loaded_sources = {'experiments/planner_comparison_matrix/goal_pilot_source/'+name: digest
                               for name,digest in source_manifest['sources_sha256'].items()}
        self.ordered = OrderedVisits(route)
        self.pending_feedback = deque()
        self.feedback_overflow = 0
        self.first_plans = {}
        self.last_state_trace_ns = 0
        self.native_binding = None
        self.owned_identity = None
        self.cancel_requested = False
        self.cancel_future = None
        self.cancel_reason = None
        self.finalizing = False
        self.stop_evidence = None
        self.scan_aging_episodes = []
        self.ownership_metadata_reported_check_ns = set()
        super().__init__(args, output)
        # Base opens this handle but all batch events use the asynchronous writer.
        self.events.close()
        self.writer = AsyncEvidenceWriter(output, publish_event=self.publish_serialized_event)

    def publish_serialized_event(self, line):
        if self.event_pub is not None:
            message = String()
            message.data = line
            self.event_pub.publish(message)

    def emit(self, kind, **fields):
        event = dict(kind=kind, case_uuid=self.case_uuid, case_id=self.args.case_id,
                     variant=self.args.variant, unix_ns=time.time_ns(), steady_ns=time.monotonic_ns(), **fields)
        self.writer.submit('events', event)
        return event

    def scan_timing_init(self):
        # Original first-invalid check/snapshot references and ring50 are retained.
        super().scan_timing_init()
        self.scan_timing_trace = None

    def scan_info_cb(self, msg, info):
        enter_unix_ns, enter_steady_ns = time.time_ns(), time.monotonic_ns()
        value = dict(stamp_ns=frozen.stamp_ns(msg.header.stamp),
                     frame_id=msg.header.frame_id, ranges=len(msg.ranges))
        if self.args.scan_aging_confirm_sec > 0.:
            value['confirmation_payload_valid'] = scan_confirmation_payload_valid(msg)
        remembered = self.remember('scan', value)
        remember_end = time.monotonic_ns()
        try:
            sample = dict(header_ns=remembered['stamp_ns'], frame_id=remembered['frame_id'], ranges=remembered['ranges'],
                callback_enter_unix_ns=enter_unix_ns, callback_enter_steady_ns=enter_steady_ns,
                remembered=dict(remembered), remember_end_steady_ns=remember_end,
                clock_sample_ros_ns=self.get_clock().now().nanoseconds,
                message_info=message_info_contract.message_info_sample(info, enter_unix_ns),
                callback_end_unix_ns=time.time_ns(), callback_end_steady_ns=time.monotonic_ns())
            self.scan_timing_ring.append(sample)
            self.writer.submit('scan_trace', sample)
        except Exception as exc:
            self.scan_timing_capture_errors += 1
            self.scan_timing_last_capture_error = dict(exception_type=type(exc).__name__, exception_message=str(exc)[:512])

    def pose_cb(self, message):
        # Preserve the original source fields and one remember() call. A finite
        # projected yaw alone cannot prove that its raw quaternion was finite.
        pose, quaternion = message.pose, message.pose.orientation
        position = dict(x=pose.position.x, y=pose.position.y, z=pose.position.z)
        orientation = dict(x=quaternion.x, y=quaternion.y, z=quaternion.z, w=quaternion.w)
        self.remember('pose', dict(frame_id=message.header.frame_id, stamp_ns=frozen.stamp_ns(message.header.stamp),
            x=pose.position.x, y=pose.position.y, yaw=frozen.yaw(quaternion), raw_position=position,
            raw_quaternion=orientation, raw_pose_values_finite=all(math.isfinite(value) for value in
                                                               [*position.values(), *orientation.values()])))

    def execution_ready(self, require_stopped=False):
        self.check_interruption()
        entered_unix_ns, entered_steady_ns = time.time_ns(), time.monotonic_ns()
        valid, issues, snapshot = self.input_ready(require_camera=True, require_stopped=require_stopped)
        if snapshot['last'].get('pose', {}).get('raw_pose_values_finite') is not True:
            issues.append('pose:raw_position_or_quaternion_nonfinite_or_missing')
        if not finite_velocity(snapshot['last'].get('/cmd_vel_safe', {})):
            issues.append('/cmd_vel_safe:nonfinite_or_missing_velocity')
        if (self.args.scan_aging_confirm_sec > 0.
                and snapshot['last'].get('scan', {}).get('confirmation_payload_valid') is not True):
            issues.append('scan:confirmation_payload_nonfinite_or_invalid_or_missing')
        if require_stopped:
            stopped, stop_issues = strict_stopped(snapshot)
            if not stopped:
                issues.extend(stop_issues)
        if self.args.execute:
            readiness_stage, ownership_checked_now_ns = 'read_ready', None
            try:
                self.latest_ready, binding, self.latest_ready_sha = frozen.read_ready(self.args.ready, self.args.variant, self.runtime_binding)
                readiness_stage = 'waypoint_binding'
                native = waypoint_binding(self.latest_ready, self.native_binding)
                readiness_stage, ownership_checked_now_ns = 'validate_ownership', time.time_ns()
                metadata_deviation = ownership_metadata_lease_deviation(
                    self.latest_ready, ownership_checked_now_ns, self.ownership_metadata_reported_check_ns)
                if metadata_deviation is not None:
                    self.emit('ownership_metadata_lease_deviation',
                        latest_ready_sha256=self.latest_ready_sha, **metadata_deviation)
                validate_ownership(self.latest_ready.get('waypoint_ownership',{}),ownership_checked_now_ns,
                    None if self.owned_identity is None else self.owned_identity.value.hex(),self.goal_sent_ns)
                readiness_stage = 'tolerances_and_source_binding'
                tolerance = binding['goal_tolerances']
                if tolerance != dict(xy=.15, yaw=.08, stateful=False):
                    raise ValueError('Reviewed batch requires .15m/.08rad nonstateful original-goal tolerances')
                if self.runtime_binding is None:
                    self.runtime_binding, self.native_binding = binding, native
                if native['waypoint_contract']['client_sources_sha256'] != self.loaded_sources:
                    raise ValueError('Loaded batch client sources differ from the launcher source binding')
            except Exception as exc:
                issues.append('readiness:'+str(exc))
                self.emit('readiness_validation_failed', stage=readiness_stage, message=str(exc),
                    ownership_validation_unix_ns=ownership_checked_now_ns,
                    latest_ready_sha256=getattr(self, 'latest_ready_sha', None),
                    latest_ready_is_failing_read=readiness_stage != 'read_ready',
                    latest_ready=getattr(self, 'latest_ready', None))
            if self.event_pub.get_subscription_count() == 0:
                issues.append('goal_event_subscriber_disappeared')
        self.scan_timing_record_check(require_stopped, issues, snapshot, entered_unix_ns, entered_steady_ns)
        return not issues, issues, snapshot

    def confirm_scan_aging(self, issues, snapshot, terminal):
        """Drain callbacks for <=50ms; any recovered run remains strict-timing FAIL."""
        seconds = self.args.scan_aging_confirm_sec
        if (seconds == 0. or terminal.done() or self.cancel_requested
                or not scan_aging_confirmation_eligible(issues, snapshot)):
            return False, issues, snapshot
        started_ns = self.scan_timing_latest_check['check_enter_steady_ns']
        budget_ns = int(seconds*1e9)
        original_stamp = snapshot['last']['scan']['stamp_ns']
        episode = dict(index=len(self.scan_aging_episodes), original_stamp_ns=original_stamp,
            started_steady_ns=started_ns, deadline_steady_ns=started_ns+budget_ns,
            confirmation_budget_sec=seconds, original_issues=list(issues), original_snapshot=snapshot,
            strict_timing_status='FAIL', strict_timing_accepted=False, outcome='pending')
        self.scan_aging_episodes.append(episode)
        self.invalidate_artifact(issues)
        self.emit('performance_violation', violation='scan_header_age', **episode)
        # Do not bridge an unhealthy interval into a continuous stopped visit.
        self.observe_ordered(False, snapshot)
        outcome = 'pending'
        while rclpy.ok():
            self.check_interruption()
            now_ns = time.monotonic_ns()
            elapsed = time.monotonic()-self.goal_sent_steady
            if (terminal.done() or self.cancel_requested or self.ordered.violation or self.feedback_overflow
                    or elapsed >= self.args.timeout
                    or (self.args.cancel_after is not None and elapsed >= self.args.cancel_after)):
                outcome = 'interrupted_by_existing_terminal_or_cancel_guard'
                break
            remaining_ns = started_ns+budget_ns-now_ns
            if remaining_ns <= 0:
                outcome = 'expired'
                break
            rclpy.spin_once(self, timeout_sec=min(.005, remaining_ns/1e9))
            _, current_issues, current_snapshot = self.execution_ready()
            checked_ns = time.monotonic_ns()
            outcome = scan_aging_confirmation_outcome(current_issues, current_snapshot, original_stamp,
                                                       checked_ns-started_ns, budget_ns)
            self.emit('scan_aging_confirmation_check', episode_index=episode['index'], outcome=outcome,
                checked_steady_ns=checked_ns, elapsed_ns=checked_ns-started_ns,
                issues=list(current_issues), snapshot=current_snapshot)
            if outcome == 'recovered':
                episode.update(outcome=outcome, completed_steady_ns=checked_ns,
                    recovered_stamp_ns=current_snapshot['last']['scan']['stamp_ns'],
                    recovery_snapshot=current_snapshot)
                self.emit('scan_aging_confirmation_finished', **episode)
                return True, current_issues, current_snapshot
            if outcome != 'pending':
                issues = list(current_issues) or ['scan:aging_confirmation_not_recovered']
                snapshot = current_snapshot
                break
            issues, snapshot = list(current_issues), current_snapshot
        if outcome == 'pending':
            outcome = 'ros_context_stopped'
        episode.update(outcome=outcome, completed_steady_ns=time.monotonic_ns(),
                       final_issues=list(issues), final_snapshot=snapshot)
        self.emit('scan_aging_confirmation_finished', **episode)
        return False, issues, snapshot

    def check_interruption(self):
        if self.args.stop_signals and not self.finalizing:
            raise InterruptedError('Controlled stop requested by signal '+str(self.args.stop_signals[0]))

    def spin_until(self, predicate, timeout):
        end = time.monotonic()+timeout
        while rclpy.ok() and time.monotonic() < end:
            self.check_interruption()
            rclpy.spin_once(self, timeout_sec=.02)
            if predicate():
                return True
        return False

    def record_stop(self, settled, snapshot, basis):
        strict, issues = strict_stopped(snapshot)
        self.stop_evidence = dict(attempted=True, observed=bool(settled and strict), basis=basis,
            checked_unix_ns=time.time_ns(), strict_stop_issues=issues, snapshot=snapshot)
        return self.stop_evidence

    def finalize_own_goal(self):
        """All post-send exits reach cancellation, then independent stop evidence."""
        self.finalizing = True
        if self.owned_identity is None:
            return dict(goal_may_have_been_sent=False, stop_check_required=False)
        report = dict(goal_may_have_been_sent=True, action_uuid=self.owned_identity.value.hex(),
                      cancel_response_observed=None)
        try:
            cancel = self.cancel_own('pilot_finally_without_terminal_result')
            if cancel is not None:
                report['cancel_response_observed'] = cancel.done() or self.spin_until(cancel.done, 3.)
                if report['cancel_response_observed']:
                    response = cancel.result()
                    report['cancel_return_code'] = int(response.return_code)
                    report['goals_canceling'] = [bytes(goal.goal_id.uuid).hex() for goal in response.goals_canceling]
        except BaseException as exc:
            report['cancel_observation_error'] = dict(type=type(exc).__name__, message=str(exc))
        if self.stop_evidence is None:
            try:
                settled, snapshot = self.wait_settled(self.args.settle_seconds)
                self.record_stop(settled, snapshot, 'finally_after_owned_uuid_cancel_or_terminal_exception')
            except BaseException as exc:
                self.stop_evidence = dict(attempted=True, observed=False, basis='finally_stop_observation_failed',
                                          error_type=type(exc).__name__, error_message=str(exc))
        report['stop'] = self.stop_evidence
        self.emit('owned_goal_cleanup', **report)
        return report

    def native_feedback(self, message):
        entry = dict(index=int(message.feedback.current_waypoint), unix_ns=time.time_ns(), steady_ns=time.monotonic_ns())
        if len(self.pending_feedback) >= 512:
            self.feedback_overflow += 1
        else:
            self.pending_feedback.append(entry)
        self.emit('action_feedback', action_uuid=None if self.owned_identity is None else self.owned_identity.value.hex(),
                  current_waypoint=entry['index'], callback_unix_ns=entry['unix_ns'], callback_steady_ns=entry['steady_ns'],
                  association='Native FollowWaypoints feedback; child NavigateToPose UUID is not exposed.')

    def plan_cb(self, message):
        self.counts['plan'] += 1
        if not self.args.execute or self.goal_sent_ns is None or not message.poses:
            return
        pose = message.poses[-1].pose
        endpoint = dict(x=pose.position.x, y=pose.position.y, yaw=frozen.yaw(pose.orientation))
        matches = [entry['index'] for entry in self.route if message.header.frame_id == 'map'
                   and math.hypot(endpoint['x']-entry['pose'][0], endpoint['y']-entry['pose'][1]) <= .02
                   and abs(math.atan2(math.sin(endpoint['yaw']-entry['pose'][2]), math.cos(endpoint['yaw']-entry['pose'][2]))) <= .10]
        received = time.monotonic_ns()
        item = dict(frame_id=message.header.frame_id, stamp_ns=frozen.stamp_ns(message.header.stamp), poses=len(message.poses),
                    endpoint=endpoint, matching_original_indices=matches, parent_action_uuid=self.owned_identity.value.hex(),
                    child_action_uuid=None, native_feedback_index=self.ordered.active, received_steady_ns=received,
                    association='Endpoint/time inference only; /plan contains no Action UUID or route index.')
        self.emit('plan_observation', **item)
        if len(matches) == 1 and matches[0] not in self.first_plans:
            self.first_plans[matches[0]] = item

    def observe_ordered(self, healthy, snapshot):
        while self.pending_feedback:
            feedback = self.pending_feedback.popleft()
            event = self.ordered.feedback(feedback['index'], feedback['steady_ns'], feedback['unix_ns'])
            if event:
                kind = event.pop('kind')
                self.emit(kind, parent_action_uuid=self.owned_identity.value.hex(), child_action_uuid=None, **event)
        now = time.monotonic_ns()
        event = self.ordered.observe(snapshot, now, time.time_ns(), healthy)
        if event:
            kind = event.pop('kind')
            self.emit(kind, parent_action_uuid=self.owned_identity.value.hex(), child_action_uuid=None, **event)
        if now-self.last_state_trace_ns >= 100_000_000:
            self.last_state_trace_ns = now
            self.writer.submit('state_trace', dict(unix_ns=time.time_ns(), steady_ns=now,
                parent_action_uuid=self.owned_identity.value.hex(), child_action_uuid=None,
                native_feedback_index=self.ordered.active,
                launcher_action_observation=self.latest_ready.get('waypoint_ownership',{}),
                phase=None if self.ordered.active is None else self.ordered.visits[self.ordered.active]['phase'],
                healthy=healthy, snapshot=snapshot))

    def cancel_own(self, reason):
        """Single own-UUID dispatch, including missing or delayed goal responses."""
        if self.owned_identity is None or self.action_result is not None or self.cancel_requested:
            return self.cancel_future
        self.cancel_requested, self.cancel_reason = True, reason
        self.emit('cancel_send', action_uuid=self.owned_identity.value.hex(), reason=reason)
        try:
            matching_handle = self.action_goal_handle is not None and self.action_goal_handle.accepted
            if matching_handle:
                try:
                    self.owned_identity.verify(self.action_goal_handle.goal_id.uuid)
                except ValueError:
                    matching_handle = False
                    self.emit('foreign_action_handle_rejected', action_uuid=self.owned_identity.value.hex())
            if matching_handle:
                self.cancel_future = self.action_goal_handle.cancel_goal_async()
            else:
                service = self.create_client(CancelGoal, ACTION+'/_action/cancel_goal')
                request = CancelGoal.Request()
                request.goal_info.goal_id = UUID(uuid=list(self.owned_identity.value))
                if service.wait_for_service(timeout_sec=2.):
                    self.cancel_future = service.call_async(request)
                else:
                    self.emit('cancel_service_unavailable', action_uuid=self.owned_identity.value.hex())
        except Exception as exc:
            self.emit('cancel_dispatch_error',action_uuid=self.owned_identity.value.hex(),type=type(exc).__name__,message=str(exc))
        return self.cancel_future

    def run_batch(self):
        self.emit('case_created', requested_route_map=self.route,
                  evidence_boundary='Native Lightning map pose only; GT /odom is reserved for offline comparison.')
        if not self.spin_until(lambda: self.event_pub.get_subscription_count() > 0, 5.):
            return dict(classification='invalid_event_recording_not_ready_goal_not_sent')
        started = time.monotonic()
        deadline = started+self.args.preflight_seconds
        self.emit('preflight_budget_started', timeout_s=self.args.preflight_seconds,
                  required_continuous_healthy_s=self.args.stable_preflight_seconds, deadline_steady_s=deadline)
        if not self.spin_until(lambda: self.execution_ready(require_stopped=True)[0], max(0., deadline-time.monotonic())):
            valid, issues, snapshot = self.execution_ready(require_stopped=True)
            return dict(classification='invalid_preflight_not_sent', issues=issues, snapshot=snapshot)
        client = ActionClient(self, FollowWaypoints, ACTION)
        if not client.wait_for_server(timeout_sec=min(5., max(0., deadline-time.monotonic()))):
            return dict(classification='invalid_action_server_unavailable_goal_not_sent')
        valid, issues, snapshot = self.wait_stable_preflight(deadline, started)
        if not valid:
            return dict(classification='invalid_preflight_not_sent', issues=issues, snapshot=snapshot)
        valid, issues, snapshot = self.execution_ready(require_stopped=True)
        if not valid:
            return dict(classification='invalid_final_preflight_not_sent', issues=issues, snapshot=snapshot)
        start_pose = snapshot['last']['pose']
        if math.hypot(self.route[0]['pose'][0]-start_pose['x'], self.route[0]['pose'][1]-start_pose['y']) >= 8.:
            return dict(classification='invalid_first_leg_outside_reviewed_rolling_window_goal_not_sent')
        request = FollowWaypoints.Goal()
        try:
            budget = mission_budget(self.latest_ready,self.args.timeout,time.clock_gettime(time.CLOCK_BOOTTIME),
                                    Path('/proc/sys/kernel/random/boot_id').read_text().strip())
        except ValueError as exc:
            return dict(classification='invalid_recording_deadline_budget_goal_not_sent',reason=str(exc))
        request.number_of_loops, request.goal_index = 0, 0
        for entry in self.route:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            # Route intent is created now; native input sample stamps stay in snapshots.
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = entry['pose'][:2]
            pose.pose.orientation.z = math.sin(entry['pose'][2]/2.)
            pose.pose.orientation.w = math.cos(entry['pose'][2]/2.)
            request.poses.append(pose)
        requested_uuid = uuid.uuid4()
        self.owned_identity = OwnedGoalIdentity(requested_uuid.bytes)
        self.goal_uuid = UUID(uuid=list(requested_uuid.bytes))
        self.goal_sent_steady, self.goal_sent_ns = time.monotonic(), time.time_ns()
        self.emit('goal_send', action_uuid=str(requested_uuid), action_server=ACTION,
                  number_of_loops=0, goal_index=0, requested_route_map=self.route, start_snapshot=snapshot,
                  readiness_manifest=self.latest_ready, readiness_manifest_sha256=self.latest_ready_sha,
                  preregistered_budget=budget,
                  pose_header_stamps_ns=[frozen.stamp_ns(pose.header.stamp) for pose in request.poses])
        sent = client.send_goal_async(request, goal_uuid=self.goal_uuid, feedback_callback=self.native_feedback)
        if not self.spin_until(sent.done, 10.):
            return dict(classification='goal_response_timeout_acceptance_unknown')
        self.action_goal_handle = sent.result()
        self.owned_identity.verify(self.action_goal_handle.goal_id.uuid)
        self.emit('goal_response', accepted=bool(self.action_goal_handle.accepted), action_uuid=str(requested_uuid))
        if not self.action_goal_handle.accepted:
            return dict(classification='goal_rejected')
        terminal = self.action_goal_handle.get_result_async()
        cancel_logged = False
        while rclpy.ok() and not terminal.done():
            rclpy.spin_once(self, timeout_sec=.02)
            elapsed = time.monotonic()-self.goal_sent_steady
            valid, issues, snapshot = self.execution_ready()
            if not valid and self.args.scan_aging_confirm_sec > 0.:
                valid, issues, snapshot = self.confirm_scan_aging(issues, snapshot, terminal)
                elapsed = time.monotonic()-self.goal_sent_steady
            if not valid:
                self.invalidate_artifact(issues)
            self.observe_ordered(valid, snapshot)
            reason = 'input_became_invalid:'+','.join(issues) if not valid else None
            if self.ordered.violation:
                reason = self.ordered.violation
            if self.feedback_overflow:
                reason = 'native_feedback_observation_queue_overflow'
            if self.args.cancel_after is not None and elapsed >= self.args.cancel_after:
                reason = 'scheduled_cancel'
            if elapsed >= self.args.timeout:
                reason = 'pilot_navigation_deadline'
            if reason:
                self.cancel_own(reason)
            if self.cancel_future is not None and self.cancel_future.done() and not cancel_logged:
                cancel_logged = True
                response = self.cancel_future.result()
                self.emit('cancel_response', return_code=int(response.return_code),
                          goals_canceling=[bytes(goal.goal_id.uuid).hex() for goal in response.goals_canceling])
            if elapsed >= self.args.timeout+10.:
                break
        valid, issues, snapshot = self.execution_ready()
        if not valid:
            self.invalidate_artifact(issues)
        self.observe_ordered(valid, snapshot)
        if terminal.done():
            self.action_result = terminal.result()
            result = self.action_result.result
            missed = [dict(index=int(item.index), error_code=int(item.error_code),
                           goal=dict(frame_id=item.goal.header.frame_id, stamp_ns=frozen.stamp_ns(item.goal.header.stamp),
                                     x=item.goal.pose.position.x, y=item.goal.pose.position.y, yaw=frozen.yaw(item.goal.pose.orientation)))
                      for item in result.missed_waypoints]
            fields = dict(status=int(self.action_result.status), error_code=int(result.error_code), error_msg=result.error_msg,
                          missed_waypoints=missed)
        else:
            fields = dict(status=None, error_code=None, error_msg='No terminal response observed', missed_waypoints=None)
        self.emit('action_result', action_uuid=str(requested_uuid), terminal_snapshot=snapshot, **fields)
        settled, final_snapshot = self.wait_settled(self.args.settle_seconds)
        settled = self.record_stop(settled, final_snapshot, 'normal_terminal_result')['observed']
        valid, issues, snapshot = self.execution_ready()
        if not valid:
            self.invalidate_artifact(issues)
        self.observe_ordered(valid, snapshot)
        ordered = self.ordered.summary()
        final_error = frozen.error(final_snapshot['last'].get('pose'), self.route[-1]['pose'])
        final_within = (final_error is not None and final_error['xy_m'] <= .15 and final_error['yaw_rad'] <= .08
                        and final_error['pose'].get('receive_age_s', math.inf) <= .3
                        and -.05 <= final_error['pose'].get('header_age_s', math.inf) <= .3)
        passed = (fields['status'] == 4 and fields['error_code'] == 0 and fields['missed_waypoints'] == []
                  and ordered['all_visited'] and ordered['violation'] is None and settled and final_within)
        plans = {}
        for index, item in self.first_plans.items():
            entered = self.ordered.visits[index]['entered']
            latency = None if entered is None else (item['received_steady_ns']-entered['steady_ns'])/1e9
            plans[str(index)] = dict(item, feedback_to_plan_receipt_s=latency,
                measurement_boundary='Feedback callback to geometrically matching /plan callback; may be negative if plan precedes feedback. Child dispatch and planner computation duration are unavailable.')
        return dict(classification='invalid_runtime_evidence' if self.artifact_invalid_reasons else
                    'completed_native_batch_and_all_original_visits_pass' if passed else
                    'native_action_success_but_independent_acceptance_failed' if fields['status'] == 4 else
                    'canceled_by_pilot' if self.cancel_reason else 'completed_action_failure_or_missing_result',
                    action_uuid=str(requested_uuid), cancel_reason=self.cancel_reason, ordered=ordered, first_plans=plans,
                    settled_measured_and_commanded_stop=settled, settled_original_goal_error=final_error,
                    settled_within_original_tolerance=final_within, final_snapshot=final_snapshot,
                    artifact_invalid_reasons=self.artifact_invalid_reasons, native_feedback_queue_overflow=self.feedback_overflow,
                    runtime_binding=self.latest_ready, **fields)

    def finish_evidence(self):
        writer = self.writer.close()
        evidence = dict(schema='matrix_native_follow_waypoints_scan_timing_v1', scan_ring_limit=50,
            scan_capture_errors=self.scan_timing_capture_errors, scan_last_capture_error=self.scan_timing_last_capture_error,
            first_runtime_invalid=self.scan_timing_firstbad, latest_readiness_check=self.scan_timing_latest_check,
            last_good_readiness_check=self.scan_timing_last_good_check, scan_ring=list(self.scan_timing_ring), writer=writer)
        (self.output/'scan_timing_evidence.json').write_text(evidence_json(evidence, indent=2)+'\n')
        return writer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--ready', type=Path)
    parser.add_argument('--route', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case-id', default='native_waypoint_batch')
    parser.add_argument('--variant', choices=['official', 'segmented', 'hybrid'], default='official')
    parser.add_argument('--kind', choices=['benchmark', 'setup', 'fault'], default='benchmark')
    parser.add_argument('--observe-seconds', type=float, default=8.)
    parser.add_argument('--preflight-seconds', type=float, default=40.)
    parser.add_argument('--stable-preflight-seconds', type=float, default=10.)
    parser.add_argument('--timeout', type=float, help='Must equal route mission_budget_sec; defaults to its preregistered value')
    parser.add_argument('--settle-seconds', type=float, default=8.)
    parser.add_argument('--cancel-after', type=float)
    parser.add_argument('--odom-topic', default='/odom/current_pose')
    parser.add_argument('--scan-topic', default='/laser_scan')
    parser.add_argument('--scan-aging-confirm-sec', type=float, default=0.,
                        help='Opt-in existing-callback confirmation in [0,.05]s; every episode fails strict timing')
    parser.add_argument('--camera-topic', default='/camera/front/image/compressed')
    parser.add_argument('--camera-info-topic', default='/camera/front/camera_info')
    args = parser.parse_args()
    args.stop_signals = []
    args.goal = args.relative_goal = None
    try:
        route_raw = args.route.read_bytes()
        document = json.loads(route_raw)
        route = validate_route(document)
        if args.timeout is not None and args.timeout != document['mission_budget_sec']:
            raise ValueError('--timeout must equal the preregistered route mission_budget_sec')
        args.timeout = float(document['mission_budget_sec'])
        frozen.validate_numeric_arguments(args)
        validate_settle_budget(args.settle_seconds)
        validate_scan_aging_confirmation(args.scan_aging_confirm_sec)
        if args.preflight_seconds > 40. or args.stable_preflight_seconds < 10. or args.timeout <= 0 or args.timeout > 3600.:
            raise ValueError('Batch requires >=10s continuous preflight within <=40s, and timeout in (0,3600]')
        if args.execute and args.ready is None:
            raise ValueError('--execute requires --ready')
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    args.output = args.output.resolve()
    if HERE not in args.output.parents or args.output.exists():
        parser.error('Use a new output directory beneath this deployed pilot source directory')
    args.output.mkdir(parents=True)
    attempt_uuid, created_ns = str(uuid.uuid4()), time.time_ns()
    (args.output/'route_original.json').write_bytes(route_raw)
    instrumentation = freeze_sources(args.output)
    invocation = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
    (args.output/'invocation.json').write_text(json.dumps(dict(attempt_uuid=attempt_uuid, created_ns=created_ns,
        args=invocation, route_sha256=hashlib.sha256(route_raw).hexdigest(), instrumentation=instrumentation), indent=2)+'\n')
    node, exit_code, result = None, 0, {}
    previous_handlers = {number: signal.signal(number, lambda signum, frame: args.stop_signals.append(signum))
                         for number in [signal.SIGINT, signal.SIGTERM]}
    try:
        if os.environ.get('ROS_DOMAIN_ID') != '185':
            raise ValueError('Only isolated Matrix domain185 is supported')
        # Keep the context alive for exact-own cancellation and stop observation.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = WaypointPilot(args, args.output, route, instrumentation)
        if args.execute:
            result = node.run_batch()
        else:
            node.spin_until(lambda: False, args.observe_seconds)
            valid, issues, snapshot = node.execution_ready(require_stopped=True)
            result = dict(mode='observe_only', goals_sent=0, ready_for_new_batch=valid, issues=issues, snapshot=snapshot)
    except BaseException as exc:
        exit_code = 2
        result = dict(classification='pilot_exception_or_invalid_invocation', exception_type=type(exc).__name__, exception_message=str(exc))
        if node is not None:
            node.emit('pilot_exception', type=type(exc).__name__, message=str(exc))
    finally:
        if node is not None:
            result['owned_goal_shutdown'] = node.finalize_own_goal()
            if hasattr(node, 'stable_preflight_evidence'):
                result['stable_preflight'] = node.stable_preflight_evidence
            result['ordered'] = node.ordered.summary()
            result['artifact_invalid_reasons'] = node.artifact_invalid_reasons
            result['scan_aging_confirmation'] = dict(configured_sec=args.scan_aging_confirm_sec,
                episodes=node.scan_aging_episodes, strict_timing_status='FAIL' if node.scan_aging_episodes
                else 'NO_CONFIRMATION_VIOLATION_RECORDED',
                strict_timing_accepted=False if node.scan_aging_episodes else None)
            try:
                result['evidence_writer'] = node.finish_evidence()
            except BaseException as exc:
                result['evidence_writer'] = dict(evidence_complete=False, persistence_error_type=type(exc).__name__, persistence_error_message=str(exc))
            if not result['evidence_writer']['evidence_complete'] or node.scan_timing_capture_errors:
                result['classification_before_evidence_gap'] = result.get('classification')
                result['classification'] = 'invalid_callback_evidence_gap'
            if node.owned_identity is not None:
                result['final_measured_and_commanded_stop'] = bool(node.stop_evidence and node.stop_evidence.get('observed'))
        result.update(attempt_uuid=attempt_uuid, case_id=args.case_id, variant=args.variant, kind=args.kind,
            sources_sha256=instrumentation['sources_sha256'], route_sha256=hashlib.sha256(route_raw).hexdigest(),
            goals_sent=int(node is not None and node.goal_sent_ns is not None), stop_signals=args.stop_signals,
            created_ns=created_ns, finished_ns=time.time_ns())
        (args.output/'result.json').write_text(evidence_json(result, indent=2)+'\n')
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for number, handler in previous_handlers.items():
            signal.signal(number, handler)
    print(json.dumps(dict(result_file=str(args.output/'result.json'), classification=result.get('classification'), status=result.get('status'))))
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
