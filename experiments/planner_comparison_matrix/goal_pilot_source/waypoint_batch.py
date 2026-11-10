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
import struct
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
from waypoint_contract import (ACTION, AdvancingStop, AsyncEvidenceWriter, OrderedVisits, OwnedGoalIdentity, initial_leg_limit,
                               evidence_json, finite_velocity, mission_budget, strict_stopped, validate_ownership, validate_route, waypoint_binding)
from waypoint_contract import (ReadyReadCapture, ownership_metadata_lease_deviation, validate_settle_budget,
                               producer_scan_confirmation_failure, producer_scan_confirmation_outcome)
from waypoint_contract import (simulation_scan_latency_profile, validate_simulation_scan_latency_profile,
    operational_scan_age_issues, strict_simulation_scan_deviations, producer_scan_signed_age_bounds, record_scan_timing_health)
from waypoint_contract import (scan_aging_confirmation_eligible, scan_aging_confirmation_outcome,
                               scan_confirmation_payload_valid, validate_scan_aging_confirmation)


HERE = Path(__file__).resolve().parent
FROZEN_PILOT_SHA256 = 'a63bce9490f55438e7af5d16025e4dd20d86293cec12f1560d403ecb629cbe31'


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def freeze_sources(output, simulation_scan_max_age_sec=.3):
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
        input_limits='Original .3s old/.05s future and .5s READY lease; original JPEG/Info preflight and QoS retained. Running camera quality is recorded independently and does not cancel navigation.',
        acceptance='One native batch, all original map points independently in order with .3s fresh measured+commanded stop. Native feedback and Action success alone are insufficient.',
        cancellation='Only this client FollowWaypoints UUID; the upstream Jazzy server internally cancels all goals on its NavigateToPose endpoint. Exclusive experiment ownership is required.',
        timing_boundary='Native source pose sample stamps are retained. /plan receipt latency is transport/BT/observer latency, not planner computation time. Child NavigateToPose UUID is unavailable to this client.')
    manifest['nonfinite_evidence_encoding']='Writer/final serialization only: {nonfinite_float: NaN/+Infinity/-Infinity}; original caches, first-invalid references and decision values remain unchanged.'
    manifest['ownership_metadata_lease']='NavigateToPose ownership metadata uses the same 500ms old/50ms future lease as READY. Each checked_unix_ns exceeding the historical 300ms metadata boundary is recorded once with original validation time and READY; this is separate metadata latency evidence and never changes physical-input classification.'
    manifest['raw_plan_observer']='Only inherited /plan subscription uses raw=True with identical Path type/QoS. Plain CDR1 endian-aware observer scans headers/count/last pose only; full original Path remains recorded by MCAP. Decode failures are explicit observer gaps, never fresh input replacements.'
    manifest['runtime_callback_pump']='Runtime batches callbacks within 20ms before unchanged guards; interruption/terminal/own-cancel/existing deadlines checked per callback. Single-callback overruns, original topic-count deltas and check intervals are recorded; no 300ms/500ms/QoS changes. Fixed .002s spin kwargs preserve Jazzy ready iterator fairness; one idle wait may exceed a wall window, and single callbacks remain nonpreemptible.'
    manifest['ready_read_failure_capture']='Unchanged a63b validator receives one path-like single-file read with fd stat and wall/steady bounds. Only the first read_ready exception retains exact raw bytes in memory, persisted after own-goal cleanup; internal validator now remains unknown. Last-passing READY cache is labelled separately.'
    manifest['stop_cleanup']='Every possibly sent goal reaches finally own-UUID cancellation and a fresh finite measured/commanded stop observation, including response timeout, signal and exception. Default rclpy shutdown-on-SIGINT is disabled until cleanup completes.'
    manifest['stop_callback_pump']='Stop/finally/reconcile drain the same executor for <=20ms with fixed .002s spin calls, independent of terminal/cancel state; one full READY/ownership health check per batch. After full validation, original caches and that same READY are re-aged with current clocks before unchanged strict stopped/advancing-Odom .3s dwell. No second READY/state read, callback pump, timestamp refresh or extra executor; original settle deadline retained.'
    manifest['producer_scan_confirmation']='Opt-in same-scan producer-only not_ready may use the existing <=50ms window only after current raw metadata/bindings/UUID revalidation. New strict READY with a later original producer scan stamp and all own original guards is required; every episode permanently fails strict timing. Preflight/runtime failed reads are separate.'
    manifest['scan_aging_confirmation']='Default 0 preserves strict behavior. Opt-in <=.05s drains existing callbacks only after sole scan-header aging; a newer original stamp must restore every original guard. Every episode permanently fails strict timing acceptance; no header-age threshold changes.'
    manifest['simulation_scan_latency_profile']=simulation_scan_latency_profile(simulation_scan_max_age_sec)
    (output/'instrumentation_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def path_cdr_endpoint(raw):
    """Read only Path header/count/last pose from plain CDR1 (both endians).

    Alignment is relative to the four-byte encapsulation header. PoseStamped
    frame strings may have different byte lengths; every skipped body is bounded.
    This is observation only; the recorder retains the complete original Path.
    """
    data=memoryview(raw)
    if len(data)<4:raise ValueError('Truncated Path CDR encapsulation')
    representation=bytes(data[:2])
    if representation not in (b'\x00\x00',b'\x00\x01'):
        raise ValueError('Unsupported Path CDR encapsulation')
    endian='<' if representation==b'\x00\x01' else '>'
    offset=4
    def align(size):
        nonlocal offset
        offset+=(-(offset-4))%size
        if offset>len(data):raise ValueError('Truncated Path CDR alignment')
    def header(observe=False):
        nonlocal offset
        align(4)
        if offset+12>len(data):raise ValueError('Truncated Path CDR header')
        sec,nanosec,size=struct.unpack_from(endian+'iII',data,offset);offset+=12
        if size<1 or offset+size>len(data) or data[offset+size-1]!=0:
            raise ValueError('Invalid Path CDR frame string')
        frame=bytes(data[offset:offset+size-1]).decode('utf-8') if observe else None
        offset+=size
        return (frame,sec*1_000_000_000+nanosec) if observe else None
    frame,stamp=header(True);align(4)
    if offset+4>len(data):raise ValueError('Truncated Path CDR sequence count')
    count=struct.unpack_from(endian+'I',data,offset)[0];offset+=4
    if count>(len(data)-offset)//69:raise ValueError('Path CDR pose count exceeds available bytes')
    endpoint=None
    for index in range(count):
        header();align(8)
        if offset+56>len(data):raise ValueError('Truncated Path CDR pose')
        if index+1==count:
            x,y,z,qx,qy,qz,qw=struct.unpack_from(endian+'7d',data,offset)
            endpoint=dict(x=x,y=y,yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz)))
        offset+=56
    return dict(frame_id=frame,stamp_ns=stamp,poses=count,endpoint=endpoint)


class WaypointPilot(frozen.Pilot):
    def __init__(self, args, output, route, source_manifest, mission=None):
        self.mission = mission
        self.simulation_scan_max_age_sec=getattr(args,'simulation_scan_max_age_sec',.3)
        self.scan_latency_profile=simulation_scan_latency_profile(self.simulation_scan_max_age_sec)
        self.strict_scan_latency_first_fault=None;self.strict_scan_latency_count=0;self.strict_scan_latency_seen=set()
        self.simulation_scan_last_stamp_ns=None;self.simulation_scan_sequence_fault=None
        self.route = route
        self.loaded_sources = {'experiments/planner_comparison_matrix/goal_pilot_source/'+name: digest
                               for name,digest in source_manifest['sources_sha256'].items()}
        self.ordered = (mission.ordered(route) if mission is not None else OrderedVisits(route))
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
        self.first_ready_read_failure = None
        self.first_runtime_ready_read_failure=None;self.current_ready_read_failure=None
        self.producer_scan_failure_reads=[]
        self.plan_raw_parse_errors=0;self.plan_raw_last_error=None
        self.runtime_last_pump=None;self.runtime_last_ready_check_ns=None
        super().__init__(args, output)
        # Base opens this handle but all batch events use the asynchronous writer.
        self.events.close()
        self.writer = AsyncEvidenceWriter(output, publish_event=self.publish_serialized_event)
        self.recovery_handoffs=[]
        if mission is not None:
            self.subscriptions_kept.append(self.create_subscription(String,'/experiment/recovery/events',self.recovery_event,100))

    def recovery_event(self,message):
        try:
            value=json.loads(message.data)
            run_id=self.latest_ready.get('recovery_runtime',{}).get('run_id')
            if value.get('schema')=='matrix.recovery.handoff.v1' and value.get('run_id')==run_id:
                self.recovery_handoffs.append(value)
                self.emit('recovery_handoff',handoff=value)
        except (ValueError,AttributeError):
            self.emit('invalid_recovery_handoff',raw=message.data)

    def sub(self, cls, topic, callback):
        if cls is frozen.NavPath and topic=='/plan':
            self.subscriptions_kept.append(self.create_subscription(
                cls,topic,callback,frozen.qos_profile_sensor_data,raw=True))
        else:
            frozen.Pilot.sub(self,cls,topic,callback)

    def remember(self, key, value):
        remembered=super().remember(key,value)
        dwell=getattr(self,'_stop_dwell',None)
        if dwell is not None and key in ('odom','/cmd_vel_safe','loc_status','pose_valid'):
            # A short delivered moving sample between health checks must break
            # the stop history even when the last value is zero again. This is
            # invalidation only: no callback can grant stop or refresh an age.
            bad=(key=='loc_status' and remembered.get('value')!=1) or (
                key=='pose_valid' and remembered.get('value') is not True)
            if key in ('odom','/cmd_vel_safe'):
                linear,angular=(.03,.05) if key=='odom' else (.01,.01)
                bad=not finite_velocity(remembered) or math.hypot(
                    remembered.get('vx',math.inf),remembered.get('vy',math.inf))>linear or abs(
                    remembered.get('wz',math.inf))>angular
                source=remembered.get('stamp_ns') if key=='odom' else remembered['received_unix_ns']
                previous=self._stop_sample_previous.get(key)
                if key=='odom':
                    bad=bad or remembered.get('frame_id')!='odom' or remembered.get('child_frame_id')!='base_link'
                if type(source) is not int or source<=0:
                    bad=True
                elif previous is not None and not 0<source-previous<=300_000_000:
                    bad=True
                self._stop_sample_previous[key]=source
            if bad:
                dwell.reset();self._stop_callback_resets+=1
        return remembered

    def publish_serialized_event(self, line):
        if self.event_pub is not None:
            message = String()
            message.data = line
            self.event_pub.publish(message)

    def emit(self, kind, **fields):
        if getattr(self,'simulation_scan_max_age_sec',.3)>.3:
            fields.setdefault('simulation_scan_latency_profile',self.scan_latency_profile)
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
        if self.args.scan_aging_confirm_sec > 0. or self.simulation_scan_max_age_sec>.3:
            value['confirmation_payload_valid'] = scan_confirmation_payload_valid(msg)
        if self.simulation_scan_max_age_sec>.3:
            stamp=value['stamp_ns'];prior=self.simulation_scan_last_stamp_ns
            if prior is not None and stamp<=prior:
                self.simulation_scan_sequence_fault=dict(previous_original_header_ns=prior,original_header_ns=stamp,callback_unix_ns=enter_unix_ns)
            self.simulation_scan_last_stamp_ns=stamp if prior is None else max(prior,stamp)
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
        # Run the unchanged camera checks and retain their original snapshots.
        # After dispatch they describe recording quality, not navigation input health.
        if getattr(self, 'goal_sent_ns', None) is not None:
            camera_issues=[issue for issue in issues if issue in (
                'camera_missing_stale_or_decode_failed','camera_info_missing_or_invalid','camera_shape_mismatch')]
            report=getattr(self,'camera_recording_quality',None)
            if report is None:
                report=dict(schema='matrix.camera.recording_quality.v1',status='NO_CAMERA_FAULT_RECORDED',
                    strict_timing_status='NO_CAMERA_FAULT_RECORDED',fault_checks=0,first_fault=None)
                self.camera_recording_quality=report
            if camera_issues:
                report.update(status='FAIL',strict_timing_status='FAIL',fault_checks=report['fault_checks']+1)
                if report['first_fault'] is None:
                    report['first_fault']=dict(checked_unix_ns=entered_unix_ns,issues=list(camera_issues),
                        camera=dict(snapshot['last'].get('camera',{})),camera_info=dict(snapshot['last'].get('camera_info',{})))
                    self.emit('camera_recording_quality_fault',**report['first_fault'],
                        recording_quality_status='FAIL',navigation_cancelled_by_camera=False)
            issues=[issue for issue in issues if issue not in camera_issues]
        if snapshot['last'].get('pose', {}).get('raw_pose_values_finite') is not True:
            issues.append('pose:raw_position_or_quaternion_nonfinite_or_missing')
        if not finite_velocity(snapshot['last'].get('/cmd_vel_safe', {})):
            issues.append('/cmd_vel_safe:nonfinite_or_missing_velocity')
        if ((self.args.scan_aging_confirm_sec > 0. or self.simulation_scan_max_age_sec>.3)
                and snapshot['last'].get('scan', {}).get('confirmation_payload_valid') is not True):
            issues.append('scan:confirmation_payload_nonfinite_or_invalid_or_missing')
        if require_stopped:
            stopped, stop_issues = strict_stopped(snapshot)
            if not stopped:
                issues.extend(stop_issues)
        if self.simulation_scan_sequence_fault is not None:
            issues.append('scan:source_stamp_not_strictly_increasing')
        self.current_ready_read_failure=None
        validated_ready=None
        if self.args.execute:
            readiness_stage, ownership_checked_now_ns = 'read_ready', None
            ready_capture = ReadyReadCapture(self.args.ready)
            ready_call_bounds = dict(call_begin_unix_ns=time.time_ns(), call_begin_steady_ns=time.monotonic_ns())
            try:
                self.latest_ready, binding, self.latest_ready_sha = frozen.read_ready(ready_capture, self.args.variant, self.runtime_binding)
                readiness_stage='simulation_scan_latency_profile'
                validate_simulation_scan_latency_profile(self.latest_ready,self.simulation_scan_max_age_sec)
                if self.simulation_scan_max_age_sec>.3:
                    bounds=producer_scan_signed_age_bounds(self.latest_ready)
                    if (min(bounds['header_age_bounds_s'])<-.05 or min(bounds['receive_age_bounds_s'])<0
                            or min(bounds['header_age_bounds_s'])>.5 or min(bounds['receive_age_bounds_s'])>.5):
                        raise ValueError('scan:producer_observation_outside_simulation_bound')

                readiness_stage = 'waypoint_binding'
                native = waypoint_binding(self.latest_ready, self.native_binding)
                if getattr(self, 'mission', None) is not None:
                    self.mission.validate_ready(self.latest_ready)
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
                validated_ready=self.latest_ready
            except Exception as exc:
                call_end_unix_ns, call_end_steady_ns = time.time_ns(), time.monotonic_ns()
                captured_first_failure = False
                failure_file=None
                if readiness_stage == 'read_ready':
                    ready_call_bounds.update(call_end_unix_ns=call_end_unix_ns, call_end_steady_ns=call_end_steady_ns)
                    self.current_ready_read_failure=ready_capture.failure(exc,ready_call_bounds)
                    runtime=self.goal_sent_ns is not None and self.owned_identity is not None
                    slot='first_runtime_ready_read_failure' if runtime else 'first_ready_read_failure'
                    failure_file='runtime_ready_read_failure' if runtime else 'ready_read_failure'
                    self.current_ready_read_failure['metadata']['raw_file']=None
                    if getattr(self,slot,None) is None:
                        self.current_ready_read_failure['metadata']['raw_file']=failure_file+'.raw'
                        setattr(self,slot,self.current_ready_read_failure);captured_first_failure=True
                issues.append('readiness:'+str(exc))
                self.emit('readiness_validation_failed', stage=readiness_stage, message=str(exc),
                    ownership_validation_unix_ns=ownership_checked_now_ns,
                    first_failed_read_capture=captured_first_failure,
                    first_failed_read_metadata_file=failure_file+'.json' if failure_file is not None else None,
                    current_failed_read_sha256=None if self.current_ready_read_failure is None else self.current_ready_read_failure['metadata']['sha256'],
                    latest_ready_sha256=getattr(self, 'latest_ready_sha', None),
                    latest_ready_is_failing_read=readiness_stage != 'read_ready',
                    latest_ready=getattr(self, 'latest_ready', None))
            if self.event_pub.get_subscription_count() == 0:
                issues.append('goal_event_subscriber_disappeared')
        producer_evidence_ready=validated_ready
        producer_evidence_context='current_fully_validated_READY' if validated_ready is not None else None
        if self.simulation_scan_max_age_sec>.3:
            if producer_evidence_ready is None and self.current_ready_read_failure is not None:
                try:
                    candidate=json.loads(self.current_ready_read_failure['raw_bytes'])
                    validate_simulation_scan_latency_profile(candidate,self.simulation_scan_max_age_sec)
                    producer_scan_signed_age_bounds(candidate)
                    producer_evidence_ready=candidate
                    producer_evidence_context='failed_original_READY_raw_strict_evidence_only_never_health'
                except (ValueError,TypeError,KeyError):pass
        return record_scan_timing_health(self,require_stopped,issues,snapshot,entered_unix_ns,entered_steady_ns,
            producer_evidence_ready,producer_evidence_context)

    def producer_scan_confirmation_failure(self,issues,snapshot):
        topic=getattr(self.args,'scan_topic','/laser_scan_leveled')
        allowed={'readiness:launcher_not_ready:Stale/future stamp '+topic,
                 'readiness:launcher_not_ready:Stale receive '+topic}
        failure=getattr(self,'current_ready_read_failure',None)
        if len(issues)!=1 or issues[0] not in allowed or failure is None or failure['raw_bytes'] is None:return None
        try:
            verified=producer_scan_confirmation_failure(failure['raw_bytes'],self.args.variant,
                self.runtime_binding,self.native_binding,self.loaded_sources,
                None if self.owned_identity is None else self.owned_identity.value.hex(),self.goal_sent_ns,
                time.time_ns(),topic)
            return dict(verified,failed_read=failure)
        except Exception as exc:
            self.emit('producer_scan_confirmation_rejected',message=str(exc),
                current_failed_read_sha256=failure['metadata']['sha256'])
            return None

    def confirm_scan_aging(self, issues, snapshot, terminal):
        """Drain callbacks for <=50ms; any recovered run remains strict-timing FAIL."""
        seconds = self.args.scan_aging_confirm_sec
        if seconds == 0. or terminal.done() or self.cancel_requested:return False,issues,snapshot
        producer=None
        if not scan_aging_confirmation_eligible(issues,snapshot):
            producer=getattr(self,'producer_scan_confirmation_failure',lambda *args:None)(issues,snapshot)
            if producer is None:return False,issues,snapshot
        started_ns = self.scan_timing_latest_check['check_enter_steady_ns']
        budget_ns = int(seconds*1e9)
        original_stamp = producer['original_stamp_ns'] if producer is not None else snapshot['last']['scan']['stamp_ns']
        episode = dict(index=len(self.scan_aging_episodes), original_stamp_ns=original_stamp,
            started_steady_ns=started_ns, deadline_steady_ns=started_ns+budget_ns,
            confirmation_budget_sec=seconds, original_issues=list(issues), original_snapshot=snapshot,
            strict_timing_status='FAIL', strict_timing_accepted=False, outcome='pending')
        if producer is not None:
            captures=getattr(self,'producer_scan_failure_reads',None)
            if captures is None:self.producer_scan_failure_reads=[];captures=self.producer_scan_failure_reads
            name='producer_scan_confirmation_failure_'+str(len(captures))
            failure=producer['failed_read'];saved=dict(raw_bytes=failure['raw_bytes'],metadata=dict(failure['metadata'],raw_file=name+'.raw'))
            captures.append((name,saved))
            episode['producer_ready_failed_read']=saved['metadata']
            episode['producer_scan_observation']=producer['producer_scan_observation']
            episode['basis']='producer_scan_observer_disagreement'
            self.emit('producer_consumer_health_disagreement',producer_ready=producer['ready'],
                failed_read_metadata=saved['metadata'],consumer_original_snapshot=snapshot,
                boundary='Own original guards passed; producer stopped at first scan error, later producer guards unknown. New strict READY is still required.')
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
            rclpy.spin_once(self, timeout_sec=.005)
            _, current_issues, current_snapshot = self.execution_ready()
            checked_ns = time.monotonic_ns()
            if producer is None:
                outcome = scan_aging_confirmation_outcome(current_issues, current_snapshot, original_stamp,
                                                           checked_ns-started_ns, budget_ns)
            else:
                pending=getattr(self,'producer_scan_confirmation_failure',lambda *args:None)(current_issues,current_snapshot) if current_issues else None
                outcome=producer_scan_confirmation_outcome(current_issues,current_snapshot,producer,
                    self.latest_ready,checked_ns-started_ns,budget_ns,pending is not None)
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
            checked_unix_ns=time.time_ns(), strict_stop_issues=issues, snapshot=snapshot,
            callback_observation=getattr(self,'stop_wait_observation',None))
        return self.stop_evidence

    def pump_stop_inputs(self, deadline_ns):
        """Drain original subscriptions after terminal/cancel as well as before.

        The active-goal pump deliberately stops at terminal/cancel, so it cannot
        supply fresh stop evidence. This uses its own finite wall window on the
        same executor and never refreshes a cached message's receipt/header.
        """
        started=time.monotonic_ns();cpu=time.thread_time_ns();before=dict(self.counts)
        end=min(deadline_ns,started+20_000_000);calls=0
        try:
            while rclpy.ok() and time.monotonic_ns()<end:
                self.check_interruption()
                rclpy.spin_once(self,timeout_sec=.002)
                calls+=1
        finally:
            finished=time.monotonic_ns()
            self.stop_last_pump=dict(started_steady_ns=started,finished_steady_ns=finished,
                budget_ns=max(0,end-started),wall_ns=finished-started,
                thread_cpu_ns=time.thread_time_ns()-cpu,spin_calls=calls,
                overrun_ns=max(0,finished-end),
                observed_count_deltas={key:self.counts.get(key,0)-before.get(key,0)
                    for key in set(before)|set(self.counts) if self.counts.get(key,0)!=before.get(key,0)})
        return self.stop_last_pump

    def stop_decision_ready(self, prior_issues=()):
        """Re-age original caches and the already-read lease after full validation.

        This runs on the same executor thread without spinning, file I/O or a
        mission-state read. Camera health retains its preflight-only policy.
        """
        entered_unix_ns,entered_steady_ns=time.time_ns(),time.monotonic_ns()
        _,issues,snapshot=self.input_ready(require_camera=self.goal_sent_ns is None)
        issues=list(dict.fromkeys(list(prior_issues)+issues));bounds=None;ready=None
        if self.args.execute:
            ready=getattr(self,'latest_ready',{})
            now=time.time_ns();issued=ready.get('time_ns',0);expiry=ready.get('valid_until_ns',0)
            # Identical absolute lease bounds to the unchanged frozen read_ready.
            if (not isinstance(issued,int) or not isinstance(expiry,int) or
                    not 0<expiry-now<=550_000_000 or not -50_000_000<=now-issued<=500_000_000):
                issues.append('readiness:launcher_lease_missing_expired_or_future')
            try:
                validate_ownership(ready.get('waypoint_ownership',{}),now,
                    None if self.owned_identity is None else self.owned_identity.value.hex(),self.goal_sent_ns)
                if self.simulation_scan_max_age_sec>.3:
                    bounds=producer_scan_signed_age_bounds(ready,
                        clock_ns=self.get_clock().now().nanoseconds,steady_ns=time.monotonic_ns())
                    if (min(bounds['header_age_bounds_s'])<-.05 or min(bounds['receive_age_bounds_s'])<0
                            or max(bounds['header_age_bounds_s'])>.5 or max(bounds['receive_age_bounds_s'])>.5):
                        raise ValueError('scan:producer_observation_outside_simulation_bound')
            except ValueError as error:issues.append('readiness:'+str(error))
        return record_scan_timing_health(self,False,issues,snapshot,entered_unix_ns,entered_steady_ns,
            ready if bounds is not None else None,
            'same_cached_READY_reaged_at_stop_decision' if bounds is not None else None,producer_bounds=bounds)

    def wait_settled(self, seconds):
        """One full health read per batch, then unchanged advancing-Odom stop."""
        started=time.monotonic_ns();deadline=started+int(seconds*1e9)
        dwell=AdvancingStop();recent=deque(maxlen=32);checks=0;settled=False
        self._stop_dwell=dwell;self._stop_sample_previous={};self._stop_callback_resets=0
        snapshot=None
        try:
            while rclpy.ok() and time.monotonic_ns()<deadline:
                pump=self.pump_stop_inputs(deadline)
                if time.monotonic_ns()>=deadline:break
                checked=time.monotonic_ns();cpu=time.thread_time_ns()
                # One full READY/state validation, then a cheap current-clock
                # check of the same original caches/lease before stop accrual.
                healthy,issues,snapshot=self.execution_ready()
                full_health_finished=time.monotonic_ns()
                full_odom_age=snapshot.get('last',{}).get('odom',{}).get('header_age_s')
                current,age_issues,snapshot=self.stop_decision_ready(issues)
                healthy=healthy and current;issues=list(dict.fromkeys(issues+age_issues))
                if not healthy:self.invalidate_artifact(issues)
                finished=time.monotonic_ns();checks+=1
                qualified=dwell.observe(snapshot,finished,healthy)
                recent.append(dict(pump=pump,health_started_steady_ns=checked,
                    health_finished_steady_ns=finished,health_wall_ns=finished-checked,
                    full_health_finished_steady_ns=full_health_finished,
                    full_health_odom_header_age_s=full_odom_age,
                    decision_odom_header_age_s=snapshot.get('last',{}).get('odom',{}).get('header_age_s'),
                    health_thread_cpu_ns=time.thread_time_ns()-cpu,
                    healthy=healthy,issues=list(issues),stop_qualified=qualified,
                    odom_stamp_ns=snapshot.get('last',{}).get('odom',{}).get('stamp_ns'),
                    odom_received_unix_ns=snapshot.get('last',{}).get('odom',{}).get('received_unix_ns')))
                if qualified and finished<=deadline:
                    settled=True
                    return True,snapshot
            return False,self.current_summary()
        finally:
            self.stop_wait_observation=dict(schema='matrix.stop_callback_observation.v1',
                started_steady_ns=started,deadline_steady_ns=deadline,
                finished_steady_ns=time.monotonic_ns(),checks=checks,settled=settled,
                recent=list(recent),ring_limit=32,advancing_odom_dwell=dict(dwell.__dict__),
                callback_invalidations=self._stop_callback_resets)
            self._stop_dwell=None
            self.emit('stop_wait_finished',**self.stop_wait_observation)

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
        if getattr(self, 'mission', None) is not None and (self.owned_identity is None or
                bytes(message.goal_id.uuid) != self.owned_identity.value):
            self.emit('old_action_feedback_ignored')
            return
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
        if not self.args.execute or self.goal_sent_ns is None:return
        entered=time.monotonic_ns();cpu=time.thread_time_ns()
        try:observed=path_cdr_endpoint(message)
        except Exception as exc:
            self.plan_raw_parse_errors+=1
            self.plan_raw_last_error=dict(type=type(exc).__name__,message=str(exc),cdr_bytes=len(message),
                cdr_sha256=hashlib.sha256(message).hexdigest(),callback_wall_ns=time.monotonic_ns()-entered,
                callback_thread_cpu_ns=time.thread_time_ns()-cpu)
            self.emit('plan_observation_parse_error',**self.plan_raw_last_error)
            return
        if not observed['poses']:return
        endpoint=observed['endpoint']
        matches = [entry['index'] for entry in self.route if observed['frame_id'] == 'map'
                   and math.hypot(endpoint['x']-entry['pose'][0], endpoint['y']-entry['pose'][1]) <= .02
                   and abs(math.atan2(math.sin(endpoint['yaw']-entry['pose'][2]), math.cos(endpoint['yaw']-entry['pose'][2]))) <= .10]
        received = time.monotonic_ns()
        item = dict(frame_id=observed['frame_id'], stamp_ns=observed['stamp_ns'], poses=observed['poses'],
                    raw_cdr_bytes=len(message),observer_parse_wall_ns=received-entered,
                    observer_parse_thread_cpu_ns=time.thread_time_ns()-cpu,
                    endpoint=endpoint, matching_original_indices=matches, parent_action_uuid=self.owned_identity.value.hex(),
                    child_action_uuid=None, native_feedback_index=self.ordered.active, received_steady_ns=received,
                    association='Endpoint/time inference only; /plan contains no Action UUID or route index.')
        self.emit('plan_observation', **item)
        if len(matches) == 1 and matches[0] not in self.first_plans:
            self.first_plans[matches[0]] = item

    def observe_ordered(self, healthy, snapshot, *, defer_storage=False):
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
        if getattr(self, 'mission', None) is not None:
            self.mission.observe(self.ordered,defer=defer_storage)
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

    def pump_runtime_inputs(self, terminal):
        """Bound callback work before the unchanged original guards, not their ages.

        spin_once does not report whether a callback ran. spin_calls and original
        per-topic counter deltas are distinct evidence, not invented callback counts.
        A single callback can exceed the window; that overrun is recorded explicitly.
        """
        started=time.monotonic_ns();cpu=time.thread_time_ns();before=dict(self.counts)
        calls=0;reason='window_elapsed';budget_ns=20_000_000
        try:
            while rclpy.ok():
                self.check_interruption()
                if terminal.done():reason='terminal';break
                if self.cancel_requested and calls:reason='cancel_pending';break
                now=time.monotonic_ns();elapsed=time.monotonic()-self.goal_sent_steady
                if not self.cancel_requested and (elapsed>=self.args.timeout or
                    (self.args.cancel_after is not None and elapsed>=self.args.cancel_after)):
                    reason='existing_deadline';break
                remaining_ns=budget_ns-(now-started)
                if remaining_ns<=0:break
                # Fixed kwargs retain Jazzy's cached ready iterator and its
                # subscription order. The window/deadlines may exceed by one
                # 2ms idle wait; callback overruns remain explicitly measured.
                rclpy.spin_once(self,timeout_sec=.002);calls+=1
            else:reason='context_not_ok'
        except BaseException:
            reason='interruption_or_callback_exception';raise
        finally:
            finished=time.monotonic_ns()
            self.runtime_last_pump=dict(started_steady_ns=started,finished_steady_ns=finished,
                budget_ns=budget_ns,wall_ns=finished-started,thread_cpu_ns=time.thread_time_ns()-cpu,
                spin_calls=calls,stop_reason=reason,overrun_ns=max(0,finished-started-budget_ns),
                observed_count_deltas={key:self.counts.get(key,0)-before.get(key,0)
                    for key in set(before)|set(self.counts) if self.counts.get(key,0)!=before.get(key,0)})
        return self.runtime_last_pump

    def pump_storage_wait(self):
        """One executor; callback state stays on the main thread during fsync."""
        if not rclpy.ok():
            raise RuntimeError('ROS context stopped while durable checkpoint is pending')
        rclpy.spin_once(self,timeout_sec=.002)
        active=self.goal_sent_ns is not None
        if self.args.stop_signals:
            if active:self.cancel_own('pilot_interrupted')
            return
        now=time.monotonic()
        if now-getattr(self,'storage_last_check',0)<.02:return
        self.storage_last_check=now
        terminal=getattr(self,'storage_terminal',None)
        if terminal is not None and terminal.done():return
        try:
            valid,issues,snapshot=self.execution_ready(require_stopped=not active)
            if not active:
                if not valid:self.storage_preflight_gap=True
                return
            if not valid:self.invalidate_artifact(issues)
            self.observe_ordered(valid,snapshot,defer_storage=True)
            elapsed=now-self.goal_sent_steady
            reason='input_became_invalid:'+','.join(issues) if not valid else None
            if self.ordered.violation:reason=self.ordered.violation
            if self.feedback_overflow:reason='native_feedback_observation_queue_overflow'
            if self.args.cancel_after is not None and elapsed>=self.args.cancel_after:reason='scheduled_cancel'
            if elapsed>=self.args.timeout or self.mission.store.remaining()<=0:reason='pilot_navigation_deadline'
            if reason:self.cancel_own(reason)
        except BaseException:
            if active:self.cancel_own('storage_wait_observation_failed')
            raise

    def post_intent_preflight(self,deadline,started):
        # Keep the original preflight deadline and original stable duration.
        # Reuse this operation's durable UUID; never reserve again for a wait.
        deadline=min(deadline,time.monotonic()+self.mission.store.remaining())
        valid,issues,snapshot=self.wait_stable_preflight(deadline,started)
        if valid:valid,issues,snapshot=self.execution_ready(require_stopped=True)
        if not valid:return valid,issues,snapshot
        if time.monotonic()>=deadline or self.mission.store.remaining()<=0:
            return False,['post_intent_deadline'],snapshot
        limit=initial_leg_limit(self.args.simulation_first_leg_max_distance_m,
            self.latest_ready,self.args.simulation_scan_max_age_sec)
        pose=snapshot['last']['pose'];goal=self.route[self.mission.start_index]['pose']
        if math.hypot(goal[0]-pose['x'],goal[1]-pose['y'])>=limit:
            return False,['post_intent_first_leg_outside_reviewed_window'],snapshot
        self.args.timeout=min(self.args.timeout,self.mission.store.remaining())
        self.emit('post_intent_preflight_passed',action_uuid=str(self.mission.action_uuid),
            original_deadline_steady_s=deadline,original_stable_sec=self.args.stable_preflight_seconds,
            persistence_health_gap=bool(getattr(self,'storage_preflight_gap',False)),snapshot=snapshot)
        return True,[],snapshot

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
        try:
            first_limit = initial_leg_limit(self.args.simulation_first_leg_max_distance_m,
                self.latest_ready, self.args.simulation_scan_max_age_sec)
        except ValueError as exc:
            return dict(classification='invalid_initial_request_window_goal_not_sent',reason=str(exc))
        start_index = self.mission.start_index if getattr(self, 'mission', None) is not None else 0
        first_distance = math.hypot(self.route[start_index]['pose'][0]-start_pose['x'], self.route[start_index]['pose'][1]-start_pose['y'])
        self.emit('first_leg_window_check', distance_m=first_distance, limit_m=first_limit,
                  source_pose_stamp_ns=start_pose.get('stamp_ns'), final_goal_tolerances_unchanged=True)
        if first_distance >= first_limit:
            return dict(classification='invalid_first_leg_outside_reviewed_rolling_window_goal_not_sent')
        self.storage_preflight_gap=False
        requested_uuid = self.mission.reserve() if getattr(self, 'mission', None) is not None else uuid.uuid4()
        if getattr(self,'mission',None) is not None:
            valid,issues,snapshot=self.post_intent_preflight(deadline,started)
            if not valid:return dict(classification='invalid_post_intent_preflight_not_sent',issues=issues,snapshot=snapshot)
        request = FollowWaypoints.Goal()
        try:
            budget = mission_budget(self.latest_ready,self.args.timeout,time.clock_gettime(time.CLOCK_BOOTTIME),
                                    Path('/proc/sys/kernel/random/boot_id').read_text().strip())
        except ValueError as exc:
            return dict(classification='invalid_recording_deadline_budget_goal_not_sent',reason=str(exc))
        request.number_of_loops, request.goal_index = 0, start_index
        for entry in self.route:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            # Route intent is created now; native input sample stamps stay in snapshots.
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = entry['pose'][:2]
            pose.pose.orientation.z = math.sin(entry['pose'][2]/2.)
            pose.pose.orientation.w = math.cos(entry['pose'][2]/2.)
            request.poses.append(pose)
        self.owned_identity = OwnedGoalIdentity(requested_uuid.bytes)
        self.goal_uuid = UUID(uuid=list(requested_uuid.bytes))
        self.goal_sent_steady, self.goal_sent_ns = time.monotonic(), time.time_ns()
        self.emit('goal_send', action_uuid=str(requested_uuid), action_server=ACTION,
                  number_of_loops=0, goal_index=start_index, requested_route_map=self.route, start_snapshot=snapshot,
                  readiness_manifest=self.latest_ready, readiness_manifest_sha256=self.latest_ready_sha,
                  preregistered_budget=budget,
                  pose_header_stamps_ns=[frozen.stamp_ns(pose.header.stamp) for pose in request.poses])
        self.check_interruption()
        if getattr(self,'mission',None) is not None:self.mission.send_invoked=True
        sent = client.send_goal_async(request, goal_uuid=self.goal_uuid, feedback_callback=self.native_feedback)
        if not self.spin_until(sent.done, 10.):
            return dict(classification='goal_response_timeout_acceptance_unknown')
        self.action_goal_handle = sent.result()
        self.owned_identity.verify(self.action_goal_handle.goal_id.uuid)
        self.emit('goal_response', accepted=bool(self.action_goal_handle.accepted), action_uuid=str(requested_uuid))
        terminal=self.action_goal_handle.get_result_async() if self.action_goal_handle.accepted else None
        self.storage_terminal=terminal
        if getattr(self, 'mission', None) is not None:
            self.mission.acknowledge(bool(self.action_goal_handle.accepted))
        if not self.action_goal_handle.accepted:
            return dict(classification='goal_rejected')
        cancel_logged = False
        while rclpy.ok() and not terminal.done():
            pumped=self.pump_runtime_inputs(terminal)
            elapsed = time.monotonic()-self.goal_sent_steady
            check_started=time.monotonic_ns();check_cpu=time.thread_time_ns()
            valid, issues, snapshot = self.execution_ready()
            self.emit('runtime_input_pump',**pumped,
                execution_check_interval_ns=None if self.runtime_last_ready_check_ns is None else
                    check_started-self.runtime_last_ready_check_ns,
                execution_check_wall_ns=time.monotonic_ns()-check_started,
                execution_check_thread_cpu_ns=time.thread_time_ns()-check_cpu)
            self.runtime_last_ready_check_ns=check_started
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
        passed = (valid and not self.cancel_requested and self.cancel_reason is None
                  and fields['status'] == 4 and fields['error_code'] == 0 and fields['missed_waypoints'] == []
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
                    functional_status='PASS' if passed else 'FAIL',simulation_scan_latency_profile=self.scan_latency_profile,
                    camera_recording_quality=getattr(self,'camera_recording_quality',None),
                    strict_scan_timing_status='FAIL' if (self.strict_scan_latency_first_fault or self.scan_aging_episodes) else 'NO_STRICT_SCAN_FAULT_RECORDED',
                    action_uuid=str(requested_uuid), cancel_reason=self.cancel_reason, ordered=ordered, first_plans=plans,
                    settled_measured_and_commanded_stop=settled, settled_original_goal_error=final_error,
                    settled_within_original_tolerance=final_within, final_snapshot=final_snapshot,
                    artifact_invalid_reasons=self.artifact_invalid_reasons, native_feedback_queue_overflow=self.feedback_overflow,
                    runtime_binding=self.latest_ready, **fields)

    def finish_evidence(self):
        writer = self.writer.close()
        writer['camera_recording_quality']=getattr(self,'camera_recording_quality',None)
        if self.first_ready_read_failure is not None:
            failure = self.first_ready_read_failure
            if failure['raw_bytes'] is not None:
                with (self.output/'ready_read_failure.raw').open('xb') as stream:
                    stream.write(failure['raw_bytes'])
            with (self.output/'ready_read_failure.json').open('x') as stream:
                stream.write(evidence_json(failure['metadata'], indent=2)+'\n')
            writer['first_ready_read_failure'] = failure['metadata']
        saved_reads=list(getattr(self,'producer_scan_failure_reads',[]))
        runtime_failure=getattr(self,'first_runtime_ready_read_failure',None)
        if runtime_failure is not None:saved_reads.insert(0,('runtime_ready_read_failure',runtime_failure))
        for name,failure in saved_reads:
            if failure['raw_bytes'] is not None:
                with (self.output/(name+'.raw')).open('xb') as stream:stream.write(failure['raw_bytes'])
            with (self.output/(name+'.json')).open('x') as stream:stream.write(evidence_json(failure['metadata'],indent=2)+'\n')
        writer['first_runtime_ready_read_failure']=None if runtime_failure is None else runtime_failure['metadata']
        evidence = dict(schema='matrix_native_follow_waypoints_scan_timing_v1', scan_ring_limit=50,
            plan_raw_parse_errors=getattr(self,'plan_raw_parse_errors',0),plan_raw_last_error=getattr(self,'plan_raw_last_error',None),
            runtime_last_pump=getattr(self,'runtime_last_pump',None),
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
    parser.add_argument('--simulation-first-leg-max-distance-m',type=float,choices=[8.,8.5],default=8.,
                        help='Simulation-only initial request envelope; original route legs remain <8m')
    parser.add_argument('--simulation-scan-max-age-sec',type=float,choices=[.3,.5],default=.3,
                        help='Explicit simulation-only scan header/receive operational cap; strict original .3 evidence remains FAIL')
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
    instrumentation = freeze_sources(args.output,args.simulation_scan_max_age_sec)
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
            result['simulation_scan_latency_profile']=node.scan_latency_profile
            result['functional_status']=result.get('functional_status','FAIL_OR_NOT_EXECUTED')
            result['strict_scan_latency_evidence']=dict(first_fault=node.strict_scan_latency_first_fault,unique_source_stamp_deviations=node.strict_scan_latency_count,strict_timing_status='FAIL' if (node.strict_scan_latency_first_fault or node.scan_aging_episodes) else 'NO_STRICT_SCAN_FAULT_RECORDED')
            if node.strict_scan_latency_first_fault is not None or node.scan_aging_episodes:result['strict_scan_timing_status']='FAIL'
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
