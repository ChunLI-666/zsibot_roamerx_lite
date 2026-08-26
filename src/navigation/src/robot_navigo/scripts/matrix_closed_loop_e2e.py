#!/usr/bin/env python3

"""Run a Matrix + Lightning NavigateToPose closed-loop acceptance test."""

import argparse
import bisect
import json
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


LOC_NORMAL = 1
NORMAL_SAMPLES_REQUIRED = 10
POSE_PAIR_MAX_SKEW_SEC = 0.15
NONZERO_EPSILON = 1e-4


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


@dataclass
class PoseSample:
    receive_time: float
    x: float
    y: float
    yaw: float


class TopicStats:
    def __init__(self):
        self.reset()

    def reset(self):
        self.times = []
        self.source_times = []

    def add(self, receive_time, source_time=None):
        self.times.append(receive_time)
        if source_time is not None:
            self.source_times.append(source_time)

    @staticmethod
    def _timing_report(times):
        count = len(times)
        if count < 2:
            return {
                'count': count,
                'frequency_hz': None,
                'max_interval_sec': None,
            }
        intervals = [
            current - previous
            for previous, current in zip(times[:-1], times[1:])
            if current > previous
        ]
        duration = times[-1] - times[0]
        return {
            'count': count,
            'frequency_hz': (
                (count - 1) / duration if duration > 0.0 else None),
            'max_interval_sec': max(intervals) if intervals else None,
        }

    def report(self):
        receive = self._timing_report(self.times)
        source = self._timing_report(self.source_times)
        return {
            **receive,
            'timing_basis': 'receive_monotonic',
            'source_count': source['count'],
            'source_frequency_hz': source['frequency_hz'],
            'source_max_interval_sec': source['max_interval_sec'],
        }


class MatrixClosedLoopE2E(Node):
    TOPICS = (
        '/lightning/loc_status',
        '/odom/current_pose',
        '/odom/mujoco_odom',
        '/cmd_vel_nav',
        '/cmd_vel',
        '/cmd_vel_safe',
        '/laser_scan',
        '/nav_safety_gate/gate_status',
    )
    LIFECYCLE_NODES = (
        'map_server',
        'planner_server',
        'controller_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    )

    def __init__(self, args):
        super().__init__('matrix_closed_loop_e2e')
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.standup_client = (
            self.create_client(Trigger, args.standup_service)
            if args.standup_service else None
        )
        self.lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in self.LIFECYCLE_NODES
        }

        self.stats = {topic: TopicStats() for topic in self.TOPICS}
        self.normal_streak = 0
        self.last_loc_status = None
        self.non_normal_count = 0
        self.non_open_gate_count = 0
        self.gate_monitoring_active = False
        self.last_readiness_log_time = 0.0
        self.lightning_samples = []
        self.gt_samples = []
        self.latest_lightning = None
        self.latest_gt = None
        self.nonzero_command_count = {
            '/cmd_vel_nav': 0,
            '/cmd_vel': 0,
            '/cmd_vel_safe': 0,
        }
        self.first_nonzero_time = {
            '/cmd_vel_nav': None,
            '/cmd_vel': None,
            '/cmd_vel_safe': None,
        }

        self.create_subscription(
            UInt8, '/lightning/loc_status', self._loc_status_callback, 10)
        self.create_subscription(
            Odometry, '/odom/current_pose',
            lambda msg: self._odom_callback(msg, False), 20)
        self.create_subscription(
            Odometry, '/odom/mujoco_odom',
            lambda msg: self._odom_callback(msg, True), qos_profile_sensor_data)
        for topic in ('/cmd_vel_nav', '/cmd_vel', '/cmd_vel_safe'):
            self.create_subscription(
                Twist, topic,
                lambda msg, topic_name=topic: self._cmd_callback(topic_name, msg),
                20)
        self.create_subscription(
            LaserScan, '/laser_scan', self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            UInt8, '/nav_safety_gate/gate_status', self._gate_status_callback, 10)
        self.create_timer(0.05, self._sample_lightning_global_pose)

    @staticmethod
    def _now_monotonic():
        return time.monotonic()

    def _loc_status_callback(self, msg):
        now = self._now_monotonic()
        self.stats['/lightning/loc_status'].add(now)
        self.last_loc_status = msg.data
        if msg.data == LOC_NORMAL:
            self.normal_streak += 1
        else:
            self.normal_streak = 0
            self.non_normal_count += 1

    def _odom_callback(self, msg, ground_truth):
        now = self._now_monotonic()
        topic = '/odom/mujoco_odom' if ground_truth else '/odom/current_pose'
        source_time = (
            float(msg.header.stamp.sec) +
            float(msg.header.stamp.nanosec) * 1e-9)
        sample = PoseSample(
            receive_time=now,
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=yaw_from_quaternion(msg.pose.pose.orientation),
        )
        self.stats[topic].add(now, source_time)
        if ground_truth:
            self.latest_gt = sample
            self.gt_samples.append(sample)
        else:
            self.latest_lightning = sample

    def _sample_lightning_global_pose(self):
        """Sample map->base_link so trajectory scoring includes global corrections."""
        transform = self._lookup_map_to_base()
        if transform is None:
            return
        translation = transform.transform.translation
        self.lightning_samples.append(PoseSample(
            receive_time=self._now_monotonic(),
            x=translation.x,
            y=translation.y,
            yaw=yaw_from_quaternion(transform.transform.rotation),
        ))

    def _cmd_callback(self, topic, msg):
        now = self._now_monotonic()
        self.stats[topic].add(now)
        magnitude = max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.angular.z))
        if magnitude > NONZERO_EPSILON:
            self.nonzero_command_count[topic] += 1
            if self.first_nonzero_time[topic] is None:
                self.first_nonzero_time[topic] = now
            if topic == '/cmd_vel_safe':
                # Before the first safe motion command, CMD_TIMEOUT is the
                # expected fail-closed idle state. Start evaluating the gate
                # only once the command chain has actually been enabled.
                self.gate_monitoring_active = True

    def _scan_callback(self, msg):
        source_time = (
            float(msg.header.stamp.sec) +
            float(msg.header.stamp.nanosec) * 1e-9)
        self.stats['/laser_scan'].add(self._now_monotonic(), source_time)

    def _gate_status_callback(self, msg):
        self.stats['/nav_safety_gate/gate_status'].add(self._now_monotonic())
        if self.gate_monitoring_active and msg.data != LOC_NORMAL:
            self.non_open_gate_count += 1

    def _spin_until(self, predicate, timeout_sec):
        deadline = self._now_monotonic() + timeout_sec
        while rclpy.ok() and self._now_monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _lookup_map_to_base(self):
        try:
            return self.tf_buffer.lookup_transform(
                'map', 'base_link', Time(), timeout=Duration(seconds=0.2))
        except TransformException:
            return None

    def _wait_for_lifecycle_active(self, name, client, deadline):
        remaining = deadline - self._now_monotonic()
        if remaining <= 0.0 or not client.wait_for_service(
                timeout_sec=min(5.0, remaining)):
            self.get_logger().error(f'Lifecycle service unavailable: {name}')
            return False

        last_label = 'unknown'
        while rclpy.ok() and self._now_monotonic() < deadline:
            future = client.call_async(GetState.Request())
            remaining = deadline - self._now_monotonic()
            if self._spin_until(future.done, min(2.0, max(0.0, remaining))):
                response = future.result()
                if response is not None:
                    last_label = response.current_state.label
                    if response.current_state.id == State.PRIMARY_STATE_ACTIVE:
                        return True
            remaining = deadline - self._now_monotonic()
            if remaining > 0.0:
                self._spin_until(lambda: False, min(0.2, remaining))

        self.get_logger().error(
            f'Lifecycle node did not become active: {name} ({last_label})')
        return False

    def wait_until_ready(self):
        self.get_logger().info(
            'Waiting for 10 consecutive NORMAL localization samples, odometry, '
            'ground truth, LaserScan, map->base_link TF, and NavigateToPose server')

        def ready():
            now = self._now_monotonic()
            status_times = self.stats['/lightning/loc_status'].times
            checks = {
                'normal_streak': self.normal_streak >= NORMAL_SAMPLES_REQUIRED,
                'status_fresh': bool(status_times) and now - status_times[-1] <= 0.25,
                'lightning_odom': self.latest_lightning is not None,
                'ground_truth': self.latest_gt is not None,
                'laser_scan': bool(self.stats['/laser_scan'].times),
                'map_tf': self._lookup_map_to_base() is not None,
                'action_server': self.action_client.server_is_ready(),
            }
            if not all(checks.values()) and now - self.last_readiness_log_time >= 2.0:
                self.last_readiness_log_time = now
                status_age = now - status_times[-1] if status_times else math.inf
                self.get_logger().info(
                    'Readiness pending: '
                    + ', '.join(f'{name}={value}' for name, value in checks.items())
                    + f', normal_samples={self.normal_streak}, status_age={status_age:.3f}s')
            return all(checks.values())

        if not self._spin_until(ready, self.args.timeout):
            return False

        lifecycle_deadline = self._now_monotonic() + self.args.timeout
        for name, client in self.lifecycle_clients.items():
            if not self._wait_for_lifecycle_active(name, client, lifecycle_deadline):
                return False
        self.get_logger().info('All Nav2 lifecycle nodes and action server are ready')
        return True

    def reset_measurement_window(self):
        for stats in self.stats.values():
            stats.reset()
        self.lightning_samples = []
        self.gt_samples = []
        self.non_normal_count = 0
        self.non_open_gate_count = 0
        self.gate_monitoring_active = False
        for topic in self.nonzero_command_count:
            self.nonzero_command_count[topic] = 0
            self.first_nonzero_time[topic] = None

    def request_standup(self):
        if self.standup_client is None:
            return
        if not self.standup_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                f'stand-up service unavailable: {self.args.standup_service}')
        self.get_logger().info(
            f'Requesting Matrix stand-up via {self.args.standup_service}')
        future = self.standup_client.call_async(Trigger.Request())
        if not self._spin_until(future.done, 5.0):
            raise RuntimeError('Matrix stand-up request timed out')
        response = future.result()
        if response is None or not response.success:
            message = response.message if response is not None else 'no response'
            raise RuntimeError(f'Matrix stand-up request failed: {message}')
        self.get_logger().info(
            f'{response.message}; settling for {self.args.standup_settle:.1f}s')
        self._spin_until(lambda: False, self.args.standup_settle)

    def build_goal(self):
        transform = self._lookup_map_to_base()
        if transform is None:
            raise RuntimeError('map->base_link TF disappeared before goal creation')
        origin = transform.transform.translation
        current_yaw = yaw_from_quaternion(transform.transform.rotation)
        cosine = math.cos(current_yaw)
        sine = math.sin(current_yaw)
        goal_x = origin.x + cosine * self.args.relative_x - sine * self.args.relative_y
        goal_y = origin.y + sine * self.args.relative_x + cosine * self.args.relative_y
        goal_yaw = normalize_angle(current_yaw + self.args.relative_yaw)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        qx, qy, qz, qw = quaternion_from_yaw(goal_yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        return goal, {'x': goal_x, 'y': goal_y, 'yaw': goal_yaw}

    def execute_goal(self, goal):
        send_future = self.action_client.send_goal_async(goal)
        if not self._spin_until(send_future.done, min(10.0, self.args.timeout)):
            return {'accepted': False, 'status': 'SEND_TIMEOUT'}
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {'accepted': False, 'status': 'REJECTED'}

        self.get_logger().info('NavigateToPose goal accepted')
        result_future = goal_handle.get_result_async()
        if not self._spin_until(result_future.done, self.args.timeout):
            cancel_future = goal_handle.cancel_goal_async()
            self._spin_until(cancel_future.done, 2.0)
            return {'accepted': True, 'status': 'TIMEOUT'}

        wrapped_result = result_future.result()
        status_names = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        status = status_names.get(wrapped_result.status, str(wrapped_result.status))
        return {
            'accepted': True,
            'status': status,
            'status_code': wrapped_result.status,
        }

    @staticmethod
    def _relative_pose(sample, origin):
        dx = sample.x - origin.x
        dy = sample.y - origin.y
        cosine = math.cos(origin.yaw)
        sine = math.sin(origin.yaw)
        return (
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
            normalize_angle(sample.yaw - origin.yaw),
        )

    def trajectory_metrics(self):
        if not self.lightning_samples or not self.gt_samples:
            return {'matched_samples': 0}

        gt_times = [sample.receive_time for sample in self.gt_samples]
        pairs = []
        for lightning in self.lightning_samples:
            index = bisect.bisect_left(gt_times, lightning.receive_time)
            candidates = []
            if index < len(self.gt_samples):
                candidates.append(self.gt_samples[index])
            if index > 0:
                candidates.append(self.gt_samples[index - 1])
            if not candidates:
                continue
            ground_truth = min(
                candidates,
                key=lambda sample: abs(sample.receive_time - lightning.receive_time))
            skew = abs(ground_truth.receive_time - lightning.receive_time)
            if skew <= POSE_PAIR_MAX_SKEW_SEC:
                pairs.append((lightning, ground_truth, skew))

        if not pairs:
            return {'matched_samples': 0}

        lightning_origin, gt_origin, _ = pairs[0]
        position_errors = []
        yaw_errors = []
        skews = []
        for lightning, ground_truth, skew in pairs:
            lx, ly, lyaw = self._relative_pose(lightning, lightning_origin)
            gx, gy, gyaw = self._relative_pose(ground_truth, gt_origin)
            position_errors.append(math.hypot(lx - gx, ly - gy))
            yaw_errors.append(abs(normalize_angle(lyaw - gyaw)))
            skews.append(skew)

        return {
            'alignment': 'first matched pose; independent SE(2) relative trajectories',
            'matched_samples': len(pairs),
            'max_pair_skew_sec': max(skews),
            'position_rmse_m': math.sqrt(
                sum(error * error for error in position_errors) / len(position_errors)),
            'position_max_error_m': max(position_errors),
            'yaw_rmse_rad': math.sqrt(
                sum(error * error for error in yaw_errors) / len(yaw_errors)),
            'yaw_max_error_rad': max(yaw_errors),
        }

    def final_metrics(self, goal_pose):
        transform = self._lookup_map_to_base()
        if transform is None:
            goal_error = {'position_m': None, 'yaw_rad': None}
        else:
            translation = transform.transform.translation
            yaw = yaw_from_quaternion(transform.transform.rotation)
            goal_error = {
                'position_m': math.hypot(
                    translation.x - goal_pose['x'], translation.y - goal_pose['y']),
                'yaw_rad': abs(normalize_angle(yaw - goal_pose['yaw'])),
            }

        if len(self.gt_samples) >= 2:
            gt_start = self.gt_samples[0]
            gt_end = self.gt_samples[-1]
            physical = {
                'translation_m': math.hypot(gt_end.x - gt_start.x, gt_end.y - gt_start.y),
                'yaw_change_rad': abs(normalize_angle(gt_end.yaw - gt_start.yaw)),
            }
        else:
            physical = {'translation_m': None, 'yaw_change_rad': None}
        return goal_error, physical

    def build_report(self, action_result, goal_pose, readiness_ok):
        trajectory = self.trajectory_metrics()
        goal_error, physical = self.final_metrics(goal_pose)
        topic_stats = {topic: stats.report() for topic, stats in self.stats.items()}

        command_delivery = {
            topic: {
                'nonzero_count': self.nonzero_command_count[topic],
                'reached': self.nonzero_command_count[topic] > 0,
                'first_nonzero_offset_sec': (
                    self.first_nonzero_time[topic] - self.measurement_start
                    if self.first_nonzero_time[topic] is not None else None),
            }
            for topic in self.nonzero_command_count
        }

        requested_translation = math.hypot(self.args.relative_x, self.args.relative_y)
        requested_yaw = abs(self.args.relative_yaw)
        physical_motion_detected = False
        if physical['translation_m'] is not None:
            translation_ok = (
                requested_translation < 0.1 or
                physical['translation_m'] >= max(0.05, 0.5 * requested_translation))
            rotation_ok = requested_yaw < 0.1 or physical['yaw_change_rad'] >= 0.05
            physical_motion_detected = translation_ok and rotation_ok

        odom_stats = topic_stats['/odom/current_pose']
        scan_stats = topic_stats['/laser_scan']
        controller_stats = topic_stats['/cmd_vel_nav']
        loc_status_stats = topic_stats['/lightning/loc_status']
        gate_status_stats = topic_stats['/nav_safety_gate/gate_status']
        command_offsets = [
            command_delivery[topic]['first_nonzero_offset_sec']
            for topic in ('/cmd_vel_nav', '/cmd_vel', '/cmd_vel_safe')
        ]
        propagation_ok = all(offset is not None for offset in command_offsets)
        command_latency = {
            'smoother_startup_sec': None,
            'safety_gate_sec': None,
        }
        if propagation_ok:
            command_latency = {
                'smoother_startup_sec': command_offsets[1] - command_offsets[0],
                'safety_gate_sec': command_offsets[2] - command_offsets[1],
            }
            propagation_ok = (
                command_offsets[0] <= command_offsets[1] + 0.05 and
                command_offsets[1] <= command_offsets[2] + 0.05 and
                command_latency['smoother_startup_sec'] <=
                self.args.max_smoother_startup_latency and
                command_latency['safety_gate_sec'] <=
                self.args.max_safety_gate_latency)

        checks = {
            'readiness': readiness_ok,
            'localization_remained_normal': self.non_normal_count == 0,
            'localization_status_rate': (
                loc_status_stats['frequency_hz'] is not None and
                loc_status_stats['frequency_hz'] >= 5.0 and
                loc_status_stats['max_interval_sec'] <= 0.30),
            'safety_gate_remained_open': (
                self.non_open_gate_count == 0 and
                gate_status_stats['frequency_hz'] is not None and
                gate_status_stats['max_interval_sec'] <= 0.30),
            'action_succeeded': action_result.get('status') == 'SUCCEEDED',
            'all_command_stages_reached': all(
                item['reached'] for item in command_delivery.values()),
            'physical_motion_detected': physical_motion_detected,
            'command_propagation_latency': propagation_ok,
            'odom_rate': (
                odom_stats['source_frequency_hz'] is not None and
                odom_stats['source_frequency_hz'] >= 8.0 and
                odom_stats['source_max_interval_sec'] <= 0.25),
            'scan_rate': (
                scan_stats['frequency_hz'] is not None and
                scan_stats['frequency_hz'] >= 5.0 and
                scan_stats['max_interval_sec'] <= 0.30),
            'controller_command_rate': (
                controller_stats['frequency_hz'] is not None and
                controller_stats['frequency_hz'] >= 8.0 and
                controller_stats['max_interval_sec'] <= 0.25),
            'enough_trajectory_pairs': trajectory.get('matched_samples', 0) >= 10,
            'position_rmse': (
                trajectory.get('position_rmse_m', math.inf) <= self.args.max_rmse),
            'yaw_rmse': (
                trajectory.get('yaw_rmse_rad', math.inf) <= self.args.max_yaw_rmse),
            'position_max_error': (
                trajectory.get('position_max_error_m', math.inf) <= self.args.max_error),
            'yaw_max_error': (
                trajectory.get('yaw_max_error_rad', math.inf) <=
                self.args.max_yaw_error),
            'goal_position_error': (
                goal_error['position_m'] is not None and
                goal_error['position_m'] <= self.args.goal_tolerance),
            'goal_yaw_error': (
                goal_error['yaw_rad'] is not None and
                goal_error['yaw_rad'] <= self.args.goal_yaw_tolerance),
        }
        failures = [name for name, passed in checks.items() if not passed]
        return {
            'schema_version': 1,
            'generated_at': datetime.now(timezone.utc).isoformat(),
            'parameters': {
                'relative_x_m': self.args.relative_x,
                'relative_y_m': self.args.relative_y,
                'relative_yaw_rad': self.args.relative_yaw,
                'timeout_sec': self.args.timeout,
                'max_rmse': self.args.max_rmse,
                'max_error': self.args.max_error,
                'goal_tolerance': self.args.goal_tolerance,
                'max_yaw_rmse_rad': self.args.max_yaw_rmse,
                'max_yaw_error_rad': self.args.max_yaw_error,
                'goal_yaw_tolerance_rad': self.args.goal_yaw_tolerance,
                'max_smoother_startup_latency_sec':
                    self.args.max_smoother_startup_latency,
                'max_safety_gate_latency_sec': self.args.max_safety_gate_latency,
            },
            'goal_map': goal_pose,
            'action': action_result,
            'topic_stats': topic_stats,
            'command_delivery': command_delivery,
            'command_latency': command_latency,
            'relative_trajectory': {
                **trajectory,
                'lightning_source': 'TF map->base_link',
                'ground_truth_source': '/odom/mujoco_odom (evaluation only)',
            },
            'final_goal_error': goal_error,
            'ground_truth_physical_motion': physical,
            'acceptance': checks,
            'failures': failures,
            'pass': not failures,
        }

    def run(self):
        if not self.wait_until_ready():
            raise RuntimeError(
                'readiness timeout: localization/TF/action/sensor prerequisites not met')

        self.request_standup()
        goal, goal_pose = self.build_goal()
        self.reset_measurement_window()
        self.measurement_start = self._now_monotonic()
        self.get_logger().info(
            f'Sending map goal x={goal_pose["x"]:.3f}, y={goal_pose["y"]:.3f}, '
            f'yaw={goal_pose["yaw"]:.3f}')
        action_result = self.execute_goal(goal)
        # CMD_TIMEOUT after a completed action is the required stopped state,
        # not a navigation-time gate failure.
        self.gate_monitoring_active = False
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.build_report(action_result, goal_pose, True)


def positive_float(value):
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError('must be greater than zero')
    return parsed


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Matrix + Lightning NavigateToPose closed-loop E2E test')
    parser.add_argument('--relative-x', type=float, default=1.0)
    parser.add_argument('--relative-y', type=float, default=0.0)
    parser.add_argument('--relative-yaw', type=float, default=0.0)
    parser.add_argument('--timeout', type=positive_float, default=90.0)
    parser.add_argument('--output', default='matrix_closed_loop_e2e.json')
    parser.add_argument('--max-rmse', type=positive_float, default=0.20)
    parser.add_argument('--max-error', type=positive_float, default=0.40)
    parser.add_argument('--goal-tolerance', type=positive_float, default=0.25)
    parser.add_argument('--max-yaw-rmse', type=positive_float, default=0.20)
    parser.add_argument('--max-yaw-error', type=positive_float, default=0.40)
    parser.add_argument('--goal-yaw-tolerance', type=positive_float, default=0.25)
    parser.add_argument(
        '--max-smoother-startup-latency', type=positive_float, default=0.50,
        help='Maximum expected delay from raw controller output to the first '
             'nonzero velocity-smoother output')
    parser.add_argument(
        '--max-safety-gate-latency', type=positive_float, default=0.10,
        help='Maximum delay from velocity-smoother output to safety-gate output')
    parser.add_argument('--standup-service', default='')
    parser.add_argument('--standup-settle', type=positive_float, default=4.0)
    return parser.parse_known_args(argv)


def write_report(output_path, report):
    path = Path(output_path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')


def main(argv=None):
    args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = MatrixClosedLoopE2E(args)
    exit_code = 2
    try:
        report = node.run()
        exit_code = 0 if report['pass'] else 1
    except Exception as error:  # Ensure infrastructure failures are machine-readable.
        report = {
            'schema_version': 1,
            'generated_at': datetime.now(timezone.utc).isoformat(),
            'pass': False,
            'failures': ['infrastructure_error'],
            'error': str(error),
        }
        node.get_logger().error(str(error))
    finally:
        write_report(args.output, report)
        node.get_logger().info(
            f'{"PASS" if report["pass"] else "FAIL"}: report written to {args.output}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
