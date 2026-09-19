#!/usr/bin/python3

"""
Navigation Safety Gate - Blocks cmd_vel when localization is not NORMAL.

Subscribes:
  /cmd_vel (Twist) - raw velocity from navigation stack
  /lightning/loc_status (UInt8) - localization status (0=UNKNOWN, 1=NORMAL, 2=DEGRADED, 3=LOST)
  /emergency_stop (Bool) - emergency stop latch/state

Publishes:
  /cmd_vel_safe (Twist) - safe velocity output
  ~/gate_status (UInt8) - current gate policy for debugging

Policy:
  NORMAL(1) and emergency_stop=false: forward cmd_vel as-is
  emergency_stop=true: output zero velocity
  DEGRADED(2)/LOST(3)/UNKNOWN(0): output zero velocity
  Watchdog timeout: output zero velocity if loc_status not received within timeout
"""

import math
import copy
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt8


class EpochGatePolicy:
    """Same-host, monotonic-clock authorization; never stamps an untyped command."""

    def __init__(self, gate_session, boot_id, localization_ttl_ns=400_000_000,
                 intent_ttl_ns=400_000_000, execution_ttl_ns=300_000_000,
                 command_ttl_ns=300_000_000):
        self.gate_session, self.boot_id = gate_session, boot_id
        self.loc_ttl, self.intent_ttl = localization_ttl_ns, intent_ttl_ns
        self.execution_ttl, self.command_ttl = execution_ttl_ns, command_ttl_ns
        self.loc = self.intent = self.execution = self.command = None
        self.loc_receipt = self.intent_receipt = self.execution_receipt = 0
        self.output_advanced = self.output_stamp = 0
        self.output_regressed = self.map_changed = False
        self.loaded_map = None
        self.epoch_highwater = self.commits_highwater = 0
        self.output_identity = None
        self.retired_loc, self.retired_nav, self.retired_controllers = set(), set(), set()
        self.source_highwater, self.relay_highwater = {}, {}
        self.relay_session = None
        self.retired_relays = set()
        self.completed_tokens = set()

    @staticmethod
    def fresh(stamp, now, ttl):
        return stamp > 0 and stamp <= now and now - stamp <= ttl

    @staticmethod
    def token_valid(token):
        identity = token.localization
        return (identity.process_session_id and identity.map_loaded_instance and identity.epoch > 0 and
                identity.commits > 0 and token.navigation_session_id and token.task_sequence > 0 and
                token.plan_sequence > 0 and token.gate_session_id)

    @staticmethod
    def token_key(token):
        identity = token.localization
        return (identity.process_session_id, identity.map_loaded_instance, identity.epoch, identity.commits,
                token.navigation_session_id, token.task_sequence, token.plan_sequence, token.gate_session_id)

    def terminal_current(self):
        return self.intent is not None and self.token_key(self.intent.token) in self.completed_tokens

    @staticmethod
    def pose_valid(pose):
        p, q = pose.pose.position, pose.pose.orientation
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        return (pose.header.frame_id and pose.header.stamp.sec >= 0 and pose.header.stamp.nanosec < 1_000_000_000 and
                (pose.header.stamp.sec > 0 or pose.header.stamp.nanosec > 0) and
                all(math.isfinite(v) for v in (p.x, p.y, p.z, norm)) and abs(norm - 1.) < 1e-3)

    def localization(self, message, now):
        session = message.identity.process_session_id
        if not session or session in self.retired_loc:
            return
        if self.loc and session == self.loc.identity.process_session_id:
            if message.heartbeat_sequence <= self.loc.heartbeat_sequence:
                return
            ready_claim = message.output_ready and message.fusion_ready and message.lifecycle_enabled and message.health == 1
            if ready_claim and (message.identity.epoch < self.epoch_highwater or message.identity.commits < self.commits_highwater):
                return
        elif self.loc:
            self.retired_loc.add(self.loc.identity.process_session_id)
            self.epoch_highwater = self.commits_highwater = 0
        loaded = message.identity.map_loaded_instance
        if loaded:
            if self.loaded_map is None:
                self.loaded_map = loaded
            if loaded != self.loaded_map:
                self.map_changed = True
        self.epoch_highwater = max(self.epoch_highwater, message.identity.epoch)
        self.commits_highwater = max(self.commits_highwater, message.identity.commits)
        stamp = message.output_pose.header.stamp.sec * 1_000_000_000 + message.output_pose.header.stamp.nanosec
        if (message.identity.map_loaded_instance and message.identity.epoch > 0 and message.identity.commits > 0 and
                message.identity.epoch >= self.epoch_highwater and message.identity.commits >= self.commits_highwater):
            if message.identity != self.output_identity:
                self.output_identity = copy.deepcopy(message.identity)
                self.output_regressed = False
                self.output_stamp, self.output_advanced = max(0, stamp), now if stamp > 0 else 0
            elif 0 < stamp < self.output_stamp:
                self.output_regressed = True
            elif stamp > self.output_stamp:
                self.output_stamp, self.output_advanced = stamp, now
        self.loc, self.loc_receipt = copy.deepcopy(message), now

    def navigation_intent(self, message, now):
        token = message.token
        session = token.navigation_session_id
        if (not self.token_valid(token) or not self.loc or token.localization != self.loc.identity or
                token.gate_session_id != self.gate_session or message.boot_id != self.boot_id or
                session in self.retired_nav or not self.fresh(message.source_steady_time_ns, now, self.intent_ttl)):
            return
        if self.intent and session == self.intent.token.navigation_session_id:
            previous = self.intent.token
            if (message.heartbeat_sequence <= self.intent.heartbeat_sequence or token.task_sequence < previous.task_sequence or
                    (token.task_sequence == previous.task_sequence and token.plan_sequence < previous.plan_sequence)):
                return
        elif self.intent:
            self.retired_nav.add(self.intent.token.navigation_session_id)
        self.intent, self.intent_receipt = copy.deepcopy(message), now

    def execution_state(self, message, now):
        session = message.controller_session_id
        if (not session or session in self.retired_controllers or not self.intent or message.token != self.intent.token or
                message.boot_id != self.boot_id or not self.fresh(message.source_steady_time_ns, now, self.execution_ttl)):
            return
        if self.execution and session == self.execution.controller_session_id:
            if message.heartbeat_sequence <= self.execution.heartbeat_sequence:
                return
        elif self.execution:
            self.retired_controllers.add(self.execution.controller_session_id)
        # Normal completion must survive the following controller_idle heartbeat.
        # Only the controller that actually installed this token may seal it.
        if (not message.active and message.reason == 'goal_reached' and self.execution and
                self.execution.active and self.execution.token == message.token and
                self.execution.controller_session_id == session):
            self.completed_tokens.add(self.token_key(message.token))
            self.command = None
        self.execution, self.execution_receipt = copy.deepcopy(message), now

    def authority_reason(self, now):
        if self.terminal_current():
            return 'task_completed'
        loc = self.loc
        if (not loc or self.map_changed or self.output_regressed or loc.schema_version != 1 or
                not loc.lifecycle_enabled or not loc.fusion_ready or not loc.output_ready or loc.health != 1 or
                not loc.base_frame_id or not self.pose_valid(loc.output_pose) or
                not self.fresh(self.loc_receipt, now, self.loc_ttl) or
                not self.fresh(self.output_advanced, now, self.loc_ttl)):
            return 'localization_not_ready'
        intent = self.intent
        if (not intent or not intent.active or intent.token.localization != loc.identity or
                intent.token.gate_session_id != self.gate_session or
                not self.fresh(self.intent_receipt, now, self.intent_ttl) or
                not self.fresh(intent.source_steady_time_ns, now, self.intent_ttl)):
            return 'intent_not_ready'
        execution = self.execution
        if (not execution or not execution.active or execution.token != intent.token or
                not self.fresh(self.execution_receipt, now, self.execution_ttl) or
                not self.fresh(execution.source_steady_time_ns, now, self.execution_ttl)):
            return 'path_not_installed'
        return None

    def revocation_reason(self, now):
        # Changing intent to a new valid plan is an ordinary install handshake.
        # It must stop output, but must not invalidate that new plan's challenge.
        reason = self.authority_reason(now)
        if reason in ('localization_not_ready', 'intent_not_ready'):
            return reason
        if self.execution and self.intent and self.execution.token == self.intent.token:
            if reason == 'path_not_installed':
                return reason
            if self.command and self.command.token == self.intent.token:
                return self.command_reason(self.command, now)
        return None

    def command_reason(self, message, now):
        reason = self.authority_reason(now)
        if reason:
            return reason
        if (message.token != self.intent.token or message.controller_session_id != self.execution.controller_session_id or
                message.boot_id != self.boot_id or message.command_sequence == 0 or
                not message.relay_session_id or message.relay_sequence == 0 or message.max_age_ms == 0 or
                not self.fresh(message.source_steady_time_ns, now, min(self.command_ttl, message.max_age_ms * 1_000_000))):
            return 'command_identity_or_age'
        if not all(math.isfinite(v) for vector in (message.velocity.linear, message.velocity.angular)
                   for v in (vector.x, vector.y, vector.z)):
            return 'nonfinite_command'
        return None

    def accept_command(self, message, now):
        reason = self.command_reason(message, now)
        if reason:
            return reason
        previous = self.source_highwater.get(message.controller_session_id)
        if previous and (message.command_sequence < previous[0] or
                (message.command_sequence == previous[0] and
                 (message.source_steady_time_ns != previous[1] or message.token != previous[2])) or
                (message.command_sequence > previous[0] and message.source_steady_time_ns <= previous[1])):
            return 'source_sequence_regression'
        if message.relay_session_id in self.retired_relays:
            return 'retired_relay_session'
        if message.relay_sequence <= self.relay_highwater.get(message.relay_session_id, 0):
            return 'relay_sequence_regression'
        if self.relay_session and self.relay_session != message.relay_session_id:
            self.retired_relays.add(self.relay_session)
        self.relay_session = message.relay_session_id
        self.source_highwater[message.controller_session_id] = (
            message.command_sequence, message.source_steady_time_ns, copy.deepcopy(message.token))
        self.relay_highwater[message.relay_session_id] = message.relay_sequence
        self.command = copy.deepcopy(message)
        return None

    def stop_reason(self, now):
        if self.command is None:
            return self.authority_reason(now) or 'no_authorized_command'
        return self.command_reason(self.command, now)


class NavSafetyGate(Node):

    LOC_UNKNOWN = 0
    LOC_NORMAL = 1
    LOC_DEGRADED = 2
    LOC_LOST = 3
    GATE_EMERGENCY_STOP = 4
    GATE_CMD_TIMEOUT = 5
    GATE_INVALID_CMD = 6

    def __init__(self, **kwargs):
        super().__init__('nav_safety_gate', **kwargs)

        self.declare_parameter('enable_epoch_contract', False)
        self.epoch_contract = self.get_parameter('enable_epoch_contract').value
        self.declare_parameter('watchdog_timeout_ms', 200)
        self.declare_parameter('cmd_timeout_ms', 300)
        self.declare_parameter('stop_publish_period_ms', 50)
        self.declare_parameter('cmd_vel_input_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_output_topic', '/cmd_vel_safe')
        self.declare_parameter('loc_status_topic', '/lightning/loc_status')
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')

        self.watchdog_timeout_sec = self.get_parameter('watchdog_timeout_ms').value / 1000.0
        self.cmd_timeout_sec = self.get_parameter('cmd_timeout_ms').value / 1000.0
        stop_publish_period_sec = (
            self.get_parameter('stop_publish_period_ms').value / 1000.0)
        if not all(math.isfinite(value) and value > 0 for value in (
                self.watchdog_timeout_sec, self.cmd_timeout_sec,
                stop_publish_period_sec)):
            raise ValueError('Safety gate timeouts and timer period must be positive and finite')
        cmd_vel_in = self.get_parameter('cmd_vel_input_topic').value
        cmd_vel_out = self.get_parameter('cmd_vel_output_topic').value
        loc_status_topic = self.get_parameter('loc_status_topic').value
        emergency_stop_topic = self.get_parameter('emergency_stop_topic').value

        self.current_status = self.LOC_UNKNOWN
        self.emergency_stop_active = False
        self.last_status_time = None
        self.last_cmd_time = None
        self.invalid_cmd = False
        # These are receipt-time actuator watchdogs. A paused or rewound ROS
        # /clock must not keep the last motion command alive.
        self.safety_clock = Clock(clock_type=ClockType.STEADY_TIME)

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.cmd_vel_sub = self.loc_status_sub = None
        if not self.epoch_contract:
            self.cmd_vel_sub = self.create_subscription(Twist, cmd_vel_in, self.cmd_vel_callback, 1)
            self.loc_status_sub = self.create_subscription(UInt8, loc_status_topic, self.loc_status_callback, qos_reliable)

        self.emergency_stop_sub = self.create_subscription(
            Bool, emergency_stop_topic, self.emergency_stop_callback, qos_reliable)

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_out, 10)
        self.gate_status_pub = self.create_publisher(UInt8, '~/gate_status', 1)
        if self.epoch_contract:
            self.configure_epoch()
        self.watchdog_timer = self.create_timer(
            max(0.02, stop_publish_period_sec), self.watchdog_callback,
            clock=self.safety_clock)

        self.get_logger().info(
            f'NavSafetyGate started: {cmd_vel_in} -> {cmd_vel_out}, '
            f'emergency_stop={emergency_stop_topic}, '
            f'loc_watchdog={self.watchdog_timeout_sec*1000:.0f}ms, '
            f'cmd_watchdog={self.cmd_timeout_sec*1000:.0f}ms, '
            f'policy: only NORMAL and no emergency stop passes')

    def configure_epoch(self):
        from navigo_epoch_msgs.msg import LocalizationEpoch, NavigationIntent, NavExecutionState, EpochCommand
        from std_msgs.msg import String
        def timeout(name, default):
            self.declare_parameter(name, default)
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid epoch timeout: ' + name)
            return int(value * 1e9)
        self.epoch_policy = EpochGatePolicy(str(uuid.uuid4()),
            Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            timeout('epoch_localization_timeout', .4), timeout('epoch_intent_timeout', .4),
            timeout('epoch_execution_timeout', .3), timeout('epoch_command_timeout', .3))
        self.epoch_armed = False
        self.gate_session_pub = self.create_publisher(String, '/nav_epoch/gate_session', 1)
        self.epoch_loc_sub = self.create_subscription(LocalizationEpoch, '/lightning/localization_epoch',
            lambda message: self.epoch_authority_callback('localization', message), 10)
        self.epoch_intent_sub = self.create_subscription(NavigationIntent, '/nav_epoch/intent',
            lambda message: self.epoch_authority_callback('navigation_intent', message), 10)
        self.epoch_execution_sub = self.create_subscription(NavExecutionState, '/nav_epoch/execution',
            lambda message: self.epoch_authority_callback('execution_state', message), 10)
        self.epoch_command_sub = self.create_subscription(EpochCommand, '/cmd_vel_epoch', self.epoch_command_callback, 1)

    def epoch_authority_callback(self, method, message):
        getattr(self.epoch_policy, method)(message, time.monotonic_ns())
        self.enforce_epoch()

    def revoke_epoch(self):
        if not self.epoch_armed:
            return
        from std_msgs.msg import String
        self.epoch_armed = False
        self.epoch_policy.gate_session = str(uuid.uuid4())
        self.epoch_policy.command = None
        self.gate_session_pub.publish(String(data=self.epoch_policy.gate_session))

    def enforce_epoch(self):
        now = time.monotonic_ns()
        if self.epoch_policy.terminal_current():
            # Keep the challenge stable until the BT consumes the action result.
            # The sealed token cannot regain authority, even after active replay.
            self.epoch_armed = False
            self.epoch_policy.command = None
            self.publish_zero(self.GATE_EMERGENCY_STOP if self.emergency_stop_active else self.LOC_LOST)
            return
        if self.emergency_stop_active or self.epoch_policy.revocation_reason(now):
            self.revoke_epoch()
        elif self.epoch_policy.authority_reason(now) is None:
            self.epoch_armed = True
        if self.emergency_stop_active:
            self.publish_zero(self.GATE_EMERGENCY_STOP)
        elif self.epoch_policy.stop_reason(now):
            self.publish_zero(self.LOC_LOST)

    def epoch_command_callback(self, message):
        if self.emergency_stop_active:
            self.publish_zero(self.GATE_EMERGENCY_STOP)
            return
        reason = self.epoch_policy.accept_command(message, time.monotonic_ns())
        if reason:
            self.publish_zero(self.LOC_LOST)
        else:
            self.epoch_armed = True
            self.publish_cmd(message.velocity, self.LOC_NORMAL)

    def loc_status_callback(self, msg: UInt8):
        prev = self.current_status
        self.current_status = msg.data
        self.last_status_time = self.safety_clock.now()
        if prev != self.current_status:
            self.get_logger().info(
                f'Loc status changed: {prev} -> {self.current_status}')
        # Stop at the loss notification, not at the next command/timeout.
        # Returning to NORMAL does not replay a cached pre-loss command.
        if self.current_status != self.LOC_NORMAL:
            self.publish_zero(self._status_gate())

    def emergency_stop_callback(self, msg: Bool):
        prev = self.emergency_stop_active
        self.emergency_stop_active = msg.data
        if prev != self.emergency_stop_active:
            self.get_logger().warn(
                f'Emergency stop changed: {prev} -> {self.emergency_stop_active}')
            if self.emergency_stop_active:
                if self.epoch_contract:
                    self.revoke_epoch()
                self.publish_zero(self.GATE_EMERGENCY_STOP)

    def cmd_vel_callback(self, msg: Twist):
        if self.epoch_contract:
            self.publish_zero(self.LOC_LOST)
            return
        now = self.safety_clock.now()
        self.last_cmd_time = now
        self.invalid_cmd = not all(math.isfinite(value) for value in (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z))
        safe_cmd = Twist()
        gate_status = self.LOC_UNKNOWN

        timed_out = self._is_timed_out(now)

        if self.emergency_stop_active:
            gate_status = self.GATE_EMERGENCY_STOP
        elif timed_out:
            gate_status = self.LOC_LOST
        elif self.current_status != self.LOC_NORMAL:
            gate_status = self._status_gate()
        elif self.invalid_cmd:
            gate_status = self.GATE_INVALID_CMD
        elif self.current_status == self.LOC_NORMAL:
            safe_cmd = msg
            gate_status = self.LOC_NORMAL
        # else: UNKNOWN or LOST -> zero velocity (already default)
        # DEGRADED also outputs zero velocity for safety

        self.publish_cmd(safe_cmd, gate_status)

    def watchdog_callback(self):
        """Continuously enforce stop when either heartbeat becomes stale."""
        if self.epoch_contract:
            from std_msgs.msg import String
            self.gate_session_pub.publish(String(data=self.epoch_policy.gate_session))
            self.enforce_epoch()
            return
        now = self.safety_clock.now()
        if self.emergency_stop_active:
            self.publish_zero(self.GATE_EMERGENCY_STOP)
        elif self._is_timed_out(now):
            self.publish_zero(self.LOC_LOST)
        elif self.current_status != self.LOC_NORMAL:
            self.publish_zero(self._status_gate())
        elif self.invalid_cmd:
            self.publish_zero(self.GATE_INVALID_CMD)
        elif self._is_cmd_timed_out(now):
            self.publish_zero(self.GATE_CMD_TIMEOUT)

    def _status_gate(self):
        if self.emergency_stop_active:
            return self.GATE_EMERGENCY_STOP
        if self.current_status in (self.LOC_UNKNOWN, self.LOC_DEGRADED, self.LOC_LOST):
            return self.current_status
        return self.LOC_UNKNOWN

    def publish_zero(self, gate_status: int):
        self.publish_cmd(Twist(), gate_status)

    def publish_cmd(self, cmd: Twist, gate_status: int):
        self.cmd_vel_pub.publish(cmd)
        status_msg = UInt8()
        status_msg.data = gate_status
        self.gate_status_pub.publish(status_msg)

    def _is_timed_out(self, now) -> bool:
        if self.last_status_time is None:
            return True
        elapsed = (now - self.last_status_time).nanoseconds / 1e9
        return elapsed > self.watchdog_timeout_sec

    def _is_cmd_timed_out(self, now) -> bool:
        if self.last_cmd_time is None:
            return True
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
        return elapsed > self.cmd_timeout_sec


def main(args=None):
    rclpy.init(args=args)
    node = NavSafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
