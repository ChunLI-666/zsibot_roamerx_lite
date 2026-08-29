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
  Watchdog timeout: output zero velocity if loc_status or cmd_vel becomes stale
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt8


class NavSafetyGate(Node):

    LOC_UNKNOWN = 0
    LOC_NORMAL = 1
    LOC_DEGRADED = 2
    LOC_LOST = 3
    GATE_EMERGENCY_STOP = 4
    GATE_CMD_TIMEOUT = 5

    def __init__(self):
        super().__init__('nav_safety_gate')

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
        cmd_vel_in = self.get_parameter('cmd_vel_input_topic').value
        cmd_vel_out = self.get_parameter('cmd_vel_output_topic').value
        loc_status_topic = self.get_parameter('loc_status_topic').value
        emergency_stop_topic = self.get_parameter('emergency_stop_topic').value

        self.current_status = self.LOC_UNKNOWN
        self.emergency_stop_active = False
        self.last_status_time = None
        self.last_cmd_time = None

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist, cmd_vel_in, self.cmd_vel_callback, 10)

        self.loc_status_sub = self.create_subscription(
            UInt8, loc_status_topic, self.loc_status_callback, qos_reliable)

        self.emergency_stop_sub = self.create_subscription(
            Bool, emergency_stop_topic, self.emergency_stop_callback, qos_reliable)

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_out, 10)
        self.gate_status_pub = self.create_publisher(UInt8, '~/gate_status', 1)
        self.watchdog_timer = self.create_timer(
            max(0.02, stop_publish_period_sec), self.watchdog_callback)

        self.get_logger().info(
            f'NavSafetyGate started: {cmd_vel_in} -> {cmd_vel_out}, '
            f'emergency_stop={emergency_stop_topic}, '
            f'loc_watchdog={self.watchdog_timeout_sec*1000:.0f}ms, '
            f'cmd_watchdog={self.cmd_timeout_sec*1000:.0f}ms, '
            f'policy: only NORMAL and no emergency stop passes')

    def loc_status_callback(self, msg: UInt8):
        prev = self.current_status
        self.current_status = msg.data
        self.last_status_time = self.get_clock().now()
        if prev != self.current_status:
            self.get_logger().info(
                f'Loc status changed: {prev} -> {self.current_status}')

    def emergency_stop_callback(self, msg: Bool):
        prev = self.emergency_stop_active
        self.emergency_stop_active = msg.data
        if prev != self.emergency_stop_active:
            self.get_logger().warn(
                f'Emergency stop changed: {prev} -> {self.emergency_stop_active}')
            if self.emergency_stop_active:
                self.publish_zero(self.GATE_EMERGENCY_STOP)

    def cmd_vel_callback(self, msg: Twist):
        now = self.get_clock().now()
        self.last_cmd_time = now
        safe_cmd = Twist()
        gate_status = self.LOC_UNKNOWN

        timed_out = self._is_timed_out(now)

        if self.emergency_stop_active:
            gate_status = self.GATE_EMERGENCY_STOP
        elif timed_out:
            gate_status = self.LOC_LOST
        elif self.current_status == self.LOC_NORMAL:
            safe_cmd = msg
            gate_status = self.LOC_NORMAL
        elif self.current_status == self.LOC_DEGRADED:
            gate_status = self.LOC_DEGRADED
        # else: UNKNOWN or LOST -> zero velocity (already default)
        # DEGRADED also outputs zero velocity for safety

        self.publish_cmd(safe_cmd, gate_status)

    def watchdog_callback(self):
        """Continuously enforce stop when either heartbeat becomes stale."""
        now = self.get_clock().now()
        if self.emergency_stop_active:
            self.publish_zero(self.GATE_EMERGENCY_STOP)
        elif self._is_timed_out(now):
            self.publish_zero(self.LOC_LOST)
        elif self._is_cmd_timed_out(now):
            self.publish_zero(self.GATE_CMD_TIMEOUT)

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
