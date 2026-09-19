#!/usr/bin/env python3
"""Behavior and real-executor watchdog regression; use an isolated ROS domain."""
import importlib.util
from pathlib import Path
import time
import unittest

import rclpy
from rclpy.clock import ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt8

SPEC = importlib.util.spec_from_file_location(
    'nav_safety_gate', Path(__file__).resolve().parents[1] / 'scripts/nav_safety_gate.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def motion(x=0.1):
    result = Twist()
    result.linear.x = x
    return result


class GateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.gate = MODULE.NavSafetyGate()
        self.outputs = []
        self.gate.publish_cmd = lambda command, status: self.outputs.append((command, status))

    def tearDown(self):
        self.gate.destroy_node()

    def assertStopped(self, status):
        command, actual_status = self.outputs[-1]
        self.assertEqual(command, Twist())
        self.assertEqual(actual_status, status)

    def normal(self):
        self.gate.loc_status_callback(UInt8(data=self.gate.LOC_NORMAL))

    def test_initial_unknown_blocks(self):
        self.gate.cmd_vel_callback(motion())
        self.assertStopped(self.gate.LOC_LOST)

    def test_fresh_normal_preserves_command(self):
        self.normal()
        command = motion()
        command.angular.z = 0.05
        self.gate.cmd_vel_callback(command)
        self.assertEqual(self.outputs[-1], (command, self.gate.LOC_NORMAL))

    def test_loss_stops_immediately_without_next_command(self):
        for status in (self.gate.LOC_DEGRADED, self.gate.LOC_LOST,
                       self.gate.LOC_UNKNOWN, 255):
            self.normal()
            self.gate.cmd_vel_callback(motion())
            count = len(self.outputs)
            self.gate.loc_status_callback(UInt8(data=status))
            self.assertEqual(len(self.outputs), count + 1)
            expected = status if status != 255 else self.gate.LOC_UNKNOWN
            self.assertStopped(expected)
            self.gate.watchdog_callback()
            self.assertStopped(expected)

    def test_recovery_does_not_replay_pre_loss_command(self):
        self.normal()
        self.gate.cmd_vel_callback(motion())
        self.gate.loc_status_callback(UInt8(data=self.gate.LOC_DEGRADED))
        count = len(self.outputs)
        self.normal()
        self.gate.watchdog_callback()
        self.assertEqual(len(self.outputs), count)
        self.gate.cmd_vel_callback(motion(0.05))
        self.assertEqual(self.outputs[-1][0].linear.x, 0.05)

    def test_emergency_stop_dominates_and_release_does_not_replay(self):
        self.normal()
        self.gate.cmd_vel_callback(motion())
        self.gate.emergency_stop_callback(Bool(data=True))
        self.assertStopped(self.gate.GATE_EMERGENCY_STOP)
        self.gate.loc_status_callback(UInt8(data=self.gate.LOC_DEGRADED))
        self.assertStopped(self.gate.GATE_EMERGENCY_STOP)
        self.gate.emergency_stop_callback(Bool(data=False))
        self.gate.cmd_vel_callback(motion())
        self.assertStopped(self.gate.LOC_DEGRADED)

    def test_nonfinite_any_axis_is_blocked_until_valid_command(self):
        self.normal()
        for field in ('linear', 'angular'):
            for axis in ('x', 'y', 'z'):
                for value in (float('nan'), float('inf'), float('-inf')):
                    command = motion()
                    setattr(getattr(command, field), axis, value)
                    self.gate.cmd_vel_callback(command)
                    self.assertStopped(self.gate.GATE_INVALID_CMD)
                    self.gate.watchdog_callback()
                    self.assertStopped(self.gate.GATE_INVALID_CMD)
        self.gate.cmd_vel_callback(motion())
        self.assertEqual(self.outputs[-1][1], self.gate.LOC_NORMAL)

    def expire(self, attribute, seconds):
        setattr(self.gate, attribute, Time(
            nanoseconds=self.gate.safety_clock.now().nanoseconds - int(seconds * 1e9),
            clock_type=ClockType.STEADY_TIME))

    def test_independent_receipt_watchdogs(self):
        self.normal()
        self.gate.cmd_vel_callback(motion())
        self.expire('last_cmd_time', self.gate.cmd_timeout_sec + 0.01)
        self.gate.watchdog_callback()
        self.assertStopped(self.gate.GATE_CMD_TIMEOUT)
        self.gate.cmd_vel_callback(motion())
        self.expire('last_status_time', self.gate.watchdog_timeout_sec + 0.01)
        self.gate.watchdog_callback()
        self.assertStopped(self.gate.LOC_LOST)

    def test_real_timer_expires_while_ros_clock_is_paused(self):
        self.gate.set_parameters([Parameter('use_sim_time', value=True)])
        self.assertEqual(self.gate.get_clock().now().nanoseconds, 0)
        self.normal()
        self.gate.cmd_vel_callback(motion())
        executor = SingleThreadedExecutor()
        executor.add_node(self.gate)
        start = time.monotonic()
        try:
            while time.monotonic() - start < 1.0:
                executor.spin_once(timeout_sec=0.02)
                if self.outputs[-1][1] == self.gate.LOC_LOST:
                    break
        finally:
            executor.remove_node(self.gate)
            executor.shutdown()
        self.assertStopped(self.gate.LOC_LOST)
        self.assertEqual(self.gate.get_clock().now().nanoseconds, 0)
        self.assertLess(time.monotonic() - start, 0.5)

    def test_real_topics_stop_on_loss_and_paused_clock_timeout(self):
        # Restore production publishing and observe DDS output. No actuator
        # bridge is launched; the test command exists only in its isolated domain.
        self.gate.publish_cmd = MODULE.NavSafetyGate.publish_cmd.__get__(self.gate)
        self.gate.set_parameters([Parameter('use_sim_time', value=True)])
        probe = Node('nav_safety_gate_test_probe')
        observed = []
        probe.create_subscription(Twist, '/cmd_vel_safe',
                                  lambda message: observed.append(message), 10)
        status_pub = probe.create_publisher(UInt8, '/lightning/loc_status', 1)
        cmd_pub = probe.create_publisher(Twist, '/cmd_vel', 1)
        executor = SingleThreadedExecutor()
        executor.add_node(self.gate)
        executor.add_node(probe)

        def until(condition, timeout=2.0):
            deadline = time.monotonic() + timeout
            while not condition() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
            self.assertTrue(condition())

        try:
            until(lambda: status_pub.get_subscription_count() == 1 and
                  cmd_pub.get_subscription_count() == 1 and
                  self.gate.cmd_vel_pub.get_subscription_count() == 1)
            status_pub.publish(UInt8(data=self.gate.LOC_NORMAL))
            until(lambda: self.gate.current_status == self.gate.LOC_NORMAL)
            observed.clear()
            cmd_pub.publish(motion())
            until(lambda: any(message.linear.x == 0.1 for message in observed))
            observed.clear()
            status_pub.publish(UInt8(data=self.gate.LOC_DEGRADED))
            until(lambda: observed and observed[-1] == Twist())
            status_pub.publish(UInt8(data=self.gate.LOC_NORMAL))
            until(lambda: self.gate.current_status == self.gate.LOC_NORMAL)
            observed.clear()
            cmd_pub.publish(motion())
            until(lambda: any(message.linear.x == 0.1 for message in observed))
            observed.clear()
            until(lambda: observed and observed[-1] == Twist(), timeout=0.6)
            self.assertEqual(self.gate.get_clock().now().nanoseconds, 0)
        finally:
            executor.remove_node(probe)
            executor.remove_node(self.gate)
            executor.shutdown()
            probe.destroy_node()


if __name__ == '__main__':
    unittest.main()
