#!/usr/bin/python3
"""
Emergency Stop Node with Keyboard Control

This node provides:
1. Emergency stop (SPACE or 's') - requests nav_safety_gate to output zero velocity
2. Resume (ENTER or 'r') - allows navigation to resume
3. Velocity monitoring - shows current cmd_vel

Usage:
    ros2 run robot_navigo emergency_stop.py

Controls:
    SPACE / s : Emergency STOP
    ENTER / r : Resume (stop overriding, let navigation control)
    q         : Quit

Author: Claude Code
"""

import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')

        # Parameters
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')
        self.declare_parameter('publish_rate', 50.0)  # Hz

        emergency_stop_topic = self.get_parameter('emergency_stop_topic').value
        publish_rate = self.get_parameter('publish_rate').value

        # State
        self.emergency_stop_active = False
        self.last_nav_vel = Twist()
        self.running = True

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )

        # Subscribe to navigation velocity (to monitor)
        self.nav_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav',  # Before velocity smoother
            self.nav_vel_callback,
            qos
        )

        # Publisher for emergency stop state. nav_safety_gate is the only node
        # allowed to publish final /cmd_vel_safe.
        self.emergency_stop_pub = self.create_publisher(
            Bool, emergency_stop_topic, qos)

        # Timer for publishing stop state periodically.
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # Statistics
        self.stop_count = 0

        self.get_logger().info('=' * 50)
        self.get_logger().info('  EMERGENCY STOP NODE READY')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Controls:')
        self.get_logger().info('  SPACE / s : Emergency STOP')
        self.get_logger().info('  ENTER / r : Resume navigation')
        self.get_logger().info('  q         : Quit')
        self.get_logger().info(f'Publishing emergency stop state to {emergency_stop_topic}')
        self.get_logger().info('=' * 50)

    def nav_vel_callback(self, msg: Twist):
        """Monitor navigation velocity commands."""
        self.last_nav_vel = msg

    def timer_callback(self):
        """Publish emergency stop state."""
        msg = Bool()
        msg.data = self.emergency_stop_active
        self.emergency_stop_pub.publish(msg)
        if self.emergency_stop_active:
            self.stop_count += 1

    def activate_stop(self):
        """Activate emergency stop."""
        if not self.emergency_stop_active:
            self.emergency_stop_active = True
            self.stop_count = 0
            self.get_logger().warn('!' * 50)
            self.get_logger().warn('  EMERGENCY STOP ACTIVATED!')
            self.get_logger().warn('  Robot should stop immediately.')
            self.get_logger().warn('  Press ENTER or "r" to resume.')
            self.get_logger().warn('!' * 50)

    def deactivate_stop(self):
        """Deactivate emergency stop."""
        if self.emergency_stop_active:
            self.emergency_stop_active = False
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'  RESUMED - Stop was active for {self.stop_count} cycles')
            self.get_logger().info('  Navigation can now control the robot.')
            self.get_logger().info('=' * 50)

    def get_status(self):
        """Get current status string."""
        if self.emergency_stop_active:
            return f'\r[STOPPED] Vel blocked | Nav wants: vx={self.last_nav_vel.linear.x:.2f} vy={self.last_nav_vel.linear.y:.2f} w={self.last_nav_vel.angular.z:.2f}'
        else:
            return f'\r[RUNNING] Nav vel: vx={self.last_nav_vel.linear.x:.2f} vy={self.last_nav_vel.linear.y:.2f} w={self.last_nav_vel.angular.z:.2f}     '


def get_key(settings, timeout=0.1):
    """Get a single keypress."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = EmergencyStop()

    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('\nEmergency Stop ready. Press SPACE to stop, ENTER to resume, q to quit.\n')

    try:
        while node.running and rclpy.ok():
            key = get_key(settings)

            if key == ' ' or key == 's' or key == 'S':
                node.activate_stop()
            elif key == '\r' or key == '\n' or key == 'r' or key == 'R':
                node.deactivate_stop()
            elif key == 'q' or key == 'Q' or key == '\x03':  # q or Ctrl+C
                print('\nShutting down...')
                node.running = False
                break

            # Show status
            print(node.get_status(), end='', flush=True)

    except Exception as e:
        print(f'\nError: {e}')
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        # Keep the final state visible to nav_safety_gate on shutdown.
        if node.emergency_stop_active:
            stop_msg = Bool()
            stop_msg.data = True
            node.emergency_stop_pub.publish(stop_msg)

        node.destroy_node()
        rclpy.shutdown()
        print('\nEmergency Stop node terminated.')


if __name__ == '__main__':
    main()
