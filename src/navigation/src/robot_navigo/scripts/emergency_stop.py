#!/usr/bin/python3
"""
Emergency Stop Node with Keyboard Control

This node provides:
1. Emergency stop (SPACE or 's') - immediately publishes zero velocity
2. Resume (ENTER or 'r') - allows navigation to resume
3. Velocity monitoring - shows current cmd_vel

Usage:
    ros2 run robot_navigo emergency_stop.py

Controls:
    SPACE / s : Emergency STOP (publish zero velocity continuously)
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


class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate', 50.0)  # Hz

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
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

        # Publisher for emergency stop
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            qos
        )

        # Timer for publishing stop command when active
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
        self.get_logger().info('=' * 50)

    def nav_vel_callback(self, msg: Twist):
        """Monitor navigation velocity commands."""
        self.last_nav_vel = msg

    def timer_callback(self):
        """Publish zero velocity when emergency stop is active."""
        if self.emergency_stop_active:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = 0.0
            stop_msg.angular.x = 0.0
            stop_msg.angular.y = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_msg)
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

        # Send final stop command
        if node.emergency_stop_active:
            stop_msg = Twist()
            node.cmd_vel_pub.publish(stop_msg)

        node.destroy_node()
        rclpy.shutdown()
        print('\nEmergency Stop node terminated.')


if __name__ == '__main__':
    main()
