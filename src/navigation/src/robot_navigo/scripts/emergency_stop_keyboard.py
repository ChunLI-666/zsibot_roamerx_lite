#!/usr/bin/python3
"""
Emergency Stop Node with USB Keyboard Support

This node monitors a USB keyboard for emergency stop input.
ANY key press triggers emergency stop, specific keys to resume.

Features:
- Monitors USB keyboard directly via /dev/input/eventX
- Works without terminal (can run as service)
- Any key = STOP (safe default)
- ESC or 'r' = Resume

Usage:
    ros2 run robot_navigo emergency_stop_keyboard.py
    ros2 run robot_navigo emergency_stop_keyboard.py --ros-args -p keyboard_device:=/dev/input/event3

Requirements:
    sudo apt install python3-evdev
    sudo usermod -a -G input $USER  # Add user to input group (then re-login)

Author: Claude Code
"""

import os
import glob
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist

try:
    import evdev
    from evdev import ecodes
    EVDEV_AVAILABLE = True
except ImportError:
    EVDEV_AVAILABLE = False
    print("WARNING: evdev not installed. Run: sudo apt install python3-evdev")


# Key codes for resume (ESC, R, r)
RESUME_KEYS = {
    ecodes.KEY_ESC,
    ecodes.KEY_R,
    ecodes.KEY_ENTER,
} if EVDEV_AVAILABLE else set()

# Key codes to ignore (modifiers, etc.)
IGNORE_KEYS = {
    ecodes.KEY_LEFTSHIFT,
    ecodes.KEY_RIGHTSHIFT,
    ecodes.KEY_LEFTCTRL,
    ecodes.KEY_RIGHTCTRL,
    ecodes.KEY_LEFTALT,
    ecodes.KEY_RIGHTALT,
    ecodes.KEY_LEFTMETA,
    ecodes.KEY_RIGHTMETA,
    ecodes.KEY_CAPSLOCK,
    ecodes.KEY_NUMLOCK,
    ecodes.KEY_SCROLLLOCK,
} if EVDEV_AVAILABLE else set()


def find_keyboard_device():
    """Auto-detect USB keyboard device."""
    if not EVDEV_AVAILABLE:
        return None

    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]

    for device in devices:
        capabilities = device.capabilities()
        # Check if device has KEY events (keyboard)
        if ecodes.EV_KEY in capabilities:
            keys = capabilities[ecodes.EV_KEY]
            # Check for typical keyboard keys (A-Z, space, etc.)
            if ecodes.KEY_A in keys and ecodes.KEY_SPACE in keys:
                print(f"Found keyboard: {device.name} at {device.path}")
                return device.path

    return None


class EmergencyStopKeyboard(Node):
    def __init__(self):
        super().__init__('emergency_stop_keyboard')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('keyboard_device', '')  # Auto-detect if empty
        self.declare_parameter('any_key_stops', True)  # Any key triggers stop

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        keyboard_device = self.get_parameter('keyboard_device').value
        self.any_key_stops = self.get_parameter('any_key_stops').value

        # State
        self.emergency_stop_active = False
        self.running = True
        self.keyboard_connected = False
        self.last_key_time = 0
        self.stop_count = 0

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )

        # Publisher for emergency stop
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            qos
        )

        # Timer for publishing stop command
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # Status timer (print status every second)
        self.status_timer = self.create_timer(1.0, self.status_callback)

        # Find keyboard device
        if keyboard_device:
            self.keyboard_path = keyboard_device
        else:
            self.keyboard_path = find_keyboard_device()

        if not self.keyboard_path:
            self.get_logger().error('=' * 60)
            self.get_logger().error('  NO KEYBOARD FOUND!')
            self.get_logger().error('  Please connect a USB keyboard or specify device path:')
            self.get_logger().error('    --ros-args -p keyboard_device:=/dev/input/eventX')
            self.get_logger().error('  List devices: ls -la /dev/input/event*')
            self.get_logger().error('=' * 60)
        else:
            # Start keyboard monitoring thread
            self.keyboard_thread = threading.Thread(target=self.keyboard_monitor, daemon=True)
            self.keyboard_thread.start()

        self.get_logger().info('=' * 60)
        self.get_logger().info('  EMERGENCY STOP NODE (USB KEYBOARD)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Keyboard: {self.keyboard_path or "NOT FOUND"}')
        self.get_logger().info(f'  Mode: {"Any key = STOP" if self.any_key_stops else "Space = STOP"}')
        self.get_logger().info('  Controls:')
        self.get_logger().info('    ANY KEY  : Emergency STOP')
        self.get_logger().info('    ESC or R : Resume navigation')
        self.get_logger().info('=' * 60)

    def keyboard_monitor(self):
        """Monitor keyboard in separate thread."""
        if not EVDEV_AVAILABLE:
            self.get_logger().error('evdev not available')
            return

        while self.running:
            try:
                device = evdev.InputDevice(self.keyboard_path)
                self.keyboard_connected = True
                self.get_logger().info(f'Keyboard connected: {device.name}')

                for event in device.read_loop():
                    if not self.running:
                        break

                    # Only process key press events (not release)
                    if event.type == ecodes.EV_KEY and event.value == 1:  # 1 = key press
                        key_code = event.code

                        # Ignore modifier keys
                        if key_code in IGNORE_KEYS:
                            continue

                        # Get key name for logging
                        key_name = ecodes.KEY.get(key_code, f'KEY_{key_code}')

                        if key_code in RESUME_KEYS:
                            # Resume key pressed
                            self.get_logger().info(f'Resume key pressed: {key_name}')
                            self.deactivate_stop()
                        else:
                            # Any other key = STOP
                            self.get_logger().warn(f'STOP key pressed: {key_name}')
                            self.activate_stop()

                        self.last_key_time = time.time()

            except (OSError, FileNotFoundError) as e:
                self.keyboard_connected = False
                self.get_logger().warn(f'Keyboard disconnected: {e}')
                time.sleep(1.0)  # Wait before retry
            except Exception as e:
                self.get_logger().error(f'Keyboard error: {e}')
                time.sleep(1.0)

    def timer_callback(self):
        """Publish zero velocity when emergency stop is active."""
        if self.emergency_stop_active:
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            self.stop_count += 1

    def status_callback(self):
        """Print status periodically."""
        if self.emergency_stop_active:
            self.get_logger().warn(f'[STOPPED] Publishing zero velocity (count: {self.stop_count})')
        else:
            status = "connected" if self.keyboard_connected else "DISCONNECTED"
            self.get_logger().info(f'[RUNNING] Keyboard: {status}')

    def activate_stop(self):
        """Activate emergency stop."""
        if not self.emergency_stop_active:
            self.emergency_stop_active = True
            self.stop_count = 0
            self.get_logger().fatal('!' * 60)
            self.get_logger().fatal('  EMERGENCY STOP ACTIVATED!')
            self.get_logger().fatal('  Robot is STOPPED.')
            self.get_logger().fatal('  Press ESC or R to resume.')
            self.get_logger().fatal('!' * 60)

    def deactivate_stop(self):
        """Deactivate emergency stop."""
        if self.emergency_stop_active:
            self.emergency_stop_active = False
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'  RESUMED (was stopped for {self.stop_count} cycles)')
            self.get_logger().info('  Navigation can now control the robot.')
            self.get_logger().info('=' * 60)

    def shutdown(self):
        """Clean shutdown."""
        self.running = False
        # Send final stop
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)


def main(args=None):
    if not EVDEV_AVAILABLE:
        print("ERROR: python3-evdev is required.")
        print("Install with: sudo apt install python3-evdev")
        return 1

    rclpy.init(args=args)
    node = EmergencyStopKeyboard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
