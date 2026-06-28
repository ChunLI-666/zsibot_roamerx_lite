# cmd_vel_to_zsibot

ROS2 node that converts `geometry_msgs/Twist` velocity commands to local `yz-robot-ctrl` UDP commands.

## Overview

This package provides a bridge between the standard ROS2 navigation stack (which publishes `cmd_vel` messages) and the local `yz-robot-ctrl` service. The service owns the ZsiBot SDK connection; this node only sends UDP command payloads to it.

## Features

- Subscribes to standard `geometry_msgs/Twist` messages
- Configurable velocity limits
- Command timeout safety (sends zero velocity if no command received)
- Enable/disable functionality via topic
- Connection status publishing
- Event-driven UDP commands aligned with `yz_robot_ctrl`
- Low-rate command checking and status publishing
- Outgoing command debug topic
- Auto-standup disabled for safety

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `cmd_vel` | `geometry_msgs/Twist` | Velocity commands (configurable topic name) |
| `~/enable` | `std_msgs/Bool` | Enable/disable the bridge |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~/connected` | `std_msgs/Bool` | Robot connection status |
| `~/sent_command` | `robots_dog_msgs/CmdVelToZsibotDebug` | Actual command record sent to `yz-robot-ctrl` |
| `~/debug_command` | `robots_dog_msgs/CmdVelToZsibotDebug` | Structured debug record with reason, input cmd_vel, normalized command, payload, and send result |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `local_ip` | string | "192.168.168.2" | Local IP for UDP communication |
| `local_port` | int | 43988 | Local UDP port |
| `robot_ip` | string | "192.168.168.168" | ZsiBot robot IP address |
| `output_mode` | string | "udp" | `udp` sends to `yz-robot-ctrl`; `fake` only publishes debug topics; SDK direct control is disabled |
| `control_host` | string | "127.0.0.1" | Local `yz-robot-ctrl` UDP host |
| `control_port` | int | 6002 | Local `yz-robot-ctrl` algorithm UDP port |
| `max_linear_x` | double | 0.15 | Max forward/backward velocity (m/s) |
| `max_linear_y` | double | 0.15 | Max lateral velocity (m/s) |
| `max_angular_z` | double | 0.1 | Max angular velocity (rad/s) |
| `cmd_timeout` | double | 0.5 | Command timeout in seconds |
| `command_check_rate` | double | 20.0 | Rate to check pending commands and timeout state (Hz) |
| `status_rate` | double | 1.0 | Battery/connectivity polling rate (Hz) |
| `min_command_interval` | double | 0.1 | Non-zero command refresh interval; keep below `yz-robot-ctrl` watchdog |
| `command_epsilon` | double | 0.001 | Velocity delta required before resending `move()` |
| `cmd_vel_topic` | string | "cmd_vel" | Input velocity command topic |
| `auto_standup` | bool | false | Disabled by default; stand up must be explicit |

## Usage

### Build

```bash
cd ~/colcon_ws
colcon build --packages-select cmd_vel_to_zsibot
source install/setup.bash
```

### Run

**Using launch file:**
```bash
ros2 launch cmd_vel_to_zsibot cmd_vel_to_zsibot.launch.py
```

**With custom parameters:**
```bash
ros2 launch cmd_vel_to_zsibot cmd_vel_to_zsibot.launch.py \
    robot_ip:=192.168.168.168 \
    cmd_vel_topic:=/nav2/cmd_vel
```

**Using executable directly:**
```bash
ros2 run cmd_vel_to_zsibot cmd_vel_to_zsibot_node \
    --ros-args -p robot_ip:=192.168.168.168
```

### Test with teleop

```bash
# Terminal 1: Start the bridge
ros2 launch cmd_vel_to_zsibot cmd_vel_to_zsibot.launch.py

# Terminal 2: Use teleop keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Velocity Mapping

The node maps ROS2 Twist messages to ZsiBot SDK as follows:

| Twist Field | ZsiBot SDK | Description |
|-------------|------------|-------------|
| `linear.x` | `vx` | Forward velocity (positive = forward) |
| `linear.y` | `vy` | Lateral velocity (positive = left) |
| `angular.z` | `yaw_rate` | Angular velocity (positive = counter-clockwise) |

## Safety Features

1. **Command Timeout**: If no `cmd_vel` message is received within `cmd_timeout` seconds, zero velocity is sent to the robot.

2. **Velocity Clamping**: All velocity commands are clamped to the configured maximum values.

3. **Enable/Disable**: The bridge can be disabled via the `~/enable` topic, which immediately sends zero velocity.

4. **Graceful Shutdown**: Zero velocity is sent when the node is terminated.

## Dependencies

- ROS2 Humble
- geometry_msgs
- std_msgs
- rclcpp
- rclcpp_components
- zsibot_sdk (mc_sdk)

## License

Apache-2.0
