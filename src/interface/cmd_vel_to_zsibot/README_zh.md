# cmd_vel_to_zsibot

将 ROS2 标准速度命令 (`geometry_msgs/Twist`) 转换为本机 `yz-robot-ctrl` UDP 控制指令的桥接节点。

## 概述

本包提供了 ROS2 导航栈（发布 `cmd_vel` 消息）与本机 `yz-robot-ctrl` 服务之间的桥接功能。`yz-robot-ctrl` 独占 ZsiBot SDK 连接；本节点只向它发送 UDP 控制 payload。

## 功能特性

- 订阅标准 `geometry_msgs/Twist` 消息
- 可配置的速度限制
- 命令超时安全机制（未收到命令时发送零速度）
- 通过话题启用/禁用功能
- 发布连接状态
- 与 `yz_robot_ctrl` 对齐的事件驱动 UDP 命令
- 低频命令检查和连接状态发布
- 发布实际下发命令的调试话题
- 出于安全考虑禁用自动站立

## 话题

### 订阅的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `cmd_vel` | `geometry_msgs/Twist` | 速度命令（话题名称可配置） |
| `~/enable` | `std_msgs/Bool` | 启用/禁用桥接 |

### 发布的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `~/connected` | `std_msgs/Bool` | 机器人连接状态 |
| `~/sent_command` | `std_msgs/String` | 实际发送到 `yz-robot-ctrl` 的 UDP payload |
| `~/debug_command` | `std_msgs/String` | JSON 调试记录，包含 reason、输入 cmd_vel、归一化命令、payload 和发送结果 |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `local_ip` | string | "192.168.168.2" | 本地 UDP 通信 IP |
| `local_port` | int | 43988 | 本地 UDP 端口 |
| `robot_ip` | string | "192.168.168.168" | ZsiBot 机器人 IP 地址 |
| `output_mode` | string | "udp" | `udp` 发送到 `yz-robot-ctrl`；`fake` 只发布调试话题；SDK 直连控制已禁用 |
| `control_host` | string | "127.0.0.1" | 本机 `yz-robot-ctrl` UDP 地址 |
| `control_port` | int | 6000 | 本机 `yz-robot-ctrl` UDP 端口 |
| `max_linear_x` | double | 0.15 | 最大前后速度 (m/s) |
| `max_linear_y` | double | 0.15 | 最大横向速度 (m/s) |
| `max_angular_z` | double | 0.25 | 最大角速度 (rad/s) |
| `cmd_timeout` | double | 0.5 | 命令超时时间（秒） |
| `command_check_rate` | double | 20.0 | 检查待发送命令和超时状态的频率 (Hz) |
| `status_rate` | double | 1.0 | 电量/连接状态轮询频率 (Hz) |
| `min_command_interval` | double | 0.1 | 非零命令刷新间隔；必须小于 `yz-robot-ctrl` 看门狗 |
| `command_epsilon` | double | 0.001 | 重新发送 `move()` 所需的速度变化阈值 |
| `cmd_vel_topic` | string | "cmd_vel" | 输入速度命令话题 |
| `auto_standup` | bool | false | 默认禁用；站立必须显式触发 |

## 使用方法

### 编译

```bash
cd ~/colcon_ws/src/zsibot/zsibot_roamerx_lite
colcon build --packages-select cmd_vel_to_zsibot
source install/setup.bash
```

### 运行

**使用 launch 文件：**
```bash
ros2 launch cmd_vel_to_zsibot cmd_vel_to_zsibot.launch.py
```

**使用自定义参数：**
```bash
ros2 launch cmd_vel_to_zsibot cmd_vel_to_zsibot.launch.py \
    robot_ip:=192.168.168.168 \
    cmd_vel_topic:=/nav2/cmd_vel
```

**直接运行可执行文件：**
```bash
ros2 run cmd_vel_to_zsibot cmd_vel_to_zsibot_node \
    --ros-args -p robot_ip:=192.168.168.168
```

### 配合键盘遥控测试

```bash
# 终端 1: 启动桥接节点
ros2 launch cmd_vel_to_zsibot cmd_vel_to_zsibot.launch.py

# 终端 2: 使用键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 速度映射

节点将 ROS2 Twist 消息映射到 ZsiBot SDK：

| Twist 字段 | ZsiBot SDK | 说明 |
|------------|------------|------|
| `linear.x` | `vx` | 前进速度（正值 = 前进） |
| `linear.y` | `vy` | 横向速度（正值 = 左移） |
| `angular.z` | `yaw_rate` | 角速度（正值 = 逆时针旋转） |

## 安全特性

1. **命令超时**：如果在 `cmd_timeout` 秒内未收到 `cmd_vel` 消息，将向机器人发送零速度。

2. **速度限幅**：所有速度命令都会被限制在配置的最大值范围内。

3. **启用/禁用**：可通过 `~/enable` 话题禁用桥接，禁用时立即发送零速度。

4. **优雅关闭**：节点终止时会发送零速度。

## 依赖

- ROS2 (Humble/Jazzy)
- geometry_msgs
- std_msgs
- rclcpp
- rclcpp_components
- zsibot_sdk (mc_sdk)

## 许可证

Apache-2.0
