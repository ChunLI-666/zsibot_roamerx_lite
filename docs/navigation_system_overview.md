# Navigo 导航系统概述

本文档描述了 `zsibot_roamerx_lite` 项目中基于 Gazebo 仿真的完整导航系统架构，包括地图加载、定位、导航和控制模块。

## 1. 系统架构总览

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            Navigo Navigation Stack                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐    ┌───────────┐  │
│  │  Map Server  │───▶│   Costmap    │───▶│   Planner    │───▶│Controller │  │
│  │ (地图加载)    │    │   (代价图)    │    │  (路径规划)    │    │ (MPPI控制)│  │
│  └──────────────┘    └──────────────┘    └──────────────┘    └───────────┘  │
│         │                   │                   │                   │       │
│         ▼                   ▼                   ▼                   ▼       │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐    ┌───────────┐  │
│  │  OccupancyGrid│   │Obstacle Layer│    │  nav_msgs/   │    │ cmd_vel   │  │
│  │    /map       │   │InflationLayer│    │    Path      │    │ 速度命令   │  │
│  └──────────────┘    └──────────────┘    └──────────────┘    └───────────┘  │
│                                                                      │      │
│                                                                      ▼      │
│                          ┌──────────────────────────────────────────────┐   │
│                          │     Velocity Command Publisher (LCM/UDP)     │   │
│                          │           发送到机器人运动控制器                 │   │
│                          └──────────────────────────────────────────────┘   │
│                                                                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                          Behavior Tree Navigator                             │
│                         (行为树协调所有导航动作)                             │
└─────────────────────────────────────────────────────────────────────────────┘
         │
         │ TF Transforms
         ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           TF Manager / Localization                          │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      lightning-lm (激光定位)                         │   │
│  │         Mid360 LiDAR + IMU → LIO → NDT Matching → Pose             │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
│  坐标变换: map → odom → base_link                                           │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 2. 模块说明

### 2.1 地图加载模块 (Map Server)
- **源码位置**: `src/navigation/src/navigo_map_server/`
- **功能**: 加载预建地图并发布为 OccupancyGrid 消息
- **详细文档**: [map_loading.md](./map_loading.md)

### 2.2 定位模块 (Localization)
- **系统**: lightning-lm 激光SLAM/定位系统
- **传感器**: Mid360 LiDAR + IMU
- **功能**: 提供机器人在地图中的精确位姿
- **详细文档**: [localization.md](./localization.md)

### 2.3 导航规划模块 (Navigation Planner)
- **源码位置**: `src/navigation/src/navigo_navfn_planner/`
- **功能**: 全局路径规划 (A*/Dijkstra 算法)
- **详细文档**: [path_planning.md](./path_planning.md)

### 2.4 运动控制模块 (Motion Controller)
- **源码位置**: `src/navigation/src/navigo_mppi_controller/`
- **功能**: MPPI (Model Predictive Path Integral) 局部路径跟踪
- **详细文档**: [motion_control.md](./motion_control.md)

### 2.5 行为树导航器 (Behavior Tree Navigator)
- **源码位置**: `src/navigation/src/navigo_bt_navigator/`
- **功能**: 协调规划、控制和恢复行为
- **详细文档**: [behavior_tree.md](./behavior_tree.md)

## 3. 启动流程

### Gazebo 仿真启动命令:
```bash
# 启动导航栈
ros2 launch robot_navigo navigation_bringup.launch.py \
    platform:=GAZEBO \
    mc_controller_type:=RL_TRACK_VELOCITY \
    communication_type:=LCM \
    map:=/path/to/map/map.yaml

# 启动TF发布器 (Gazebo模式)
ros2 launch pub_tf pub_tf.launch.py tf_type:=gazebo_tf
```

### 启动的节点列表:
| 节点名称 | 包名 | 功能 |
|---------|------|------|
| map_server | navigo_map_server | 地图加载和发布 |
| controller_server | navigo_path_controller | 路径跟踪控制器服务 |
| planner_server | navigo_path_planner | 全局路径规划服务 |
| behavior_server | navigo_behaviors | 恢复行为服务 |
| bt_navigator | navigo_bt_navigator | 行为树导航协调器 |
| waypoint_follower | navigo_waypoint_follower | 多点航迹跟踪 |
| velocity_optimizer | navigo_velocity_optimizer | 速度优化器 |
| vel_cmd_lcm_pub | robot_navigo | LCM速度命令发布 |
| mode_status_pub | robot_navigo | 模式状态发布 |
| tf_manager | pub_tf | TF坐标变换管理 |

## 4. 数据流图

```
                                    用户目标点
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────┐
│                        BT Navigator                              │
│   ┌─────────────────────────────────────────────────────────┐   │
│   │              NavigateToPose Action Server                │   │
│   └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                         │                │
           ComputePathToPose         FollowPath
                         │                │
                         ▼                ▼
┌────────────────────────────┐  ┌────────────────────────────────┐
│     Planner Server         │  │      Controller Server          │
│  ┌──────────────────────┐  │  │  ┌────────────────────────────┐│
│  │   NavFn Planner      │  │  │  │    MPPI Controller         ││
│  │  (A*/Dijkstra)       │  │  │  │  (采样优化控制)             ││
│  └──────────────────────┘  │  │  └────────────────────────────┘│
└────────────────────────────┘  └────────────────────────────────┘
         │                                    │
         │ nav_msgs/Path                      │ geometry_msgs/Twist
         ▼                                    ▼
┌────────────────────────────┐  ┌────────────────────────────────┐
│     Global Costmap         │  │       /cmd_vel Topic            │
│  ┌──────────────────────┐  │  └────────────────────────────────┘
│  │   Static Layer       │  │                    │
│  │   Inflation Layer    │  │                    ▼
│  └──────────────────────┘  │  ┌────────────────────────────────┐
└────────────────────────────┘  │    Vel Cmd LCM/UDP Publisher    │
                                │  ┌────────────────────────────┐│
                                │  │ 转换为LCM/UDP协议发送       ││
                                │  │ 到四足机器人运动控制器      ││
                                │  └────────────────────────────┘│
                                └────────────────────────────────┘
```

## 5. 与 lightning-lm 的集成

本项目设计为与 lightning-lm 激光SLAM系统配合使用：

### 建图流程 (使用 lightning-lm):
1. 运行 lightning-lm SLAM 模式建立地图
2. 保存地图为 PCD 点云 + PGM 栅格图
3. 生成 `map.yaml` 配置文件

### 定位导航流程:
1. 启动 lightning-lm 定位模式 (提供 map→odom→base_link 变换)
2. 启动 Navigo 导航栈 (加载地图并进行路径规划)
3. 发送目标点，行为树协调完成导航任务

详细的集成说明请参考 [integration_with_lightning.md](./integration_with_lightning.md)。

## 6. 关键参数配置

参数文件位置: `src/navigation/src/robot_navigo/params/navigo_params.yaml`

主要参数:
- **MPPI控制器**: batch_size=2000, time_steps=56, vx_max=1.5 m/s
- **全局代价图**: 使用 static_layer + inflation_layer
- **局部代价图**: 8x8m, obstacle_layer + inflation_layer
- **机器人尺寸**: footprint=[[0.30,0.15],[0.30,-0.15],[-0.30,-0.15],[-0.30,0.15]]

## 7. 文档索引

1. [地图加载模块详解](./map_loading.md)
2. [定位系统说明](./localization.md)
3. [路径规划算法](./path_planning.md)
4. [MPPI运动控制](./motion_control.md)
5. [行为树导航](./behavior_tree.md)
6. [与lightning-lm集成](./integration_with_lightning.md)
