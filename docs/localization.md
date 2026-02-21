# 定位系统说明 (Localization)

## 1. 概述

本项目的定位模块使用 **lightning-lm** 激光SLAM/定位系统，基于 Mid360 LiDAR 和 IMU 实现高精度定位。

## 2. 定位系统架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        lightning-lm 定位系统                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                     传感器输入                                    │  │
│  │  ┌─────────────┐              ┌─────────────┐                    │  │
│  │  │  Mid360     │              │    IMU      │                    │  │
│  │  │   LiDAR     │              │ (100-200Hz) │                    │  │
│  │  │  (10Hz)     │              │             │                    │  │
│  │  └──────┬──────┘              └──────┬──────┘                    │  │
│  └─────────┼────────────────────────────┼───────────────────────────┘  │
│            │                            │                               │
│            ▼                            ▼                               │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                    LIO 前端 (FasterLIO)                           │  │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐          │  │
│  │  │ 点云预处理  │───▶│    ESKF     │───▶│   IVox3D    │          │  │
│  │  │(去畸变/降采样)│   │(误差状态卡尔曼)│   │(增量体素)   │          │  │
│  │  └─────────────┘    └─────────────┘    └─────────────┘          │  │
│  └──────────────────────────────────────────────────────────────────┘  │
│                              │                                          │
│                              ▼                                          │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                    定位后端                                       │  │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐          │  │
│  │  │  NDT匹配    │───▶│  位姿图优化  │───▶│  平滑输出   │          │  │
│  │  │ (全局地图)  │    │    (PGO)    │    │ (50-100Hz) │          │  │
│  │  └─────────────┘    └─────────────┘    └─────────────┘          │  │
│  └──────────────────────────────────────────────────────────────────┘  │
│                              │                                          │
│                              ▼                                          │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                    输出                                           │  │
│  │  ┌─────────────────────────────────────────────────────────────┐│  │
│  │  │           TF: map → odom → base_link                        ││  │
│  │  │           Odometry: /odom                                    ││  │
│  │  └─────────────────────────────────────────────────────────────┘│  │
│  └──────────────────────────────────────────────────────────────────┘  │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

## 3. lightning-lm 核心模块

### 3.1 LIO 前端 (Laser-Inertial Odometry)
- **位置**: `lightning-lm/src/core/lio/`
- **算法**: AA-FasterLIO (Anderson Acceleration 加速)
- **功能**: 
  - 点云去畸变
  - IMU预积分
  - ESKF (Error-State Kalman Filter) 状态估计
  - IVox3D 增量体素结构用于快速最近邻搜索

### 3.2 NDT定位
- **位置**: `lightning-lm/src/core/localization/`
- **算法**: NDT (Normal Distribution Transform)
- **功能**:
  - 当前帧与预建全局地图匹配
  - 动态/静态层分离
  - 分块地图动态加载

### 3.3 位姿图优化
- **位置**: `lightning-lm/src/core/miao/`
- **算法**: 增量式图优化
- **功能**: 融合里程计和NDT匹配结果

## 4. TF坐标变换

导航系统依赖以下TF变换链：

```
map
 │
 │  (定位模块发布 - lightning-lm)
 ▼
odom
 │
 │  (里程计发布 - LIO或仿真)
 ▼
base_link
 │
 │  (URDF定义的静态变换)
 ▼
各传感器坐标系 (lidar_link, imu_link, etc.)
```

### 4.1 Gazebo仿真模式下的TF

在Gazebo仿真中，使用 `tf_manager` 节点管理TF变换：

```yaml
# pub_tf/config/config.yaml
tf_manager:
  ros__parameters:
    input_gazebo_pose_topic: "/odom/gazebo"    # Gazebo真值里程计
    output_pose_topic: "/odom/ground_truth"
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
```

### 4.2 实机运行模式下的TF

实机运行时，lightning-lm 直接发布TF变换：
- `map → odom`: 定位校正量
- `odom → base_link`: LIO里程计

## 5. 与 Navigo 导航栈的集成

### 5.1 定位输出供导航使用

```
┌──────────────────┐
│   lightning-lm   │
│    定位模块      │
└────────┬─────────┘
         │
         │ TF: map→odom→base_link
         │ Topic: /odom
         ▼
┌──────────────────────────────────────────────────────┐
│                    Navigo 导航栈                      │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  │
│  │  Costmap    │  │  Planner    │  │ Controller  │  │
│  │ (需要TF查询) │  │ (需要当前位姿)│  │ (需要里程计) │  │
│  └─────────────┘  └─────────────┘  └─────────────┘  │
└──────────────────────────────────────────────────────┘
```

### 5.2 关键参数配置

```yaml
# navigo_params.yaml
bt_navigator:
  ros__parameters:
    global_frame: map           # 全局坐标系
    robot_base_frame: base_link # 机器人底盘坐标系
    odom_topic: /odom           # 里程计话题
    transform_tolerance: 0.1    # TF变换容忍度(秒)

local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom        # 局部代价图使用odom坐标系
      robot_base_frame: base_link

global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map         # 全局代价图使用map坐标系
      robot_base_frame: base_link
```

## 6. 建图与定位流程

### 6.1 建图流程 (使用 lightning-lm SLAM 模式)

```bash
# 1. 启动SLAM (在线建图)
ros2 run lightning run_slam_online --config ./config/default.yaml

# 2. 播放数据包或实时运行
ros2 bag play recording.bag

# 3. 保存地图
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: my_map}"
```

建图输出文件：
```
data/my_map/
├── global.pcd        # 全局点云 (可视化用)
├── map.pgm           # 2D栅格图 (导航用)
├── map.yaml          # 地图配置文件
├── chunks/           # 分块点云地图
└── keyframes/        # 关键帧数据
```

### 6.2 定位导航流程

```bash
# 1. 启动 lightning-lm 定位模式
ros2 run lightning run_loc_online --config ./config/default.yaml

# 2. 启动 Navigo 导航栈
ros2 launch robot_navigo navigation_bringup.launch.py \
    platform:=REAL \
    map:=/path/to/my_map/map.yaml

# 3. 发送导航目标
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "..."
```

## 7. lightning-lm 关键配置参数

```yaml
# config/default.yaml

# 传感器配置
common:
  lidar_topic: /livox/lidar        # LiDAR话题
  imu_topic: /livox/imu            # IMU话题

# LIO前端参数
fasterlio:
  lidar_type: 1                    # 1=Livox, 2=Velodyne, 3=Ouster
  point_filter_num: 3              # 点云降采样
  ivox_grid_resolution: 0.5        # IVox体素分辨率
  max_iteration: 6                 # ICP最大迭代次数
  use_aa: true                     # 启用Anderson加速

# 定位参数
lidar_loc:
  force_2d: false                  # 强制2D定位
  init_with_fp: true               # 使用特征点初始化
  update_kf_dis: 0.5               # 动态层更新距离

# 地图参数
maps:
  load_map_size: 3                 # 加载相邻地图块数量
  
# 系统功能
system:
  with_loop_closing: false         # 定位模式关闭回环
  with_ui: false                   # 关闭可视化
  with_g2p5: false                 # 定位模式不需要栅格图
  map_path: ./data/my_map/         # 地图路径
```

## 8. 故障排查

### 8.1 TF变换问题
```bash
# 检查TF树是否完整
ros2 run tf2_tools view_frames

# 查看特定变换
ros2 run tf2_ros tf2_echo map base_link
```

### 8.2 定位漂移问题
- 检查IMU标定参数
- 确认激光雷达外参
- 调整NDT匹配参数

### 8.3 定位初始化失败
- 确保机器人在已建图区域内
- 尝试手动设置初始位姿
- 检查地图加载是否正确

## 9. 参考

- lightning-lm 详细文档: `/colcon_ws/CLAUDE.md`
- lightning-lm 配置示例: `/colcon_ws/src/lightning-lm/config/`
