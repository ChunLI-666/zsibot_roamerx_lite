# Lightning-LM 集成指南

## 概述

本文档描述如何将 lightning-lm 激光 SLAM/定位系统与 navigo 导航栈集成，实现完整的「建图 → 定位 → 导航规控」流程。

## 系统架构

```
┌────────────────────────────────────────────────────────────────────────┐
│                           传感器层                                      │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────────────┐   │
│  │  Mid360      │  │    IMU       │  │      其他传感器             │   │
│  │  LiDAR       │  │              │  │   (深度相机等)              │   │
│  └──────────────┘  └──────────────┘  └────────────────────────────┘   │
│         │                 │                                            │
│         └────────┬────────┘                                            │
│                  ↓                                                     │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │                    Lightning-LM                                   │ │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────────────────────┐ │ │
│  │  │ LIO 前端   │→│ 回环检测   │→│   位姿图优化               │ │ │
│  │  │ (ESKF+IVox)│  │ (NDT)      │  │   (miao/g2o)              │ │ │
│  │  └────────────┘  └────────────┘  └────────────────────────────┘ │ │
│  │         ↓                                    ↓                   │ │
│  │  ┌────────────┐                    ┌────────────────────────┐   │ │
│  │  │ 里程计输出 │                    │   地图管理 (分块)      │   │ │
│  │  │ /odom      │                    │   + G2P5 栅格地图生成  │   │ │
│  │  └────────────┘                    └────────────────────────┘   │ │
│  └──────────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────────┘
                  ↓                                    ↓
           TF 发布 (odom→base_link)           地图文件 (.pcd + .pgm)
                  │                                    │
                  ↓                                    ↓
┌────────────────────────────────────────────────────────────────────────┐
│                        导航层 (Navigo Stack)                           │
│  ┌────────────────────────────────────────────────────────────────┐   │
│  │                      TF Manager                                 │   │
│  │              (map→odom 变换 from 定位)                          │   │
│  └────────────────────────────────────────────────────────────────┘   │
│         ↓                              ↓                               │
│  ┌──────────────┐              ┌──────────────────────────────────┐   │
│  │  Map Server  │              │      Costmap 2D                  │   │
│  │  (加载.pgm)  │              │  (global + local costmap)        │   │
│  └──────────────┘              └──────────────────────────────────┘   │
│         ↓                                    ↓                         │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │                    BT Navigator                                   │ │
│  │     ┌──────────────┐            ┌──────────────────────────┐     │ │
│  │     │ NavFn Planner│            │   MPPI Controller        │     │ │
│  │     │ (全局路径)   │            │   (局部跟踪)             │     │ │
│  │     └──────────────┘            └──────────────────────────┘     │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│                              ↓                                         │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │                Velocity Command Publisher                         │ │
│  │                  (LCM/UDP → 底层控制)                             │ │
│  └──────────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────────┘
```

## 工作流程

### 阶段一：建图 (SLAM)

1. **启动 lightning-lm SLAM 模式**
   ```bash
   # 在线建图 (配合 bag 播放或实时数据)
   ros2 run lightning run_slam_online --config ./config/default_mid360.yaml
   
   # 或离线建图 (直接处理 bag 文件)
   ros2 run lightning run_slam_offline --config ./config/default_mid360.yaml \
     --input_bag /path/to/recorded.bag
   ```

2. **配置文件关键参数** (`config/default_mid360.yaml`)
   ```yaml
   common:
     lidar_topic: /livox/lidar             # Mid360 点云话题
     imu_topic: /livox/imu                 # IMU 话题
   
   fasterlio:
     lidar_type: 1                         # 1=Livox
     time_scale: 1.0                       # 时间戳缩放
     point_filter_num: 3                   # 点云采样
     ivox_grid_resolution: 0.5             # 体素分辨率
   
   system:
     with_loop_closing: true               # 启用回环检测
     with_g2p5: true                       # 启用栅格地图生成
     map_path: ./data/new_map/             # 地图保存路径
   
   loop_closing:
     loop_kf_gap: 10                       # 回环检测间隔
     closest_id_th: 30                     # 最小关键帧间隔
     max_range: 15.0                       # 回环检测范围
   ```

3. **保存地图**
   ```bash
   ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: my_map}"
   ```

4. **输出文件**
   ```
   data/new_map/
   ├── global.pcd        # 点云地图 (可视化用)
   ├── map.pgm           # 2D 栅格地图 (导航用)
   ├── map.yaml          # 地图元数据
   └── chunks/           # 分块点云 (定位用)
   ```

### 阶段二：定位

1. **启动 lightning-lm 定位模式**
   ```bash
   ros2 run lightning run_loc_online --config ./config/default_mid360.yaml
   ```

2. **定位配置参数**
   ```yaml
   lidar_loc:
     force_2d: false                       # 是否强制2D定位
     init_with_fp: true                    # 使用特征点初始化
     update_kf_dis: 1.0                    # 动态层更新距离
   
   maps:
     load_map_size: 3                      # 加载周围地图块数
     dyn_cloud_policy: short               # 动态层策略
   ```

3. **输出**
   - TF 变换: `odom` → `base_link` (高频, 50-100Hz)
   - 里程计: `/odom` (nav_msgs/Odometry)
   - 定位位姿: `/localization_pose` (geometry_msgs/PoseStamped)

### 阶段三：导航

1. **TF 树配置**
   
   完整的 TF 树应为:
   ```
   map
    └── odom (由 tf_manager 或 lightning-lm 发布)
         └── base_link (由 lightning-lm 里程计发布)
              └── sensor_frames...
   ```

2. **启动导航栈**
   ```bash
   # 启动完整导航
   ros2 launch robot_navigo navigation_bringup.launch.py \
     map:=/path/to/map.yaml \
     use_sim_time:=false
   ```

3. **关键配置**

   **map_server 配置** (加载 lightning-lm 生成的地图):
   ```yaml
   map_server:
     ros__parameters:
       yaml_filename: "/path/to/data/new_map/map.yaml"
       frame_id: "map"
   ```

   **costmap 配置** (使用激光雷达更新):
   ```yaml
   local_costmap:
     ros__parameters:
       global_frame: odom
       robot_base_frame: base_link
       update_frequency: 5.0
       publish_frequency: 2.0
       
       plugins: ["obstacle_layer", "inflation_layer"]
       obstacle_layer:
         plugin: "navigo_costmap_2d::ObstacleLayer"
         observation_sources: scan
         scan:
           topic: /livox/scan                    # Mid360 2D 扫描
           data_type: "LaserScan"
           clearing: true
           marking: true
   ```

## 接口对接

### 1. TF 变换对接

Lightning-lm 发布的变换:
- `odom` → `base_link`: 里程计变换 (高频)

需要补充的变换 (通过 tf_manager 或外部节点):
- `map` → `odom`: 全局定位变换

```cpp
// tf_manager 或自定义节点发布 map→odom
geometry_msgs::msg::TransformStamped t;
t.header.stamp = this->now();
t.header.frame_id = "map";
t.child_frame_id = "odom";
t.transform = compute_map_to_odom();  // 从定位结果计算
tf_broadcaster_->sendTransform(t);
```

### 2. 里程计对接

Lightning-lm 发布 `/odom` 话题，导航栈订阅:

```yaml
# bt_navigator.yaml
bt_navigator:
  ros__parameters:
    odom_topic: /odom
    global_frame: map
    robot_base_frame: base_link
```

### 3. 地图对接

Lightning-lm 生成的 `map.pgm` 和 `map.yaml` 可直接被 map_server 加载:

```yaml
# map.yaml (lightning-lm 生成)
image: map.pgm
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## 启动顺序

### 推荐启动顺序

```bash
# Terminal 1: 启动 lightning-lm 定位
ros2 run lightning run_loc_online --config ./config/default_mid360.yaml

# Terminal 2: 启动导航栈
ros2 launch robot_navigo navigation_bringup.launch.py \
  map:=/path/to/map.yaml \
  use_sim_time:=false

# Terminal 3: 启动 RViz 可视化
ros2 launch robot_navigo display.launch.py

# Terminal 4: 发送导航目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0}}}}"
```

### 使用单一 Launch 文件

创建综合 launch 文件 `full_navigation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Lightning-lm 定位节点
    lightning_loc = ExecuteProcess(
        cmd=['ros2', 'run', 'lightning', 'run_loc_online',
             '--config', '/path/to/config.yaml'],
        output='screen'
    )
    
    # 导航栈
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('robot_navigo'),
            '/launch/navigation_bringup.launch.py'
        ]),
        launch_arguments={
            'map': '/path/to/map.yaml',
            'use_sim_time': 'false'
        }.items()
    )
    
    return LaunchDescription([
        lightning_loc,
        navigation_launch,
    ])
```

## 调试检查清单

### 1. 检查 TF 树

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 检查 map→odom→base_link 是否完整
ros2 run tf2_ros tf2_echo map base_link
```

### 2. 检查话题

```bash
# 里程计
ros2 topic echo /odom --once

# 地图
ros2 topic echo /map --once

# 代价地图
ros2 topic echo /local_costmap/costmap --once

# 速度指令
ros2 topic echo /cmd_vel
```

### 3. 检查服务

```bash
# Planner
ros2 action info /compute_path_to_pose

# Controller
ros2 action info /follow_path

# Navigator
ros2 action info /navigate_to_pose
```

## 常见问题

### 1. TF 变换超时

**症状**: `Could not transform... timeout`

**原因**: lightning-lm 和导航栈时间不同步

**解决**:
- 确保 `use_sim_time` 参数一致
- 检查时钟源
- 参考 CLAUDE.md 中的时间戳问题说明

### 2. 代价地图不更新

**症状**: 障碍物不显示或不消失

**原因**: 激光雷达话题配置错误

**解决**:
```yaml
obstacle_layer:
  observation_sources: scan
  scan:
    topic: /livox/scan           # 确认话题名正确
    data_type: "LaserScan"       # 或 "PointCloud2"
```

### 3. 定位跳变

**症状**: 机器人位置突然跳动

**原因**: 定位初始化失败或回环检测误匹配

**解决**:
- 检查 lightning-lm 初始化是否成功
- 调整定位参数
- 确保地图质量良好

### 4. 路径规划失败

**症状**: `Failed to get a plan`

**原因**: 机器人或目标点在障碍物内

**解决**:
- 检查机器人位姿是否准确
- 确认地图正确加载
- 增加 planner 的 `tolerance` 参数

## 性能调优

### Lightning-lm 优化

```yaml
# 降低计算负载
fasterlio:
  point_filter_num: 5              # 增加采样间隔
  max_iteration: 4                 # 减少ICP迭代

# 定位优化
lidar_loc:
  force_2d: true                   # 2D 定位更稳定
```

### 导航栈优化

```yaml
# 降低代价地图更新频率
local_costmap:
  update_frequency: 3.0            # 从 5.0 降低
  publish_frequency: 1.0           # 从 2.0 降低

# 减少 MPPI 采样数
FollowPath:
  batch_size: 1000                 # 从 2000 降低
  time_steps: 40                   # 从 56 降低
```

## 参考资源

- [Lightning-LM README](/home/charles/project/colcon_ws/src/lightning-lm/README.md)
- [CLAUDE.md 项目说明](/home/charles/project/colcon_ws/CLAUDE.md)
- [Nav2 官方文档](https://navigation.ros.org/)
- [BehaviorTree.CPP 文档](https://www.behaviortree.dev/)
