# 行为树导航模块 (Behavior Tree Navigation)

## 概述

行为树 (Behavior Tree, BT) 是本导航系统的任务协调核心，负责组织和调度各个导航子任务。本项目使用 `navigo_bt_navigator` 包实现基于 BehaviorTree.CPP v3 的导航协调。

## 架构位置

```
┌─────────────────────────────────────────────────────────────────┐
│                      用户层 / RViz                               │
│              NavigateToPose Goal 发布                            │
└─────────────────────────────────────────────────────────────────┘
                               ↓
┌─────────────────────────────────────────────────────────────────┐
│                    BT Navigator                                  │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │              Behavior Tree Engine                          │  │
│  │        (BehaviorTree.CPP v3 执行引擎)                      │  │
│  │  ┌─────────────────────────────────────────────────────┐  │  │
│  │  │              XML Behavior Tree                       │  │  │
│  │  │   navigate_to_pose_w_replanning_and_recovery.xml    │  │  │
│  │  └─────────────────────────────────────────────────────┘  │  │
│  └───────────────────────────────────────────────────────────┘  │
│                                                                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │ Navigation  │  │  Recovery   │  │    Behavior Plugins     │  │
│  │   Plugins   │  │  Plugins    │  │  (Spin, Wait, BackUp)   │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
            ↓                   ↓                    ↓
    Planner Server      Controller Server     Behavior Server
```

## 源码结构

```
navigo_bt_navigator/
├── include/navigo_bt_navigator/
│   ├── bt_navigator.hpp              # BT Navigator 主类
│   └── navigators/
│       ├── navigate_to_pose.hpp      # NavigateToPose 导航器
│       └── navigate_through_poses.hpp # NavigateThroughPoses 导航器
├── src/
│   ├── main.cpp                      # 节点入口
│   ├── bt_navigator.cpp              # BT Navigator 实现
│   └── navigators/
│       ├── navigate_to_pose.cpp
│       └── navigate_through_poses.cpp
├── behavior_trees/                   # 行为树 XML 定义
│   ├── navigate_to_pose_w_replanning_and_recovery.xml
│   ├── navigate_through_poses_w_replanning_and_recovery.xml
│   └── ...
└── CMakeLists.txt
```

## 行为树基础

### 节点类型

| 类型 | 功能 | 示例 |
|------|------|------|
| **Action** | 执行具体动作 | ComputePathToPose, FollowPath |
| **Condition** | 检查条件 | GoalReached, IsBatteryLow |
| **Control** | 流程控制 | Sequence, Fallback, Parallel |
| **Decorator** | 修饰子节点 | RateController, RecoveryNode |

### 返回状态

- **SUCCESS**: 节点执行成功
- **FAILURE**: 节点执行失败
- **RUNNING**: 节点正在执行中

## 核心行为树

### navigate_to_pose_w_replanning_and_recovery.xml

这是最常用的导航行为树，包含重规划和恢复行为：

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <!-- 主导航流程 -->
      <PipelineSequence name="NavigateWithReplanning">
        <!-- 以 1Hz 频率重规划路径 -->
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" 
                                service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        
        <!-- 跟踪路径 -->
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" 
                              service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      
      <!-- 恢复行为序列 -->
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" 
                                service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" 
                                service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>      <!-- 原地旋转 90° -->
          <Wait wait_duration="5"/>      <!-- 等待 5 秒 -->
          <BackUp backup_dist="0.30" backup_speed="0.05"/>  <!-- 后退 0.3m -->
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

### 执行流程

```
1. RecoveryNode (最多重试6次)
   │
   ├── PipelineSequence (主导航)
   │   ├── RateController (1Hz)
   │   │   └── ComputePathToPose → 调用 Planner Server
   │   │       └── 失败时 ClearGlobalCostmap
   │   │
   │   └── FollowPath → 调用 Controller Server
   │       └── 失败时 ClearLocalCostmap
   │
   └── ReactiveFallback (恢复行为)
       ├── GoalUpdated? → 目标更新则退出恢复
       │
       └── RoundRobin (轮询恢复动作)
           ├── ClearCostmaps → 清除代价地图
           ├── Spin → 原地旋转
           ├── Wait → 等待
           └── BackUp → 后退
```

## BT Navigator 实现

### 主类结构

```cpp
// bt_navigator.cpp
namespace navigo_bt_navigator
{

class BtNavigator : public nav2_util::LifecycleNode
{
public:
  explicit BtNavigator(const rclcpp::NodeOptions & options);

protected:
  // 生命周期回调
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
  // BT 插件管理
  pluginlib::ClassLoader<navigo_core::NavigatorBase> class_loader_;
  std::vector<pluginlib::UniquePtr<navigo_core::NavigatorBase>> navigators_;
  
  // TF 和反馈发布
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace navigo_bt_navigator
```

### Action 接口

```
Action: /navigate_to_pose
Type: nav2_msgs/action/NavigateToPose

Goal:
  - geometry_msgs/PoseStamped pose    # 目标位姿
  - string behavior_tree              # 行为树文件 (可选)

Feedback:
  - geometry_msgs/PoseStamped current_pose
  - builtin_interfaces/Duration navigation_time
  - builtin_interfaces/Duration estimated_time_remaining
  - int16 number_of_recoveries
  - float32 distance_remaining

Result:
  - std_msgs/Empty result
```

## BT 插件节点

### Action 节点

| 节点名 | 功能 | 对应服务/Action |
|--------|------|-----------------|
| ComputePathToPose | 计算路径 | /compute_path_to_pose |
| ComputePathThroughPoses | 计算多点路径 | /compute_path_through_poses |
| FollowPath | 跟踪路径 | /follow_path |
| Spin | 原地旋转 | /spin |
| Wait | 等待 | /wait |
| BackUp | 后退 | /backup |
| DriveOnHeading | 沿航向驾驶 | /drive_on_heading |
| ClearCostmapService | 清除代价地图 | /clear_*_costmap |

### Condition 节点

| 节点名 | 功能 |
|--------|------|
| GoalReached | 检查是否到达目标 |
| GoalUpdated | 检查目标是否更新 |
| IsBatteryLow | 检查电量 |
| IsStuck | 检查是否卡住 |
| TransformAvailable | 检查 TF 是否可用 |

### Control 节点

| 节点名 | 功能 |
|--------|------|
| PipelineSequence | 流水线序列 (子节点可并行启动) |
| RoundRobin | 轮询子节点 |
| RecoveryNode | 恢复节点 (主任务失败时执行恢复) |

### Decorator 节点

| 节点名 | 功能 |
|--------|------|
| RateController | 控制子节点执行频率 |
| DistanceController | 距离控制器 |
| SpeedController | 速度控制器 |
| GoalUpdater | 目标更新器 |

## 参数配置

### bt_navigator.yaml

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10          # BT 循环周期 (ms)
    default_server_timeout: 20    # 服务超时 (s)
    
    # 导航器插件
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "navigo_bt_navigator/NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "navigo_bt_navigator/NavigateThroughPosesNavigator"
    
    # 默认行为树
    default_nav_to_pose_bt_xml: 
      "navigate_to_pose_w_replanning_and_recovery.xml"
    default_nav_through_poses_bt_xml: 
      "navigate_through_poses_w_replanning_and_recovery.xml"
    
    # BT 插件库
    plugin_lib_names:
      - navigo_compute_path_to_pose_action_bt_node
      - navigo_compute_path_through_poses_action_bt_node
      - navigo_follow_path_action_bt_node
      - navigo_spin_action_bt_node
      - navigo_wait_action_bt_node
      - navigo_back_up_action_bt_node
      - navigo_drive_on_heading_bt_node
      - navigo_clear_costmap_service_bt_node
      - navigo_is_stuck_condition_bt_node
      - navigo_goal_reached_condition_bt_node
      - navigo_goal_updated_condition_bt_node
      - navigo_rate_controller_bt_node
      - navigo_distance_controller_bt_node
      - navigo_speed_controller_bt_node
      - navigo_recovery_node_bt_node
      - navigo_pipeline_sequence_bt_node
      - navigo_round_robin_bt_node
      - navigo_transform_available_condition_bt_node
```

## 恢复行为

### Behavior Server

恢复行为由 Behavior Server 执行：

```cpp
// behavior_server.cpp
BehaviorServer::BehaviorServer(const rclcpp::NodeOptions & options)
: LifecycleNode("behavior_server", "", options),
  plugin_loader_("navigo_core", "navigo_core::Behavior"),
  default_ids_{"spin", "backup", "drive_on_heading", "wait"},
  default_types_{
    "navigo_behaviors/Spin",
    "navigo_behaviors/BackUp",
    "navigo_behaviors/DriveOnHeading",
    "navigo_behaviors/Wait"}
{
  // 参数声明
  declare_parameter("costmap_topic", 
    rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
  declare_parameter("footprint_topic", 
    rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));
  declare_parameter("cycle_frequency", rclcpp::ParameterValue(10.0));
  declare_parameter("behavior_plugins", default_ids_);
}
```

### 恢复行为类型

| 行为 | 参数 | 说明 |
|------|------|------|
| Spin | spin_dist (rad) | 原地旋转指定角度 |
| BackUp | backup_dist (m), backup_speed (m/s) | 后退指定距离 |
| Wait | wait_duration (s) | 等待指定时间 |
| DriveOnHeading | dist (m), speed (m/s) | 沿当前航向前进 |

## 自定义行为树

### 创建新的 BT XML

```xml
<!-- my_custom_navigation.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="MainSequence">
      <!-- 自定义节点组合 -->
      <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      <FollowPath path="{path}" controller_id="FollowPath"/>
    </Sequence>
  </BehaviorTree>
</root>
```

### 使用自定义行为树

```bash
# 方法1: 在 Action Goal 中指定
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}}}, \
    behavior_tree: '/path/to/my_custom_navigation.xml'}"

# 方法2: 修改默认配置
# 在 bt_navigator.yaml 中设置 default_nav_to_pose_bt_xml
```

## 调试方法

### 1. BT 状态可视化

使用 Groot 工具查看 BT 执行状态：

```bash
# 安装 Groot
sudo apt install ros-jazzy-groot

# 启动 Groot
ros2 run groot Groot
```

### 2. 日志调试

```bash
# 启动时启用 debug 日志
ros2 run navigo_bt_navigator bt_navigator --ros-args --log-level debug
```

### 3. 查看 Action 状态

```bash
# 查看导航 Action 反馈
ros2 action info /navigate_to_pose
ros2 topic echo /navigate_to_pose/_action/feedback
```

## 常见问题

### 1. 行为树加载失败

**可能原因**:
- XML 文件路径错误
- 缺少所需插件库

**解决方案**:
- 检查 `default_nav_to_pose_bt_xml` 路径
- 确认 `plugin_lib_names` 包含所有需要的插件

### 2. 恢复行为不触发

**可能原因**:
- `number_of_retries` 设置为 0
- 恢复行为服务未启动

**解决方案**:
- 检查 RecoveryNode 的 `number_of_retries` 参数
- 确认 behavior_server 节点运行正常

### 3. 导航超时

**可能原因**:
- 服务响应慢
- 目标不可达

**解决方案**:
- 增加 `default_server_timeout`
- 检查目标点是否在障碍物内

## 与其他模块的关系

```
                    BT Navigator
                         │
         ┌───────────────┼───────────────┐
         ↓               ↓               ↓
   Planner Server  Controller Server  Behavior Server
         │               │               │
         ↓               ↓               ↓
   Global Costmap  Local Costmap    Recovery Actions
```

- **Planner Server**: 提供 ComputePathToPose Action
- **Controller Server**: 提供 FollowPath Action
- **Behavior Server**: 提供 Spin/Wait/BackUp 等 Actions
- **Costmap**: 通过 ClearCostmap 服务清除
