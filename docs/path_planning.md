# 路径规划模块 (Path Planning)

## 概述

路径规划模块负责在全局代价地图上计算从起点到目标点的最优路径。本项目使用 `navigo_navfn_planner` 包实现全局路径规划，基于 NavFn 算法（支持 A* 和 Dijkstra）。

## 架构位置

```
定位输出 (robot pose)
        ↓
┌───────────────────────────────────────┐
│         BT Navigator                   │
│  ┌─────────────────────────────────┐  │
│  │    ComputePathToPose Action     │  │
│  └─────────────────────────────────┘  │
└───────────────────────────────────────┘
        ↓ goal + start pose
┌───────────────────────────────────────┐
│      Planner Server                    │
│  ┌─────────────────────────────────┐  │
│  │    NavFn Planner Plugin         │  │
│  │    (A* / Dijkstra)              │  │
│  └─────────────────────────────────┘  │
└───────────────────────────────────────┘
        ↓ nav_msgs/Path
┌───────────────────────────────────────┐
│      Controller Server                 │
│      (MPPI Controller)                 │
└───────────────────────────────────────┘
```

## 源码结构

```
navigo_navfn_planner/
├── include/navigo_navfn_planner/
│   ├── navfn.hpp              # NavFn 核心算法
│   └── navfn_planner.hpp      # ROS2 Planner 插件接口
├── src/
│   ├── navfn.cpp              # 导航函数计算实现
│   └── navfn_planner.cpp      # Planner 插件实现
└── CMakeLists.txt
```

## 核心算法

### NavFn 算法原理

NavFn (Navigation Function) 是一种基于势场的路径规划算法：

1. **导航函数计算**: 从目标点开始，通过波前传播计算每个栅格到目标的代价
2. **路径回溯**: 从起点开始，沿着代价梯度下降方向回溯到目标点
3. **支持两种搜索策略**:
   - **A***: 使用启发式函数加速搜索，适合大多数场景
   - **Dijkstra**: 完整搜索，保证全局最优，但计算量大

### 关键实现

#### 1. Planner 插件注册

```cpp
// navfn_planner.cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(navigo_navfn_planner::NavfnPlanner, navigo_core::GlobalPlanner)
```

#### 2. 参数配置

```cpp
// navfn_planner.cpp:56-82
void NavfnPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<navigo_costmap_2d::Costmap2DROS> costmap_ros)
{
  // 关键参数
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".tolerance", rclcpp::ParameterValue(0.5));        // 目标容差
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_astar", rclcpp::ParameterValue(true));       // 使用A*
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));   // 允许穿越未知区域
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
}
```

#### 3. 路径计算核心

```cpp
// navfn_planner.cpp:124-180
nav_msgs::msg::Path NavfnPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  // 1. 设置代价地图
  planner_->setNavArr(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());
  planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  // 2. 设置起点和目标点（世界坐标→栅格坐标）
  unsigned int mx, my;
  worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  int map_start[2] = {static_cast<int>(mx), static_cast<int>(my)};
  
  worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  int map_goal[2] = {static_cast<int>(mx), static_cast<int>(my)};

  planner_->setStart(map_goal);  // 注意: NavFn从目标开始计算
  planner_->setGoal(map_start);

  // 3. 执行路径搜索
  if (use_astar_) {
    planner_->calcNavFnAstar();
  } else {
    planner_->calcNavFnDijkstra(true);
  }

  // 4. 提取路径
  float *x, *y;
  int len = planner_->getPathX() && planner_->getPathY();
  // ... 转换为 nav_msgs::msg::Path
}
```

### 代价地图交互

NavFn Planner 使用全局代价地图进行路径规划：

```cpp
// 代价值定义 (costmap_2d)
const unsigned char NO_INFORMATION = 255;   // 未知区域
const unsigned char LETHAL_OBSTACLE = 254;  // 致命障碍物
const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;  // 内切膨胀障碍
const unsigned char FREE_SPACE = 0;         // 自由空间
```

**代价传播规则**:
- 障碍物代价为最大值 (254)
- 距离障碍物越近，代价越高
- 膨胀半径由机器人足迹决定

## 参数配置

### Launch 文件配置

```yaml
# params/planner_server.yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "navigo_navfn_planner/NavfnPlanner"
      tolerance: 0.5              # 目标点容差(米)
      use_astar: true             # 使用A*算法
      allow_unknown: true         # 允许规划穿越未知区域
      use_final_approach_orientation: false
```

### 关键参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `tolerance` | double | 0.5 | 目标点容差，单位米 |
| `use_astar` | bool | true | 使用A*算法（false则用Dijkstra） |
| `allow_unknown` | bool | true | 允许规划路径穿越未知区域 |
| `use_final_approach_orientation` | bool | false | 使用最终接近方向 |

## Planner Server

Planner Server 是路径规划的 ROS2 服务节点：

### Action 接口

```
Action: /compute_path_to_pose
Type: nav2_msgs/action/ComputePathToPose

Goal:
  - geometry_msgs/PoseStamped goal
  - geometry_msgs/PoseStamped start (可选)
  - string planner_id

Result:
  - nav_msgs/Path path
  - builtin_interfaces/Duration planning_time
```

### 生命周期管理

```cpp
// Planner Server 生命周期状态
unconfigured → configure() → inactive
inactive → activate() → active
active → deactivate() → inactive
inactive → cleanup() → unconfigured
```

## 路径输出格式

```cpp
// nav_msgs/msg/Path
struct Path {
  std_msgs::msg::Header header;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
}

// 每个 PoseStamped 包含:
// - header.stamp: 时间戳
// - header.frame_id: 坐标系 (通常是 "map")
// - pose.position: (x, y, z) 位置
// - pose.orientation: 四元数朝向
```

## 与其他模块的交互

### 输入
- **全局代价地图**: 来自 costmap_2d
- **起点**: 当前机器人位姿（从 TF 获取或指定）
- **目标点**: 来自 BT Navigator 或直接 Action 调用

### 输出
- **规划路径**: nav_msgs/Path 发送到 Controller Server

### TF 依赖
- `map` → `odom`: 定位变换
- `odom` → `base_link`: 里程计变换

## 性能考虑

### A* vs Dijkstra

| 特性 | A* | Dijkstra |
|------|-----|----------|
| 搜索效率 | 高（启发式） | 低（全搜索） |
| 路径最优性 | 近似最优 | 全局最优 |
| 适用场景 | 大多数场景 | 需要绝对最优路径 |
| 计算时间 | 快 | 慢 |

### 优化建议

1. **大地图场景**: 使用 A* 算法
2. **复杂障碍物**: 适当增加 tolerance
3. **实时性要求**: 降低代价地图分辨率

## 调试方法

### 1. 可视化路径

```bash
# RViz 中添加 Path 显示
# Topic: /plan
# 或查看 BT Navigator 发布的路径
```

### 2. 查看规划日志

```bash
ros2 run navigo_planner_server planner_server --ros-args --log-level debug
```

### 3. 检查代价地图

```bash
# 查看全局代价地图
ros2 topic echo /global_costmap/costmap
```

## 常见问题

### 1. 规划失败

**可能原因**:
- 起点或目标点在障碍物内
- 起点和目标点之间没有可行路径
- 代价地图未更新

**解决方案**:
- 检查机器人位姿是否准确
- 增加 `tolerance` 参数
- 设置 `allow_unknown: true`

### 2. 路径不平滑

NavFn 生成的原始路径可能有锯齿。可以：
- 在 Controller 中使用路径平滑
- 使用 Smoother Server 后处理

### 3. 规划时间过长

- 减小代价地图尺寸
- 使用 A* 代替 Dijkstra
- 增加代价地图分辨率（粗粒度）
