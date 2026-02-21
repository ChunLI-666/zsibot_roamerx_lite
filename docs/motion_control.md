# 运动控制模块 (Motion Control)

## 概述

运动控制模块负责将全局路径转换为机器人可执行的速度指令。本项目使用 **MPPI (Model Predictive Path Integral)** 控制器，这是一种基于采样的模型预测控制方法，特别适合非线性系统和复杂约束场景。

## 架构位置

```
┌─────────────────────────────────────────────────────────────┐
│                   BT Navigator                               │
│              FollowPath Action 调用                          │
└─────────────────────────────────────────────────────────────┘
                            ↓ nav_msgs/Path
┌─────────────────────────────────────────────────────────────┐
│                  Controller Server                           │
│  ┌───────────────────────────────────────────────────────┐  │
│  │              MPPI Controller Plugin                    │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐  │  │
│  │  │ Path Handler│  │  Optimizer  │  │ Trajectory   │  │  │
│  │  │             │  │             │  │ Visualizer   │  │  │
│  │  └─────────────┘  └─────────────┘  └──────────────┘  │  │
│  │         ↓               ↓                             │  │
│  │  ┌─────────────────────────────────────────────────┐  │  │
│  │  │            Critic Manager                        │  │  │
│  │  │  ┌────────┐ ┌────────┐ ┌────────┐ ┌─────────┐  │  │  │
│  │  │  │Obstacle│ │ Goal   │ │ Path   │ │Constraint│  │  │  │
│  │  │  │ Critic │ │ Critic │ │Align   │ │ Critic  │  │  │  │
│  │  │  └────────┘ └────────┘ └────────┘ └─────────┘  │  │  │
│  │  └─────────────────────────────────────────────────┘  │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            ↓ geometry_msgs/Twist
┌─────────────────────────────────────────────────────────────┐
│              Velocity Command Publisher                      │
│        (vel_cmd_lcm_pub / vel_cmd_udp_pub)                  │
└─────────────────────────────────────────────────────────────┘
                            ↓ LCM/UDP
┌─────────────────────────────────────────────────────────────┐
│              Robot Motion Controller                         │
│                  (四足机器人底层)                            │
└─────────────────────────────────────────────────────────────┘
```

## 源码结构

```
navigo_mppi_controller/
├── include/navigo_mppi_controller/
│   ├── controller.hpp           # 控制器主类
│   ├── optimizer.hpp            # MPPI 优化器
│   ├── path_handler.hpp         # 路径处理
│   ├── trajectory_visualizer.hpp # 轨迹可视化
│   ├── critic_manager.hpp       # 评价函数管理
│   ├── parameters_handler.hpp   # 参数处理
│   ├── noise_generator.hpp      # 噪声生成器
│   ├── motion_models.hpp        # 运动模型
│   ├── critic_data.hpp          # 评价数据结构
│   └── critics/                 # 各种评价函数
│       ├── obstacles_critic.hpp
│       ├── goal_critic.hpp
│       ├── path_align_critic.hpp
│       └── ...
├── src/
│   ├── controller.cpp
│   ├── optimizer.cpp
│   ├── critic_manager.cpp
│   ├── trajectory_visualizer.cpp
│   ├── path_handler.cpp
│   ├── parameters_handler.cpp
│   ├── noise_generator.cpp
│   └── critics/
│       ├── obstacles_critic.cpp
│       ├── goal_critic.cpp
│       └── ...
└── CMakeLists.txt
```

## MPPI 算法原理

### 核心思想

MPPI 是一种基于采样的最优控制方法：

1. **采样**: 在当前控制序列基础上添加噪声，生成大量候选轨迹
2. **评估**: 使用多个评价函数计算每条轨迹的代价
3. **加权平均**: 使用 softmax 对轨迹进行加权，代价低的轨迹权重高
4. **更新**: 加权平均得到新的控制序列

### 数学表达

控制更新公式：
```
u_new = Σ(w_i * u_i)
其中 w_i = exp(-cost_i / λ) / Σexp(-cost_j / λ)
```

- `u_i`: 第 i 条采样轨迹的控制序列
- `cost_i`: 第 i 条轨迹的总代价
- `λ`: 温度参数（控制探索程度）

## 核心实现

### 1. Controller 主类

```cpp
// controller.cpp:67-120
void MPPIController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<navigo_costmap_2d::Costmap2DROS> costmap_ros)
{
  // 初始化组件
  path_handler_ = std::make_unique<PathHandler>(...);
  trajectory_visualizer_ = std::make_unique<TrajectoryVisualizer>(...);
  optimizer_ = std::make_unique<Optimizer>(...);
}

// controller.cpp:150-180
geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  // 1. 处理路径
  auto transformed_plan = path_handler_->transformPath(robot_pose);
  
  // 2. 调用优化器计算控制
  auto cmd = optimizer_.evalControl(
    robot_pose, robot_speed, transformed_plan, goal, goal_checker);
  
  // 3. 可视化
  trajectory_visualizer_->add(optimizer_.getGeneratedTrajectories());
  
  return cmd;
}
```

### 2. Optimizer 优化器

```cpp
// optimizer.cpp - 核心优化循环

void Optimizer::optimize()
{
  // 关键参数
  // batch_size_: 2000 (采样轨迹数)
  // time_steps_: 56 (预测步数)
  // model_dt_: 0.05s (时间步长)
  
  for (size_t i = 0; i < iteration_count_; ++i) {
    // 1. 生成噪声控制序列
    generateNoisedTrajectories();
    
    // 2. 应用运动模型积分轨迹
    integrateStateVelocities(state_, control_sequence_);
    
    // 3. 计算所有轨迹的代价
    xt::noalias(costs_) = critic_manager_.evalTrajectoriesScores(critic_data_);
    
    // 4. Softmax 加权更新控制序列
    updateControlSequence();
  }
}

// optimizer.cpp:280-320
void Optimizer::updateControlSequence()
{
  // 计算 softmax 权重
  auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
  auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
  auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
  
  // 加权平均控制序列
  auto && softmaxes_extened = xt::view(softmaxes, xt::all(), xt::newaxis());
  xt::noalias(control_sequence_.vx) = 
    xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
  xt::noalias(control_sequence_.vy) = 
    xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
  xt::noalias(control_sequence_.wz) = 
    xt::sum(state_.cwz * softmaxes_extened, 0, immediate);
}
```

### 3. 运动模型

MPPI 使用差速或全向运动模型进行轨迹积分：

```cpp
// motion_models.hpp
// 全向运动模型 (Omni)
x_new = x + vx * cos(θ) * dt - vy * sin(θ) * dt
y_new = y + vx * sin(θ) * dt + vy * cos(θ) * dt
θ_new = θ + wz * dt

// 差速运动模型 (DiffDrive)
x_new = x + v * cos(θ) * dt
y_new = y + v * sin(θ) * dt
θ_new = θ + w * dt
```

## 评价函数 (Critics)

评价函数是 MPPI 的核心，决定了轨迹的好坏：

### 1. Obstacles Critic (障碍物避障)

```cpp
// obstacles_critic.cpp:65-120
void ObstaclesCritic::score(CriticData & data)
{
  // 遍历所有轨迹的所有点
  for (size_t i = 0; i < batch_size; i++) {
    for (size_t j = 0; j < time_steps; j++) {
      // 获取轨迹点的代价地图代价
      float cost = collision_checker_.pointCost(traj_x(i,j), traj_y(i,j));
      
      // 根据距离障碍物的距离计算惩罚
      if (cost >= INSCRIBED_INFLATED_OBSTACLE) {
        data.costs(i) += critical_weight_ * 
          (cost - INSCRIBED_INFLATED_OBSTACLE) / (LETHAL_OBSTACLE - INSCRIBED_INFLATED_OBSTACLE);
      } else if (cost > 0) {
        data.costs(i) += inflation_weight_ * cost / INSCRIBED_INFLATED_OBSTACLE;
      }
    }
  }
}
```

### 2. Goal Critic (目标吸引)

```cpp
// goal_critic.cpp:40-80
void GoalCritic::score(CriticData & data)
{
  // 计算到目标的距离代价
  auto goal_x = data.path.x.back();
  auto goal_y = data.path.y.back();
  
  // 轨迹终点到目标的距离
  auto dx = data.trajectories.x(xt::all(), -1) - goal_x;
  auto dy = data.trajectories.y(xt::all(), -1) - goal_y;
  auto dists = xt::sqrt(dx * dx + dy * dy);
  
  data.costs += weight_ * dists;
}
```

### 3. Path Align Critic (路径对齐)

```cpp
// path_align_critic.cpp
void PathAlignCritic::score(CriticData & data)
{
  // 计算轨迹与参考路径的偏离程度
  for (size_t i = 0; i < batch_size; i++) {
    for (size_t j = 0; j < time_steps; j++) {
      // 找到路径上最近点
      auto nearest_idx = findNearestPoint(traj_x(i,j), traj_y(i,j), data.path);
      
      // 计算偏离距离
      float dx = traj_x(i,j) - data.path.x(nearest_idx);
      float dy = traj_y(i,j) - data.path.y(nearest_idx);
      data.costs(i) += weight_ * (dx*dx + dy*dy);
    }
  }
}
```

### 4. Constraint Critic (运动约束)

```cpp
// constraint_critic.cpp
void ConstraintCritic::score(CriticData & data)
{
  // 速度约束惩罚
  auto vx_violation = xt::maximum(xt::abs(data.trajectories.vx) - vx_max_, 0.0);
  auto vy_violation = xt::maximum(xt::abs(data.trajectories.vy) - vy_max_, 0.0);
  auto wz_violation = xt::maximum(xt::abs(data.trajectories.wz) - wz_max_, 0.0);
  
  data.costs += weight_ * xt::sum(vx_violation + vy_violation + wz_violation, 1);
}
```

### 评价函数列表

| Critic | 功能 | 默认权重 |
|--------|------|----------|
| ObstaclesCritic | 避障 | 10.0 |
| GoalCritic | 目标吸引 | 5.0 |
| GoalAngleCritic | 目标朝向 | 3.0 |
| PathAlignCritic | 路径对齐 | 10.0 |
| PathFollowCritic | 路径跟踪 | 5.0 |
| PathAngleCritic | 路径角度 | 2.0 |
| PreferForwardCritic | 偏好前进 | 5.0 |
| TwirlingCritic | 防止原地旋转 | 10.0 |
| ConstraintCritic | 运动约束 | 4.0 |
| VelocityDeadbandCritic | 速度死区 | 35.0 |

## 参数配置

### 核心参数

```yaml
# params/controller_server.yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "navigo_mppi_controller::MPPIController"
      
      # 优化器参数
      time_steps: 56              # 预测步数
      model_dt: 0.05              # 时间步长 (秒)
      batch_size: 2000            # 采样轨迹数
      iteration_count: 1          # 优化迭代次数
      temperature: 0.3            # Softmax 温度
      gamma: 0.015                # 控制序列衰减
      
      # 运动模型
      motion_model: "Omni"        # Omni/DiffDrive/Ackermann
      
      # 速度限制
      vx_max: 0.5                 # 最大前进速度 (m/s)
      vx_min: -0.35               # 最大后退速度 (m/s)
      vy_max: 0.5                 # 最大侧向速度 (m/s)
      wz_max: 1.9                 # 最大角速度 (rad/s)
      
      # 加速度限制
      ax_max: 3.0                 # 最大线加速度
      ay_max: 3.0
      az_max: 3.5                 # 最大角加速度
      
      # 噪声参数
      noise_vx: 0.1
      noise_vy: 0.1
      noise_wz: 0.3
      
      # 评价函数配置
      critics: ["ConstraintCritic", "ObstaclesCritic", "GoalCritic",
                "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic",
                "PathAngleCritic", "PreferForwardCritic"]
      
      ObstaclesCritic:
        enabled: true
        cost_power: 1
        repulsion_weight: 1.5
        critical_weight: 20.0
        consider_footprint: true
        
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.4
```

### 参数说明

| 参数 | 类型 | 说明 |
|------|------|------|
| `batch_size` | int | 采样轨迹数量，越大探索越充分但计算量增加 |
| `time_steps` | int | 预测时域步数 |
| `model_dt` | double | 积分时间步长 |
| `temperature` | double | Softmax 温度，越小越贪婪 |
| `iteration_count` | int | 每次控制的优化迭代次数 |
| `vx_max/vy_max/wz_max` | double | 速度上限 |

## 速度指令发布

### LCM 发布器

```cpp
// vel_cmd_lcm_publisher.cpp:28-45
void HandlPlannerVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  gamepad_lcmt lcmt;
  
  // 速度映射到游戏手柄格式
  lcmt.leftStickAnalog[1] = msg->linear.x;   // 前进速度
  lcmt.leftStickAnalog[0] = msg->linear.y;   // 侧向速度
  lcmt.rightStickAnalog[0] = -msg->angular.z; // 旋转速度
  
  // 通过 LCM 发送到底层控制器
  lc.publish("vel_cmd_lcm_data", &lcmt);
}
```

### UDP 发布器

```cpp
// vel_cmd_udp_publisher.cpp
// 类似逻辑，通过 UDP 发送速度指令
```

## 性能优化

### xtensor SIMD 加速

MPPI 使用 xtensor 库进行向量化计算：

```cmake
# CMakeLists.txt
add_definitions(-DXTENSOR_ENABLE_XSIMD)
add_definitions(-DXTENSOR_USE_XSIMD)

# 编译优化
add_compile_options(-O3 -finline-limit=10000000 -ffp-contract=fast -ffast-math)
add_compile_options(-msse4.2 -mavx2 -mfma)
```

### 批量计算

所有 2000 条轨迹并行计算：
- 轨迹积分: 批量矩阵运算
- 代价评估: 向量化操作
- 权重计算: 批量 softmax

## 调试方法

### 1. 可视化轨迹

```bash
# RViz 中查看:
# - /mppi_controller/optimal_trajectory: 最优轨迹
# - /mppi_controller/trajectories: 采样轨迹云
```

### 2. 查看控制输出

```bash
ros2 topic echo /cmd_vel
```

### 3. 调试日志

```bash
ros2 run navigo_controller_server controller_server --ros-args --log-level debug
```

## 常见问题

### 1. 机器人振荡

**原因**: 控制参数过激进或采样噪声过大

**解决**: 
- 减小 `noise_vx/vy/wz`
- 增大 `temperature` 使控制更平滑
- 减小速度上限

### 2. 避障不及时

**原因**: 预测时域太短或 ObstaclesCritic 权重太低

**解决**:
- 增加 `time_steps` 或 `model_dt`
- 增加 `ObstaclesCritic.critical_weight`

### 3. 路径跟踪不准

**原因**: PathAlignCritic 权重不足

**解决**:
- 增加 `PathAlignCritic.cost_weight`
- 检查路径变换是否正确

### 4. 计算负载高

**解决**:
- 减少 `batch_size` (建议 ≥ 1000)
- 减少 `time_steps`
- 确保编译优化开启
