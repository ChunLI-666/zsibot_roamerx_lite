# Matrix 闭环规划控制批测设计

## 目标

批测用于分别回答两个问题：

1. 定位完全正确时，规划、costmap、控制器和 Matrix 执行链是否正常。
2. 使用 Lightning 实时定位时，完整链路是否仍满足相同验收标准。

两种模式不得混用。Lightning 失败时禁止自动切换 GT，否则会掩盖定位问题。

## 两条测试链

### GT 规划控制隔离基线

```text
Matrix GT odom -> 唯一 GT TF publisher -> Navigo/Nav2
-> cmd_vel_nav -> cmd_vel -> cmd_vel_safe -> LCM -> Matrix
```

Lightning 不启动。该模式只验证规划控制、障碍物、控制接口和仿真执行。

### Lightning 正式闭环

```text
Matrix LiDAR/IMU -> Lightning -> map -> odom -> base_link
-> Navigo/Nav2 -> cmd_vel_nav -> cmd_vel -> cmd_vel_safe -> LCM -> Matrix
```

GT 只被 recorder 和离线评估器读取。发送 goal 前要求连续定位正常、TF 新鲜；运行中
定位丢失时取消 action 并在 0.3 秒内输出零速。

## 第一版范围

当前只启用 Warehouse，按以下顺序串行运行：

| Case | 目的 |
|---|---|
| `straight_forward` | 基础前进和速度响应 |
| `straight_lateral` | Omni 横移能力 |
| `rotate_in_place` | 原地旋转 |
| `turn_then_forward` | 航向调整和路径跟踪 |
| `obstacle_detour` | 全局/局部 costmap 与绕障 |
| `long_route` | 长时间定位、规划和控制稳定性 |

每个 case 先跑 GT 基线，再跑 Lightning。定位失败时控制指标标记为
`NOT_EVALUATED`，不能归因于 planner/controller。

## 串行隔离

第一版禁止并行启动 Matrix：

```text
flock 获取服务器唯一锁
-> 创建独立 result_dir 和 ROS_DOMAIN_ID
-> 启动并登记 Matrix/MC/UE PGID
-> preflight 传感器、TF、action 和 costmap
-> 执行一个 case 并录包
-> 发送零速、取消 action、SIGINT 结束 recorder
-> 按登记 PGID 清理并检查残留
-> 写 case_status.json
-> 释放锁
```

禁止全局 `pkill mc_ctrl`，禁止直接覆盖共享生产参数。Matrix 尚无 reset service，
所以每个 case 使用冷启动，且不能和人工仿真实例同时运行。

## 指标

### 任务与轨迹

- action/waypoint 成功率、耗时、最终 XY/yaw 误差；
- 规划路径长度、实际轨迹长度、detour ratio；
- 横向跟踪误差、路径航向误差、曲率变化和重规划次数；
- 不必要倒车距离、目标附近振荡次数。

### 控制链

- `/cmd_vel_nav`、`/cmd_vel`、`/cmd_vel_safe` 频率和最大间隔；
- 三段命令传播延迟、各轴饱和比例和同时非零比例；
- 命令速度到 MuJoCo 实际速度的响应延迟和误差；
- 非零命令但不运动、零命令但仍运动的持续时间。

### 障碍物

- plan/trajectory 穿越 lethal cost 的次数；
- 实际轨迹最小障碍距离；
- LaserScan 到 local/global costmap 的更新时延；
- Matrix 当前没有 ROS contact 真值，严格碰撞指标标记为 `UNAVAILABLE`。

## 失败分类

```text
INVALID_INPUT, SIM_STARTUP, SIM_RESET, SENSOR_STALE, TF_CONTRACT,
LIGHTNING_INIT, LIGHTNING_LOST, LOCALIZATION_JUMP, MAP_OR_COSTMAP,
PLANNER_NO_PATH, CONTROLLER_NO_VALID_TRAJECTORY, COMMAND_PIPELINE,
ACTUATOR_NO_MOTION, COLLISION_PROXY, GOAL_TIMEOUT, GOAL_TOLERANCE, UNKNOWN
```

## 实施顺序

1. 为现有 `matrix_closed_loop_run.sh` 增加 `--domain-id`、模式互斥和 PID 残留检查。
2. 先固化 GT 模式的直行、横移、旋转、转弯四个基础 case。
3. 扩展 bag analyzer，生成路径、控制、障碍和失败分类指标。
4. 增加串行 batch runner 与 suite，统一输入签名和报告。
5. GT 基线通过后加入绕障和长路线。
6. Lightning 定位批测通过后，使用相同 case 执行正式闭环对比。

当前阻塞：Lightning Warehouse 连续定位覆盖率仅 9.03%；Matrix 启动脚本仍会修改
共享配置，且没有可靠 contact topic。二者未解决前不能宣称完整闭环批测通过。
