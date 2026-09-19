# 有身份的受限后退：实现与隔离闭环验证（2026-09-11）

当前提交增加真正的 `BackUpEpoch` action；默认关闭，尚未部署到实机。普通 TRACK 保持原有控制频率及前向约束，显式 BACKUP 才能发受限负 vx。PD1 主线及建图算法未由本模块改变。

## 执行契约

- `NavigationToken` 的 `execution_kind` 区分 TRACK/BACKUP；后者带不可变速度上限和 boot-monotonic deadline，使用新的请求序号。不能靠裸 Twist 取得后退权限，也不把 BackUp 伪装成 FollowPath。
- ControllerServer 独占执行互斥锁和原有 controller session。请求捕获的定位身份、心跳和 source-stamped 起点 TF 必须匹配；实际安装的连续 odom 起点通过 `executed_start` 返回。
- 控制器、relay、gate 三处验证模式：TRACK 禁止负 vx；BACKUP 仅负 vx，其他五轴为零。BACKUP 生命周期最长 5 s，最大速度 0.1 m/s，距离预算 0.03–0.30 m。结束发布停令、撤权并封存 token；下一次规划必须新身份序号。
- 使用独立 `epoch_backup_control_frequency`，默认 20 Hz，只用于 BackUp action。普通 MPPI 的 `controller_frequency=10 Hz` 不变。低于 20 Hz 的 BackUp 请求被拒绝；relay 必须至少 20 Hz，gate watchdog 不超过 50 ms。

## 距离、障碍和停稳

`distance` 是**最大累计路径预算**，不是要求精确倒退到某个点。剩余预算按 `distance - max(累计路径, 后向投影)` 计算；姿态/横向偏移和里程跳变也受限。停止速度上限使用：

```
t_reaction = 0.30 s 最大接受 odom 源滞后
           + command TTL（默认 0.30 s）
           + BackUp / relay / gate 各 0.05 s
           = 0.75 s（默认配置）
v * t_reaction + v² / (2 * 0.2) <= 剩余距离 - 0.005 m
```

如果允许的速度低于当前配置的最低有效速度，进入 STOPPING，不把速度抬回 deadband。最低速度由当前 smoother vx deadband（实验为 0.05 m/s）显式配置；0.2 m/s² 是软件 relay 准入下限，**不是已测量的机械狗刹车能力**。

每周期按新鲜 source TF 对完整填充 footprint 做后方连续扫掠，检查内部和边界，拒绝 unknown/越界；STOPPING 中也检查按实测线速度/角速度估计的剩余停车扫掠。取消、定位失效、epoch 变化、反馈陈旧、deadline 或障碍立即撤权停令。成功需要连续 200 ms 的新鲜实测平移/角速均不超过 0.005，并且有至少 1 cm 有效后向投影；无有效后退返回 STOPPED/no_effective_retreat。

## 实验方法及明确边界

`epoch_backup_validation/run_backup_stack.py` 启动真实导航进程、ControllerServer action、smoother、gate，仅接入隔离 SE2 plant 和传感器夹具，ROS domain 190，不启动 SDK/硬件通道。独立 SAT/连续扫掠审计不导入生产碰撞函数。延迟实验保留 odom 原始 stamp，注入 250 ms 反馈滞后，并将 relay 减速度设为 0.2 m/s²。实际 plant 累计路径、最大偏移、action 报告投影/路径均严格比较最大距离预算，不接受超距后返回失败码作为满足距离约束。

旧模型仅计 0.3 s 反应时间，在 `delayed_odom_repeat_2` 实测 0.204206306 m 超过 0.2 m；该失败保留，不能计为成功。修正为上述组合预算后，要求同条件独立重复三次。模型没有包含已实测 RK/SDK/机械执行器时延；模型 footprint 也不等于实测摆腿完整包络。不能据此宣称实机硬停止距离已保证。

DDS 审计保存源发布时刻、接收时刻和回调双时钟采样；原回调审计独立保留。缺少 timestamp、时钟漂移或发布区间过宽将明确不可评分，不能把接收队列积压当作实际发送时间，也不靠放宽门控 TTL 掩盖问题。

## 验证结果

最终 19 次隔离运行：动作预期 18/19；实际累计路径/最大偏移及已报告距离预算 19/19 未越界，几何审计 19/19 零碰撞/零不确定（既有地图失败例为事后独立几何评分）。严格综合通过仅 1/19：DDS 时序可评分 2 例，其中既有地图动作失败；其余 17 例 DDS 不可评分，不能记总体通过。所有运行的生产源码/二进制 manifest 前后不变。

| 场景 | 实际累计路径 m | 动作结果 | 严格 DDS |
|---|---:|---|---|
| blocked_final | 0.000000 | Backup rear footprint sweep blocked | 不可评分 |
| cancel_final | 0.015793 | Backup canceled/preempted | 不可评分 |
| deadline_final | 0.062112 | Backup time bound | 不可评分 |
| delayed_final_1 | 0.170633 | stopped_within_distance_budget | 不可评分 |
| delayed_final_2 | 0.169886 | stopped_within_distance_budget | 不可评分 |
| delayed_final_3 | 0.170475 | stopped_within_distance_budget | 不可评分 |
| epoch_final | 0.017401 | Backup authority/TF/costmap unavailable | 不可评分 |
| free_final | 0.152399 | stopped_within_distance_budget | 不可评分 |
| invalid_final | 0.000000 | Invalid bounded BackUp request | 不可评分 |
| loss_final | 0.017315 | Backup authority/TF/costmap unavailable | 不可评分 |
| moving_feedback_final | 0.156339 | Backup time bound | 不可评分 |
| no_effective_final | 0.000000 | no_effective_retreat | 不可评分 |
| odom_freeze_final | 0.042989 | Execution odometry unavailable/stale/invalid | 不可评分 |
| recorded_map_final | 0.012864 | Execution odometry unavailable/stale/invalid（未满足自由后退预期） | 通过 |
| replan_final | 0.155444 | stopped_within_distance_budget | 不可评分 |
| slow_controller_final | 0.000000 | Backup requires epoch_backup_control_frequency >=20 Hz | 通过 |
| slow_gate_final | 0.000000 | Backup time bound | 不可评分 |
| slow_relay_final | 0.000000 | Backup time bound | 不可评分 |
| small_budget_final | 0.000000 | no_effective_retreat | 不可评分 |

三个 250 ms / 0.2 m/s² 重复的实际累计路径、最大偏移和最终报告后向投影均相同，分别为 **0.170633479、0.169885713、0.170474898 m**；三次 action 成功返回时实测执行速度均为零，并满足连续停稳检查。旧模型 0.204206306 m 失败保留在 `delayed_odom_repeat_2`。

既有地图 `recorded_map_final` 在 0.411 s 因 odom 陈旧/无效中止，实际仅走 0.012864 m；该例自由后退闭环目标未完成。DDS wire 可评分且零违例，旧回调评分有 3 条违例，最大发布到回调约 309 ms。`loss_final` 另保留 1 条旧回调“非 NORMAL 定位期间出现 safe 非零”证据，DDS 本身不可评分，不能把该条确定归因为实际撤权后发布。

`replan_final` 的 BackUp 成功停止后，真实 NavigateToPose 使用新的 TRACK 身份，action 状态 SUCCEEDED；`moving_feedback_final` 持续伪报 -0.03 m/s，最终超时停止，没有错误报告成功。3 cm 小预算在更保守预算下不产生有效运动，返回 no_effective_retreat。低 BackUp / relay / gate 频率配置均无实际运动。

Plant 时间边界：Fixture 使用 `dt=min(wall_delta,0.1)` 并在 tick 末端评估 0.35 s watchdog，不是逐段精确 wall-time 执行模型。三次 delay 的最大 dt 分别为 0.100000、0.043033、0.059683 s；第 1 次全程有一次 0.1 s 截断，记录间隔 0.322831 s，发生在动作前且 executed=0。三次动作期间均无截断。不能把被压缩的 wall 间隔称为已模拟，也不能据此推导任意调度停顿下的真实持续执行最坏距离。完整 piecewise wall-time/watchdog plant 与区间传播式 DDS 审计列为后续工作。

机器可读结果见 [epoch_backup_results_20260911.json](epoch_backup_results_20260911.json)；原始事件、配置、二进制/源码 manifest 和日志保存在 workspace `analysis_outputs/epoch_backup_20260911/`。最终 v2 数据目录为 `budget_v2/*final*`；所有早期失败和观察器启动错误均保留。

## 开发与交付

Canonical repository 为 `zsibot_roamerx_lite`，分支 `feature/forward-aligned-navigation`，本轮 base 为 `e826d6ffbb40783bd27529406afe7e387da5c01c`。使用已有 canonical checkout，没有新 worktree、没有 merge。接口与 MPPI/smoother phase 变化由协作 agent 原子联合提交，等待用户 review。

已执行 8 包联合构建，以及独立 BackUp 频率修改后的 controller 增量构建。authority 8、BackUp 数学/footprint 6、BT lineage 9、gate 25 个测试通过；phase/helper 与观察器测试由导航验证报告汇总。Codacy 工具在当前可调用工具中不可用，未执行该检查。实机性能与全后退恢复 BT 的端到端触发仍需单独验收；当前 action 真执行与结束后新 TRACK 重新规划在隔离全栈中验证。
