# 规控开发闭环验证：2026-09-08

本轮在 `feature/forward-aligned-navigation` 上实现并运行实际生产 C++ 控制器的反馈试验。最终生产版本 `d2ed7f2`：36/36 个可达端点试验成功；同一二进制关闭新策略时为 16/36。该结果支持继续 review 新控制策略，**不代表 NavigateToPose 全栈、完整实机路线或四足动力学验收通过**。

## 试验层级与冻结条件

- 仓库基线提交 `6942974`。生产改动为 `6240955`、`4032358`、`d2ed7f2`；验证入口由 `c013562` 引入，后续补充结果评分保护与报告。未创建 worktree、未合并分支。
- `controller_feedback.cpp` 使用 pluginlib 加载真实 `navigo_mppi_controller::MPPIController`，调用 `computeVelocityCommands()`，使用真实 `StoppedGoalChecker`。不是 Python 复刻策略或单独测试 heading 函数。
- 新命令经各轴死区 0.05/0.10/0.02 和上限 0.15/0.15/0.10 处理，反馈给理想 SE(2) 运动学模型。模型的新 pose/实际执行速度成为下一周期输入，因此属于控制器层闭环。
- 10 Hz 控制、每周期 10 次积分/碰撞子步。独立 SAT oracle 检查完整矩形与栅格相交，包含内部障碍、未知区、边界越界；没有复用新控制器的 collision guard 当作唯一评分器。
- 各组足迹相同：0.60×0.30 m 加每侧 0.01 m padding。目标阈值 0.25 m/0.25 rad，平移/旋转停稳阈值均 0.01，stateful=false。程序直接验证实际 GoalChecker 的 tolerance 返回值，当前命令也必须停稳才能记成功。
- 三个优化器初始 seed：42/43/44。每个场景相同初态、路径、地图、速度模型和试验预算。seed 是软件采样对照，不是独立实机试验。
- 最终 controller library SHA256：`5849539c7d6f3d91f5e944320f00a77811d21f0d52a93566a738e1cd18bfbe5a`；critics：`33b209e0e3c598c0d6d70aad04d9b52e8858fd353c7f439b13248a11981eea3d`。candidate 与 feature_disabled 的最终全集均使用该版本重跑。

仓库试验使用已有 `artifacts/matrix_scene_terrain_wh_gt_filtered_20260827/map.yaml`，从已观测自由区确定同一 2 m 直线路径 `(2.81746, 0.85399) → (4.81746, 0.85399)`。中心线最小静态障碍/未知区净距约 4.55 m，属于开阔区基线。与原 Matrix runner 一致，使用 `free_thresh=0.196`，防止把 PGM 的 205 未知像素误当自由区。各组使用相同预计算 inflation；没有加载运行中的 InflationLayer，因此日志有缺少该优化层的提示，但 footprint 检查未被关闭。

0829 的三个反馈试验保留录制窗口开始时的 pose 与第一条局部路径坐标/姿态，从静止开始。终点是该**录制局部路径终点**，不是原实机完整任务目标。其有界地图为空，没有重建历史动态 costmap，也没有混用不同 generation 的办公室地图。因此只能用于检验后退机制相关的几何条件，不能据此声称已复现真实办公室障碍。

## 最终结果：可达端点闭环，12 类场景 × 3 seed

| 场景 | 旧安装快照成功数 | 同版本策略关闭 | 新策略 | 新策略结束时间中位数，模拟秒 |
|---|---:|---:|---:|---:|
| 仓库初始航向 0° | 3/3 | 3/3 | 3/3 | 21.1 |
| 45° | 1/3 | 1/3 | 3/3 | 36.2 |
| 90° | 1/3 | 1/3 | 3/3 | 44.0 |
| 135° | 1/3 | 1/3 | 3/3 | 51.7 |
| 180° | 2/3 | 2/3 | 3/3 | 59.7 |
| 终点 yaw 对齐 | 2/3 | 2/3 | 3/3 | 13.6 |
| 终点对齐时 XY 被外移 0.4 m | 1/3 | 1/3 | 3/3 | 79.9（固定观测终点） |
| 50% 限速、控制器 reset | 3/3＊ | 0/3 | 3/3 | 77.2 |
| 绝对限速 0.05、动态上限变化及 reset | 3/3＊ | 0/3 | 3/3 | 96.2 |
| 0829 原 10 s 后退窗口的初始局部路径 | 2/3 | 2/3 | 3/3 | 73.8 |
| 0829 原 14 s 后退窗口的初始局部路径 | 2/3 | 2/3 | 3/3 | 78.0 |
| 0829 原 20 s 后退窗口的初始局部路径 | 1/3 | 1/3 | 3/3 | 53.6 |
| **合计** | **22/36＊** | **16/36** | **36/36** | — |

＊旧安装快照是本轮修改前保存的 **as-built** 安装件，并非本轮从 `6942974` 重新构建。旧件在限速场景到达，但不满足限速约束：50% 限速 reset 后 raw vx/wz 最高约 0.138/0.100；绝对 0.05 限速场景 raw vx 最高约 0.126。因此 22/36 仅为“到达端点”计数，不是安全通过率。主要对照应使用同一最终源码/库的 feature_disabled 与 candidate；其中 disabled 保留原 MPPI/输出平滑行为，也仍可出现小幅限速超调。

36 个新策略可达试验共 20594 条命令：普通 raw vx<−1e−6 为 0，vy 非零为 0，低于可执行阈值的非零 yaw 为 0，独立碰撞为 0，异常为 0。最终 XY 误差最大约 0.1997 m。新策略的取舍是承认 0.1 rad/s 的低速转身耗时，部分路线比旧策略更慢；此处没有证明它提高了贴路径精度、窄门通过率或运行效率。

## Shadow 与预期停车单独计分

- **9 个 shadow 试验**：三个 0829 窗口 × 三 seed，共 1314 个实际 C++ controller 输入。每次输入历史 pose/velocity 和原 source timestamp，路径固定为窗口开始时局部路径。新 raw vx 负值、非零 vy、不可执行 yaw 均为 0。录制速度持续非零时，部分新输出为 `HOLD/waiting_translation_stop`；不能拿历史后续轨迹判定这些停车命令的到达效果。
- **3 个旋转阻挡试验**：在同一仓库地图加入一个位于初始足迹外、旋转扫掠足迹内的单格障碍。新控制器均在全部 50 步拒绝旋转并输出零速，独立 oracle 无碰撞。共 150 次预期拒绝异常。旧安装件和 feature_disabled 各出现 3 次 oracle 碰撞后被试验器终止。
- **3 个过低限速试验**：10% 限速使可用角速度低于最小可执行阈值，reset 后保持零速/HOLD，属于预期行为，不应算作到达失败。

不能将上述 9+6 个试验加入导航成功率分母，也不能将其无碰撞直接外推到真实动态场景。

## 闭环实际发现并推动修复的问题

1. 最初将小于 0.05 的正向均值直接归零，导致对齐完成后永久停滞。0829 和多种仓库朝向都复现；改为最近可执行动作投影后，原 33 个可达试验全部通过。
2. root review 增加了速度限制及 reset 验证，生产代码补齐百分比/绝对限速状态的保存和基于动态 base constraints 的重新计算。
3. 新增“绝对上限恰等最小速度 0.05”场景再次发现离散零速吸引域：零中心采样经过投影后，可行动作占比不足，均值重新跌入零区域，3/3 均停在起点至 180 s。经单测确认，此次不是放宽浮点容差的问题。最终给合法约束下的优化序列设置可执行前向 warm start，真实速度反馈和历史仍保持原值，最终轨迹仍经碰撞复检。最终 3/3 达到端点。
4. 验证器补充了评分防护：历史出现过成功但末态漂出不计成功；进程 crash/timeout、任意碰撞都不能仅凭残留 CSV 的 success 字段记成功；初始姿态已到目标但与障碍重叠时优先判碰撞。

失败数据保留在 `candidate_prefixed_projection`、`candidate_before_warmstart_fix` 等目录。没有通过删失败数据、修改历史 odom 或扩大目标阈值来获得通过结果。

## 检查、复现与剩余验证

实际 C++ oracle 自检通过：自由空间、越界、足迹内部障碍、未知区、旋转碰撞、非有限输入。5 个 Python 评分回归测试通过。相同 seed 的 180° 新策略独立重跑，598 行 CSV（含轨迹、原始/执行命令和 mode）逐字段一致。153 份最终试验均已重新评分；评分保护不改变本表结果。Codacy MCP 当前无可调用工具，未运行 Codacy。

完整 Matrix 路径本轮没有运行：现存 `run_sim.sh` 含全局 pkill，缺少 runner 要求的 ROS domain 隔离支持，主机也没有所需 dummy 控制接口。本轮未执行该脚本、未修改主机网络、未启动真实 UDP 控制桥。主矩阵仅在 localhost、独立 ROS domain 179/180/181 运行，专项复核使用 182/183，直接调用控制器而不向实机发命令。

剩余验收包括：带传感器/TF 时延的全栈 BT/action/progress checker 与显式 BackUp；同地图同初始条件的 MuJoCo 四足动力学闭环；实测动作 deadband、组合轴、延迟、停稳、SDK 控制权；窄门摆腿包络、真实环境动态障碍；定位失效停车及重定位提交后的重新规划。上述均不能用本轮理想模型结果替代。

复现入口：[controller_feedback/README.md](/home/charles/project/colcon_ws/src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/controller_feedback/README.md)。本轮产物根目录：`/home/charles/project/colcon_ws/analysis_outputs/feature_navigation_20260908/`。

- [最终分组与逐场景比较](/home/charles/project/colcon_ws/analysis_outputs/feature_navigation_20260908/comparison.json)
- [逐试验约束检查](/home/charles/project/colcon_ws/analysis_outputs/feature_navigation_20260908/acceptance_checks.json)
- [输入和二进制哈希](/home/charles/project/colcon_ws/analysis_outputs/feature_navigation_20260908/provenance.json)
- [候选逐次试验、命令和错误](/home/charles/project/colcon_ws/analysis_outputs/feature_navigation_20260908/candidate/summary.json)

![反馈轨迹与原始控制命令](/home/charles/project/colcon_ws/analysis_outputs/feature_navigation_20260908/feedback_comparison.png)
