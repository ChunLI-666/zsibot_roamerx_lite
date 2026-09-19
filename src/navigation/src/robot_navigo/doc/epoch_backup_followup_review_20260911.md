# 自动恢复与受限后退仿真补充验收（2026-09-11）

本轮使用 localhost ROS domain 190 的真实导航节点、BT、控制器、平滑器及门控，机械运动由 SE(2) 仿真提供。没有 SDK、LCM 或实机指令。PD1 主线不变。自动恢复完整闭环已通过一次最终源码验证；动态后障碍残余运动检测仍缺覆盖，不能宣布全部仿真验收完成。

## 验收设计

- 自动恢复只发送一次 NavigateToPose。用临时牵引约束使真实 progress checker 在配置的 **15 s** 后失败；收到实际 BACKUP 安装事件才解除约束。要求 BT 自主后退、停稳、同 navigation session/task 的更大 plan sequence TRACK，再到原目标。
- 动态后障碍在源 HOLD 且里程计仍有负 vx 时插入。修改相同 world occupancy，扫描与独立几何 oracle 采用同一有效时间。真实 local costmap service 的 lethal cell 回应同时记录请求/回复边界和执行速度。
- 既有地图 free 与确定性 350 ms executor stall 分开。stall 必须在实际执行 vx<0 时发生，期望撤权、动作失败并停止，不要求任务成功。
- 每个输出路径独立，保留失败与协议未覆盖结果。不调生产 TTL、减速度或动作超时来使实验通过。

## 已冻结实验记录

根目录：`/home/charles/project/colcon_ws/analysis_outputs/epoch_backup_followup_20260911/`。具体运行的 manifest 是其实际源码/参数/二进制依据；早期结果不能冒充最终源码复测。

| 运行 | 实际结果 | 判定 |
|---|---|---|
| automatic_bt | 未进入后退，等待超时 | 失败；严格 navigator 固定选择旧 XML |
| automatic_bt_wired | 新 XML 已接线，但仍超时 | 失败；后续定位到 terminal/门控交互 |
| automatic_bt_terminal_fixed | 客户端 terminal 优先修复后仍超时 | 失败；不能以 mock 通过代替闭环 |
| automatic_bt_return_chain | 参数与 BT 日志确认恢复树被直接 halt | 失败；progress failure 更换 gate challenge、task 重建 |
| automatic_bt_progress_terminal | 自动触发 BackUp，后退 0.155944214 m，后续使命失败 | 失败；约 2.1 s 客户端中止，诊断证实反馈抢先导致 ACK 饥饿 |
| automatic_bt_ack_diagnostic | 真实日志确认 handle=0 / age=2.100 / timeout=6.000 | 失败；ACK 饥饿证据保留 |
| automatic_bt_feedback_fixed | NavigateToPose 成功；BackUp 后退 0.155948728 m 后重新 TRACK 到原目标 | **通过自动恢复闭环**；DDS interval / callback 0 失败 |
| rear_during_braking | 后退 0.156194539 m，正常停止；实际间隙约 7.64 cm | 未覆盖；没有触发 STOPPING 障碍检查 |
| rear_during_braking_visible | 后退 0.159405224 m；控制器返回 stopping footprint blocked，实际间隙 2.6091 cm | 仍未覆盖完整残余运动检测；首次 costmap 可见证据时实际速度已为零 |
| recorded_free | 后退 0.153077261 m，动作成功，独立几何通过 | 通过该次既有地图 free 条件 |
| recorded_recorded_stall | 已经零速才发生 stall | 前置条件不足，不能当作运动中故障覆盖 |
| recorded_moving_stall | 真实负 vx 时注入，完整路径 0.044329679 m；动作按预期失败、停止 | 通过该故障场景；DDS 审计通过，保留 5 个旧 callback 标记 |

动态后障碍第二协议保留未覆盖结果，不再通过继续移动障碍或反复复测寻找通过。独立几何无碰撞不等于生产检查在残余运动期及时发现障碍。

## 接线与终止处理修复

1. navigator 新增默认 false 的 `enable_epoch_backup_recovery`，只能在 epoch 开启时选择审核过的固定 BackUp XML；不能传任意 XML。原非 epoch 行为兼容。
2. 已到达的 Action terminal/rejected 优先于同时更新的 path，防止真实失败被新计划覆盖而无法进入 RecoveryNode。真实插件加 mock action 的同一测试做 old-source red→fixed-source green；早期缺少客户端 ACK 前置的失败测试日志也保留。
3. progress checker 失败先发布零命令，再发布窄分类 `progress_failed`。门控仅封存真正已安装令牌，保持恢复挑战；旧令牌 raw/relay/重新 active 均不可复活，新执行仍需完整身份、时效、安装匹配。定位失效、身份变更、急停、intent 超时仍撤权。

4. 实际 `EpochBackUp` 在 `handle=0 age=2.100 timeout=6.000` 分支失败。无反馈 mock 同类型动作正常 ACK 且可执行超过 2 s；生产 SimpleActionServer 模板加 20 Hz feedback、BT 10 Hz 后稳定 red（2.007 s ACK 超时）。Jazzy 28.1.13 的 action waitable 每轮按 feedback→status→goal→result→cancel 选择一个事件。新 ActionNode 用正预算 `spin_all(1 ms)` 重新收集就绪事件，覆盖 ACK/result/cancel 与 Guard 等待时的 late ACK，不增加 2 s ACK 或 6 s 动作总超时。预算限制每次泵继续取回调的时间；不能抢占一个已经运行的阻塞 callback，也不保证无限输入下公平。

对应官方源码：[Jazzy rclcpp_action client.cpp](https://github.com/ros2/rclcpp/blob/525a41818866ca44aa7418393fe5f94732ab3831/rclcpp_action/src/client.cpp#L330)。本机 ROS 为 Jazzy，不能照工作区概述中的 Humble 推定实现细节。

## 仿真有效范围

新共享 plant 按完整 wall 时间积分，并在 350 ms 模拟 SDK watchdog 精确切分，不再截断 100 ms 长间隔。shutdown 审计在 `Trial.close()` 完整结算尾部并 finish 后进行，失败路径同样保留几何结果。新恒定 Twist 段使用独立弧线扫掠 oracle；历史缺少完整字段的段只保留 legacy 判定。

这是对平滑后安全速度的理想瞬时跟随模型，不包含关节、步态、惯性和真实 SDK 延迟。0.2 m/s² 是 relay 准入配置要求，.05 m/s 是当前 deadband 相关配置，不是实测硬件能力。

既有地图新 free 的 max tick 7.427 ms / thread CPU 7.407 ms，scan 6.289 ms，没有再次出现旧 336 ms 异常。单独扫描 benchmark 的 recorded p50 2.544 ms、max 3.574 ms；这些证据不能追溯证明旧未插桩的 stall 原因。新确定性 sleep 的 clock_odom wall 329.361 ms / CPU .577 ms 是人为注入证据，不得用来解释旧问题。


## 最终闭环与测试

最终运行 `automatic_bt_feedback_fixed` 保持同 navigation session / task 1 / gate challenge：TRACK plan 12 进展失败 → BACKUP plan 13 → `backup_finished` 封存并成功 → TRACK plan 14 → NavigateToPose SUCCEEDED。目标 (0.8,0,0)，最终 (0.606345356,0,0)；从本次冻结生产 `general_goal_checker` 读取 XY=0.25 m、yaw=0.25 rad，位置误差约 0.193655 m，符合既有终点容差。早期 runner 的额外 .15/.1 坐标门槛被误标“existing”；在本次开始前已改为真实 XY 欧氏距离/yaw 参数判定，前面的失败记录不被重判。

完整关闭后总路径 0.918242812 m，反向路径 0.155948728 m；3481 个新恒定 Twist 段，零碰撞、零不确定，零轨迹一致性失败。运行前后 manifest 一致。DDS publication interval 审计可评分且零失败，历史 callback 审计本次也是零失败。这是一次确定性故障注入闭环，不是长期可靠率估计。

- gate 29 项通过：包括封存旧令牌、同任务新 BACKUP 完整准入、普通错误不享受例外、LOST/急停/定位与 intent 超时、健康换 epoch / gate。
- 实际 BT 动态插件 12 项通过：原 late ACK/取消、终止优先与本次反馈饥饿回归。20 Hz feedback / 10 Hz BT 最大完整 tick 0.266 ms；无反馈 mock 为 0.231 ms，仅该测试机器/负载的观测，不是实时上界。
- 固定树选择 4 项通过；注入与独立几何 helper 6 项通过；Python compile、git diff whitespace 检查通过。
- 构建：navigo_bt_navigator、navigo_behavior_tree、navigo_path_controller 单包顺序构建通过；未并发覆盖运行中的二进制。

证据日志在同输出根目录：`terminal_acknowledged_before_fix.log` / `terminal_acknowledged_after_fix_all.log`、`backup_feedback_clean_before_fix.log` / `backup_feedback_fixed_all_tests.log`、`progress_terminal_gate_tests.log`、`bt_selection_tests.log`、`recovery_helper_tests.log`。最早反馈 red 测试因 ASSERT 早退造成 mock 线程收尾崩溃，保留 `backup_feedback_before_fix.log`；修正测试清理后的同一生产旧逻辑仍干净 red，不能用崩溃本身作为生产故障证据。

`progress_failed` 窄终止依赖同机可靠 execution 消息按序到达；若终止分类丢失而只观察 generic inactive，会保守撤权并阻止恢复，不能保证恢复活性。旧令牌始终不能恢复运动授权。生产 recovery 仍默认关闭，feature 分支待用户 review，无合并。
