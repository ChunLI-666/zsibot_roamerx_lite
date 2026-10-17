# 定位与导航联动阶段审查（2026-09-10）

本轮在上一阶段导航 `2b8effd`、定位 `72532e8` 上继续开发。总 goal 保持 active；代码只在 canonical checkout 的两个 feature 分支，未推送、合并或启动实机运动。

## 当前实现

定位的非学习式检索接口和粗定位、精定位、验证提交、Tracking 流程保持不变。本轮将不可变定位身份从实际完成的定位帧传过 PGO 和外推器，再与实际发布的 map→导航 base 位姿及源时间绑定。发布就绪不再由几个异步状态的即时拼接决定。

导航新增共享 `navigo_epoch_msgs` 接口和严格实验开关。实际规划请求捕获带身份的定位起点；标准规划/平滑 action 的返回值保留请求身份；typed FollowPath 安装路径后才确认执行。控制器、平滑器与最终 gate 共同核对定位、业务任务、规划版本及控制器身份。平滑器保留源命令时限，ROS 时钟暂停不会延长授权。控制计算使用原源时刻验证过的位姿，周期重规划保留进展计时。

行为树保留业务目标，定位失效后停车并清理旧路径和异步请求，健康恢复后重新规划、安装、跟踪。正常终态则停车并封存路径，允许成功结果到达业务 action。严格模式仅允许清图和等待恢复，带身份的受限 Spin/BackUp 仍未实现。

## 已取得的证据

定位提交 `322f745`：9 套 CTest、6 项 typed stream 审计测试通过。实际传感器在线回放记录 539 个 typed heartbeat，183 个 ready 状态均与同源时间的实际 PoseStamped 和 TF 组合相符；两次进程启动具有不同 session。断流恢复产生新 epoch/commit，LOST→融合就绪 15.206 秒、LOST→ACTIVE 19.742 秒。35 个执行文件/库与 20 个源码/协议文件的冻结哈希在实验前后保持一致。这是生产输出一致性证据，不是独立定位真值或板端性能验收。

导航提交 `b2c0d4e`、`842f38f`、`01adaa0`、`4b8cb03`：共享接口、BT navigator、core、MPPI、controller、smoother、gate 已构建。9 项真实 ROS action 血缘测试、1 项真实 BtActionServer 终态抢占测试、8 项 C++ authority、16 项严格 gate 和 9 项原门控测试通过。新增 core 虚函数后已重建实际使用的 MPPI 插件。多途经点规划使用同一带身份起点移除已通过中间点，保留最终点的 yaw/停稳检查，避免回到已访问点。

初次全栈 `normal_v2` 到点停稳却在 90 秒内无法结束 NavigateToPose：137 次 controller `goal_reached` 被 gate 的 138 次 challenge 轮换打断。该结果明确记为失败，不因输出身份审计没有违规就算完成。修复为正常完成封存 token、停车且保持 challenge；同 token 的后续 active/命令无法复活。后续全栈结果单独记录。

修复后的 `normal_v3` 在 31.05 秒内完成真实 NavigateToPose，最终位置误差 0.19977 m、yaw 误差 0、最终命令为零；运行输入哈希未变。实际参数中的 goal checker 容差为 0.25 m / 0.25 rad，停稳阈值各 0.01；0.2 m 是 FINAL 阶段进入边界。终点容差不能代表穿门所需的定位精度。

独立审查另复现了初版 wire audit 的漏检：近期 authority 消息被 50 ms 宽限整体跳过，构造错误 token、inactive authority、不同 controller session 且没有 raw 来源的短 trace 仍能通过。因此初版 `wire_audit_failures=0` 不能证明严格授权链，必须使用修正后的审计器对原始事件重新评分。真实 action 终态也需结合独立几何误差断言，不能只检查 status=SUCCEEDED。原始运行、工具失败和重新评分结果分别保留。

修正后的独立审计要求明确匹配 active token、controller session、boot、源年龄，以及 raw→smooth 不变的来源字段，并要求定位与 gate 证据。8 项审计回归（含上述错误授权负例）通过。对 `normal_v3`、`loss_recovery_v1`、`controller_silence_v1`、`pending_planner_v1` 的原始事件重新评分，分别核对 446、437、16、434 个非零输出，未发现违反修正审计规则的输出；新版证据保存为各目录的 `wire_audit_strict_v2.json`，没有修改原始日志。

最终执行范围为 16 类场景：正常到达、失定位恢复、ROS 时钟暂停、控制器停发、取消、迟到规划结果、迟到平滑结果、多途经点、gate 重启、定位进程 session 变化、业务抢占、裸恢复旁路、乱序命令、源位姿/TF 不一致、未来 TF 污染、持续重规划下卡住。最终正常基线 `normal_final` 使用 `4b8cb03` 的安装库与收尾工具，31.34 秒完成，位置误差 0.19808 m、yaw 误差 0、最后命令为零，输入哈希未变。

乱序实验在旧命令源年龄 163.70 ms 时已观察到零速，早于其 300 ms TTL；因此该用例不是等过期才停车。1e-4 m 的源位姿/TF 偏差触发实际 controller 拒绝，恢复一致后到达。卡住实验在反复重规划时仍于约 14.48 秒触发实际进展失败，没有通过每次新 plan 重置计时而无限掩盖。完整分景、注入完成证据、失败轮及最终离线复评见 [全栈实验报告](epoch_navigation_integration_review_20260910.md)。

对实际转向 `preempt_v1` 另做只读检查：按完整 token、controller session、command sequence 配对 335 条 raw 与 672 条 smoothed。41 条速度差异包含重规划清零和限加速度，但 raw 的零转速/零平移被平滑成非零的情况均为 0；raw、smoothed 和 1592 条 safe 中同时平移与旋转、负 vx 均为 0。两次 ALIGN→TRACK 的旧转速约 0.0511/0.0533 rad/s，均可在一个当前平滑周期内清零；TRACK→FINAL 先零速再转向。本次没有复现静态条件推演中的混合速度，也没有据此推断碰撞。

## Feature 分支交接

| 仓库 | 本轮 base → 实现/测试 review HEAD | 本轮变化 |
|---|---|---|
| 导航 `feature/forward-aligned-navigation` | `2b8effd` → `1a9405b` | 5 个提交，55 文件，+6691/-22 |
| 定位 `feature/descriptor-relocalization-verification` | `72532e8` → `322f745` | 1 个提交，22 文件，+1275/-6 |

导航五个提交为共享协议 `b2c0d4e`、BT/规划血缘 `842f38f`、控制/平滑/gate `01adaa0`、多途经点 `4b8cb03`、真实全栈验证 `1a9405b`。统计包含测试、结果 JSON 和报告，不全部是生产代码。随后仅增加本次交接文档；上述 HEAD 冻结实现/测试范围。完整 SHA、逐文件 diff、测试和风险见 [交接清单](epoch_feature_handoff_20260910.json)。定位新增依赖导航分支中的 `navigo_epoch_msgs`，两个 feature 变更需配套构建与 review。

## 验收边界

- 官方模型推导的 home 站姿 footprint 和 padding 仍是实验配置。真实步态、负载外伸、导航基准变换和板端时延需要实测。
- 当前源单调时钟协议要求各导航进程处于同一 Linux boot 和时钟域；没有提供跨机时钟同步协议。
- 固定绑定首个地图加载实例；完整热切图需要同步更新定位与导航地图，尚未实现。
- 全栈夹具提供 SE(2) 反馈、地图 raycast、TF 和可控定位状态，实际规划及控制由真实 Nav2 节点完成。此处定位 fixture 不能替代实际 Lightning 几何误差验证；本轮身份审计也不能替代完整轨迹碰撞验收。
- plant 自带命令失联超时，且没有机械惯性/SDK 制动模型。gate 重启后的新授权验证不证明实际下位机在 gate 进程退出期间会停车；这依赖后续 SDK/下位机 watchdog 与实机验证。
- 平滑器按各轴加速度修改控制器速度，当前协议保证来源与时效，尚未证明平滑后的执行轨迹与 MPPI 碰撞预测完全一致。静态条件推演存在从转向切换到平移时的混合速度可能；`normal_v3` 全程无转速，未复现该模式混合或碰撞。需在后续转向/门边场景中联合审计 mode、raw、smoothed、safe，并把平滑动力学纳入预测或最终碰撞责任范围，不能仅靠末端裁剪宣称完成。
- 独立参考/留出集成功率、显式受限运动恢复、共享 LM 回环失败处理与完整 SLAM 回归、RK3588 及实机路线验收继续保留。Codacy 工具不可用，未执行该检查。

相关设计与实现见 [协议契约](epoch_navigation_contract_20260910.md)、[控制器和平滑器/gate 实现](epoch_controller_gate_implementation_20260910.md)；完整总目标见 [项目 goal](project_goal_20260910.md)。
