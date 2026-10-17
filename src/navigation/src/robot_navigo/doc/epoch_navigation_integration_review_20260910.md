# 定位版本与导航联动：真实 Nav2 集成验证（2026-09-10）

最终 16 个场景均达到各自验收条件。正常及恢复任务真实 action 返回 SUCCEEDED；取消/故障场景验证停车与 CANCELED，不能计作导航到达率。全部日志由最终严格独立 wire auditor 复核，非零安全命令无授权链违规。每景源码和安装库前后 hash 相同；最终正常基线使用生产 commit `4b8cb03`。

## 运行范围与独立边界

实际启动 planner、path smoother、BT navigator、controller、MPPI、velocity smoother、gate 和 lifecycle manager。场景为已有 warehouse 地图 `artifacts/matrix_scene_terrain_wh_gt_filtered_20260827/map.yaml`，50 Hz planar SE2 命令反馈 plant、25 Hz raycast scan、源时间一致的 TF/odom 和 typed localization fixture。没有 robot bridge/LCM/UDP 执行器或实机，ROS 域 188/190、localhost，最多两个 stack 并行，仅清理本试验的子进程组。

该轮是**真实导航栈的身份、失效停车、重新规划及动作终态契约验证**。定位输入由 fixture 注入，未在本轮连接真实 Lightning 特征重定位。没有实机动力学、实测步态/载荷、碰撞动力学或独立全栈 SAT 评分；此前 459 次 controller-feedback 几何矩阵另列，不能合并宣称全栈零碰撞。停稳指 `/cmd_vel_safe` 连续为零及运动学 plant 速度为零，不代表物理停止距离。

使用同一 ZSL1 模型矩形及 0.01 m padding、0.50 m inflation；模型仍受 home 姿态与坐标标定假设限制。地图加载实例固定；map→odom 纠正不等于切换地图。有效 goal checker 为 XY 0.25 m、yaw 0.25 rad、线/角停止阈值 0.01；0.20 m 是 FINAL_ALIGN 进入界限。

## 最终场景结果

所有目录均位于 `/home/charles/project/colcon_ws/analysis_outputs/epoch_navigation_20260910/`。`4` 为 SUCCEEDED，`5` 为 CANCELED，抢占旧业务目标 `6` 为 ABORTED。取消类几何误差不作为到达评分。

| 场景与实际注入 | 证据目录 | 终态 | XY / yaw 误差 | 结论 |
|---|---|---:|---:|---|
| 正常完成 | `normal_final` | 4 | 0.1981 m / 0.0000 rad | 通过 |
| LOST→epoch2，map→odom纠正0.3m | `loss_recovery_v1` | 4 | 0.1899 m / 0.0000 rad | 通过 |
| SIGSTOP控制器，时钟继续 | `controller_silence_v1` | 5 | 不评分（期望取消） | 通过 |
| 暂停ROS时钟并SIGSTOP控制器 | `clock_pause_v1` | 5 | 不评分（期望取消） | 通过 |
| 真实业务action cancel | `cancel_v1` | 5 | 不评分（期望取消） | 通过 |
| 真实规划旧结果延迟/拒绝cancel | `pending_planner_v1` | 4 | 0.1897 m / 0.0000 rad | 通过 |
| 真实平滑旧结果延迟/拒绝cancel | `pending_smoother_v2` | 4 | 0.1965 m / 0.0000 rad | 通过 |
| 两航点有序prune | `through_poses_v1` | 4 | 0.1980 m / 0.0000 rad | 通过 |
| 真实gate进程重启 | `gate_restart_v1` | 4 | 0.1920 m / 0.0000 rad | 通过 |
| 定位发布者身份替换 | `localization_session_v1` | 4 | 0.1941 m / 0.0000 rad | 通过 |
| TTL内旧序号重放 | `command_reorder_v1` | 5 | 不评分（期望取消） | 通过 |
| 真实业务目标抢占/转向 | `preempt_v1` | 4 | 0.1920 m / 0.2440 rad | 通过 |
| typed/TF源时刻位姿相差1e-4m | `pose_mismatch_v1` | 4 | 0.1969 m / 0.0000 rad | 通过 |
| 插入未来3s、错误1m的TF | `future_tf_v1` | 4 | 0.1942 m / 0.0000 rad | 通过 |
| 真实裸Spin/BackUp绕过尝试 | `bare_recovery_v1` | 5 | 不评分（期望取消） | 通过 |
| plant受阻、11次重规划仍超时 | `stuck_replan_v1` | 5 | 不评分（期望取消） | 通过 |

正常与恢复到达均独立检查最终地图位姿和连续零命令。`normal_final` 31.34 s、XY 0.19808 m；抢占场景新目标 XY 0.19203 m、yaw 0.24405 rad，实际经历 FORWARD_TRACK 133、ALIGN_TO_PATH 112、FINAL_ALIGN 86 个 mode 样本，旧目标未被误报成功。

关键注入证据：

- 规划/平滑代理转发真实 backend，不生成路径；旧 request 1 被实际 hold，cancel 被拒绝，在新 epoch 已恢复运动之后才实际 forwarded。held/backend/forwarded 内容 hash 一致。旧 planner 轮使用 CDR hash，该 request 三者恰好一致，额外 `late_delivery_postaudit.json` 记录时序；新 proxy 用规范字段 JSON hash 排除 CDR padding 不稳定。该测试是迟到 result，未覆盖迟到 ACK。
- 乱序命令 seq 8 在 seq 9 已接受后重放，注入源年龄 162.10 ms，首个零命令时 163.70 ms，仍低于 300 ms TTL，具备新鲜度余量。
- typed/TF 位姿 1e-4 m 偏差被生产 controller 拒绝；去偏后重新规划并到达。未来 TF 污染下 13 个实际 controller debug 样本仍与同源时间位姿一致。
- 受阻 14.478 s 后实际 progress checker 报错，期间安装 11 个 plan revision，证明周期 replan 没有无限重置进展计时。
- Through action feedback 剩余点数从 2 变为 1，保留最终点完成 yaw/stop。
- LOST 时实际 Spin 和 BackUp 均发出 legacy 非零命令，safe 输出保持零；业务任务随后真实取消。

## 审计修正与保留失败

1. `normal_v1` 是工具 UUID numpy uint8 无法 JSON 序列化，未作为生产失败。
2. `normal_v2` 是真实生产缺陷：goal_reached 导致 gate 轮换 session，BT 在消费成功前反复撤销重规划；约 137 次终态循环后超时。生产 gate 修复封存完成 token、立即零速但保留 session，`normal_v3` 和最新 `normal_final` 均真实成功。
3. `pending_smoother_v1` 是工具过早宣布 ready、DDS 尚未发现真实 backend，第一请求失败导致 hold 注入未发生；该轮保留失败，修正后 `pending_smoother_v2` 通过。
4. 最初 wire auditor 对最近 50 ms 的 authority mismatch 豁免，被独立反例证明可误通过；最终改为必须精确匹配 active token/controller/boot/source、raw→relay 来源字段、localization/gate。所有有效旧日志重新严格评分，原文件不覆盖。
5. `localization_session_v1` 实际 action4 到达及停车，但旧 audit 将早先 LOST transition 无限沿用，得到 428 个假失败。最终以之后实际 NORMAL publication 覆盖旧健康观察，仍保留 LOST 边沿停车检查；两个负向/正向回归覆盖，最终原日志零违规。原 result=false 原样保留，results JSON 明确原判与复核判。

最终 observer 保留 50 ms DDS 跨 topic 匹配窗口、30 ms age 观测余量、2 ms 未来源时间容差；intent 400+30 ms、execution 300+30 ms。它不是传输层精确因果证明，也不是认证机制。14 项工具单测全部通过，覆盖错身份/boot/controller、未来来源时间、缺授权/raw、relay刷新来源时间、NaN、空跑、超过 50 ms 的未来 raw 来源及健康恢复判据。Codacy callable tool 当前不可用，未宣称完成 Codacy 检查。

## 交付与复现

结果索引为同目录 `epoch_navigation_integration_results_20260910.json`，每景包含目标、原始终态、实际注入、几何误差、验证事件、源/动态库 hash、完整 manifest 路径与 hash、原始及最终 audit hash。原始大日志保留在 workspace `analysis_outputs`。源码/依赖 hash/ldd 位于每景 `runtime_manifest{,_after}.json`；版本混合逐景明示，未将旧产物冒称最终同一二进制。

工具 README 给出新输出目录复现命令；冻结结果目录只读。运行需 ROS Jazzy 已构建 workspace，不能一边构建/改源码一边试验。feature 分支 base 为 `6942974`，本交付不合入主分支，待 review。

仍未覆盖的集成边界：实际 controller 迟到 ACK 与 current/pending goal cancel 竞态；controller/velocity smoother 进程重启；定位恢复 pending 期间再次 LOST 的组合；严格显式地图切换/重新加载流程。组件级单测不替代这些真实全链证据。
