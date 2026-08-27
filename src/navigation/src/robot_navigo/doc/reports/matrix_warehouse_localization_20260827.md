# Matrix Warehouse 定位批量评测报告

## 结论

本轮 Warehouse 定位批测有效执行，但结果未通过。当前只能把冻结的历史地图用于
算法诊断，不能将当前 Lightning 定位配置作为规划控制批测的合格定位输入。

- 当前源码建图不可复现历史地图，状态为 `FAIL`。
- 归档地图文件校验通过，但这不等于当前源码重新建图成功。
- 3 个初始化窗口全部未达到绝对精度门限。
- 全程回放中定位连续覆盖率仅 `9.03%`，最大 pose 断档 `351.71 s`。
- 其他 Matrix 场景缺少独立地图和评测 bag，本轮均为 `SKIPPED/unavailable`。

## 输入与可信边界

| 项目 | 值 |
|---|---|
| 场景 | Warehouse，scene ID 1 |
| 地图 | `scene_terrain_wh/20260315_lightning` 冻结归档地图 |
| 地图来源 | `archived_baseline` |
| 当前源码建图可复现 | 否 |
| 建图 bag | `rosbag2_2026_03_15-17_13_12` |
| 独立定位 bag | `matrix_warehouse_loop_final_20260827/closed_loop_bag` |
| GT | `/odom/mujoco_odom`，仅录制和离线评估 |
| 固定对齐 | 建图首帧计算一次完整 SE(3)，评测过程不拟合轨迹 |

评测器重新计算并核验了地图、建图 bag、独立定位 bag、回放输出 bag 和 replay
manifest 的完整内容 SHA256。GT 未发布导航 TF，也未进入 Lightning 输入。

## 初始化结果

| 起始偏移 | 脚本状态 | 有效初始化 | 初始化耗时 | 初始 XY 误差 | 初始 yaw 误差 |
|---:|---|---|---:|---:|---:|
| 0 s | `pose_valid=true` | 否 | 7.79 s | 0.283 m | 6.18 deg |
| 120 s | `TIMEOUT` | 否 | >60 s | 无输出 | 无输出 |
| 240 s | `TIMEOUT` | 否 | >60 s | 无输出 | 无输出 |

有效初始化门限为 XY 不超过 `0.25 m`、yaw 不超过 `5 deg`。`pose_valid=true` 只是
状态机输出，不能替代绝对误差判定。

## 全程跟踪结果

| 指标 | 实际值 | 门限 | 结果 |
|---|---:|---:|---|
| ATE RMSE | 0.313 m | 0.15 m | FAIL |
| ATE P95 | 0.469 m | 0.25 m | FAIL |
| ATE max | 0.487 m | 0.50 m | PASS |
| yaw P95 | 6.38 deg | 5 deg | FAIL |
| 1 m RPE RMSE | 0.141 m | 报告项 | - |
| 非物理跳变 | 3 次 | 0 次 | FAIL |
| pose 平均有效频率 | 0.91 Hz | 10 Hz | FAIL |
| pose 最大断档 | 351.71 s | 0.50 s | FAIL |
| 连续时间覆盖率 | 9.03% | 98% | FAIL |

连续覆盖率只累计相邻 pose 间隔不超过 `0.5 s` 的区间。旧算法按首尾时间跨度
计算，会被末尾恢复的一帧误导为接近 100%，该 fail-open 已修复并增加回归测试。

## 当前判断

初始 0 s 窗口在约 8 秒后能给出定位，但随后全局匹配不能持续跟上；120 s 和
240 s 窗口无法初始化。debug 证据如下：

- LIO 收到并处理 3946 帧，丢弃 0 帧，约 9.99 Hz；
- LidarLoc 收到 265 帧，只处理 51 帧，跳过 142 帧、丢弃 72 帧；
- 全局匹配约 0.129 Hz，PGO 融合约 0.048 Hz，单次匹配最长约 2.67 s；
- 状态按配置从 `ACTIVE` 进入 `DEGRADED/LOST`，原因为全局匹配 holdover 过期；
- 120 s 和 240 s 始终停在候选 `1/3`，归档地图只有一个 `start` 功能点。

LIO 前 100 秒相对误差较小，之后逐渐漂移；目前没有瞬时 LIO 发散证据。主要根因链
是“单一功能点导致非起点初始化候选不稳定”和“持续 NDT 吞吐远低于输入，导致
全局锚点过期”。不能通过放宽状态机超时掩盖该问题。

另有一个必须先排除的坐标契约风险：运行时声明 tracking frame 为 `livox_frame`，
而当前固定对齐声明为 `imu`；归档地图又没有 `map_frame.yaml` 明确旧版 start pose
所属 frame。0 s 的固定 `0.283 m / 6.18 deg` 偏差可能包含该契约误差。

下一组最小消融依次为：统一 tracking frame/外参后复测 0 s；为 0/120/240 s 注入
正确初始位姿以隔离 FP 搜索；增加沿路线分布的 functional points；最后再降低
LidarLoc 负载并要求队列无丢弃、全局匹配不再超时。

## 产物

服务器正式运行目录：

```text
/mnt/data/regression/matrix_localization/matrix_lightning_localization/warehouse_20260827_154300
```

关键文件：

```text
LOCALIZATION_REPORT.md
warehouse/map/map_ground_truth_alignment.json
warehouse/init/cases/*/localization_accuracy.json
warehouse/tracking/localization_accuracy_v2.json
warehouse/tracking/replay/lightning_debug_report.json
```

## 对规划控制批测的影响

1. Lightning 正式闭环 case 必须先通过定位门控；失败时不发送 goal。
2. 先用 MuJoCo GT 建立规划控制隔离基线，但该模式必须与 Lightning 模式互斥。
3. GT 基线通过只能证明规划控制链，不代表 Lightning 定位闭环通过。
4. 在定位连续性修复前，不执行长路线 Lightning 闭环性能结论。
