# 转向—跟踪过渡与独立 footprint 闭环审计

本轮以 `e826d6f` 为起点，在 `feature/forward-aligned-navigation` 的 canonical checkout 开发，保留 pd1 定位主线。新增显式运动阶段、跨阶段停车过渡和独立几何审计，与受限 BackUpEpoch 的授权接口一起提交供 review；不合入主分支，不连接实机，不发送 LCM。

## 结论与适用范围

真实 Nav2 控制器、MPPI、规划器、BT、平滑器、安全 gate 在 localhost ROS domain 中运行。先前八场景矩阵的 **8/8 几何审计、8/8 运动阶段审计通过，7/8 综合通过**；低角减速度场景仍有 11 条旧回调时间线的 wire 审计失败。这些失败未删除，也未用“到达终点”覆盖权限时序结果。

本轮修复保证跨运动阶段的制动来源与执行约束一致，并保留 MPPI 预测内的合法曲线跟踪。它没有证明同一 TRACK 内的平滑后曲率与 MPPI 动力学完全一致，更不是实机刹车距离或机械狗步态包络认证。

## 可复现缺陷与方案

原实现在 ALIGN_TO_PATH 退出后立即允许平移，而平滑器仍保留旧角速度。仅将平滑器角减速度从 `1.5` 调低到 `0.2 rad/s²`、保持退出角 `0.20 rad`，可得到原始命令 `[vx=0.051179, wz=0]`，实际 safe 命令 `[vx=0.051179, wz=-0.038860]`，下一周期仍有 `wz=-0.028860`。该次共有 4 个 relay、4 个 safe 平移与转动并发周期。原始失败保存在 `slow_decel90_v1/`，没有改变阈值来掩盖它。

这里的 `0.2` 是仅作用于平滑器的故障注入；MPPI 原预测角减速度仍为 `1.5`。生产部署仍须统一预测与执行的加减速度约束。新的方案不通过直接裁掉最终 Twist 的某一轴来声称模型一致：

1. `Controller::getExecutionMotionPhase()` 增加默认兼容接口。MPPI 输出 HOLD、ROTATE、TRACK；显式 BackUp 输出 TRANSLATE。TRACK 可以包含预测内的 `vx+wz` 曲线。
2. 控制器在旋转与平移切换时先输出 HOLD，使用新鲜原始 odometry 判断停稳。epoch 控制不再先将小速度按阈值清零。角速度停车条件来自 goal checker（当前 `0.01 rad/s`），线速度阈值为 `0.02 m/s`。
3. 平滑器在加速度限制**之前**把换阶段目标设为零，沿原阶段逐轴制动，不增幅、不反号。即使新目标又要求恢复旧阶段，制动也锁存到输出严格零；经过一个零输出周期才能启动新阶段。
4. `EpochCommand` 保留原 token、source monotonic 时间及序号，附带 `source_motion_phase`、`transition_braking`、`braking_from_phase`。gate 验证制动来源和逐轴约束，relay 不能更新授权年龄。`execution_kind=TRACK/BACKUP` 与运动阶段是两个不同维度。

## 实验与几何证据

夹具以 `/cmd_vel_safe` 驱动理想 SE(2) 运动学，再发布 TF、LaserScan 与实际执行速度的 odometry。物理运动积分继续使用回调接收时间；模拟 watchdog/冻结停车时 odometry 反映已执行的零速度。真实 Nav2 消费这些反馈，但没有 SDK、关节惯性、地面打滑或步态模拟。

**时间模型限制：** Fixture每次积分使用 `dt=min(wall_delta,0.1)`，watchdog在当前回调末端判定。长调度停顿时，它没有模拟完整wall时间内先持续运动、再在授权到期时停车的分段过程。因此几何通过只适用于这个受限SE(2)模型，不能证明长调度停顿下完整执行动力学安全。各场景的100ms截断次数已保存在附属JSON的 `plant_time_model`，不将截断区间误当作真实机器人停止。

footprint 来自 `params/zsl1_model_envelope.yaml` 的 ZSL-1 模型姿态包络，加四周 1 cm padding 后为 **0.661974 × 0.396306 m**，相对 base_link 的前后长度不对称。每次从实际 global/local costmap 的 published footprint 校验加载结果，未仅检查 YAML 字符串。模型文件 SHA256 为 `94965f8269a74137490fa07416ac231824831b5601ef035d8c686677d99c5855`。

地图为 8×8 m、0.05 m 分辨率的可重复生成静态占据图，包括开阔场地、0.8 m 门和 0.35 m 门。独立 oracle 使用多边形—栅格 SAT；连续旋转/平移扫掠的顶点采样间距不超过 2 mm，并按半间距增加保守包络（本矩阵最大约 1 mm）。障碍、未知、越界和数值不确定分别记录；几何“不确定”也不能计为通过。oracle 不调用生产的 `footprintFree/sweepFree`。

前一版矩阵：

| 场景 | 结果 | XY误差 m | yaw误差 rad | 几何/阶段 | 旧 wire |
|---|---|---:|---:|---|---|
| 初始90°，正常角减速度1.5 | 到达停稳 | 0.1965 | 0.2466 | 通过/通过 | 通过 |
| 初始90°，低角减速度0.2 | 到达停稳 | 0.1910 | 0.2362 | 通过/通过 | **11项失败** |
| 初始180° | 到达停稳 | 0.1962 | 0.0464 | 通过/通过 | 通过 |
| 终点yaw=90° | 到达停稳 | 0.1972 | 0.2451 | 通过/通过 | 通过 |
| 0.8m门，初始90° | 穿门到达停稳 | 0.2346 | 0.0253 | 通过/通过 | 通过 |
| 0.35m门 | 拒绝不可通行路径、保持零速 | — | — | 通过/通过 | 通过 |
| 定位失效及新epoch恢复 | 停车后恢复到达 | 0.1947 | 0.0462 | 通过/通过 | 通过 |
| 对角线路径 | 到达停稳 | 0.1997 | 0.2479 | 通过/通过 | 通过 |

所有八例实际轨迹确认碰撞数、扫掠不确定数、反向距离均为零；最终安全命令为零。这里“到达”沿用当前 goal checker 的 `0.25 m / 0.25 rad` 容差，并非厘米级泊车。

0.8 m 门产生 **7 个 raw、6 个 relay/safe 合法耦合 TRACK 命令**，构成曲线回归证据。名为 `curved_track` 的对角线路径实际选择了分阶段运动，不能用名字冒充曲线覆盖。

原实验退出角 `0.20 rad` 在门前安全停住、未能完成穿门；实验 overlay 将退出角收紧至 `0.05 rad` 后成功。只修改 `forward_alignment_experiment.yaml`，没有改变生产 `navigo_params.yaml`。正常90°和低减速度反例仍显式覆盖 `0.20 rad`，避免把修复效果和调参混淆。

## 权限时序观察与最后复测

旧低减速度失败发生在不同 topic 的 callback 成批交付时：raw 三个100 ms间隔的源序号在约10 ms内被观察到，inactive execution 源时间与 safe 回调时间相差约48 ms。旧 safe Twist 没有发布时间证据，无法只靠 observer 到达顺序判定 gate 是否在撤权后继续发非零。

新增观察仅补元数据：DDS source/received timestamp、callback 的 wall/monotonic bracket、本地 localization publish 的 before/after bracket。保留所有原 `monotonic_ns`、plant 积分与旧 `callback_audit`；wire 审计独立投影到经验证的发布时刻。采样 bracket 超100µs、wall/monotonic offset漂移超过1ms、DDS时间缺失/非法时明确不可评分。匹配窗口仍50ms；新DDS source TTL不沿用旧30ms observer余量，边界落入时钟不确定区也不能判通过。

两例已在 BackUp `.75s` 距离反应预算修复后的冻结二进制上串行复测，均在运行中保持 source/二进制/参数哈希不变。此次运行早于后续 BackUp 独立控制频率参数调整，TRACK仍为原10Hz，不将不同版本结果混作同一hash验证。结果见附属 JSON 的 `final_serial_replays`。

| 串行复测 | action/停稳 | 几何/阶段 | 旧callback wire | 新DDS wire |
|---|---|---|---|---|
| 低角减速度0.2 | 通过，54.00s | 通过/通过 | 0项失败 | **不可评分** |
| 正常角减速度1.5 | 通过，54.88s | 通过/通过 | 0项失败 | **不可评分** |

两例DDS不可评分均源于local publish bracket超过预设100µs精度限，分别10次/18次，最大134.972µs/209.390µs；DDS自身clock offset spread约18/19.5µs，最大观察排队52.2/52.3ms。未放宽阈值或修改历史结果，因此本轮不能宣称严格DDS wire闭环已通过。旧callback零失败只是串行复测的独立观察，不能反向证明此前11项失败均为积压。

## 验证与未完成项

- C++ phase 11项测试通过：双向换轴、严格零周期、极小残余、反馈先归零、合法曲线、耦合制动、伪造/增幅/反号拒绝、非有限数拒绝、制动锁存。
- Python 独立审计32项和地图/积分2项测试通过：包含空轨迹不通过、障碍位于footprint内部、端点之间碰撞、旋转角扫掠、缺失授权、过期/错误来源、350ms观察积压、真实源过期、时钟失效与边界不确定反例。
- 所有实验保存运行前后 source/二进制/依赖库/参数/地图 SHA256；旧矩阵均无运行中变更。
- 消息schema与Controller虚接口有变更；部署须重编依赖这些接口的组件/插件并统一版本，不能将新header与旧运行库混装。测试运行的实际依赖库已记入manifest。
- Codacy 工具在当前会话不可用，未运行。

仍需真实录制点云/地图场景回放与上线前步态尺寸、足端摆动、定位延迟、SDK停止语义/刹车响应校准。当前检查静态二维地图及模型包络，不能替代门框高度、台阶、动态障碍和实际地面条件验证。同一 TRACK 中平滑器引起的曲率滞后仍是后续预测约束一致化事项。

原始证据根目录：`/home/charles/project/colcon_ws/analysis_outputs/epoch_geometry_20260910/`。附属 JSON 保存各case绝对路径与trace SHA256，不把大体积原始trace提交进源码仓库。
