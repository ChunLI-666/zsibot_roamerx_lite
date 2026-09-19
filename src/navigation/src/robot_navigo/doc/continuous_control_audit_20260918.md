# 连续控制闭环对照：巡航前向与终点微调（2026-09-18）

## 结论

**前向分阶段策略在本轮理想运动学闭环中消除了非预期倒退，但效率与终点精度存在取舍。** 旧策略背向目标时连续倒退 14.2–21.3 秒；前向策略三个种子均无倒退并完成任务。越过目标 0.35 m 时，旧策略倒退修正约需 4.2–4.8 秒，前向策略转身、向前修正再调整最终朝向需 62.6–62.7 秒。用户关于“完全禁止负 vx 会损害近目标微调”的担忧得到本轮实验支持。

这支持继续设计独立的近目标微调阶段，**并不代表已经实现或验证有界微退策略**。当前生产算法与参数未改。本轮不是0829历史全栈复现，也不是机械狗接触动力学或现场安全验收。

## 1. 配对实验

同一私有构建的当前生产 Controller/MPPI，分别加载旧策略参数和已有 forward overlay。旧策略指当前库关闭前向模式，并非0829原始二进制；overlay同时改变运动阶段、vx_min、vy_max和采样约束，是策略组合对照，不能把所有差异单独归于禁止负vx。

- 六种场景，每种三个优化器种子42/43/44，两种策略，共36条。
- 完整路径 `(0,0)→(4,0)`，终点yaw=0，间隔0.05m。每周期由生产PathHandler提取局部参考，完整目标独立保留；不以局部参考末端冒充目标。
- 初始状态：正常正向、背向π、横向偏移0.6m、朝向π/2、越过目标0.15m（容差内对照）、越过目标0.35m（容差外修正）。均从静止开始。
- 每条只创建一次控制器，保留上一轮控制序列、SG历史和噪声；允许生产状态切换正常reset。相同种子不保证两臂进入不同状态后仍有相同采样序列。
- 10Hz连续反馈，预算180秒模拟时间；MPPI预测时域仍为3秒。每个子进程另有120秒墙钟保护。
- 两臂使用同一ZSL-1模型footprint和padding，校验运行时实际多边形及文件SHA；它不是已标定的真实步态摆腿包络。
- 空旷静态20×20m地图。真实InflationLayer对象提供元数据，不启动地图更新，不重新膨胀。恒定body twist按SE(2)精确积分，每10ms检查footprint。
- 使用实际StoppedGoalChecker：XY≤0.25m、yaw≤0.25rad、线/角停稳≤0.01；当前执行命令也须停稳。连续六次采样满足，即首次到末次至少0.5秒，才判成功。

这比单周期评分保留了优化状态反馈；仍未执行全局规划重规划、BT、实际速度平滑器、epoch gate、SDK、传感器更新或惯性动力学。执行端仅模拟既有死区与速度上限。

## 2. 结果

| 场景 | 旧策略成功 | 前向策略成功 | 旧策略到达时间/s | 前向策略到达时间/s |
|---|---:|---:|---:|---:|
| 正常正向 | 3/3 | 3/3 | 28.1–35.5 | 44.2–46.0 |
| 背向目标 | 2/3 | 3/3 | 36.1–57.9 | 89.4–90.0 |
| 横向偏移0.6m | 1/3 | 3/3 | 33.7 | 81.2–81.9 |
| 朝向偏差90° | 1/3 | 3/3 | 48.7 | 73.8–74.3 |
| 越过目标0.15m | 3/3 | 3/3 | 2.3–5.4 | 0.5 |
| 越过目标0.35m | 3/3 | 3/3 | 4.2–4.8 | 62.6–62.7 |
| 合计 | **13/18** | **18/18** | 仅统计成功案例 | 仅统计成功案例 |

越过0.15m已经在位置容差内，前向策略的0.5秒是停稳确认，不是精确位置修正。三个种子是算法敏感性重复，18/18不能当作广泛环境可靠率。

| 场景 | 旧策略累计后退/m | 旧策略最长连续倒退/s | 前向策略后退 |
|---|---:|---:|---:|
| 背向目标 | 1.257–2.279 | 14.2–21.3 | 0 |
| 越过目标0.35m | 0.293–0.355 | 3.0–3.6 | 0 |

其余旧策略场景也可能出现小幅倒退；完整数据见结果JSON。前向策略全部18条的执行负vx和raw负vx均为零。两臂均无控制器异常与几何碰撞标记，但**空地图零碰撞不能证明窄门和障碍环境安全**。

前向策略一般停在约0.19–0.20m位置误差，部分yaw约0.249rad，接近约定接受边界；旧策略成功样本位置更接近终点。这不是精度提升。前向策略正常正向也较慢，需要后续单独评估约束采样、死区和速度配置的效率影响，不能将本轮参数视为最优调优结果。

旧策略失败的五条：背向一条仍距目标1.15m；侧偏两条及90°两条已进入XY容差，但yaw分别约0.35–1.07rad，不满足最终朝向。后四条出现约137–155秒静止。它们是180秒预算内未满足完整到达条件，不应统一称为路径不可达，也不能把正常旋转判成停滞。

![连续反馈轨迹与执行速度](/home/charles/project/colcon_ws/src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/doc/continuous_control_comparison_20260918.png)

图使用seed42，位置坐标轴比例各自缩放；下排是执行body vx。轨迹与指标截止最后记录的命令前位姿，终止行不再累计一个完整dt，避免碰撞中止或成功结束时高估后退距离。模式订阅记录可辅助检查，不把异步topic时间视为内部状态切换的精确因果时间。

## 3. 对0829归因的增量

[上一轮成本报告](/home/charles/project/colcon_ws/src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/doc/mppi_reverse_cost_audit_20260918.md)证明软惩罚生效但不能保证前向。本轮进一步证明：同一实际实现，在明确完整目标和连续warm state反馈下，可以产生长达数十秒的倒退，不是单次输出负号的统计假象。

不过环境、初始状态、实际参数和随机历史不等于0829记录；本轮没有历史障碍层，不能解释当时CostCritic逐项权衡。原始完整历史根因仍不能唯一确定。本轮也未加入逐周期所有候选成本遥测；只有连续实际控制输出与状态辅助记录。单周期分项工具与本轮连续反馈证据需分别理解。

## 4. 下一开发阶段：巡航与近目标微调

建议保持当前巡航前向约束，在独立实验overlay实现 `CRUISE → APPROACH → FINAL_ALIGN → SETTLED`，以本轮36条作为回归基线：

1. APPROACH进入不能只看直线目标距离，还需路径末端剩余长度、目标/定位身份有效、制动停稳。进入/退出距离有迟滞，目标或定位身份变化立即撤销旧阶段预算。
2. 只有APPROACH允许有限负vx。速度上下限、采样投影、critic开关、最终预测扫掠、速度平滑和gate的阶段权限必须一致；不能只放开末端Twist或MPPI一个参数。
3. 后退距离、持续时间和次数分别预算，预算跨周期累计；后方有效地图必须覆盖运动、制动和延迟扫掠，未知或超预算即停稳后重规划/显式恢复。首轮数值应作为仿真假设，不冒充实机标定。
4. XY进入容差后停稳，转FINAL_ALIGN；若位置逸出退出阈值再回APPROACH。始终保留当前0.25m/0.25rad验收基线，精确泊车另立指标。
5. 验收必须同时覆盖近目标障碍、后方动态障碍、横向偏差、目标更新、定位失效、预算耗尽、延迟与噪声。比较到达、累计后退、时间、停稳及扫掠，不只检查负vx是否出现。

上述是下一阶段设计边界，当前提交仅实现诊断能力，没有放开生产倒退权限。

## 5. 验证与复现

- 36条实际生产Controller反馈运行；所列输入、私有MPPI库、harness前后SHA一致。未将它称为全部系统依赖冻结。
- 背向、侧偏、越界终点各两臂，seed42共六条原样复跑；15个数值轨迹/命令字段最大差异为零。
- harness独立footprint自检通过；11项Python评估/几何测试通过，1项依赖旧目录布局的实际Costmap加载测试跳过。本轮36条运行均实际加载并校验模型footprint。
- Python语法及Git whitespace检查通过。当前无Codacy工具，未执行。
- 独立只读subagent复核未发现推翻结论的指标错误，并要求保留时间、精度与历史归因限制。

[完整运行结果](/home/charles/project/colcon_ws/analysis_outputs/continuous_control_20260918/matrix/results.json) · [复跑证据](/home/charles/project/colcon_ws/analysis_outputs/continuous_control_20260918/matrix/repeat_validation/results.json) · [紧凑结果](/home/charles/project/colcon_ws/src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/doc/continuous_control_summary_20260918.json)

复现入口为 [continuous_audit.py](/home/charles/project/colcon_ws/src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/controller_feedback/continuous_audit.py)。先用同目录CMake构建harness，并按上一轮[成本工具说明](/home/charles/project/colcon_ws/src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/mppi_cost_audit/README.md)准备当前生产MPPI私有安装，再执行：

```bash
python3 continuous_audit.py --root NEW_OUTPUT --binary HARNESS_BUILD/controller_feedback --mppi-prefix PRIVATE_MPPI_INSTALL
python3 validate_continuous_repeat.py --root NEW_OUTPUT --mppi-prefix PRIVATE_MPPI_INSTALL
python3 analyze_continuous.py --root NEW_OUTPUT
```

`--root`必须是新目录。新增脚本生成完整路径/参数、运行配对矩阵、评估并保留全部失败；不沿用旧工具会删写的输出目录。开发基线`45882dd`，代码在canonical checkout的`feature/forward-aligned-navigation`提交，生产导航源码和默认参数未改，用户review前不合入。
