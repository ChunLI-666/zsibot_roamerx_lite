# ZSL-1 goal 开发进展与剩余验收（2026-09-10）

总目标仍 active，未达到完成条件。沿用 `feature/forward-aligned-navigation` 和 `feature/descriptor-relocalization-verification`，没有创建 worktree、合并或向实机发送命令。以下补充 2026-09-08 首轮报告，不能覆盖旧实验的条件和失败记录。

总目标：面向 ZSL-1 办公室/厂房自主巡检，形成可替换非学习式描述子的统一重定位与跟踪流程，以及符合真实机体、步态和执行能力的前向优先导航。最终验收要求可靠定位、无非预期后退、足迹/扫掠空间与实际执行一致、到达停稳、失效停车并可恢复。桌面已有数据实验之后，还须完成独立参考、RK3588 和实机验收。

| 阶段 | 本轮达到的状态 | 退出条件中尚缺的内容 |
|---|---|---|
| P0 模型与环境基线 | 官方模型站姿包络、参数生成及实际加载断言；三个现场局部地图重建 | 实测步态/载荷/坐标关系和现场净宽 |
| P1 通用重定位流程 | 可替换检索接口、粗精定位/验证/提交、epoch 与恢复融合；离线和隔离在线丢帧恢复 | 完整几何退化验证、板端时限及独立定位真值 |
| P2 规控与失效保护 | 前向约束控制器、对齐/停稳、定位失效立即零速与单调 watchdog | 新定位 epoch—新规划—放行握手及完整任务恢复 |
| P3 已有数据闭环 | 新模型控制器矩阵和冻结现场上下文验证 | 动态感知与全导航栈闭环、留出路线及独立参考 |
| P4 板端与实机 | 尚未验收 | RK3588 资源/延迟和整机步态/路线验证 |

## P0：已有模型依据的 footprint 与现场环境

官方固定版本的 URDF 和 17 个 STL 与本地模型匹配；Matrix xgb 的相同网格和 home 关节姿态提供了可复现的模型输入。该姿态下原点相对包络约为 x=[-0.359787,0.282187] m、y=[-0.188144,0.188162] m，长 0.641974 m、宽 0.376306 m。前后不对称，不能重新居中。实验保守矩形另加每侧 0.01 m padding，全局/局部使用同一数据来源，膨胀半径按外接半径和栅格推导为 0.50 m。

这只覆盖模型 home 站姿。现有 0829 数据没有关节轨迹，尚缺实际摆腿包络、导航基准与模型原点的实物对应、传感器/载荷外伸及门框净宽。因此生产默认 footprint 没有被一个未经实测的尺寸直接替代；模型实验 overlay 和验收工具已提供，并明确 `hardware_deployment_ready=false`。P0 完整退出条件尚未满足。

三个后退片段额外重建了真实录制的局部 costmap 全图与增量，最新消息接收年龄分别为 0.295/0.096/0.095 s。增量源时间戳为 0 是已有 publisher 的行为，重建使用接收顺序并明确记录此限制。冻结现场环境优于旧的空地图几何试验，但仍不等于动态感知或新 footprint 膨胀的实时全栈闭环。

模型和现场上下文图：

![模型站姿与现场局部地图](/home/charles/project/colcon_ws/analysis_outputs/zsl1_goal_20260910/zsl1_footprint_and_recorded_context.png)

模型工具及审查已提交 `658c282`，12 项几何/模型测试通过，官方 18 文件和四份生成产物重现一致。完整实验参数经隔离 ROS 节点解析，全局/局部 footprint、padding、inflation 一致；详见 [模型包络报告](zsl1_model_envelope_review_20260910.md)。

### 新尺寸下的控制器与历史上下文验证

最终有效模型矩阵中，新策略的原始 36 个可达用例全部到达；增加宽门用例后为 42/42，同一生产二进制关闭策略为 22/42。三个实际录制 costmap 上各三个种子的冻结局部路径，开启策略 9/9 到达、关闭 6/9；两组这九个场景均无初始重叠、越出地图或独立判定的碰撞。它们单列，不混入开阔矩阵到达率，也不表示整个历史任务复现。

模型加 padding 后宽约 0.3963 m：0.36 m 门洞新策略安全停车；0.44 m 门洞在这三次实验中通过。门洞与旋转扫掠中的旧策略碰撞记录均保留。初始就与障碍重叠的用例单列为无效初态，新策略零命令且不会误记为到达；不能把它混入“运动造成碰撞”的计数。

验证实际发现并修复了 YAML 输入适配缺陷：block-style polygon 被错误转换成参数字符串，Costmap2DROS 回退为 radius=0.1 m。该轮新尺寸实验已判无效并保留。最终工具逐项核对实际加载的顶点与预期 padded polygon，一旦回退或不一致就在任何控制计算和评分前退出；评分还单独检查 sidecar。有效结果来自 `analysis_outputs/feature_navigation_envelope_20260910/zsl1_model_verified/`，不能使用早期 `zsl1_model/` 的结果。

## P1：通用状态流程与真实在线回放

`lightning-lm` 提交 `253a0f0`（本轮 base `1e51919`）把 pd1 封装为可替换 `PlaceRetriever`，共同流程执行 WAIT_INPUT → COARSE_SEARCH → FINE_ALIGN → VERIFY → COMMIT → TRACKING。查询携带 epoch、地图加载实例、源扫描时间和单调时间预算；重复/过期输入不能参与投票或提交。暂不引入学习式描述子，也没有把 Scan Context/STD/BTC 冒充已接入。

健康状态丢失后，需要新一轮验证提交并重置绝对位姿融合历史；新的有效融合结果之后才进入恢复确认，随后对导航报告正常。仅 LiDAR 内部进入 TRACKING 不能直接作为导航放行依据。八套 CTest、六个严格日志证据测试通过；0718 地图对 0829 留出片段完整 260 帧重复初始化仍得到 18 次接受，接受数量不是独立真值成功率。

另用 2025-11-21 的正例地图/数据做隔离在线实验：保持原 CDR 与接收时间不变，仅移除 45 秒前缀中 +8 至 +12.5 秒的 45 帧 LiDAR，保留全部 8999 条 IMU。派生 bag 独立读回与所选源消息流哈希一致。冻结生产二进制以 1 倍速、模拟时钟、localhost ROS domain 185 运行，没有控制器或执行器桥接。

实际日志与 DDS 观察走通 ACTIVE → DEGRADED（lio_stale）→ LOST（退化约 3.000 秒）→ RELOCALIZING → 新 epoch 3 / commit 2 → 融合重置与有效结果 → ACTIVE_RECOVERED → ACTIVE。严格审计要求提交、epoch、源时间、reset/ready 对应且不能复用旧提交，`--require-recovery` 通过。回放 EOF 后再次 LOST 是停止输入的预期结果。证据位于 `analysis_outputs/zsl1_goal_20260910/online_dropout_trial/`，派生数据清单位于相邻 `online_dropout_bag/`。

该首轮在线 LOST → ACTIVE_RECOVERED 为 14.042 秒，随后健康确认 4.679 秒，总计 18.721 秒才回到 ACTIVE。日志和真实 DDS 接收时间相互对应，详见 `recovery_metrics.json`。这不能用离线单查询约半秒的延迟替代，也尚未证明满足目标路线时限。

这一在线结果仍暴露图优化返回契约的检查需求：恢复后的 LM 有 Cholesky 重试日志，中间重试不直接等于最终失败，但旧 PGO 确实没有检查 Optimize 的最终结果。后续数值失败补丁与复测单列，不能将这次状态恢复表述为已证明数值融合始终有效。

旧快照仅保存 online/offline 和主库，没有独立冻结当时的 miao/Pangolin/消息类型动态依赖。原始及工具审查回放均在新 miao.core 链接产物生成前结束，旧行为日志保留有效，但不能声称旧目录单独即可重现全部数值依赖。新版 `numerical_guard/frozen_binary` 补齐项目动态库、哈希和 `ldd` 加载路径；最终数值复测使用该目录。

数值补丁之后，LM 不再应用失败求解的步长，全部重试仍无有限解时明确失败；PGO 拒绝失败/非有限结果并清理绝对图，新提交融合失败则重新定位。五项定向测试覆盖全失败、重试成功、零步收敛、非有限步长/建系统失败、失败清图后同优化器继续有效融合；九套定位 CTest 通过。此补丁涉及共享 miao 的 LM，影响范围包含使用它的建图/回环，未重跑完整 SLAM 数据集，不能将有限结果检查当作完整收敛/几何退化保证。

最终在线证据在 `analysis_outputs/zsl1_goal_20260910/online_dropout_trial_numerical_guard/`。相同 45 秒传感器输入和配置，完整 snapshot 运行 52.49 秒；运行器、严格生命周期审计分别通过。仍为 2 commits、2 LOST（含 EOF）、1 条新 epoch 恢复链；新提交源时间 1763715084.47296。LOST → ACTIVE_RECOVERED 为 15.133 秒，再经 4.674 秒确认，总计 19.807 秒到 ACTIVE。本轮有 16 条中间 Cholesky 重试日志、0 条最终 PGO_REJECT；正常重试未被一刀切误拒绝，但不代表已覆盖真实数值失败，故另有故障注入测试。

最终记录的 11 个输入文件和 26 个 executable/shared-library 文件在运行后重新核对全部未变，见 `source_and_binary_unchanged_check.json`。这一次桌面运行不是受控性能比较，18.721 与 19.807 秒不用于声称改进/退化的因果结论。

详见 [重定位流程报告](/home/charles/project/colcon_ws/src/lightning-lm/doc/relocalization_lifecycle_review.md)。上述实验是桌面在线回放，不是 RK3588 或独立定位真值验收。

## P2：定位失效后的输出门控

提交 `ca7a192`：收到非 NORMAL 状态立即发布零速；单调时钟 watchdog 在 ROS /clock 暂停时仍生效；非法/非有限速度归零；恢复状态不重放缓存命令。9 项行为测试覆盖真实 executor 与 DDS 输入输出，未启动执行器桥接。

仍未实现跨进程 epoch、定位提交、新全局路径与速度输出之间的握手；收到恢复后的新命令并不能证明它属于新规划。此项保留为 P2 下一工作包，不能由已有 UInt8 状态门替代。

## 当前构建与审查证据

`robot_navigo` 构建通过；CTest 中 `test_nav_safety_gate`、`test_zsl1_envelope`、`test_recorded_costmap`、`test_navigation_contract` 四套通过。日志保存在 `analysis_outputs/zsl1_goal_20260910/robot_navigo_build.log` 和 `p0_p2_ctest.log`。重建脚本另有 7 项成本编码/更新/时序测试，提交 `9391748`；其输入流与输出哈希均已保存。

Codacy MCP 不可调用，未执行该项检查。这里的模型、ROS 节点和控制器测试不证明 RK3588 时限或实机可靠性。完整目标的独立真值、路线覆盖、板端和实机验收继续保留。

定位回放工具另有 [独立审查报告](localization_trial_tools_review_20260910.md)：真实 MCAP 原样读写、源文件未变、清理隔离和节点提前退出误判的 10 项测试通过。工具执行成功与定位行为成功分开验收；回放正常结束之后仍需通过严格生命周期分析器，不能仅依据进程退出码给定位判成功。

## Feature 分支交接

下表冻结实现与测试对应的提交，随后导航分支仅增加本交接文档。完整新提交列表、原始 feature base、本轮 base、diff 统计、测试和剩余风险保存在 [分支清单](feature_branch_handoff_20260910.json)。统计包含生成的模型/数据清单、结果 JSON 与文档，不全部是生产代码。

| 仓库/分支 | Feature base | 本轮 base → 实现 review HEAD | 本轮差异 |
|---|---|---|---|
| zsibot_roamerx_lite / `feature/forward-aligned-navigation` | `69429743ffc8a3f912eea02a4280e90cddd4431f` | `5c49587` → `c339d1c` | 37 文件，+5486/-55 |
| lightning-lm / `feature/descriptor-relocalization-verification` | `c19f20b4`（完整 SHA 见清单） | `1e51919` → `72532e8` | 27 文件，+2101/-45 |

本轮关键提交：导航 `0d5e492` 定义目标、`ca7a192` 失效门控、`9391748` 现场 costmap、`658c282` 模型包络、`0632e26` 新尺寸控制验证、`931929a` 复现命令修正、`c339d1c` 在线实验工具；定位 `253a0f0` 生命周期、`72532e8` 数值失败契约。定位 `94748d6` 是此前保留的用户评估工作基线，不能当作本轮从零新增的成果。

下一开发包优先补定位 epoch—新规划—速度放行的跨进程握手，以及全导航栈的失效/恢复试验；实机尺寸/摆腿资料到位后再冻结可部署 footprint。共享 LM 的回环调用方尚未完整处理优化失败返回值，完整 SLAM 回归和该调用方处理也需单列跟进。总 goal 保持 active；本报告不申请合并，也不表示已经通过整机验收。
