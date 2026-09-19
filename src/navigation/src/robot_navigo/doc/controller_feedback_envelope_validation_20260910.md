# ZSL-1 包络与规控闭环验证（2026-09-10）

本轮将测试足迹改为读取外部模型配置，生产 costmap 与独立碰撞 oracle 使用同一份运行时有效凸多边形。最终 **新模型原核心闭环 36/36 到达局部端点，对照 feature-disabled 为 16/36；现场冻结 costmap 上额外 9/9 对 6/9**。这里执行实际 C++ MPPI 插件及 StoppedGoalChecker，新命令经过死区/限幅驱动理想 SE(2) 运动，再反馈下一帧姿态；没有执行 BT、progress checker、NavigateToPose action、实时感知或四足动力学。

开发位于 `feature/forward-aligned-navigation`，原分支 base `6942974`。本轮工具修改在 `658c282` 上完成；模型输入由 `658c282` 提供，现场 costmap 重建工具由 `9391748` 提供。未合入主分支，未连接实机、未启动 UDP 桥接、未修改主机网络。三个对照使用 localhost ROS domains 179/180/181，加载输入回归使用 182。

## 冻结输入与实际加载

| 项目 | 历史测试矩形 | ZSL-1 模型矩形 |
|---|---:|---:|
| 未 padding X 范围 / m | [-0.300000, 0.300000] | [-0.359787, 0.282187] |
| 未 padding Y 范围 / m | [-0.150000, 0.150000] | [-0.188144, 0.188162] |
| padding / m | 0.01 | 0.01 |
| 有效长 × 宽 / m | 0.620000 × 0.320000 | 0.661974 × 0.396306 |
| 仓库静态地图 inflation support / m | 0.40 | 0.50 |
| inflation cost scaling | 3.0 | 3.0 |
| 仓库 / 合成门洞 / 现场地图分辨率 / m | 0.05 / 0.02 / 未运行 | 0.05 / 0.02 / 约 0.05 |

模型矩形保留不居中的 base_link 原点。它是官方碰撞网格在选定模型姿态下的保守外接矩形，并非现场测量的步态、摆腿和载荷动态包络。Matrix home keyframe 不是实机 joint feedback；URDF 不包含额外传感器/载荷；BASE_LINK 到导航 base_link、根姿态水平及实际安装关系仍需物理核对。0.01 m padding 沿用实验配置，不能解释为已测量的不确定性裕量。新几何扩大后，仓库 inflation 支持域随模型 schema 扩大至 0.50 m，未把这部分参数变化隐藏为纯控制算法变化。

配置文件及 SHA256：

- `scripts/controller_feedback/legacy_footprint.yaml`：`f97f2259bb5a58ed26c508387966eca60b5fd8e022c0431f66f6b26f03622c42`。
- `params/zsl1_model_envelope.yaml`：`94965f8269a74137490fa07416ac231824831b5601ef035d8c686677d99c5855`。
- 对应生产实验 overlay：`baeef4a4f445fc1d5b705a93bcf6e3f841202e34bf4c90f5c91ed40b9ce0035b`。

每个 case 冻结输入 SHA；harness 在 ROS 初始化前验证 SHA，并在控制器 configure/compute 前验证 costmap 实际顶点数量、padding 后坐标。每条 CSV 的 `.footprint.yaml` 记录源多边形、SHA、padding、实际 float 转换后的顶点、初始碰撞状态、地图及模型假设。oracle 消费实际顶点，采用独立 polygon/cell SAT；生产控制器继续使用原有栅格化保护，不复用 oracle 的算法。未知单元及出地图均拒绝，接触 occupied/unknown 栅格边界算碰撞；SAT 覆盖足迹内部，不限于轮廓。

## 最终实验矩阵

同一 seed 42/43/44、同一输入路径和初态分别运行三组。`candidate` 与 `feature_disabled` 使用同一个新库；`baseline` 是 09/08 保存的 **as-built 安装快照**，本轮没有从 base 源码重新编译。原核心 12 类 × 3 seeds = 36，覆盖五种大航向、限速/reset、绝对限速/动态参数、final yaw、final XY 扰动，以及三段 08/29 几何初态。额外阴影回放、HOLD、不可达门洞、初始重叠不混入到达率。

| Profile / 类别 | 试验数 | candidate 到达 | 同库 disabled 到达 | as-built 到达 |
|---|---:|---:|---:|---:|
| 旧尺寸：原核心 | 36 | 36 | 16 | 21 |
| 旧尺寸：原核心 + 明确宽门 | 45 | 45 | 24 | 29 |
| 新模型：原核心 | 36 | 36 | 16 | 22 |
| 新模型：原核心 + 明确宽门 | 42 | 42 | 22 | 28 |
| 新模型：现场冻结 costmap 的局部端点 | 9 | 9 | 6 | 6 |
| 新模型：0.44 m 门洞余量诊断 | 3 | 3 | 0 | 0 |

旧尺寸完整为 24 类 × 3 seeds × 3 arms = **216**；新模型加入三段现场上下文，27 类 × 3 seeds × 3 arms = **243**。两个最终 profile 合计 **459 次**，没有用旧尺寸结果替代新模型结果。不同 profile 的分类及宽门集合不同，因此不直接比较 45 与 42 的到达率。

所有 candidate 非初始重叠轨迹均无 oracle 碰撞，vx 非负、vy=0、无不可执行 yaw 小死区命令；百分比/绝对限速在动态参数和 reset 后仍满足约束。新模型核心及宽门 42 次产生 22576 条命令，现场上下文 9 次产生 6172 条，均无异常、倒车样本和 sub-deadband yaw。3 个有意初始重叠用例由 oracle 检出碰撞，candidate 当周期命令为零、没有误计成功；这表示没有新增运动，不能声称初始状态无碰撞。shadow 9 个用例只检查历史输入上的实际控制器输出，不能算控制闭环到达。

as-built 的到达次数也不是安全通过数：它不满足新增模式所要求的限速/无倒车/死区契约。旧尺寸本轮 as-built 核心 21/36，与 09/08 历史 22/36 的差异是 `warehouse_absolute_limit_seed43` 未在预算内到达；不将此边界结果解释为已证明的因果。新工具最终前后两轮旧尺寸所有 success 标志一致，candidate 核心均 36/36。

## 门洞与旋转扫掠

| 场景 | 实际几何含义 | 新模型 candidate | 同库 disabled |
|---|---|---|---|
| 0.36 m 门洞 | 小于模型最小支撑宽 0.396306 m，任何朝向都放不下 | 3/3 安全停住，0 到达 | 3/3 发生 oracle 碰撞 |
| 自适应窄门 0.28 m | 同样几何不可达 | 3/3 安全停住 | 3/3 oracle 碰撞 |
| 0.44 m 门洞 | 几何可通过，低余量诊断 | 3/3 到达、无碰撞 | 3/3 oracle 碰撞 |
| 0.60 m 固定门及自适应宽门 | 明确可通过 | 6/6 到达 | 6/6 到达 |
| 初态无碰撞、90° 扫掠遇单格障碍 | 根据模型 polygon 独立构造的旋转阻断 | 3/3 零平移停止、无碰撞 | 3/3 oracle 碰撞 |

旧尺寸的 0.36 m 门洞比有效宽 0.32 m 大，几何可以通过，但 candidate 3/3 都在门前停止。这保留了生产栅格化保护过于保守的具体例子，不能把它改标成几何不可达来提高到达率。新模型 0.44 m 虽被预先列为低余量诊断，实际全部通过；诊断分类并不预言一定失败。门洞不通时出现的 repeated exception 是生产 guard 的控制拒绝，测试没有运行 BT/recovery 去处理这些拒绝，不能声称已完成不可达任务的上层恢复。

SAT 每 0.01 s 检查理想运动，属于离散轨迹碰撞检查，仍有子步之间的运动离散误差。它不是连续接触动力学，也不是对实机安全间隙的保证。

## 现场冻结上下文的解释范围

使用 root 重建的 `/local_costmap/costmap` 全图加增量，frame 均为 odom，与录制的 controller pose / transformed local path 一致。10s/14s/20s 三段最近 costmap receipt age 分别约 0.295 / 0.096 / 0.095 s，应用 9 / 12 / 11 条增量，无拒绝。原 publisher 的增量 header stamp 为 0，因此重建按 bag receipt 时间、frame、bounds 排序合成，并显式保留 unstamped 证据。输入文件 SHA 由 prepare 验证，case 留存完整快照元数据。

这些 costmap 保留原 publisher 的 inflation、self-clearing 和有限滚动窗口，**没有用新包络重新声称还原现场感知**。回放起点速度设为零，跟随第一条录制局部路径直到该局部端点；三段均初始 SAT free，所有组均未发生地图越界/碰撞。候选 9/9 到达，对照 6/9；未到达的对照为 10s seed44、14s seed44、20s seed42。它们在 180 s 仿真预算内未满足局部端点检查，不能由此断言整个真实任务失败。

![门洞与现场上下文](/home/charles/project/colcon_ws/analysis_outputs/feature_navigation_envelope_20260910/zsl1_model_verified/geometry_context_comparison.png)

## 验证工具自身发现与修复

首次新模型运行 `zsl1_model/` **无效**：yaml-cpp 对已有 YAML Node 保留 block-style，直接作为字符串传给 Costmap2DROS 后，footprint 解析失败并回退半径 0.1 m 圆。日志及运行时 sidecar 的 16 顶点暴露了错误。已停止自己的运行进程、保留 `INVALID_RUN.md` 和部分 CSV；不评分该目录。

修复后显式构造 flow-style 数值序列，并在发布/计算前核验实际顶点及坐标；runner 也独立拒绝同 SHA、错误实际多边形的结果。集成测试真正启动 Costmap2DROS，验证 block/flow 两种外部文件都得到同一 4 角形；故意注入无效参数造成真实 radius fallback 时，在任何 CSV/compute 前 fatal。最终在新目录重新运行两套完整矩阵，使用相同最终 harness SHA。

验证：独立 C++ oracle self-test 通过（内部障碍、未知、越界、旋转、非有限姿态、凸 diamond/外接框假阳性排除、凹多边形拒绝、min/max 精确栅格接触）；10 个 Python/真实 ROS 加载/评分测试通过；两个最终 profile acceptance 全通过。Codacy 工具检索无可调用工具，本轮未执行 Codacy。

## 复现与交付证据

工作区产物根目录：`analysis_outputs/feature_navigation_envelope_20260910/`。

- `legacy_verified/`：最终旧尺寸 216 次；`zsl1_model_verified/`：最终新模型 243 次。
- 各目录 `cases/manifest.json`、case YAML、参数、冻结 `.costmap`、三组 CSV/日志/sidecar/`summary.json`、`comparison.json`、`acceptance_checks.json`。
- `binary_provenance.json`：最终 harness 与三个实验臂库路径/SHA；`test_flow_fix.log`、`oracle_self_test.log`、`tamper_test.json`。
- `legacy_final/`：输入序列修复前、旧 flow-style 文件有效回归；`zsl1_model/`：已明确作废的错误包络运行。

最终二进制：

| 文件 | SHA256 |
|---|---|
| harness `controller_feedback` | `fb3e6d95ed36cb8240fafb786292b2f9672cf3b46668355a951c1f269d9b149d` |
| 新 `libmppi_controller.so` | `5849539c7d6f3d91f5e944320f00a77811d21f0d52a93566a738e1cd18bfbe5a` |
| 新 `libmppi_critics.so` | `33b209e0e3c598c0d6d70aad04d9b52e8858fd353c7f439b13248a11981eea3d` |
| as-built controller | `b532fdc0d70a06d224c173fd9f6e7d8c15e896fb99c233bedaec9404d91327c9` |
| as-built critics | `76dcd1e956bd1e43998fa48fb51bf62d980b60048f4d207726a131e9602a2795` |

`legacy_verified/` 和 `zsl1_model_verified/` 是冻结证据目录，只直接读取已有 CSV、`comparison.json` 和 `acceptance_checks.json`。不要对它们运行 `prepare_cases.py` 或 `run_cases.py`；后者会替换 CSV/日志，分析和绘图脚本也会重写派生产物。重新运行必须使用全新目录。

以下从 canonical workspace 创建唯一的新输出目录，生成新 case 并读取原始地图/快照，再运行模型包络矩阵；不会覆盖本报告对应的冻结结果：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
TEST_SOURCE=src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/controller_feedback
mkdir -p analysis_outputs
TEST_OUTPUT="$(mktemp -d "$PWD/analysis_outputs/controller_feedback_repro.XXXXXX")"
touch "$TEST_OUTPUT/COLCON_IGNORE"
cmake -S "$TEST_SOURCE" -B "$TEST_OUTPUT/harness_build"
cmake --build "$TEST_OUTPUT/harness_build" -j2
"$TEST_OUTPUT/harness_build/controller_feedback" --self-test
python3 "$TEST_SOURCE/prepare_cases.py" --workspace . --output "$TEST_OUTPUT/cases" \
  --footprint-file "$TEST_SOURCE/../../params/zsl1_model_envelope.yaml" --geometry-cases \
  --recorded-costmap-dir analysis_outputs/zsl1_goal_20260910/recorded_costmaps
# Reuse archived libraries read-only; all new CSV/log files go into TEST_OUTPUT.
ln -s "$PWD/analysis_outputs/feature_navigation_20260908/baseline_install" "$TEST_OUTPUT/baseline_install"
python3 "$TEST_SOURCE/run_cases.py" --root "$TEST_OUTPUT" --variant candidate
python3 "$TEST_SOURCE/run_cases.py" --root "$TEST_OUTPUT" --variant feature_disabled
python3 "$TEST_SOURCE/run_cases.py" --root "$TEST_OUTPUT" --variant baseline
python3 "$TEST_SOURCE/analyze_results.py" --root "$TEST_OUTPUT"
python3 "$TEST_SOURCE/plot_geometry_results.py" --root "$TEST_OUTPUT"
```

旧尺寸矩阵须另外创建一个新目录，将 `--footprint-file` 改为 `$TEST_SOURCE/legacy_footprint.yaml` 并去掉 `--recorded-costmap-dir`。基线链接只用于只读加载已归档库，CSV、日志、分析和图均写入新的 `TEST_OUTPUT`。集成加载回归使用临时输出，不改冻结 CSV：

```bash
CONTROLLER_FEEDBACK_ROOT=analysis_outputs/feature_navigation_envelope_20260910 python3 -m pytest -q "$TEST_SOURCE/test_footprint_loading.py" "$TEST_SOURCE/test_geometry_cases.py"
```

本轮可 review 的结果是模型输入一致性、实际控制器的命令反馈闭环及明确的不可达/保护保守性证据。后续仍需实测动态包络、ROS action/BT 与定位失效联动、动态感知、真实底盘执行误差，才能缩小到实机验收的剩余差距。
