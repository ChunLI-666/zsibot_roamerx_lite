# ZSL-1 模型包络与配置审查（2026-09-10）

**已从官方完整碰撞网格生成可复现的模型包络，修正旧尺寸实验的几何前提。当前结果只覆盖 Matrix `home` 明确关节姿态，不能证明站立变化、行走、转向或载荷在实机上的包络。默认生产尺寸没有改变。**

## 冻结输入与证据

- 官方来源：[genisom_model / ZSL-1](https://github.com/zsibot/genisom_model/tree/e6aa98e22d38ae3fdf4d448f79820295e78e83a5/zsl-1)，commit `e6aa98e22d38ae3fdf4d448f79820295e78e83a5`。
- 本地 URDF：`src/zsibot/zsibot_model/zsl-1/urdf/ZSL-1.urdf`，SHA256 `992c91921be1099794554295f731064d108d096a5bb6068a9cdc35eb903e0c76`。URDF 与 17 个 STL 均从固定提交下载到内存逐字节核对，18/18 一致。详情见 [zsl1_official_source_manifest.json](../params/zsl1_official_source_manifest.json)。
- 所有 17 个 collision mesh 均参与计算，没有用 visual 简化、凸包网格或只用机身。共 245,690 个三角形、737,070 个三角形顶点，含重复顶点。
- 姿态来源：`src/zsibot/matrix/src/robot_mujoco/jszr_robots/xgb/xgb.xml` 的 `home` keyframe。Matrix checkout HEAD 为 `f5406c5c97642d66c80d686bdde2929966fa643e`，具体 XML/mesh 的内容哈希也单独冻结，不以仓库 HEAD 替代实际文件哈希。
- Matrix 17 个 STL 与上述官方 STL 全部一致。导入器核对 12 个旋转关节的父子关系、原点、轴、顺序及 4 个固定足端原点；`FAR→FR, FBL→FL, RAR→RR, RBL→RL`。每腿 `ABAD=0, HIP=0.9, KNEE=-1.8` rad，全部在官方 URDF 范围内。使用 `qpos`；`ctrl` 字段不是关节姿态，不能拿它的相反符号替代 `qpos`。
- [zsl1_model_postures.yaml](../params/zsl1_model_postures.yaml) 保存完整显式关节名/值、原始 qpos、模型坐标变换、源文件哈希和未验证假设。缺失任意可动关节会拒绝；没有隐式零姿态。URDF 膝关节范围 `[-2.723,-0.602]`，全零姿态本身就不合法。

## 包络与原点

| 项目 | 数值 |
|---|---:|
| 未加 padding 的 x 范围 | `[-0.359786995, 0.282186717] m` |
| 未加 padding 的 y 范围 | `[-0.188144226, 0.188161811] m` |
| 外接矩形长 × 宽 | `0.641973711 × 0.376306037 m` |
| 原始平面凸包 | 89 顶点 |
| 实验消费的 footprint | 同一边界的 4 顶点保守矩形，逆时针 |
| costmap padding | 每坐标按符号增加 `0.01 m`，沿用既有实验值 |
| padding 后 x 范围 | `[-0.369786995, 0.292186717] m` |
| padding 后 y 范围 | `[-0.198144226, 0.198161811] m` |
| padding 后外接半径 | `0.419536083 m` |

矩形不关于原点对称。工具保留官方基准，不重新中心化，也不把后端 `-0.3598 m` 改成对称 `±0.3210 m`。原导航 `±0.30 × ±0.15 m` 不能覆盖该 home 模型。原始 89 点凸包保留为 `convex_hull` 字段，可用于几何对照；默认矩形避免大量 footprint 顶点的计算开销，并保守包含该姿态全部网格。

[权威模型包络 zsl1_model_envelope.yaml](../params/zsl1_model_envelope.yaml) SHA256：`94965f8269a74137490fa07416ac231824831b5601ef035d8c686677d99c5855`。模型、生成器、姿态和官方源 manifest 的哈希都记录在该文件中。对每一个网格顶点进行凸多边形半平面检查，737,070/737,070 均在包络内，数值检查容差 `1e-9 m`，最大越界为 0。凸包包含三角形全部顶点即可包含完整三角形；这里的静态覆盖检查并非只抽样少量点。

生成器也支持 `--shape convex_hull`，但 Nav2 的按坐标符号 padding 不是标准多边形等距外扩，某些凸包经过该操作会变凹。工具会拒绝 padding 后非凸或未覆盖原始 footprint 的结果，不能假定任意凸包均可直接用于当前 costmap。

## 坐标、姿态与载荷边界

模型实验使用 `base_link←BASE_LINK` identity。这由 Matrix `base_link` 与官方 URDF 根 mesh、对应关节原点/轴一致所支持，属于模型坐标约定。它**不证明**实机 SDK odometry 原点、Lightning tracking frame、导航 `base_link` 与机械 CAD 根之间的实际外参已经标定。转换在姿态输入中明确保存，并标记 `hardware_verified: false`。

没有把 Matrix home 当成稳定支撑平衡状态：其 qpos 根高度为 0.27 m，而该姿态网格最低点相对根为 `-0.292437822 m`，直接放在该高度时最低点约低于地面 0.02244 m。它是有出处的初始模型姿态，不能替代仿真接触收敛后或实机站立的关节记录。

URDF 没有额外传感器或载荷的碰撞 link。Matrix 里的 LiDAR/camera `site`、相机观测点不是碰撞体积，不能据此认定载荷没有外伸。本轮不填写虚构的尺寸或步态角度范围。工具遇到未实现的额外 payload 输入会拒绝，要求先明确建模。

[录包证据](../params/zsl1_recording_evidence.json) 保存 0829 六包 metadata 的完整 topic/type/count 和哈希。没有 `JointState`、motor、LowState/HighState 等关节消息。后三包既有 lossless debug 提取共 6,179 条 YzRobotCtrlDebug，其 JSON payload 键仅为 `action/battery/owner/type/vx/vy/wz`，没有关节位置。这里复核了 metadata 和既有提取文件，未冒充本轮重新反序列化全部 MCAP。底盘速度和 SDK odometry 不能反推出腿部轨迹。

因此当前固定包络只声明 `static_matrix_home` 样本，不声明覆盖 physical stance、forward gait、turning gait、lateral gait 或 recovery gait。工具可接受后续明确关节样本和 root roll/pitch 的集合，生成固定并集；本轮测试中的随机合法姿态仅验证 FK/投影实现，没有加入生产实验包络，也没有当成实际步态。

## 配置接入

[模型 overlay](../params/zsl1_model_footprint_overlay.yaml) SHA256：`baeef4a4f445fc1d5b705a93bcf6e3f841202e34bf4c90f5c91ed40b9ce0035b`。全局与局部 costmap 从同一结果生成，footprint、padding 一致，输出没有 YAML anchors/aliases。

两个 costmap 的实验 inflation radius 同步设为 0.50 m：按 `ceil((padded_circumscribed_radius + resolution)/resolution)*resolution`、resolution 0.05 m 推导。原值 0.40 m 小于新 padded 半径；当前 MPPI `CostCritic` 用 center-cell cost 判断是否需要完整足迹检查，inflation 的有效范围必须覆盖其外接半径。0.50 m 是该模型实验的计算域选择，不是增加到实机步态尺寸上的经验量。`cost_scaling_factor=3.0` 保留。

**overlay 不是完整导航配置，不能直接把 launch 的 `params_file` 指向它。** 使用 [zsl1_merge_params.py](../scripts/zsl1_merge_params.py) 按以下顺序递归合并：完整 `navigo_params.yaml` → `forward_alignment_experiment.yaml` → `zsl1_model_footprint_overlay.yaml`。工具检查全局/局部参数与权威包络一致、基础 costmap plugins 没有丢失，输出完整 YAML 与输入/输出哈希 manifest，不启动或部署任何节点。

当前完整生成样例：[zsl1_experiment.params.yaml](/home/charles/project/colcon_ws/analysis_outputs/zsl1_model_envelope_20260910/zsl1_experiment.params.yaml)。它仅供模型实验与 review，不是已批准的实机配置。

`livox_scan_projector` 的自身裁剪独立配置为 x `[-0.35,0.35]`、y `[-0.20,0.20]`、高度 `[0.05,0.45]` m。它是删除点云的传感器处理区域；碰撞包络是禁止机器人进入障碍的区域，两者作用不同。此 overlay 没有扩大或改变自身裁剪，避免把近处真实障碍误删。后续应依据实际传感器外参、被观测到的机体点和载荷重新核对裁剪。

## 复现与验证

从 workspace 根目录运行；下列命令不会启动机器人：

```bash
python3 src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/zsl1_envelope.py verify-official \
  --urdf src/zsibot/zsibot_model/zsl-1/urdf/ZSL-1.urdf \
  --commit e6aa98e22d38ae3fdf4d448f79820295e78e83a5 \
  --output /tmp/zsl1_official_source_manifest.json
python3 src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/zsl1_envelope.py import-home \
  --urdf src/zsibot/zsibot_model/zsl-1/urdf/ZSL-1.urdf \
  --matrix-xml src/zsibot/matrix/src/robot_mujoco/jszr_robots/xgb/xgb.xml \
  --output /tmp/zsl1_model_postures.yaml
python3 src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/zsl1_envelope.py generate \
  --urdf src/zsibot/zsibot_model/zsl-1/urdf/ZSL-1.urdf \
  --postures /tmp/zsl1_model_postures.yaml \
  --official-manifest /tmp/zsl1_official_source_manifest.json \
  --output /tmp/zsl1_model_envelope.yaml --overlay /tmp/zsl1_model_footprint_overlay.yaml
python3 src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/zsl1_test_envelope.py
```

完整配置合并示例：

```bash
python3 src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/zsl1_merge_params.py \
  --base src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/params/navigo_params.yaml \
  --controller-overlay src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/params/forward_alignment_experiment.yaml \
  --model-overlay src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/params/zsl1_model_footprint_overlay.yaml \
  --envelope src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/params/zsl1_model_envelope.yaml \
  --output /tmp/zsl1_complete_experiment.params.yaml
```

最终 12/12 项 unittest 通过（5.4 s，无 skip）。按上述命令再次联网核对官方 18 文件并重新生成，源 manifest、姿态、包络和 overlay 四份输出均逐字节一致，见 [复现检查](/home/charles/project/colcon_ws/analysis_outputs/zsl1_model_envelope_20260910/reproducibility_check.json)。实际用隔离 ROS domain 184 的 rclpy 节点加载完整参数，全局/局部 `footprint` 均解析为正确 JSON 字符串，4 顶点、padding 与 inflation 一致，见 [ROS 参数解析检查](/home/charles/project/colcon_ws/analysis_outputs/zsl1_model_envelope_20260910/ros_parameter_parse_check.json)。这验证了 rcl YAML 读取；实际 Costmap2DROS 消费后的顶点仍由独立闭环 harness 核验。

纯几何与模型回归覆盖：STL binary/ASCII 和非有限输入、轴角/URDF 变换次序、独立足端解析 FK、完整关节输入与限位、官方/Matrix 对应、全部网格顶点覆盖、非中心原点、padding 凹性、未建模 payload 拒绝、冻结源不一致拒绝、完整参数合并与全局/局部不一致拒绝。现有模型存在时不跳过集成检查；模型缺失时会显式 skip，并保留纯几何测试，不能把这种运行称为已验证官方模型。

模型包络消费与新几何闭环由独立 validation subagent 检查，结果另见其报告。本文件不沿用旧尺寸下的 36/36 成功率来证明新包络通过，也不把有限模型用例的无碰撞解释为实机认证。原 controller 还保留完整网格扫掠外的一格 guard，数学上刚能放进 footprint 的门洞未必会被控制器接受，应分别报告几何可行性与保守净距拒绝。

图示：[模型包络与录包环境](/home/charles/project/colcon_ws/analysis_outputs/zsl1_goal_20260910/zsl1_footprint_and_recorded_context.png)。该图已由 root 目视核验。

代码均保留在 `feature/forward-aligned-navigation`；未改生产尺寸、未启动实机、未合并。Codacy MCP 当前不可用，未执行该检查。
