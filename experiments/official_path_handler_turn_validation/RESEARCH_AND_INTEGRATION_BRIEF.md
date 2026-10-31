# 机械狗原地转向方案调研与验证说明

**结论：Lattice＋官方路径处理接口＋RPP 候选已在本机固定场景中解决原地转向和近目标绕大圈问题，值得继续推进。它可以进入 ARM 隔离编译和软件验证，尚未接成正式导航服务。**

目标是 A→B 点到点导航，包含起步对向、途中必要转向、终点朝向和窄门通过。本轮不涉及多途经点产品设计，也不证明实机过门成功率。

## 调研了哪些路线

| 路线 | 能力和取舍 | 本轮核对范围 |
|---|---|---|
| NavFn＋RPP | 便于路径跟踪和对向，但全局搜索不逐朝向检查长方形机身；不作为最终候选 | 文档及机制比较，未独立重跑该组合 |
| Smac Hybrid＋MPPI 或 RPP | 保留朝向和轮廓规划；当前 DUBIN 连续圆弧模型不表达原地转向动作，单换控制器不会改变已生成的绕圈路径 | 实跑 Hybrid 全局规划；未在本轮联测这两种完整组合 |
| Rotation Shim | 可在接到新路径时先对向，RPP 自身也有类似能力；它不负责在全局路径中选择合适的途中旋转地点 | 官方文档与源码机制比较 |
| Lattice＋FeasiblePathHandler＋RPP | 规划器可使用直行和原地转向动作；路径处理器约束旋转段顺序；RPP 计算速度 | 本轮完成组件编译、真实规划联测和隔离 ROS Action 测试 |

上述能力边界依据 [Nav2 插件指南](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/)、[Lattice 文档](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/)和 [Rotation Shim 说明](https://docs.nav2.org/rolling/tutorials/general_tutorials/using_shim_controller/)。据此选择 Lattice 是针对本次运动需求的工程判断，并非“机械狗通用最优配置”的结论。

## 当前候选实现了什么

链路是 **Lattice 规划位置和机身朝向 → PathHandler 保留当前旋转段 → RPP 输出前向速度和角速度 → 虚拟机体反馈**。官方 [FeasiblePathHandler](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/controller_server/controller_server_plugins/feasible_path_handler/) 已提供旋转约束机制；本轮还做了四项适配和修正：

1. **接口兼容**：适配 Jazzy 节点与发布接口，桥接现有 FollowPath 消息。规划器使用官方 Jazzy 1.3.13；控制器使用[固定的官方重构版本](https://github.com/ros-navigation/navigation2/pull/5446)，不是整体升级最新 Nav2。
2. **保留完整旋转段**：修改一个辅助函数，避免只保留旋转前半段便放行后续路径。这是实际逻辑修正，见[补丁](/home/charles/project/colcon_ws/analysis_outputs/upstream_nav2_interface_isolated_20261031/evidence/complete_rotation_constraint.patch)。
3. **保留原始目标朝向**：在离散规划的终点 XY 处追加用户请求的朝向，避免把“到达离散朝向”误当成满足原始请求。
4. **统一动作和容差**：使用冻结的 1 cm 停转动作集，调整转向距离估价、限制解析捷径，并对齐控制器和路径处理器的 XY 容差。动作集来自主实验的冻结副本，不能称作未经修改的官方生成配置；原地转向来自动作本身，不是把 Hybrid 半径设成零。

## 做了哪些实验及结论依据

真实运行了规划器、路径处理器、RPP、costmap 轮廓检测及隔离 Action 请求；TF 和运动反馈由理想机体生成，没有运行 Lightning、SDK 或真实腿部运动。最终规划联测统一使用 0.60×0.30 m 矩形、每侧 3.5 cm padding，合成门宽 46 cm；这些是测试模型，并非新测得的机械狗尺寸。

五场 A→B 分别是：**同位置掉头 180°、20 cm 近目标且朝向差 110°、窄门通过后对向、途中直角窄通道、旧地图近目标大朝向差**。两档原地转速参数下共 **10/10 通过**，满足原始请求的位置误差 ≤1 cm、朝向误差 ≤2.86°，运动段密集轮廓检查通过。[联测证据](/home/charles/project/colcon_ws/analysis_outputs/upstream_nav2_interface_isolated_20261031/evidence/ab_chain_tight_xy_result.json)

| 对照 | 原结果 | 修正后结果及依据 |
|---|---|---|
| 旧图中仅约 4 cm 的目标位移 | Hybrid 路径约 4.96 m | 停转 Lattice 约 4.47 cm；新控制器可执行。[规划对照](/home/charles/project/colcon_ws/analysis_outputs/upstream_nav2_interface_isolated_20261031/evidence/global_planner_comparison.json) |
| 同一停转动作集的 20 cm 近目标 | 半米转向估价仍规划约 2.46 m | 同样精确搜索容差下，小半径估价得到约 19.9 cm。[原估价](/home/charles/project/colcon_ws/analysis_outputs/upstream_nav2_interface_isolated_20261031/evidence/global_old_radius_exact.log)、[新估价](/home/charles/project/colcon_ws/analysis_outputs/upstream_nav2_interface_isolated_20261031/evidence/global_ab_planner_repro.log) |
| 同一条直角通道路径 | 初始宽松 XY 容差使控制器提前转，被碰撞检查拦住 | 控制器 1 cm／路径处理器 2 cm 配套容差通过。[失败记录](/home/charles/project/colcon_ws/analysis_outputs/upstream_nav2_interface_isolated_20261031/evidence/baseline_xy4cm/result.json) |

绕圈缩短的收益来自规划和动作配置；按序转向的收益来自路径处理与控制执行，不能全部归功于“换了新接口”。另已通过取消后接新目标、无效请求拒绝、Action 请求／反馈／结果及取消往返、旋转遇障拒绝和两项 CTest。[控制与 Action 证据](/home/charles/project/colcon_ws/analysis_outputs/upstream_nav2_interface_isolated_20261031/evidence/repro_result.json)

这些是固定场景的确定性回归，不是长期实机成功率。原始朝向量化失败、提前转向失败均保留；旧 Hybrid 路径的额外密集检查还有碰撞样本，因此该条不能作为已验收的安全参考路径。

## Pros and Cons

**收益**：保留机身朝向和轮廓规划；允许在途中及终点原地转向，避免靠大圆弧掉头；复用官方主体和现有外部消息，无需重写整套导航算法。

**代价**：当前 RPP 候选不输出横移，也不等同于 MPPI 的局部轨迹优化；需要维护接口兼容层、旋转段小修正及配套动作配置。1 cm XY 容差目前在理想运动模型下成立，实机定位与跟踪误差能否满足需要测量。腿部包络、执行延迟、速度死区、动态障碍和 ARM 实时性尚未在本候选中验证。

## 距离正式部署还差什么

1. **接成控制服务**：将候选放入实际 ControllerServer，固定正确的官方库加载环境；接入真实 TF、odom、局部 costmap、生命周期与进度检查，核实正常原地转向不会被判为停滞。
2. **接回现有链路**：接收现有 BT 的 FollowPath 请求，输出到 `/cmd_vel_nav`，继续经过原有速度平滑器、安全门和执行链。BT、平滑器、安全门可复用；还需整链验证新目标、取消、重规划和限速／死区后的真实速度效果。
3. **分开验收软件和实机**：先做 ARM 隔离原生构建与软件闭环，再验证执行延迟和实际机身通过性。当前结论只支持前一阶段，不代表可以直接替换正式导航进程。

## 复现与留存

[规划联测入口](/home/charles/project/colcon_ws/analysis_outputs/upstream_nav2_interface_isolated_20261031/run_planner_chain_isolated.sh) · [Action 与取消入口](/home/charles/project/colcon_ws/analysis_outputs/upstream_nav2_interface_isolated_20261031/run_isolated.sh) · [官方源码版本与校验](/home/charles/project/colcon_ws/analysis_outputs/upstream_nav2_interface_isolated_20261031/evidence/runtime_source_integrity.json)

导航仓库本地分支为 `feature/official-path-handler-turn-validation`，基线 `419f28b`；代码和证据存于该分支的 `experiments/official_path_handler_turn_validation/`。未切换主会话分支，未为本候选部署或切换板端导航。
