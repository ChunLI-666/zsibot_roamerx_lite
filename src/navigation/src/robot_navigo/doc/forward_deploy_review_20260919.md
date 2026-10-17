# 0829后提交整理与前向规控部署审阅

> 历史记录：本文对应独立移植版 deploy `d732767`。随后已按用户要求统一到 feature 代码；当前构建依赖、分支关系和验证见 [分支统一说明](navigation_branch_unification_20260919.md)。本文所述“未移入epoch代码／旧公共接口”不再代表统一后的源码；前向实机运行模式仍显式关闭epoch。

## 范围与结果

用户要求review roamerx_lite在0829后新增提交，按功能squash，并把前向规控代码移到deploy/rockdog-navigation，仅用于下一轮前向运动实机测试。

原feature：`6942974 → f259fc4`，31笔。保留`feature/forward-aligned-navigation`，另建`feature/navigation-0829-functional-squash`整理为5笔；最终Git tree与f259fc4完全相同。全部操作在canonical checkout，没有创建或使用额外worktree。

部署基线：`71aa8e9`，不是6942974；它已有实机命令新鲜度保护，并删掉了一些仿真包内容。采用选择性源码移植而非合并整个feature，保留部署既有启动、感知、Lightning桥接、安全gate与SDK接口。

## 功能squash分组

| 新提交 | 原提交范围（左端不含） | 内容 | 本次部署 |
|---|---|---|---|
| b51372d | 6942974..0d5e492 | 前向对齐、MPPI可执行控制、限速修复、初始反馈实验 | 移植前向生产代码；验证工具按需 |
| 6938086 | 0d5e492..2b8effd | 模型footprint、定位失效停车、地图/足迹验证 | 取模型几何和碰撞必要部分；不替换部署gate |
| 61a2544 | 2b8effd..e826d6f | 定位epoch身份、规划/安装/执行协议 | 不移植 |
| 9f4abbc | e826d6f..c34631c | epoch有界恢复、制动、完整时间仿真 | 仅取控制器内部停稳交接及共用扫掠；不移植epoch/BackUp |
| 09f6cf8 | c34631c..f259fc4 | 成本审计、连续反馈实验 | 取最小连续控制验证工具；不带全套成本研究数据 |

功能整理保持每组原修改顺序；原始提交仍由保留feature可达。不是将所有feature行为默认打开。

## 部署review发现与处理

1. **epoch ABI耦合**：最终feature MPPI的reset override、getExecutionMotionPhase override依赖新的navigo_core虚接口。移植后恢复旧reset声明，去掉getter，保留内部转动/平移停稳判定。部署navigo_core、ControllerServer协议未改。
2. **缺失几何依赖**：同时移入costmap的footprint_sweep.hpp。它检查多边形内部、边界及旋转/轨迹扫掠；未知与地图外拒绝。移植不是末端Twist截零。
3. **默认目标检查器不兼容**：专用profile明确选择StoppedGoalChecker，stateful=false，XY/yaw=.25，停止阈值=.01；同时强制插件选择器，防止自定义base选到别的checker。
4. **自动恢复可能倒退**：两种导航BT均移除Spin/BackUp/DriveOnHeading，只保留清图与等待；behavior server只注册wait。smoother设置min_vx=0、vy上下限=0，作为标准导航输出链的附加限制。保留清图恢复这一既有行为，不将它当感知失效安全认证。
5. **自定义base与预测上限不一致**：profile固定MPPI vx_max=.15、wz_max=.1、vx_min=0、vy_max=0、Omni、一次迭代，并显式固定10Hz。smoother上限一致。不改变critic权重和全局规划算法。
6. **角速度启动耦合**：默认角加速度.2/10Hz=.02，等于最小可执行转速；改20Hz可能每次小于死区而不转。本轮固定10Hz且加入配置测试，不宣称任意频率都可用。
7. **模型与真实包络**：同时应用ZSL-1模型footprint到全局/局部costmap，含padding约0.662×0.396m。模型文件仍保留hardware_deployment_ready=false；没有摆腿/负载/实际TF标定。scan projector自过滤仍旧x±.35/y±.20，可能有后侧自点残留或前侧额外过滤，现场应看costmap与点云，不宣称感知包络已统一。
8. **执行链边界**：没有改SDK或旧安全gate。仅保证专用入口的标准Controller→smoother链与默认BT前向约束；外部直接发布cmd_vel/cmd_vel_safe可绕过smoother，不宣称所有命令来源均受约束。
9. **后验几何与动力学不同**：扫掠未包含实机惯性、SDK延迟及完整制动距离。转身受阻会拒绝，不会自行寻找更宽位置。先在宽敞环境验证行为，不以本次结果批准窄门通行。

## 具体部署改动

- `4b1ff88`：前向ALIGN/TRACK/FINAL、路径几何前视、迟滞、停稳交接；采样前及平均/滤波后可执行投影，最终footprint扫掠；重置后保持绝对/百分比限速；关闭fast-math以保留非有限数保护；相关C++测试。
- `e5b5da6`：独立`rockdog_forward_test.launch.py`，合并原部署参数、前向测试overlay、模型footprint；显式无自动运动恢复的BT与profile契约测试。
- 后续验证/文档提交：仅离线反馈、独立平滑器检查、审阅清单。脚本不由实机launch自动启动。

原`zsibot_nav_bringup.launch.py`和`navigo_params.yaml`未改；使用原命令仍是原策略。测试必须用新入口，避免误以为拉取分支就自动启用功能。新入口输出实际合并参数临时路径，退出清理；运行期间应保存ros2 param dump。

## 验证证据

本地主机ROS Jazzy，未连接机械狗，未做RK3588/ARM交叉编译。

- 独立build/install前缀构建部署版navigo_costmap_2d、navigo_core、navigo_mppi_controller、navigo_path_controller、navigo_velocity_optimizer：5包成功。其余未变依赖来自现有工作区，非完整根文件系统封装。
- MPPI两个C++测试套件：9个测试通过（状态迟滞、朝向、可执行约束、限速、扫掠、deadband等）。
- 6项专用profile测试通过：参数流、原全局规划/critic不变、checker选择、上下限、模型足迹及无运动恢复BT。
- 使用部署版旧navigo_core ABI及新MPPI真实插件，6场景×3seed×2策略=36条连续反馈：原策略13/18，新策略18/18，后者raw与执行累计倒退均零。与前一轮feature结果一致。这是理想SE(2)闭环，不包含SDK、BT、动态感知或真实惯性。
- 实际启动独立部署版smoother，隔离ROS域，注入前进/负向/横向/旋转输入及断流：81个输出样本无负vx、无横移，最终停稳。不启动机器人桥接或SDK。
- Python语法、Git whitespace检查通过；无可用Codacy工具，未执行该检查。

本地原始证据目录：`/home/charles/project/colcon_ws/analysis_outputs/deploy_forward_review_20260919/`，含初始分支状态、build日志、9项gtest、36条反馈及smoother_runtime/result.json。结果摘要随本文件提交。

已知取舍保持：严格前向在越过终点0.35m后需约63秒转身修正，原策略约4–5秒；近目标有界微退尚未实现。当前验收是满足0.25m/0.25rad及停稳，不是精确泊车。

## 上机编译与启动

在实际机器人对应工作区更新此部署分支后，重新构建导航包，不复用epoch feature的旧插件与navigo_core头文件。可用独立安装前缀隔离旧产物（这是构建目录，不是Git worktree）：

```bash
# 在机器人上，先 source 其现有ROS与传感器/SDK依赖环境，再在colcon_ws执行。
colcon build --base-paths src/zsibot/zsibot_roamerx_lite/src/navigation/src \
  --build-base build_rockdog_forward --install-base install_rockdog_forward \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install_rockdog_forward/setup.bash

# Lightning定位沿用部署既有版本和启动方式；本轮不更新Lightning。
ros2 launch robot_navigo rockdog_forward_test.launch.py \
  robot_ip:=192.168.234.1 local_ip:=192.168.234.100 \
  map:=/home/charles/project/colcon_ws/data/new_map/map.yaml \
  use_sim_time:=false auto_standup:=false
```

如需指定原部署配置，传`base_params_file:=...`，不要传`params_file`绕过合并。测试overlay会覆盖运动/checker/BT/足迹等契约字段，其余参数沿用base。

先确认安装包位置、实际参数以及cmd_vel_safe订阅者，避免source到别的工作区。保存：

```bash
ros2 pkg prefix robot_navigo
ros2 param dump /controller_server
ros2 param dump /velocity_smoother
ros2 param dump /bt_navigator
ros2 param dump /local_costmap/local_costmap
ros2 topic info /cmd_vel_safe -v
ros2 bag record /odom/current_pose /cmd_vel_nav /cmd_vel /cmd_vel_safe \
  /controller_server/debug /controller_server/FollowPath/motion_mode \
  /plan /transformed_global_plan /local_costmap/costmap \
  /local_costmap/costmap_updates /tf /tf_static
```

本轮实机用例：正向直行；90°/180°先对齐再走；横向偏离修正；终点yaw并停稳；取消任务后停车；宽敞处观察越过终点的已知效率限制。核对raw、平滑后及SDK入口命令均无非预期负vx，模式交接与odom停止一致。定位不可信时沿用原部署安全链停车。

不在本轮测试近目标有界倒退、自动BackUp、epoch恢复、定位算法替换或新地图建图。需要回退行为时停止新launch并用原zsibot_nav_bringup.launch.py及原参数启动，无需改写Git历史。
