# Feature与Deploy统一（2026-09-19）

## 最终组织方式

以`feature/forward-aligned-navigation`为共同代码基线，回收原deploy独有的实机入口、配置、专用BT、验证与说明。原deploy前向算法已经来自feature，不重复移植，也不把旧公共接口的适配反向覆盖feature。

将deploy rebase到集成后的feature。因为所有需要保留的部署内容都已纳入feature，deploy无需再保留独有补丁，两分支可以指向同一提交、同一文件树。这满足“feature包含deploy能力”，以后新增实机配置先进入feature，deploy选定已经验证的feature提交，避免长期维护重复代码。

原历史保留：

- `archive/deploy-rockdog-navigation-before-unify-20260919` → `d732767`。
- `archive/feature-forward-aligned-navigation-before-unify-20260919` → `09f6cf8`。
- 更早31笔历史仍由`archive/forward-aligned-navigation-before-squash-20260919`保留。

`feature/navigation-0829-functional-squash`仍作为此前5笔整理结果的快照；当前主要开发分支是`feature/forward-aligned-navigation`。

## Epoch协议是什么

定位消息告诉规划器“机器人在哪里”，epoch协议另外回答“这个位置属于哪次有效定位，以及哪次任务和规划可以据此执行”。

链路保留定位进程会话、地图加载身份、定位epoch/commit，以及导航任务、规划和执行身份。规划请求捕获身份和起点，规划结果保留来源身份；控制器真正安装匹配路径后发布执行状态，命令携带对应身份，平滑器与gate校验新鲜度及授权。

例如规划还在运行时发生重定位，旧请求随后返回路径：普通topic订阅本身不保证这条路径仍匹配最新定位。epoch链路会拒绝旧身份的结果或命令，等待新规划安装；不是把最新定位消息订阅到就直接执行旧路径。

它包含request/result、安装确认及持续状态/命令校验，不是单次request-response，也不是新的MPPI cost或全局规划算法。超时、身份不匹配、定位失效或撤权会阻止执行。

## 保留与取舍

纳入feature的deploy独有内容：

- `rockdog_forward_test.launch.py`：以已有部署参数为base，覆盖前向测试契约。
- 专用前向overlay及无自动运动恢复的BT。
- 配置契约测试、隔离smoother实测工具、部署报告和历史结果。
- Python YAML运行依赖。

继续采用feature已有内容：

- 完整epoch消息、Controller虚接口、BT、smoother与gate实现。
- 具有默认Livox CustomMsg且可选PointCloud2的感知代码，原始消息时间戳保留。
- 模拟LCM与Matrix扩展，不用deploy旧版删除功能的实现覆盖它们。
- 普通模式gate已有命令超时保护，并增强steady-clock watchdog、定位失效立即零速、非有限命令拒绝。保留这些增强，不追求与旧deploy逐行相同。
- 使用可移植Python解释器入口，不恢复旧deploy硬编码python3.12。

## 实机前向测试仍关闭epoch

专用profile强制以下节点`enable_epoch_contract=false`：

- bt_navigator
- controller_server
- velocity_smoother
- nav_safety_gate

另外强制`enable_epoch_backup=false`及`enable_epoch_backup_recovery=false`。即使base配置误含epoch=true，也不会启动混合协议；新增回归测试覆盖这一情形。

实机测试仍使用普通FollowPath、Twist、原SDK入口。controller内部的ALIGN/TRACK/FINAL、速度约束与停稳逻辑正常运行，不需要Lightning新增epoch输出。自动运动恢复关闭，近目标有界微退仍未实现。

**运行模式关闭不等于免构建依赖。** feature C++实现依赖epoch消息类型及新版navigo_core虚接口，需要在同一构建前缀重新编译相关节点和插件；不能混用此前独立移植版的旧core/ControllerServer二进制。gate的Python epoch导入只在epoch开启时发生，但C++链接依赖仍需安装。

入口和用户参数保持：

```bash
ros2 launch robot_navigo rockdog_forward_test.launch.py \
  robot_ip:=192.168.234.1 local_ip:=192.168.234.100 \
  map:=/home/charles/project/colcon_ws/data/new_map/map.yaml \
  use_sim_time:=false auto_standup:=false
```

先用独立build/install前缀构建完整导航目录并source新安装环境，具体命令见历史部署文档“上机编译与启动”；统一后该目录还会构建navigo_epoch_msgs。不要仅更新MPPI一个动态库。

## 验证范围

本轮9包构建成功；45项Python测试、57项C++测试及导航契约检查通过；36条反馈对照中前向18/18到达、零倒退；实际平滑器80个样本通过前向/横移限制及断流停车检查。

本轮验证记录见同目录`navigation_branch_unification_validation_20260919.json`与本地`analysis_outputs/unify_navigation_branches_20260919`。普通模式gate和epoch模式gate均回归；实机profile额外验证自定义epoch base不会绕过关闭开关。新的统一安装环境下重新验证控制器/平滑器，避免使用旧deploy ABI。

不将这些本地结果当成实机验收：尚无机械狗、RK3588、真实步态/惯性或SDK响应测试。模型footprint和旧自过滤范围的差异、严格前向在终点附近效率较低等已知限制仍保留。
