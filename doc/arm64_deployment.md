# ARM64 定位与导航发版、构建及启动

更新：2026-10-31。本文对应两仓配对的 ARM 正式发版线；导航分支基点为
`b9632efe280c1ab2a5bdc55c891e1b92efba7846`。构建入口的实现以
[`scripts/arm64/workspace.py`](../scripts/arm64/workspace.py) 为准。

当前本机 host build、完整旧包回放及四种闭环仿真已通过；本轮 ARM 板端构建和实机验收留待下次。
本文中的命令是后续执行流程，不表示已经部署。构建、静态检查、回放、闭环仿真、
板端运行和机械狗运动验收分别记录，不能相互替代。

## 1. 两仓配对与升级方式

| 仓库 | ARM 发版分支 | 职责 |
|---|---|---|
| LocMap（ROS 包名 `lightning`） | `release/arm64-localization` | 在线定位及其必要库，关闭 UI 和开发工具构建 |
| zsibot_roamerx_lite | `release/arm64-navigation` | 导航、扫描投影、安全门、共用消息和构建入口 |

`develop` 承载实验和回放回归。验收后的变更按可审查的独立提交，通过
`git cherry-pick -x <source-commit>` 进入对应 ARM 分支；不整体 merge `develop`。
共同消息、插件接口或公共库有变更时，两仓记录同一交付批次，连同依赖的提交一起迁移。
每次发版保存两仓完整 SHA、dirty 状态、地图/站点配置摘要、构建和测试结果。
源码保留开发、建图、回放和仿真工具；ARM profile 只编译、安装需要的运行子集。

下面假定板端已有 canonical workspace，两个 checkout 已处于上述分支。
在 workspace 根执行，按实际路径设置变量；定位仓既支持 `src/LocMap`，也支持
历史目录名 `src/lightning-lm`。两者同时存在时必须明确选一个。

```bash
ARM_WS="$PWD"
ARM_NAV="$ARM_WS/src/zsibot/zsibot_roamerx_lite"
ARM_LOC="$ARM_WS/src/LocMap"  # 若现有目录名是 lightning-lm，在此改成其实际路径
```

先确认两仓分支与 clean 状态，再分别 fast-forward 更新。以下检查不通过时先处理
现有改动，不强制覆盖、重置或自动换分支。

```bash
(
set -e
test "$(git -C "$ARM_LOC" branch --show-current)" = release/arm64-localization
test -z "$(git -C "$ARM_LOC" status --porcelain)"
git -C "$ARM_LOC" pull --ff-only origin release/arm64-localization

test "$(git -C "$ARM_NAV" branch --show-current)" = release/arm64-navigation
test -z "$(git -C "$ARM_NAV" status --porcelain)"
git -C "$ARM_NAV" pull --ff-only origin release/arm64-navigation
)
```

逐条确认前一条成功再继续；分支尚未建立时，先在 clean checkout 中取得并选择对应远端
分支。构建脚本记录当前分支和 SHA，但不替用户执行 pull、切分支或要求特定分支名。
正在运行的服务可能引用源码，更新 checkout 和切换服务应安排在获准的维护窗口。

## 2. 首次准备与一条构建命令

目标是原生 ARM64、ROS 2 Jazzy，underlay 固定为 `/opt/ros/jazzy`。需事先准备
编译器、CMake、colcon、ROS 和当前源码所需的数学/消息依赖；上游
`nav2_smoother` 及其运行插件由 Jazzy underlay 提供。脚本不会安装依赖。

已配置好 Ubuntu/Jazzy 软件源的机器可先安装 Ninja；ccache 是可选加速器。
这些是首次准备命令，不由构建入口自动执行：

```bash
sudo apt update
sudo apt install ninja-build ccache
```

这两项不代表完整依赖已齐全。不要为精简构建对整个历史 workspace 无差别安装依赖，
以所选包及实际 CMake 缺项为准。建议在首次构建前完成依赖准备：系统 dpkg 包版本改变后，
缓存的工具链身份会改变，需要新的输出目录。

在导航仓根，默认一条命令同时构建定位和导航：

```bash
cd "$ARM_NAV"
scripts/arm64/build_workspace.sh --localization-repo "$ARM_LOC"
```

脚本自动定位 workspace；需要时显式传 `--workspace "$ARM_WS"`。默认输出是
`$ARM_WS/.arm64/<实际机器架构>-jazzy-runtime`，不覆盖普通 `build/install/log`。
默认行为：

- 21 个源码包，按真实依赖顺序串行构建包；每包编译并发 `2`，链接池并发 `1`。
- Ninja、Release、`BUILD_TESTING=OFF`，生成 compile commands。
- 自动使用 PATH 中已安装的 ccache；缓存放在
  `$ARM_WS/.arm64/ccache/<实际机器架构>`。未安装则正常编译，不自动下载。
- 使用干净 Jazzy 构建环境，仅复用当前输出的 overlay；不继承任意旧 overlay 或 CPU 预设。
- workspace 级锁阻止定位/导航构建同时运行；启动入口也会检查构建锁。

默认 21 包由导航 19 包、共用 `livox_ros_driver2` 消息包和 `lightning` 组成：

```text
robots_dog_msgs          robot_debug_msgs          robot_debug_tools
navigo_epoch_msgs        bondcpp                   nav2_lifecycle_manager
navigo_util             navigo_core               navigo_costmap_2d
navigo_map_server        navigo_navfn_planner      navigo_path_planner
navigo_path_controller   navigo_mppi_controller    navigo_behavior_tree
navigo_bt_navigator      navigo_behaviors          navigo_velocity_optimizer
robot_navigo             livox_ros_driver2         lightning
```

Navfn、MPPI、BT/costmap 的动态插件也显式包含在白名单中，不能只按 executable 判断依赖。
`navigo_epoch_msgs` 保留必要的接口依赖；这不等于启用 epoch 实验行为。

## 3. 常用选项与缓存边界

以下选项传给 `build_workspace.sh`：

| 选项 | 含义 |
|---|---|
| `--plan` | 只读显示包级 `build/install/reuse` 计划，不写输出或编译；仍要求本机具备构建工具并读取当前依赖身份 |
| `--scope all` | 默认，选择完整 21 包；跨仓发版和共享接口变化优先使用 |
| `--scope localization` | 选择定位及本地依赖闭包，当前共 4 包 |
| `--scope navigation` | 选择导航及共用 Livox 消息，当前共 20 包 |
| `--jobs N` | 编译并发预算，允许 1–8，默认 2；包仍串行，不是每包再乘一层并发 |
| `--rebuild` | 让所选包重新进入构建，保留 Ninja/ccache 缓存；不是删除目录后的冷建 |
| `--output /absolute/path` | 指定独立、持久的构建目录；不能用源码树、普通 build/install/log 或不属于此工具的非空目录 |
| `--host-validation` | 显式允许 x86 本机验证；产物记录实际架构，不能用作 ARM 二进制或板端验收 |

```bash
# 查看本次需要重编什么
scripts/arm64/build_workspace.sh --localization-repo "$ARM_LOC" --plan

# 内存紧张时降低编译并发；不改变运行算法或线程参数
scripts/arm64/build_workspace.sh --localization-repo "$ARM_LOC" --jobs 1

# 新工具链或独立冷建批次使用全新目录
scripts/arm64/build_workspace.sh --localization-repo "$ARM_LOC" \
  --output "$ARM_WS/.arm64/arm64-release-candidate-01"
```

编译输入、工具链、构建规则、依赖签名和已安装产物摘要共同决定复用。
只有 YAML、launch 等运行资源变化时可只执行 install；输入和产物都未变时直接 reuse，避免全仓重复 CMake。
安全门 Python 代码和定位启动入口参与构建输入，更新时重新调用构建以刷新 Debug 元数据或
入口摘要；Ninja 仍复用未变的 C++ 对象，不将新版脚本配上旧的生成身份。
共用消息/头文件/库变化会沿依赖图使消费者失效。`--scope` 不意味着可以混用旧 ABI：
若只更新导航却留下依赖旧消息的定位产物，检查和启动会拒绝该不一致组合，需要以
`--scope all` 补齐重建。

工具链或 dpkg 依赖集合改变时脚本要求新 `--output`。`/usr/local` 等非包管理器依赖
不在 dpkg 摘要内，修改后也应使用新输出完整构建并记录外部库版本。
不得手工改 `state.json` 或忽略产物摘要来强行复用。

缓存采用每包的真实构建来源。纯文档提交或其他不影响该包输入的 commit 更新不应触发
全包重新编译；此时 manifest 的当前源码 SHA 与复用产物的 `built_from` SHA 可以不同，
后者仍保留原始构建身份，不伪装成刚用最新 SHA 编译。

## 4. 静态产物检查与资源记录

构建成功后执行：

```bash
scripts/arm64/check_workspace.sh --localization-repo "$ARM_LOC"
```

使用自定义输出或 scope 构建时，检查传相同 `--output` / `--scope`；host 验证必须继续
带 `--host-validation`。检查验证 manifest/state、产物摘要、包解析前缀、依赖签名、
ELF 架构及缺失动态库，并确认 `nav2_smoother` 来自 Jazzy underlay。
结果写到输出根的 `check.json`。这是静态检查，不证明节点就绪、定位精度或运动安全。

输出目录的重要记录：

| 路径 | 用途 |
|---|---|
| `state.json` | 最近成功构建的缓存签名、依赖签名、产物清单及每包原始构建来源 |
| `manifest.json` | 最近构建尝试的总体状态、源码身份、包动作与当前成功状态摘要 |
| `runs/<时间>/report.json` | 单次构建命令、结果、耗时和资源测量 |
| `runs/<时间>/build.log`、`colcon/` | 编译输出和 colcon 包级日志 |
| `check.json` | 静态产物检查结果 |

构建报告每 250 ms 采样构建进程及其后代的 RSS 总和
`peak_descendant_rss_sum_bytes`，共享页可能重复计入，短峰值可能漏采。
`--jobs` 限并发，未设置内存硬上限。冷建、同输入零改动复用、单业务包增量和共享接口变化
应分别在目标板测耗时、峰值 RSS 和 ccache 命中；当前不承诺未知的加速倍数或 ARM 内存峰值。
新 `--output` 会获得新的 CMake/Ninja 构建目录，但仍可能命中共用 ccache，测量时要注明
ccache 是冷还是热；不要把这种构建误记为所有缓存皆空。

构建失败或中途源码变更不会记录为新成功状态。构建缓存本身可变，不能当作上一版不可变
回退产物；启用前另行保存上一版完整产物、配置/地图、manifest 和服务环境。

## 5. 使用显式站点配置启动

在批准的运行窗口，分别在两个终端启动定位、导航；先用 `--dry-run` 检查最终命令。
将以下路径替换为实际已核对的站点文件。启动器保留调用者 CWD，因此定位 YAML 内的
相对 `system.map_path`、recover 文件等仍受 CWD 影响，优先使用已核对的绝对路径。
二维地图必须与定位分块地图/索引的坐标和版本一致。

```bash
cd "$ARM_WS"
"$ARM_NAV/scripts/arm64/run_localization.sh" --localization-repo "$ARM_LOC" \
  --config /absolute/site/lightning.yaml --dry-run

"$ARM_NAV/scripts/arm64/run_navigation.sh" --localization-repo "$ARM_LOC" \
  --map /absolute/site/map.yaml --params /absolute/site/navigo_params.yaml --dry-run
```

确认文件、运行环境和启动窗口后，去掉相应命令的 `--dry-run`。采用自定义构建输出时
两条启动命令也传相同 `--output`。`--config` 和 `--map` 必填；`--params` 可省略并使用
安装的已审阅默认 YAML，实际交付仍建议显式传入并留存摘要。默认 `use_sim_time=false`，
只有回放/仿真才加 `--sim-time`。host 验证入口仍须显式 `--host-validation`。
这些启动器不操作 systemd、不启动驱动、不切自治模式、不直接发送 SDK/UDP 运动命令；
连接既有控制链后的运行另行验收。

ARM 导航入口使用普通 `lightning_nav_bringup.launch.py`，明确关闭
`enable_waypoint_follower` 和旧 PointCloud2 可视化转换器 `enable_livox_converter`。
独立 C++ `livox_scan_projector` 继续消费 CustomMsg，生成 LaserScan；安全门也保留。
默认组件容器为地图、规划、控制、恢复、速度平滑、BT、LifecycleManager 七个组件。
上游 `nav2_smoother` 继续独立进程启动，其库搜索优先 Jazzy underlay，保持与 Navigo fork
的 ABI 隔离；不要将它并入容器或换成历史硬编码库路径。

`robot_navigo` 的 ARM profile 不编译/安装旧 UDP/LCM 速度发送器、重复 TF/Odom 桥、
回放/仿真工具和开发资源。相机、SDK/ControlMux、autonomy_mux、真实 Livox 驱动均不在
这 21 包构建中，仍由独立交付链和服务管理；不从当前构建推定它们已升级。

## 6. Livox 消息与驱动环境隔离

本 overlay 中的 **`livox_ros_driver2` 只有消息类型，没有硬件驱动 executable**。
它从配对 LocMap 的 `thirdparty/livox_ros_driver/msg` 生成 CustomMsg/CustomPoint，
Lightning 与扫描投影共用这一份 ROS 类型。脚本会在存在标准路径硬件驱动源码时比较
消息语义；驱动位于其他路径或已单独部署时，还需核对其实际消息定义。

真实 Livox 驱动服务继续使用原先独立的 underlay/overlay 环境。不要让它 source 此 ARM
overlay 后再执行 `ros2 run livox_ros_driver2 ...`：同名消息包会遮蔽真实驱动包入口。
消息兼容与 topic/type/QoS/时间戳契约验证完成后，通过 DDS 连接定位和扫描消费者。

## 7. 每次正式启用前的验收顺序

1. 在 develop 审查并回归目标改动，按 `cherry-pick -x` 迁移最小提交；冻结两仓 SHA。
2. 用独立输出完成本机 host build、静态检查、完整旧 bag 回放和闭环仿真。
   runner 验证精简运行集合时显式传 `--arm-runtime`；仿真/回放工具从源码运行，不因此
   加回板端安装集合。host 输出不可复制为 ARM 二进制。
3. 板端更新配对 ARM 分支，原生构建、静态检查，保存日志、资源和产物身份。
4. 另行批准服务切换及上狗测试窗口，核对实际加载版本、输入和 TF/时钟、Debug 输出、
   目标导航及人工接管/停车。回放成功或 SE2 到达不替代物理验收。

2026-10-31 本机证据位于 workspace 的 `analysis_outputs/arm64_release_20261030/`：

- 21 包编译和静态产物检查通过；无变化复用约 1.83 秒，纯配置安装约 1.86 秒，均无算法编译。
- 512.48 秒完整旧包：5,118 帧雷达、102,361 条 IMU 数量和有序内容摘要一致，产生 106 条规划路径，Debug/版本/退出检查通过。
- `/odom/current_pose` 和 `/lightning/pose` 各 102,209 条，重复、回退和 frame 错误均为零；历史软件对照最大位置差约 4.2 mm，不代表物理精度。
- SE2 闭环的到达、取消、TF 丢失、扫描丢失四例通过。到达目标相距 4 米，最终位置误差 0.071 m、朝向误差 0.249 rad，满足原配置的 0.25 m/0.25 rad 阈值。
- 默认组件容器、无 profiling/临时退出规避环境；各场景正常退出。闭环模型未验证真实动力学、SDK 或机械狗运动。

上述时间均为 x86 本机测量，不能推成 ARM 冷构建收益或内存上限。板端原生构建、服务切换、
硬件输入和实机导航仍未验收，本轮未部署。
