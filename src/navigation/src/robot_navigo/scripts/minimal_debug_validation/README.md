# Minimal Debug 本机验证

这两个 runner 只创建本轮独立 ROS domain 的进程和文件；不连接机器人、不启动 UDP/LCM/SDK，不改现有算法参数、BT、checker 或恢复策略。不要与 Matrix 的 LCM 启动入口混用。

`--install-prefix` 指向本轮构建的 overlay 根目录（包含 `setup.bash`）。关键导航包和插件（包括修复后的 `nav2_lifecycle_manager`、`bondcpp`）必须全部来自该 overlay，不允许默默回退到根 install 的旧实验产物。实际映射 manager core 的容器 PID 还必须加载唯一的修复版 `libbondcpp.so`；隔离的上游 smoother 保留原 `/opt` 依赖，只记录其真实路径/hash。runner 拒绝设置 `LD_PRELOAD` 或 `MAGICK_THREAD_LIMIT` 的临时规避环境。自动 source 后重新执行，设置 localhost discovery，并清除远程发现服务器/静态 peer 配置。所有子进程各自有本轮 process group；退出先标准 lifecycle SHUTDOWN 并确认八个托管节点 finalized，保持 clock 到导航关闭后再停止传感器。只清理自己创建的组，不使用全局 `pkill`。输出目录必须是新的。

## 1. 旧传感器 bag 开环

```bash
/usr/bin/python3 /home/charles/project/colcon_ws/src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/minimal_debug_validation/replay.py \
  --install-prefix /home/charles/project/colcon_ws/install/minimal_debug \
  --output /home/charles/project/colcon_ws/analysis_outputs/lifecycle_fix_replay_20261017/replay_default_04 \
  --bag /home/charles/datasets/rock_dog/0919/full_robot_debug_20260918_225259_848790_89926/bag \
  --map /home/charles/project/colcon_ws/data/hz_office_2f_part_0718/map.yaml \
  --config /home/charles/project/colcon_ws/src/lightning-lm/config/mid360.yaml \
  --duration 180 --observe 15 --goal -0.45 -0.5 2.63 --domain 42
```

只播放配置对应的 LiDAR/IMU；TF、定位、scan、map、Action 和控制输出由新进程产生。原始 LiDAR/IMU 默认不重复录制以控制体积，确需保存时使用 `--record-raw-sensors`。发送目标前要求定位状态 NORMAL、最近 pose/scan/odom、TF 和 lifecycle active。目标通过真正的 `/goal_pose`（`frame_id=map`）订阅入口进入 BT；随后只观察其新 UUID 并调用原结果/取消服务，不另发重复 Action。若出现多个新 UUID，拒绝猜测要取消哪一个。例中目标来自当前二维地图的空闲区；可用 `--goal X Y YAW` 指定其他合法测试目标。观察后受控 cancel，等待真实结果，录制终态尾段。

先让真实 bag `/clock` 生效，再以 `--use-sim-time` 开始录制并等待 recorder 订阅就绪，然后启动规控；定位的最初几帧及录制前的 lifecycle 事件不在新包内。静态 TF、map、build_info 用 transient-local 订阅保留，发送目标后的业务链完整录制。闭环同样先建立 plant 时钟。新包 log_time 与 TF/pose/scan/goal 的原始 header 使用同一回放时域；不使用墙钟时间录包，也不在打包时改时间戳。

这项测试中的历史运动不随新速度变化，所以 **arrival=N/A**；通过只表示当前输入下的旁路记录、真实请求/状态和输出可观测。不要把它当作闭环成功。bag 必须够长，不能在初始化或 cancel 完成前结束；结束过早会判失败，不伪造追加传感器/TF。

完整原包模式使用 `--full-bag-mode --goal-at-source-time 270 --goal -3.7772 -2.4332 -1.25793 --domain 42`，替换上例的 `--duration/--observe`，输出另选新目录 `analysis_outputs/full_replay_readable_layout_20261017/full_replay_01`。该原包长 512.4805 秒，前约 277 秒基本静止；270 秒下发目标以保留完整开头，又能在随后的实际运动中观察规划。目标位于旧轨迹后段；约 3.6 m 直距、4.9 m 绕门静态路线只是离线可行性初筛，仍须检查真实规划器结果。

完整模式禁止 duration 上限和非零 offset。player 先暂停，在真实初始 clock、recorder 和原始 CDR 审计订阅就绪后恢复，完整播放两个原始传感器话题。EOF 前 20 秒受控取消（若已结束则保留原结果）、零命令尾段和标准导航 lifecycle 关闭；定位、原传感器和 recorder 继续到自然 EOF，校验退出码 0 并 drain 后才停录。不合成延长 clock，不把 500 秒截断称作完整。`full_source_coverage.json` 对原包与实际接收的 raw CDR 逐 topic 校验完整数量、首末 header 和有序哈希，证明首尾样本均已播放。`/trajectories` 的 `Optimal Trajectory` 是原 MPPI 最优输出，`/transformed_global_plan` 是变换后的参考路径；二者均录制，不修改原算法 horizon 或输出开关。

## 2. 普通导航算法闭环

```bash
/usr/bin/python3 /home/charles/project/colcon_ws/src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/minimal_debug_validation/closed_loop.py \
  --install-prefix /home/charles/project/colcon_ws/install/minimal_debug \
  --output /home/charles/project/colcon_ws/analysis_outputs/lifecycle_fix_replay_20261017/closed_loop_final_02/arrive \
  --scenario arrive --domain 43
```

默认依次运行正常到达、受控取消、TF 断流、scan 断流，分别用独立 domain；可用 `--scenario arrive|cancel|tf_loss|scan_loss` 单独运行。默认地图为 8 m 带边墙空房间。SE2 plant 仅积分真实 `/cmd_vel_safe`，提供 odom/TF/scan/定位状态，不替代规划、平滑、BT、MPPI、checker 或 gate。使用批准基线 `navigo_params.yaml`，只改变时间、map 路径与 ordinary epoch=false；没有 forward-alignment、epoch 或随机数参数覆盖。

本轮最终批次逐场景运行，`arrive/cancel/tf_loss/scan_loss` 分别选空闲低域 43/44/45/46，源记录关闭对照用 47，直接 Ctrl+C probe 用 48；它们是隔离选择，不是算法参数或 DDS 故障根因结论。所有输出均用新目录，中间失败/旧构建的结果保留。

到达必须同时满足真实 Action SUCCEEDED、真实 checker=true 的旁路记录、独立 plant 最终几何误差和实际 modeled displacement。反例要求看到真实已有失败分支，随后受控取消/原始失败并检查尾段安全命令为零。它验证的是平面规控软件闭环；**不验证 Lightning、Matrix 物理、SDK 或实机**。

## 3. 组合容器线程预算（可选）

两个 runner 默认保留上游 `component_container_isolated`。要使用有明确线程预算的隔离容器，在相同 runner 命令后加 `--component-executor-threads 2`；这会同时向导航 launch 传入 `use_bounded_component_container:=true` 和 `component_executor_threads:=2`。不要同时指定 `--no-composition`。它只限制每个组件的 executor worker 数，不限制 DDS、costmap 或 Action 等辅助线程，也不改变 MPPI batch/horizon、critic、随机数或控制频率。

直接使用 `bringup_launch.py` 或 `lightning_nav_bringup.launch.py` 时，三个参数必须一起生效：

```text
use_composition:=true use_bounded_component_container:=true component_executor_threads:=2
```

`use_bounded_component_container` 默认是 `false`；`component_executor_threads` 默认值 `2` 本身不会启用新容器。`--component-executor-threads 0`（launch 中对应 `component_executor_threads:=0`）保留同一个新容器 binary 的 CPU 自动线程数，供对照使用。要回退到上游容器，runner 不传 `--component-executor-threads`，或在直接 launch 中设置 `use_bounded_component_container:=false`。

所选 `--install-prefix` 必须实际安装本轮 `robot_navigo` 的两个 launch 文件及 `lib/robot_navigo/bounded_component_container`；仅修改 `src` 不代表运行生效。检查 `stage_manifest.json` 中实际 launch 参数、进程退出和清理结果，结合所选 overlay 的可执行文件 hash 确认产物。已有本机 x86 的四种 SE2 功能/故障场景验证，不代表 ARM 板端或更重并发负载也适用；每组件两个 worker 的配置仍需在目标负载下核对控制节拍、服务响应和完整退出。

## 4. 定位进程 CPU 亲和性（可选）

Lightning YAML 的 `performance.cpu_affinity` 默认为 `[]`，不改变原命令或亲和性。`replay.py` 从输出目录的有效 `lightning_config.yaml` 读取非空列表，仅为 `localization` 命令前置 `taskset --cpu-list CPU列表`，在 native executable 及其库加载前设置集合；导航、recorder、player 和父 runner 不受此开关约束。CPU IDs 必须是无重复的整数、全部属于调用方当前允许集合，并且本机可找到 `taskset`，否则启动前拒绝执行。`replay_inputs.json` 和 `stage_manifest.json` 保留实际命令。

先依据本机 CPU 拓扑与允许集合选取逻辑 CPU IDs，不能照搬其他 x86 或 ARM 平台的编号，也不能把两个逻辑 CPU 直接称为两个物理核。所选 overlay 中的 Lightning 必须包含预算校验：非空配置会检查主线程有效 mask，所有已存在库线程也必须已具有完全相同的集合；未受控则拒绝启动。后建线程继承该集合，仅在 `main` 中修改调用线程不足以约束库在加载时已创建的线程。

手动启动 `run_loc_online` 或 `run_loc_offline` 时，同样须让 `taskset` 先于原生命令 exec。例如只有在核对本机拓扑、允许集合及 YAML 中确实选择 `[0, 2]` 后，才使用 `taskset --cpu-list 0,2 ros2 run lightning run_loc_online --config 有效配置.yaml`；offline 命令也采用同样前缀。回退时将有效 YAML 中的 `cpu_affinity` 设为 `[]`，并移除手动 `taskset` 前缀。

清除会冲突的 OpenMP/第三方线程绑定配置；`runtime_manifest.json` 记录 `OMP_PLACES`、`OMP_PROC_BIND`、`GOMP_CPU_AFFINITY`、`KMP_AFFINITY`。库仍可能在运行中重新绑定线程，验收须逐 TID 核对 `/proc/PID/task/TID/status` 的 `Cpus_allowed_list`。亲和性是 OS 调度预算，CPU 秒/帧的计算量优化、初始化耗时、输入吞吐、位姿质量和发布频率分别验收；设置集合本身不证明节省 CPU 工作量。

## 证据

- `runtime_manifest.json`：所有关键 package prefix、实际 executable/依赖路径和 hash、ROS distro/domain、harness hash。
- `profile.json`、`navigation_params.yaml`：原参数与实际参数；保留 checker、MPPI、BT 范围。
- `stage_manifest.json`：每个进程/阶段、退出码及仅本轮 group 的清理结果；launch leader 的零退出不能掩盖 native 子进程崩溃。
- `events.jsonl`、`observations_report.json`：接收时间、原始 Action UUID/status、真实旁路 checker/branch 和实际调用链证据。
- `geometry_report.json`：独立 plant 几何 oracle（仅闭环）。
- `bag/`、`schema_report.json`：新消息和静态 TF/版本录包；检查 MCAP schema，并至少按每种实际 datatype 用 native ROS class 解码一条。不会订阅旧 `/controller_server/debug`。
- `viewing_bag/viewing.mcap`、`viewing_bag/metadata.yaml`、`delivery_bag_report.json`：供 Foxglove 直接打开的完整自包含 MCAP 及标准 ROSbag 目录。保留全部原始 topic/CDR/log/publish/sequence 和已有完整 schema，仅补缺失的实际 Action schema；逐条字节/时间和独立 reader 对照通过。精确类型 `livox_ros_driver2/msg/CustomMsg` 先原生解码一条并核对 Header，后续逐条检查序列化 Header，避免仅为读时间戳而构造全部 Python 点对象；未知 CDR representation 回退原生解码，其余类型仍逐条原生解码。报告分列 `native_decoded_records` 和 `cdr_header_audited_records`，两者之和必须覆盖全部记录。原始 `bag/` 不改。交付时只打开这一个 MCAP，避免与原包/companion 重复。
- `time_domain_report.json`：逐条 log_time 与非零 header/TF/clock 时戳核验，CustomMsg 使用上述已原生校验的 Header 快路；短试验沿用 300 秒时域界限，完整原包按 metadata 起止包络及显式 2 秒容差核验，允许跨全长缓存的原 header。时间戳不修改；该检查不是网络或算法延迟指标。零时戳元信息不冒充有采样时间。`delivery_bag_report.json` 的 `generator_source_path`、`generator_sha256` 和 `timestamp_audit_method` 记录离线后处理模块实际 import 身份；惰性 import 可晚于启动时的 `runtime_manifest.json`，与 native 运行阶段的构建指纹分别追溯。
- `action_schemas.mcap`、`schema_recovery.json`：内部 schema 恢复审计副产物。本机 rosbag2 对隐藏 Action 仍可能写空 schema；定义来自当前安装 Action/MSG/IDL，不覆盖新/旧原包。每条原 CDR/log time/publish time/sequence 必须逐字节一致，所有 Action 消息原生解码和独立 rosbag2 reader 对照通过。交付观看只打开完整 `viewing_bag/viewing.mcap`；不要再同时打开或播放原包和 companion，避免重复 Action 流量。
- `build_metadata_report.json`、`view_contract_report.json`：自有 source/reporter 的完整 commit、dirty/build 指纹，以及 UUID/路径 revision/终态下 checker 不能越界显示的实测检查。原生 Action 来源构建 unknown 不被伪造。
- `result.json`：该阶段通过/失败和限制；schema 缺失不能通过。

`--observer-off` 只关闭展示 observer；`--observations-off` 仅通过 `enable_debug_observations=false` 关闭源旁路记录，不改变算法阈值。主四例完成后，再运行 `--scenario arrive --observations-off` 作诊断开关对照。`control_timing_report.json` 分别报告 `/cmd_vel_nav`、`/cmd_vel`、`/cmd_vel_safe` 的 DDS 发布时刻间隔及订阅 callback 间隔；这是控制输出节拍观测，不能当作算法计算耗时。不能把两次随机 MPPI 轨迹不同直接判成行为逻辑改变。

## 历史 Jazzy 退出故障及默认环境复验

上一轮本机默认组合启动在八个节点均 finalized 后，容器的 Context 析构调用已经卸载的系统 `nav2_lifecycle_manager_core` 回调而崩溃。零目标、关闭全部新增 C++ 记录与元信息 publisher 的隔离 probe 也复现；`MAGICK_THREAD_LIMIT=1` 未修复。历史取证见 `analysis_outputs/debug_implementation_20261017/native_shutdown_rootcause.md`。

上一轮显式 preload 系统 manager 库只证明受控环境功能，不构成生产修复。本轮改为构建并核验修复包、默认组合进程启动和退出；runner 拒绝上述环境规避，`runtime_manifest.json` 明确记录其缺席以及真实加载库 hash。新结果写入 `analysis_outputs/lifecycle_fix_replay_20261017/`，不得拿旧对照结果代替新验收。`shutdown_probe.py` 仅作故障隔离，不代替完整 Debug 合同验收。

静态/安全单测不启动 ROS：

```bash
/usr/bin/python3 -m pytest /home/charles/project/colcon_ws/src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/minimal_debug_validation/test_validation.py -q
```
