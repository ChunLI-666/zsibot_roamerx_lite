# 定位 / 导航性能采样

同一份模块随两个独立仓库交付。全局 `LOCNAV_PROFILE=0|1` 缺省为 `0`；关闭时 runner 直接 exec 原命令，不创建目录、加载采样库或启动监控线程。开启采样需要显式选择新输出目录。只采集本次启动的进程树，支持组合式导航容器、子进程和子 session。不会修改 governor、sysctl、ROS 时间戳、算法参数或系统服务。

构建安装后，也可分别使用 `ros2 run lightning profile_workload` 或
`ros2 run robot_navigo profile_workload`，其余参数与下面的 Python 入口相同；
报告命令为各包的 `profile_report`。采样库必须与目标 CPU 架构匹配，
本次实验私有解包的 x86 工具不作为 ARM 交付产物。

```bash
# source 与实际构建相符的 ROS/overlay；以下路径均由调用方指定。
export LOCNAV_PROFILE=1
export LOCNAV_PROFILE_LIB=/absolute/path/to/libprofiler.so
python3 scripts/performance/run.py --backend gperftools \
  --output /absolute/path/to/new_profile -- /absolute/path/to/program ARGUMENTS

# 可访问 perf_event 时，可采 CPU 调用栈（保留线程信息的 perf.data）。
python3 scripts/performance/run.py --backend perf \
  --output /absolute/path/to/new_perf_profile -- /absolute/path/to/program ARGUMENTS
```

`LOCNAV_PROFILE_FREQUENCY` 缺省为 99 Hz。gperftools 使用 CPU timer，清除 `CPUPROFILE_REALTIME`；只在最终动态原生 ELF 的单次 Popen 环境注入，拒绝 env/taskset/ros2/解释器及解析后同名别名作为采样目标；其合并栈不提供逐线程归因，也不采 kernel 栈或 off-CPU 等待。程序必须正常退出以刷新 profile；SIGKILL/崩溃可能导致缺失或截断，不能把缺失算作零成本。perf 权限探针失败会退出并保存错误，不会默默改用其他 backend。

建议独立构建保留原有 Release 优化，再附加 `-g1 -fno-omit-frame-pointer`；不改采样率、batch、horizon、critic、地图或算法阈值来制造收益。Lightning 须设置独立 `CMAKE_RUNTIME_OUTPUT_DIRECTORY`，避免覆盖源码 `bin/`。第三方库无符号、内联和尾调用会影响栈的完整性；保留实际 executable/DSO 的路径及 hash。

## CPU 图与内存图

```bash
python3 scripts/performance/report.py --profile /absolute/path/cpu.prof \
  --binary /absolute/path/actual_executable --pprof /absolute/path/google-pprof \
  --kind cpu --output /absolute/path/to/new_cpu_report
```

每个 native 子进程单独转换其 profile，`--binary` 必须指向该进程真正执行的文件，不能一律填 ros2/Python。输出 `flame.svg`、`stacks.folded`、`top.txt`、`summary.json`；图宽为采样数，横轴不是时间线。CPU 样本占比与 wall 耗时、callback 延迟、进程 CPU 秒分别解释。可直接传入 `--collapsed FILE` 渲染由其他工具导出的 folded stacks。

进程树的 `processes.csv` 保存累计 CPU ticks、RSS/PSS/VmHWM；`resources.csv` 保存同时刻聚合值；`system.jsonl` 保存频率、温度、共享 cgroup 限流/OOM 指标。聚合 RSS 会重复计算共享页，应同时看 PSS；聚合采样峰值不等于各进程 VmHWM 之和，也不保证捕获亚采样间隔峰值。频率变化本身不能证明热降频。短命子进程可能在两次采样间退出，CPU 计数是有边界的观测，不冒充完整计费。

堆分配需另跑一次，不能用 RSS 伪造内存火焰图：

```bash
export LOCNAV_PROFILE_HEAP_LIB=/absolute/path/to/libtcmalloc_and_profiler.so
python3 scripts/performance/run.py --backend heap \
  --output /absolute/path/to/new_heap_profile -- /absolute/path/to/program ARGUMENTS
python3 scripts/performance/report.py --kind inuse --profile /absolute/path/snapshot.heap \
  --binary /absolute/path/actual_executable --pprof /absolute/path/google-pprof \
  --output /absolute/path/to/new_heap_report
```

`inuse` 是某个快照仍存活的堆字节，`alloc` 是累计分配字节，二者均非 RSS。heap backend 更换 allocator 为 tcmalloc，会改变 CPU、内存与延迟；只用于定位分配热点，不用于宣称原 allocator 的 CPU 收益或内存泄漏。工具不会自动安装依赖。

需要更细的堆快照时，可显式设置 `HEAP_PROFILE_INUSE_INTERVAL=16777216`
（新增高水位每 16 MiB）和 `HEAP_PROFILE_ALLOCATION_INTERVAL=268435456`
（累计分配每 256 MiB）；它们与 10 秒时间间隔一并记入 manifest。
退出快照可能接近空堆，分析存活内存时应选择运行中的快照，不能把退出快照当峰值。

## 采样式活跃堆（显式 `heap_sampled`）

`heap_sampled` 使用真实 `MallocExtension::GetHeapSample()`，按分配字节间隔采样
当下仍存活的对象及其分配栈。它与上述 `HEAPPROFILE` 逐次分配追踪分开；
固定 `TCMALLOC_SAMPLE_PARAMETER=524288`，普通线程每 10 秒采集完整 `heap_v2`
字符串及实际 maps、PID/starttime、TID、wall/thread-CPU 成本，退出唤醒并 join。
不在 signal handler 中分配或采集，不使用接近空堆的退出快照冒充运行期内存。
采样、更换 allocator 及采集器自身均有开销，不能把该轮 CPU 当原 allocator 收益。

helper 源码在 `heap_sampled/`，只需调用方显式提供**同版本/架构**的真实开发头和
plain `libtcmalloc.so.*`，独立构建，不修改产品 CMake/native target，不自动下载或安装。
已有 helper 也必须提供匹配 build manifest；runner 检查 helper/library SHA、
真实头文件来源摘要、ELF 架构/ABI、helper DT_NEEDED 与 plain allocator SONAME。
combined、minimal、额外预加载 allocator、错误架构或不匹配产物在启动前拒绝。
构建产物是调用方编译目标，x86 私有产物不能复制到 ARM 使用；ARM 须独立验证。
已安装的 `run.py` 只读取外部绝对路径产物，不要求运行机保留源码或开发头。

```bash
python3 scripts/performance/heap_sampled/build_helper.py \
  --include-dir /absolute/matching/include \
  --tcmalloc-lib /absolute/plain/libtcmalloc.so.4 \
  --output /absolute/new_helper_build --with-fixture
python3 scripts/performance/heap_sampled/verify_helper.py \
  --build /absolute/new_helper_build --pprof /absolute/google-pprof \
  --output /absolute/new_helper_verification

LOCNAV_PROFILE=1 python3 scripts/performance/run.py --backend heap_sampled \
  --heap-lib /absolute/plain/libtcmalloc.so.4 \
  --heap-sampled-helper /absolute/new_helper_build/liblocnav_heap_sample.so \
  --heap-sampled-manifest /absolute/new_helper_build/build_manifest.json \
  --output /absolute/new_live_heap -- /absolute/dynamic_native_program ARGUMENTS
```

对应环境路径为 `LOCNAV_PROFILE_HEAP_LIB`、`LOCNAV_PROFILE_HEAP_SAMPLED_HELPER`、
`LOCNAV_PROFILE_HEAP_SAMPLED_MANIFEST`。全局 `LOCNAV_PROFILE=0` 时在检查这些依赖
之前直接 exec，缺少 helper 也不影响禁用路径；`resources` 不加载它们。

此 backend 只接受**直接、动态链接的业务原生 ELF**，拒绝 ros2/launch/Python、
shell、env、taskset 等启动器和解释器，不向整棵启动树预加载。fork/exec 子进程
不在该单进程采集契约内，观测到额外进程或多个/缺失 sampler 会失败。
短命子进程仍可能漏观测，不能宣称完整子进程覆盖。联合 ROS 启动须在实际目标
Node/ELF exec 边界单独注入；不要对现有 `minimal_debug_validation` 整树选择
`LOCNAV_PROFILE_BACKEND=heap_sampled`。该入口的默认 resources/CPU 路径保持。
需要 CPU 亲和性时，将 taskset 放在 profiler **外侧**，在所有库加载前建立约束；
helper 线程继承同一集合，仍须通过原生已有线程检查和运行期 mask 审计。

输出 `heap_sampled_profiles/PID-starttime/sample_N.{heap,json,maps}`、sampler
manifest/summary，以及原资源表。必须有非空运行期样本、正确 PID/实际 maps 和
正常 worker 回收；缺失、截断或采集错误不能当零占用。每个快照是一个观测点，
不是连续峰值。helper 默认不开线程/文件，但显式预加载 helper 会连带其 tcmalloc
依赖；正常禁用运行应同时不预加载这两个库。

```bash
python3 scripts/performance/report.py --kind sampled_inuse \
  --profile /absolute/new_live_heap/heap_sampled_profiles/PID-START/sample_N.heap \
  --sample-metadata /absolute/new_live_heap/heap_sampled_profiles/PID-START/sample_N.json \
  --binary /absolute/actual_native_program --pprof /absolute/google-pprof \
  --output /absolute/new_live_heap_report
```

report 校验完整 raw buckets/metadata，要求兼容 gperftools pprof 明确确认 v2
`1-exp(-size/period)` 概率校正；图宽为**估计活跃堆字节**。raw 采中对象/字节与
pprof 校正后的估计分别保存。`heap_v2` 的方括号列重复同一份活跃样本，
**不含累计分配量**：禁止 `alloc_space`，禁止把多个快照相加当累计 allocation，
也不能把它当 RSS/PSS、精确普查或泄漏证明。合法空快照保留原始数据但拒绝画伪图。
`sampled_inuse` 不接受没有原始 profile/metadata 的 collapsed 输入。

## 导航联合回放与软件闭环

导航仓库 `scripts/minimal_debug_validation/{replay,closed_loop}.py` 的 gperftools 模式将完整工作负载父进程树按 resources 如实记录，再使用 Jazzy `ros2 run --prefix` / `ros2 launch --launch-prefix` 在明确白名单原生子进程前建立干净的 profile runner。启动器、Python helper、observer、recorder、player、plant 不预载 CPU 库；组件 DSO 在实际原生容器内采样。匹配以解析后的包路径和锚定 executable basename 共同约束，未知 launcher、自定义 executable 或冲突 prefix 必须先拒绝。其它 backend 与默认关闭分支保持原路径。该集成依赖实际源码/安装布局内同版 runner，不跨仓猜测工具位置。

使用已有 runner README 的 bag、匹配地图与经核对的 install-prefix。真实 bag 回放中的原运动不随新速度变化，arrival=N/A；`closed_loop.py` 是真实导航算法驱动 SE2 软件闭环，不等于实体动力学、SDK 或 ARM 验收。

性能优化验收分三步：先采样定位热点；关闭所有 profiler，在相同数据/参数/种子/优化等级下交替重复基线与候选并记录 CPU 秒/帧和 p95；再比较完整轨迹/成本/命令或定位数值及闭环结果。采样本身开销、后台编译、功耗/温度和源码并行变化均需记录。x86 结果不能外推 ARM 降频原因或绝对收益。

```bash
cd scripts/performance
python3 -m unittest -v test_run.py test_report.py test_heap_sampled.py
```

## 实时 CPU 核数与并发预算

将原始传感器以 `--rate 1` 回放，使用
`LOCNAV_PROFILE=1 LOCNAV_PROFILE_BACKEND=resources LOCNAV_PROFILE_FAST_CPU=1`。
`resources` 不加载 CPU/heap 分析器；额外快速线程每 100ms 只读取已发现 PID 的
CPU ticks，每秒读取一次线程 ticks。输出 `fast_cpu.csv`、`thread_cpu.csv`、
`fast_cpu_summary.json`。CPU 核数为进程树 CPU 秒 / 墙钟秒，不除以主机核数。
报告整段平均、滚动 1s 峰值和 100ms 突发峰值；采样峰值不等于硬实时上限。
新 PID 的慢速发现、退出前最后 CPU、10ms tick 和调度抖动均有误差，须核对
采样间隔并分别统计启动、静止和运动阶段，不能用加速离线回放占用代替实时核数。
线程表用于归因，不能再加到同一进程 CPU 上。

Lightning 的主 YAML 可显式加入调度预算（缺省保持 TBB 自动 / NDT 4 线程）：

```yaml
performance:
  tbb_max_parallelism: 1  # 0=原自动调度；正数限制 TBB 活跃并行度
  ndt_threads: 2         # 1..128；不改变 NDT 点、阈值或迭代配置
  cpu_affinity: []       # 默认继承；显式填本机允许的逻辑 CPU IDs，例如 [0, 2]
  imu_queue_depth: 10    # 1..65536；原 IMU 订阅深度
  lidar_queue_depth: 10  # 1..65536；原 LiDAR 订阅深度
  lio_queue_size: 1      # 1..4096；原 LIO 异步队列长度
  loc_queue_size: 1      # 1..4096；原定位匹配异步队列长度
  sensor_input_audit: false  # 可选 native 回调入口 Header 审计；默认不计算哈希
  publication_audit: false   # 可选 odom/pose 原生发布调用审计；默认不计算哈希
```

队列预算缺省保留上述原值，订阅可靠性、输入/发布频率、滤波和数学运算不变。
显式提高深度用于缓冲暂时拥塞，避免待处理输入被队列覆盖；内存需求及可能积累的
处理延迟随深度增加，不能把更长队列视为吞吐提升或硬实时延迟保证。必须核验 native
实际接收/处理数量、有序输入 Header 审计、队列丢弃和最大积压；旁路 source auditor
收到完整原包，不代表 native 订阅与内部队列也完整接收或处理。

`sensor_input_audit: true` 在 IMU、PointCloud2、Livox CustomMsg 各自回调入口，
对原整数 `sec*1e9+nanosec` 的小端 8 字节序列累计 count、first/last stamp 和
`ordered_header_fnv1a64`；FNV-1a64 offset 为 `14695981039346656037`、prime 为
`1099511628211`。Spin 结束每通道输出一条 `[INPUT_AUDIT]` JSON；未用通道
count=0、hash 保持 offset。它证明 native 回调入口的 Header 序列，既非完整 CDR
哈希，也不等于算法实际接受/处理数量；默认 `false` 不执行该哈希计算。

`publication_audit: true` 另为 `/odom/current_pose` 和 `/lightning/pose` 各自记录
原生 `publish()` 成功返回后的**实际消息 Header**：count、first/last stamp 和相同
LE8 FNV-1a64 有序哈希。`LocSystem` 在 Spin 结束并 `Finish()` 回收工作线程后，
析构输出两条 `[OUTPUT_AUDIT]` JSON；关闭时不做该哈希计算。格式化失败输出
`[OUTPUT_AUDIT_ERROR]`，缺失汇总不能算通过。它证明原生发布调用及其 Header 序列，
不证明 DDS、recorder 或其他 subscriber 已收到；没有新增 TF audit 通道。
对照 source 输入、native callback、算法处理、原生发布与录包观测时，须分层比较，
不能把 recorder 起录前缺少的输出直接判成算法掉帧。原 `[INPUT_AUDIT]` 契约保持。

`run_loc_online` 和 `run_loc_offline` 在工作线程创建前建立 TBB 生命周期预算，
初始及动态地图 NDT matcher 都读取相同配置。可独立实验
`OMP_WAIT_POLICY=PASSIVE GOMP_SPINCOUNT=0`，并保存这些环境变量。
这些是实验候选设置，须用所在平台的输入吞吐、队列、逐层位姿和频率验证后选择。
没有减少线程数就一定减少 CPU 或提高延时的保证。

导航两个 bringup launch 显式支持
`use_bounded_component_container:=true component_executor_threads:=2`；
保持每组件隔离的执行器，用同一官方 ComponentManagerIsolated，控制每组件
worker 数。默认关闭以保留原启动行为，`0` 在新容器中恢复自动核数。
该预算只对 composition 生效。最小验证入口对应
`--component-executor-threads 2`，`all` 闭环子用例会传递这个选项。


CPU 亲和性是额外的进程运行预算，与节省计算量分别解释。显式非空时，
须在 exec 原生程序及库加载前设置集合，不能只在 `main` 中修改调用线程；
后建线程继承该集合。先用本机 CPU 拓扑和调用方允许集合选取可用 IDs；本轮 x86 的 `0,2` 分属两个
独立物理 P-core，不能照搬到 ARM。平均核数仍用 CPU 秒 / 墙钟秒，不除以 2。
默认空集合不调用 affinity API；不改系统 governor、全机 CPU 集合或 cgroup quota。

非空配置会检查 CPU 位于调用方原允许集合，并读取有效 mask 验证。
所有已存在库线程必须已通过程序加载前的启动器限制到完全相同的集合；发现未受约束
的已有线程会拒绝启动。联合 replay 入口读取输出的有效 `lightning_config.yaml`，
非空时仅对定位命令前置 `taskset --cpu-list CSV`；导航、recorder、player 和父 runner 不变。
列表须为无重复整数，全部属于调用方允许集合，且 `taskset` 可用，否则预检拒绝启动。
默认 `[]` 保持原命令。实际前缀保存在 `replay_inputs.json` 和 `stage_manifest.json`。
手动在线/离线启动也须前置 `taskset`，例如核对本机允许集与 YAML 的 `[0, 2]` 后使用
`taskset --cpu-list 0,2 ros2 run lightning run_loc_online --config ...`；不能只填写 YAML。
预算未生效不会默默退回未受约束运行；回退须设 `cpu_affinity: []` 并移除手动前缀。
须清除冲突的 `GOMP_CPU_AFFINITY`、`OMP_PLACES`、`KMP_AFFINITY` 和启用绑定的
`OMP_PROC_BIND`。线程仍可能被外部库重新绑定，因此验收须在启动、实际匹配、
稳定段与退出前核对每个 `/proc/PID/task/TID/status` 的 `Cpus_allowed_list`。
上述绑定环境记录在 `runtime_manifest.json`，并分别验收初始化耗时、吞吐、位姿质量和发布频率。
作用域析构只恢复调用线程，应用应先停止并回收自身工作线程。
带预加载采样器的火焰图运行应与资源验收分开；若采样器提前创建线程，
必须在库加载前建立相同集合并保留审计证据，无此证据时使用空亲和性的诊断配置，
不能绕过前置检查。

DR 仅状态预测与时间轴正确性分开验证：主 LIO 保留完整协方差传播；DR
副本不消费协方差。重置从已完成积分的 LiDAR scan_end 开始，在同一锁区
重放所有已接收后续 IMU，每步用原始 IMU 时间戳作为积分终点。空 IMU 或
异常积分不能仅修改时间戳后发布有效状态。在线前后比较应使用同一时序修复
和字节选择掩码的控制组，另保留旧版本的回归失败记录。


## 分阶段等待见证（独立诊断开关）

`LOCNAV_STAGE_WITNESS=1` 在原生在线 `LocSystem::Init()` 开始时显式启用；
缺省或 `0` 关闭。关闭热路径只读原子 flag，不读时钟、不取采集锁、不分配、不开
新线程或逐帧日志；环境初始化在业务对象/ROS node 创建前进行。runner 只接受
未设置、`0`、`1`，其他值预检失败。它与全局 `LOCNAV_PROFILE` 分开，
`LOCNAV_PROFILE=0` 不会覆盖显式 stage 开关；仅设置环境不会为旧 binary 添加埋点。

退出并回收工作线程后 `[STAGE_WITNESS]` 汇总 lifetime stage 调用、monotonic wall、
当前线程 CPU、时钟/采集错误，以及 wall 最大和 CPU 最大各自的同次调用 witness。
每个 witness 保留原调用 TID、起止 monotonic 时间、该次 wall/CPU；只有线程上下文
确有原始 ROS Header 时才填整数 `original_header_ns`，否则为 null，不借用最新 IMU
伪造来源。已知的功能状态时间另列 `functional_stamp_seconds`，未知也为 null。
墙钟时长包含等待/抢占，线程 CPU 不包含其他 worker 的工作，不能据此当作整个
NDT/进程 CPU。阶段可能嵌套或重叠，inclusive 合计不能相加为总耗时。

该模式用于定位 callback、LIO/DR、PGO data mutex 等真正的等待位置，以及原生
TF/odom/pose/debug 构造发布阶段。启用会引入时钟、原子和最大值采集开销；正式 CPU
收益轮关闭它，诊断轮单列。`runtime_manifest.json` 记录实际环境值、
`stage_witness_enabled`，显式 `1` 不标成 `default_environment=true`。缺失/截断汇总、
时钟或采集错误要保留，不能解释成零成本或默认环境验收。

## 正确性修复与验收边界

PGO 在平滑前按最终输出时间检查 `timestamp > previous + 1e-6`；同/旧 epoch
不再重复 `PushDRPose`/`PushPose` 或回调，避免外层本已丢弃的重复输出先影响下一帧
EMA。融合输入仍先处理并留给下一新 DR 输出，正常新 epoch 的系数和计算保持。
Reset/DR 倒退会清门槛；`[PGO_OUTPUT_EPOCH]` 退出计数用于确认实际发生次数。
合成重复调用可改变未来 EMA 的证据，不能代替当前回放中的发生/因果证据。

`PoseInterp` 查询恰等首样本时间时返回首样本，修正原先可能落到未来队尾的边界；
before/after 窗口、单样本拒绝和其他插值政策保持。旧/新首戳数值发生预期变化，
不应伪称所有版本逐位相同。LIO ready-drain 区分“未 ready 未消费”和“已消费但
初始化/skip/少点无输出”，每个产出仍走原下游；目前仅新 cloud 唤醒，最后一帧
可能等待下一 cloud 或在 EOF 留存，须核对 `[LIO_DRAIN]` 实际 pending/last-end。
外层队列零 drop 不能证明内部无积压；没有硬实时延迟保证。旧 empty-IMU/stale-scan
及时间倒退 reset 边界不因该调度修复自动消失。

CI6 是本轮当前 x86 本机，已有本机实验；每个最终候选的完整输入、逐层质量、
频率、CPU/内存和正常退出仍须用对应冻结产物单独验收。ARM 板端/AKS 未验收，
本机 bag 与 SE2 软件闭环没有独立物理真值，也不证明现场动力学/SDK 或板端降频根因。

在线 LOC 队列把已完成 LIO 的 `scan_end` 与该帧点云一起传递；队列载荷的 epoch
不可重赋值。匹配、初始化接受结果和重定位指标使用同一个显式 epoch，避免后台
LOC 处理积压帧时读取后来扫描更新的全局平均周期。原始 cloud Header、点内容和
消息采样时间不变；显式 epoch 非有限或不大于零时拒绝该次 LOC 入口。保留的直接
调用旧接口仍遵守原先的 mean/点尾时间契约，不能把两种接口的时间期望混用。

异步队列退出标志采用原子读写，并在与等待端相同的 mutex 内设置唤醒条件后通知，
避免退出通知丢失。已取出的整批回调仍处理完毕；这不新增 EOF 自动排空承诺，
队列 skip/drop、内部 pending 和最终产出仍需分别检查。

`performance.deterministic_loc_tasks: true` 可选择按源序决定 LOC 任务，默认 `false`
保留旧分支。每个已产出的 LIO scan 连同不可变 scan_end、Produced ordinal 和关键帧
ordinal 入队一次，生产端不再以后台初始化完成的墙钟时刻决定选帧。消费者按自身
状态执行初始化或恢复；GOOD 状态下仅选中固定 cadence 的帧，其余元数据任务不
匹配、不重复发送旧结果给 PGO。初始化成功帧只处理一次。序号从一次 Init 的 1
开始；启用 skip 时 ordinal 能被 skip_num 整除才选中，关闭 skip 或 skip_num=1
每个符合条件的帧均可选。loc_on_kf=true 使用首次见到的非空新关键帧序号，false
使用 Produced 序号。恢复不会按墙钟重置相位；重新 Init 重置会话。

这是任务调度语义修复：避免旧模式在初始化完成前积压的额外帧随后执行正常配准；
不改变 matcher 点数、迭代、阈值或主 LIO 数学。它不保证历史 DR 插值队列、输入
丢帧或全链位姿都逐位相同。全 Produced 入队需要用实际 overflow、缺失序号和尾部
审计验收，不能把更深队列当作吞吐提升或强制排空。回退设 false 并重新启动。

该模式在工作线程回收后输出 `[LOC_TASKS]` 和 `[NDT_CALLS]` JSON，分别记录源序
任务分类/派发、有序 epoch 哈希、缺失/尾部，以及实际 NDT align 入口的初始化、
恢复、正常 tracking 和 rough/fine 计数。初始化 yaw 搜索的多次 align 不等于多帧
定位；队列回调、配准帧数、内部 align 次数和融合位姿发布频率须分别解释。摘要
缺失、重复、拒绝、overflow 或不同选中序列须如实失败，不能用 200 Hz 发布证明
NDT 计算达到 200 Hz。生命周期摘要不提供逐阶段调用时间，不替代火焰图。

## 原生 CPU 采样与启动边界

`LOCNAV_PROFILE=0` 是默认值，直接执行原工作负载，不创建 profiling 目录、不检查
采样依赖、不预载库。`LOCNAV_PROFILE=1` 显式启用所选 backend。
`resources` 记录 owned 进程树的 CPU/RSS/PSS，不加载 CPU 或 heap 库；核数按
CPU 秒 / 单调墙钟秒计算，不除以 CPU 数或亲和性集合大小。

CPU backend 使用 plain `libprofiler`。预载只能发生在最终动态原生 ELF 的进程中，
Python、env、taskset、ros2 和 launch 保持无 `LD_PRELOAD/CPUPROFILE`。
不要把预载变量设置到整棵启动树；原生目标之后再次 exec 其他镜像不在支持范围内。
heap / heap_sampled 仍按前文独立契约运行，不能据 CPU prefix 支持推广 launcher heap。

直接 ELF 可用原 `--output /absolute/new_directory`。ROS 或预算入口使用
`--native-output-root /absolute/new_root`，并至少指定一个
`--native-allow-executable /absolute/resolved/native_ELF`。每次 prefix 原子创建独立
`native-PREFIXPID-*` 子目录，respawn 保留各次证据。未知启动器/解释器、符号链接
解析后的启动器别名、错误架构或有冲突的预载环境在启动前拒绝。

定位预算入口的 `--profile-native-output-root` 只在全局开关开启且 CPU backend
被选中时建立原生 prefix。源码布局读取邻居 `run.py`；安装布局的 `run_budget`
必须与同版本邻居 `profile_workload` 一起交付。taskset 放在 prefix 外侧，先限制
亲和性，再加载 profiler。全局关闭时保持原预算执行路径。

频率优先级为显式 `--frequency`、`LOCNAV_PROFILE_FREQUENCY`、默认 **99 Hz**。
1000 Hz 用于单列诊断，会增加开销并改变线程竞争、初始化进度和队列积压，不能将
它的 CPU 或输出数量直接作为未采样性能验收。CPU 预算和定位质量/频率验收应在
关闭采样的同输入运行中完成；不通过更改采样频率来掩盖 exec 或退出问题。

每次采样必须保留真实 native PID/starttime、ELF SHA、实际 maps、采样库 SHA、
非空 profile、flush 及 owned 进程清理证据。SIGPROF、崩溃、空/截断 profile、
缺失实际映射、未知 native、TERM/KILL 升级或残留进程均保留为失败。

| SIGINT 场景 | 实际返回及证据 |
| --- | --- |
| 显式 native prefix 收到恰一次 SIGINT，native 自行返回 0，maps/profile/清理检查全部通过 | prefix 返回 0，summary 仍记录 interrupted、原 SIGINT 和 `verified_graceful_prefix_shutdown`。 |
| 上述 prefix 外层还有接收 SIGINT 的 resources/ROS 启动管理 | 内层验证通过不改变外层真实 130；分别保留内层 0 和外层 interrupted / child 结果。 |
| 普通直接 `--output` 或 resources 的兼容路径 | 保持原 SIGINT 130 语义。 |

不能把所有 130、负信号退出或 wrapper 返回 0 改写成原生成功。CI6 工具边界矩阵
已对 4 种启动布局 × 99/1000 Hz × 正常/SIGINT，加 2 项兼容路径完成 **18/18**
验证；它只证明采样边界、flush 和清理，不代替 bag、算法、CPU 或 ARM 验收。

火焰图合并所有线程的用户态 CPU 样本，不是逐线程时间线，也不包含内核栈或完整
等待墙钟。inclusive 百分比相互重叠。stripped 库可能把内部地址归到附近的导出
符号：例如本机 `std::error_code::default_error_condition`、RCLCPP `cancel` /
`get_c_vector_string` 和 TBB `cancel_group_execution` 的大段标签已发现地址越出
该导出函数大小。不能按这些名字认定错误处理/取消耗时；须结合真实 PC、原生 maps
和同构建 debug symbols 核实。只有模块地址覆盖完整，也不代表所有函数名可靠。

原生库、heap helper 和调试符号必须匹配运行平台、ABI 与实际二进制。当前 x86_64
私有 gperftools 库不能复制到 ARM 板；ARM 要独立提供对应架构库并执行工具与性能
验证。工具不会自动下载或安装依赖。

## 显式 DDS loopback 参数

定位与导航的 `run_budget` 均可显式接受 `--dds-udp-loopback-profile`，缺省关闭。
定位入口可另外接受 `--profile-native-output-root`；导航入口不接收这个定位专用
参数，导航 native profiling 仍由 `profile_workload` 与验证工具的既有 prefix
接口完成。DDS 开关和 `LOCNAV_PROFILE` 相互独立：关闭采样不会取消另外显式请求
的 DDS 参数，两个开关均未启用时沿用原预算执行路径。

两包 XML 源码相对路径均为 `config/performance/ci6_udp_loopback.xml`，XML 内容
绑定 SHA `78e6281cc011d0b164647b539e41074666e684bffca30aed45814d4221359a23`。
安装分别位于 `lib/lightning/config/performance/ci6_udp_loopback.xml` 和
`lib/robot_navigo/config/performance/ci6_udp_loopback.xml`。导航 CMake 显式安装
该单一文件；仅有 share 下的 XML 不满足预算入口的安装布局。

定位 DDS 开关作用于所选定位进程；导航 DDS 开关作用于预算入口 exec 的完整
Nav launch tree，包含 container 内组件和独立 smoother 进程。它们使用兼容
UDPv4 loopback participant transport，保留原 RMW 选择、endpoint QoS、argv、
CWD、PID 及既有亲和性；不宣称其他独立 participant 自动继承同一环境。入口只
设置声明的三项 transport 环境差异：移除 ROS_LOCALHOST_ONLY，设置
ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT 与固定 FASTRTPS_DEFAULT_PROFILES_FILE。
预算进程的 metadata 查询、XML 校验和 UDP socket 创建不是 native 服务健康证明。

未传 DDS 参数时不检查 XML/RMW、不导入 ROS metadata 依赖、不改变默认/resources
路径。显式传参只支持已核查的 Linux x86_64 / Jazzy、rmw_fastrtps_cpp 8.4.4 和
Fast DDS 2.14.6；ARM 与其他版本另行验证。实际 native container、projector、
smoother 的环境、owned UDP loopback sockets、participant SHM FD/maps 需要逐进程
见证；DataSharing 另列，不等同于 participant transport。该资产接入没有证明
TIMEOUT 的根因，也没有预写可靠性、CPU、相对精度、频率或正式 bag 验收结论。
