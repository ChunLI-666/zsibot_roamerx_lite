# 私有仿真工作流

固定入口是 `workflow.py`。driver、worker 和 ledger 来自已使用的 v13 工作流；源目录和原失败证据保留于 `analysis_outputs/planner_comparison_20261105/space_agent_recovery_20261110`。此入口管理阶段、进程和证据，任务索引、UUID、停止证明、恢复和预算仍由 canonical `goal_pilot_source/mission_resume.py` 与 MissionStore 决定。

```text
python3 workflow.py prepare --candidate candidate.json --source source.json \
  --map-identity map_identity.json --route route.json --policy policy.json \
  --template execution_template.json --run-id CASE --run-dir /absolute/runs/CASE
python3 workflow.py dry-run --run-dir /absolute/runs/CASE
python3 workflow.py run --run-dir /absolute/runs/CASE --candidate candidate.json --execute --follow
python3 workflow.py status --run-dir /absolute/runs/CASE
python3 workflow.py reconcile --run-dir /absolute/runs/CASE --candidate candidate.json --client-stage mission --execute --follow
python3 workflow.py close --run-dir /absolute/runs/CASE --candidate candidate.json --execute --follow
```

`prepare` 不启动业务。相同输入重入复用同一 run、ledger 和 request id；不同输入拒绝覆写。执行默认 dry-run，显式 `--execute` 才进入已授权的仿真。新候选或 source 使用新的输入文件；运行中不能换绑旧 run。所有路径为当前执行机的绝对路径，跨机需要字节一致的源码/安装产物及原收据，安装前缀变更使用 driver 原有 derived receipt 校验。

Candidate schema 为 `matrix.simulation.candidate.v1`：`status=ACCEPTED`、`variant=official|segmented`、完整 `commit`、`client_request_idempotency_tested=true`、非空 `source_files` / `build_receipts` / `code_reviews` / `software_tests`。引用为 `{path,sha256}`，source_files 另带唯一 repo 相对 `name`。代码审查 JSON schema 为 `matrix.simulation.code_review.v1`、`decision=APPROVED`，软件测试为 `matrix.simulation.software_tests.v1`、`status=PASS`；二者必须有 `candidate_binding_sha256`，由 `workflow_inputs.candidate_binding` 对 variant/commit、逻辑源码名与 hash、parent build receipt hashes 计算。旧版的任意 PASS/APPROVED 文件不能用于新的源码。build receipt 还由 driver 核实提交、源码表交叉一致和实际安装产物。

`entrypoints` 指定同一候选目录的 `mission_client`，及五个 workflow 脚本名到实际绝对路径的映射。实际 workflow、整个客户端目录内的 Python 依赖、stage Python 入口/文件都必须在 accepted source 表中，client prefix 只支持直接 Python 调用同一树中的 `mission_resume.py`。`scope` 固定 `environment=matrix_private_simulation`、`ros_domain_id=185`，并绑定 `map_identity_sha256`、`policy_sha256`、`original_route={path,sha256}`。这是软件身份收据；执行权限来自用户授权及运行环境。

Source schema 为 `matrix.simulation.source.v1`：

- `source_id`、`ros_domain_id=185`、`boot_id`、`pid_namespace`、`network_namespace`；
- `map_identity`、`config`、`localization_release` 的文件引用；
- `native.binary` / `native.library` / `native.runtime_config` 文件引用，以及 `native.process` 实际进程身份；
- `roles.sensor|localization|floor|command` 各有 `plan` / `started` 文件引用及绝对 `result_path`。started 沿原 owner receipt，包含 `plan_sha256`、`deadline_boottime_sec`、`owner` / `child`，command 还包含 `relay`；
- 可选 `retained_processes`。进程身份为 PID、birth ticks、原 argv hash、boot、PID namespace；若旧 actor receipt 没有 net namespace 字段，仍核实际进程属于当前 source 的 network namespace。

既有 sensor/command Python actor plan 没有 `executable_sha256` 时，原 plan 不补写；source.roles 对应角色提供 `interpreter={path,sha256}`，核实际 Python /proc/exe，原计划直接脚本与 relay 脚本须被 plan.inputs pin，实际 argv 仍逐项一致。Native/floor 原生可执行继续要求原 plan 的 executable hash。三种 build receipt（navigation/recovery/waypoint）均不可缺少或重复，source/artifact 表必须非空。

Localization release schema 为 `matrix.simulation.localization_release.v1`，绑定 `source_id`、map/config/native 的 SHA256，另有 `runtime_config_sha256`、`localization_plan_sha256`、`localization_started_sha256`；Native 必须是该 localization actor 的原 child。`GT_used=false`、`fresh_scanfit_pass=true`、`ndt_confirm_2_of_2=true`，并引用实际 `stationary_capture`、`stationary_scanfit`、`ndt_confirmation` 原始证据。checker 解析现有 `terra.source11.stationary_capture.v1` 和 `terra.source.stationary_native_scanfit.v1`，沿既有至少 3 pair、fraction≥.9、无 rejected、status=[1]/valid=[true] 规则，核原 capture hash、样本对应及本代 actor 启动后的接收时刻；不重跑拟合。NDT snapshot 的实际 2/2 与 ACTIVE 行必须来自当前 actor.log。

Source admission 还检查实际 owner/child/relay、未关闭结果、有限 lease、运行中 Native binary/library/config 及唯一仿真命令链。每次真正 start/resume 准备还会对原 source hash、实际进程与当前任务剩余预算加 300 秒关闭余量重新核验，避免复用旧 SUCCEEDED 的 source stage；reconcile/cancel/close 不受运动 lease 门阻断。这些不能替代后续正常 launcher/client 的原样本健康检查。SceneWorld 命令接收/应用时刻没有回执时记录 unknown。

Execution template 沿 `terra.workflow.execution_plan.v1` 的 stages 结构，覆盖 environment/map/localization/recording/runtime/mission/closure/mcap/video/video_qa/acceptance。`prepare` 填入候选、构建收据、原路线/政策，插入固定 source admission，并固定所有输入 hash。模板可用 `{candidate}`、`{source}`、`{map_identity}`、`{route}`、`{policy}`、`{run_id}`、`{candidate_commit}`、`{variant}`、`{workflow_dir}`；`{run_dir}` 继续由 driver 在各次 invocation 解析。所有 Python 入口与依赖仍须在 stage.files 中固定。

Closure stage 只使用本 run 的原 actor/录包身份，显式 `exact_owned_cleanup=true`。`close` 可在原 mission HELD 后运行已绑定的关闭及录制 QA 阶段，保留原任务错误/未知状态；关闭成功不升级任务结论。UUID/所有权不明仍需要 exact-UUID reconcile 或明确封存。

`prepare-cycle` 接收原完整 route、policy、map identity 和有限的 Native map anchor JSON（`frame=map,x,y,yaw`），生成同一 40 条有向边的循环旋转与独立等价收据。它不导入旧 prefix；实际派发仍使用正常客户端检查当前原 pose、8.5m 首段窗口和完整机身碰撞。

同一失败指纹连续两次应停止该路径并回报最小证据；第三次无新证据则封存，不能反复重跑全图或重建任务绕过失败预算。测试运行、采样标定、录制、前视相机、手机 MP4 和实际 QoS/socket/drop 的证据均由执行代理提供。

当前固定工作流支持同一 source、同一版本的原任务中断恢复。source lease 到期后关闭并准备新评测使用独立 run，不把旧通过点或预算迁移过去。历史 `mission_continuation` 有专用 A15/F016 约束，本入口没有宣称通用跨 source 的同任务 continuation 能力。
