# Matrix 多场景建图批测编排

入口是 `scripts/matrix_mapping_batch.py`。它消费 YAML manifest，并在一个非阻塞
`flock` 内严格按 manifest 场景顺序执行；已有批次持锁时直接失败，不会与其并发。
每个场景使用 `<output>/<scene>/`，其两个状态文件分别位于
`collect_mapping_bag/phase_status.json` 和
`offline_lightning_mapping/phase_status.json`。每次状态更新均以临时文件加 `os.replace`
原子提交。

```bash
python3 scripts/matrix_mapping_batch.py --suite mapping_suite.yaml \
  --output /mnt/data/regression/matrix_mapping/run_20260828
```

可选 `--resume` 只复用输入签名相同且产物仍存在的 `PASS` 阶段；缺证据的
`UNAVAILABLE` 和未执行命令的 `SKIPPED` 不会被缓存，证据补齐后可继续运行。
`--dry-run` 不执行命令，写入可审计的 `SKIPPED` 状态和展开后的 argv。

## Manifest 契约

每个可执行场景必须具有唯一 `name`/`scene_id`、单独的 `ros_domain_id`、`mapping_bag`、
独立的 `localization_bag`、`coverage_metadata`，以及下面两个阶段：

```yaml
schema: robot_navigo.matrix_mapping_suite
schema_version: 1
scenes:
  - name: warehouse
    scene_id: 1
    available: true
    enabled: true
    ros_domain_id: 43
    mapping_bag: /data/warehouse_mapping_bag
    localization_bag: /data/warehouse_later_localization_bag
    mapping_dataset_id: warehouse_mapping_20260828_001
    localization_dataset_id: warehouse_eval_20260828_002
    coverage_metadata: /data/warehouse_coverage.json
    collect_mapping_bag:
      command: [./record_mapping_bag.sh]
      required_outputs: [/data/warehouse_mapping_bag]
    offline_lightning_mapping:
      command: [./run_lightning_mapping_offline.sh]
      required_evidence: [/data/warehouse_mapping_bag]
      required_outputs: [/data/warehouse_map]
```

`mapping_bag` 与 `localization_bag`（也就是后续 localization/eval bag）必须具有不同的
规范化路径和不同的 dataset ID；相同即两个阶段均为 `UNAVAILABLE`。采集成功后仍须读取
`map_coverage.py evaluate` 生成的 coverage metadata。门禁读取
`metadata.coverage_pct` 和 `metadata.uncovered_cells`，默认要求 100% 且无未覆盖栅格；
同时要求地图指纹存在，并且 `metadata.trajectory_source` 指向本场景的 mapping bag。
仅有规划路线的 `planned_coverage_pct` 不能作为实际覆盖通过证据。
缺少静态证据、采集产物、coverage metadata 或场景不可用，一律写 `UNAVAILABLE`，绝不写
`PASS`。

GT 只允许存在于采集时的探索路径生成和 coverage 评估证据。正式
`offline_lightning_mapping.command` 必须显式消费 mapping bag，且不得包含
`ground_truth`、`--gt` 或 `/odom/mujoco_odom`；GT 不能进入正式定位或正式地图生成。

每条命令的完整 argv、工作目录和该场景的 `ROS_DOMAIN_ID` 都写入 phase 的 `command.log`
及状态记录。子命令以新的 session/进程组启动；超时时仅通过该子命令 PID 的 `killpg`
清理，不使用 `pkill`、`killall` 或全局进程匹配。

编排器只能静态拒绝 manifest argv 中出现的全局清理命令；如果 manifest 调用的外部脚本
内部仍使用全局 `pkill`，必须先修复该外部脚本后才能纳入正式 suite。
