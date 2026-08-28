# Matrix 闭环批测编排

批测入口是 `scripts/matrix_closed_loop_batch.py`，默认 suite 为
`regression/matrix_closed_loop_suite.yaml`。它在一个 `flock` 锁内串行执行
场景、路线和模式组合；每个组合写入独立的
`<run>/<scene>/<case>/<mode>/` 目录。

```bash
python3 scripts/matrix_closed_loop_batch.py \
  --suite regression/matrix_closed_loop_suite.yaml \
  --output /mnt/data/regression/matrix_closed_loop/run_YYYYMMDD_HHMMSS
```

## 两种模式

- `gt_baseline`：启动测试专用 `matrix_ground_truth_baseline.py`，把 Matrix 的
  `/odom/mujoco_odom` 变成测试导航使用的 TF、`/odom/current_pose` 和正常状态。
  该模式只回答规划、costmap、控制器和执行链是否工作。
- `lightning_formal`：使用现有 `matrix_closed_loop_run.sh` 启动 Lightning，
  `/odom/mujoco_odom` 只用于录制和离线评分。Lightning 失败不会自动切换成 GT。

两种模式在 suite 中是显式的独立命令。runner 启动的进程由自身 PID/PGID
清理；编排器不执行 `pkill`，也不会跨 case 杀进程。
启动新仿真前若检测到已有 Matrix 进程，runner 会直接失败，不会清理未知会话。
Matrix 上游 `run_sim.sh` 仍含历史全局清理逻辑，因此批测必须使用 `flock` 串行运行；
该上游脚本的进程所有权改造仍是后续工程任务。

## 路线和指标边界

`central_smoke` 只验证进程和控制链路，不能作为地图覆盖通过证据。
`full_map_loop` 路线必须是 `map` frame、至少 8 个 waypoint、返回起点，并携带
`map_coverage.py` 生成的地图哈希、至少 99% 规划覆盖率和逐段安全检查元数据。
因此中心环线不会被误报为全地图覆盖。

报告只读取现有 `matrix_closed_loop_e2e.py` 的 `result.json`，包括 action、waypoint、
定位误差、实际路径和已有 topic 统计。当前 Matrix 没有 ROS contact topic，所有
真实碰撞检测统一标为 `UNAVAILABLE`；路径穿越或 costmap 代理不能写成碰撞结论。

每个 case 的 `case_status.json` 先原子写入 `RUNNING`，结束时再原子替换为最终
状态。批次输出包括 `summary.json`、`MATRIX_REPORT.html` 和 `batch_status.json`。

## 当前覆盖范围

当前只有 Warehouse 有冻结地图和中心环线 smoke 路线，因此执行 `--scene warehouse`。House、
Office、IROS Flat、Yard 在 suite 中保留为 `UNAVAILABLE`，原因写入状态和 HTML，
在取得版本化建图结果及全覆盖路线前不伪造通过结果。
