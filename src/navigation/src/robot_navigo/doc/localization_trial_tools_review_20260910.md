# 定位 dropout 工具独立审查与验证（2026-09-10）

本轮审查修复了 `run_localization_online_trial.py` 的真实成功误判：播放器退出后观察 source loss 的 4.5 秒内，定位节点若自行退出，原循环会直接结束，而最终仅检查播放器退出码。因此节点即使崩溃或提前以 0 退出，也可能被误判成功。

现在观察窗口内和清理前都检查定位节点是否仍存活；任何主动清理之前的退出均失败。`trial.json` 保留 `passed/failure`、清理前退出码、清理信号与自建进程组残留状态，异常路径也保存日志与事件；清理阶段需要 SIGTERM/SIGKILL 强停或发生崩溃同样判失败。`passed` 明确仅表示运行器执行成功，不是定位行为或精度得分；恢复行为由独立生命周期分析器验收。

清理仅针对 `start_new_session=True` 创建的进程组，即使组长先退出，也会继续清理其所属子进程，绝不使用全局 kill。超时 NaN/inf 被拒绝。输入 config、bag 全部文件和 tiled map 全部文件记录 SHA256，启动前固定 `run_loc_online` 及 snapshot 目录全部 `*.so*` 哈希，保留原有两个哈希键。这只覆盖 snapshot 内文件，外部系统/工作区动态依赖仍需另行 ldd 核验。本次旧 snapshot 仅有 run_loc* 与 lightning.libs.so，miao.core/Utils 当时由 install 加载；新数值 snapshot 的完整项目动态库闭包另行验收。输出目录不能位于源 bag、map 或 binary 目录内。

`prepare_localization_dropout_bag.py` 的 CDR 原样复制与接收时间保存实现通过审查；新增拒绝在源 bag 内创建输出，并记录实际选中数据时长与明确的传感器 topic。它以接收时间定义删除区间，区间左闭右开，只移除 LiDAR，既不改 header/点时间，也不写源文件。只读源与内容一致性由真实 MCAP 回归验证。

## 最终证据

- 10/10 工具测试通过，无 skip，约 3.2 秒。包含真实 MCAP 写出/读回、drop 边界、payload 与时间戳逐条相等、源文件哈希不变；包含真实独立 session 的清理隔离。
- 原缺陷有端到端回归：隔离 domain 187 中只发布状态的测试节点在播放器结束后以 0 退出，完整运行器返回失败，`trial.json` 明确记录 post-playback failure 和两个退出码 0。测试不启动生产定位或执行器。
- 冻结生产定位二进制重新播放来自 `2025-11-21/rosbag2_2025_11_21-08_50_59` 的原 45 秒 dropout bag（不是 0829 包），domain 186、LOCALHOST，仅定位与传感器播放器。运行 52.27 秒，记录 1,170 条状态/pose-valid 事件。清理前 player=0，node=null（仍存活）；node 接收运行器 SIGINT 后返回 0，两个进程组均无残留。
- [reviewed trial](/home/charles/project/colcon_ws/analysis_outputs/zsl1_goal_20260910/online_dropout_trial_reviewed/trial.json) 与原试验分目录保存。原始日志未覆盖，源 config/bag/map 共 11 个记录文件经运行后再次 SHA256 核对均一致，见 [只读检查](/home/charles/project/colcon_ws/analysis_outputs/zsl1_goal_20260910/online_dropout_trial_reviewed/source_unchanged_check.json)。
- 对新 `node.log` 运行 `lightning-lm/scripts/analyze_relocalization_lifecycle.py --require-recovery` 成功，得到 2 次 commit、2 次 lost、1 条 recovery chain：[行为验收](/home/charles/project/colcon_ws/analysis_outputs/zsl1_goal_20260910/online_dropout_trial_reviewed/lifecycle_audit.json)。这不证明位姿精度、真机性能或导航端到端成功。

最终追加的强停/清理崩溃失败判据也已用于重新审核同一 trial 原始退出证据，结果仍通过，见 [最终判据复核](/home/charles/project/colcon_ws/analysis_outputs/zsl1_goal_20260910/online_dropout_trial_reviewed/final_outcome_reassessment.json)。该复核记录最终脚本哈希，不修改原始 trial。

CMake 安装两个工具并注册 `test_localization_trial_tools`，package.xml 声明 ROS 消息、bag Python/CLI 与 MCAP 存储依赖。最终 robot_navigo 构建通过；CTest 该套件 10/10 通过（3.17 s，无 skip），见 [构建日志](/home/charles/project/colcon_ws/analysis_outputs/zsl1_goal_20260910/localization_trial_tools_build.log) 与 [CTest 日志](/home/charles/project/colcon_ws/analysis_outputs/zsl1_goal_20260910/localization_trial_tools_ctest.log)。CTest 使用 LOCALHOST/domain 187、串行执行；其中真实 MCAP/ROS 子进程回归需要 ROS 环境，缺少依赖时会显式 skip，不能将这种运行报告为完整证据。

## 独立复现

从 workspace 根目录运行，先 source Jazzy 与 workspace。下例每次建立新的临时父目录，其 `bag` 和 `trial` 子目录尚不存在，因此不会覆盖已冻结证据；执行前应确保 domain 186 没有其他试验占用。这一组固定使用本轮工具验证的旧 snapshot，新数值契约 snapshot 的结果另行记录。

```bash
cd /home/charles/project/colcon_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
trial_review_root=$(mktemp -d /tmp/zsl1-dropout-review.XXXXXX)
python3 src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/prepare_localization_dropout_bag.py \
  --input /home/charles/datasets/rock_dog/2025-11-21/rosbag2_2025_11_21-08_50_59 \
  --output "$trial_review_root/bag" --duration 45 --drop-start 8 --drop-duration 4.5
python3 src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/run_localization_online_trial.py \
  --binary-dir analysis_outputs/feature_relocalization_20260910/frozen_binary \
  --config analysis_outputs/zsl1_goal_20260910/online_dropout_trial/config.yaml \
  --map data/hz_office_2f_desc_0829 --bag "$trial_review_root/bag" \
  --output "$trial_review_root/trial" --domain 186 --timeout 90
python3 src/lightning-lm/scripts/analyze_relocalization_lifecycle.py \
  "$trial_review_root/trial/node.log" \
  --output "$trial_review_root/trial/lifecycle_audit.json" --require-recovery
```

代码保留在 `feature/forward-aligned-navigation`，未改生产导航、未合并。Codacy 工具不可用。
