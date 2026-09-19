# 0829 三个后退片段的现场 costmap 重建

为补足之前空地图下的路径几何实验，从最后一个 0829 bag 重建三个起始时刻之前实际录制的局部 costmap。选择最近完整 `nav_msgs/OccupancyGrid`，按接收时间顺序应用之后的 `map_msgs/OccupancyGridUpdate`；仅使用目标时刻之前的数据。MCAP 缺少索引时可能返回存储顺序，工具显式对所选消息排序。

输入：`/home/charles/datasets/rock_dog/20260829/rosbag2_2026_08_29-04_41_05`。

| 片段 | 起始接收时间（ns） | 图尺寸 / frame | 已应用增量 | 最新接收时间距起点 |
|---|---:|---|---:|---:|
| reverse_10s | 1788003737633904500 | 160×160 / odom | 9 | 0.295 s |
| reverse_14s | 1788003822933751800 | 160×160 / odom | 12 | 0.096 s |
| reverse_20s | 1788004164634043500 | 160×160 / odom | 11 | 0.095 s |

三图各覆盖约 8×8 m，保留各自 rolling origin；框架/矩形边界校验无拒绝。发布器 `navigo_costmap_2d/src/costmap_2d_publisher.cpp` 将增量消息的 header.stamp 显式设置为 `rclcpp::Time()`，所以这些零时间戳不能被当作过期观测，也不能声称存在真实传感器新鲜度证据；工具使用 bag 接收顺序，单独记录 unstamped_updates。非零且回退的源时间仍拒绝。

OccupancyGrid 是成本的有损编码。按本仓库 publisher 的翻译表反解，0→0、99→253、100→254、-1→255，中间每个 bucket 取其最大原始成本。这样保守保留膨胀成本，且不会把原本 1–252 的膨胀格变成 lethal。7 项测试覆盖全部 256 个成本、未知格、增量索引、无效/过期增量、滚动窗口替换、零时间戳和快照不可变性。

产物：`analysis_outputs/zsl1_goal_20260910/recorded_costmaps/` 中包含三份 `.costmap`、JSON/YAML 元数据和总 manifest，记录所选输入消息流哈希、脚本哈希和输出哈希。控制器验证可读取各 JSON 的 `map` 字段。

这些是冻结的历史局部环境，膨胀和自身清空仍基于历史 footprint；没有模拟新足迹的感知更新，也没有重新执行整个导航任务。冻结局部路径终点的到达只能计为该场景的闭环结果，超出窗口须拒绝；模型 standing footprint 更不等于实机摆腿包络。

复现（workspace 根目录）：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/scripts/reconstruct_recorded_costmap.py \
  --bag /home/charles/datasets/rock_dog/20260829/rosbag2_2026_08_29-04_41_05 \
  --output analysis_outputs/zsl1_goal_20260910/recorded_costmaps \
  --snapshot reverse_10s:1788003737633904500 \
  --snapshot reverse_14s:1788003822933751800 \
  --snapshot reverse_20s:1788004164634043500
```
