# 地图覆盖路线与覆盖率工具

`scripts/map_coverage.py` 是不依赖导航运行时的离线工具，包含两个模式：

```bash
python3 scripts/map_coverage.py generate \
  --map-yaml /path/to/map.yaml --start-x 1.0 --start-y 2.0 \
  --safety-radius 0.35 --row-spacing 1.0 --waypoint-spacing 0.8 \
  --coverage-radius 0.75 --output coverage_route.json

python3 scripts/map_coverage.py evaluate \
  --map-yaml /path/to/map.yaml --trajectory-bag /path/to/bag \
  --topic /odom/current_pose --start-x 1.0 --start-y 2.0 \
  --safety-radius 0.35 --coverage-radius 0.5 \
  --output coverage_metadata.json
```

## 定义

- **有效区域**：PGM 中的 free cell，先按 `safety-radius` 做圆形腐蚀，再取包含起点的 4 连通域。起点不在该域内时直接失败，避免生成机器人无法安全通过的路线。
- **未知区**：PGM 阈值之间的像素，永远不是有效区域，也不会被覆盖率计入。
- **覆盖半径**：以规划路线或 Odometry 轨迹栅格化结果为中心的圆形半径；只与有效区域求交。它是评估参数，不是机器人碰撞半径。
- **未覆盖区域**：有效区域减去覆盖半径膨胀后的轨迹区域。metadata 中给出未覆盖 cell 数、比例和连通区域 bbox，PNG 用橙色标出。
- metadata 同时给出 `valid/covered/uncovered_area_m2` 以及 unknown、occupied cell 数，便于不同分辨率地图之间比较。
- **连接段**：蛇形扫描行之间使用 4 邻域 A* 在同一个安全自由域中连接，并对最终每一段做采样检查；不会穿过 occupied、unknown 或腐蚀后的禁行区域。

`evaluate` 支持 ROS2 `nav_msgs/msg/Odometry` bag；读取 bag 需要已 source 的 ROS2 环境以及 `rosbag2_py`、`rclpy`、`rosidl_runtime_py`。为便于单测和无 ROS 分析，也支持包含 `x,y` 列的 CSV。

运行时缺少 `numpy` 或 `Pillow` 会明确报错。形态学和连通域计算不依赖 SciPy，避免 ROS 主机上的 NumPy/SciPy ABI 不一致。工具只评估 `Odometry.pose.pose.position.x/y`，不会假设或偷偷转换 frame；使用前必须保证轨迹和 map.yaml 处于同一坐标系。
