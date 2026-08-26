# robot_navigo

The `robot_navigo` package is an example bringup system for Nav2 applications.

This is a very flexible example for nav2 bringup that can be modified for different maps/robots/hardware/worlds/etc. It is our expectation for an application specific robot system that you're mirroring `robot_navigo` package and modifying it for your specific maps/robots/bringup needs. This is an applied and working demonstration for the default system bringup with many options that can be easily modified.

Usual robot stacks will have a `<robot_name>_nav` package with config/bringup files and this is that for the general case to base a specific robot system off of.

Dynamically composed bringup (based on  [ROS2 Composition](https://docs.ros.org/en/galactic/Tutorials/Composition.html)) is optional for users. It can be used to compose all Nav2 nodes in a single process instead of launching these nodes separately, which is useful for embedded systems users that need to make optimizations due to harsh resource constraints. Dynamically composed bringup is used by default, but can be disabled by using the launch argument `use_composition:=False`.

* Some discussions about performance improvement of composed bringup could be found here: https://discourse.ros.org/t/nav2-composition/22175.

To use, please see the Nav2 [Getting Started Page](https://navigation.ros.org/getting_started/index.html) on our documentation website. Additional [tutorials will help you](https://navigation.ros.org/tutorials/index.html) go from an initial setup in simulation to testing on a hardware robot, using SLAM, and more.

Note:
* gazebo should be started with both libgazebo_ros_init.so and libgazebo_ros_factory.so to work correctly.
* spawn_entity node could not remap /tf and /tf_static to tf and tf_static in the launch file yet, used only for multi-robot situations. Instead it should be done as remapping argument <remapping>/tf:=tf</remapping>  <remapping>/tf_static:=tf_static</remapping> under ros2 tag in each plugin which publishs transforms in the SDF file. It is essential to differentiate the tf's of the different robot.

## Launch

```shell
ros2 launch robot_navigo navigation_bringup.launch.py platform:="$PLATFORM" map:="$MAP" tf_type:="$TF_TYPE" mc_controller_type:="$MC_CONTROLLER_TYPE" communication_type:="$COMMUNICATION_TYPE"
```

## Matrix + Lightning 闭环回归

Matrix 仿真回归使用 `matrix_lightning_closed_loop.launch.py`。导航位姿和 TF 只能来自 Lightning；MuJoCo `/odom/mujoco_odom` 只由只读评分器和 rosbag 消费。

完整架构、环境约束、验收指标和 2026-08-27 基线结果见
[`doc/matrix_lightning_closed_loop.md`](doc/matrix_lightning_closed_loop.md)。

```bash
ros2 run robot_navigo matrix_closed_loop_run.sh \
  --matrix-root /home/charles/project/matrix_closed_loop_ws/src/zsibot/matrix \
  --workspace /home/charles/project/matrix_closed_loop_ws \
  --map /home/charles/project/locnav_ws/data/matrix/scene_terrain_wh/20260315_lightning/map.yaml \
  --relative-x 1.0 --relative-y 0.0 --relative-yaw 0.0
```

脚本依次检查传感器、定位发布权、TF、LaserScan、Nav2 lifecycle 和控制链，并输出 bag、日志和机器可读 `result.json`。不要使用 Matrix 的 `run_sim_with_nav.sh`、`pub_tf` 或旧 `sim_tf_bridge.py`，这些入口会把真值位姿注入导航闭环。
