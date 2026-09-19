# 定位失效停车与独立 watchdog（2026-09-10）

P2 的第一项生产增量：当 `/lightning/loc_status` 从 NORMAL 变为 DEGRADED/LOST/UNKNOWN 或非法值时，安全门立即发布零速度。此前只在下条速度命令或超时检查时拦截，存在延迟停车窗口。watchdog 还会持续重发失效停车。

状态和命令的新鲜度依据单调时钟的接收时间，timer 同样使用 STEADY_TIME；即使启用 `use_sim_time` 并暂停 `/clock`，定位 200 ms、命令 300 ms 的超时仍可触发。状态/命令输入队列深度改为 1，减少旧输入排队；这不等于验证消息源时间。所有六个 Twist 分量必须有限，NaN/Inf 输入归零，诊断码 6 表示无效命令。急停优先，恢复 NORMAL 或解除急停都不会主动重放缓存速度；必须有后续新命令。

保留现有话题、0/1/2/3 状态与 4/5 诊断码，正常且新鲜的合法 Twist 不修改。该安全门没有提供重定位 epoch 与新路径的跨进程握手，不能判断恢复后的命令是否来自新规划；此项仍属于 P2 后续工作。终端实际停车还依赖传输、执行器 watchdog 和机械响应，不能把发布零速度直接等同于实机已停稳。

验证：9 项测试通过，涵盖失效通知立即停车、持续停车、异常状态、NaN/Inf 各轴、急停优先、恢复不重放、两个独立超时。真实 rclpy executor 在 ROS 时间固定为 0 时触发 steady watchdog；另外通过 DDS 话题实际发送 NORMAL/速度/DEGRADED，观察生产 publisher 输出及暂停时钟后的超时零速。测试仅在 localhost ROS domain 182，无执行器 bridge。既有导航契约测试通过。

复现（从 workspace 根目录）：

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=182 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 src/zsibot/zsibot_roamerx_lite/src/navigation/src/robot_navigo/test/test_nav_safety_gate.py -v
```

日志：`analysis_outputs/zsl1_goal_20260910/nav_safety_gate_tests.log`。CMake 注册了相同隔离环境的 `test_nav_safety_gate`。Python 节点通过源码直接运行验证；本增量不涉及 C++ ABI。Codacy 工具当前不可调用，未执行。
