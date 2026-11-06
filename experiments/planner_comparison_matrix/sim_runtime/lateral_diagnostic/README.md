仅准备与审查；所有模板默认 `ROOT_HOLD`，本目录没有执行真实ROS、UDP或运动。

一次60秒父进程：e850原字节deadline helper只登记唯一recorder；录制使用`-a --include-hidden-topics`、MCAP与原CDR，摘要必须包含原GT、原LiDAR/IMU及前视camera共6个指定topic。helper ACK、recorder精确身份及MCAP初始magic齐备后才允许probe。

固定顺序仅两次：官方`walk --forward 0 --lateral +0.20/-0.20 --yaw 0 --duration 1`。官方文件SHA为`5a31cd800317f9d280a453424143312584332caf7f38da5144b0730e25284eb3`，字节未改。薄wrapper仅包住`socket.sendto`记录实际系统调用前后时间并拒绝其它轴/按钮；正常或SIGINT仍走官方`finally neutral/balance`。每个非零包都检查父进程存活与≤150ms小观测lease，lease保留原header时间，所有传感器仍须原age在[-.05,.3]秒、receipt≤.3秒。

停止门保持`linear3Dnorm<=.03m/s`、`angular3Dnorm<=.05rad/s`，位置/yaw稳定且原GT连续推进≥.3秒。两次之间额外3秒停稳，最后额外5秒原始数据尾段。`rclpy.init(SignalHandlerOptions.NO)`保留父进程的信号处理；失败先SIGINT自己的tool，再测停与尾段，最后只SIGINT自己的recorder，不升级signal、不signal process group或任何source/UE。

root审批的工程候选patch为x[-1.5,1.5]、y[-3,3]，原始5cm oracle全部7200格class1，未做body侵蚀。完整±(.74+.15)、cell guard .05、global lateral .8m触发、cross .1m、yaw .08rad和另1.0m停止余量所得保守包络在patch内。该停止余量是静态SIM工程假设，尚非实测plant保证。GTz≤.2、roll/pitch> .25rad、speed>1m/s、越界、过期、时序异常均停止本次诊断，不加速/重试。

真实运行前由root按实际服务器绝对路径另存case与release；填新UUID、唯一output、当前boot/netns/containerPIDns，绑定case SHA、package SHA、sensor config与坐标审计证据，显式批准唯一velocity owner和patch假设。不要更改本冻结源码。`host_input.HOLD.json`包含这次新UE1172665/start9427026与旧worker完整身份；root需核所有host artifacts后置reviewed=true。只读collector即时输出外部host证明，核UID/完整UE与container身份、共享netns、TCP/UDP7447归属、旧63/123/2689 Z或absent及UID1000下Nav/recorder/writer不在。该证明仅为瞬时host观察，容器没有host/proc，不声称持续监控；首次及每个新probe首包使用时证明≤30秒。

审查后的命令形状（本轮未执行）：

```bash
# host，只读证据；ROOT需先完成实际路径/哈希/identity freeze
python3 host_preflight.py --input host_input.approved.json --output host_preflight.fresh.json --collect
# 已有Jazzy容器、当前root批准的原ROS环境中；无mount/source恢复
python3 lateral_once.py --case case.approved.json --root-release root_release.approved.json --output /absolute/new_unique_run --execute
```

不加`--execute`会输出ROOT_HOLD，既不创建ROS上下文也不Popen/socket/signal。run marker在第一个Popen前持久化，每个probe还有独立planned/tool-once marker。不能复用release重跑。失败零动作/无可测横移同样保留完整bag并报告failed，不用短探针推成通用速度或停止保证。

速度采用原GT header差分，世界位移按原body yaw变换；stop delay标明相对首次neutral实际sendto区间，不能当模拟器执行时刻。`stdout.receipt.log`只是原工具输出，不能代替send时间。bag审计为footer/schema/count/SHA，未逐chunk CRC或逐CDR/image解码。

`test_lateral_once.py`只使用fake socket/sensor与本机harmless standin PID；不使用ROS、仿真或真实UDP。原e850 helper的18项既有过程证据复用，不重复其整套测试。Codacy MCP工具当前不可用，未安装或声称通过。未修改canonical导航源码/速度/footprint，未build、commit、worktree。
