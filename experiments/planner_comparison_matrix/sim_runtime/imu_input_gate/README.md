仅analysis源码准备，默认ROOT_HOLD。没有启动ROS Node、mapper、UDP或替换任何source；canonical与原frozen文件不改。

本次闭包`666dfe6484757ea8a13dbf6731d10e075a71832bb32295d94e4af4f911425aa6`显示一次原始IMU回跳438744ns，全部header唯一，派生IMU继承相同回跳。上游producer/传输/bridge接收的根因仍未知。此门是显式Sim-only采样接纳策略，不能称为已修好发布源，也不更改旧失败分类。

`baseline/`保存现frozen adapter、converter与旧cal parent原字节。`matrix_sensor_contract.py`原字节保持；`adapter_with_gate.py`只改IMU订阅为raw=True入口与其审计/诊断，lidar、camera、TF/config以及接受后原imu()/clock代码保持。`sim_imu_adapter.py`在root独立release前只输出ROOT_HOLD；同release用O_EXCL/fsync marker防重复启动。它不负责停止旧60，不把两个布尔字段当实际替换证明；root仍需单独exact supervisor/旧60退出、新唯一publisher、24/61保留与完整raw recorder核验。

`imu_gate.py`按原CDR解出整数header ns，只有`stamp > last_accepted`且frame、37项numeric、原header age[-.05,.3]秒、callback receipt age[0,.3]秒、接纳间gap≤100ms均有效才接纳。所有门在commit accepted前检查；不排序、不补点、不插值、不重写header，不清空epoch自动恢复。NaN、变帧、过期/未来、gap/receipt异常标fault。非推进消息可被正常拒弃；拒弃只更新拒计数/事件，不更新accepted header/receipt、有效snapshot、observe('imu')或clock。源持续旧包/时钟重置会耗尽原freshness并HOLD，不能靠新callback receipt续租。

每个reject包含原CDR base64/SHA、原header/frame（可解析时）、last accepted、真实callback wall/steady receipt、原因/fatal。adapter发布`/simulation/sensor_adapter/imu_reject`并累计diagnostics；事件发布失败明确fault/抛出，不静默继续。没有重新序列化typed消息冒称original CDR。接受后仅调用原converter改变frame为imu_link，数值与采样时间保持。

`calibration_observer.py`是独立新Observer模块，不改旧校准包。只对rawIMU非推进记录reject并保留原latest/receipt；NaN/变帧/gap等仍进errors，GT/lidar/camera非推进仍原strictHOLD。reject日志I/O失败进errors。未来parent仍使用原c.check_live与每个正包150ms lease；拒包不能刷新lease中的有效receipt，原0.3s无有效新进展照旧HOLD。旧已审Observer/owner仍严格HOLD，不会因adapter更换自动获得新接纳契约。

`actual_replay_v2/result.json`为最终源码的闭包离线回读：raw2448→accepted2447/reject1，derived2451→accepted2450/reject1；接纳顺序严格单调，header/CDR不改，gap未放宽。2447个共同接纳raw经原converter所得完整CDR逐字节等于已录derived，37项值也完全相等。实际Observer.capture离线同样只拒1条，latest/receipt未推进，没有Node。离线receipt采用bag record time代理并明确标记；runtime在callback入口取真实receipt。

`test_gate.py`涵盖last_accepted语义[10,9,9.5,10,11]、原CDR审计、时钟重置/持续旧包不续freshness、过期/未来/receipt/gap在commit前拒、NaN/变帧、Observer有效snapshot保持和I/O故障、GT/lidar/camera原strict、adapter reject不调用clock/observe、非IMU方法/原converter不变、默认HOLD。

`future_semantic_case.HOLD.json`仅预留未来单次semantic lateral canary的scope/新Observer绑定；它不能由旧父包执行，未加lateral/reverse能力。独立backend、父程序/contract整合及实际source60替换需root另行审定。raw topic与原始MCAP完整保留；门不授权source/机器人动作。Codacy MCP工具不可用，未安装或声称通过。
