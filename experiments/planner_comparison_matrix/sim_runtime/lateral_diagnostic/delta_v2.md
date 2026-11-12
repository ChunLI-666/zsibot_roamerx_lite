v2仅修独立审查阻断，v1 snapshot/tar/stage证据保留；没有真实执行。
- host瞬时preflight明确拒绝matrix_mc_udp_test.py、official_tool_timed.py、lateral_once.py、sim_udp_relay_frozen.py，proc fixture验证精确basename。
- Popen后立即retain pidfd并读stat-only birth3(pid/start/boot)，再做argv admission与watch。exec空argv不丢birth；early guard失败仅可向相同birth3的retained ownfd发SIGINT。完整argv已读且不匹配、或已admit后无法复核完整argv仍拒信号。
- signal失败仍在finally尝试bounded wait；spawn_cleanup JSON记录真实signal/error/exit，不虚报退出。普通tool/recorder关闭也不再因interrupt异常跳过wait。
- metrics stop窗口与live相同，原GT gap>100ms reset，非推进失败；阈值、传感器门、动作范围均未变。
15项harmless测试PASS，包括pid77/fd999反例、exec连续空argv、错birth/argv拒信号但wait、admitted child不放宽、host proc名单、gapped stop metrics。原helper/官方工具/sensor config字节不变。
