默认ROOT_HOLD的adapter60替换准备，未执行任何signal/Popen/ROS Node。此driver仅容器内精确生命周期控制；它没有ROS client、UDP、recorder、Nav或motion逻辑。

root独立批准的步骤：

1. 新IMU组件SOURCE_GO后，root填本目录另存的plan/replacement release与新adapter自身release。绑定四source SHA、config38a、原60完整identity、保留24/61完整identity、同container/UID1000/185/netns、fresh≤30s的外部newUE1172665/start9427026/7447证明。review凭据绑定SHA；root需实际读其GO结论，不能把“存在文件”当审批。
2. root执行一次替换driver。持有old60 kernel pidfd后复核完整身份；once marker fsync先于唯一SIGINT。只正常SIGINT原container60/host31984/start4460360/argv2182…；最多15秒，必须pidfd已exit且/proc Z或absent。失败没有新start、升级signal或自动rollback。
3. 24/61与host证明再核后，唯一Popen新`sim_imu_adapter.py --execute`。new-start marker在Popen前；inner source release另有once。捕获新PID/完整身份并留日志；启动失败不重试、不停止/重启其它actor。driver不会signal新source。24/61/UE始终无signal。
4. 正常返回仅`SOURCE_STARTED_PENDING_ROOT_READINESS`，不是READY/运动许可。root用既有新canary全raw recorder共录5–10秒，完成`readiness_requirements.json`的实际graph+CDR/header+TF/clock核验。单MCAP channel不能代替单publisher证明。观察失败保持HOLD，root另立下一exact步骤。

当前固定身份：24 host31888/start4460159；60 host31984/start4460360；61 host31985/start4460362；boot c3697337-f336-48f4-8127-4af9e461035d。完整argv/hash/UID/namespace/CWD均在plan。新PID必须重新冻结，不能把旧60授权套给新source。

静态TF baseline共3条edge、2个publisher：61负责base_link→livox_frame；adapter负责base_link→imu_link及front_camera_optical。不得误要求/tf_static只有1 publisher。无map→odom、odom→base_link或新dynamic TF；/clock仍恰1个adapter承担原IMU派生职能。

oldscript、原converter、config38a路径/hash、原argv/CWD退回参考全部保留，driver不修改它们。若root后来授权rollback，必须先对NEW完整identity正常停稳/退出，再用保留旧字节另启新PID，冻结新身份；本driver不自动执行rollback或复活旧PID。

命令形状（本轮未执行）：

```bash
python3 replace_adapter60.py                    # ROOT_HOLD，无signal/start
python3 replace_adapter60.py --execute --plan plan.approved.json --root-release replacement.approved.json --output /absolute/new_unique_attempt
```

所有批准文件由root另存，填当前实际server路径和新UUID后重新算SHA。不可覆盖模板/原frozen证据来假装执行过。异常即HOLD，禁止自动重跑；source24/61/UE、system config、mount、正式Nav21/footprint和canonical源码均不改。验证仅pure mocks，不启动standin或真实进程。Codacy MCP当前不可用，未声称通过。
