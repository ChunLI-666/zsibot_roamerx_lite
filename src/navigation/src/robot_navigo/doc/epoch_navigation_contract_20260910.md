# 定位—规划—控制身份契约（2026-09-10，P2 实验）

本轮基线：导航 `2b8effd`、定位 `72532e8`。所有实现继续在原 canonical checkout 的 feature 分支，未经用户 review 不合并、不启动实机运动。本文先定义可验证的契约；测试与最终结果另附，不将协议设计本身当作已实现或已验收。

## 需要修复的实际缺口

原 gate 只根据 UInt8 NORMAL 与收包超时放行。真正路径来自 ComputePath 的 action result，经 SmoothPath、BT 黑板、FollowPath action 到 controller；`/plan` 只用于观测。controller 丢掉路径/任务来源后发布裸 Twist，平滑器周期重发会掩盖源控制命令已经停止。因此“收到新 /plan”“收到恢复后的新 Twist”都不能证明导航使用了新定位结果。

## 端到端身份

共享接口包为 `navigo_epoch_msgs`，与实现包分离，定位与导航共同依赖。ROS2 消息原有时间戳不被重写。

| 对象 | 身份及语义 |
|---|---|
| LocalizationIdentity | 随进程重新生成的 session、地图加载实例、epoch、commit 计数；计数本身不能跨重启唯一 |
| LocalizationEpoch | 单调心跳序号、健康/融合/实际输出就绪标记；固定 commit 源时间；真实发布的 map→导航 base 位姿及原始源时间 |
| NavigationToken | 定位身份 + 随 BT 上下文生成的 session + 任务版本 + 规划请求序号 + gate session |
| EpochPath | 上述不可变 token + 原请求的权威起点/定位心跳编号 + 实际规划/平滑返回路径 |
| NavigationIntent | BT 正在执行的任务/路径授权心跳；取消、失效、终态先撤销 |
| NavExecutionState | controller 确认实际安装该路径后发布的执行状态与 controller session |
| EpochCommand | 源 controller 命令序号、源单调时间、有效期与同一 token；平滑器仅另加自己的 relay session/序号 |

规划和路径平滑保留标准 Nav2 action。BT 在请求发送时固定上下文，使用原 ROS action goal UUID 接收结果；规划显式 `use_start=true`，起点取带身份的实际定位输出。结果必须属于原请求且原身份仍有效，不能在结果返回时补上“当前”身份。控制端使用 `FollowPathEpoch` typed action，禁止把信息塞进 frame_id、伪造 header 时间或按路径哈希配对。

严格 NavigateThroughPoses 要求各目标在定位 map frame 中；每次规划用同一带身份起点按序移除已到达的中间点，默认 XY 半径 0.2 m，并更新业务黑板。最终点始终保留，由控制器执行 yaw 和停稳验收。该逻辑避免周期重规划不断回到已通过点，也不使用另一次无身份的 latest TF 来决定是否经过。

## 失效与恢复

严格 BT 的 `EpochGuard` 在失定位、身份变化、gate session 变化或业务目标更新时撤销授权、halt 子树、清空路径并作废未完成请求。业务目标保留；健康和数据恢复后，重新规划、平滑、安装新路径，再放行。

每条非零最终命令必须同时符合当前可信定位、活动 BT intent、controller 的已安装路径；三者 token 必须完整相等，controller session 也要一致。任何旁路裸 Twist 均不能进入严格输出链。gate 重启生成新 session；授权已经建立后发生实际失效也撤销 session，从而强制新规划，不能靠恢复心跳复活旧路径。正常重规划中 intent 先到而安装确认尚未到的短窗口仅停车，不应不断自我撤销新规划。

不同 ROS topic 不具有事务性。定位输出采用不可变内部身份，以及发布前/后的身份核对；controller 还需在对应源时刻核对 TF 与 typed pose 相符。这里的保证以消费者收到的状态与超时为界，不声称网络传播延迟为零。

正常到达与故障撤销分别处理：controller 对已安装 token 报告 `goal_reached` 后，gate 立即停车、清缓存并永久封存该 token，保持 challenge 不变，让原 FollowPath action 的成功结果能够被行为树消费。后续 `controller_idle`、同 token 的 active 重放或命令都不能重新放行。新的任务/路径需要新的安装握手。初次全栈实验确实发现若把正常到达当成故障轮换 challenge，会在到点后不断重新规划而无法完成业务目标；该失败记录保留为回归证据。

定位重复 heartbeat 不能延长不再推进的 output source stamp 的寿命；倒退源时间需失效。规划 action 的迟到 ACK 在 halt 后仍须取消，不能因没有获得 goal handle 就遗留可重新执行的动作。

## 时钟和部署边界

规划、BT、controller、平滑器及 gate 在同一 Linux boot/单调时钟域运行。命令源年龄使用 `CLOCK_MONOTONIC` 纳秒与 boot ID；平滑过程不得更新源命令时间、序号或有效期。ROS `/clock` 暂停或回退不会延长源命令寿命。不同 boot 的命令失败关闭，尚不支持跨机器传递该单调时间契约。

当前 BT 的定位心跳、有效输出推进与 gate 心跳 freshness 预算为 0.4 秒；规划/平滑动作默认 5 秒、ACK 等待 2 秒，均为单调时钟。健康恢复等待 60 秒，业务目标总时限 600 秒。数值为本轮桌面实验预算，尚未作为 RK3588 时延验收。

本轮绑定首个地图加载实例；变化后停止等待显式重载导航地图，不能让新定位地图搭配旧 global costmap。完整热切图仍未实现。

## 实验入口与尚未完成的范围

`epoch_navigation_experiment.yaml` 在基线、前向策略和模型 footprint overlay 之后合并。`enable_epoch_contract=true` 同时用于 BT navigator、controller、velocity smoother 和 gate。严格 navigator 选取包内 `navigate_to_pose_with_epoch.xml` / `navigate_through_poses_with_epoch.xml`，拒绝业务请求指定未经验证的替代 BT；原默认模式仍兼容。

严格 BT 本轮仅支持清图和等待恢复，裸 Spin/BackUp 不会自动获得身份。带身份的显式受限运动恢复仍是完整 P2 的待办，不能因本轮静态恢复可运行就缩小总目标。全栈故障测试使用真实 Nav2 算法/节点，plant 仅提供反馈、raycast 感知和可控定位 fixture；这不等于实际 Lightning 定位精度、RK3588 或实机验收。
