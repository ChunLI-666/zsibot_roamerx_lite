# DDS 发布区间审计与旧轨迹复审

本轮只修改独立审计器，不修改 Nav2、gate、安全阈值、运动积分或旧轨迹。旧版将本地 publish bracket 超过100µs视为整场不可评分；新版保留完整区间，判断该区间是否真正跨越相关安全边界。总goal仍未完成；本轮完成的是仿真证据的时间测量改进。

## 时间模型与判定

- DDS `source_timestamp` 保持原始wall整数；callback的monotonic前后采样与中间wall采样给出offset区间。若全场存在共同offset交集，使用同一个未知offset；DDS-DDS相减时该公共项消去，不重复叠加两份独立误差。
- 如果共同交集为空，但offset矛盾仍在原1ms监测限内，保留offset漂移包络；DDS-DDS相减仍保留漂移误差。超过监测限、时间戳缺失/非法或倒置仍不可评分。宽但有效的采样区间不能凭宽度直接判错。
- 本地localization publish保留完整before/after区间。中点仅用于显示和候选遍历；TTL、消息匹配、最新授权/定位状态都使用区间上下界。身份更新区间跨过safe发布时间且两个可能状态不同，判为不可评分。
- 每个判据采用三态：整个区间符合契约为PASS；整个区间违反契约为FAIL；同时包含可能通过和可能失败的时序为UNSCORABLE。聚合时确定失败优先，不能被其他不可评分项覆盖。
- 保持safe↔relay的对称±50ms观察匹配窗口、raw来源窗口、定位失效100ms观察断言。这个既有匹配模型允许窗口内的稍后同值relay作为观察匹配候选，不是对gate实际接收因果关系的证明。source TTL按实际producer/gate契约：command自身`max_age_ms`、execution 300ms、intent 400ms；不沿用旧callback审计的30ms额外观察余量，也不允许未来source时间。BackUp deadline按排他截止时刻检查。
- 已发布的确定inactive不能由随后发布的active反向授权。同一topic所有可能最新的state均参与判断。即使一个宽区间完全远离边界，也不会掩盖确定的定位失效或过期。

时钟结论依赖同机同boot的DDS时间和callback采样约束；有限离散采样无法证明两次采样之间不存在未观察到的时钟突跳。审计输出是发布端证据，未观察gate内部接收时刻，单个FAIL不能直接等同于gate实现缺陷。

`monotonic_ns`原记录、plant执行顺序、原审计JSON均不改动。新版输出单列`publication_clock`、`assessment`、`definite_failure_count`、`unscorable_count`和各边界最小余量；保留callback时间审计诊断。phase审计使用同一发布时间区间匹配safe与relay，并保留callback诊断。

## 冻结旧轨迹结果

只读复审21条既有trace：2条导航串行转向回归和19条BackUp矩阵，合计2248个非零safe样本。**21/21新版wire审计PASS**；两条导航的新版phase审计也通过。旧结果仍保留：18条有1项旧wire失败、1条有2项、2条原wire无失败。

导航两条旧本地publish bracket最大分别134.972µs和209.390µs。它们可以评分，是因为完整区间未跨越相关边界，而非阈值被放宽。两条trace的callback采样存在共同offset交集，宽度分别144ns和117ns；旧版约18/19.5µs的包络宽度不能直接解读为已证实的时钟漂移。

**重新通过的是wire证据维度。** BackUp既有地图case此前动作因odometry stale中止，新wire PASS不改变该动作失败；仅产生零速的预先声明拒绝case也不构成运动覆盖。旧plant中的100ms积分截断及watchdog末端判定限制仍存在，新审计不补写缺失运动，不把旧几何结果升级成完整执行动力学证明。此前没有DDS元数据的旧8场景矩阵不在这21条重新评分范围内，其原失败结论保持不变。

每条trace重审前后SHA256相同。新版完整结果另存：
`/home/charles/project/colcon_ws/analysis_outputs/epoch_interval_audit_20260911/`

附属`epoch_interval_audit_results_20260911.json`包含每条源路径、SHA256、旧失败数量、新结果路径及审计器源码SHA256。

## 验证

定向测试覆盖宽且远离边界的本地publish、跨身份切换、跨50ms/TTL、公共offset消去、残留漂移、未来active不得覆盖确定撤权、300ms真实execution TTL、缺失/非法DDS和不改变输入数据。45项定向测试通过，包含全DDS时间戳无效、确定失败优先于不可评分、有效/无效时钟下全NaN safe都确定FAIL。日志路径见附属JSON。

本轮不需要C++重建。Codacy在当前会话不可用。后续新增仿真使用root独立完善的连续wall时间plant；本报告不会将新plant的能力追溯应用于旧trace。
