# 2026-11-05 21:20 手动行走尝试：只读聚合

来源：板端 `integration-record/acceptance-20261105T212015/{summary.json,trace.csv,node.log}`，未复制原始轨迹。时间为板端 monotonic 秒。

- ArmHoldStand accepted，观测窗口 3.109 s；ArmHold accepted（8985.707），状态 `navigation_ready=true` 首见 8988.787。`policy_cmd=0.12,0,0` 首见 8992.077，之后发生超速。summary 的 `manual_policy_nonzero_observed=true`、`manual_policy_zero_in_armhold_observed=false`；events 中无 `pulse.start`，不能凭此判定其精确时间。
- 首个 trace 超 2 样本 8992.294464：SDK 9，dq=+2.16938448 rad/s，q=0.952445 rad，最近命令 q=1.026248 rad、kp=200、kd=4。summary 首次观测 8992.297018，累计峰 2.16938448，phase=arm_hold。
- trace 最高 8992.304727：SDK 9，dq=+2.22022390 rad/s，q=0.971380 rad，最近命令相同；这在撤权请求之后，无法用采样时间声称内部精确触发时刻或因果。
- 错误 `Preparation joint speed exceeded 2 rad/s`，preparation aborted，result `needs_operator_check`。revoke_output 请求 8992.297453，响应成功，summary `revocation_confirmed=true`，node/monitor 退出码均 0；服务明确只证实停止发布，未验证电机行为。只读进程检查未见相关目标进程。
- 这是本轮原有速度保护实际触发，与事后新增的工程 precheck 是不同事件；原因未确定，未验证物理安全或步行成功。
