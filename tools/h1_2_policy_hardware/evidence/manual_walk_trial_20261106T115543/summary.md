# 2026-11-06 11:55 短速度试验复核

板端来源：`/home/unitree/h1_nav2_ros2_ws/third_party/h1_policy_runtime/integration-record/acceptance-20261106T115543/{summary.json,trace.csv,telemetry.json}`。本地仅保存聚合；时间均为板端 monotonic 秒，trace 为约 10 ms 抽样，不等于 500 Hz 全部状态。

- ArmHold 请求 4876.142879，4876.143135 接受；4879.192796 状态为 ArmHold、navigation_ready=true、policy_cmd=0。summary 的 policy_entered 为 `service_accepted_unconfirmed_by_dds`，不能把服务响应解释成独立 DDS 模式确认。
- summary 记录 `manual_nonzero_published=true` 和 `manual_policy_nonzero_observed=true`；4879.362492 状态响应首次观察 policy_cmd=0.12,0,0。代码中这两个标记分别在非零发布和状态观察后设置；进入发布分支需先调用 pulse.start，但记录没有保存其确切时刻。不能从首次状态观察推定首次发布时刻，也不能证实现场迈步。
- 4879.473561 trace 首个 |SDK9 dq|>3 采样为 -3.130776 rad/s，状态接收 4879.472787；最近记录低层命令接收 4879.453915，SDK9 目标 q=0.732345、kp=200、kd=4。守卫 4879.481677 首次观察累计峰 3.130776>3；撤权请求 4879.481989、响应 4879.482726，回复表示停止发布，电机行为未验证。
- 窗口仅按可证实的标记划分：ArmHold 服务响应至首次非零状态请求前，314 个 trace 样本 SDK9 最大绝对速度 1.419614（4879.360820）；该窗口末端可能已开始发令，不能称为严格脉冲前。首次非零状态响应至撤权请求前，11 样本最大 3.130776；撤权后观测窗口 50 样本最大 3.912912（4879.483762，状态接收 4879.482960，最近命令接收 4879.473859，q=0.678114、kp=200、kd=4）。这些是抽样峰值，不等于全频峰值；最终 telemetry 的 SDK9 累计 4.016044、SDK10 累计 5.362251 包含撤权阶段，不能归为撤权前。
- `manual_policy_zero_in_armhold_observed=false`，中止前未证实脉冲完成后归零。结果 `needs_operator_check`，`revocation_confirmed=true`，node/monitor return code 0；只读进程检查当时未见目标进程。撤权及进程退出不证明机器人目前物理静止或已安全支撑。原因尚不能在策略瞬态和速度输入作用之间定论。
