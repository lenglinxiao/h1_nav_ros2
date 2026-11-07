# 2026-11-06 12:04 现场观察短速度试验复核

板端来源：`/home/unitree/h1_nav2_ros2_ws/third_party/h1_policy_runtime/integration-record/acceptance-20261106T120400/{summary.json,trace.csv,telemetry.json}`；仅保存聚合。以下时间为板端 monotonic 秒，trace 约 10 ms 抽样，不等于全部状态。

- 用户报告本轮看到腿部迈步，且试验后已静止稳定；这是现场人员观察。日志独立证明命令和监测结果，不能单独证明物理迈步或持续行走。
- ArmHold 服务 5373.239691 接受；5376.289493 状态报告 ArmHold、navigation_ready=true、policy_cmd=0。summary 对进入状态标记 `service_accepted_unconfirmed_by_dds`，不将服务响应等同独立 DDS 模式确认。
- `manual_nonzero_published=true`、`manual_policy_nonzero_observed=true`；首次观察非零策略命令状态响应 5376.459571，policy_cmd=0.12,0,0。记录没有精确首次发布时间。`manual_policy_zero_in_armhold_observed=false`，保护中止了发令后持续零速站立全流程；由于没有精确 start 时间，无法确认 0.5 秒发令窗口是否完成，也未证明持续行走通过。
- SDK3 左膝首个 |dq|>3 的 trace 采样在 5376.871638，dq=+3.153418 rad/s，状态接收 5376.869947；最近低层命令接收 5376.864362，目标 q=0.801530、kp=200、kd=4。守卫 5376.878772 观察累计峰 3.153418>3；撤权请求 5376.879401，响应 5376.880024。响应说明停止发布，电机行为未验证。
- 窗口按可证实标记划分：ArmHold 响应至首次非零状态请求前 313 个 trace 样本，SDK3 最大绝对速度 0.193404；由于首次发布时间未知，此窗口不能称严格脉冲前。首次非零状态响应至撤权请求前 41 样本最大 3.153418；撤权后观测窗口 49 样本最大 4.099905（5376.943702，dq 为负）。最终 telemetry SDK3 累计峰 4.256898 包含撤权后，不能归为撤权前；抽样峰值不等于全频峰值。
- 结果 `needs_operator_check`，`revocation_confirmed=true`，node 和 monitor 返回码 0；事后只读 pgrep 未见目标进程。进程退出或撤权本身不证明电机物理行为。
