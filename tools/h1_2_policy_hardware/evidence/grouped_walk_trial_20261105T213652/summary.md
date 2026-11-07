# 2026-11-05 21:36 分组门限试验只读聚合

板端记录 `/home/unitree/h1_nav2_ros2_ws/third_party/h1_policy_runtime/integration-record/acceptance-20261105T213652/` 的 `summary.json` 与 `trace.csv` 按时间和关节索引只读聚合；未复制原始整包。时间为板端 monotonic，角度 rad，速度 rad/s。

| 观测 | 左膝 SDK3 q / dq | 右膝 SDK9 q / dq |
|---|---:|---:|
| ArmHoldStand 端点（monitor time 7.845545 s） | 0.963871 / -0.001389 | 0.913728 / 0.011334 |
| 撤权请求前最近 trace（monotonic 9990.432408 s） | 0.970870 / -0.000202 | 0.918026 / 0.000596 |

膝门条件是严格 `abs(q-0.9)<0.07`，即 0.83 < q < 0.97。端点双膝满足；撤权前最近轨迹中左膝高于上界约 0.000870 rad，右膝满足。相对端点，左、右分别漂移约 +0.006999、+0.004298 rad；端点至撤权前最近轨迹约 39.95 s。端点后 trace 中左膝首次超上界为 monotonic 9951.096273 s，q=0.970055、dq=0.020068，距端点约 0.615 s；此后至撤权前最后一个超界采样相隔约 39.336 s。上述只是采样点的时间范围，不证明期间连续越界。右膝该区间 trace 无越界采样。

现场 `CONFIRM` 后的 `watch(.25)` 最后成功窗口为 0.259800 s，随后脚本报 `Knee position gate failed after onsite confirmation`。最后复核读取的精确 telemetry 帧未独立保存，因此撤权前的 trace 是邻近观测，不能称为精确触发帧，也不能仅据该漂移判断原因。

- summary：`preparation_result=aborted`、最终 `needs_operator_check`；`arm_hold` 服务事件为 0，所有 817 条 status 中 `policy_cmd=0,0,0`，无 manual pulse 记录，因此未进入策略行走阶段。
- 本次配置：准备及 pulse 前全身 2 rad/s；pulse 启动后腿 SDK0–11 为 3 rad/s、上身 SDK12–26 保持 2 rad/s，撤权观察沿用分组。不是关闭速度保护。全程累计峰 1.374758 rad/s，无记录超限，观测完整。
- `revoke_output` 请求 monotonic 9990.432662 s，summary 确认撤权，后续 0.5 s 观察写计数不变；node、monitor 退出码均为 0。只读 `/proc` 检查无目标进程。撤权后电机行为不因此得到验证。用户随后报告静止稳定、支撑仍在。
- 分组 monitor 已板端编译，15 项专项 fake 测试通过；只读约 3 s 收到 1500 条状态、0 条命令。runner、monitor、speed_guard 已备份并安装于 `releases/grouped_walk_guard_20261105`。这些证据不证明分组门限下的实机行走通过。
