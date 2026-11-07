# ArmHold 内重复单次行走入口

- 状态：训练接管角差调整已部署，首次3秒软件窗口结束、ArmHold repeat idle；初次用户确认持续迈步、归零后站稳；含无外部悬吊/防摔支撑的一次重复 WALK 已实机确认，用户报告迈步并独立站稳。当前 PTY27864 保持。
- 目标：未来在安全维护窗口安装后，在同一 runner、同一 publisher 和 ArmHold 内由明确的新终端 `WALK` 指令触发一次固定 `(0.12,0,0)`、3 秒脉冲，归零后继续保持。
- 范围：仅 opt-in 的人工 runner 与定向 fake 测试；默认单次行为不变。非目标：当前进程热更新、外部竞争发布、自动重播、任意速度或变更模型/保护。
- 实施：①设计输入排队边界，脉冲中旧 WALK 不得在结束后执行，不能丢失 REVOKE；②保留每次独立单次 pulse、150ms 租约及既有状态/CRC/guard；③验证正常重复、输入排队、就绪丢失、租约到期、EOF 和故障；④仅在未来明确安全维护条件及授权下部署。
- 验收：每次新命令至多一个脉冲，结束全零且不重播；每次单独记录请求、起止、命令、非零及零观测、取消原因；异常保留保护和人工交接。
- 风险：当前运行的 Python 进程无法热加载该入口，不能通过覆盖源文件或并发发布实现；无吊带条件下不得撤权、释放、切模式、重启或停止进程。

- 2026-11-07：用户确认可靠支撑允许一次撤权更新重启，旧26347撤权确认、node/monitor退出0；备份 releases/repeat_walk_20261107/backup.tar。runner hash 07796ab1d3c70bb96fcb463563ade924719870c2978a9dff21997c5b4b9e7702，repeat_input hash d83f034c4a7de8b9ace8860bc2b4edadddc1933f48af7ee29c4bea9455aa57cb。试验记录 acceptance-20261107T115142，prearm SDK4偏差0.273897拒绝，未发送非零速度；未验证repeat功能。

## 2026-11-07 ArmHold 接管角差调整（本地）

- 目标：训练配置下 ArmHoldStand 到 ArmHold 的默认角差和膝目标角差只记录，不作接管门限；legacy 维持原膝门限。
- 范围：节点接管判断、runner 及定向测试；不修改模型、PD、准备至少 3 秒、有效状态、重力、授权和运行期保护，不改变单次入口时序。
- 步骤：①保留偏差诊断并调整训练接管；②验证训练较大角差、legacy 拒绝及运行期保护；③记录结果。此项已部署。
- 风险：训练接管时目标与实测仍有偏差，现场根因未确认；离线测试不证明真机安全或动作表现。

- 完成：Node 记录最大默认角偏差与 SDK 索引但训练接管不再拒绝；Controller 仅训练接管不检查膝目标差，legacy 保留。状态字段 `entry_angle_gate` 显示执行或仅诊断；runner 原有训练路径已强制 continuous，不另改。
- 验证：`cmake --build build/h1_2_policy_bridge -j2 --target h1_policy_node_integration_test h1_policy_core_test` 成功；`ctest --test-dir /home/llx/h1_nav2_ros2_ws/build/h1_2_policy_bridge -R '^(core|node_integration)$' --output-on-failure` 2/2 通过；新增实际位置软界场景后单独重跑 node_integration 1/1 通过。Fake IO 中 SDK3 偏差 0.27 接管，越实际位置软界连续三样本后触发原生阻尼；legacy 膝界测试通过。无本次保存的 hash。
- 历史阶段待办已完成：主会话审阅、部署及首次软件窗口结束；多轮重复 WALK 仍待实机验证。

- 2026-11-07 部署验证：备份及源码/产物 hash 位于板端 releases/entry_diagnostic_20261107；未授权 status 为 diagnostic_only、state valid、无 publisher、write_count 0。一次试验 acceptance-20261107T171020 报告进入 ArmHold、非零后零命令和窗口结束，PTY27864 idle；用户确认持续迈步、归零后站稳。

- 重复 WALK：同一 PTY27864 一次请求前后 status 为 ArmHold、ready、零命令，guard未触发；PID和 writer GUID `011053416022fa9b8f51f8a700000403` 不变，约3.7秒后返回idle。runner在窗口内检查ArmHold；repeat事件尚在内存，非零观测待记录落盘，用户确认再次迈步、结束后站稳；第一轮同进程 repeat 已实机确认。未调用模式切换服务、重启或撤权，授权已使用；未追加第二次。记录 acceptance-20261107T171020。

- 无外部悬吊/防摔支撑重复试验：用户确认站稳、净空、可急停并授权一次WALK；前后status均ArmHold、ready、零命令、guard未触发，同PID/GUID，runner窗口中检查模式，PTY返回idle。repeat事件仍在内存、非零观察未独立落盘；用户确认已迈步、结束后独立站稳。未切模式/重启/撤权/重复触发，PTY27864 idle，本次授权已用；不推及长时或导航安全。记录 acceptance-20261107T171020。
