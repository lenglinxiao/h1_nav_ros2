# H1-2 ONNX 运动与导航开发计划

- 更新：2026-11-09；状态：短直行和单次左转已有现场基线；1A–1C 已在隔离副本完成本地实现，1D owner/session 集成仍在本地开发，真实导航待验收。
- 由本次讨论补记后续计划：此前的任务和故障文档已记录具体实现及试验，但未见将动作通过后的导航联调顺序汇总为完整计划。本计划中待办步骤是当前讨论形成的方案，不代表已有现场动作授权。
- 当前任务、授权及历史现场记录见 [训练保护与行走试验](../../tasks/20261106-trained-policy-guard.md)；入口见 [SESSION.md](../../SESSION.md)，分阶段执行标准见 [测试计划](test-plan.md)；具体任务及建议架构见 [开发计划](development-plan.md) 和 [设计提案](design.md)。现场状态和运行版本须在操作时重新核实。

## 目标与范围

保留现有定位、地图、Nav2 路径规划和速度平滑，将运动执行切换至 ONNX 策略，并通过分阶段真机验收建立可追溯的运动与导航基线。范围涵盖已有接口的联调、停止和恢复行为，以及必要的导航参数调整；不以本计划重写 Nav2、修改模型或 PD 增益、扩展既有速度限制，也不关闭状态、速度、急停或撤权保护。

控制后端在进程启动时选择：离线导航入口支持 `motion_backend=unitree|policy`，默认 `unitree`；运行期间不切换控制器。停止/恢复仅针对导航会话，常驻 ONNX 控制生命周期另行管理。在线 `mode=online` 的 camera_init launch 暂不支持 ONNX 后端。

目标链路：

```text
定位 / 地图 → Nav2 → /cmd_vel → velocity_smoother_ema
→ 会话 cmd_vel → coordinator → /h1_2/policy/cmd_vel → h1_2_policy_bridge → ONNX → rt/lowcmd
                                                                  ↑ rt/lowstate
```

已有 `motion_backend:=policy`、连续策略模式和 [policy_navigation_gate](../../src/h1_2_nav_tools/h1_2_nav_tools/policy_navigation_gate.py)。1A–1C 的会话接口、coordinator/action proxy 和 claimed GID 输入已合入 main 并完成本地验证；owner、runtime identity 和 session launch 已纳入本次待提交改动，1D 的启动准入联测与 verifier 接线仍未完成。实现已加入严格 session namespace 校验及 shutdown inactive-writer gate。离线导航 shell 已增加启动时选择参数，但 GUI 尚未透传。以上本地实现不代表部署或真实导航验收。接口和已有限速、超时说明见 [策略桥 README](../../src/h1_2_policy_bridge/README.md)。`/h1_2/policy_ready` 是软件就绪信号，不证明现场平衡或控制权独占。

## 进度与验收

2026-11-07 左转软件窗口：固定(0,0,+0.1 rad/s) 3秒，runner观察预期命令及归零，用户确认向左转动、归零后站稳；仅此单次参数通过，右转、更大角度、长时和导航转向待验。当时 PTY26347 保持，现已是历史记录。见[转向计划](../h1_2-onnx-turn-test/plan.md)。

2026-11-07：一次授权0.12 m/s、3秒窗口runner报告结束且观察到非零和随后零 policy_cmd，无取消提示；用户现场确认持续迈步、归零后站稳。随后PTY72345已确认撤权，节点/监测器退出0；受支撑release与真实导航仍待验。记录板端 acceptance-20261107T112810。

| 阶段 | 状态 | 计划和通过条件 |
| --- | --- | --- |
| 1. 固定动作基线 | 短直行、单次左转已验证，扩展待验 | 已有 0.12 m/s 直行 3 秒、+0.1 rad/s 原地左转 3 秒现场观察；同一 ArmHold 重复 WALK 和无外部防摔支撑下站稳由用户确认。右转、长时未验。 |
| 2. 常驻控制与接入 | 软件实现进行中，未部署 | 1A–1C 已在隔离副本本地验证；主工作区 1D owner/session 仍待端到端 fake-service 测试、身份核验接线与 scoped review。GUI 尚未透传后端；不得假定私有实例可热接。 |
| 3. 停止与恢复 | 待办 | 导航停止/心跳丢失只取消目标、零速 hold，不杀策略；恢复要求新目标，详见开发任务 4。 |
| 4. 限速与短直线目标 | 待办 | 默认禁用导航源，隔离 mock 与只读核对后，另获现场授权补验双向 yaw，再做不超过 3 秒命令链短测、取消归零；到达验收另行授权时长与范围，详见开发任务 5–6。 |
| 5. 扩展场景 | 待办 | 基于已验证步态范围，逐步验证转向、多目标及障碍场景；每项记录目标、通过条件和实际结果，再根据跟踪表现调整必要导航参数。 |
| 独立事项：退出与交接 | 未解决 | 受支撑释放、撤权后的实际电机状态和控制交接须单独分析、验证；行走或导航通过不代表退出安全。故障见 [问题记录](../h1_2-policy-troubleshooting.md)。 |

历史 1.5 秒试验曾因 ready lease 失效中止，不能视为完整窗口；后续 3 秒结果及 repeat 的证据边界以[当前任务](../../tasks/20261106-trained-policy-guard.md)为准。当前 PTY27864 最近记录为 ArmHold idle，无外部悬吊/防摔支撑；用户确认独立站稳，现场状态易变须复核。
## 下一步与约束

1. 完成 1D owner/session fake-service 联测与身份准入接线，再接 GUI 启动后端选择；不触碰现有 PTY 或自动迁移。
2. 保留 0.4 秒超时、训练运行期保护和所有授权/急停/撤权保护；不关闭底板进程或切模式来接导航。
3. 命令、控制状态和现场运动证据分开记录；真实导航、右转及长时尚未验证，现场操作须有当时有效授权。

## 计划变更

- 2026-11-07：根据本轮讨论补记动作通过后的分阶段导航计划；历史实现和证据继续保留在原任务、README 与问题记录中，未修改现场配置或执行动作。

- 2026-11-07：基于接入缺口拆分为六项可验收开发任务，设计仍待审阅；无现场操作。
- 2026-11-09：用户确认只需要进程启动时选择控制后端，不需要运行中热切换。`run_ai_slam_nav.sh` 的离线导航增加 `motion_backend=unitree|policy`，默认 Unitree；在线 camera_init 入口仍不支持 policy。静态入口测试 8 项、launch 后端测试 9 项通过，bash 语法检查通过；真机未验证。
- 2026-11-09：本地继续实现 1D session namespace 和 owner shutdown gate；隔离构建 `/tmp/h1nav-owner-colcon-*` 中 `--packages-up-to h1_2_nav_tools` 成功，session launch、coordinator、owner 和 launch backend 定向测试 35 项通过。仍未做 owner ROS claim/confirm fake-service 联测，Task 1 未完成；未启动 ROS 控制节点或连接机器人。
