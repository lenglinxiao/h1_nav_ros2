# H1-2 ONNX 运动与导航开发计划

- 更新：2026-11-14；状态：原导航 Web 平台已部署 PC4 主工作区，12 包构建及 Web 服务检查通过；Nav2/Policy 真机闭环未启动或验收。
- 本计划保留历史实施和试验记录；下方 2026-11-12 架构收敛决定优先于旧阶段描述。待办步骤不代表已有现场动作授权。
- 当前任务、授权及历史现场记录见 [训练保护与行走试验](../../tasks/20261106-trained-policy-guard.md)；入口见 [SESSION.md](../../SESSION.md)，分阶段执行标准见 [测试计划](test-plan.md)；具体任务及建议架构见 [开发计划](development-plan.md) 和 [设计提案](design.md)。现场状态和运行版本须在操作时重新核实。

## 2026-11-12 架构收敛决定

- 目标按用户确认的核心链路收敛：`Nav2 /cmd_vel -> 现有速度平滑与限幅 -> /cmd_vel_final -> ONNX bridge -> LowCmd`。Unitree 与 Policy 仅启动时二选一，运行中不切换。
- 删除仅为动态热切换、多源仲裁和隔离旧 session 引入的常驻 Owner/Coordinator、session token/epoch、prepare/activate、输入 writer claim、action proxy 和独立 Nav2 session manager。Nav2 action 恢复为标准全局 action 名。
- 保留 bridge 现有硬件输出人工授权、LowState 新鲜度、训练运行期保护、速度输入超时归零和进程唯一锁；删除服务不能绕过现场支撑/撤权流程。Policy 后端实际 Nav2 转向和闭环包络仍按阶段验收，历史人工脉冲不是全向安全证明。
- 停止流程：Policy 输入超时归零并进入人工支持/撤权后再结束控制进程；不让 stop 脚本在 Policy 路径发送 Unitree UDP 命令。若本地调试/假节点验证失败，不部署、不启动真机控制。

## 2026-11-12 简化实现进度

- 已实现：两个 Nav2 launch 都启动标准 Nav2 action 和既有 `velocity_smoother_ema`；Policy 启动时直接运行固定 bridge，读取 `/cmd_vel_final`，与 Unitree launch 分支互斥。Shell 将 Policy ROS 全局 RMW 设为 Fast DDS，以匹配 Unitree bridge；输入 0.4 秒过期归零，Policy 默认速度上限由启动入口设为 0.12 m/s、0.10 rad/s。GUI 增加 debug/独占控制确认。
- 已移除：源码的 Policy owner/coordinator/session manager、token化 action 客户端、claimed writer 与 runtime identity 服务和接口；Policy 导航不再代理 `/navigate_to_pose`。
- 保护保留：硬件输出必须操作员调用授权服务开启；训练保护参数从固定审阅 profile 加载；LowState/CRC/进程锁/输入超时/ArmHold/受支撑 release/revoke 流程不移除。停止脚本只在 bridge 确认 ArmHold 后停 Nav2 子进程，Policy bridge 本身继续运行等待人工支撑和撤权。
- 验收状态：隔离 colcon 构建 3 包通过；Policy bridge CTest 6/6、launch/profile/navigation backend 定向测试通过；Web GUI 31项通过；py_compile、shell -n、git diff --check 通过。未部署、未启动 ROS 控制栈或做真机测试。
- 遗留：仅目标机部署/运行和真实 Nav2 短测未完成；需按硬件核对清单重新核验现场状态与授权，软件单测不替代真机闭环验收。

## 2026-11-13 原导航开发平台接入在线 Policy 目标控制

- 目标：在原 `h1_2_nav_web_gui.py` 中继续使用现有任务/点位/开始巡航界面，通过标准 Nav2 action 驱动在线 ONNX Policy 链路。
- 范围：在线 `camera_init` 帧下记录和读取点位；在线巡航 readiness 改为检查 `camera_init -> body` TF；在线点位 runner 使用同一坐标系。离线地图仍使用 `map` 和 ICP 状态。不同 frame 的旧点位明确拒绝，不自动变换或覆盖。
- 非目标：不改 `8781` 手动速度测试页，不新建网页，不绕过 Policy 输出授权/训练保护，不执行真机导航动作。
- 完成与验证：在线点位保存及路线 TF 门控统一到 `camera_init -> body`；原 Web GUI 在线状态采用新鲜位姿 TF，离线仍走 ICP/map；Policy 巡航按钮还要求 bridge 授权、ArmHold、有效 LowState、publisher 与 navigation_ready；页面也可将匹配模式/后端的终端启动实例接入当前任务。定向测试 85 passed，相关 Python/launch `py_compile`、`git diff --check` 通过；ROS Humble overlay 下隔离构建 `h1_2_nav_tools h1_2_slam_bringup` 两包通过。
- 状态：本地实现完成；尚未部署 PC4 或执行真机目标。在线模式请新建任务重新记录 `camera_init` 点位；旧 `map` 点位不会自动换算。

## 2026-11-14 PC4 原导航平台部署

- 已同步本地源码至 PC4 主工作区并完成 12 包原生构建；旧 build/install 缓存改名保留后重建。Policy bridge 构建显式使用 PC4 本地 ONNX Runtime 和 Unitree SDK 路径。
- 修复 PC4 Cyclone DDS 网卡配置：运行时只有 `eth0` 活跃，旧 XML 绑定的 WiFi 网卡已不存在。部署后 Web 服务 active，8765/8766 监听，HTTP 200，页面已包含 ONNX/Policy 选项。
- 未启动 Nav2 或 Policy bridge、未授权输出、未发送目标。下一步由操作者刷新原网页、选 online + policy、新建任务并重录 `camera_init` 点位；实机目标前先新鲜核验 writer/控制权和现场准入。

## 2026-11-13 原导航开发平台接入在线 Policy 目标控制

- 目标：在原 `h1_2_nav_web_gui.py` 中继续使用现有任务/点位/开始巡航界面，通过标准 Nav2 action 驱动在线 ONNX Policy 链路。
- 范围：在线 `camera_init` 帧下记录和读取点位；在线巡航 readiness 改为检查 `camera_init -> body` TF；在线点位 runner 使用同一坐标系。离线地图仍使用 `map` 和 ICP 状态。不同 frame 的旧点位明确拒绝，不自动变换或覆盖。
- 非目标：不改 `8781` 手动速度测试页，不新建网页，不绕过 Policy 输出授权/训练保护，不执行真机导航动作。
- 验收：定向测试覆盖在线/离线 frame 参数、旧 frame 拒绝、在线网页 readiness 与 Nav2 标准路线入口；隔离构建及静态检查通过。真机启动与发目标另行按现场清单准入。
- 状态：实现中。剩余工作为改造 memory point frame 可配置、在线 launch 配置、网页 readiness 与任务 frame 检查，并运行定向测试。

## 目标与范围

保留现有定位、地图、Nav2 路径规划和速度平滑，将运动执行切换至 ONNX 策略，并通过分阶段真机验收建立可追溯的运动与导航基线。范围涵盖已有接口的联调、停止和恢复行为，以及必要的导航参数调整；不以本计划重写 Nav2、修改模型或 PD 增益、扩展既有速度限制，也不关闭状态、速度、急停或撤权保护。

控制后端在进程启动时选择：在线建图和离线地图导航均支持 `motion_backend=unitree|policy`，默认 `unitree`；运行期间不切换控制器。ONNX 只替换运动执行接口；FAST-LIO、实时占据栅格、定位、Nav2 规划和 GUI/目标上游保持原有数据流。Policy 模式下 bridge 独立于 Nav2 进程运行；停止导航先令 bridge 进入 ArmHold，再停止 Nav2，随后按现场支持/撤权流程结束 bridge。

当前目标链路：

```text
定位 / 地图 → Nav2 → /cmd_vel → velocity_smoother_ema
→ /cmd_vel_final → h1_2_policy_bridge → ONNX → rt/lowcmd
                                                                  ↑ rt/lowstate
```

当前实现由 `run_ai_slam_nav.sh` 在 Policy 模式单独启动 bridge，Nav2 stack 使用标准 `/navigate_to_pose`，速度经已有 smoother 输出至 `/cmd_vel_final`。Unitree bridge 与 Policy bridge 按启动模式互斥。Policy bridge 的固定训练 guard 参数和速度上限在 bridge launch 配置；物理输出仍由操作者授权。`/h1_2/policy_ready` 只是软件状态，不证明现场平衡或控制权独占。

## 进度与验收

2026-11-07 左转软件窗口：固定(0,0,+0.1 rad/s) 3秒，runner观察预期命令及归零，用户确认向左转动、归零后站稳；仅此单次参数通过，右转、更大角度、长时和导航转向待验。当时 PTY26347 保持，现已是历史记录。见[转向计划](../h1_2-onnx-turn-test/plan.md)。

2026-11-07：一次授权0.12 m/s、3秒窗口runner报告结束且观察到非零和随后零 policy_cmd，无取消提示；用户现场确认持续迈步、归零后站稳。随后PTY72345已确认撤权，节点/监测器退出0；受支撑release与真实导航仍待验。记录板端 acceptance-20261107T112810。

| 阶段 | 状态 | 计划和通过条件 |
| --- | --- | --- |
| 1. 固定动作基线 | 短直行、单次左转已验证，扩展待验 | 已有 0.12 m/s 直行 3 秒、+0.1 rad/s 原地左转 3 秒现场观察；同一 ArmHold 重复 WALK 和无外部防摔支撑下站稳由用户确认。右转、长时未验。 |
| 2. 固定链路接入 | 本地实现完成，未部署 | 标准 Nav2 action、已有 smoother 和直接 Policy bridge 已构建；只做启动时后端选择，不支持运行中切换。 |
| 3. 停止与恢复 | 软件停止逻辑本地检查通过，现场待验 | 停止脚本请求 bridge ArmHold 并确认后结束 Nav2 进程组；bridge 保持运行，退出由人工支持/撤权完成。浏览器心跳策略仍是 Web GUI 自身行为，不参与 Policy 输入或 Nav2 会话仲裁。 |
| 4. 限速与短直线目标 | 待办 | Policy 默认限速 0.12 m/s、0.10 rad/s，速度输入 0.4 秒失鲜归零；只读核验现场状态及授权后，做不超过 3 秒命令链短测、取消归零。真实 Nav2 闭环和到达验收尚未完成。 |
| 5. 扩展场景 | 待办 | 基于已验证步态范围，逐步验证转向、多目标及障碍场景；每项记录目标、通过条件和实际结果，再根据跟踪表现调整必要导航参数。 |
| 独立事项：退出与交接 | 未解决 | 受支撑释放、撤权后的实际电机状态和控制交接须单独分析、验证；行走或导航通过不代表退出安全。故障见 [问题记录](../h1_2-policy-troubleshooting.md)。 |

历史 1.5 秒试验曾因 ready lease 失效中止，不能视为完整窗口；后续 3 秒结果及 repeat 的证据边界以[当前任务](../../tasks/20261106-trained-policy-guard.md)为准。当前 PTY27864 最近记录为 ArmHold idle，无外部悬吊/防摔支撑；用户确认独立站稳，现场状态易变须复核。
## 下一步与约束

1. 在目标机安装本地验证版本后，先只读核验 ROS graph、writer 与地图，再按独立准入步骤做受限导航短测；本地完成不代表获准真机操作。
2. 保留 0.4 秒超时、训练运行期保护和所有授权/急停/撤权保护；不关闭底板进程或切模式来接导航。
3. 命令、控制状态和现场运动证据分开记录；真实导航、右转及长时尚未验证，现场操作须有当时有效授权。

## 计划变更

- 2026-11-07：根据本轮讨论补记动作通过后的分阶段导航计划；历史实现和证据继续保留在原任务、README 与问题记录中，未修改现场配置或执行动作。

- 2026-11-07：基于接入缺口拆分为六项可验收开发任务，设计仍待审阅；无现场操作。
- 2026-11-09：用户确认只需要进程启动时选择控制后端，不需要运行中热切换。最初仅离线导航接入 Policy。
- 2026-11-10：用户澄清优先验证在线建图，并要求仅替换宇树运控接口，上下游尽量不变。当时在线 Policy launch 曾接入 coordinator/ONNX；该中间方案已被 2026-11-12 架构收敛决定取代。
- 2026-11-09：曾实现 1D session namespace 和 owner shutdown gate，并留下相应隔离构建/测试记录。此实现随后被用户要求移除；只作为历史，不是当前接口或待办。
