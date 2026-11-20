2026-11-07 无外部悬吊/防摔支撑：原PTY27864一次WALK前后均ArmHold、ready、零命令、训练guard未触发，PID及writer GUID不变；窗口中runner检查模式，repeat内存事件未落盘。用户确认已迈步、结束后独立站稳；未切模式、重启、撤权或再次WALK，现idle，本次授权已用。不推及长时或导航安全。记录 `acceptance-20261107T171020`。

2026-11-07 重复 WALK：原 ArmHold 同一 runner 一次请求前后 mode=ArmHold、ready、零命令，PID和 writer 不变，约3.7秒后 idle；runner窗口内检查同一模式。repeat事件尚在内存，非零观测未独立读取，用户确认再次迈步、结束后站稳，第一轮同进程 repeat 已实机确认；未调用模式切换服务、重启或撤权，授权已使用且未追加第二次；记录 `acceptance-20261107T171020`。

2026-11-07 最新：训练入口角差诊断已部署，ArmHold 运行期保护保留；记录 `acceptance-20261107T171020` 的单次 0.12 m/s、3 秒软件窗口结束并归零，用户确认持续迈步、归零后站稳。PTY27864 ArmHold idle，未追加 WALK，repeat 多轮未实机验证；每次 WALK 需新明确指令。板端源码与产物完整散列：`releases/entry_diagnostic_20261107/sha256.txt`。

2026-11-10 开发范围更新：优先验证在线 camera_init 建图。GUI/shell的online Policy接入、camera-init launch session切换与保留实时映射链路已通过定向软件验证；下一步为制作包含该更新的独立板端release，然后重新做只读准入核对。现场writer归属及速度envelope未确认，不将软件通过当作可行走许可。

# H1-2 ONNX 运动与导航历史测试记录

> **当前实现已变更（2026-11-12）：** 本文中关于 Owner/Coordinator、会话 token、action proxy、writer claim 的实现和测试是旧架构历史证据，不是当前导航链路的验收要求。当前架构、软件验证结果和剩余真机验收见[开发计划](plan.md)及[当前任务](../../tasks/20261107-onnx-navigation-development.md)。

- 更新：2026-11-10；PC4 独立 release 已构建；增强 monitor 最近只读窗口观察到一个 `rt/lowcmd` writer GUID，但尚未映射到主机/进程。Owner/Policy/Nav2 未启动，实机控制验证暂停。功能范围、步骤进度见[开发计划](development-plan.md)，会话接续见[开发任务](../../tasks/20261107-onnx-navigation-development.md)。每次现场执行前按当时明确范围复核支撑、急停、调试状态、控制权及运行版本；历史确认不自动延续。
- 目标：逐层确认 ONNX 接管、受控直线行走、归零站立和 Nav2 集成，分别留下软件与现场证据。非目标：修改模型/PD、扩展速度上限、关闭保护或把模拟通过当作实机通过。

## 已有证据与待核对状态

- 隔离 ROS 的双 action 代理、ready 失效取消及旧目标清理已验证，证据在 [接口通信记录](../../tools/h1_2_policy_hardware/evidence/interface_comms_20261105/)；它不证明实机导航。代码、接口及条件未变时复用已有通过结果，仅在相关代码、依赖或运行条件改变时重测受影响项。
- 2026-11-06 的 0.5 秒试验观察到非零命令及归零 ArmHold，用户报告迈步后站稳；1.5 秒试验发生 `ready lease expired`，已观察非零及归零，不能认定完整窗口。
- 2026-11-06 会话执行记录显示：旧 1.5 秒会话已确认撤权、节点及监测器退出 0；3 秒上限及续租调度脚本已部署，备份位于板端 `releases/manual_pulse_3_20261106/backup.tar`，随后 3 秒试验会话已启动。**其输出及现场结果未取得**，不得写成通过或失败，也不得假定当前仍保持或已撤权。编写前任务/开发计划曾存在“未部署”及旧会话保持的陈旧表述，现已标记待核对；需在有条件时核对保存的板端记录和现场状态；本计划不以旧 PID、会话或用户确认作为当前状态。
- 训练配置的默认角差和膝目标差入口检查已部署为诊断；训练运行期位置、速度、姿态等保护保留。训练统计见 [训练记录](../train.md)。运行时核对实际 profile、模型、URDF、代码/产物 hash 与状态，缺失或不匹配即暂停。

**当前开发验证与下一项**：2026-11-10 本地 `nav_commissioning` 修复异常仍请求 hold、goal acceptance 计入deadline、失败 lifecycle 返回非零；定向6项通过。隔离 ROS Humble overlay 合并运行 commissioning、owner identity/claim、真实 owner supervisor、manager/fake Nav2 lifecycle 共9项通过；真实 owner 停止时会等 fake runtime 确认输出关闭。`h1_2_nav_tools` 隔离构建通过。ROS 测试使用不同 domain 和独立 context，避免进程内默认 context 泄漏及旧 participant 冲突。未启动真实 hardware bridge、monitor 或现场 Nav2。PC4 最新连接复核未能 SSH：开发机以太网 `NO-CARRIER`，地址路由误走 Wi-Fi；此前 PC4 状态快照不可当作当前状态。当前无现场批准 envelope，真机 nav source 仍应拒绝启动；writer归属、地图确认、右转、速度包络和Nav2实机均未验证。

2026-11-10 用户明确当前优先验证 online camera_init 建图，并要求 ONNX 仅替代 Unitree 运控接口、保持上游 SLAM/占据栅格与 Nav2 行为。软件入口现允许 online 选择 policy；目标板是否能运动仍取决于writer身份、现场速度envelope和控制准入，均未验证。

**2026-11-09 PC4 部署与只读核验**：代码部署至 `/home/unitree/h1_nav2_ros2_ws/releases/onnx-navigation-20261109-rc1`，源码包 SHA256 `da84c1227974d36c548614a94a86f761094eebfe655de31e721a4b52dcfa95cd`。目标 x86_64 / ROS Humble 独立构建 5 包成功，Policy Bridge CTest 6/6 通过，Owner `--check` 和 session launch `--show-args` 通过；Monitor 产物 SHA256 `72e26bbf3e9c6f05cb31b55c8645300b25b538879227c48a0b3dfc037d9e4be2`。只读 ROS 图未见 Policy、Coordinator 或 Nav2。3 秒 Monitor 记录 1493 条 LowState、1488 条 `rt/lowcmd`、CRC 0、状态 age 约 0.001 秒、mode 6/PR 0，但预期 writer GUID 未配置，命令来源无法归属。依据停止条件，没有安装/启动 Owner、Policy 或 Nav2，也没有发送运动命令；因此这是部署和软件检查通过，实机控制与导航仍未验证。详见[板端核对清单](hardware-validation-checklist.md)。

**2026-11-09 rc2/rc3 跟进**：增强 monitor 离线检查17/17通过，PC4编译成功。rc2五秒窗口2493条LowState/LowCmd，唯一GUID `0110d3b854fae87cff15d4d100000203`；DDS participant 同时发布 `rt/SymState`，但无host/PID。rc3增加 `last_seen_age`，五秒窗口2487条state/command、CRC0、age0.001603秒、mode6/PR0，同一GUID在采样末尾 `last_seen_age=0.000026`秒。PC4真实模型、URDF、rc3 policy executable/monitor 的 Owner `--check` 通过，只验证配置，不启动进程。PC4抓包权限不足，访问 `.161` 被SSH拒绝，writer主机/进程仍未确认。当前达到软件部署和静态配置准入，尚未达到Owner graph、策略验收或真机Nav2测试准入。详见[核对清单](hardware-validation-checklist.md)。

**2026-11-10 PC4 复核**：PC4 ROS Humble / domain 0 当前图包含 `/QtServer`、`/dog_control_pub`、`/switch_node`、`/bms_ctr_node`、Web GUI 等既有节点；`ros2 action list` 为空，没有 Policy、Coordinator 或 Nav2 action。rc3 monitor 5秒窗口为2488 LowState、2489 LowCmd、CRC0、age0.001529秒、mode6/PR0；唯一 writer GUID 更新为 `01109970b33f1c7fcc2df0e400000203`，participant 仍发布 `rt/SymState`，但无host/PID，不能确定来源或控制权。目标机 `/home/unitree/h1_nav2_ros2_ws/maps` 的 YAML SHA256 `0494cc2fce2266225aaca0717a229c9827fc5f134a64070b93313810c65b4598` 与开发机相同；PGM SHA256 `e478470c8e72eb7e63fa9854835e68f0bfe3a44e80f9b95b3025cd024208603f`、PCD SHA256 `743bd1f9936a018b8a41fee9cb6ce12de8de19c18f09f80ad23a02a2cd02df03` 与开发机不同。未覆盖地图。未启动控制节点或发命令；真机 Nav2 仍因 writer 归属和地图选择未通过准入。

**2026-11-10 后续连接复核**：用户确认PC4 IP未变。本机 `enp43s0` 为 `NO-CARRIER`，`ip route get 192.168.123.164` 选择 Wi-Fi 网关 `192.168.8.1`，SSH 连接超时。上面的 ROS graph、writer 和地图记录是此前快照，当前状态未复核；需恢复有线链路后先做只读核验。

**2026-11-10 接口复通**：本机有线地址 `192.168.123.99/24`、直连路由和 SSH 已恢复。PC4 节点图仅见既有 Unitree 节点和 Web GUI；action list 为空，未发现 Policy/Coordinator/Nav2 service/topic。Web GUI 首页 HTTP 200。没有启动 monitor、Owner、Policy、Nav2 或动作；writer 和地图需按板端清单重新核验。

## 分阶段执行与验收

| 阶段 | 前提与入口 | 通过标准、停止条件及证据 |
| --- | --- | --- |
| A. 离线及隔离 ROS | 检查改动范围、版本和已有证据；仅对改动项运行 `test_manual_pulse.py`、`test_acceptance_workflow.py` 对应案例，或 `ctest` 中受影响的 core/node/guard。导航接口使用独立 ROS 域、本地 mock Nav2 与 fake ready；不连接机器人控制域。 | 默认 0.5 秒及选定窗口按时归零；租约到期/ready 丢失锁存取消且不重播，旧 goal 不恢复；记录命令、结果、代码及环境 hash。已有结果适用且条件未变时不重复。注入延迟、失联、CRC 等故障只在 mock 或输出禁用环境中进行。 |
| B. 板端只读 | 仅在明确输出禁用且无冲突的条件下核对模型、配置和部署；入口为 [桥接 README 的 hardware 输出禁用启动方式](../../src/h1_2_policy_bridge/README.md)及 [监测工具说明](../../tools/h1_2_policy_hardware/README.md)。 | 新鲜 LowState、CRC、监测器 schema 3/`until_stopped`，节点 `authorized=false`、`publisher_present=false`、`write_count=0`；分别记录策略 writer 与其他 LowCmd 来源。发现冲突、状态过期或监测异常则停止进入下一阶段。GUID 只证明发布来源，不证明电机控制权。 |
| C. 单次短行走 | A/B 适用且现场明确授权；先核对上次退出及真实控制权。人工入口为 `tools/h1_2_policy_hardware/run_acceptance.py --continuous-walk-trial`，`position_pd`、训练 profile 和实际 URDF；在可交互终端指定经授权的 `--manual-pulse-seconds`，默认0.5、当前代码范围0.5–3秒。速度沿用 `vx=0.12 m/s`；单次窗口及动作范围由现场批准，不凭空增加次数、速度或时长。 | 确认准备、ArmHold、`policy_cmd` 非零和归零，以及现场迈步/停稳分别成立；记录窗口实际情况、现场观察及保护事件。ready/租约失效、训练 guard、CRC/状态异常、非预期运动或急停均中止并按现有路径交接，不自动重播、不追逐峰值提高门限。此前1.5秒取消；2026-11-07的3秒窗口和现场观察如上，不能泛化为持续行走安全。 |
| D. 零速保持与交接 | C 归零后继续观察，时长和条件现场确定；单独讨论机器人及双臂外部支撑和撤权/释放范围。 | 零 `policy_cmd`、状态新鲜和策略输出持续仅说明软件路径；现场站稳须人工报告。记录保持时长、guard/状态、LowCmd 及现场现象。按明确授权和现有 runner 交接，记录撤权确认、写入停止、进程退出及撤权后观测；不把 ArmHold/Hold 等同机械支撑，受支撑释放及撤权后电机行为仍是独立待验事项。 |
| E. 真实 Nav2 集成 | 软件入口为 `run_ai_slam_nav.sh mode=navigation motion_backend=policy ...` 或 GUI Policy 选择；必须另行配置并启动已审查的常驻 owner。owner/coordinator 加载有批准现场证据的 envelope 后才可接受 nav；当前没有该证据，因此不得期待 activate 成功。现场授权、地图定位、场地及安全支撑均须当时复核。 | 独立 Nav2 session 使用 coordinator 返回的 token topic/action namespace；确认 `/h1_2/policy/hold` 后才停会话，Unitree UDP stop 不用于 Policy。先完成独立域 mock graph；现场先按单独授权补右转与双向 yaw，再做不超过 3 秒命令链短测、到时 cancel/hold，不要求到达。目标板 profile、ready、guard、速度、定位及现场运动/停稳分开留证；当前均未完成。 |
| F. 停止与恢复 | 先在隔离 mock 或输出禁用环境验证 ready 丢失、命令过期、目标取消及恢复；真机仅在明确授权的受控、可恢复条件下验证适用行为。 | 确认旧目标/旧速度不在恢复时重播，超时命令归零；现场停止另由观察确认。不要在真机自动杀节点、拔网线或关闭 vendor 进程制造故障。异常时中止并分析记录，再决定下一步。 |

## 保护、记录和回退

- 保留 0.4 秒速度超时、0.15 秒 ready 租约、训练保护、授权与撤权、急停、状态/CRC和逐关节监测。当前修复在 ArmHold 以约 20 ms 请求起始间隔安排状态查询，仅有效响应续租；超过租约仍取消。该调度的 fake 延迟测试通过，不代表现场真实延迟已定位。软件时间窗口及看门狗不保证物理上即时停步。
- 记录分开保存：运行代码与产物 hash（没有则写无）、模型/URDF/profile、命令与配置、板端 `integration-record/acceptance-*` 的 `summary.json`、`trace.csv`、`node.log`、监测摘要，以及操作员现场观察；不要把累计关节峰值等同训练保护连续新样本触发，也不要把来源归因等同控制权。
- 每项验收最少填写：日期与环境；代码/模型/配置 hash；授权范围及当时现场条件；命令与记录路径；软件观察；现场观察；通过、未通过或未验证；异常和下一步。
- 任何保护触发、状态失效、意外动作、控制权不明、监测或交接异常：停止追加目标和脉冲，按现有 runner/现场急停及人工支撑交接流程处理；保存证据并检查原因，不关闭保护或无改动重试。若部署或软件回退必要，先确认现场处于适合操作的状态，核对本次部署备份、版本与回退范围，再按授权恢复对应文件并核对 hash；不要在机器人保持输出期间覆盖运行文件或自动切换控制模式。
