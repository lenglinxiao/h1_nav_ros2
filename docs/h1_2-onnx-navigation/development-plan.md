# H1-2 ONNX 导航历史实施记录

> **方案已被取代（2026-11-12）：** 本文以下任务拆分描述的是曾采用的 Owner/Coordinator/session 方案，相关运行时已按用户要求从当前代码删除。当前架构和剩余验收以[总计划](plan.md)顶部“架构收敛决定/简化实现进度”为准；不要按本文旧任务恢复这些接口。

> **实施状态（2026-11-11）：** 任务 1A–1D 和任务 2 的 owner/runtime identity 已完成首轮验证。任务 3/4 已接上 GUI、启动、动态 action namespace 与 hold-first 停止；任务 5 已加 fail-closed 速度包络、Policy 专属 Nav2 参数和人工脉冲选项；任务 6 已加入 mock-first commissioning CLI、fake ROS lifecycle 测试与[板端核对清单](hardware-validation-checklist.md)。在线 camera_init Policy 接入已部署到PC4独立RC5；构建、启动参数、离线验收通过。雷达开启时实测点云进入FAST-LIO，动态栅格连续发布`/map`；`/slam_odom`/TF和长稳仍待确认。未启动Owner/Nav2/运控；LowCmd writer身份和速度包络仍未验收。证据见[RC5记录](rc5-online-mapping-delivery.md)。

**Goal:** 建立受保护的常驻 ONNX 控制与可独立启停的 Nav2 会话，启动时选择 Unitree 或 ONNX 后端，停止导航后保持零速 ArmHold。运行期间不切换控制器。

**Architecture:** 独立 owner 启动并监护 policy、coordinator 和 monitor；coordinator 在同一锁内管理输入仲裁、双 action 准入和完整会话 token。bridge 生产模式按 GID 校验唯一最终命令及心跳，GUI/导航只控制会话进程组。

**Tech Stack:** ROS 2 Humble、rclpy、C++、Nav2、launch、Bash、pytest。

**Spec:** [设计提案](design.md)、[总计划](plan.md)、[测试计划](test-plan.md)。用户已授权本地实施；现场迁移与动作另行验收。接续见[开发任务](../../tasks/20261107-onnx-navigation-development.md)。

## Global Constraints

- 不触碰现有 PTY27864，不发送 WALK、撤权、release、切模式或关闭底板进程；本计划不授权新动作。现场状态须复核。
- 保留输出授权、急停、CRC/状态、逐关节位置/速度、倾斜保护及 0.4 秒速度超时；训练入口角差仅诊断不等于运行期保护关闭。
- 策略 LowCmd、最终速度和心跳各保持唯一 writer；识别来源而不杀未知发布者。已通过且代码和条件不变的测试不重复。
- 当前 ROS43 localhost 私有实例不能假定热接；迁移须当时有效授权、可靠安全支撑及受控交接，不覆盖运行文件。模拟、只读、实机证据分开。

## Review Focus

1. 同时发布或重复 writer：输入不串源，冲突 hold（任务 1）。
2. 域/localhost 不匹配或不可达：有证据才报不匹配，超时仅报不可达（任务 2）。
3. ready 丢失与恢复：旧 goal/速度不重播，保留 policy 平衡输出（任务 1、4）。
4. stop_all/心跳丢失：不能杀 policy、coordinator 或 monitor（任务 4）。
5. 训练配置或运行文件漂移：绑定运行实例核验并拒绝（任务 2）。

## 文件职责与依赖

- 新建 `src/h1_2_policy_interfaces/`：本计划列出的 typed ROS 消息与服务。
- 新建 `src/h1_2_nav_tools/h1_2_nav_tools/policy_motion_coordinator.py`：会话仲裁、目标代理、心跳；复用 `policy_navigation_gate.py` 的 action 清理逻辑，在 `setup.py` 注册入口。
- 新建 `tools/h1_2_policy_hardware/policy_owner.py`：启动与监护常驻 policy/coordinator/monitor 的独立进程组、生成运行 manifest；新建 `tools/h1_2_policy_hardware/systemd/h1-policy-owner.service` 和 `tools/h1_2_policy_hardware/systemd/h1-policy-owner.env.example`：未来维护窗口托管 owner，不安装当前服务、不自动重启并重授权；新建 `src/h1_2_nav_tools/launch/policy_nav_session.launch.py`：仅负责 Nav2 会话进程组和命名空间 remap。
- 修改 `src/h1_2_policy_bridge/include/h1_2_policy_bridge/node.hpp` 及必要实现：claimed 输入 GID、runtime identity；修改 bridge 和导航 launch、shell/GUI、Nav2 配置和各自测试。
- 依赖顺序：任务 1 → 2 → 3 → 4 → 5 → 6；3、4 可在 2 接口固定后分别开发。每项独立审查并按项目约定本地提交；新的开发提交尚未获额外推送指令。

## 实施记录

- 基线：`65f8adae590dd35508fbeb24265a8e656637e062`；主工作区在本轮开始时干净，本地 `origin/main` 同基线。前轮已记录推送成功，本轮未重新联网推送。
- 隔离目录：项目内 `.codex_tmp/onnx-navigation-dev`，分支 `feat/onnx-navigation`。旧交接所列 `/tmp` 开发目录与相关实现提交在本轮环境中不可用，不将其测试结果当作当前证据。改用项目内隔离副本保存实现，主目录暂不覆盖。
- 任务 1A：完成本地验证与审查，提交 `00691a01284269266c895a373e87411f8a85fe81` 与 `c729fe232f03ae9964f362ac592cfcb0a904af7b`。纯状态机定向测试 17 项通过；接口包单独构建成功，两个消息和四个服务的 Python 3.10 原生类型支持加载成功。审查发现的 readiness 快照及 source 比较顺序问题已修复并复审通过。任务 1B 后续完成；1C–1D 及 2–6 未完成。
- 任务 1B：ROS coordinator、双 action proxy 和本地假节点测试已完成，24 项定向测试通过；`colcon build --packages-up-to h1_2_nav_tools` 5 包构建通过。详细命令与边界见隔离副本 `.superpowers/sdd/development-plan/task-1b-report.md`。
- 任务 1C：bridge claimed GID 输入认领已在隔离分支完成本地验证；当前整合保留 owner ID、完整非零 GID 检查及新旧 writer 过滤测试。整合后的 bridge 构建和 CTest 待复验。
- 任务 1A 验证命令：`PYTHONPATH=src/h1_2_nav_tools /usr/bin/python3 -m pytest -q src/h1_2_nav_tools/test/test_policy_motion_coordinator.py`（17 passed）；隔离 `colcon build --packages-select h1_2_policy_interfaces` 固定 `Python3_EXECUTABLE` 和 `PYTHON_EXECUTABLE` 为 `/usr/bin/python3`，原生类型支持逐类检查非空。过程报告与复审记录在隔离副本 `.superpowers/sdd/development-plan/task-1a-{report,review,fix1-review}.md`。未运行机器人或 ROS 图。
- 启动顺序澄清：prepare 在旧会话隔离、零速 barrier 后返回新 namespace，客户端据此启动 Nav2；node/backend readiness 在 activate 时再次检查。若 prepare 先等待尚未启动的 backend，就会形成启动循环。此澄清不放开运动准入。
- 本地验证边界：离线单元测试、FakeIO、本机独立 ROS 域中的假节点；不创建实际 Unitree LowCmd writer，不进行迁移、授权、撤权或切模式。当前未连接机器人，历史现场记录不能证明当前状态。

### 任务 1：常驻 coordinator 与会话隔离

**文件：** 新建 `policy_motion_coordinator.py`、`policy_owner.py`、`policy_nav_session.launch.py`、`tools/h1_2_policy_hardware/systemd/h1-policy-owner.service`、`tools/h1_2_policy_hardware/systemd/h1-policy-owner.env.example`、`src/h1_2_policy_interfaces/`、`src/h1_2_nav_tools/test/test_policy_motion_coordinator.py`；修改 `policy_navigation_gate.py`、`setup.py`、bridge `node.hpp` 和 node integration 测试。

**生产/消费接口：** coordinator 实现 `prepare_source(source: str, expected_token: SessionToken) -> PrepareMotionSource.Response`、`activate_source(token: SessionToken) -> ActivateMotionSource.Response`、`hold() -> Trigger.Response`；详见[设计](design.md)中的字段、namespaces 及锁语义。owner 调用 bridge `claim_input(owner_boot_id, cmd_writer_gid, heartbeat_writer_gid) -> ClaimPolicyInput.Response`；bridge 校验两个 ROS opaque GID 的 `uint8[]` 长度等于运行 RMW_GID_STORAGE_SIZE（Humble 当前为 24），并以 `MessageInfo.publisher_gid` 全量核对每条最终 Twist/Bool；原生 DDS LowCmd GUID 保留 16 字节，不混用。输入会话 topic、前后端 action namespace 由 prepare 返回且不复用。owner 常驻组与 Nav2 会话组分离。

**当前进度（2026-11-09）：** 1A–1D 与 owner/runtime identity 的实现和前序验证已整合。nav launch 接入 `policy_nav_session_manager`：prepare 后按 coordinator 返回的 token/topic 启动独立 `policy_nav_session.launch.py`，activate 后 route/action client 使用 token namespace；旧综合 launch 不再为 Policy 模式启动 bridge/gate。新增单个 fake ROS 生命周期测试，覆盖 coordinator prepare、fake Nav2 readiness、activate、通过 token action namespace 完成一个 goal，以及 hold 后确认 session 子进程退出；同时修复 `use_sim_time` 重复声明和子进程仍存活却标记 stopped 的问题。owner 可选读取 `H1_POLICY_OWNER_NAV_ENVELOPE_FILE`，无文件时保持 nav 禁用。未运行真实 owner、Nav2 或控制器；生产配置、owner 安装与目标板路径尚未核验。

- [x] 完成 coordinator 状态机、typed 接口、独立 owner/session 进程边界、完整会话 token、claimed GID、prepare/activate/hold 和 fail-closed 启动实现；系统服务不自动重启，输出初始关闭，故障不自动恢复授权。
- [x] 已有必要定向覆盖：hold 与旧 token、prepare generation 竞争、重启 token、readiness barrier、迟到 goal/ready-loss、owner identity 与唯一/完整 GID；桥接 node integration 覆盖错误长度、旧 writer 和新 writer。验证证据见本任务记录和各包测试。
- [x] 当前可用依赖环境构建了 `h1_2_nav_tools`、`h1_2_slam_bringup`；桥接及接口包的先前隔离构建/CTest 证据见本任务记录。构建不触发机器人节点。
- [x] coordinator + session manager + fake Nav2 action 的最小进程生命周期联测通过（1 passed）；真实 persistent owner 未启动。
- [x] 真实 owner CLI/ProcessSupervisor + fake policy runtime identity/claim + 真实 coordinator 子进程生命周期联测通过（1 passed）：输出状态 active 时收到 SIGTERM 仍保持 owner；输出关闭后 owner 有序退出0。fake runtime 不创建 LowCmd publisher。
- [ ] owner/session manager/Nav2 的合并进程联测；目前 owner 生命周期与 session manager/fake Nav2 生命周期分开验证。目标板依赖路径和部署配置尚未核实。

### 任务 2：运行实例身份、配置及 launch

**依赖：** 任务 1。**文件：** `policy_owner.py`、bridge `node.hpp`、`src/h1_2_policy_bridge/launch/policy_bridge.launch.py`、`src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py`；测试各包现有 `test_policy_launch.py`、`test_navigation_backend.py`。

**接口：** owner `build_runtime_manifest(deployment_paths: list[str], profile: Mapping[str, Any]) -> dict` 从板端实际文件生成静态 manifest；profile 含 `joint_names: list[str]`、`arrays: dict[str, list[float]]`（各关节数组 27 维）、`source_sha256: str` 与 `profile_sha256: str`；node `PolicyRuntimeIdentity.srv` 返回设计所列 UUID、PID、FQN、域、localhost、路径/hash、profile hash 和动态授权/writer 状态。LowCmd GUID 仅在授权且 writer 创建后由 node status 和现有 GUID 来源采集，携时间与身份绑定，不预填静态 manifest。`verify_runtime_identity(manifest: dict, status: PolicyRuntimeIdentity.Response, env: dict) -> str` 返回 `OK` 或设计的失败码。`policy_node_mode:=existing|launch`，兼容默认 launch、生产显式 existing；existing 校验真实 manifest/status，不创建替代节点。

- [x] 实现静态 manifest、完整训练数组/profile hash、owner 与 policy PID、文件 hash 及 ROS 域/localhost runtime identity 检查；启动时要求输出未授权且无 LowCmd writer，身份不符 fail closed。
- [x] owner/coordinator/bridge 定向验证已在前序记录完成；此次追加 owner envelope 文件作为可选 coordinator 参数，路径不存在即拒绝配置，未配置时 nav 默认关闭。
- [x] 本地 fake-service owner graph：owner 读取 runtime identity，校验 manifest/profile/file hash/domain/授权状态并 claim coordinator endpoints；真实 coordinator 接受唯一 publisher GID。另有真实 owner supervisor inactive-only shutdown 联测。
- [ ] 部署配置 review 和目标板路径核对。PC4 RC4仅做独立 release 准备，不覆盖主工作区/现有 release；Owner 安装和启动、Policy/Nav2 启动均待执行。不得以 mock/离线验证声称实机通过。

### 任务 3：shell 与 GUI 后端及会话入口

**依赖：** 任务 2。**文件：** `run_ai_slam_nav.sh`、`src/h1_2_nav_tools/h1_2_nav_tools/h1_2_nav_web_gui.py`、`test/static/test_h1_2_nav_web_gui.py`、`test/unit/test_web_task_profiles.py`。

**接口：** shell 和 GUI 的 online/offline 导航均支持 `motion_backend=unitree|policy`，默认 unitree；后端只在启动时选择。Policy 通过常驻 owner/coordinator prepare 独立 Nav2 session，activate 后 GUI/route 使用 coordinator 返回的 frontend action namespace，不创建第二个 bridge、不改全局 action alias。在线 camera_init 保留 FAST-LIO、点云转动态占据栅格、TF/里程计和 Nav2 参数；只将 Nav2 动作会话与速度输出接到 Policy。Unitree 路径不变。

**进度（2026-11-10）：** 在线 GUI 和 shell 已接受启动时 `motion_backend=policy`，camera-init launch 条件切换到 session manager/Nav2 参数，保持 FAST-LIO 和动态地图节点；Policy 分支不启动 Unitree smoother/bridge。Nav2 goal 与路线节点按 coordinator state 使用 token action namespace。Unitree 默认分支保留。定向入口/launch 测试通过；PC4 GUI 尚未切换到 RC4，也未实机启动。

- [x] GUI默认 Unitree；online/offline均转发后端选择；Policy 不传 Unitree gesture/turn route，shell保留Unitree默认路径。
- [x] policy session 使用 coordinator 返回的 topic/action namespace；无全局 action alias，不自动发送 goal。
- [x] manager prepare/activate 与 coordinator token action goal 的隔离 ROS 联测通过；完整 GUI/shell route 流程和目标板现场核对仍待单独进行。owner 配置边界和板端只读步骤见[核对清单](hardware-validation-checklist.md)。

### 任务 4：停止、失联和新目标恢复

**依赖：** 任务 1–3。**文件：** coordinator、GUI、`stop_ai_slam_nav.sh`，测试 `src/h1_2_nav_tools/test/test_policy_motion_coordinator.py`、`test/unit/test_web_heartbeat.py`。

**接口：** GUI stop、stop_all、心跳超时先调用 `/h1_2/policy/hold`，只停止会话组；coordinator 负责取消旧 action，不能确认终态、退出或强隔离时保持 hold。恢复重新 prepare/activate 后由客户端显式提交新目标，不复用 namespace。

- [x] Policy stop/watchdog/GUI shutdown 调用 hold-first stop script；确认 hold 失败时保留 session，不发 Unitree UDP stop，也不由 GUI 强杀 policy 导航进程组。常驻 owner 不在导航进程组内。
- [x] coordinator ready-loss/hold/迟到 goal 竞态已有前序测试；本次 stop 路由 shell 语法检查通过。
- [ ] 增加最小 fake owner/session stop 联测，验证 hold 失败保留 session、成功后仅退出会话且 owner 子进程存活；当前尚未实测整条运行图。

### 任务 5：受验速度包络

**依赖：** 任务 1–4。**文件：** 新建 `src/h1_2_nav_tools/h1_2_nav_tools/policy_speed_envelope.py`、`tools/h1_2_policy_hardware/policy_manual_trial.py`、`src/h1_2_nav_tools/config/nav2_h1_2_policy.yaml`、`src/h1_2_nav_tools/behavior_trees/navigate_to_pose_policy.xml` 和 `src/h1_2_nav_tools/test/test_policy_speed_limits.py`，修改 coordinator 与 policy launch 选择参数；保留共享 unitree YAML 和 BT。

**接口：** `ValidatedEnvelope` 定义在 `policy_speed_envelope.py`，字段为 `vx_min/vx_max/vy_min/vy_max/wz_min/wz_max: float`、`validated: bool`、`evidence_id: str`；从批准的现场补验证据加载，缺证据或值非法时未验证。`validate_command(vx: float, vy: float, wz: float, envelope: ValidatedEnvelope) -> bool`；导航 source 默认不可激活，右转及双向 yaw 现场受控补验后配置 `vx=[0,0.12]`、`vy=0`、`wz=[-0.1,0.1]`。越界拒绝并 hold；沿用 bridge 原有限幅与 guard。

- [x] 加入 fail-closed envelope loader/runtime bounds，未验证命令不能激活；初始硬上限 `vx<=0.12`、`vy=0`、`|wz|<=0.1`，越界输入触发 hold。
- [x] 受支撑人工脉冲入口新增固定 `turn_right=(0,0,-0.1)`，沿用 0.5–3 秒、ready/telemetry/撤权门控；只增加代码选项，未执行现场动作。
- [x] 每个 policy session 从当前 Nav2 参数派生独立临时配置，指定无 Spin/BackUp/drive-on-heading 的已有 BT、将现存 DWB/RPP 速度参数及 velocity_smoother 裁进 envelope，并在 launch shutdown 清理临时文件；Unitree 参数文件未改。
- [ ] 完成现场人工短脉冲后，保存批准 envelope evidence；没有证据时保持 nav disabled。右转和双向 yaw 仍未实机验收。

### 任务 6：隔离集成、只读核验与现场验收

**依赖：** 任务 1–5。**文件：** 新建 `src/h1_2_nav_tools/test/test_policy_navigation_integration.py`、`tools/h1_2_policy_hardware/nav_commissioning.py`，更新[测试计划](test-plan.md)与现场任务记录。

**接口：** 仅在独立 ROS 域使用 fake ready/Nav2/policy，不创建实际 LowCmd writer；生产前核对真实 owner manifest/status；真实动作须另有当时有效授权。

- [x] 增加并运行最小 fake graph：coordinator prepare→session manager 启动 fake Nav2 action server→activate→token namespace 显式 goal→hold→确认 session child 退出；1 passed。ready-loss、hold 竞争和旧会话拒绝由已有 coordinator 测试覆盖；尚未覆盖新 token 重开和真实 owner 进程。
- [x] 准备板端只读清单：实际进程/UUID/LowCmd GUID、状态新鲜度、profile/模型/URDF/产物 hash、ROS 域和 topic；不启动第二节点或覆盖运行文件，任何不符暂停。清单见[hardware-validation-checklist.md](hardware-validation-checklist.md)，尚未在目标板执行。
- [x] 增加 `tools/h1_2_policy_hardware/nav_commissioning.py`：默认 mock 不导入 ROS、不发 goal；live 显式选择 coordinator frontend，限时 ≤3 秒，到时取消并请求 hold，分别记录取消和 hold 确认。
- [x] 独立 ROS 域验证真实 owner/coordinator 生命周期与 fake runtime；独立 fake Nav2 session 测试覆盖 prepare/activate/goal/hold/child exit。合并同进程回归9项通过；完整 owner/session/Nav2 同一进程图尚未合并联测。
- [ ] 单独规划并在有效现场授权下先补验右转与双向 yaw。通过后首次 Nav2 只做 ≤3 秒命令链短测、定时取消归零，不要求到达；后续目标到达验收须新的明确时长和范围授权。记录目标、定位、命令、ready/guard、现场运动与停稳；3 秒不是永久生产门限。现场未执行则标未验证。
- [ ] 完成标准：mock、只读和现场分别有证据及结论；异常停止新目标并按现场控制和急停流程交接，回退仅在安全交接和有效授权后进行。
