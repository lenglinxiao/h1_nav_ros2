# H1-2 ONNX 导航 Implementation Plan

> **实施状态（2026-11-07）：** 用户已要求实施，任务 1A 已通过本地验证与审查，当前推进 1B；所有验证仅在本地完成，不连接机器人。任务 1 拆为 1A 类型与纯状态机、1B ROS 协调器/action、1C bridge 输入 GID、1D owner/会话 launch，逐项验证与审查。后续任务仍待办。

**Goal:** 建立受保护的常驻 ONNX 控制与可独立启停的 Nav2 会话，停止导航后保持零速 ArmHold。

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
- 任务 1A：完成本地验证与审查，提交 `00691a01284269266c895a373e87411f8a85fe81` 与 `c729fe232f03ae9964f362ac592cfcb0a904af7b`。纯状态机定向测试 17 项通过；接口包单独构建成功，两个消息和四个服务的 Python 3.10 原生类型支持加载成功。审查发现的 readiness 快照及 source 比较顺序问题已修复并复审通过。任务 1B 进行中，1C–1D 及 2–6 未完成。
- 任务 1A 验证命令：`PYTHONPATH=src/h1_2_nav_tools /usr/bin/python3 -m pytest -q src/h1_2_nav_tools/test/test_policy_motion_coordinator.py`（17 passed）；隔离 `colcon build --packages-select h1_2_policy_interfaces` 固定 `Python3_EXECUTABLE` 和 `PYTHON_EXECUTABLE` 为 `/usr/bin/python3`，原生类型支持逐类检查非空。过程报告与复审记录在隔离副本 `.superpowers/sdd/development-plan/task-1a-{report,review,fix1-review}.md`。未运行机器人或 ROS 图。
- 启动顺序澄清：prepare 在旧会话隔离、零速 barrier 后返回新 namespace，客户端据此启动 Nav2；node/backend readiness 在 activate 时再次检查。若 prepare 先等待尚未启动的 backend，就会形成启动循环。此澄清不放开运动准入。
- 本地验证边界：离线单元测试、FakeIO、本机独立 ROS 域中的假节点；不创建实际 Unitree LowCmd writer，不进行迁移、授权、撤权或切模式。当前未连接机器人，历史现场记录不能证明当前状态。

### 任务 1：常驻 coordinator 与会话隔离

**文件：** 新建 `policy_motion_coordinator.py`、`policy_owner.py`、`policy_nav_session.launch.py`、`tools/h1_2_policy_hardware/systemd/h1-policy-owner.service`、`tools/h1_2_policy_hardware/systemd/h1-policy-owner.env.example`、`src/h1_2_policy_interfaces/`、`src/h1_2_nav_tools/test/test_policy_motion_coordinator.py`；修改 `policy_navigation_gate.py`、`setup.py`、bridge `node.hpp` 和 node integration 测试。

**生产/消费接口：** coordinator 实现 `prepare_source(source: str, expected_token: SessionToken) -> PrepareMotionSource.Response`、`activate_source(token: SessionToken) -> ActivateMotionSource.Response`、`hold() -> Trigger.Response`；详见[设计](design.md)中的字段、namespaces 及锁语义。owner 调用 bridge `claim_input(owner_boot_id, cmd_writer_gid, heartbeat_writer_gid) -> ClaimPolicyInput.Response`；bridge 校验两个 ROS opaque GID 的 `uint8[]` 长度等于运行 RMW_GID_STORAGE_SIZE（Humble 当前为 24），并以 `MessageInfo.publisher_gid` 全量核对每条最终 Twist/Bool；原生 DDS LowCmd GUID 保留 16 字节，不混用。输入会话 topic、前后端 action namespace 由 prepare 返回且不复用。owner 常驻组与 Nav2 会话组分离。

- [ ] 写 `test_hold_wins_activate_and_rejects_goal`、`test_switch_waits_terminal_or_isolation`、`test_old_session_late_samples_and_goals_rejected`、`test_claim_rejects_old_gid_and_clears_command`、`test_hold_heartbeat_enables_zero_ready`、`test_coordinator_restart_never_reuses_session_and_rejects_old_token`、`test_hold_invalidates_inflight_prepare`、`test_later_prepare_invalidates_earlier_prepare`、`test_claim_gid_length_and_truncation_rejected`、`test_owner_survives_ssh_gui_nav_clients_exit`：覆盖竞争、取消终态或组退出/强隔离、捕获旧 token 回调、coordinator 重启后新 boot ID 与旧 token/GID 均被拒、hold 或后继 prepare 抢占旧异步 prepare 后旧结果为 STALE_PREPARE 且不改变 phase 或返回会话接口、无 goal 时 ready、拒绝短/长/截断 ROS GID、SSH/GUI/Nav2 客户端退出后 owner 存活及最终 cmd/heartbeat 各唯一 writer。
- [ ] 在已加载 ROS Humble 的隔离开发环境运行 `python3 -m pytest src/h1_2_nav_tools/test/test_policy_motion_coordinator.py -q`，预期新增测试因实现缺失失败；bridge GID 测试随现有 node integration 运行，预期对应新增断言失败。
- [ ] 实现上述 typed 接口、独立进程组、systemd 用户服务模板、同锁状态机、会话 namespace 和 bridge claimed 模式；服务不得自动重启重新授权，启动保持输出未授权、故障须维护恢复；hold 优先递增 epoch 与 operation_generation；每次接受 prepare 递增并捕获 generation，所有 await 回调及返回前在锁内核对 generation 和完整 token，过期返回 STALE_PREPARE；只有 barrier、清理、node 和后端 ready 后 activate 成功。手动模式在旧 Nav2 组确认退出或旧会话强隔离后可激活，不能确认则 hold。历史 Twist 模式兼容，生产显式 claimed。
- [ ] 在隔离开发 checkout 中配置实际可用的 `ORT_ROOT`、`UNITREE_SDK2_ROOT`、`H1_POLICY_TEST_MODEL_PATH`（路径按运行机架构核对，变量见 bridge README），分别确认 ORT 库、SDK 库和模型可读。运行 `colcon build --packages-select h1_2_policy_interfaces h1_2_policy_bridge h1_2_nav_tools --cmake-args -DBUILD_TESTING=ON -DORT_ROOT="$ORT_ROOT" -DUNITREE_SDK2_ROOT="$UNITREE_SDK2_ROOT" -DH1_POLICY_TEST_MODEL_PATH="$H1_POLICY_TEST_MODEL_PATH"`，预期三个包构建成功；随后 `source install/setup.bash` 并执行 `ros2 interface show h1_2_policy_interfaces/srv/PrepareMotionSource`，预期字段与设计一致。这一步生成 ROSIDL 类型，不启动节点；不得使用正在运行控制器的安装目录。
- [ ] 重跑 pytest，并运行 `colcon test --packages-select h1_2_policy_bridge --ctest-args -R '^node_integration$'` 后 `colcon test-result --verbose`；预期受影响测试通过。`colcon test` 不代替前一步构建。产出常驻控制栈和隔离会话；回退为停用新生产模式，已部署实例仅经安全交接后恢复备份。

### 任务 2：运行实例身份、配置及 launch

**依赖：** 任务 1。**文件：** `policy_owner.py`、bridge `node.hpp`、`src/h1_2_policy_bridge/launch/policy_bridge.launch.py`、`src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py`；测试各包现有 `test_policy_launch.py`、`test_navigation_backend.py`。

**接口：** owner `build_runtime_manifest(deployment_paths: list[str], profile: Mapping[str, Any]) -> dict` 从板端实际文件生成静态 manifest；profile 含 `joint_names: list[str]`、`arrays: dict[str, list[float]]`（各关节数组 27 维）、`source_sha256: str` 与 `profile_sha256: str`；node `PolicyRuntimeIdentity.srv` 返回设计所列 UUID、PID、FQN、域、localhost、路径/hash、profile hash 和动态授权/writer 状态。LowCmd GUID 仅在授权且 writer 创建后由 node status 和现有 GUID 来源采集，携时间与身份绑定，不预填静态 manifest。`verify_runtime_identity(manifest: dict, status: PolicyRuntimeIdentity.Response, env: dict) -> str` 返回 `OK` 或设计的失败码。`policy_node_mode:=existing|launch`，兼容默认 launch、生产显式 existing；existing 校验真实 manifest/status，不创建替代节点。

- [ ] 写 `test_existing_never_spawns_policy`、`test_manifest_binds_running_identity`、`test_guid_absent_before_authorization_then_bound`、`test_domain_mismatch_vs_unreachable`、`test_train_profile_and_hash_mismatch`；断言 runtime policy_pid 不等于 owner 启动子进程 PID、owner_boot_id 不匹配、27 维数组或模型/URDF/产物 hash 不符均拒绝，远端超时只报 UNREACHABLE。
- [ ] 运行 `python3 -m pytest src/h1_2_policy_bridge/test/test_policy_launch.py src/h1_2_slam_bringup/test/test_navigation_backend.py -q`，预期新增案例失败。
- [ ] 实现完整训练数组传参、规范化 profile hash、不可变静态 manifest 分别记录 owner_pid/policy_pid，runtime policy_pid 与 owner 启动子进程的 PID、启动 ID、路径及 profile hash 绑定，并按授权前无 writer、授权后有 writer 分别采集动态证据；在实际可得证据基础上检查域/localhost，冲突或缺失 fail closed。
- [ ] 重跑相同命令预期 PASS；产出部署配置及维护迁移说明。回退为未激活 existing，并仅在安全交接后恢复对应备份。

### 任务 3：shell 与 GUI 后端及会话入口

**依赖：** 任务 2。**文件：** `run_ai_slam_nav.sh`、`src/h1_2_nav_tools/h1_2_nav_tools/h1_2_nav_web_gui.py`、`test/static/test_h1_2_nav_web_gui.py`、`test/unit/test_web_task_profiles.py`。

**接口：** shell `--motion-backend unitree|policy` 默认 unitree；GUI payload `motion_backend` 同值；policy 使用 existing、claimed，先 prepare 并启动返回的 Nav2 会话，activate 成功后 GUI/route 使用返回的 frontend action namespace。禁止全局 action alias；unitree 行为保留。

- [ ] 写 `test_policy_backend_and_session_forwarded`、`test_unitree_default_preserved`、`test_invalid_backend_rejected`；运行 `python3 -m pytest test/static/test_h1_2_nav_web_gui.py test/unit/test_web_task_profiles.py -q`，新增案例预期失败。
- [ ] 实现入口参数和返回 namespace 的传递；禁止自动发 goal，重跑同命令预期 PASS。
- [ ] 产出两个一致的启动入口；回退为 unitree 原默认并恢复备份，不终止运行中策略。

### 任务 4：停止、失联和新目标恢复

**依赖：** 任务 1–3。**文件：** coordinator、GUI、`stop_ai_slam_nav.sh`，测试 `src/h1_2_nav_tools/test/test_policy_motion_coordinator.py`、`test/unit/test_web_heartbeat.py`。

**接口：** GUI stop、stop_all、心跳超时先调用 `/h1_2/policy/hold`，只停止会话组；coordinator 负责取消旧 action，不能确认终态、退出或强隔离时保持 hold。恢复重新 prepare/activate 后由客户端显式提交新目标，不复用 namespace。

- [ ] 写 `test_stop_preserves_owner_process_group`、`test_heartbeat_no_replay`、`test_goal_races_cleanup_is_rejected`；断言 hold 优先、被抢占 prepare 不改 phase 并返回 STALE_PREPARE、旧 goal 被拒、owner 子进程存活、恢复须新会话。
- [ ] 运行 `python3 -m pytest src/h1_2_nav_tools/test/test_policy_motion_coordinator.py test/unit/test_web_heartbeat.py -q`，新增案例预期失败；实现上述顺序后重跑预期 PASS。
- [ ] 完成标准：导航停止与控制退出隔离；回退为禁用 policy 导航入口，不用 stop 脚本杀 owner。

### 任务 5：受验速度包络

**依赖：** 任务 1–4。**文件：** 新建 `src/h1_2_nav_tools/h1_2_nav_tools/policy_speed_envelope.py`、`tools/h1_2_policy_hardware/policy_manual_trial.py`、`src/h1_2_nav_tools/config/nav2_h1_2_policy.yaml`、`src/h1_2_nav_tools/behavior_trees/navigate_to_pose_policy.xml` 和 `src/h1_2_nav_tools/test/test_policy_speed_limits.py`，修改 coordinator 与 policy launch 选择参数；保留共享 unitree YAML 和 BT。

**接口：** `ValidatedEnvelope` 定义在 `policy_speed_envelope.py`，字段为 `vx_min/vx_max/vy_min/vy_max/wz_min/wz_max: float`、`validated: bool`、`evidence_id: str`；从批准的现场补验证据加载，缺证据或值非法时未验证。`validate_command(vx: float, vy: float, wz: float, envelope: ValidatedEnvelope) -> bool`；导航 source 默认不可激活，右转及双向 yaw 现场受控补验后配置 `vx=[0,0.12]`、`vy=0`、`wz=[-0.1,0.1]`。越界拒绝并 hold；沿用 bridge 原有限幅与 guard。

- [ ] 写 `test_nav_disabled_without_validated_envelope`、`test_bidirectional_envelope_boundaries`、`test_out_of_range_holds`、`test_policy_bt_excludes_unvalidated_recovery`、`test_unitree_configuration_unchanged`、`test_manual_trial_fixed_right_and_mock_default`；检查 policy BT 不加载或调用 spin、back_up、drive_on_heading，unitree 配置保持不变；运行 `python3 -m pytest src/h1_2_nav_tools/test/test_policy_speed_limits.py -q`，新增案例预期失败。
- [ ] 实现固定 forward/turn_left/turn_right 工程试验入口（0.12 m/s 或 ±0.1 rad/s，≤3 秒，默认 mock；prepare→activate→单次→hold），允许待验右转仅用于另行现场授权的工程试验，nav source 保持未验禁用；实现显式包络验证及独立 policy YAML/BT，保留原 controller 算法，设置零横移、非倒车，禁用未经验证的倒车和大角度自动恢复；重跑预期 PASS；不声称双向 yaw 已现场验证、不开发新步态算法。
- [ ] 完成标准：未验导航无法激活，已配置范围不被突破。回退为禁用 policy 导航配置，不改变 bridge 保护。

### 任务 6：隔离集成、只读核验与现场验收

**依赖：** 任务 1–5。**文件：** 新建 `src/h1_2_nav_tools/test/test_policy_navigation_integration.py`、`tools/h1_2_policy_hardware/nav_commissioning.py`，更新[测试计划](test-plan.md)与现场任务记录。

**接口：** 仅在独立 ROS 域使用 fake ready/Nav2/policy，不创建实际 LowCmd writer；生产前核对真实 owner manifest/status；真实动作须另有当时有效授权。

- [ ] 写 `test_prepare_activate_then_new_goal`、`test_hold_switching_reject_goals_and_late_old_session`、`test_hold_wins_inflight_activation`、`test_hold_invalidates_inflight_prepare`、`test_coordinator_restart_never_reuses_session_and_rejects_old_token`；依次 prepare、启动会话、activate 成功、显式新 goal、hold、重建新会话、重新激活及新 goal。注入 cleanup 期间 goal、hold 或后继 prepare 抢占、重启后旧 token、旧速度/goal、ready 丢失及竞争，验证全部旧请求拒绝。
- [ ] 在已加载 ROS Humble 的隔离环境运行 `ROS_DOMAIN_ID=201 ROS_LOCALHOST_ONLY=1 python3 -m pytest src/h1_2_nav_tools/test/test_policy_navigation_integration.py -q`；新测试先失败，实现 mock 后 PASS。复用未受影响的历史双 action 测试。
- [ ] 准备板端只读清单：实际进程/UUID/LowCmd GUID、状态新鲜度、profile/模型/URDF/产物 hash、ROS 域和 topic；不启动第二节点或覆盖运行文件，任何不符暂停。
- [ ] 实现 `nav_commissioning.run_trial(action_namespace: str, goal: PoseStamped, max_seconds: float, mode: str = "mock") -> TrialResult`：默认只连接 mock；live 需显式选择并限定首次窗口 ≤3 秒，到时请求 cancel/hold 并记录归零，未完成取消须报告且保持 hold。测试 `test_mock_trial_cancels_by_deadline` 和 `test_live_requires_explicit_opt_in`；运行 `python3 -m pytest src/h1_2_nav_tools/test/test_policy_navigation_integration.py -q` 预期 PASS。live 不在自动测试执行；未来现场命令形式为 `python3 tools/h1_2_policy_hardware/nav_commissioning.py --mode live --action-namespace <prepare返回的frontend> --goal-x <米> --goal-y <米> --goal-yaw <弧度> --max-seconds 3`，只有当时明确授权、现场条件及双向 yaw 前置通过后人工执行，目标坐标由现场确定。
- [ ] 单独规划并在有效现场授权下先补验右转与双向 yaw。通过后首次 Nav2 只做 ≤3 秒命令链短测、定时取消归零，不要求到达；后续目标到达验收须新的明确时长和范围授权。记录目标、定位、命令、ready/guard、现场运动与停稳；3 秒不是永久生产门限。现场未执行则标未验证。
- [ ] 完成标准：mock、只读和现场分别有证据及结论；异常停止新目标并按现场控制和急停流程交接，回退仅在安全交接和有效授权后进行。
