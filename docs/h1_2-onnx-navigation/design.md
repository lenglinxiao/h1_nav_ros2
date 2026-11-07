# ONNX 导航接入设计提案

状态：2026-11-07，主代理建议，待用户审阅；尚未实现，也不构成现场迁移或运动授权。范围见[总计划](plan.md)，任务见[开发计划](development-plan.md)。

## 方案选择

方案 A（建议）：独立常驻 `PolicyMotionCoordinator` 拥有速度仲裁、目标准入与会话 epoch，复用现有 gate 的双 action 代理和清理逻辑；policy bridge 保持运动控制与 guard，但新增对输入发布者身份的校验。Nav2 会话可单独重建。方案 B：把仲裁和 action 准入全部嵌入 C++ policy bridge，减少跨进程接口，但将 Nav2 生命周期与安全敏感控制循环耦合。首版建议 A，尚未获用户批准；不新建通用调度框架。

## 生命周期和接口

未来维护窗口安装 `tools/h1_2_policy_hardware/systemd/h1-policy-owner.service` 及同目录 `h1-policy-owner.env.example`，由 systemd --user 托管 `tools/h1_2_policy_hardware/policy_owner.py`，使其独立于 SSH、PTY、GUI 与 Nav2。service 不自动重启并重新授权输出；owner 启动时保持输出未授权，需人工准备，故障退出由明确的维护流程恢复。本次不安装或修改服务。独立常驻进程组由 owner 启动和监护：policy bridge、coordinator、已有 monitor 属于 owner 组；新文件 `src/h1_2_nav_tools/launch/policy_nav_session.launch.py` 只启动可独立停止的 Nav2 会话进程组。GUI、watchdog、`stop_ai_slam_nav.sh` 只请求 coordinator hold 并停止会话组，不能终止 owner 组；真正的控制退出仍走原独立人工授权和受保护流程。owner 故障沿已有监护与保护路径处理，不用 Nav2 自动重启来接管。

以下是拟议接口，具体 ROS 类型统一放新包 `h1_2_policy_interfaces`：

- `SessionToken.msg`：`string owner_boot_id`、`string coordinator_boot_id`、`uint64 epoch`。
- `PrepareMotionSource.srv`：请求 `string source`（`manual|nav`）、`SessionToken expected_token`；响应 `bool success`、`SessionToken token`、`string cmd_topic`、`string action_namespace`、`string reason`。
- `ActivateMotionSource.srv`：请求 `SessionToken token`；响应 `bool success`、`string reason`。
- `/h1_2/policy/hold`：`std_srvs/srv/Trigger`，立即进入 hold、零速、递增 epoch 并使异步操作失效；它优先于 prepare/activate 和 goal 准入。
- `/h1_2/policy/source_state`：`PolicySourceState.msg`，含 `SessionToken token`、`string state`（`hold|switching|manual|nav`）、`string reason`。以上服务分别位于 `/h1_2/policy/prepare_source`、`/h1_2/policy/activate_source` 和上述绝对路径。客户端从 source_state 获得完整 token 后才可 prepare，不能以单独的 epoch 作为 CAS。

owner 每次启动生成新 `owner_boot_id=uuid4().hex`；coordinator 每次启动或 respawn 均生成新 `coordinator_boot_id=uuid4().hex`，绝不从 manifest 复用。coordinator 启动先 hold、清空输入和目标缓存，创建新发布器并由 owner 完成新 GID claim；不恢复旧 source 或 goal。epoch 仅在该 coordinator 生命周期内单调递增；所有 prepare CAS 和 activate 必须比对 owner、coordinator 两个 boot ID 与 epoch 的完整 token，数字相同的旧 token 也拒绝。

coordinator 使用同一状态锁处理完整 token、`operation_generation` 和 goal 准入。匹配 CAS 的 prepare 在锁内递增 epoch 与 operation_generation，捕获新 generation 和完整 token，封锁 goal 与输入、输出零并清除旧缓存，然后异步取消两个旧 action 并等待已知目标终态。hold 和每次被接受的新 prepare 均递增 operation_generation，使旧操作失效。每个 await 回调和服务返回之前必须在同一锁内重比 generation 与完整 token；失效的 prepare 不得修改 source/phase、返回可用 topic 或 namespace，返回 `STALE_PREPARE`。若终态不可确认，只在旧 Nav2 受控进程组已确认退出或旧会话已强隔离时可继续，否则保持 hold 并报错。prepare 成功返回新会话接口时仍为 switching，拒绝 goal 和速度；activate 仅在完整 token 匹配、零速 barrier 已执行、清理完成、node ready 及所选后端 ready 成立时成功。hold 抢占后旧 activate 必须失败。Nav2 不运行时，已确认旧组退出或已强隔离即可为 manual 建立新会话，不依赖已消失的清理服务。物理停稳须单独现场确认。

每次会话使用当前 owner/coordinator 各自启动时生成的两个 `uuid4().hex` 和十进制 epoch 构造永不复用的 `/h1_2/policy/session/o_<owner_boot_id>/c_<coordinator_boot_id>/e_<epoch>/cmd_vel`、`.../frontend` 和 `.../backend`；前者为 `geometry_msgs/msg/Twist`，双 action 前后端均映射到会话命名空间。coordinator 不再订阅旧 topic，回调闭包捕获创建时完整 token，旧 velocity 和 goal 到达时再次在锁内拒绝。Nav2 smoother 和 action 后端由会话 launch 按 prepare 返回值 remap，GUI/route 客户端使用返回的 frontend namespace；禁止全局 action alias 将旧请求转至新会话。健康会话内多个新目标无需重建；失效恢复需要新会话，Nav2 会话进程组可重建，但 policy/ArmHold 不重启。这个方案不承诺原地热接当前私有实例。

coordinator 唯一发布最终 `/h1_2/policy/cmd_vel` 与 `/h1_2/policy_gate_alive`。硬件 node 的 ready 依赖心跳新鲜；hold 健康时即使无 goal/速度，coordinator 仍每 20 ms 发布 true，允许零速 ready，不能等待 ready 或非零速度再发心跳。ready 丢失立即 hold 并清除输入，不关闭 policy 平衡输出；重获 ready 不恢复旧源和目标。原 gate 的 cleanup_complete 逻辑合并进 coordinator，不再有第二个最终心跳 publisher。

coordinator 重启后，bridge 生产模式启用输入身份校验：新增 `/h1_2/policy/claim_input`（`ClaimPolicyInput.srv`，请求 `string owner_boot_id`、`uint8[] cmd_writer_gid`、`uint8[] heartbeat_writer_gid`；响应 `bool success`、`string reason`）。claim 仅在本机 owner 启动及身份核验后生效，只租用输入，不授权 LowCmd、不切模式；claim 成功立即清零命令缓存，接收端校验两个数组长度均等于运行时 `RMW_GID_STORAGE_SIZE` 并全量比较，拒绝截断或长度错误；ROS Humble 当前该常量为 24。bridge 对两种消息用 ROS `MessageInfo.publisher_gid` 逐帧验证，丢弃旧 GID，保持 0.4 秒速度超时。该 ROS opaque GID 与原生 DDS LowCmd 的 16 字节 GUID 不同，后者保留 `uint8[16] lowcmd_writer_gid`。coordinator 先创建发布器并向 owner 提供其 GID，owner 完成 claim 后 coordinator 再开始心跳；未 claim 或失联无非零输入。旧 Twist 接口作为兼容模式保留，生产显式 `input_identity_mode:=claimed`。claim 的访问控制以本地可信 owner 及现有授权边界为前提，不声称防御恶意网络参与者。

## 部署与证据

owner 启动时从板端实际部署文件生成不可变的静态运行 manifest：`owner_boot_id`、`owner_pid`、`policy_pid`、node FQN、ROS_DOMAIN_ID、ROS_LOCALHOST_ONLY、模型/URDF/可执行文件实际路径及 SHA256、完整训练 profile 的规范化数组 SHA256。LowCmd writer GUID 只有授权并创建发布器后才出现，不属于启动时静态 manifest；owner 通过现有 node status 和 writer GUID 来源记录采集动态 writer 证据，附带 owner_boot_id、policy_pid、采集时间和授权状态，与静态 manifest 绑定。新 `PolicyRuntimeIdentity.srv` 返回上述 typed 静态字段及动态 `bool authorized`、`bool writer_present`、`uint8[16] lowcmd_writer_gid`、`bool valid`、`string reason`；未授权时 GUID 明确标记不存在，不能预填。runtime 报告的 policy_pid 必须等于 owner 实际启动的 policy 子进程 PID；owner_pid 单独记录，不要求两者相同。owner_boot_id、路径和 profile hash 按实际证据绑定。`policy_node_mode:=existing` 从运行实例 manifest 和 runtime status 读取比较，并在导航准入时检查已授权 writer 动态证据及身份唯一性；不是把 PC 期望文件当运行事实。先比本机环境和已取回的部署 manifest，明确证实才报 `DOMAIN_MISMATCH` 或 `LOCALHOST_MISMATCH`；远端服务超时仅报 `UNREACHABLE`。还需明确 `IDENTITY_MISMATCH`、`PROFILE_MISMATCH`、`HASH_MISMATCH`、`WRITER_CONFLICT`。缺失字段、节点不可达或冲突均失败关闭。`launch` 模式保持兼容且不得产生重复 writer，生产使用 `existing`。
当前人工 runner 使用 ROS_DOMAIN_ID=43、ROS_LOCALHOST_ONLY=1 和 PID 私有 topic；生产入口默认域 0、允许网络通信。PTY27864 最近记录为 ArmHold idle，用户确认无外部悬吊/防摔支撑下独立站稳，现场易变需复核。未来迁移须当时有效授权、可靠安全支撑和独立维护窗口，先受控交接并确认旧实例结束再启动新栈；释放/撤权仍是独立未解决事项。

## 速度与验收边界

已观察一次直行 `vx=0.12 m/s` 3 秒和一次原地左转 `wz=+0.1 rad/s` 3 秒；同 ArmHold 重复 WALK 与独立站稳为用户现场观察。右转、双向纠偏、长时及真实 Nav2 未验。首版提供显式 validated envelope，默认不允许激活导航 source；先另行获得现场授权完成右转/双向 yaw 受控补验，再允许 `vx=[0,0.12] m/s`、`vy=0`、`wz=[-0.1,+0.1] rad/s` 的首期导航。超范围拒绝并 hold；不得以只允许正 yaw 就宣称可完成 Nav2 闭环。不扩展原 bridge 限幅或开发新步态算法。

首期 policy 使用独立 `src/h1_2_nav_tools/config/nav2_h1_2_policy.yaml` 和 `src/h1_2_nav_tools/behavior_trees/navigate_to_pose_policy.xml`，保留现有 controller 算法，设置速度包络、零横移与非倒车；不加载或调用未经验证的 spin、back_up、drive_on_heading 自动恢复，现有 unitree 参数及 BT 保持原样。首次实机 Nav2 仅做不超过 3 秒命令链短测：由受控 driver 到时取消并归零，不要求到达；后续目标到达验收须另有明确时长和范围授权。3 秒是首次验收范围，不是永久生产门限。

`ValidatedEnvelope` 定义于 `src/h1_2_nav_tools/h1_2_nav_tools/policy_speed_envelope.py`：`vx_min: float`、`vx_max: float`、`vy_min: float`、`vy_max: float`、`wz_min: float`、`wz_max: float`、`validated: bool`、`evidence_id: str`；validated 只从明确批准的现场补验证据加载，缺失证据或值非法时保持 false。导航 source 依此门控；工程手动试验与导航授权分离。

新增 `tools/h1_2_policy_hardware/policy_manual_trial.py`，提供 `forward`、`turn_left`、`turn_right` 固定选项（直行 `vx=0.12`，转向 `wz=+0.1/-0.1`，其余分量为零，单次不超过 3 秒），由 prepare → activate → 单次窗口 → hold 执行；每次现场试验必须另有明确授权、复核现场与保护，默认 mock。待验右转仅在此人工授权的工程试验入口使用，不解锁未验证的 nav source。当前不运行、不安装、不迁移。
