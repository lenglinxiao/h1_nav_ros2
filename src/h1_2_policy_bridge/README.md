# H1-2 policy bridge

The default profile remains isolated simulation: Unitree DDS `lo`, domain `42`, `mode_machine=0`, `mode_pr=0`. The node requires ROS `rmw_fastrtps_cpp`, selected by this launch; the Unitree domain parameter does not set `ROS_DOMAIN_ID`. Startup does not publish motor commands. The hardware profile is initially read-only and creates no LowCmd publisher until explicitly authorized. See [hardware adaptation](../../docs/h1_2-policy-hardware-adaptation.md) for its contract, observations and outstanding verification.

The model default remains `/home/llx/policy.onnx` (MD5 `b27c034920b671ffb33c920a58798d0a`). The policy and parameters follow the supplied 2026-10-29 H1_2 ArmHold guide. Original robot encoder offsets are retained. The guide calls wrist order pitch, roll while SDK/MJCF order is roll, pitch; explicit name mapping implements that assumption, but actual training order is unverified. Numerical regression cannot validate physical joint semantics.

The startup-only `joint_command_mode` defaults to `torque`. The optional `position_pd` mode derives native PD fields from the same controller, with the hardware projection and transitions described below, and is available on hardware only through the existing output authorization and state gates. Motor-side PD is not equivalent to the existing software-clipped torque: the software torque clamp does not constrain motor-side PD between frames, real-time torque limits are unverified, and the successful deployment configuration has not been established. Hardware Passive remains native damping. Simulated behavior does not validate hardware safety or establish that prior behavior was reproduced.

Hardware `position_pd` projects each positive-kp target against the latest sampled state: it clips `kp*(q_target-q)-kd*dq` to the existing joint torque limits and solves for the corresponding position target. Unsaturated targets are unchanged. This constrains the ideal torque at that sample only; communication delay, motion between frames, firmware behavior and zero-kp native damping remain outside this constraint. It is not a firmware torque limiter or an absolute joint-angle clamp. The raw ONNX observation, action and action history are unchanged. Status reports this separately as `position_pd_sample_torque_projection`; `software_torque_clamp` remains false for native PD. Limited frames are counted and logged.

Normal hardware position-PD preparation and policy entry blend from the last successfully written target and gains over 3 seconds. Preparation retains its internal position interpolation, so intermediate trajectories change while the final preparation target is retained. A subsequent stage must wait for `output_transition=none`.

Normal `passive` requests and policy-budget exhaustion now enter `Hold`, retaining the last successfully written desired target and gains rather than automatically reducing support gains. No further policy inference runs; the target is still projected against each fresh state, so the actual transmitted position can change when a sampled torque limit is reached. Repeated Passive requests leave Hold unchanged, and active-mode services cannot bypass it. Hold is not a balance policy or a guarantee of mechanical stability. In particular, holding joint targets must not be treated as safe unsupported standing.

The current implementation uses `hold_supported_release_v3`; it was deployed on 2026-11-05 and passed output-disabled board validation, but has not been validated in a real-robot action. Only `~/release_supported` starts the supported transition from Hold to native damping. Calling this Trigger follows the operator's `RELEASE` assertion that the robot and both arms have external support; that assertion is not machine checked. V3 blocks release at the 0.5 rad/s motion threshold. The exit first transitions for 3 seconds toward the per-joint damping endpoint `max(from, passive)` while reducing kp. It then checks the latest LowState dq samples for all 27 joints once per 50 Hz control cycle; every sampled value must satisfy strict `|dq| < 0.1 rad/s` for 0.5 seconds before kd is reduced over 3 seconds. Any sampled non-quiet value during the kd phase freezes the transition and recomputes the remaining reduction. These are discrete checks of the latest state, not continuous feedback: short peaks between control cycles may be missed. The status field `release_phase` identifies the phase; the overall 10-second deadline latches `release_blocked`, and status also reports `release_kp_zero` alongside `exit_protocol=hold_supported_release_v3` and `output_transition=exit|hold|none`. The independent monitor's 500 Hz cumulative 2 rad/s protection remains active. A blocked request remains in Hold and is not retried. Hold may already have kp=0 and provide damping only; it does not guarantee positional or mechanical support. The runner keeps monitoring and requires an explicit operator safety handoff followed by `REVOKE` before revoking output. State, monitor, CRC, authorization or speed faults still take the immediate revocation path. These thresholds and transitions are engineering conditions, not guarantees of safe motion or mechanical release.

The new transition behavior is confined to hardware `position_pd`; simulation and explicit-torque output keep their existing behavior. State faults, stale samples, write failure and `revoke_output` still stop output immediately, without waiting for a normal exit. Revocation never secretly sends another command. With a one-frame budget, the policy is evaluated once but only the entry blend's starting command is written; this does **not** demonstrate completed ONNX takeover. A positive frame budget must not be treated as a completed 3-second entry or a motor execution guarantee.

After sourcing ROS Humble, build and test with `colcon build --packages-select h1_2_policy_bridge` and `colcon test --packages-select h1_2_policy_bridge`. The test model path is configurable at CMake time with `-DH1_POLICY_TEST_MODEL_PATH=<path>`. The public header `h1_2_policy_bridge/core.hpp` and library `h1_policy_core` expose policy-order state, observation, torque, ONNX inference and the manual controller. `ORT_ROOT` and `UNITREE_SDK2_ROOT` are CMake cache variables.

Default launch: `ros2 launch h1_2_policy_bridge policy_bridge.launch.py`. Arguments are `model_path`, `cmd_vel_topic`, `profile` (`sim`), `network_interface` (`lo`), `dds_domain` (`42`), `enable_hardware_output` (`false`), `max_policy_steps` (`0`), `operator_confirmed_debug_mode` (`false`) and `operator_confirmed_exclusive_control` (`false`). Only `sim`/`lo`/`42` and `hardware`/`eth0`/`0` combinations are accepted. Start the simulator separately on loopback domain 42.

For a future **read-only** board run after deployment and review, explicitly select all hardware connection settings and the board model:

```bash
export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=1
ros2 launch h1_2_policy_bridge policy_bridge.launch.py profile:=hardware network_interface:=eth0 dds_domain:=0 model_path:=/home/unitree/h1_nav2_ros2_ws/models/h1_2_armhold/policy.onnx enable_hardware_output:=false operator_confirmed_debug_mode:=false operator_confirmed_exclusive_control:=false max_policy_steps:=0
ros2 service call /h1_2_policy_node/shadow std_srvs/srv/Trigger '{}'
```

Set the same ROS environment for the service call. ROS domain 43 and localhost isolation separate this read-only session from the existing Web ROS domain 0; Unitree DDS independently uses `eth0` and domain 0. The main navigation launch remains restricted to phase-1 simulation; this does not integrate hardware into Web or online navigation.

Shadow requires fresh valid advancing LowState and evaluates zero-command inference without a LowCmd publisher. Logs do not validate physical motion. Hardware FSM services reject requests before `~/authorize_output`; authorization creates a publisher and starts Passive output. `~/revoke_output` stops publication and removes the publisher; software cannot establish motor behavior after interruption. Hardware `max_policy_steps=0` rejects ArmHold. Each authorization permits one positive budget: hardware position PD enters Hold when it is exhausted; explicit torque retains its Passive-damping exit. Faults revoke and stop publication. A frame budget is not a hard real-time duration or firmware guarantee. Actual output parameters and authorization must be reviewed for a specific test before use. The two operator confirmation flags record human confirmation of the official debug-mode and exclusive-control procedures; they are not machine proof. The software cannot prove firmware ownership or motor response to a stream interruption and does not automatically call `ReleaseMode`. Existing sling and emergency-stop preparation does not authorize a specific output test.

The `~/passive`, `~/fix_stand`, `~/arm_hold_stand`, `~/arm_hold` Trigger services are manual. Fresh valid state is required. Each stand transition takes at least 3 seconds; ArmHold checks upright orientation; the legacy profile also checks knee positions. The training profile records entry angle deviation without rejecting on that deviation, while retaining its runtime joint limits. Status reports `entry_angle_gate=diagnostic_only` for training and `entry_angle_gate=enforced` for legacy; `arm_hold_knee_tolerance` remains visible as a diagnostic value. These checks do not establish physical height or sling support. A low-state or inference fault stops publication; a velocity timeout of 0.4 seconds instead sends zero velocity while the policy maintains standing. In `torque` mode, active control computes clipped explicit PD torque at 50 Hz with zero embedded kp/kd. In `position_pd` mode, it sends position targets and motor-side gains at 50 Hz. The status `software_torque_clamp` describes the selected active control mode; it does not constrain motor-side PD or hardware Passive damping.

For the isolated DDS smoke tests, build with `BUILD_TESTING=ON`, then run `bash src/h1_2_policy_bridge/test/run_dds_smoke.sh stale` and `bash src/h1_2_policy_bridge/test/run_dds_smoke.sh nonfinite` from the workspace root with loopback access and no physical robot connection. Logs are in `/tmp/h1_policy_smoke/`. `bash src/h1_2_policy_bridge/test/run_lock_smoke.sh` verifies the same-UID advisory lock when domain 42 is idle. The lock is not cross-user hardware arbitration. The node and SDK-only test use DT_RPATH for the SDK's matched CycloneDDS libraries; do not replace these with ROS's differently built libraries.

## Navigation integration

The independent bridge still starts read-only on hardware. `enable_navigation_commands` and `continuous_policy_navigation` default to false. A real navigation run requires both startup parameters true, hardware `position_pd`, and `max_policy_steps=0`; conflicting settings are rejected. The existing finite policy budget remains available for diagnostics when continuous navigation is off. Authorization and the Passive → FixStand → ArmHoldStand → ArmHold sequence remain manual, and neither parameter releases the Unitree controller or establishes control ownership.

`/h1_2/policy_ready` is a 50 Hz heartbeat: true means software has an active ArmHold output, fresh valid state, completed entry transition, and (on hardware) authorized continuous navigation. It is not a measurement of balance or physical safety. Hardware velocity input is ignored until ready; only commands received after readiness may be used. A command older than 0.4 seconds becomes zero. Limits remain vx ±0.2 m/s, vy 0, yaw ±0.5 rad/s. Output inhibition, fault, Hold, exit, stale state or loss of the heartbeat must block navigation. Do not interpret this heartbeat as proof of exclusive motor ownership or as an emergency stop.

Hardware navigation also requires a fresh `/h1_2/policy_gate_alive` heartbeat. At proxy startup and after either Nav2 action cancel service disappears, the proxy requests cancel-all on both backend actions, waits for acknowledged goals and any known active goals to reach terminal status, and waits 0.2 seconds for status delivery before publishing alive. Until then, and whenever the heartbeat expires, velocity commands are zero; on recovery only a fresh velocity command received after readiness can be used. This is a software recovery protocol, not proof that all action status messages were delivered or that motors stopped.

The deploy reference uses q/kp/kd with zero dq and tau; these fields match `position_pd`. This bridge currently sends LowCmd at 50 Hz and applies hardware-only entry blending and sampled target projection, unlike the reference deploy's 1 kHz FSM loop and unmodified position target. Matching output fields therefore does not establish timing or physical equivalence. The projection is not a motor-side force limit. Navigation must remain inactive until robot ownership, preparation, policy takeover, stopping and emergency procedures have been independently verified on the actual robot.

## 导航运控后端切换

2026-11-05，新版三个相关包已部署到板端 `192.168.123.164`。板端正常 `source /home/unitree/h1_nav2_ros2_ws/install/setup.bash` 可加载新版。正式安装的三个包是指向持久目录 `/home/unitree/h1_nav2_ros2_ws/staging_policy_switch_20261105_5IcSXL/install/` 的符号链接，**不要删除该目录**；旧源码及安装保存在该目录的 `promoted_old/` 和 `backup/`，回滚脚本为 `rollback.sh`，回滚前必须停止相关节点。部署后的板端检查仅覆盖输出禁用的状态及shadow推理，不代表真机运动通过，已有准备与退出异常仍需处理。该板端记录属于当时的 v2 版本；本地 v3 目前尚未部署，也没有真机验证结果。

以下为开发机工作区示例。开发机 `/tmp/h1_switch_install` 属于临时构建目录，若不存在请先执行下面的重新构建命令；普通 `source install/setup.bash` 可能仍加载旧版：

```bash
cd /home/llx/h1_nav2_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source /tmp/h1_switch_install/local_setup.bash
```

重新构建本机隔离 overlay 的命令（不运行机器人控制）：

```bash
colcon --log-base /tmp/h1_switch_log build \
  --packages-select h1_2_policy_bridge h1_2_nav_tools h1_2_slam_bringup \
  --build-base /tmp/h1_switch_build --install-base /tmp/h1_switch_install \
  --cmake-args -DBUILD_TESTING=ON
```

机器人板端需要 source **板端的** install；开发机 `/tmp` 不会自动出现在板端。模型路径、ORT/SDK 路径、地图以及网卡按运行机实际位置设置。所有 ROS 节点需要处于可互通的 ROS_DOMAIN_ID 和 ROS 网络；`policy_dds_domain` 是宇树 SDK 的 DDS domain，不是 ROS_DOMAIN_ID，不能混用。

旧宇树后端仍使用原有导航启动命令，`motion_backend:=unitree` 是默认值；它启动原有 LocoClient 桥。切换前先停止旧导航及旧运动桥，并按机器人实际控制流程确认原运控已释放、机器人处于允许使用低层控制的状态。两个后端的 launch 是互斥的，但两个独立启动的进程之间，软件不能证明真实电机控制权独占。

策略后端在原有导航 launch 参数上增加以下参数。下面的模型、网卡和 domain 为示意，必须换成现场实际值；两个 `operator_confirmed_*` 只有现场确实完成相应确认时才设为 true，它们是人工声明，不是机器验证：

```bash
ros2 launch h1_2_slam_bringup h1_2_nav2_navigation.launch.py \
  map:=<原来的地图yaml路径> \
  motion_backend:=policy policy_profile:=hardware \
  policy_model_path:=<运行机上的policy.onnx路径> \
  policy_network_interface:=<机器人SDK网卡> policy_dds_domain:=<机器人SDK域号> \
  policy_enable_hardware_output:=true \
  policy_operator_confirmed_debug_mode:=true \
  policy_operator_confirmed_exclusive_control:=true \
  policy_enable_navigation_commands:=true \
  policy_continuous_policy_navigation:=true policy_max_policy_steps:=0
```

其余定位、点云地图和导航参数沿用原来的启动方式。hardware 默认选 `position_pd`，对应 deploy 的 q/kp/kd 输出字段；传 `policy_joint_command_mode` 可明确覆盖，但连续导航当前只接受 `position_pd`。`policy_enable_hardware_output` 和确认项只允许后续手动授权，不会自行打开输出。当前 policy 导航门控只支持 `localization_backend:=icp` 或 `global_gicp`。

启动后，先用 `ros2 service call /h1_2_policy_node/status std_srvs/srv/Trigger '{}'` 查看状态。准备输出需由现场操作人员明确调用 `/h1_2_policy_node/authorize_output`，再按顺序调用 `/h1_2_policy_node/fix_stand`、`/h1_2_policy_node/arm_hold_stand`、`/h1_2_policy_node/arm_hold`（均为 `std_srvs/srv/Trigger`），各阶段等待实际完成和状态检查，不把服务返回 success 当成动作安全或到位的证明。

`ros2 topic echo /h1_2/policy_ready` 只有在已授权、状态有效、ArmHold 输出过渡完成后才会出现 true；这是软件就绪信号，不证明机器人已经平衡或适合开始导航。代理只对 `navigate_to_pose` 和 `navigate_through_poses` 入口进行就绪门控并在就绪失效时请求取消；直接调用私有后端 action、FollowWaypoints 或其他底层 action 不经过该代理，不应作为该流程的导航入口。

门控节点还发布 `/h1_2/policy_gate_alive`。代理启动或后台 action 服务重连时，会请求取消两个后台 action 的旧目标，等待已知目标进入结束状态；在清理完成前，或者心跳中断超过0.2秒时，策略速度命令归零。恢复后需要收到新的速度消息才会再次采用非零指令。这个软件流程不能证明消息必然送达，也不能证明电机已经停止；此前实机退出异常仍须处理。

正常请求 `/h1_2_policy_node/passive` 时，本地 v3 硬件 position PD 会先进入 **Hold**，保持最后命令；blocked 时 kp 可能已经为零，此时只有阻尼，不能据此认为位置支撑或机械支撑仍在。v3 在 0.5 rad/s 运动阈值处阻断释放；释放先用 3 秒降低 kp，并保持/平滑到每关节 `max(from, passive)` 阻尼端点。之后每个 50 Hz 控制周期读取最新 LowState，在 27 关节上检查 dq 样本；所有被采样值必须严格满足 `|dq| < 0.1 rad/s` 并连续保持 0.5 秒，才用 3 秒降低 kd。kd 阶段任一被采样的 non-quiet 值会冻结并重算剩余变化。这些是离散的最新状态检查，不是连续反馈，可能漏掉控制周期之间的短峰。状态字段是 `release_phase`、`release_blocked` 和 `release_kp_zero`，总 deadline 10 秒会锁存 blocked。独立监测器以 500 Hz 累计执行原有 2 rad/s 保护。blocked 后保持 Hold 并持续监测，不重试 release；只有现场明确确认物理安全交接后输入 `REVOKE` 才撤权。状态、监测、CRC 或超过原有独立 2 rad/s 保护等故障仍立即尝试撤权。正常释放 runner 等待最多 12 秒，超时走撤权检查。以上是工程条件，不保证安全；v3 已于 2026-11-05 部署并通过板端关闭输出验收，尚未进行真实动作验证。`/h1_2_policy_node/revoke_output` 是停止发送命令的撤权，不证明电机已经停止或进入阻尼，也不能替代物理急停。旧运控重新接管须遵循现场已验证的宇树流程，不由本节点自动执行。
