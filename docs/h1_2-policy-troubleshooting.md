# H1-2 策略接口问题与处理记录

## 2026-11-18 ArmHold 请求成功后退回 Passive（待核实）

网页日志在 12:18:43、12:19:31、12:20:30、12:21:34 分别记录 authorize、FixStand、ArmHoldStand、ArmHold 请求成功。12:22:33 的只读 `/api/status` 却报告 `mode=Passive`、`authorized=true`、`publisher_present=true`、`state_valid=true`、`navigation_ready=false`、`remote_source=zero`；Policy bridge PID12189仍运行，Nav2 未启动。因此目前证据指向“ArmHold 后模式回退”，不是 bridge 进程退出。

状态形态与 2026-11-17 已确认的训练 guard 事件相符：guard 触发会把控制模式切回 Passive，同时保持授权和 publisher；此前已记录 SDK index 2 `left_hip_roll_joint` 速度 `3.91083455 rad/s` 超过 `2.5 rad/s` 限值。但本次 Web 状态解析没有返回 `trained_guard_tripped` 及 `trained_guard_reason/index/value/limit`，后续 SSH 又因本机有线口 `NO-CARRIER`、路由误走 Wi-Fi 而超时，未拿到当前原始 status 或 ROS 日志。故本次是否再次 guard trip、具体关节和数值仍未确认。

此次 bridge 启动参数含 `max_linear_speed=0.20`，但快照显示 Nav2 未运行且遥控速度源为 zero；没有证据表明该速度参数触发了这次状态回退。未发送额外控制、停止、撤权或重授权命令。保持现场支撑；恢复有线后先只读保全原始 status 与 ROS guard 日志，不能通过撤权/重授权来试探或清除状态。

## 2026-11-17 训练保护锁存，FixStand 被拒绝

用户日志中重复出现 `training guard latched; revoke and reauthorize before active modes`，对应的网页 `fix_stand` 调用均为 `success=false`。只读查询 `/h1_2_policy_node/status` 得到：`mode=Passive`、`trained_guard_tripped=true`、`trained_guard_native_damping=true`、`authorized=true`、`publisher_present=true`、`navigation_ready=false`、`policy_cmd=0,0,0`、`state=valid`。触发记录为 `trained_guard_reason=joint_speed`，SDK index 2（`left_hip_roll_joint`）读数 `3.91083455 rad/s`，超过限值 `2.5 rad/s`；事件 `elapsed=522.570327s`、非 warmup。

**含义与边界**：guard 已把 bridge 切回 Passive，并持续请求 native damping；`trained_guard_native_damping=true` 仅是软件输出状态，不验证电机行为或物理支撑。guard latch 下所有非 Passive 转场都会在状态读取/transition 前被拒绝，重复 FixStand 不会清锁存。Web `/api/status` 当前仅返回 mode/授权等字段，没有暴露 `trained_guard_reason/index/value/limit`，所以界面只显示通用拒绝。

**安全处理**：没有调用 revoke、authorize 或姿态服务。`revoke_output` 会调用 `inhibit()`、关闭 LowCmd publisher；后续重新授权会清除 guard latch 并重新开放主动准备，所以不能把它当普通网页重置。触发原因和状态应先保全并查明；摔倒事故审查前保持物理支撑，不继续 FixStand/ArmHold、撤权、重新授权或遥控接管。

## 2026-11-17 旧手动测试网页与当前导航网页的控制交接差异

用户记得旧 `web_test` 可以在测试后恢复遥控，这个记忆与代码流程一致，但“切换”主要来自会话生命周期，不是网页调用了 H1-2 专用的官方遥控器交接 API。旧手动测试 runner 在结束测试时进入 Passive/完成现场支撑确认，然后调用 `revoke_output`，检查 `authorized=false`、`publisher_present=false` 及写计数不再增长，最后退出 runner；因此 Policy 的 LowCmd writer 会消失，现场已有的宇树控制路径才可能重新获得机会。

当前导航网页采用长期运行的 Policy bridge。停止导航只停止 Nav2/SLAM/路线，不撤销 bridge；正常状态会保留 ArmHold/LowCmd writer，供下一次导航继续使用。训练保护触发时 bridge 会回到 Passive 并清零策略速度，但仍可能保持 `authorized=true`、`publisher_present=true`，所以遥控器不会自动接管。当前 bridge 代码也没有调用 `ReleaseMode`、订阅 `rt/wirelesscontroller` 或验证固件控制权转移。

因此，旧网页“能切回遥控”不能直接证明当前导航网页具备同样能力。要恢复旧行为，需要单独实现并验证“停止 Policy 输出→确认 writer 消失→按 H1-2 固件支持的流程交给遥控器”的完整交接；不能只把导航停止按钮或 `Passive` 状态当作遥控器接管。

## 2026-11-17 速度指令与遥控器能否共存

旧 Unitree 链路的共存方式是高层接口共存：`/cmd_vel` 经过 `h1_2_loco_bridge` 和 UDP daemon 调用 `LocoClient.SetVelocity`，遥控器输入仍由宇树运动控制层处理。它不是网页、导航节点和遥控器各自同时写 `rt/lowcmd`。旧 daemon 还有速度超时后调用 `StopMove` 的路径，因此外部速度停止后，固件侧仍有机会按原有遥控逻辑工作。

当前 ONNX 链路不同：Policy bridge 直接根据 `/cmd_vel_final` 生成整套关节 `rt/lowcmd`，当前没有订阅 `rt/wirelesscontroller`，也没有把遥控器按钮/摇杆纳入同一个仲裁器。若再启动另一个遥控 LowCmd writer，会形成两个底层写者竞争，不能当作安全共存。

要实现旧链路那种共存，必须让一个唯一的 LowCmd writer 在 bridge 内完成仲裁，例如急停/遥控器退出优先于外部速度，外部速度只在新鲜且获得授权时生效，超时归零，并明确 FSM、死区、优先级和交接状态；不能简单地同时启动 ONNX 和旧遥控控制进程。当前尚未实现或验证这条 H1-2 HG LowCmd 遥控仲裁路径。

## 2026-11-17 导航目标延迟与 ArmHold 遥控器控制

**目标延迟现象**：用户提供的日志显示 NavigateToPose 在16:10:05开始跟踪 `(0.73, 1.45)`，16:10:15 报 `Failed to make progress`，随后清除局部代价地图并再次计算控制；同一时段出现一次 10 Hz controller loop missed。当前在线 camera-init 配置的 SimpleProgressChecker 要求10秒内移动至少0.3m，时间与失败相符。自定义 BT 在同一个导航 action 内清局部代价地图并重试 FollowPath；这不是自动进入下一个路线点。memory route 仅在成功后默认等待2秒发下一点，最终失败后默认等待5秒重试同一点。

若用户是在 RViz 连续下发 `/goal_pose`，单独的 `goal_pose_to_nav2` 会异步取消旧 action 后立即发送新 action，没有等待取消完成，因此存在短暂的 goal preemption 竞态；这段逻辑没有固定10秒等待。现有日志没有第二个目标的发布时间、取消确认和 action accept/result 时间，不能据此确认它就是延迟原因。运行中的物理位移、TF、`/cmd_vel` 与 `/cmd_vel_final` 也没有同步数据；不把配置速度上限当作实际速度。

**ArmHold 与遥控器**：Policy bridge 的 ArmHold 是桥内 ONNX/position-PD 状态，不是 Unitree 手柄控制模式。bridge 订阅导航速度输入（默认 `/cmd_vel_final`），当前代码没有订阅 `rt/wirelesscontroller` 或把手柄输入转换为速度。硬件输出已授权且 publisher 存在时，bridge 持续写 LowCmd；无新鲜导航速度时策略命令归零并维持控制。因此 ArmHold 下实体遥控器不能控制属于当前接口能力边界，不能让手柄和 Policy 同时抢控制。恢复遥控需另行完成受支撑的控制权交接与官方模式切换；故障审查前不做切换或运动试验。

**诊断边界**：以上为用户日志与当前本地/已部署源码配置相符的分析；没有取得第二个目标的完整时间线，也没有执行当前 ROS 状态查询。需要确认延迟时，按同一时钟记录 `/goal_pose` 发布、`goal_pose_to_nav2` cancel/send/accept/result、Nav2 action 状态、`/cmd_vel`、`/cmd_vel_final`、Policy `policy_cmd`、`/slam_odom` 和 TF age。遥控器问题需同时核对 Policy `mode/authorized/publisher_present/output_transition`、`/cmd_vel_final` 是否有消息、LowCmd writer GUID 及经固件确认的遥控器输入遥测；单独看到 ArmHold 或 LowCmd writer 不等于确认实际控制权。未发控制命令或修改代码。

## 2026-11-17 SLAM 停止后的 RViz 旧场景与导航速度偏低

**已确认原因**：此前“停止全部”只处理 PC4 的 SLAM/Nav2；AI PC 上通过 `h1-rviz://open` 单独启动的 RViz 不会随 ROS 发布者停止而关闭。URI 锁已持有时，启动器只会激活旧窗口，因此重开仍看见缓存场景。Web 还可能把外部启动导航的后端误按默认 Unitree 处理，没有等待 Policy 停止脚本的退出结果。速度方面，近期在线 camera_init 模式用 DWB，`max_vel_x`/`max_speed_xy` 与 Policy bridge 上限均为 `0.20 m/s`；离线 RPP 的 `desired_linear_vel=0.10 m/s` 是另一配置。历史日志没有当次 `/cmd_vel`、`/cmd_vel_final`、Policy 命令和实体位移同步数据，无法确认上限导致原地踏步。

**本地修改**：Web 现在读取实际导航实例后端，等待停止脚本退出并核验 PID 状态；失败时不返回成功，也不触发后续 RViz 关闭。成功后 Web 调用 `h1-rviz://close`。本机启动器识别 `ros2` 的 Python shim 命令行；优先关闭登记 PID，旧版未登记时仅在 `/proc` 中唯一找到 SID/PGID/PID一致且命令精确匹配 H1-2 RViz launch 的进程组时才发 SIGINT，超时再发 SIGTERM。停止脚本的 PID fallback 只接受已知 H1-2 导航 launch 文件，避免 PID 复用后误停同包其他 launch。交互目标要求目标 frame 与当前导航 frame 一致，且 `map/camera_init→body` TF 存在并新鲜（不超过0.5秒）。RPP 名义速度改为 `0.15 m/s`，只影响离线 RPP；在线 DWB 速度上限未改，不提高硬件输出上限。

**验证及边界**：此前全静态测试169项、定向单元测试69项、纯函数 TF gate 3项通过；本轮停止脚本身份回归与 RViz launcher 测试25项通过，Python 编译、`bash -n` 和 scoped `git diff --check` 通过。全 unit+static 收集被本机缺少 ROS `builtin_interfaces` 阻断。没有部署到 PC4、关闭真机节点、启动 RViz 或进行运动试验；停止修改尚未板端验证，在线慢速的实际原因和实机速度仍未验证，摔倒故障仍需审查。详见[计划](h1_2-stop-rviz-readiness/plan.md)及[当前任务](../tasks/20261107-onnx-navigation-development.md)。

## 2026-11-16 Policy Fault 后机器人摔倒（根因未完全确认）

**现场安全状态**：用户报告机器人刚刚摔倒。仅做 PC4 日志/状态只读检查，没有发控制、停止、撤权或复位命令。现场应保持物理急停并支撑机器人；不得重试控制。

**已确认时间线**：PC4 Policy bridge 进程 4132/4134 从 19:29 起运行，启动参数为 `max_linear_speed=0.12 m/s`、`max_angular_speed=0.10 rad/s`，不是本地源码刚调整的 `0.20 m/s`。Nav2 在线栈 PID 58439 于20:51启动，memory route runner PID 58602 存活。Policy 日志在21:21:18（epoch `1794835278.555`）报 `Policy fault: state invalid before write`。故障后只读状态为 `mode=Fault`、`authorized=false`、`publisher_present=false`、`navigation_ready=false`、`policy_cmd=0,0,0`、`state=valid`。代码中该异常路径调用 `inhibit()`，立即撤权并关闭 LowCmd publisher；没有先把 position-PD 最后输出转入 ArmHold。21:28:09 网页心跳超时，停止脚本因无法确认 Policy ArmHold 而拒绝停止导航，导航进程仍在运行。现存网页日志未发现21:21之后的新导航目标；21:54前一次导航目标日志与本次摔倒时间不相邻。

**判断与限制**：`state invalid before write` 由写入前组合门槛触发，包括授权、fault pending、状态存在/新鲜度（100ms）、LowState 更新进度新鲜度及命令快照新鲜度；错误文本没有指出具体失败项。关闭唯一 Policy LowCmd 输出会让 position-PD 关节支撑消失，是导致摔倒的强相关软件机制，但现有证据不能证明具体状态源失效原因，也不能完全排除机械/执行器/其他控制器因素。当前可排除“刚改成0.20导致这次摔倒”：该改动只在本地，新现场进程明确仍是0.12。

**处理边界**：机器人摔倒后保持现场急停和支撑，不重启桥、不点导航/站立/授权/撤权，不以远程停止脚本替代物理急停。保留 PC4 原始 ROS 日志、节点日志、LowState/LowCmd trace 和 web ring log；尚未确认这些数据是否覆盖故障前完整关节轨迹。下一步需先由现场人员确认无人受伤、机器人固定安全，再在不驱动机器人的情况下保存完整故障证据并判定写入门槛的具体失效项。禁止实机复测直至故障机理和故障输出策略经审查。

**后续软件状态（21:37 只读）**：用户贴出网页“启动请求已接受”日志后，查询确认旧 Fault 节点已退出；网页在21:36启动了新 bridge PID 2570。新节点报告 `Passive / authorized=false / publisher_present=false / write_count=0 / state=valid`，未发现 Nav2 或 route runner。新节点未发布 LowCmd，但这不构成倒地机器人的安全复位或重新控制许可。应停在该状态，不点击授权、FixStand、ArmHoldStand 或 ArmHold。

**21:47–21:51 停止与 RViz 场景**：用户日志显示网页再次授权并逐步进入 ArmHold；21:47启动在线 Nav2/FAST-LIO。costmap 多次报 `camera_init` frame 不存在，`goal_pose_to_nav2` 的 `(1.32,-0.67)` 目标被拒绝。随后21:48:05 Nav2 lifecycle 收到 SIGINT；21:48:33日志中出现新的 FAST-LIO/Nav2节点初始化，21:48:56再次点击停止，21:49:00报告 `Stopped H1-2 ai slam nav processes.`。21:51:42 PC4只读进程检查未发现 FAST-LIO、Livox、占据栅格、Nav2或route runner；只剩Policy bridge。当前 Policy status 为 `ArmHold / authorized=true / publisher_present=true / navigation_ready=false / policy_cmd=0`。因此最后一次停止已结束SLAM/Nav2流程；RViz继续显示此前场景是因为RViz独立运行并保留最后收到的地图，不证明SLAM仍在运行。在线目标被拒绝的直接证据是启动时缺少`camera_init` TF；任务文件又是`frame_id=map`，不能直接混入在线坐标系。鉴于同一事故后 Policy 已重新授权且位置状态仅能从遥测获知，禁止继续发送导航目标；须现场确认机器人状态并先处理跌倒事故。

**21:55 停止日志复核**：新附件记录Nav2/FAST-LIO收到SIGINT并清理；`planner_server` 在ROS context关闭时出现 lifecycle cleanup 错误并退出码 `-6`，其余进程相继退出。launch于21:55:32退出码0，停止脚本也打印成功并退出码0。21:56:02 PC4进程检查没有FAST-LIO、Nav2、Livox或route runner，仅Policy bridge PID2568/2570仍在运行。因此该次SLAM流程已停；planner_server异常发生在关闭清理期间，未阻止整组退出。RViz是独立进程，地图显示不会随ROS publisher退出自动清空。

## 2026-11-16 ONNX 导航给定目标后原地踏步

**现象与日志**：用户反馈给定方向后机器人原地踏步。导航日志中目标 `(1.90, 0.00)` 多次规划失败；目标 `(1.64, -0.63)` 被接受后，Controller 每约10秒报告 `Failed to make progress` 并重试。20:54:58 又报 `camera_init→body` TF 比控制数据旧约2.52秒，却接着输出 `Reached the goal` 和 `Goal succeeded`，因此成功状态不能证明机器人实际到点。

**速度与现场快照**：该请求是 Nav2 目标位姿，不是直接速度输入。此前 Policy bridge 线速度限幅为 `0.12 m/s`、角速度 `0.10 rad/s`；Nav2 DWB 的 `max_vel_x=0.2 m/s`，实际命令还会经过 bridge 限幅。用户反馈速度偏慢后，启动脚本和 bridge 默认线速度上限已提高到 `0.20 m/s`，角速度仍为 `0.10 rad/s`。这个值沿用旧 Nav2 软件配置上限，不是 Unitree 官方 SDK 的厂商默认值，也不是已经完成的 ONNX 真机速度验收；ONNX 真机历史验证仍只到 `0.12 m/s`。原附件没有记录动作期间 `/cmd_vel_final` 或 `policy_cmd`，不能据此认定旧行为实际达到了限幅。20:56 只读快照为 route idle、bridge `ArmHold / authorized=true / navigation_ready=true / policy_cmd=0,0,0`；不是动作期间数据。之后 `tf2_echo` 可读到新鲜 `camera_init→body`，约1秒采样位姿变化很小。根因仍未确认；速度上限变化应在安全监护下逐步验收，不能替代定位/里程计及实际速度核查。

**处理边界**：新上限仅对新启动的 bridge/Nav2 生效，运行中的桥接器参数只读，本次没有重启或发出运动命令。真机首次使用 `0.20 m/s` 前，应在有支撑/急停和现场监护下，用短时低风险命令同步记录 Nav2 `/cmd_vel`、`/cmd_vel_final`、Policy `policy_cmd`/状态、`/slam_odom` 与 `camera_init→body` TF 时间戳，并对照实体位移；若仍原地踏步，应先看命令传递和定位数据，不继续增加速度。若现场出现非预期运动，立即使用现场急停/停止动作。

## 2026-11-16 DDS 错误地址指向上位机 Wi-Fi

**现象**：PC4 的 Cyclone DDS 报向 `192.168.8.5:7410/7411` 写 UDP 失败。该地址是上位机 Wi-Fi，不是 MID360 地址。PC4 查询 `ip route get 192.168.8.5` 为 `Network is unreachable`；雷达 `192.168.124.20` 与数据目的 `192.168.124.164` 均经 `br0`。Livox 进程使用 `rmw_cyclonedds_cpp` 和 H1-2 配置 `cyclonedds_pc4.xml`，`/livox/lidar` 暖机后约 10 Hz。

**原因与修复**：上位机 Policy RViz 原先使用 Fast DDS 自动选取多网卡，并可能把 Wi-Fi locator 通告给 PC4。新增 `config/fastdds_ai_pc_policy.xml`，将 UDP transport 白名单限定为上位机有线地址 `192.168.123.99`；`scripts/ai_pc_start_rviz.sh online policy` 加载该 profile，Unitree/Cyclone 启动分支不变。正常重启本机只读 RViz 后，socket 仅绑定 `192.168.123.99` 的 7400/7410/7411 和动态 UDP 端口。PC4 新建 Cyclone ROS CLI 查询可发现同一 ROS graph，未再出现发送到 `192.168.8.5` 的报错。

**验证与边界**：RViz 启动器测试 14/14、XML 解析、shell 语法与 diff 检查通过。Fast DDS 能发现 `/livox/lidar`；PC4 侧 `/livox/lidar` 稳定约 10 Hz。查询时 `/map_preview` 当前 publisher count 为 0，因此地图显示仍需在线建图/导航节点实际运行后验证。本轮没有改 MID360 配置或 PC4 的 `br0`/Cyclone 配置，也没有重启/操作 Policy bridge、导航或 Livox 控制流程。

## 2026-11-16 Livox DDS 发往不可达的旧远端地址

**现象**：用户用 `CYCLONEDDS_URI=file:///home/unitree/h1_nav2_ros2_ws/config/cyclonedds_pc4.xml` 启动 Livox 后，不再报缺失网卡；节点读取 `MID360_config.json`，识别雷达 `192.168.124.20`，成功切到 Normal 并启用 IMU。但 Cyclone DDS 多次报向 `192.168.8.5:7410/7411` 写 UDP 失败。用户随后 Ctrl-C；日志显示 `Livox Lidar SDK Deinit completely!`，进程最终 exit `-7`。

**现场配置与路由**：PC4 DDS 配置固定活动 `eth0`（`192.168.123.164`）；MID360 原始雷达链路配置为设备 `192.168.124.20`、主机数据目的地址 `192.168.124.164`，该网段经 `br0` 路由正常。PC4 查询 `ip route get 192.168.8.5` 返回 `Network is unreachable`。该地址是历史 AI PC 网段的 peer，7410/7411是DDS端口；因此这些错误表示Cyclone向不可达DDS参与者发包失败，不是Livox设备握手失败。实际 `/livox/lidar`、`/livox/imu` 消息数量未在此日志中验证，不能仅凭 `publish use ... format` 确认有样本。

**处理建议与未决项**：不要改 MID360 的 `192.168.124.164` 原始数据目的地址来消除 `192.168.8.5` DDS 警告；先在 Livox运行时以相同Cyclone/domain查询 `/livox/lidar` 和 `/livox/imu` 频率，再从 Fast DDS 的 FAST-LIO侧确认是否收到点云。若本机Livox消息正常而FAST-LIO无样本，再单独排查Cyclone/Fast DDS跨实现发现、网卡和QoS。Ctrl-C之后的 `exit -7` 是关闭阶段异常，其具体原因未确认；不应与运行期DDS目的地址失败混为一个根因。当前Livox进程已退出，未重启。

## 2026-11-16 在线导航缺少 camera_init TF

**现象**：20:05 在线 ONNX 导航启动后，Nav2 costmap 每0.5秒报 `Invalid frame ID "camera_init"`，无法从 `body` 变换到在线地图坐标系。20:05:43 有一条 Nav2 goal 被拒绝。

**已确认现场证据**：启动命令使用 `mode=online motion_backend=policy`，launch 参数 `start_livox_driver:=false`，表示 Livox 必须由外部终端单独运行。FAST-LIO进程在20:05:06启动并报告 `Node init finished`，但本次 Web 日志未见该 session 的 `Published map`/在线栅格输出；`camera_init` TF因此未进入 TF 树。结合此前 Livox 使用旧 Go2 Cyclone XML 并因失效网卡 `wlxf8ce21a12bfe` 创建 DDS domain 失败，最可能原因是 Livox 未成功运行或没有向该 FAST-LIO实例提供点云；历史日志不能单独证明20:05瞬间 Livox进程状态，故根因细分仍待下次只读核验 `/livox/lidar`、FAST-LIO odometry/TF。

**另一个已确认问题**：`maps/tasks/数字钥匙.json` 的 `frame_id` 为 `map`，创建时间为2026-10-12；本次在线导航期望 `camera_init`。20:05:07 `memory_point_manager` 明确拒绝混用坐标系，20:05:43 的目标随后被拒绝。在线模式需要重新录制 `camera_init` 点位，不能直接复用旧 `map` 坐标。

**当前状态与处理**：20:05:57 点击停止后，Nav2/FAST-LIO/route runner已退出，导航 PID 标记清理；无Livox/FAST-LIO进程。Policy保持 `ArmHold / authorized=true / policy_cmd=0`。本轮不重启导航、不发送目标。下次启动前先用H1-2 `cyclonedds_pc4.xml` 启Livox，并只读确认 `/livox/lidar`、`/livox/imu` 有数据；随后启动在线栈，确认 `camera_init→body` 新鲜TF和动态地图输出，再新建/录制在线任务点后才能开始巡航。

## 2026-11-16 网页“停止全部”未停止导航

**现象**：在 ONNX 网页按“停止全部”后，Nav2/FAST-LIO 进程仍运行。8775 Web 日志显示 `stop_ai_slam_nav.sh` 退出码1，错误为 `/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable`。

**原因**：停止脚本启用 `set -u` 后直接 source ROS Humble setup；该 setup 在 `AMENT_TRACE_SETUP_FILES` 未定义时触发 nounset，故脚本在查询 Policy 状态及向导航 launch 进程组发送 SIGINT 之前退出。重复点击会重复相同失败。

**修复与执行**：停止脚本仅在 source ROS/Unitree/工作区环境时暂时关闭 nounset，随后恢复严格模式。更新脚本前备份至 `/home/unitree/h1_nav2_ros2_ws/backup_stop_all_nounset_20261116/`；PC4 SHA256 `c424265f5bb5141b7d1ddf9737efaccaef268f8f6545c0ca444ed05737a67939`。脚本已按原有 PID/PGID/SID/命令行身份校验验证导航 launch 专属进程组，查询到 Policy 已在 ArmHold 后发送 SIGINT，运行退出码0。

**验证结果与边界**：Nav2/FAST-LIO/route runner 进程退出，`/tmp/h1_2_ai_slam_nav.pid` 与 backend 标记清理；Policy bridge 保留运行，直接 Fast DDS status 为 `ArmHold / authorized=true / policy_cmd=0 / navigation_ready=false`。这是既有安全设计：“停止全部”停止导航会话并让 Policy 保持 ArmHold，不会撤权/关闭 bridge。停止后 Web `/api/status` 在12秒内无响应，但直接 ROS status 查询成功。未执行 RELEASE/REVOKE。PC4 部署测试7 passed，`bash -n`和 `git diff --check`通过。

## 2026-11-16 Web 显示 Policy status unavailable

**现象**：ONNX 网页显示 `/h1_2_policy_node/status is not available`，但导航启动步骤显示已完成。

**现场只读核查**：PC4 上 `run_ai_slam_nav.sh`、`h1_2_policy_node`、Nav2 和 `memory_route_runner` 均在运行。Fast DDS CLI 可发现 status 服务并返回 `mode=Passive`、`authorized=false`、`publisher_present=false`、`navigation_ready=false`；`/memory_route_status` 返回 `idle`。网页 API 起初报告 status unavailable，单独 helper 调用成功后再查 API 则 `policy_control.available=true`。因此不是 bridge 未启动，也不是正在授权/发运动命令。

**原因判断**：网页 `policy_web_client._call()` 的 `wait_for_service` 只等 0.6 秒；CLI 冷启动发现服务耗时超过这一窗口，而后续 Fast DDS discovery 已暖时 helper 可成功。网页/API 状态刷新还会顺序请求 bridge、route、定位等接口，现场观测约 7.6–8.5 秒。0.6 秒发现窗口过短是已确认的脆弱点，当前屏幕错误更符合短暂发现超时；尚未修改客户端等待策略。

**当前操作边界**：不点击授权或姿态转换步骤。服务当前显示 Passive、未授权且无 LowCmd publisher，路线只读状态 idle；Nav2 实例仍运行。另注意当次启动参数包含 `start_at_first_route_point=true`，应保持路线 idle 并核对该参数来源。没有重启 Web 或控制栈。

**后续状态变化（同日）**：用户再次报告网页仍显示 status unavailable。新鲜只读 Fast DDS status 与网页 `/api/status` 均已能发现服务，状态变为 `mode=Passive`、`authorized=true`、`publisher_present=true`、`write_count=16180`、`policy_cmd=0`、`navigation_ready=false`；路线 `idle`。Nav2 与 Policy 在运行，启动参数仍含 `start_at_first_route_point=true`。因此网页报错是陈旧/超时状态，bridge 实际已被授权并在 Passive 下写 LowCmd；不代表已进入 FixStand 或行走，但也不应视为无输出。授权发生来源未确认。不要继续 FSM/巡航或重启/取消；现场保持支撑与急停可用，先确认当前物理状态和控制权。

**再次只读复核**：Fast DDS CLI status 服务可发现，返回 `Passive / authorized=true / publisher_present=true / write_count=100214 / policy_cmd=0 / navigation_ready=false`；CLI route 为 idle。网页 `/api/status` 约 8.7 秒才返回，Policy 状态已 available，但网页 helper 的 route status 仍不可用。没有发任何控制请求。实际机器人是否处于 Debug 不能从此状态证明；启动参数中的 operator confirmation 只是确认记录。

## 2026-11-06 LowCmd 混合来源门禁

**现象与来源**：145101 预检未启动策略节点。拦截逻辑已存在于 2026-11-02 提交 `eac4a99`；监测器把收到的全部 LowCmd 累计成一个总数，无法区分策略进程与 vendor humanoid 约 500 Hz 的流量（此前来源元数据约为 `.161`）。该来源标识只说明发布者，不能证明实际执行所有权。

**处理状态**：拟为策略进程原子写出自身 writer GUID，路径由 `H1_POLICY_WRITER_GUID_FILE` 指定；监测器增加逐来源字段。已实现来源 GUID 归因；隔离域双发布者验证自身有效帧、坏 CRC 和停止更新与外源区分。发布来源不能证明执行控制权。排查期间保持底板进程及模式不变，不以总计数为由停止底板或切模式。

用户刚确认现场已准备好，确认的单次范围为 `vx=0.12 m/s`、`0.5 s`，随后归零并继续 ArmHold，不 release。该确认不是动作已发生的证据；在来源归因实现完成并验证前，不记录为新试验结果。

## 2026-11-06：约束审查与精简

12:04 单次现场观察复测：用户报告看到腿部迈步，试验后静止稳定；日志确认非零策略速度，但左膝 SDK3 累计 3.153418 rad/s 超过 3 rad/s 后撤权，保护中止发令后持续零速站立全流程；无精确 start 时间，无法确认 0.5 秒窗口是否完成，持续行走尚未通过验证。节点及监测器退出码 0；详见[聚合证据](../tools/h1_2_policy_hardware/evidence/manual_walk_trial_20261106T120400/summary.md)。

此前短速度试验 `acceptance-20261106T115543` 已观察到非零 `policy_cmd=0.12,0,0`，随后右膝 SDK9 累计速度 3.130776 rad/s 超过 3 rad/s，保护撤权；节点与监测器退出码均为 0。现场迈步和当前物理静止均未证实；发令起点没有精确记录，不能确定超限完全由速度输入造成。采样及撤权时间线见[聚合证据](../tools/h1_2_policy_hardware/evidence/manual_walk_trial_20261106T115543/summary.md)。

| 条件 | 来源/问题 | 本轮处理 |
| --- | --- | --- |
| 额外静止3秒 | 后加试验条件，已删除 | 不再恢复；保留原接管过渡 |
| 脚本前后重复膝角硬检查 | 控制器已有最终转换检查 | 人工模式改记录，由控制器最终判定；有限诊断模式保持 |
| 固定短窗口至少25Hz/2帧 | 抽样假设，不等同通信故障 | 仅记录频率诊断；保留实际计数推进与命令新鲜度检查 |
| ready短暂丢失即撤权 | 脚本将“归零”升级成“停发电机命令” | 取消剩余速度，保持健康姿态控制，不自动重播 |
| 重复60秒现场确认 | 可导致模型尚未接管就超时 | 仅在本次启动前已明确确认现场时，显式使用对应选项跳过；不默认跳过 |
| 状态/CRC/时效、真实超限、输出权故障 | 真实故障路径 | 保留；速度门限仍为试验配置，非厂家认证值 |

用户已在本轮确认今天调试/独占、支撑和急停就绪；仍限定一次0.12 m/s、0.5秒发令后归零继续策略站立，不自动放手。该授权对应的本轮试验已执行并中止；后续动作的现场条件须重新核实。


## 验证目标修正：持续策略运动与放手退出分开

用户实际流程是保持策略和抬臂控制，短时给速度，随后归零继续策略站立；不要求测试结束自动放手。此前将受支撑放手释放作为行走试验的必经条件，测试范围不符合实际需求。

- 右肘释放异常仍保留为退出路径的独立问题，不据此认定 ONNX 无法持续站立或行走。
- 有限帧验收脚本会转 Hold 并等待释放，不能直接用于本次持续策略试验。
- 人工试验应使用隔离的速度和门控话题，保持原有状态、独占、速度和超时保护；正常发令结束只归零，不调用 Passive、release 或降低增益。
- 人工入口已完成并部署：唯一命名空间私有速度话题和人工门控心跳，ROS domain 43/localhost 隔离，不向生产 Nav2 的速度或门控话题发消息。主状态租约 0.15 秒过期后不续发；桥原有门控 0.2 秒、速度命令 0.4 秒超时仍保留。0.5 秒是单调时钟定义的发令窗口，不保证实际机器人硬实时停车。


## 2026-11-05 20:52：现场再次确认支撑后仍阻断

零策略步数执行 Passive → FixStand → ArmHoldStand → Hold；现场再次确认支撑并允许 RELEASE。卸载阶段 2.621146 s 因速度保护阻断：右肘 SDK/策略索引 23，q=-0.266930 rad、dq=+0.505055 rad/s；最后命令 q=-0.570109 rad、kp=15.9541、kd=4。累计观测速度峰值 1.362759 rad/s，未观察到 2 rad/s 超限。现场再次确认静止且有支撑后授权 REVOKE，末两次状态 authorized=false、publisher=false、write_count=2413；节点和监测器退出码均为 0，随后只读检查无目标进程。结果 `needs_operator_check`，未验证原生阻尼或撤权后的电机行为。

本次支撑已由现场确认，但释放仍失败；不能断言仅增加支撑即可解决，受力细节和根因仍待查。零策略步数也不能证明 ONNX 模型有错；不要放宽保护或无改动重试。[本次证据](../tools/h1_2_policy_hardware/evidence/release_recheck_20261105T205236/compact_result.json)。

## 2026-11-05：策略接管后，释放被保护中止

**现象**：零速度 ONNX 接管约 3.1 秒后进入 Hold；释放约 0.59 秒后被中止。全程关节速度峰值 1.203 rad/s，未触发 2 rad/s 保护；现场确认支撑后已撤权。不是行走验证。

**已确认**：右肘 SDK 23 在释放后首个超阈值轨迹样本为 0.550 rad/s，与 0.5 rad/s 释放保护吻合；未到 10 秒超时。该轨迹与控制周期异步，不能当作内部精确触发样本。

| 右肘参数 | 释放开始附近 | 首个 ≥0.5 rad/s 样本 |
| --- | ---: | ---: |
| 实际角 q（rad） | -0.481 | -0.456 |
| 目标角（rad） | -1.520 | -1.323 |
| kp / kd | 7.9 / 0.5 | 6.402 / 0.5 |
| 理论 PD 力矩（Nm） | -8.205 | -5.824 |

理论 PD 力矩由采样状态及命令计算，不是实测力矩。释放过程同时将目标角移向释放时实际角、降低 kp，改变了关节支撑力矩；这与右肘加速时间相邻，**尚不能证明机械根因**。现场未观察到具体变化，需要同步观察右前臂/右肘支撑情况。

**处理**：保留 0.5 / 2 rad/s 保护、模型和 PD 参数；补充阻断原因、阶段、时间、触发关节/速度与最后成功下发目标和增益，避免再次只记录“release blocked”。不按另一套 mjlab 示例覆盖模型参数：它与本模型的配套指南并非同一合同。本模型右肘默认 -0.55 与 `trans.md` 一致。

**复现结果（20:40，准备姿态）**：终端输出过滤发生缓冲，现场确认超时，本次没有进入 ONNX。经现场允许后，从准备姿态 Hold 执行一次释放：2.625 秒时，诊断精确记录 `reason=speed`、`phase=unloading`、`SDK=23`、`dq=0.500691 rad/s`、`q=-0.270177`、最后命令 `q=-0.569702 / kp=15.7943 / kd=4`。现场明确看到右肘下落；确认静止且支撑稳固后已撤权。说明该故障不依赖 ONNX 推理，不能用“模型不好”解释。

**实际处理方式**：释放时电机位置支撑被降低，右臂下落，表明支撑与受力状态需要进一步核查；具体受力、支撑是否承重或滑动尚未确定，不能仅凭现象认定根因。退出前要让右前臂/肘部重量被独立支撑可靠承接，身体吊带或仅扶住手臂不等于承重。保持 Hold 完成物理交接，不能指望 kp 降到零后仍维持悬空抬臂；也不能把“停止发包”当作已进入阻尼。不得通过放宽阈值或突然恢复高 kp 掩盖下落。

| 本轮问题 | 处理 | 验证/边界 |
| --- | --- | --- |
| 阻断只报通用错误，不能定位 | 锁存速度/超时原因、前一阶段、SDK/策略索引、触发 q/dq、最后成功下发 q/kp/kd；撤权后保留，下一授权清除 | 本地、板端核心/节点测试各 2/2；真实准备姿态释放记录了 SDK23 精确样本 |
| 卸载期间右肘下落 | 保持保护及模型参数，明确先由外部支撑承接右前臂重量，再受监测释放 | 现场已确认下落；20:52 现场确认支撑后仍阻断，完整释放尚未验收 |
| 终端过滤缓冲耽误现场确认 | 验收脚本增加简短结尾摘要，完整 JSON 仍写文件；后续直连交互终端，不通过 awk 过滤提示 | 新增针对性测试 3/3，已同步板端并通过语法检查；本次超时不得算 ONNX 执行 |
| 板端测试引用 PC 模型路径 | 配置 `H1_POLICY_TEST_MODEL_PATH=/home/unitree/h1_nav2_ros2_ws/models/h1_2_armhold/policy.onnx` | 板端必要测试 2/2 通过 |

**部署**：诊断包位于板端 `releases/guarded_release_diagnostics_20261105`，正式策略包软链已切换；`backup/rollback.sh` 可在停用相关进程后回滚源码及安装软链。输出关闭验收：有效状态、无发布者、零写入、正常退出。

**剩余事项**：软件诊断已修复；完整机械支撑交接及原生阻尼退出仍待验证。本轮不再自动启动动作。

**证据**：[轨迹分析](../tools/h1_2_policy_hardware/evidence/onnx_bounded_20261105T202400/analysis.json)、[原始轨迹](../tools/h1_2_policy_hardware/evidence/onnx_bounded_20261105T202400/trace.csv.gz)、[结果摘要](../tools/h1_2_policy_hardware/evidence/onnx_bounded_20261105T202400/compact_result.json)。

**脚本验证补充**：完整脚本测试首次 35/36 通过，其中 parent-death 测试报告子进程未退出；单独复测该项通过，首次失败原因未确定，不能宣称已修复。检查未发现测试子进程残留。本轮不改子进程保护逻辑，也不因此继续真机动作。

**第二次证据**：[结果摘要](../tools/h1_2_policy_hardware/evidence/onnx_bounded_20261105T204047/compact_result.json)、[精确阻断分析](../tools/h1_2_policy_hardware/evidence/onnx_bounded_20261105T204047/analysis.json)。全程峰值 1.400 rad/s，无 2 rad/s 超速事件；已确认撤权，节点/监测器均退出。该次没有 ONNX 接管，不与第一次策略试验混记。

## 2026-11-05 21:05：人工行走试验未发出速度

- **范围**：持续 ArmHold，零速度接管及站立后，一次 `vx=0.12 m/s`、0.5 秒发令，结束归零继续策略站立，不自动 release/降增益。
- **软件验证**：人工正常流程/异常撤权 2 项、单次脉冲/租约 1 项、本地原离线测试 15 项通过；板端专项 3/3。首次板端测试未加载 ROS 环境导致消息模块不可导入，加载 `/opt/ros/humble/setup.bash` 后通过，未在失败环境中启动电机。
- **实机结果**：记录 `acceptance-20261105T210530`。现场确认后已进入 ONNX，entry 约 3.079 秒结束，`navigation_ready=true`、`policy_cmd=0,0,0`。随后新增静止检查失败，**尚未发送非零速度**。累计速度峰值 1.426 rad/s，无原 2 rad/s 超速事件。已确认撤权，节点与监测器退出 0；现场确认静止、稳定且支撑仍在。
- **定位**：entry 完成后轨迹中 SDK16（左肘）有 0.722、1.037、0.459、0.481、0.576 rad/s 的短时样本。报错时实际读取的那帧未记录，不能把最后轨迹帧当作精确触发帧。代码对单帧 `|dq|>=0.5` 立即异常退出，没有等待接管后的稳定过程。这是新增试验检查的缺陷，不能解释为 ONNX 行走失败，也不是右肘放手问题。
- **修复**：保持零速度，最多等待 10 秒，只有连续 3 秒所有关节 `|dq|<0.5 rad/s` 才发令；超出静止门限重置稳定计时，原有 2 rad/s/状态/独占保护仍立即生效。修复已部署，辅助逻辑 5 项及人工流程 4 项在本地、板端均通过（板端合计 9/9）。同时覆盖稳定后再次出现运动时撤销完成状态、重新计时的边界。本轮不自动重试；行走仍未验证。
- **证据限制**：自动审批拒绝完整轨迹复制，理由为内部数据及目的地授权不明确；未重试复制原始记录。仅在板端计算必要统计，原始记录保留板端。不以这些统计证明心跳从未丢失。

**部署与回滚**：人工脚本位于板端 `releases/manual_walk_trial_20261105`，稳定等待补丁及旧脚本备份位于其 `settle_update/backup/`。仅替换 `run_acceptance.py` 和 `manual_pulse.py`，未修改模型、控制器或生产导航话题。回滚时须停用验收进程，并将两个脚本成对恢复。

## 2026-11-05 21:14：静止等待期间新鲜命令帧不足

记录 `acceptance-20261105T211405`：ArmHold 零速度接管后，第二次静止等待 `watch(.1)` 报新鲜帧不足并撤权；现场确认静止稳定且支撑仍在。最后一个成功的 ArmHold 窗口为 0.110 秒、收 5/写 6 帧；失败窗口未保存收帧数及命令年龄。

**已确认原因**：`watch` 在读取初始快照和同步状态服务之前启动 0.1 秒计时。本次初始状态请求耗时 81.032 毫秒，返回写计数 2292，结束状态为 2293；仅写 1 帧低于原有最少 2 帧门限。邻近轨迹中命令到达间隔最大约 20.1 毫秒，不能代替失败时的精确快照。长初始查询吞掉窗口解释了本次不足。修复将窗口计时移至初始快照之后，失败时记录请求及实际时长、初始化耗时、收写数和命令年龄；保留原门限。离线 81 毫秒延迟、帧数不足及命令过期回归与原人工流程测试合计 7 项通过；板端同样 7/7 通过，已部署，尚未再次实机验证。旧脚本保存在板端 `releases/manual_walk_trial_20261105/watch_window_fix/run_acceptance.before_fix.py`，停止验收进程后可恢复。

全部状态采样 `policy_cmd=0,0,0`、`limited_frames=0`；静止门未完成，脉冲程序未启动，不构成行走验证。累计速度峰值 1.332 rad/s，未记录 2 rad/s 超限；撤权前状态为 ArmHold/有效/已授权，撤权后为 Fault/未授权/无发布者，不能将撤权后的 Fault 当作撤权前故障。节点日志未报告推理错误，但没有逐次推理延迟证据。撤权确认，节点和监测器退出码均为 0。不自动重试，保留原保护门限。详见 [板端聚合](../tools/h1_2_policy_hardware/evidence/manual_walk_trial_20261105T211405/aggregate.txt)。


### 2026-11-05 21:20 手动行走超速中止

ArmHoldStand 观测 3.109 秒后进入 ArmHold，状态曾 ready，且已观测非零 `policy_cmd=0.12,0,0`；随后 SDK9 首次采样超 2 rad/s 为 +2.16938，最高 +2.22022。原有准备阶段速度保护触发并确认撤权，节点和监测器退出 0；撤权后电机行为未验证。事件记录不含 `pulse.start`，精确内部触发时刻与超速根因未确定。此事件确实超过了脚本的 2 rad/s 门限，与此前新增工程 precheck 不同，但不能据此称电机超出厂家速度能力或认定步态异常。详见 `tools/h1_2_policy_hardware/evidence/manual_walk_trial_20261105T212015/summary.md`。

**2 rad/s 门限复核**：该值是当前验收工程门限，被统一用于准备及策略运动，未找到其代表硬件安全极限的依据。本轮已出现 `policy_cmd=0.12`，右膝 SDK9 峰值 2.22022；不得再写“未发速度”。原 deploy 中未发现可直接复用的实测 dq 分组保护配置。URDF 的 velocity 分组 9/14/20/23/31.4 rad/s 是模型元数据，不是实机验收授权值；`trans.md` 的 ±1.5 是观测裁剪，也不是运动速度上限。后续应依据明确的实机验证配置区分准备与策略/关节阈值，不能直接用 URDF 值、观测裁剪或删除保护替代。目前只完成调查，未修改阈值，未重试动作。

## 2026-11-05：本轮短试采用分组门限

用户要求取消统一 2 rad/s 限制并再试一次。本轮采用受限调整，不关闭全部速度保护：准备及速度发令前仍为全身 2 rad/s；单次人工速度窗口开始后，腿部 SDK0–11 使用 3 rad/s、上身 SDK12–26 保持 2 rad/s，归零后的持续策略站立和撤权观察沿用这次分组配置。3 rad/s 仅为本轮工程试验门限，不是厂家安全上限。

实现要求：监测器在保留 schema3/原累计峰值的同时增加 27 维逐关节累计峰值；使用分组门限必须验证新字段，禁止退回全身单一峰值而漏掉上身超限。默认配置仍为 2 rad/s；新选项仅用于人工连续试验。CRC、状态新鲜度、控制权、命令超时与急停流程保持。本次仍限定 0.12 m/s 发令 0.5 秒后归零继续 ArmHold，不自动放手。当前状态：已部署并完成专项假数据和只读监测验证；21:36 一次准备因确认后膝门位置门失败而撤权，未进入 ArmHold 或行走。详见下述记录。


### 2026-11-05 21:36 确认后膝位置门失败

`acceptance-20261105T213652` 的 ArmHoldStand 端点左/右膝 q 为 0.963871/0.913728 rad，均在严格区间 0.83–0.97 内；现场 CONFIRM 后复核失败，未请求 ArmHold、未发脉冲。撤权前邻近 trace 左/右为 0.970870/0.918026 rad，左侧高于上界约 0.000870 rad；确切触发帧未保存，原因未确定。累计速度峰 1.374758 rad/s，未记录超限；撤权确认、节点和监测器退出 0，无目标进程。用户报告静止稳定、支撑仍在。分组门限已部署，但本次未进入行走阶段，不能视作实机行走验证通过。详见 [只读聚合](../tools/h1_2_policy_hardware/evidence/grouped_walk_trial_20261105T213652/summary.md)。

## 2026-11-05：减少人工行走试验的额外门槛

用户明确要求减少反复中止试验的条件。人工连续试验取消额外“所有关节静止连续3秒”的要求，仍等待原3秒接管过渡完成、有效状态和软件就绪。膝角容差将作为启动配置统一传给 runner 和 Controller：默认仍为 ±0.07 rad，本次指定 ±0.10 rad，不再因附近 0.97087 rad 的左膝值触发原0.97边界；目标角仍为0.9 rad。这是试验容差调整，不是新硬件安全极限。

保留状态/CRC/时效、控制权、姿态方向检查、命令超时、分组速度保护，以及现场急停。正常发令结束只归零继续策略站立，不调用 release 或降低增益；本次仍只做0.12 m/s、0.5秒单次发令。修改和验证进行中，未将“去掉额外门槛”写成实机行走成功。

## 2026-11-06：按训练统计建立保护配置

训练方原始统计与规则完整保存在 [train.md](train.md)。用户确认两表髋pitch/roll互换后对应当前SDK；配置按名称映射、与当前URDF求交，源码与有效配置均保存hash。新显式`train_20261106`配置使用各关节当前速度、连续3个新状态去抖、接管前2秒×2速度门限、全27关节相对默认误差<0.07、基座角速度2.5及roll/pitch>0.2持续0.5秒保护。触发锁存Passive并持续发送原生阻尼，runner监测后等待人工撤权；不再由Python累计峰值2/3 rad/s抢先切断。旧legacy默认不变。

本地纯保护测试通过，节点/控制核心2/2、配置与人工流程36项通过。当前网卡enp43s0为UP但NO-CARRIER，到板端的路由误走Wi-Fi；两次SSH超时，无法部署和实机执行。没有通过修改网络配置掩盖缺失物理链路，板端旧版未变。证据：[本地验证](../tools/h1_2_policy_hardware/evidence/train_guard_20261106/validation.md)。

## 2026-11-06：LowCmd 启动拦截的来源与归因纠正

`initial['commands'] != 0` 的拦截已在2026-11-02提交 `eac4a99`（新增 bounded hardware policy acceptance runner）中存在：HEAD第183行，当前第505行。它来自验收工具层，不是原导航接口要求，也不是本轮训练保护新增。当前分析错误在于沿用这条检查，并将“收到LowCmd”直接扩大解释为“必须释放或停止底板进程”；发布数据不等同执行控制权。未kill humanoid、未调用ReleaseMode或修改底板模式；当前此检查尚未改动。

新train_20261106包已部署到板端同名release，板端C++3/3、profile2/2、runner3/3通过，正式路径关闭输出校验profile/hash正确、state valid、publisher false、write_count0、退出0。145101试验在授权前观察0.612秒301条LowCmd而中止，策略节点未启动。DDS发现元数据将发布者GUID关联至192.168.123.161上的humanoid进程；仅确认发布来源，不证明其执行所有权。监测器退出时另有CycloneDDS prevent_callbacks断言，不能混为策略故障。后续回到成功链路核对观测/归因，不操作底板进程或模式。

## 2026-11-09：当前 LowCmd writer GUID 可见但进程未归属

**现象**：rc1 的 monitor 只输出预期 `policy_writer_guid`；预期值未配置时，unknown LowCmd 只计数、不暴露来源。

**处理**：rc2 monitor 在只读订阅时按 DDS endpoint GUID 聚合 `lowcmd_writers`；不创建 LowCmd publisher。PC4 5 秒窗口观察到唯一 GUID `0110d3b854fae87cff15d4d100000203`（2493 样本）；DDS discovery 对应 participant `0110d3b8-54fa-e87c-ff15-d4d1000001c1`，另发布 `rt/SymState`。离线测试17/17、目标机编译通过。

**结论与限制**：DDS discovery 输出没有主机地址或 PID；本次 tcpdump 因权限不足未能取得网络源地址，当前进程列表也未提供 GUID 映射。历史 `.161` 归属不能证明本次 GUID 属于同一进程。Owner `--check` 通过，但 Owner/Policy/Nav2 均未启动，未发送运动命令。该 writer 的主机、进程和实际控制权仍未验证；不据此关闭或停止任何 vendor 进程。

rc3 为 writer 统计补充 `last_seen_age`，避免持续运行时把历史 GUID 误认为活跃来源。PC4 已登记 `.161` 的 SSH host key，但 BatchMode 登录被拒；rc3 五秒采样仍观察到同一 GUID 活跃，归属仍未解决。

2026-11-10 PC4 连接恢复后的新窗口观察到 GUID `01109970b33f1c7fcc2df0e400000203`，与前一日 GUID 不同；该 participant 同时发布 `rt/SymState`，但 host/PID 仍不可见。不能将 `.161` 历史来源套用到当前 GUID。PC4 ROS 图未发现 Policy、Coordinator 或 Nav2，未启动控制节点。

## 2026-11-11：当前 LowCmd participant 的发现源定位

PC4 sudo授权后，对 `eth0` 的UDP/7400 RTPS发现流量做35秒只读抓取，snaplen 64字节、最多60包，不含topic数据负载。LowCmd participant前缀 `01100565e7bb65be7fc3ec2c` 出现在源IP `192.168.123.161` 的RTPS头部；该IP的known_hosts Ed25519/ECDSA key与现场key匹配。故当前发现源主机已定位到 `.161`。SSH公钥/密码认证被拒，未取得PID/命令行；历史记录曾将 `.161` 的相关DDS publisher归到humanoid进程，但不能代替本次PID核验或证明控制权。PC4唯一LowCmd writer仍活跃；Owner inactive，导航速度envelope未验收，未启动控制栈或发运动目标。

### 不依赖 `.161` SSH 权限的处理路径

用户指出此前已用 ONNX 人工速度入口让机器人实机行走。记录确认同一 ONNX policy bridge 路径曾执行 `vx=0.12 m/s` 直行 3 秒和 `wz=+0.1 rad/s` 左转 3 秒，现场确认迈步、归零后站稳；这证明 ONNX 运控路径可用，不需要将厂商高层行走接口作为 ONNX 的控制入口。当前 Nav2 设计也将 session/coordinator 速度送入 bridge，由 ONNX 推理并发布 LowCmd。

`.161` writer 来源是后来一次空闲现场快照中的发现信息，不能推翻此前 ONNX 实机行走证据，也不是继续接 Nav2 的产品目标。当前任务应复用已验证的 ONNX bridge 运控入口，将 Nav2 coordinator 作为速度上游。仍需处理的是把当前 RC5 Policy/Nav2 session 在 PC4 正确部署并启动、复核 ONNX bridge 与 Nav2 coordinator 的运行身份及现场状态，以及补齐允许闭环导航的速度包络（历史记录显示右转/负 yaw 尚未验收）。本次没有启动控制栈或发送新运动命令。motion-switcher DDS topic 的只读查询仍可作为诊断手段，但不应表述为 ONNX 必须依赖厂商运动控制接口。

# 2026-11-06 训练保护预接管拒绝

一次授权试验完成 FixStand 和 ArmHoldStand 后，训练 prearm 因 SDK24 误差 0.0838258 rad 超过 0.07 rad 拒绝；未发布非零行走速度。SDK24 是右腕 roll，训练默认角为 0，ArmHoldStand 目标为 -0.076 rad；只读监测约 -0.08244 rad、目标 -0.076 rad，表明准备目标本身与接管门限不相容。SDK23 的准备目标 -0.633 与训练默认 -0.55 也相差 0.083 rad。不能靠放宽 0.07 门限或无修改重试解决；仅训练配置使用匹配训练默认的准备目标是待评估方案，尚未实现或验证。现场保持状态及退出结果以任务文件为准。


## 2026-11-06 训练准备目标与 SDK10 跟踪误差

SDK24 准备目标与训练默认冲突已修复：训练配置采用已校验的默认角作为准备目标，legacy 不变；本地 core/node 集成 2/2 通过并已部署。第二次现场准备在 SDK10 右踝 pitch 实测偏差 0.0731309 rad 时被 0.07 rad 预接管门限拒绝，未发布非零速度；约 95 秒时连续两秒实际位置约 -0.54873 rad、目标 -0.475 rad，持续误差原因尚未确认。用户要求继续保持、暂不撤权；最新状态见任务文件。

## 2026-11-06 预接管门限调整及短脉冲

用户明确将训练配置全27关节预接管工程门限由严格小于0.07改为严格小于0.1 rad；膝门限与其他保护保持不变。定向 trained_guard/node 集成测试2/2通过，包含0.099通过及严格等值拒绝。板端部署后一次短脉冲日志显示策略收到非零命令、随后ArmHold零指令；用户现场确认已迈步且归零后站稳；此次保持中未授权撤权，状态详见任务文件。原0.07为训练统计建议，调整不改变训练数据。

## 2026-11-06 1.5秒试验租约中断

1.5秒单次试验已观察非零及随后零策略命令，期间ready lease expired锁存取消，不能声称完整运行1.5秒；保持ArmHold。0.15秒租约由状态调用续期，现有记录未定位具体超时原因，不应仅因中断放宽保护。记录位于板端 acceptance-20261106T161329，现场反馈和当前交接见任务。

定向fake runner对一次状态调用注入81ms延迟，可复现1.5秒试验的租约过期取消；watch以响应结束为基点等待约0.1秒才再次请求，续租间隔包含下一RPC延迟。现场延迟未记录，该复现不能确证现场根因；用户报告迈步后停稳。

本地修复待部署：有效状态才续租，ArmHold按请求开始时间20ms调度，150ms租约不变。fake 81ms响应延迟下，50ms调度续租间隔0.151秒触发取消，20ms调度为0.121秒并完成；超过租约及ready丢失仍取消。定向16项通过。该模拟说明调度余量不足的可复现机制，不能证明现场RPC耗时。

用户后续要求更久测试，拟单次3秒；本地扩展时长上限到3秒，针对结束归零、2秒后中断与范围边界的测试通过，尚未部署或执行。此前1.5秒试验发生租约取消，不得作为完整1.5秒结果。

## 2026-11-07 repeat部署后的预接管拒绝

初次准备在SDK4误差0.273897 rad时被0.1 rad预接管门限拒绝，未发布非零速度，repeat idle未启动。trace 77.38–79.37秒SDK3、4、9实际相对默认分别约+0.266、-0.266、+0.276 rad，实际输出目标对应训练默认；误差持续，具体物理原因未知。用户报告支撑可靠且稳住；当前保持及交接以任务文件为准，不放宽保护或无改动重试。板端记录 acceptance-20261107T115142。

用户补充：现场支撑主要用于防摔，重量仍由双脚承担；这不证明带载PD静差或控制权为误差的确定根因。当时95666保持ArmHoldStand，WALK入口不可用，本次未运动；不绕过门限或撤权重启。


## 2026-11-07 训练接管角差拒绝

- 现象：记录 acceptance-20261107T115142 在 ArmHoldStand 因 SDK4 默认角差 0.273897 rad 超过旧 0.1 门限拒绝，未发布非零速度；物理偏差原因未确认。旧进程收到 SIGINT，summary 显示撤权确认及节点、监测器退出 0。
- 调整：训练配置仅记录接管默认角差和膝目标差；legacy 膝门限、准备时长、重力、状态和运行期保护保留。板端备份 `releases/entry_diagnostic_20261107/backup.tar`（SHA256 `c1f5a5f2a68feb398c2eb155a579a584c5f423a175f8ba4e6afead755cd95cef`）；源码与产物散列在同目录 `sha256.txt`。
- 验证：本地 core/node 定向测试通过，Fake IO 验证实际位置越软界仍触发保护；板端未授权 status 显示 `entry_angle_gate=diagnostic_only`、`state=valid`、`authorized=false`、`publisher_present=false`、`write_count=0`，检查节点退出 0。记录 acceptance-20261107T171020 报告单次前进3秒窗口结束、非零及随后零命令并处于 ArmHold repeat idle；用户确认持续迈步、归零后站稳；repeat 的 WALK 尚未实机触发。现有 PTY27864 未撤权，后续每次 WALK 需新的明确指令。

## 2026-11-10 Nav2 Owner 配置阻塞

- 现象：Owner 代码拒绝任何 `continuous_policy_navigation:=true` / `enable_navigation_commands:=true`，所以即便现场 envelope 已审查也无法配置 ONNX Nav2；示例 URDF 路径多了一个实际不存在的 `urdf/` 目录。
- 原因与修复：两项导航开关改为必须成对启用，并要求有限合法且 `validated=true` 的 envelope；导航启用还要求 bridge 输出能力参数为 true，以便操作者单独调用授权服务。Owner 本身不授权，启动仍先验证未授权、无 LowCmd writer。默认配置保持 nav/output 关闭。示例 URDF 改为目标机已存在的 `/home/unitree/h1_nav2_ros2_ws/src/h1_2_description/h1_2.urdf`。
- 验证：Owner 与 commissioning 定向测试29项通过，覆盖缺失/无效/越界 envelope、开关不成对及输出能力关闭。PC4 RC4 独立构建与只读核验结果记于导航板端核对清单；现场 writer 归属、地图版本和速度 envelope 仍未验证。

## 2026-11-10 在线建图 Policy 后端未接入

- 现象：GUI 和 shell 只允许离线 `mode=navigation` 选择 Policy；在线 camera_init launch 固定启用 Unitree bridge/smoother，因此不能用 ONNX 运控同时保留在线建图。
- 处理：online 与 offline 都使用启动时 `motion_backend`。camera_init Policy 分支保留 FAST-LIO、点云占据栅格、TF/odom、Nav2参数和 goal 上游；独立 session manager 启动参数相同的 Nav2，并由 coordinator/ONNX 承接速度输出。该分支不启动 Unitree locomotion bridge/smoother；Unitree 默认路径保持原有。
- 验证：GUI/shell 参数测试和 camera-init launch 条件测试通过，Python编译、shell语法及diff检查通过。未在PC4部署GUI，也未启动策略、导航或发运动命令；现场准入仍以硬件清单为准。
