# 任务：20261107-onnx-navigation-development

## 2026-11-18 ArmHold 请求后回到 Passive（原因待核实）

- PC4 网页日志：12:18:23 启动 Policy bridge；12:18:43 authorize、12:19:31 FixStand、12:20:30 ArmHoldStand、12:21:34 ArmHold 服务请求均记为 `success=True`。服务 success 只说明请求被接受，不表示模式随后持续保持。
- 12:22:33 只读 Web 状态显示 `mode=Passive / authorized=true / publisher_present=true / state_valid=true / navigation_ready=false / remote_source=zero`；bridge launch PID12187、node PID12189仍运行，导航栈未运行。故不是 bridge 进程退出，而是控制模式已退回 Passive。
- 参数为硬件 `position_pd`、`train_20261106`、连续导航使能、遥控器共存开启、`max_linear_speed=0.20`。该快照中导航未启动且遥控速度源为 zero，不能据此把 0.20 m/s 设为本次原因。
- 该状态形态与 2026-11-17 已确认的训练 guard 锁存一致；当时原始 status 明确记录 `joint_speed`，SDK index 2 `left_hip_roll_joint` 为 `3.91083455 rad/s`，高于 `2.5 rad/s` 限值。**本次尚未拿到 `trained_guard_reason/index/value/limit`，因此 guard 再次触发只是高概率假设，不是已证实原因。** Web 状态解析器目前丢弃这些字段。
- 随后本机有线 `enp43s0` 为 `NO-CARRIER`，到 PC4 的路由走 Wi-Fi `192.168.8.5`，SSH 超时，未取得本次 ROS 原始日志或更新状态。未发送姿态、停止、撤权或重授权请求。
- 现场先保持支撑，不要重复 ArmHold、撤权或重授权来清除可能存在的 guard latch。恢复有线连接后，先只读抓取 `/h1_2_policy_node/status` 全文和 bridge ROS 日志中的 `trained_guard_*` 事件，再判断处理。

## 2026-11-17 Policy training guard 锁存

- 网页多次尝试 `fix_stand` 均 `success=false`。只读 `/h1_2_policy_node/status` 返回 `Passive / trained_guard_tripped=true / trained_guard_native_damping=true / authorized=true / publisher_present=true / navigation_ready=false / policy_cmd=0,0,0 / state=valid`。触发记录为 `joint_speed`，SDK index 2 `left_hip_roll_joint` 的 LowState 速度 `3.91083455 rad/s` 超过 `2.5 rad/s` 限值，event elapsed `522.570327s`、warmup=false。
- guard latch 会拒绝所有非 Passive 转场；网页状态 API 未返回 guard reason/index/value/limit，造成界面只有通用拒绝信息。guard trip 已回到 Passive 并请求 native damping；该状态不是物理支撑证明。`revoke_output` 会关闭 LowCmd publisher，重新授权会清除 latch 并开放主动准备。
- 仅调用只读 status，无撤权/授权/FSM/导航操作。不要为清除 UI 提示而 revoke/reauthorize 或继续点 FixStand；先保留完整节点/LowState证据，摔倒审查前保持现场支撑并停止控制测试。详见[故障记录](../docs/h1_2-policy-troubleshooting.md)。

## 2026-11-17 在线目标延迟与遥控器接管边界

- 用户日志：16:10:05 `NavigateToPose` 开始目标 `(0.73,1.45)`；16:10:15 controller `Failed to make progress`，清局部代价地图后再次进入 FollowPath；有一次 missed 10 Hz 控制周期。当前在线 camera-init SimpleProgressChecker 为 `required_movement_radius=0.3m`、`movement_time_allowance=10s`，因此前述失败是进度检查触发后同一 BT action 的内部恢复，不是路线 runner 已进入下一个目标。`goal_pose_to_nav2` 对 RViz 新目标异步 cancel 旧 action 后立即发送新 action，存在取消竞态，但没有固定10秒延迟；缺少第二目标时间戳和 action 状态，尚未确认具体等待路径。
- ArmHold 是 Policy bridge 内部 ONNX 状态。当前 bridge 只订阅 `/cmd_vel_final`，未接入 `rt/wirelesscontroller`；`authorized=true` 时它持续拥有软件 LowCmd publisher，故不能把 ArmHold 当作实体遥控器可用的待命态。实体手柄与 Policy 同时操作会形成控制竞争。
- 未查询现场状态、未改代码、未下发命令。摔倒事故仍待审，不进行控制权切换或动作试验。详见[故障记录](../docs/h1_2-policy-troubleshooting.md)。

## 2026-11-17 PC4 部署核验

- 用户授权部署并验证当前停止/RViz修复。PC4经有线 `192.168.123.164` 可达；部署前只读检查确认 ROS 图仅有 Web GUI 节点，`h1-2-web.service` cgroup 只有 Web launcher/GUI 两个进程，没有 Nav2、Policy、FAST-LIO、Livox 或 route runner。
- 备份 PC4 源码、构建/安装产物和入口脚本至 `/home/unitree/h1_nav2_ros2_ws/backup_stop_rviz_deploy_20261117/predeploy.tar.gz`，SHA256 `2bd5c947ca4b3221167da75936e3d49c6d4997d0b95a7203ca9da47d1c4ea819`。同步当前 `h1_2_nav_tools`、`h1_2_slam_bringup`、`h1_2_policy_bridge` 与 `run_ai_slam_nav.sh`/`stop_ai_slam_nav.sh`；没有同步地图或改 DDS 配置。
- 板端 `colcon build --base-paths src --packages-select h1_2_policy_bridge h1_2_nav_tools h1_2_slam_bringup --symlink-install --parallel-workers 2` 成功（3包）。旧 Web cgroup 已确认仅含两个 Web 进程后，重启 `h1-2-web.service`；service active、`http://192.168.123.164:8765/` HTTP 200，`/api/status` 只读显示 navigation/Policy/RViz 均 stopped。在线 camera-init launch 与 Policy bridge `--show-args` 均解析成功；下次 Policy online 启动的线速度上限为 `0.20 m/s`，在线 DWB `max_vel_x=0.20 m/s`，离线 RPP 名义值 `0.15 m/s`。板端源码 SHA256 与本地一致。
- 未点击停止、未启动 Nav2/Policy/Livox/RViz、未授权 LowCmd 或发送目标。故已验证的是代码部署、三包编译、Web服务加载和速度配置；没有活动 SLAM 会话可供停止验证，物理走速仍未知。摔倒故障复盘完成前不做运动测试。功能状态见[停止/RViz计划](../docs/h1_2-stop-rviz-readiness/plan.md)。

## 2026-11-17 本地修复：停止确认、RViz 旧场景与 RPP 速度

- 用户反馈停止 SLAM 后重开 RViz 仍显示异常旧场景，ONNX 导航速度太慢。原因确认：RViz 是 AI PC 上独立窗口，旧 launcher 只能激活既有窗口；Web 对外部导航可能使用陈旧的 Unitree 后端状态且未等待 stop 子进程退出。近期运行的是在线 camera_init+DWB，配置上限已是0.20m/s；离线 RPP 名义速度原为0.10m/s。历史日志没有在线 DWB 输出、bridge输入和实体位移的同步证据，不能把限速认定为原地不动根因。
- 本地修改：Web 停止流程依据真实导航 backend，等待 stop 脚本并验证进程清理；只有成功才下发本机 `h1-rviz://close`。停止脚本只接受明确的 H1-2 导航 launch 文件，避免 PID 复用后误杀其他 H1-2 launch。URI handler 识别 `ros2` 的 Python shim 命令行；优先使用已登记 PID，未登记时仅在唯一找到精确 H1-2 RViz launch 进程组时关闭。交互目标新增 frame 与0.5s TF 新鲜度门禁。RPP `desired_linear_vel` 调至0.15m/s，只影响离线 RPP；在线速度上限不变。
- 验证：全静态测试169 passed；相关单元测试69 passed；TF纯逻辑测试3 passed。全 unit+static 收集因本机无 `builtin_interfaces` 失败；Python compile、两个shell脚本 `bash -n`、RPP YAML解析和 scoped diff check 通过。随后补充导航 launch PID 精确身份匹配及 ROS Python shim/旧 RViz 识别；本轮相关 RViz/停止单元测试25 passed，Python compile、shell语法和 scoped diff 检查通过。
- 未部署 PC4、未关停或重启任何机器人进程、未启动 RViz、未发运动命令。停止流程代码仍待部署验证；在线速度慢的原因及实机速度仍未验证，不能用 RPP 离线参数变化代表在线已提速。此前摔倒事故审查前不进行真机控制测试。详见[功能计划](../docs/h1_2-stop-rviz-readiness/plan.md)与[故障记录](../docs/h1_2-policy-troubleshooting.md)。无提交或保存 hash。

## 2026-11-16 Policy Fault 与摔倒事故（暂停实机控制）

- 用户报告机器人摔倒。PC4 只读确认 Policy bridge PID 4132/4134 的启动 cap 仍是 `0.12 m/s`；PID 4134 在21:21:18报 `Policy fault: state invalid before write`。Fault 后状态 `authorized=false`、`publisher_present=false`、`mode=Fault`、`policy_cmd=0`；`inhibit()` 撤权并关闭 LowCmd publisher。21:28网页心跳超时，导航停止脚本因无法确认 ArmHold而拒绝停止，Nav2/route runner仍存活。
- 该故障与摔倒的时间/机制强相关：position-PD输出停止会失去软件关节支撑；但状态校验哪一项失败及摔倒的完整物理因果尚未确认。当前日志未见21:21之后新导航目标，不能将摔倒归因于本地 `0.20 m/s` 上限调整（没有部署，现场仍为0.12）。
- 21:36网页请求启动新bridge后，PC4出现PID2570；只读状态 `Passive/authorized=false/publisher_present=false/write_count=0/state=valid`，未发现Nav2/route runner。当前尚未授权输出，不能把该软件状态视为机器人的物理复位或安全确认。
- 后续用户日志确认21:47又授权至ArmHold并启动在线Nav2/FAST-LIO；该次启动 `camera_init` TF持续缺失，目标 `(1.32,-0.67)` 被拒。21:48:05收到SIGINT，21:48:33出现再次启动的节点，21:48:56再次停止，21:49显示停止成功。21:51:42只读确认无SLAM/Nav2/Livox/route runner；Policy仍 `ArmHold/authorized=true/publisher=true/navigation_ready=false/policy_cmd=0`。RViz为独立进程，保留上次地图并不代表SLAM仍在跑。点位JSON的frame为`map`而在线要求`camera_init`，不能直接发在线目标。事故后禁止继续发目标。
- 21:55新日志再次确认SIGINT后FAST-LIO/Nav2退出；planner_server在context关闭期间清理异常（exit -6），launch和stop脚本均exit 0。21:56只读进程检查仅余Policy bridge，SLAM/Nav2确已停止；旧RViz场景为独立viewer保留的地图。
- 当前边界：暂停所有真机 Policy/Nav2 控制；保持现场急停、支撑，不远程重启、停止、撤权或复位。先确认人员与机器人现场安全，再保全完整 ROS/Policy/LowState/LowCmd 证据。只有完成故障输出路径审查后才能讨论重新测试。详见[事故记录](../docs/h1_2-policy-troubleshooting.md)。

## 2026-11-16 ONNX 导航目标未产生有效位移；线速度上限调整待真机验收

- 用户反馈给方向后机器人原地踏步。日志：目标 `(1.90,0)` 多次 GridBased plan 失败；目标 `(1.64,-0.63)` accepted 后 `Failed to make progress` 每约10秒重复，随后 TF 比控制数据旧约2.52秒却回报 Goal succeeded。
- 原配置 cap：Nav2 DWB `max_vel_x=0.2`；Policy bridge `max_linear_speed=0.12`、`max_angular_speed=0.10`。按用户要求，本地新启动 Policy bridge/Nav2 线速度上限统一提高到 `0.20 m/s`，角速度保持 `0.10 rad/s`；该值参考旧 Nav2 软件配置，不代表 Unitree SDK 厂商默认或 ONNX 真机已验收速度。旧值的问题日志没有当次 `/cmd_vel_final`、Policy `policy_cmd` 或同步物理位移，不能确认速度 cap 是否为原地踏步根因。
- PC4 20:56 只读 API：route idle；Policy `ArmHold / authorized=true / navigation_ready=true / policy_cmd=0,0,0`。快照发生在目标结束之后。`tf2_echo` 后续恢复可读；无新运动试验。
- 验证：`test_policy_launch.py` 更新为断言 `0.20/0.10` 速度参数；本地测试待运行。当前未部署，运行中 bridge 不会动态变速，未重启现场实例、未发送运动命令。首次真机验收需在安全监护下同步采样 `/cmd_vel`、`/cmd_vel_final`、Policy 状态/指令、`/slam_odom`、TF 时间戳及实体位移。见[故障记录](../docs/h1_2-policy-troubleshooting.md)。

## 2026-11-16 DDS 错误 Wi-Fi peer 定位与修复

- 用户指出雷达为有线连接；确认错误地址 `192.168.8.5` 是上位机 Wi-Fi locator，不是 MID360。PC4 到该地址无路由；雷达 `192.168.124.20` 和数据目的 `192.168.124.164` 均走 `br0`。
- 本机 Policy RViz 的 Fast DDS 原为自动多网卡。新增 `config/fastdds_ai_pc_policy.xml`，只允许有线地址 `192.168.123.99`；启动脚本 Policy 分支加载该 profile，Unitree/Cyclone 分支不变。RViz 已按新配置重启，当前 PID 63561，socket 仅绑定 `192.168.123.99` 的 7400/7410/7411 与动态 UDP 端口。
- PC4 Livox PID 44148 当前使用 `rmw_cyclonedds_cpp` 与 `/home/unitree/h1_nav2_ros2_ws/config/cyclonedds_pc4.xml`；PC4 侧 `/livox/lidar` 暖机后约10Hz。应用新上位机 RViz profile 后，PC4 新 Cyclone ROS graph 查询可发现话题且没有 `192.168.8.5:7410/7411` 错误；上位机 Fast DDS 也可见 `/livox/lidar` publisher。
- 本地启动器 pytest 14 passed；XML、shell 与 scoped diff 检查通过。`/map_preview` 当时 publisher count=0，未验证建图地图显示；未改雷达配置/br0/PC4 Cyclone配置，未操作 Policy bridge/导航/Livox生命周期。记录见[故障文档](../docs/h1_2-policy-troubleshooting.md)。

## 2026-11-16 Livox运行日志与DDS不可达peer

- 用户使用H1-2 Cyclone XML后，Livox识别`192.168.124.20`，成功切Normal、启IMU；Cyclone DDS仍向`192.168.8.5:7410/7411`发送失败。PC4只读路由确认无到192.168.8.5的路由；MID360配置数据目的`192.168.124.164`经br0可达。
- DDS报错属于不可达远端peer通信，不能说明Livox设备握手失败。日志没有消息hz/样本数量；`publish use livox custom format`只说明配置，不证明实际样本。用户Ctrl-C后SDK deinit，进程exit -7，关闭阶段异常原因未知。
- 当前Livox进程已退出。下一次应在运行中用Cyclone环境检查`/livox/lidar`、`/livox/imu`频率，再核验Fast DDS/FAST-LIO是否收点；不改雷达原始目的地址。见[故障记录](../docs/h1_2-policy-troubleshooting.md)。

## 2026-11-16 在线导航 TF 缺失与点位坐标系不匹配

- 用户贴出20:05 `camera_init` Invalid frame ID日志。PC4 Web日志显示会话启动 `mode=online motion_backend=policy`，`start_livox_driver:=false`；FAST-LIO 20:05:06初始化，但本次会话未见动态地图发布，Nav2持续等待 `body→camera_init` TF。
- Livox需外部启动；此前 `.bashrc` 默认加载的旧Go2 Cyclone XML绑定失效接口，是本次无TF的可能原因，但20:05时Livox实际状态无历史快照，尚不能完全确认。下次须按[PC4部署文档](../docs/pc4-migration.md)用H1-2 Cyclone XML启动Livox，并只读确认点云/IMU，再检查TF与动态地图。
- 日志还显示 `/home/unitree/h1_nav2_ros2_ws/maps/tasks/数字钥匙.json` 的 `frame_id=map`，与在线 `camera_init` 不匹配；`memory_point_manager`拒绝混用，20:05:43有一个Nav2 goal被拒绝。在线巡航须重新录制camera_init点位。
- 20:05:57停止操作已结束Nav2/FAST-LIO/route runner；当前bridge仍ArmHold，未重启栈或发送路线目标。细节见[故障记录](../docs/h1_2-policy-troubleshooting.md)。

## 2026-11-16 “停止全部”失败修复与执行

- 网页操作日志确认停止脚本在 `/opt/ros/humble/setup.bash: line 8` 因 `AMENT_TRACE_SETUP_FILES: unbound variable` 退出码1，未到查询Policy或停止导航进程步骤。
- 修改 `stop_ai_slam_nav.sh`：source ROS/Unitree/工作区 setup 时临时关闭 `set -u`，完成后恢复；定向 PC4部署测试7 passed，`bash -n`通过。
- PC4备份 `/home/unitree/h1_nav2_ros2_ws/backup_stop_all_nounset_20261116/`，部署脚本SHA256 `c424265f5bb5141b7d1ddf9737efaccaef268f8f6545c0ca444ed05737a67939`。按原有进程组身份和状态校验重试停止成功，退出码0；Nav2/FAST-LIO/route runner退出且导航PID/backend标记清理。
- Policy bridge 保持 `ArmHold / authorized=true / policy_cmd=0 / navigation_ready=false`，符合当前“停止全部”安全语义：停导航，保持策略站立，不自动撤权。网页API刷新超时12秒；直接 Fast DDS status 查询成功。没有 RELEASE/REVOKE。

## 2026-11-16 Livox Cyclone DDS 网卡绑定错误

- 用户启动 `livox_ros_driver2` 时 Cyclone DDS 报 `wlxf8ce21a12bfe: does not match an available interface`，节点初始化失败。
- PC4只读检查确认活跃机器人网络接口是 `eth0`（`192.168.123.164`）；`~/.bashrc` 加载旧 Go2 配置 `/home/unitree/slam_config/cyclone_go2_B2_ws/cyclonedds.xml`，该XML绑定已不存在的 `wlxf8ce21a12bfe`。H1-2配置 `/home/unitree/h1_nav2_ros2_ws/config/cyclonedds_pc4.xml` 绑定eth0。
- 推荐仅对Livox进程设置 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 CYCLONEDDS_URI=file:///home/unitree/h1_nav2_ros2_ws/config/cyclonedds_pc4.xml`；不修改全局 `.bashrc`，避免改变旧Go2工作区用途。相同环境下 `ros2 launch ... --show-args` 成功；没有在本轮启动驱动，`/livox/lidar` 与 `/livox/imu` 实际发布待现场验证。
- 操作步骤见[PC4部署文档](../docs/pc4-migration.md)。

## 2026-11-16 WebSocket 心跳端口错配修复与恢复

- 用户操作时 `/api/policy/control` 被拒绝：“浏览器心跳未连接或已超时；Policy 控制请求未发送”。独立 Web HTTP/WS实际监听8775/8776，但 `/api/config` 被保存配置中的旧 `heartbeat_ws_port=8766` 覆盖，浏览器因此连到旧服务。
- `WebGuiApp.default_config()` 现于读取保存配置后强制恢复当前实例计算出的 heartbeat 端口。新增回归测试先复现8766覆盖8776，修改后通过；相关测试80 passed，Python compile与scoped diff check通过。
- PC4备份原GUI于 `/home/unitree/h1_nav2_ros2_ws/backup_policy_nav_split_20261116/heartbeat-port-before/`，`h1_2_nav_tools`构建成功，板端GUI SHA256 `5be55acf050f78636652a4c2ce8c42e3677acc7eef091e657922bb59987d0434`。PC4重启后transient `8775/8776`消失；未重启 `h1-2-web.service`，仅恢复独立 `h1-2-web-policy-split.service`，`KillMode=process`。配置API返回8776；WebSocket客户端通过Origin校验并收到 `{"type":"ready"}`。
- 最新只读状态显示 `navigation.running=false`、Policy bridge未运行；未发送授权、FSM、路线或运动命令。刷新 `http://192.168.123.164:8775/` 后心跳应恢复；ONNX流程须依据实时状态和现场准入重新开始。

## 2026-11-16 Web heartbeat startup latch 已修复

- 新版 `8775` Web 页面报“网页心跳曾超时”。只读 `/api/logs` 证实服务刚启动时在首次浏览器 heartbeat 前记录 `browser heartbeat timed out after infs`；这是 watchdog 将 `last_heartbeat=None` 误判为真实断连。现场 Policy 仍 `Passive / authorized=true / publisher_present=true`，route idle。
- 修复 watchdog：首次 heartbeat 到达前不启用断连超时；后续真实 heartbeat timeout 行为不变。新增回归测试，78 项定向测试通过。
- PC4 备份旧 GUI 于 `backup_policy_nav_split_20261116/heartbeat-latch-before/`，重建 `h1_2_nav_tools`，只重启独立 `h1-2-web-policy-split.service`；原 `h1-2-web.service` 及控制栈未重启。新版页面 HTTP 200，`/api/status` 只读可得 Policy 与 route idle，新日志没有 startup timeout。GUI SHA256 `8cdaef895fbcafa2824d179dc633add50560dd28b0f752b0cc83442d65ab80a8`。未执行 FSM 或路线命令。见[功能计划](../docs/h1-2-policy-web-activation/plan.md)。

## 2026-11-16 PC4 部署与只读实机验证

- 备份 PC4 原有脚本、GUI、Fast DDS helper 至 `/home/unitree/h1_nav2_ros2_ws/backup_policy_nav_split_20261116/`；同步拆分启动实现及 helper 等待窗口修正，`h1_2_nav_tools` 构建成功。
- 原 `h1-2-web.service` 的 `KillMode=control-group` cgroup 内正运行 Policy、Nav2、FAST-LIO 和 route runner，现场 Policy `Passive / authorized=true / publisher_present=true / navigation_ready=false`，route idle。没有重启原服务、没有调用授权/FSM/route 操作。
- 新版 UI 独立 user transient service `h1-2-web-policy-split.service` active，地址 `http://192.168.123.164:8775/`，WebSocket `8776`；原 UI `8765/8766` 仍 active。HTTP 200，新页面显示拆分后的控制桥、导航栈和巡航流程；API 首次发现曾超时，Fast DDS helper 冷发现从0.6s调整为2s、响应等待从1.2s调整为4s，外层等待增至8s后只读 API 可读 bridge，route idle。
- 本地定向验证77 passed，Python/shell/JS语法和 `git diff --check` 通过。PC4 GUI/helper/script SHA256及现场状态见[功能计划](../docs/h1-2-policy-web-activation/plan.md)。尚未验证ArmHold、Nav2新启动或任何巡航目标；当前桥已授权，不应重复授权。

## 2026-11-16 ONNX 控制与导航启动拆分（本地完成，未部署）

- 用户确认流程：启动 ONNX 只启动 Policy bridge，并在 Web 内完成 ArmHold 准备；之后单独启动导航栈；最后由“开始巡航”单独发路线目标。
- 本地改动：`run_ai_slam_nav.sh mode=policy_bridge_only` 仅启动 bridge 且不占用 Nav2 锁；Web 独立报告 bridge 状态，准备步骤不依赖 Nav2/route runner；导航 API 强制 bridge 存在且 `control_ready`，再复用 bridge。Policy 的 `start_at_first_route_point` 和 `auto_start_route` 均关闭，巡航保持单独操作。
- 验证：`test/static/test_h1_2_nav_web_gui.py`、`test/unit/test_web_task_profiles.py`、`test/unit/test_policy_web_client.py` 共 75 passed；Python、shell、内嵌 JS 语法及 scoped `git diff --check` 通过。
- 未部署或操作真机。SESSION 记录曾报告 bridge 已授权且存在 LowCmd publisher；部署/重启前需重新确认其当前状态及现场控制权。功能计划见[Policy Web 启动计划](../docs/h1-2-policy-web-activation/plan.md)。

## 2026-11-16 ONNX 网页 status 显示不可用

- 现场 PC4 只读检查：Policy bridge、Nav2、route runner 均运行。Fast DDS CLI 直接查询 `/h1_2_policy_node/status` 成功，状态 `Passive / authorized=false / publisher_present=false / navigation_ready=false`；`/memory_route_status` 为 idle。
- 网页 `/api/status` 首次显示 bridge service unavailable；单独调用网页的 `policy_web_client` 后再次查 API，bridge 为 `available=true`。网页 status/API 顺序检查耗时约 7.6–8.5 秒；helper 内 `wait_for_service` 仅 0.6 秒，冷发现时限可能不足，作为后续客户端修复项。当前屏幕不代表 bridge 未启动。
- 当次导航启动参数包含 `start_at_first_route_point=true`，尽管 route status idle；保持无路线操作，后续核对该参数。未授权输出、未执行 FSM、未发路线目标；未重启 Web 或导航服务。
- 详情见[策略问题记录](../docs/h1_2-policy-troubleshooting.md)。

## 2026-11-16 未授权启动恢复入口部署

- 新增向导内“重试状态”和“取消未授权启动”。取消接口要求 status 可用、`Passive`、未授权、无 publisher、状态有效、无输出转场、route idle，并验证导航/Policy 各自专属 PID、SID、PGID 和命令行后才发送 SIGINT；未知状态与任何门控不满足均拒绝。该接口不调用 FSM 服务。`test/static/test_h1_2_nav_web_gui.py` 34 passed，Python/JS 语法与 diff 检查通过。
- PC4 备份 `/home/unitree/h1_nav2_ros2_ws/backup_onnx_reset_20261116/`；`h1_2_nav_tools` build 成功；新 GUI SHA256 `1f755093e2d2218fc32d0f715b2c8cb3cf24714a5aa7464a6b3e9bff69b508ba`。Web service active、8765 HTTP 200，页面确认包含恢复控件。
- 部署前实时状态为 Passive、未授权、无 LowCmd publisher，路线 idle。Web service restart 后 systemd cgroup 实际清理了 Nav2/Policy 进程，旧 PID 标记已清理；当前 API 显示 navigation stopped。取消 API 没有在活动控制栈上手动触发验证。没有授权输出、FSM 转换或路线目标。

## 2026-11-16 后续状态核查

- 用户页面持续显示 `/h1_2_policy_node/status is not available`。新鲜 PC4 Fast DDS 查询与网页 API 均能读到状态：Policy `Passive / authorized=true / publisher_present=true / write_count=16180 / policy_cmd=0 / navigation_ready=false`；路线 `idle`。Nav2 与 Policy 进程已在用户再次启动后出现，启动命令仍包含 `start_at_first_route_point=true`。
- 这说明页面错误文本过期/状态查询超时，实际桥已授权并在 Passive 下写 LowCmd；没有进入 FixStand/ArmHold 或启动路线。授权来源不明。没有再发送控制或停止命令；不继续 FSM/路线操作，先由现场确认支撑、急停与当前控制权。详情见[策略问题记录](../docs/h1_2-policy-troubleshooting.md)。

## 2026-11-16 最新 Policy 状态复核

- Fast DDS CLI 直接查询成功：`Passive / authorized=true / publisher_present=true / write_count=100214 / policy_cmd=0 / navigation_ready=false`，route 为 idle。网页 `/api/status` 耗时约8.7秒，返回 Policy available 但 route helper unavailable。实际 Debug 状态未知；启动参数确认项不是硬件证明。未发送控制请求；不继续 FSM/路线操作，先现场核实支撑、急停和控制权。见[策略问题记录](../docs/h1_2-policy-troubleshooting.md)。

## 2026-11-16 ONNX 控制栏整理与 PC4 部署

- 将原先分散的 ONNX 状态整理成导航栈、运动控制、巡航任务三段摘要；顶部为唯一启动入口，后端选择只显示“仅配置”的提示。向导分开呈现现场准入、准备步骤和下一步说明，窄屏弹窗宽度改用视口约束。
- 验证：`python3 -m pytest -q test/static/test_h1_2_nav_web_gui.py` 34 passed；GUI Python 编译、内嵌 JS `node --check`、scoped `git diff --check` 通过。桌面与窄屏 headless Chrome 生成 mock 截图；Chrome 窄屏 viewport 与截图尺寸不一致，窄屏需真实浏览器复核。
- PC4 `192.168.123.164` 已部署：先将旧 GUI 备份到 `/home/unitree/h1_nav2_ros2_ws/backup_onnx_ui_20261116/`，仅替换 `h1_2_nav_web_gui.py`，板端 `h1_2_nav_tools` build 成功；`h1-2-web.service` 重启后 active，8765/8766 监听，HTTP 200，页面含三段 ONNX 状态元素。新 GUI SHA256 `e51f0841f8793fb09365c619463fcc4b3faa005eea4420fdbef090a0f0f5a454`，备份旧版 SHA256 `fd7274eda3b05d061b7c35b29419f41e31de1eecfdc6993187f982abd214e9bb`。
- 部署前后均无导航、Policy 或 route runner 进程；未启动或授权 Policy、未发路线目标/运动命令。本机 mock 页面已停止；PC4 页面供用户浏览器强制刷新查看。窄屏需真实浏览器复核。
- 详情见[Policy 网页启动向导计划](../docs/h1-2-policy-web-activation/plan.md)。

## 2026-11-16 网页栏内滚动

- 用户反馈屏幕较小时网页各栏内容被裁切且滚动条不明显。现调整设置/任务/控制三列和中间任务内容为独立纵向滚动，强制给 Chromium/WebKit/Firefox 提供可见滚动条；路线表仍可横向滚动；窄屏断点保留每栏最大高度与拖动滚动。
- 定向静态测试34 passed，Python编译、内嵌JavaScript语法、`git diff --check`通过。详情见[滚动体验计划](../docs/h1-2-nav-web-scroll/plan.md)。
- 本机 mock 交互预览已重启，`http://127.0.0.1:8765/` 返回 HTTP 200；以 1440×700 和 1100×700 视口检查布局。页面模拟状态，不连接机器人。PC4 未部署。

## 2026-11-16 Policy 网页启动向导（本地完成，未部署）

- 针对“选中 ONNX 不清楚是否启动”的问题，原网页新增明确的“启动 ONNX 模式”入口和分阶段浮窗；向导说明选择只是配置，启动后读取实时 Policy 状态，逐步呈现输出授权、FixStand、ArmHoldStand、ArmHold。主页面保留简明状态摘要，进入 ArmHold 不会自动开始巡航。
- 启动前在向导集中确认 Debug、独占控制、LowCmd writer 交接、支撑和急停；原服务端门控不变。Policy 导航启动参数无条件关闭 `auto_start_route`，避免旧配置造成准备期间自动发目标。
- 本地验证：`python3 -m pytest -q test/static/test_h1_2_nav_web_gui.py test/unit/test_web_task_profiles.py` 70 passed；`python3 -m py_compile .../h1_2_nav_web_gui.py`、内嵌 JS `node --check`、`git diff --check` 通过。
- PC4 `192.168.123.164:22` 在本轮连接超时，因此没有备份/替换板端源码、构建或重启 Web 服务；板端页面仍是旧版。恢复连接后再部署本 UI 更新。没有启动 Policy/Nav2、授权输出、发送路线目标或进行真机动作。
- 功能计划：[Policy 网页启动向导](../docs/h1-2-policy-web-activation/plan.md)。

## 2026-11-14 原导航网页 Policy 准备入口部署

- 用户要求将 Policy 输出授权和站立准备入口放入原导航开发网页。现已在 `h1_2_nav_web_gui.py` 增加状态面板、现场确认和逐步 `authorize_output → FixStand → ArmHoldStand → ArmHold` 控制；服务器按 Policy 实例、bridge 当前模式/状态/转场、路线 idle 和浏览器心跳拒绝跳步。页面不自动开始路线，也明确 writer 身份需现场独立核验。
- Policy 栈使用 Fast DDS，网页主体使用 Cyclone；新增短时隔离 Fast DDS ROS 客户端，处理 bridge 状态/准备服务、Policy 路线 status/start/stop、在线 TF readiness 和点位保存。Unitree 路径未变。
- 本地验证：网页/客户端定向 pytest 72 passed；ROS Humble 隔离构建 `h1_2_nav_tools h1_2_policy_bridge` 成功，bridge CTest 7/7；Python compile、JavaScript `node --check`、`git diff --check` 通过。
- 已部署 PC4 `/home/unitree/h1_nav2_ros2_ws`：仅替换 GUI 与新增 helper，执行 `colcon build --base-paths src --packages-select h1_2_nav_tools --symlink-install --parallel-workers 2` 成功；重启 `h1-2-web.service` 后 active，8765 HTTP 200，页面含 Policy 面板。部署备份位于 `backup_policy_web_activation_20261114T172108/`。板端两个文件 SHA256：`fd7274eda3b05d061b7c35b29419f41e31de1eecfdc6993187f982abd214e9bb`、`f888f9bb1f8ad069fb791eeba0d8404973166e079812d8f1eba76db73ab31af6`。
- 部署时只读进程检查没有 Policy bridge、Nav2 或 route runner；空现场确认 API 请求在人工确认门控处被拒绝。未调用 Policy/FSM/route 控制服务、未授权输出、未发目标或运动。LowCmd writer 来源/控制权没有在本轮刷新核验；Livox 当前运行状态也没有复查。
- 下一步：浏览器强制刷新；Livox 需确认/启动后再选 online + policy。任何授权前，由现场核实 LowCmd writer 已交接、Debug/独占、支撑和急停准备。功能计划与完整操作次序见[Policy 网页准备计划](../docs/h1-2-policy-web-activation/plan.md)。

## 2026-11-14 在线模式启动

- 用户报告已启动 Livox 并要求启动在线模式。PC4 当时 Livox 节点存在，环境使用 Cyclone DDS 与当前 `cyclonedds_pc4.xml`；用户配置为 `online + policy`、自动巡航关闭。随后由主工作区 `run_ai_slam_nav.sh` 启动 `h1_2_nav2_camera_init_navigation.launch.py`，显式 `start_at_first_route_point=false`，使用 `camera_init` 参数和任务文件 `maps/tasks/数字钥匙.json`，未发送 Nav2 goal。
- 在线栈已运行：FAST-LIO 收到 cloud #90（1706 points，696 accepted），占据栅格持续发布（至少 map #15），`/map_preview` 可见；`memory_route_status` 为 idle。Livox/FAST-LIO/Nav2/route runner 与 Policy bridge 均存在。
- Policy status service 经 Fast DDS CLI 请求超时（8秒），尽管 ROS graph 可见该服务；无法确认此刻授权/发布者状态。Policy bridge 与 memory_route_runner 持续报告 Fast DDS RTPS reader history `24 bytes > 11 bytes`。因此不启动路线、不宣称控制链 readiness；继续保持无目标状态，排查该 DDS history 错误和 Policy status 超时。此时 `/map_preview` subscription count 为 0，RViz 尚未订阅。
- 验证命令：PC4 `ros2 topic list -t`（Fast DDS）见 `/livox/lidar`、`/map`、`/map_preview`；`ros2 topic info -v /map_preview` 为 `OccupancyGrid` transient-local publisher；`ros2 service call /memory_route_status ...` 返回 idle；Policy status 查询 timeout。无另存 hash。

## 2026-11-14 Web 心跳停止与外部实例

- 在线栈由 SSH 终端启动，Web GUI 因 `/tmp/h1_2_ai_slam_nav.pid` 与 backend 文件识别为 external，故网页再次点击启动时提示“导航已由其他终端启动，请先接管或停止当前实例”。代码当前没有外部进程 attach/接管实现。
- 浏览器的多个 WebSocket 在收到正常关闭码 1001 后断开；最后一个心跳超过 2 秒触发 watchdog。GUI 写下“stopping navigation”，但 `/nav_process` 仅代表 GUI 自己创建的子进程；外部 launch 并未停止。15:05:53 复核仍有 `run_ai_slam_nav.sh`、Nav2、Policy 与 route runner 进程；路线服务为 idle，没有 goal。地图持续更新。停止日志不可视为外部进程退出。
- 下一步保持当前单实例，避免重复启动。若继续在线显示，保持至少一个页面/心跳连接；在修复外部实例 attach/stop 逻辑前，不依赖 GUI watchdog 停止此类外部实例。Policy status 仍有 RTPS reader history 错误且查询超时，不开始巡航。

## 2026-11-14 原导航平台 PC4 部署

- 已按用户要求部署到原主工作区 `/home/unitree/h1_nav2_ros2_ws`，不是 `8781` 手动测试页或独立 `8770` GUI。源码/配置经 `scripts/pc4_deploy.sh` 同步；板端 12 包 `colcon build --base-paths src --symlink-install --parallel-workers 2 --cmake-args -DUNITREE_SDK2_ROOT=/home/unitree/unitree_sdk2 -DORT_ROOT=/home/unitree/h1_nav2_ros2_ws/third_party/h1_policy_runtime/onnxruntime` 成功。依赖已安装，故本次未跑 rosdep。
- 初次启动失败因 DDS XML 指向不存在的 `wlxf8ce21a12bfe`；实测 PC4 仅 `eth0` 活跃。已将 `config/cyclonedds_pc4.xml` 绑定 `eth0` 并重新同步。服务 `h1-2-web.service` active，HTTP 8765 返回 200，8765/8766 均监听；页面 HTML 包含 Policy/ONNX。旧 `build/`、`install/` 保留为同名 `.pre_20261114_web_update` 备份。
- 仅启动 Web GUI；未启动 Nav2/Policy bridge、未授权控制输出、未发送导航目标或运动命令。下一步：用户浏览器强制刷新，online + policy，新建任务并在当前 `camera_init` frame 重录点位。任何真机目标前仍须新鲜核验 LowCmd writer、控制权及现场准入。
- 验证：PC4 12 包构建成功；`systemctl --user status h1-2-web.service` 为 active，`ss -ltnp` 显示 0.0.0.0:8765/8766，`curl http://127.0.0.1:8765/` 返回 200；无控制进程。源代码/板端版本未计算 hash。

## 2026-11-13 原导航 Web 平台接入在线 ONNX 目标控制

用户澄清要使用旧导航开发平台 `h1_2_nav_web_gui.py`，而非 `manual_test_web.py` 或新建页面。已修复 online Policy 下的平台任务点位与导航坐标不一致：在线点位保存于 `camera_init`，路线 runner 使用 `camera_init -> body` 新鲜 TF 并拒绝其他 frame 点位；旧 GUI 对在线导航使用相同 TF readiness，离线仍用 ICP/map。Policy 巡航入口还要求 bridge `authorized=true`、`mode=ArmHold`、LowState valid、publisher 存在、navigation enabled/ready；不替用户授权或做姿态准备。网页启动导航后会保留所选任务；匹配模式/后端时也可将终端启动的实例接入当前任务，然后记录当前位置并开始巡航。

本地验证：相关静态/任务测试 85 passed；Python/launch `py_compile` 与 `git diff --check` 通过；source ROS Humble 和当前 workspace `install/setup.bash` 后隔离构建 `h1_2_nav_tools h1_2_slam_bringup` 2/2 通过。首次未 source overlay 的构建因依赖 package.sh 不在空临时 install 失败，未修改项目 install；随后 source 现有 overlay 后构建成功。未部署/重启网页、未启动控制栈、未授权输出、未发 Nav2 goal。部署后需在在线模式新建任务并记录 `camera_init` 点位；旧 `map` 点位会被明确拒绝。功能计划见[导航计划](../docs/h1_2-onnx-navigation/plan.md)。

## 2026-11-12 用户要求架构收敛

用户要求按奥卡姆剃刀只保留 `Nav2 cmd_vel -> 既有速度平滑/限幅 -> /cmd_vel_final -> ONNX bridge -> LowCmd`，Unitree/Policy 在启动时二选一。标准 Nav2 action、既有 smoother、直接 Policy bridge 已接好；移除了 Owner/Coordinator、token/epoch、claim、action proxy 与 session manager；训练保护 profile 和硬件输出/撤权保护保留。停止导航先确认 bridge ArmHold，bridge 不随 Nav2 进程退出；仅在确认 `authorized=false` 后提供单独 bridge-stop。验证：隔离构建 3 包、bridge CTest 6/6、Web GUI 31项和定向 launch/config/navigation 测试通过；未部署或操作机器人。详情见[功能计划](../docs/h1_2-onnx-navigation/plan.md)。

## 2026-11-12 简化框架板端部署

按用户要求部署到独立 release `/home/unitree/h1_nav2_ros2_ws/releases/onnx-navigation-20261112-simplified-rc1`。板端 x86_64 / ROS Humble overlay 构建 9 包，Policy bridge CTest 7/7，在线 camera-init 与离线 Nav2 `--show-args` 通过。Policy 模型 SHA256 `3090836d0ce454a35161982b60dde20e0fd87627150e06126aabd1e6bbfabae7`；二进制与入口 hash 及现场边界见[板端核对清单](../docs/h1_2-onnx-navigation/hardware-validation-checklist.md)。8765 GUI 仍运行主工作区版且 HTTP 200；未启动 bridge/Nav2、未授权输出或发送动作命令。剩余事项：新鲜只读核对 writer/控制权，具备现场准入和当次授权后再做真实 Nav2 闭环验收。

## 2026-11-12 闭环测试准入阻塞

用户要求开始真机闭环，成功后再考虑从隔离区合并。现场只读复核发现唯一活跃 LowCmd writer GUID `01105c981ac532473b7c2d3200000203`（5秒 2488 samples），来源主机/PID及控制权未知；ROS 图仍有既有 Unitree 和 `sport_switch`，无 Policy/Nav2。CRC 0、状态新鲜只能说明 DDS 数据在流动，不能确认独占控制。未启动控制栈、授权输出或发命令，因此闭环测试未开始，合并条件未满足。需管理员确认 writer 控制权、现场急停/支撑/debug/独占状态并确定短测授权范围后再复核。详见[核对清单](../docs/h1_2-onnx-navigation/hardware-validation-checklist.md)。

用户尝试启动 release 时发现 `config/cyclonedds_auto.xml` 放错在 release 根目录。已复制到 `config/`，SHA256 与本地一致；该次脚本在启动阶段报错退出，复查没有 Policy/Nav2 进程。闭环准入阻塞仍在，修复配置不代表可开始运动测试。

用户要求通过 Web 控制导航，避免命令行发起测试。release 版 Nav GUI 已单独运行于 loopback `8770`（心跳 `8771`），并能识别当前 shell 启动的 online Policy stack；Fast DDS GUI 支持已重构到 release，相关 65 项测试通过。当前 bridge 仍 `authorized=false / Passive`，未发目标。现场查询确认 `/slam_odom` 和 `camera_init -> body` TF 存在，但 `map -> camera_init` 不存在；现有路线点服务固定使用 `map`，GUI 等待 ICP localization，因此 online route 暂不可用。下一步先补在线 `camera_init` 目标/点位链路和 readiness，再继续短测；旧手动脉冲网页继续用于单次速度动作，不是 Nav2 action 控制页。见[核对清单](../docs/h1_2-onnx-navigation/hardware-validation-checklist.md)。

## 最新进展（2026-11-11）

### ONNX 人工短测网页

2026-11-12 用户反馈单次后需要撤权才能继续。已修复网页 repeat：一个 runner/Policy 会话准备一次，成功 pulse 且确认非零命令与归零后显示“执行下一次”；方向、速度、时长以单个有界 TRIAL 请求提交，不排队。`REVOKE` 仅结束整场会话；读取/策略/租约/速度故障后不再允许重复。legacy terminal WALK 兼容。

fake workflow 覆盖同一进程左转 `(0,0,0.05)` 1秒、前进 `(0.08,0,0)` 0.5秒，两次均观察非零/零且准备阶段只执行一次；相关测试96 passed（76条既有NumPy reload warning），py_compile/diff-check通过。无 runner/Policy 现场动作。

部署前复核PC4无网页、runner、Policy 进程；部署到独立 release `/home/unitree/h1_nav2_ros2_ws/releases/onnx-manual-test-20261112-rc2`。网页 PID6092，仅监听127.0.0.1:8781，HTTP200、API idle。主工作区/8765 GUI不变。现场步骤与hash见[手动测试网页计划](../docs/h1_2-policy-manual-test-web/plan.md)。

用户希望启动网页后选择前进/后退/转向、速度与时长。新增独立仅 localhost Web 控制台 `tools/h1_2_policy_hardware/manual_test_web.py`，PTY 驱动原 `run_acceptance.py`，不新增 LowCmd publisher；仅单会话，方向为前/后/左/右，线速度上限0.12m/s、角速度上限0.1rad/s、时长0.5–3秒，网页初始要求调试模式、独占控制、支撑/姿态三项现场确认，并分别等待 runner 姿态确认和脉冲结束后的支撑撤权。源码身份与训练 URDF 自动解析，不再要求操作者输入 commit 或 URDF。反向/右转只作为有限速人工待验方向，不记为已验证。

定向 runner/web 测试89 passed；py_compile、shell语法、diff check通过；CLI dry-run 展示后退`(-0.08,0,0)`，未连接 ROS、启动子进程或执行硬件动作。未部署到PC4；现场使用需先将该版本release到板端，再于板端运行网页并用SSH隧道访问。功能事实及验证见[网页测试计划](../docs/h1_2-policy-manual-test-web/plan.md)。

### 不依赖 `.161` SSH 的交接排查

用户提醒此前已通过 ONNX policy bridge 实机发送速度并让机器人行走。历史记录确认直行 `0.12 m/s` 3秒、左转 `+0.1 rad/s` 3秒，并有迈步及归零后站稳的现场确认；无需把厂商高层运动接口当作 ONNX 运控入口。Nav2 代码路径为 session → coordinator → `/h1_2/policy/cmd_vel` → ONNX bridge → LowCmd，与已验证的 ONNX 后端相同。此前查得 `.161` 是空闲快照的 LowCmd DDS discovery 源；这不否定 ONNX 的行走能力，也不是产品接入目标。PC4 当前 Owner inactive、无 Policy/Nav2 action。下一步聚焦部署/启动当前 RC5 路径并按现场门控核验；闭环速度 envelope 的右转/负 yaw仍待验收。未启动控制栈或发新运动命令。motion-switcher 查询只是可选诊断，不是 ONNX 的依赖接口。

用户将目标聚焦为先验证在线建图模式，并明确ONNX只替代Unitree运控、上下游尽量不变。PC4独立RC5构建、camera-init/Policy session参数和离线验收17/17通过。用户开启的Livox driver发布`/livox/lidar`约8Hz、`/livox/imu`约200Hz。RC5 Policy backend camera-init限时运行中，FAST-LIO收到至少30帧点云，动态栅格发布至少45帧1600×1600 `/map`；`/slam_odom`和TF未独立确认。测试节点退出，点云路径在`/tmp`。Owner/coordinator未就绪，未启动Nav2 session/Policy运动输出。LowCmd writer GUID `01100565e7bb65be7fc3ec2c00000203` host/PID未知。详见[RC5交付记录](../docs/h1_2-onnx-navigation/rc5-online-mapping-delivery.md)。

- 状态：进行中；更新时间：2026-11-11 Asia/Shanghai。
- 下一步：补验 `/slam_odom` 与 `camera_init -> body` TF，并做更长的在线地图稳定性观察；若再进行ONNX行走测试，先完成LowCmd writer身份和速度包络准入。地图文件不覆盖。
- 授权边界：本轮授权推进部署与只读核查；没有启动控制节点、切换 GUI 服务、授权输出或发送运动命令。

### 2026-11-11 行走短测准入续查

用户要求由我执行行走短测。PC4新鲜monitor发现唯一LowCmd writer GUID `01100565e7bb65be7fc3ec2c00000203`、约2987 samples/6秒、CRC0、mode6/PR0、状态age约0.00054秒。`ros2 topic info -v /lowcmd`报`_CREATED_BY_BARE_DDS_APP_`；participant `01100565-e7bb-65be-7fc3-ec2c000001c1`同时发布`rt/SymState`。用户提供sudo后，我在PC4 eth0仅抓取UDP/7400 RTPS发现包头（snaplen64、35秒），用participant GUID前缀匹配到源IP `192.168.123.161`；host key已验证但SSH认证被拒，PID/控制程序和实际控制权未知。历史记录提示`.161`相关发布者为humanoid进程，但不是当前PID证据。Owner inactive、Nav2 action不存在，RC5 Owner关闭输出配置`--check`通过但未启动，速度包络（右转/双向yaw）未现场验收。未启动控制栈、未发Nav2目标。下一步由`.161`管理员确认进程/控制权并完成速度包络验收。详情见[RC5交付记录](../docs/h1_2-onnx-navigation/rc5-online-mapping-delivery.md)。

## 最新进展（2026-11-10）

用户已将当前优先目标明确为在线 camera_init 建图，并要求 ONNX 只替代 Unitree 运控接口、保留上游 SLAM/动态占据栅格与 Nav2。代码现支持 online `motion_backend=policy`：保留在线映射节点，Policy 分支由独立 Nav2 session/coordinator承接速度输出，不启动Unitree bridge/smoother；GUI和shell开放在线选择。RC4尚不含本轮修改，PC4 GUI仍旧版。最近PC4样本中的LowCmd writer GUID `0110dd0b58ac41ca67ae942a00000203` 未映射host/PID，现场速度envelope未验收。本轮本地软件测试通过，但当前有线口NO-CARRIER、SSH超时，无法制备或部署新板端release；没有现场控制动作。

用户要求继续到可交付开始 Nav2 测试。PC4 独立 RC4 已构建并通过只读软件核验，详情和 hash 见[RC4记录](../docs/h1_2-onnx-navigation/rc4-delivery.md)。修复 Owner 无条件拒绝连续导航的问题：导航开关必须成对启用，并通过合法已验证 envelope；导航配置要求输出能力可由操作者单独授权，Owner 不授权，默认配置仍关闭。定向 Owner/commissioning 29项通过。RC4 monitor最新只读6秒采样唯一活跃LowCmd GUID `0110dd0b58ac41ca67ae942a00000203`，2988条state/command、CRC 0、mode6/PR0；GUID未映射host/PID。地图hash与此前PC4一致但PGM/PCD异于开发机。PC4 ROS图无Policy/Coordinator/Nav2、action为空，Owner inactive；当前8765 GUI仍从主工作区install运行，RC4 GUI未切换。用户要求我执行只读验证时，本机有线口已NO-CARRIER，SSH超时且到PC4的路由误走Wi-Fi，未能刷新现场状态。因此**现在还不能从网页选择ONNX开始Nav2行走测试**。writer身份、地图版本和现场envelope未解决。未启动Owner/Policy/Nav2、未授权输出或发送运动命令。详见[板端核对清单](../docs/h1_2-onnx-navigation/hardware-validation-checklist.md)。

- 状态：进行中；更新时间：2026-11-10 Asia/Shanghai。
- 负责人：root 实施与协调；本轮 commissioning live 路径修复、Owner fake graph 测试已分别完成并审阅。
- 目标：按[六项计划](../docs/h1_2-onnx-navigation/development-plan.md)实现常驻 ONNX 控制与独立 Nav2 会话，停止导航保留零速 ArmHold。
- 范围：代码、测试、文档及继续推进到真机 Nav2 测试准备。已部署 rc2/rc3 独立 release；PC4链路已恢复，本轮完成 SSH、ROS graph、Web GUI HTTP 的只读检查，未重测writer monitor或地图。非目标：覆盖主工作区 install/地图、安装/启动 systemd Owner、启动 Policy/Coordinator/Nav2、切模式、授权控制输出或发送运动命令；上述均未执行。
- 当前摘要：本地 `main` 基线 `364cd8a`，修改未提交/push。Policy 导航入口、独立 session、hold-first stop、速度 envelope、mock commissioning 已实现。PC4 rc1 五包构建和桥接 CTest 6/6 通过；rc3 monitor 增加 `lowcmd_writers` GUID 与 `last_seen_age`。2026-11-10 的只读快照见本任务最新进展；该 writer host/PID 和控制权未确认。目标 `h1_2_lab.yaml` hash 与开发机相同，PGM/PCD 不同，未覆盖。Owner `--check` 曾以目标实际路径通过，输出/nav关闭且无 envelope。未启动控制节点或发送运动命令；主工作区、Web 服务和现有进程未覆盖。
- 功能状态、通过标准及证据以[开发计划](../docs/h1_2-onnx-navigation/development-plan.md)为准；SDD 过程文件在隔离副本 `.superpowers/sdd/development-plan/`。
- 下一步：先按清单只读核实当前 writer/控制权和地图版本；由现场人工验收生成 envelope；经现场准入后才配置 Owner/启动授权及Nav2短测。RC4只完成独立构建，没有安装或启用Owner/GUI，当前未达到直接行走测试准入。
- 完成条件：各软件任务实现并通过必要本地验证和审查；现场验收单列未验证，不能以 mock 代替。
- 授权边界：用户要求继续到可真机 Nav2 测试，并于2026-11-10要求完成所有功能。本轮完成本地软件/构建/fake ROS graph，并对PC4仅做SSH、ROS graph与HTTP只读核验。没有安装/启动现场 Owner、Policy、Coordinator、Nav2，没有切模式、授权控制输出或发送运动命令。真实运动仍需writer/地图/envelope等现场准入和当次范围授权；历史现场授权不延续。

## 验证

- 2026-11-10 在线 camera_init Policy 接入：GUI/shell入口测试62 passed；ROS Humble `test_navigation_backend.py` 6 passed，覆盖在线Policy保留FAST-LIO/动态栅格、启动session manager并排除Unitree bridge/smoother；`colcon build --packages-select h1_2_nav_tools h1_2_slam_bringup`隔离构建通过；camera-init `--show-args`有`motion_backend`。`bash -n`、Python compile、`git diff --check`通过。未启动现场节点。

- 2026-11-10 17:54 Asia/Shanghai：用户要求执行实机只读核验。三条 `ssh -o BatchMode=yes -o ConnectTimeout=5 unitree@192.168.123.164 ...` 均连接超时；`ip -br link` 显示 `enp43s0 DOWN/NO-CARRIER`，`ip route get 192.168.123.164` 走 Wi-Fi `192.168.8.1`，`ping -c 2 -W 1 192.168.123.164` 0/2。未执行远端命令，未启动节点或发送动作。等待有线链路恢复后再复核。
- 2026-11-10 17:59 用户确认连接后复核：有线链路恢复、SSH成功。RC4 `SHA256SUMS` 全通过；ROS图只有原有Unitree/WebGUI节点，action空，Owner inactive，GUI HTTP200但仍使用主工作区install。monitor只读6秒 2992 state/2987 command、CRC0、age0.000072428秒、mode6/PR0，唯一活跃GUID `0110dd0b58ac41ca67ae942a00000203`，unknown_commands=2987、policy_commands=0，host/PID未映射。地图hash与此前PC4相同。没有启动控制节点或发运动命令；见核对清单最新段。

- 2026-11-10 PC4 RC4独立交付：`/home/unitree/h1_nav2_ros2_ws/releases/onnx-navigation-20261110-rc4`；源码归档 SHA256 `0bd15eb646585b37c42b9a7796f86ea09b18f01f6343c59d07e6da86729598b1`。五个 ROS 包构建5/5，Policy Bridge CTest 6/6，离线 acceptance 17/17；Nav2 session `--show-args`、禁输出/禁导航 Owner `--check`、临时 mock envelope 的 nav-enabled `--check` 通过。Policy binary SHA256 `3ece6216cb15404df54fbac53fc0447f035e3aad08a374db9509aff5e603e0bc`；monitor SHA256 `deb334fe27531c75eecf3dbb91363967e34610af6ea5c96b501558c126c7ecc5`；Owner脚本SHA256 `2a80e8896b3aa782d7977fbb96a26ab9fbc15c6b8850dd63ebf830a6fc4a8862`。`SHA256SUMS` 校验通过。其后RC4 monitor只读6秒：2988 state/command，CRC0，唯一GUID `0110dd0b58ac41ca67ae942a00000203` 未映射host/PID；地图hash复核一致但PGM/PCD与开发机版本不同。PC4 ROS图仅原有Unitree/WebGUI，action为空，Owner inactive；未启动控制节点、未授权输出、未发运动命令。完整记录见 `rc4-delivery.md` 和硬件核对清单。

- 2026-11-10 本轮 Owner 配置修复：`source /opt/ros/humble/setup.bash && PYTHONPATH=src/h1_2_nav_tools:tools/h1_2_policy_hardware:$PYTHONPATH /usr/bin/python3 -m pytest -q tools/h1_2_policy_hardware/test_policy_owner.py tools/h1_2_policy_hardware/test_nav_commissioning.py`：29 passed；`python3 -m py_compile tools/h1_2_policy_hardware/policy_owner.py` 及 scoped `git diff --check` 通过。新增配置测试覆盖缺失/未验证/越界 envelope、nav 标志不匹配、缺少输出能力；验证不启动任何控制节点。

- `git status --short`：主目录本轮开始时为空；本地 main 与 origin/main 均为上述基线。未重新联网核验远端。
- 2026-11-09 收尾修正：owner systemd unit 改用 `KillMode=mixed`；新增 `policy_owner.validate_child_executables()` 并接入 `--check`，缺失 monitor/coordinator executable fail fast；修复 manager 重复声明 `use_sim_time` 和未确认 session 子进程退出即标记 stopped。配置相关 unittest 2 passed；隔离 ROS overlay 下 owner 22 passed、coordinator fake ROS 4 passed、manager/coordinator/fake action lifecycle 含 token goal 与 hold 1 passed；`py_compile` 和 `git diff --check` 通过。主工作区 `install/` 的接口 Python typesupport 不完整，因此使用已构建隔离 overlay 验证。目标机 `/home/unitree` 路径不存在，不能核对板端配置。未启动真实 owner/Nav2 或连接机器人。
- 2026-11-09 PC4 release 部署：源码归档 SHA256 `da84c1227974d36c548614a94a86f761094eebfe655de31e721a4b52dcfa95cd`。PC4 release 构建 5 包、Policy Bridge CTest 6/6、Owner `--check` 和 `policy_nav_session.launch.py --show-args` 通过。只读 ROS 图显示没有 Policy/Coordinator/Nav2；独立 Monitor 3 秒观测 1493 LowState、1488 LowCmd、CRC 0、age 0.000956 秒、mode 6/PR 0，预期 writer GUID 未配置且来源未知。未安装/启动 Owner、未启动控制节点、未发送动作。停止条件和记录见 `hardware-validation-checklist.md`。
- 2026-11-09 rc2 跟进：`test_acceptance_offline.py` 17/17；PC4 编译 monitor 成功，5 秒只读采样 2493 state/command，CRC 0、age 0.001719 秒，唯一观察 GUID `0110d3b854fae87cff15d4d100000203`。`ddsls` 将其映射到 participant `0110d3b8-54fa-e87c-ff15-d4d1000001c1`，并发现同一 participant 发布 `rt/SymState`，但没有 host/PID。tcpdump 因权限不足未执行。PC4 实际路径 Owner `--check` 通过；配置使用主工作区现存模型/URDF、rc2 policy executable/monitor，硬件输出和 nav 参数关闭，envelope 未配置。无控制节点启动、无运动命令；更新证据见 `hardware-validation-checklist.md` 和 `test-plan.md`。
- 任务 1A 已实现并审查通过：`00691a01284269266c895a373e87411f8a85fe81`、`c729fe232f03ae9964f362ac592cfcb0a904af7b`；17 项纯状态测试与接口包构建/原生类型支持加载成功。命令和过程证据见功能开发计划。
- 任务 1B：隔离 ROS 假节点和纯逻辑定向测试 24 项通过；`h1_2_nav_tools` 及依赖包构建 5 包完成；报告见 `.codex_tmp/onnx-navigation-dev/.superpowers/sdd/development-plan/task-1b-report.md`。验证不连接机器人。
- 2026-11-09：owner runtime identity 接入后，本地定向 pytest 57 项通过；`colcon build --packages-up-to h1_2_policy_bridge h1_2_nav_tools` 6 包完成；bridge `node_integration` CTest 1 项通过。构建目录为 `/tmp/h1nav-owner-colcon-{build,install,log}`，source `/opt/ros/humble/setup.bash`；shell 语法、Python compile 与 diff check 通过。未运行任何控制节点、owner、monitor、Nav2 或机器人现场操作。未解决：完整 ROS graph fake-service 整体启动联测及目标板路径核验。
- 2026-11-09 本轮：ROS Humble 下 `colcon build --packages-select h1_2_nav_tools h1_2_slam_bringup` 2 包通过（`/tmp/h1nav-deliver-{build,install,log}`）。定向 GUI/owner/launch/coordinator node/speed-limit/Nav2 参数派生 98 passed；独立 ROS coordinator fake-node 4 passed；人工脉冲、commissioning、离线验收 34 passed；验收 workflow 43 passed。`ros2 launch h1_2_nav_tools policy_nav_session.launch.py --show-args` 通过；默认 commissioning 输出 `action_sent=false`。更新 `test_acceptance_offline` 过期 fixture 后相关离线测试通过。`bash -n`、Python compile、`git diff --check` 通过。未启动 owner、monitor、Nav2、控制器或连接机器人。
- 2026-11-10：`python3 -m pytest -q tools/h1_2_policy_hardware/test_nav_commissioning.py` 6 passed；覆盖异常仍请求 hold、goal acceptance 等待计入时限、失败 lifecycle 不得成功。隔离 ROS Humble overlay 下 `tools/h1_2_policy_hardware/test_owner_graph_ros.py` 1 passed：真实 owner claim 客户端验证假 runtime identity/GID，并由真实 coordinator 接受 claim。工作区 overlay 缺接口 C typesupport，隔离构建解决；`py_compile` 与 scoped `git diff --check` 通过。完整 owner 子进程、Nav2 goal/hold 生命周期仍未覆盖。PC4 SSH 因本机以太网 `NO-CARRIER` 超时；未做板端操作或运动命令。
- 2026-11-10 继续：owner supervisor fake runtime integration 加入真实 owner CLI/ProcessSupervisor 与 coordinator 子进程；确认 active output 时 stop 后 owner 保持，inactive 后 owner 返回0。独立 context 加入 coordinator 测试构造，仅供依赖注入，默认运行无变化。合并运行4个定向 ROS/commissioning 文件：9 passed，带1条 rmw response timeout 清理 warning；改变测试 domain 隔离后再次运行仍为9 passed。`colcon --log-base /tmp/h1nav-final-log build --packages-select h1_2_nav_tools --build-base /tmp/h1nav-final-build --install-base /tmp/h1nav-final-install` 成功。SSH `unitree@192.168.123.164` 超时；开发机有线 NO-CARRIER，未做板端动作。

## 交接日志

- 2026-11-07：确认旧临时实现不可用，改用项目内隔离副本；明确 prepare 先分配接口、activate 检查 readiness 的启动顺序，避免启动循环。未进行机器人操作。
- 2026-11-08：Task 1B ROS coordinator/action 完成本地验证，新增 ready-loss、重复 writer 及 hold 与迟到 backend acceptance 竞态覆盖；仅 localhost 假节点，未连接机器人。1C 以 1B 本地提交为基线继续。
- 2026-11-09：审查发现旧 launch 的 `motion_backend=policy` 仍会启动进程组内 bridge，stop 脚本也会发 Unitree UDP stop 并强杀。已将 Policy 改为依赖外部常驻 owner 的 prepare→独立 session→activate，GUI/shell stop 先 hold；Unitree stop 顺序保留。Coordinator 未加载批准 envelope 时 nav 仍返回未验证并拒绝启动；需完成 fake graph 和现场另行验收。

## 2026-11-17 Policy 与遥控器速度共存实现

- 用户要求恢复“导航速度指令 + 物理遥控器输入”共存。已在唯一 ONNX Policy bridge 内新增可选 `enable_remote_velocity`：读取 `LowState.wireless_remote`，按 SDK `REMOTE_DATA_RX` 解码；非中立摇杆覆盖新鲜导航速度，摇杆回中后恢复原 `navigation_ready`/0.4 秒超时门控。yaw 符号按 Unitree state-machine 示例采用 `-rx`。
- 未启动旧 Unitree Loco bridge，也未创建第二个 `rt/lowcmd` writer；未绑定未确认的 FSM 按键或 `ReleaseMode`。原有授权、ArmHold、训练保护、现场支撑和急停边界保留。
- Web 设置新增“ONNX 与遥控器速度共存”选项，默认开启并传递给 Policy bridge；状态显示当前速度源。默认 CLI/launch 参数仍关闭，便于非网页启动保持原行为。
- 本地 `h1_2_policy_bridge` 与 `h1_2_nav_tools` 构建成功；仲裁、硬件、launch 参数、训练保护、core、ONNX regression 定向测试通过。node integration 在本机 DDS 锁/长时间无输出，未计为通过。未部署 PC4、未授权输出、未发送真机运动命令。详情见 `docs/h1_2-policy-remote-arbitration/plan.md`。

## 2026-11-18 遥控器速度共存部署 PC4

- 用户要求部署到真机。PC4 `192.168.123.164` 在线；部署前只读核查 `h1-2-web.service` active，cgroup 仅有 Web launcher/GUI，无 Policy bridge、Nav2、route runner 或 FAST-LIO 进程。
- 已先将旧源码备份至 `/home/unitree/h1_nav2_ros2_ws/backup_remote_velocity_coexist_20261118/source-before.tar.gz`（SHA256 `b613ffe51053ef78e7814e753d12e046a78c8066dad463c815192cfe5db15a8c`），再只同步共存功能涉及的 Policy bridge、启动脚本和 Web GUI 文件。
- PC4 `h1_2_policy_bridge`、`h1_2_nav_tools` 构建成功。由于 PC4 CTest 初始模型路径默认指向 `/home/llx/policy.onnx`，重建时显式设置 `H1_POLICY_TEST_MODEL_PATH=/home/unitree/h1_nav2_ros2_ws/models/h1_2_armhold/policy.onnx`。`colcon test --base-paths src --packages-select h1_2_policy_bridge --ctest-args -E node_integration` 7/7 通过；跳过 node_integration 是本地该 DDS 集成测试已挂起且无运动相关必要，不作为本次硬件验证。
- Web 服务重启后 active；HTTP 8765 返回 200，HTML 包含“ONNX 与遥控器速度共存”，配置 API 返回 `enable_remote_velocity=true`；安装 Policy 二进制 SHA256 `c5355598cfc88a7c9a5ae5956995d934285953ec35da1778439c6f9efc729cfd`。
- 本次只部署软件并重启仅含 Web 的 cgroup；未启动 Policy/Nav2、未授权 LowCmd、未发运动命令。部署不代表实机摇杆方向/运动已验收；FSM 按键交接、摔倒后物理安全仍未验证。详见[共存计划](../docs/h1_2-policy-remote-arbitration/plan.md)。
