# H1-2 ONNX 导航板端核对清单

### 2026-11-14 在线模式启动（未发目标）

- 用户确认 Livox 已启动并要求启动在线模式。PC4 主工作区导航启动参数为 `mode=online motion_backend=policy`，路线自动开始关闭且 `start_at_first_route_point=false`；没有发送 Nav2 goal。
- FAST-LIO 收到 cloud #90（1706 点，696 点通过滤波），动态栅格持续发布（至少 map #15）。`/map_preview` 存在，RViz subscription count 为 0；`memory_route_status` 返回 `idle`。
- Fast DDS Policy status 服务请求 8 秒超时，虽然服务在图中可见；Policy bridge 与 route runner 有重复 RTPS reader history 错误：payload 24 bytes 大于 history 容量 11 bytes。当前授权状态/bridge readiness 未能查询确认，不能开始路线。需先定位 Fast DDS 接收 history 错误并让状态服务可读；保留自动巡航关闭。
- Livox 使用 Cyclone DDS，导航和 Policy bridge 使用 Fast DDS；这说明当前两类 RMW 运行并行，但尚未证明该混用是 24/11 字节错误的根因。验证时未进行机器人动作或发 goal。

状态（2026-11-11）：PC4独立RC5已部署；在线Livox→FAST-LIO→动态`/map`短测通过（见[RC5记录](rc5-online-mapping-delivery.md)）。当前行走短测仍未准入：LowCmd writer `01100565e7bb65be7fc3ec2c00000203` 对应participant `01100565-e7bb-65be-7fc3-ec2c000001c1`，RTPS发现包源IP已确认是`192.168.123.161`，PC4登记host key匹配，但SSH认证失败，PID/控制程序及实际控制权未确认。历史记录将`.161`相关publisher归为humanoid进程，仅作旁证。Owner inactive、Nav2 action不存在，也没有批准速度包络（右转/双向yaw待现场验收）。RC5 Owner `--check` 在输出/导航关闭时通过，但未启动Owner或动作链。本文不构成运动动作授权。

### 2026-11-12 简化导航框架独立部署

- 用户授权将最新简化导航框架部署到真机。Release：`/home/unitree/h1_nav2_ros2_ws/releases/onnx-navigation-20261112-simplified-rc1`；仅部署并构建，没有启动 bridge、Nav2、run script 或发送动作命令。
- 板端 x86_64 / ROS Humble overlay 构建 9 包成功；Policy bridge CTest 7/7；在线 camera-init 与离线 Nav2 launch `--show-args` 均通过。构建使用板端 ONNX Runtime `/home/unitree/h1_nav2_ros2_ws/third_party/h1_policy_runtime/onnxruntime` 与 Unitree SDK `/home/unitree/h1_nav2_ros2_ws/third_party/h1_policy_runtime/unitree_sdk2`。
- Policy 模型 `/home/unitree/h1_nav2_ros2_ws/models/h1_2_armhold/policy.onnx` SHA256 `3090836d0ce454a35161982b60dde20e0fd87627150e06126aabd1e6bbfabae7`。启动脚本已补模型路径参数，默认从 release 路径解析至主工作区模型，可用 `H1_2_POLICY_MODEL_PATH` 覆盖，并在启动 Policy 前检查可读；本轮没有启动控制栈验证此运行路径。
- Policy binary `libh1_policy_core.so` SHA256 `8dcfa1cd83900fb544459a534ea2c4cd4b81cff4689cd4459377538b02d8e0ba`；`h1_2_policy_node` SHA256 `e389008b8f73f8ce99b363270c0f9a1e3b3c5dca5cd01cee34da9e5728ded48a`；启动脚本 SHA256 `8c2a561dcfbf605b95db4e2ea4552a93c1f7186d48b84536225f82e62ce23dad`。
- 部署后 ROS 图仍只有既有 Unitree/WebGUI 节点、action 列表为空；8765 GUI 仍由 PID 1619 从主工作区运行且 HTTP 200。主工作区、现有服务及地图未覆盖。
- 部署验证不等于导航准入。历史 LowCmd writer 主机/进程和控制权仍未确认，在线建图实时状态须复核；Policy bridge 运行、速度闭环和 Nav2 运动均未验证。启动前需重新核对现场 writer/控制权、急停、支撑、debug、独占控制及当次运动授权。

### 2026-11-12 闭环测试准入复核

- 用户要求开始真机闭环测试，并在成功后评估从隔离区合并。PC4 `2026-11-12T20:41:31+08:00` SSH 可达；只读进程/ROS 检查未发现 Policy bridge、Nav2 或导航启动脚本，现有 Web GUI PID 1619 仍从主工作区运行。ROS 图为既有 Unitree/WebGUI 节点，action 列表为空；额外看到 `sport_switch` PID 1795。
- `ros2 topic info -v /lowcmd` 显示一个裸 DDS publisher。用既有 monitor 做 5 秒只读采样：2488 LowState、2488 LowCmd、CRC 0、state age `0.001565649` 秒、mode 6 / PR 0、峰值关节速度 `0.0719948262 rad/s`。唯一 writer GUID `01105c981ac532473b7c2d3200000203`，2488 samples、last_seen_age `0.000023405` 秒；host/PID 和实际控制权未知。telemetry 在 `/tmp/h1-closed-loop-preflight-20261112/telemetry.json`。
- 因存在来源及控制权未明的活跃 LowCmd writer，且现场急停、debug mode、支撑与独占控制未获得本轮新鲜确认，未启动 Policy bridge/Nav2、未授权 LowCmd 输出、未发 goal 或运动命令。闭环测试未开始，结果未验证；暂不从隔离区合并。需先由现场管理员确认该 writer 对应进程/控制权，并完成现场准备和明确短测范围授权，再重做准入检查。

### 2026-11-12 启动前配置文件修复

- 用户执行 release 启动脚本时，脚本报告找不到 `config/cyclonedds_auto.xml`，在 DDS 环境检查阶段退出。检查发现文件曾被放在 release 根目录，未进入 `config/`。
- 已将本地 `config/cyclonedds_auto.xml` 复制到 release 正确位置；板端 SHA256 `62ff10a05ac92618450dce4b2916b43b0a7da6d763341f86fe1f6f66bfbafb48` 与本地一致。复查无 Policy bridge、Nav2 或启动脚本进程；该次命令没有启动控制栈或发送运动命令。

### 2026-11-12 Web 导航控制台接入

- 用户随后从 release 启动了在线 camera-init + Policy 导航。当前 stack 进程为 shell PID `17147`、Policy launch PID `17179`、camera-init Nav2 launch PID `17209`。ROS 图含标准 Nav2 action 和 smoother。Policy status 为 `authorized=false`、`mode=Passive`、`navigation_ready=false`、`publisher_present=false`；没有发 Nav2 goal。
- 为避免旧 GUI 使用 Cyclone DDS 而看不到此 Policy 图，在 release 的导航 GUI 加入 `H1_2_RMW_IMPLEMENTATION` 选项（默认 Cyclone，Policy 控制台显式用 Fast DDS），并增加对由 `run_ai_slam_nav.sh` 启动的外部导航进程识别。板端重新构建 `h1_2_nav_tools` 成功；本地导航 GUI 单测/静态测试 65/65 通过。
- 新版 GUI PID `21254`，HTTP `127.0.0.1:8770`、WebSocket 心跳 `127.0.0.1:8771`，HTTP 200；API 识别导航 `running=true, external=true, run_mode=online, motion_backend=policy`。原 8765 GUI、8781 手动脉冲网页和正在运行的 bridge/Nav2 均未重启。访问需通过 SSH 隧道转发 8770、8771。
- 现场 `/slam_odom` 有 1 个 publisher，`camera_init -> body` TF 可查询；`map -> camera_init` 不存在。在线 Nav2 全局 frame 是 `camera_init`，但当前 memory point manager / route runner 将点位固定存为 `map`，GUI 巡航 readiness 又等待 `/icp_localization/status` 和 `map -> camera_init`。所以网页虽接入当前 stack，网页路线目标仍未具备正确的在线坐标系准入；尚未发目标。Policy 未授权前禁止发送目标。需为在线模式提供独立 `camera_init` 点位/目标链路，并以新鲜 `camera_init -> body` TF 作为在线定位准入，完成软件测试后再做现场短测。

最新 RC4 独立构建和配置核验记录见 [rc4-delivery.md](rc4-delivery.md)。它没有改变本清单的现场准入状态。

## 1. 部署前只读核对

### 2026-11-10 RC4最新只读样本

- RC4独立release源码归档SHA256：`0bd15eb646585b37c42b9a7796f86ea09b18f01f6343c59d07e6da86729598b1`。release校验、构建和Owner关闭配置核验见[RC4交付记录](rc4-delivery.md)。
- 使用RC4 monitor采样约6秒，文件位于PC4 `/tmp/h1-nav-rc4-readonly.WuxhQR/`：2988 LowState、2988 LowCmd、bad CRC 0、最后LowState age `0.000677328`秒、mode 6 / PR 0、峰值关节速度 `0.05828 rad/s`。唯一writer GUID `0110dd0b58ac41ca67ae942a00000203`，2988 samples，last seen age `0.000028237`秒；`unknown_commands=2988`，`policy_commands=0`。DDS发现尚未映射host/PID；不能推定其控制权或与历史GUID相同。telemetry SHA256 `d4b81526d9c7cd311003c1bc170c85a99f63c926f8d8bf4212de207b9d142c0f`，trace SHA256 `595c20e7a1b62b8d2f4cdb618b527b2d9947f6cd88c09113f51baffbe5f90f10`。timeout向monitor发SIGINT，返回124是采样窗口结束，不是数据错误。
- 当日地图 `/home/unitree/h1_nav2_ros2_ws/maps/h1_2_lab.yaml`、PGM、PCD SHA256依次为 `0494cc2fce2266225aaca0717a229c9827fc5f134a64070b93313810c65b4598`、`e478470c8e72eb7e63fa9854835e68f0bfe3a44e80f9b95b3025cd024208603f`、`743bd1f9936a018b8a41fee9cb6ce12de8de19c18f09f80ad23a02a2cd02df03`。与历史PC4值相同；PGM/PCD与开发机不同，版本未确认。未覆盖地图。
- 仅运行只读monitor和hash检查；未启动Owner、Policy、Nav2，未授权输出或发送运动命令。

### 2026-11-10 17:59 用户确认连接后的刷新

- 本机有线口 `enp43s0` 为 UP，地址 `192.168.123.99/24`；路由直达 `192.168.123.164`。PC4 SSH 返回 `unitree-h1-2-pc4`，时间 `2026-11-10T17:59:10+08:00`。
- RC4 `SHA256SUMS` 全部通过。ROS图仍只有 `/QtServer`、`/bms_ctr_node`、`/dog_control_pub`、`/h1_2_nav_web_gui`、`/rsLidarCheak`、`/switch_node`；action为空，Owner为inactive。8765 GUI HTTP 200，运行命令仍来自主工作区 `install/`，不是RC4。
- monitor只读6秒：2992 LowState、2987 LowCmd、CRC错误0、状态age `0.000072428`秒、mode 6 / PR 0、峰值关节速度 `0.08057 rad/s`。唯一writer GUID `0110dd0b58ac41ca67ae942a00000203`（2987 samples，last_seen_age `0.000024428`秒），`unknown_commands=2987`、`policy_commands=0`；host/PID和控制权未确认。telemetry SHA256 `3a6500fd21703024146f03f4fbe3ca2a35307880f00e9dff86cfea0751c97da4`，trace SHA256 `343078655728078a35921a62586ebf2f80a1be42b4c0a60c55cbc6220f159309`。timeout发SIGINT后返回124，为窗口结束。
- 地图YAML/PGM/PCD hash与前次相同（见上）。只运行了hash、ROS图、HTTP及只读monitor检查；未启动Owner/Policy/Nav2，未授权输出、未发运动命令。writer仍未知，因此停止在准入阶段。

在目标机、目标 ROS 域和当前已运行系统中记录：

- 主机、时间、ROS 发行版、`ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY`、RMW 实现及 workspace install 路径。
- 现有控制/导航/owner/monitor 进程 PID、启动命令、进程组和代码版本；确认没有第二个 policy 或 LowCmd writer。
- Policy runtime identity：owner boot ID、node FQN/PID、model/URDF/executable/profile SHA256、授权状态、writer 状态和 16-byte LowCmd writer GUID。
- 新鲜 LowState、CRC/状态计数和 monitor schema/lifetime；状态过期、身份不符、未知 writer 或 monitor 异常时停止，不启动新节点。
- `ros2 node list`、`ros2 topic list` 及 action server 清单，记录 policy ready、coordinator services 和 Nav2 action 的实际 namespace。

不得通过启动第二个 bridge/owner 来“探测”这些信息；只使用已部署的只读 status/monitor 接口。保存脱敏后的命令输出、时间戳和产物 hash。

### 2026-11-09 PC4 部署记录

- Release：`/home/unitree/h1_nav2_ros2_ws/releases/onnx-navigation-20261109-rc1`；源码包 SHA256：`da84c1227974d36c548614a94a86f761094eebfe655de31e721a4b52dcfa95cd`。目标机主工作区 `src`、`install` 和 Web 服务未改；Owner systemd 未安装或启动。
- x86_64 / ROS Humble 上独立构建 `h1_2_policy_interfaces`、`h1_2_nav_msgs`、`h1_2_policy_bridge`、`h1_2_nav_tools`、`h1_2_slam_bringup` 共 5 包通过；Policy Bridge CTest 6/6 通过。release Monitor 已编译，SHA256：`72e26bbf3e9c6f05cb31b55c8645300b25b538879227c48a0b3dfc037d9e4be2`；Owner `--check` 与 session launch `--show-args` 通过。
- 只读 ROS 图未发现 Policy 节点、Coordinator 服务或 Nav2 action；发现 `/bms_ctr_node`、`/dog_control_pub`、`/switch_node`、`/rsLidarCheak` 和 Web GUI。3 秒 Monitor 窗口收到 1493 条 LowState、1488 条 `rt/lowcmd`，CRC 错误 0、状态 age 约 0.001 秒、mode 6 / PR 0。writer GUID 预期值未配置，`policy_writer_guid` 为空；Monitor 无法将这些命令归属到具体 writer。不能据此判定其来源或控制权唯一，因此停止后续节点启动。
- 未启动 Policy Bridge、Coordinator、Owner、Nav2 或运动控制节点；未发送运动命令。下一步先通过现有只读状态工具确认 `rt/lowcmd` writer 身份和控制权；身份未明前不启动新控制进程。现场批准 envelope 和动作验证仍未完成。

### 2026-11-09 rc2 只读 writer 核对

- 为弥补 rc1 monitor 只报告 expected GUID、看不到未知 GUID 的问题，rc2 仅在 monitor telemetry 增加 `lowcmd_writers`（DDS writer GUID 与窗口内样本数）；代码 SHA256 `39baa269b8950a691ed46d417cbd3295011232b2fede1367d1ff5fe5139e9dc3`，PC4 编译产物 SHA256 `dd028c4805937f291578ec39f0b94bdf03963052384936379aee0a05aec483e5`。独立 release：`/home/unitree/h1_nav2_ros2_ws/releases/onnx-navigation-20261109-rc2`。
- 定向离线检查 `python3 tools/h1_2_policy_hardware/test_acceptance_offline.py`：17/17 通过；`git diff --check` 通过。PC4 使用目标 SDK 编译成功。5 秒只读 monitor 窗口：2493 条 LowState、2493 条 LowCmd、CRC 0、age 0.001719 秒、mode 6 / PR 0；只观察到一个 writer `0110d3b854fae87cff15d4d100000203`（2493 samples）。
- DDS discovery 将 endpoint 关联至 participant key `0110d3b8-54fa-e87c-ff15-d4d1000001c1`；同一 participant 同时发布 `rt/SymState`。`ddsls` 未给出主机地址或 PID；当前 PC4 进程清单中没有对应身份字段。历史记录中 `.161` 曾出现 vendor humanoid 来源，但本次 GUID 不同，不能据此认定当前来源。尝试本机抓包因 `tcpdump` 无抓包权限失败，因此**writer 主机/进程和控制权仍未确认**。
- 用真实路径执行 rc2 `policy_owner.py --check` 通过：策略 executable 在 rc2，模型 SHA256 `3090836d0ce454a35161982b60dde20e0fd87627150e06126aabd1e6bbfabae7`，URDF SHA256 `504b0cc9ea5311044876f4be4e3148fbc285aa1439e33b19d93c357a7468066c`；硬件输出显式关闭、导航参数显式关闭，envelope 未配置。检查不启动任何子进程，不等同 Owner graph 验证。
- PC4 现有 ROS 图仍只有既有 Unitree 节点/Web GUI；未发现 Policy、Coordinator 或 Nav2。未修改主工作区、现有服务或进程；未安装/启动 Owner、Policy、Nav2，未授权输出或发送运动命令。下一步需要通过 DDS writer 的主机/进程身份资料确认现有控制源，并在独立现场验收后形成批准 envelope；完成前不启动控制节点。
- rc3 为 writer 列表增加 `last_seen_age`，避免长期运行时把历史 GUID 误认为当前活跃来源。代码 SHA256 `bf0e763d774472958fd7d4da438af33f605446fab12cb4aebcfea4661b55568a`，monitor 二进制 SHA256 `deb334fe27531c75eecf3dbb91363967e34610af6ea5c96b501558c126c7ecc5`；离线检查17/17通过，PC4编译成功。5秒只读窗口2487条LowState/LowCmd、CRC0、age0.001603秒、mode6/PR0；唯一GUID `0110d3b854fae87cff15d4d100000203` 的 `last_seen_age=0.000026` 秒。
- DDS discovery 没有给出此 GUID 的 host/PID。PC4 对 `.161` 的 SSH host key 已存在，但只读 SSH 登录返回 `Permission denied (publickey,password)`；tcpdump 因权限不足无法取得网络源地址。故当前 writer 主机/进程和控制权仍未确认；需由有权限的管理员/现有监控提供归属证据，再继续 owner graph。

### 2026-11-10 PC4 只读复核

- PC4 当日可达，主机 `unitree-h1-2-pc4`、ROS Humble、ROS domain 0。只读图中有 `/QtServer`、`/dog_control_pub`、`/switch_node`、`/bms_ctr_node` 和 Web GUI；`ros2 action list` 为空，未发现 Policy/Coordinator/Nav2。
- rc3 monitor 5秒窗口：2488 LowState、2489 LowCmd、CRC0、age0.001529秒、mode6/PR0；唯一 LowCmd writer GUID `01109970b33f1c7fcc2df0e400000203`，观察到2489个样本，`last_seen_age=0.000022` 秒。其 DDS participant 同时发布 `rt/SymState`。公开 DDS discovery 仍无 host/PID，writer来源和控制权未确认。
- PC4 地图 `/home/unitree/h1_nav2_ros2_ws/maps/h1_2_lab.yaml` SHA256 `0494cc2fce2266225aaca0717a229c9827fc5f134a64070b93313810c65b4598`；对应 PGM SHA256 `e478470c8e72eb7e63fa9854835e68f0bfe3a44e80f9b95b3025cd024208603f`，PCD SHA256 `743bd1f9936a018b8a41fee9cb6ce12de8de19c18f09f80ad23a02a2cd02df03`。YAML 与开发机相同，图像和点云不同；未覆盖目标文件，需现场确认应使用的地图版本。
- 本轮只读图、monitor 和 hash 查询；未启动 Owner/Policy/Nav2、未授权输出或发运动命令。Nav2 准入仍待 writer 身份/控制权、地图版本和现场 envelope 核实。

### 2026-11-10 后续连接复核

- 用户确认 PC4 地址仍是 `192.168.123.164` 并重新连接。本机 `enp43s0` 状态为 `NO-CARRIER`，`ip route get 192.168.123.164` 选择 Wi-Fi 网关 `192.168.8.1`；`ssh -o BatchMode=yes -o ConnectTimeout=5 unitree@192.168.123.164 ...` 超时。故上面的 PC4 图、GUID、地图与 hash 是此前采样，不代表本次当前状态；等有线 carrier 恢复后先重做只读核验。
- 本次仅本机软件开发与 localhost fake ROS 测试；未连接或操作机器人，没有安装/启动 Owner、Policy、Nav2 或发送运动命令。

### 2026-11-10 PC4 接口恢复与只读检查

- 时间：PC4 `date -Is` 返回 `2026-11-10T11:30:06+08:00`。本机 `enp43s0` 已为 UP/LOWER_UP，地址 `192.168.123.99/24`；路由 `192.168.123.164` 经该接口直达。`ssh -o BatchMode=yes -o ConnectTimeout=5 unitree@192.168.123.164 'hostname'` 返回 `unitree-h1-2-pc4`，TCP/22 可达。
- ROS Humble 环境只读节点清单为 `/QtServer`、`/bms_ctr_node`、`/dog_control_pub`、`/h1_2_nav_web_gui`、`/rsLidarCheak`、`/switch_node`；`ros2 action list` 为空。Service/topic 清单未发现 `/h1_2/policy/*`、Policy ready 或 Nav2 action 接口；仍有既有 `/lowcmd`、`/lowstate` 等 topic。GUI `curl --noproxy '*' -sS -o /dev/null -w ... http://192.168.123.164:8765/` 返回 HTTP 200、59,231 bytes。
- rc3 release 的 `policy_owner.py` 可读，SHA256 `ad9448d2d3e68ec29ff082ff3e562ef399914d658a3fb5cd6cd6b859e0d6c863`。`ros2 node list` 未见 Policy、coordinator 或 Nav2；Owner/Policy/Nav2 均未启动。本次未启动 monitor，因此此前 GUID 快照不是当前 writer 核验。本次只读网络/ROS graph/HTTP 检查，没有启动控制节点、授权输出或发送运动命令。

## 2. Owner 配置

复制 `tools/h1_2_policy_hardware/systemd/h1-policy-owner.env.example` 后，必须逐项替换并 review：

- workspace setup、URDF、model、直接 policy executable 的目标机绝对路径及架构适配。
- bridge 命令中的接口、domain、profile 与输出初始关闭状态。
- monitor 命令必须指向目标机已审查、可执行的持久 monitor；不得保留 `/absolute/path/to/...` 占位路径。
- 导出目标环境后先运行 `policy_owner.py --check` 校验命令格式、模型/URDF/executable、运行 profile 和 monitor/coordinator 启动程序是否存在且可执行；该检查不证明 ROS graph readiness 或 monitor 行为，也不连接 ROS。此阶段不安装 systemd、不启动 owner。
- 只有现场人工短脉冲得到单独批准并保存 evidence 后，才配置 envelope 文件；否则保持未配置，导航准入应失败关闭。

## 3. 现场分阶段验收

每一步都需当时有效的明确范围授权、重新核对控制权/支撑/急停和运行 hash；前一步失败即停止。

1. 输出禁用时验证 owner/coordinator/monitor 身份与唯一输入 writer claim；确认没有 LowCmd writer。
2. 按独立人工试验方案补验右转及双向 yaw，记录每次命令、时长、ready/guard、归零和现场观察。不得由软件测试替代。
3. 保存获批 envelope evidence，配置后重启前复核路径与 hash；无证据时不准入 Nav2。
4. 先做一次不超过 3 秒的 Nav2 命令链短测，不要求到达；确认目标被取消、coordinator hold 成功、速度归零、仅 Nav2 session 退出且 owner/coordinator/monitor 仍在。
5. 目标到达和更长时间导航属于后续单独授权，不由本清单自动批准。

记录结论为通过、未通过或未验证，并保存命令、summary、trace、日志、版本/hash 和现场观察。任何意外运动、状态失效、writer 冲突、hold 未确认或停稳不明时，停止追加目标，按现场急停与人工支撑流程处理。

### 2026-11-10 17:54 本机尝试刷新现场状态

- 三条只读SSH检查均超时。本机 `enp43s0` 为 `DOWN/NO-CARRIER`，去 `192.168.123.164` 的路由走 Wi-Fi 网关 `192.168.8.1`，ping 2次均无响应。
- 未执行远端校验、monitor或ROS graph命令；没有启动任何节点或发送运动命令。恢复有线链路后再按第1节重新检查，旧快照不代表当前现场状态。
