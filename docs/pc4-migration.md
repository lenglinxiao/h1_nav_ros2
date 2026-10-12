# PC4 部署操作

PC4 的 Web 和 ROS 运行时使用 `unitree@192.168.8.6`。PC4 的 WiFi 接口是 `wlxf8ce21a12bfe`，用于 Web/DDS；有线 `eth0` 地址为 `192.168.123.164`，Unitree SDK 运控固定使用它。当前 PC4 可达 `192.168.123.161`。现场启动 Livox 驱动后，`/livox/lidar` 与 `/livox/imu` 已有持续数据；这说明当前话题链路可用，不再以先前的 `.120` ARP 检查结果作为雷达验收结论。

首次部署前，确认依赖安装和 SDK/Unitree ROS 工作区已经准备好。以 PC4 用户运行：

```bash
cd /home/unitree/h1_nav2_ros2_ws
mkdir -p ~/.config/h1-2
cp config/pc4.env.example ~/.config/h1-2/pc4.env
```

编辑 `~/.config/h1-2/pc4.env`，确认 `UNITREE_ROS_SETUP`、`UNITREE_SDK2_ROOT`、`H1_2_WORKSPACE`、`H1_2_NAV_WS` 和 DDS 配置路径。部署源码时运行 `scripts/pc4_deploy.sh`（默认目标 `unitree@192.168.8.6`）；它传输源码、配置及远端尚不存在的地图文件，排除可重建的 `build/`、`install/`、`log/`，不会覆盖远端既有地图或任务。之后在 PC4 执行：

```bash
cd /home/unitree/h1_nav2_ros2_ws
scripts/pc4_build.sh
```

`pc4_build.sh` 会先执行 rosdep 再 colcon（最多两个 worker）；仅在已完成依赖安装后才能显式设置 `H1_2_SKIP_ROSDEP=1`。PC4 当前 rosdep 尚未初始化。尝试初始化/更新时，rosdep 所需的 GitHub 索引受网络连接限制，完整 `rosdep check` 尚未完成；本次迁移已安装实际缺少的 APT 依赖，并在 PC4 原生构建及运行中验证。网络恢复后仍应按以下步骤补做 rosdep 检查。先 source Humble 与 Unitree ROS2，保证 `unitree_go` 从现有工作区解析；`nav2_recoveries` 已按 Humble 元数据更正为 `nav2_behaviors`：

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/unitree_ros2/install/setup.bash
sudo rosdep init  # 仅在尚未初始化时执行
rosdep update
rosdep check --from-paths src --ignore-src --rosdistro humble
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

若 rosdep 输出的 APT 包尚未安装，按它的输出以管理员权限执行对应 `sudo apt install ...`，然后重新运行 `rosdep check`；不得静默忽略失败。首次安装 Web 用户服务并固定监听端口 8765：

```bash
scripts/pc4_install_web_service.sh
systemctl --user status h1-2-web.service
```

PC4 已确认可由当前用户执行 `loginctl enable-linger unitree` 并已是 `Linger=yes`。若以后状态变为 `no`，先直接执行该命令；仅在权限被拒绝时请管理员代为执行。手动启停和日志查看：

```bash
systemctl --user restart h1-2-web.service
systemctl --user stop h1-2-web.service
journalctl --user -u h1-2-web.service -f
```

远程操作前，让 AI PC 与 PC4 接入同一 WiFi。浏览器打开 `http://192.168.8.6:8765`。Web 页面负责启动/停止 PC4 上的 SLAM 与导航；在线导航期间保持页面打开，以维持控制心跳。

当前 Linux AI PC 已注册本地 RViz 启动器。刷新 Web 后点击“打开本机 RViz”即可自动配置 ROS 与 DDS 并打开窗口；首次点击需要允许浏览器打开 `H1 Local RViz`。模式可选在线、离线或跟随运行模式；重复点击会尝试将已有 RViz 窗口切到前台，不会切换已有窗口的模式，切换模式前先关闭已有窗口。当前窗口激活已在 X11 桌面验证。其他 Linux 上位机需先在本地工程运行 `bash scripts/ai_pc_install_rviz_handler.sh`，并调整 `config/cyclonedds_ai_pc.xml` 的网卡名称。Windows 尚未配置该启动器。

也可以在 AI PC 终端 source 本地 ROS 2 与工程环境，设置相同 domain 并指定 AI PC 的 WiFi DDS 文件。以下命令只启动本地 RViz：

```bash
source /opt/ros/humble/setup.bash
source ~/h1_nav2_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file:///home/llx/h1_nav2_ros2_ws/config/cyclonedds_ai_pc.xml
ros2 launch h1_2_slam_bringup h1_2_nav2_rviz.launch.py mode:=online
```

当前 Web 运控入口修复已部署，用户可按以下步骤验证挥手动作：

1. 确认机器人周围安全并由现场人员看护；打开 `http://192.168.8.6:8765` 并刷新。选择运行模式，关闭自动执行路线，然后点击“启动当前模式”，等待运控服务就绪；该操作会初始化运控。
2. 使用 Web 的“测试挥手”手动入口触发一次测试，观察机器人响应和页面返回的真实 `sdk_ret`。测试动作会调用运控接口，需在现场具备安全条件时执行。
3. 如需查看在线状态，可按上面的 AI PC 命令启动 RViz；该命令只启动 AI PC 上的 RViz。在线配置会自动选择 `camera_init`，无需设置离线初始位姿。
4. 保持浏览器页面打开以维持 heartbeat。导航、任务执行、行走停车和断网安全行为不在本轮用户指定测试范围内；不要仅凭挥手接口返回推断这些功能已验收。

当前验收记录（2026-10-09）：Web 用户服务已启用，`Linger=yes`，服务启动不会自动启动运控。手动 gesture 与任务 `gesture_route` 入口修复已部署，相机、语音和视觉入口仍关闭，heartbeat 机制保留。在线模式自动选择 `camera_init` 参数；三份 Nav2 参数 YAML 的 inflation radius 已调整为 `0.40`。局域网 HTTP 页面创建任务时的 UUID 兼容问题、浮窗错误提示与点位请求乱序覆盖问题已修复并部署。新任务创建后需要点击“保存更改”。

当前心跳由所有浏览器共享，尚未实现独占控制权；实际操作者掉线可能被其他在线浏览器的心跳掩盖，多用户控制权限需要后续完善。

2026-10-10 心跳优化：Web HTTP 端口保持 `8765`，新增 WebSocket 心跳端口 `8766`（HTTP 端口加一），两者都需要允许上位机访问。服务端每 0.5 秒发送协议 Ping，仅收到浏览器自动返回的 Pong 才更新心跳；页面启动等待首个 Pong，旧 HTTP `/api/heartbeat` 不再刷新安全看门狗。浏览器后台 JavaScript 定时器降频不会再直接造成心跳丢失。需要 Python `websockets` 依赖（package.xml 已声明 `python3-websockets`，PC4 已安装）。

真实断网、浏览器关闭或操作系统挂起导致超过 2 秒没有 Pong 时，仍会停止导航。连接恢复后只恢复心跳，不自动恢复任务，需要操作者重新启动。更新后强制刷新浏览器以加载新页面。ROS 工具节点也改为安全处理外部关闭，避免正常停止后出现重复 shutdown 的异常堆栈。

本次修复已在 PC4 原生构建并重启 Web 服务，远程检查确认 HTTP 配置报告心跳端口 8766，首次 Pong 后返回 ready，客户端不发送业务消息超过 2 秒后连接仍存活，旧 HTTP 心跳被拒绝，导航保持未启动。相关离线及回环测试 85 项通过；未进行机器人运动测试。

同日 17:41:39 仍出现一次看门狗超时（距上次 Pong 2.17 秒），完整 Web 日志确认由心跳保护触发，服务自身未重启。用户报告页面一直在前台；因此后台定时器不能解释此次故障。当前停机状态下 30 次 ping 无丢包、延迟 1.6–90.8 ms，尚未复现运行时中断。PC4 使用 Jarvis 的 2.4 GHz 信道 10；网络拥塞、浏览器连接和运行时调度仍需进一步区分，不能将 WiFi 认定为已证实根因。新增连接地址、断开异常类型、最近 Pong 时间及延迟日志，保留 2 秒停车保护；未宣称现场故障已彻底修复。

后续 17:51:29、18:01:20 再次触发心跳保护，成功 Pong 的 RTT 曾达 1.327 秒。修正心跳发送时序：每 0.5 秒独立排程 Ping，避免旧实现等待 Pong 后再额外等 0.5 秒；仅真实成功 Pong 回调刷新心跳，连接超时按最后成功 Pong 计算，与看门狗使用同一时间基准。未完成 Ping 数量有上限，退出取消相关任务；默认 2 秒停车阈值不变。网页日志改为单个在途请求、序号去重、批量更新及 200000 字符上限。两处修复已部署并重启 PC4 Web，91 项相关测试通过（含延迟 Pong 下发送节拍、无回应超时、网页日志并发与长度上限）。现场长时间导航稳定性仍需用户验收，真实连续 2 秒无回应仍会停车。

18:10:42 的再次停机仍由两个浏览器连接同时无 Pong 触发。用户当时导航待机并使用 RViz。检查发现在线 RViz 订阅完整 `/map`，在线启动配置为 1600×1600、3 Hz，仅栅格数据理论上限约 61.44 Mb/s，PC4 当前 WiFi 协商速率为 65 Mb/s。导航停止后四分钟接口平均 TX 约 0.207 Mb/s、RX 约 0.185 Mb/s，心跳恢复正常；这些证据提示地图传输压力，但尚无运行时 A/B 对照来证明全部断开均由网络拥塞造成。

已为在线 RViz 部署独立 `/map_preview`：800×800、0.10 m 分辨率、最高 0.5 Hz，栅格数据约 2.56 Mb/s，为原配置上限的 1/24。PC4 内部 `/map` 仍为 1600×1600、0.05 m、3 Hz。预览使用相同坐标系和原点，任意 occupied 子格会保留为 occupied，未知区域保守处理；只有预览订阅者存在时才构造、发布。离线导航配置未改。地图显示每约 2 秒更新一次，存点和目标坐标仍用原坐标系。

升级后必须关闭此前打开的在线 RViz 窗口，再由 Web 的“打开本机 RViz”重新打开。新窗口的 Map 话题应为 `/map_preview`；旧窗口继续订阅 `/map` 会继续产生原有 WiFi 压力。不要用降低 RViz 渲染帧率代替切换话题，因为那不会限制 DDS 接收的数据率。2 秒心跳停车保护保留，实际导航稳定性和运动效果仍由用户现场验收。

2026-10-12 收尾核对：PC4 的源代码、安装目录中的启动文件和 RViz 配置与上位机一致，Web 服务 active，导航未启动。39 项相关测试通过；上位机及 PC4 均完成域 187、仅回环接口的合成地图收发检查。PC4 实收原图 14 条（1600×1600、0.05 m、每图 2560000 个栅格），预览 3 条（800×800、0.10 m），预览间隔约 2.318/2.002 秒；原图数据保持一致，障碍保留，两话题 QoS 均为 reliable、transient_local、depth 1。检查仅使用合成数据，未启动机器人导航或运动，未据此宣称现场长期稳定性已验收。

同日 14:56:10 仍出现新一次心跳停机（Pong age 2.001 秒，看门狗 age 2.11 秒），此前 SLAM 地图持续更新。运行态已确认上位机 RViz 订阅 `/map_preview`，没有订阅 `/map`、`/Laser_map` 或 `/cloud_registered`，因此不能把此事件解释为旧 RViz 配置未加载。停机后 CPU 空闲约 97.8%、心跳 TCP RTT 约 30 ms；WiFi 当时报告速率 135–150 Mb/s（协商速率会变化）。内核 AICWF `status=9` 记录在 14:54:57，未与 14:56:10 同时发生，也未见同段重置或断连记录。无线驱动、浏览器响应和服务运行时调度仍需同步采样区分；低带宽预览已落实，但反复断开的根因尚未确认。

采集复现证据可在 PC4 运行 `python3 scripts/pc4_capture_web_disconnect.py --duration 900`。工具每秒将网卡累计收发计数、CPU 计数、Web 进程 CPU、心跳端口 TCP 状态和增量心跳/启停日志保存为 `/tmp/pc4_web_disconnect_<时间>.jsonl`（权限 0600）。它只读取 proc、ss 和本机 HTTP 日志，不建立 WebSocket、不刷新安全心跳、不启动导航或动作；默认 15 分钟结束。通过停机前的 `bytes_sent`、`bytes_received`、`bytes_retrans` 和 CPU 变化辅助区分服务发送停顿、网络重传或客户端未回应。网络端口和原有日志不足以直接断言是哪一种故障，需检查采样窗口中的实际复现事件。

Livox 驱动运行时连续 8 秒收到 `/livox/lidar` 约 80 条（10.11 Hz，每条约 19,872–20,160 点）及 `/livox/imu` 约 1,296 条（162.82 Hz）。此前无运动检查中 Nav2 controller、planner、BT navigator 生命周期均为 active，地图为 1600×1600、分辨率 0.05 m，`/slam_odom` 和 `camera_init -> body` TF 持续更新。为满足运行依赖已安装 Nav2 等 APT 包，并升级 `diagnostic_updater` 修复缺失共享库。FAST-LIO2 的空日志文件指针关闭问题已修复，修复后的程序已部署到 PC4 并在 2026-10-08 20:01 完成编译；尚未通过退出运行复验。

机器人现场运动测试由用户执行。实际导航行走、任务路线、运动停车、FAST-LIO2 退出稳定性及断网行为尚未有完整验收记录；已有无运动话题检查和 SDK 初始化成功不能替代现场验收。

回滚仅停止和禁用本次新增的 Web 用户服务，不操作现有 `unitree_slam.service`：

```bash
systemctl --user disable --now h1-2-web.service
rm -f ~/.config/systemd/user/h1-2-web.service
systemctl --user daemon-reload
```
