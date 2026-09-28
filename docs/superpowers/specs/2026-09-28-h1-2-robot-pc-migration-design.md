# H1-2 机器人 PC4 部署与远程 Web/RViz 设计

## 1. 背景与目标

当前工程运行在外部 AI PC 上，AI PC 通过网络访问机器人侧设备并参与 ROS2、导航和运控链路。该方式对 WiFi DDS、网卡选择和底层网络连通性比较敏感，已经出现过 CycloneDDS 网卡不匹配、运控请求超时（3104）以及在线导航启动参数错误等问题。

本设计将运行时主体迁移到 H1-2 的 PC4，同时保留外部 AI PC 的浏览器和 RViz 使用体验。

目标如下：

1. Web 控制台在 PC4 上运行，并在机器人开机后自动启动。
2. SLAM、Nav2、地图、任务和 Unitree 运控桥在 PC4 本机按需运行。
3. 外部 AI PC 通过浏览器访问 PC4 的 Web 控制台。
4. 外部 AI PC 可以运行 RViz，通过 WiFi DDS 查看状态，并发布初始位姿和导航目标。
5. Unitree 底层运控固定走 PC4 的有线 `eth0`，不依赖 WiFi DDS。
6. 第一阶段不迁移相机预览、语音和视觉手势，待主链路稳定后再处理。
7. Jarvis WiFi 或浏览器链路断开时，正在执行的导航立即停止并发送停止运动命令。

## 2. 非目标

本阶段不包括：

- 把底层运控 UDP 端口暴露给 WiFi 或外部 PC。
- 让外部 PC 直接通过 ROS2 控制 Unitree SDK 或底板。
- 立即迁移相机预览、语音串口、YOLO/视觉手势识别。
- 把 RViz 的渲染过程迁移到 PC4。
- 引入容器化部署或新的远程控制协议。

## 3. 目标架构

```text
外部 AI PC（当前示例：192.168.8.5）
├── 浏览器
│   └── HTTP/WiFi ───────────────┐
└── RViz                         │
    └── ROS2 DDS/WiFi ───────────┤
                                 ▼
机器人 PC4（当前示例：192.168.8.6）
├── Web 控制台 :8765
├── ROS2/Nav2/FAST-LIO2
├── 地图、任务和记忆点文件
├── h1_2_loco_bridge
│   ├── ROS 运控适配节点
│   └── 本机 127.0.0.1 UDP daemon
└── Unitree SDK ── eth0 ──> 导航板/底板
```

正常控制链路为：

```text
浏览器或 RViz
    ↓ WiFi（HTTP 或 ROS2 DDS）
PC4 本机 ROS2/Nav2
    ↓ 本机 ROS topic/service
h1_2_loco_bridge
    ↓ 127.0.0.1 UDP
Unitree SDK
    ↓ PC4 eth0
机器人导航板/底板
```

PC4 的 WiFi 和 `eth0` 分工必须保持独立：

- WiFi：Web 访问、RViz 的 DDS 可视化和初始位姿/导航目标消息。
- `eth0`：Unitree SDK 到机器人内部导航板/底板的运控通信。
- DDS 配置不得强行绑定不存在的固定网卡；PC4 需要允许 WiFi 接口参与外部 DDS，同时保留本机通信。
- Unitree SDK 的 `network_interface` 应配置为 PC4 实际可达底层设备的接口，当前设计预期为 `eth0`，最终以 PC4 到 `192.168.123.161` 的实测结果为准。

## 4. 组件部署位置

### 4.1 机器人 PC4

PC4 承担全部运行时职责：

- ROS 2 Humble 环境。
- `unitree_ros2` 工作空间。
- Unitree SDK2 及其动态库。
- 本工程 `h1_nav2_ros2_ws`。
- FAST-LIO2、地图服务、Nav2、定位和路线执行节点。
- `h1_2_loco_bridge` 及本机 UDP daemon。
- Web GUI 和导航进程管理器。
- `maps/`、任务 JSON、参数 YAML、CycloneDDS 配置。

工程路径建议为 `/home/unitree/h1_nav2_ros2_ws`，但代码不得依赖固定用户名路径。Unitree SDK 路径应通过 CMake 配置、环境变量或自动探测得到，不能继续硬编码 `/home/llx/unitree_sdk2`。

### 4.2 外部 AI PC

AI PC 只保留：

- 浏览器访问 Web 控制台。
- RViz2 和必要的 ROS2 CLI 调试工具。
- 可选的录包或离线分析工具。

AI PC 不启动导航、运控 daemon 或 Web 后端，不直接调用 Unitree SDK。

## 5. Web 服务设计

Web GUI 在 PC4 上由 systemd 自动启动，但只启动 Web 服务，不自动启动导航任务。

建议运行参数：

```text
--host 0.0.0.0
--port 8765
```

访问地址：

```text
http://192.168.8.6:8765
```

Jarvis 网络按当前约定视为可信局域网，本阶段不增加登录认证。该选择意味着同一 WiFi 内的设备可能访问运动控制页面，因此 PC4 不应连接不可信公共网络。

网页上的“启动导航”按钮在 PC4 本机执行 `run_ai_slam_nav.sh`；“停止导航”按钮必须同时：

1. 取消 Nav2 当前目标或终止导航进程。
2. 向运控桥发送零速度。
3. 调用底层 `StopMove` 或等价停止操作。
4. 在 Web 日志中记录停止原因。

Web 服务应保持单实例，避免重复启动多个 ROS2/Unitree 运控进程。

## 6. 导航与运控设计

### 6.1 按需启动

开机后仅启动 Web。用户从浏览器选择在线 SLAM 或离线导航模式后，Web 后端在 PC4 本机启动对应 launch。

在线模式的第一阶段验收范围是：

- 相机/雷达驱动按当前工程配置可用。
- FAST-LIO2 发布里程计、点云和 `camera_init` 相关 TF。
- 栅格地图和 Nav2 生命周期节点正常。
- RViz 可以设置初始位姿和发送目标点。
- Nav2 生成速度指令并由本机运控桥转发。

### 6.2 Unitree SDK 网络

`h1_2_loco_udp_daemon` 继续只监听 `127.0.0.1:39001`。ROS 运控适配节点与 daemon 都位于 PC4，因此无需通过 WiFi 传输运动 UDP 命令。

Unitree SDK 初始化时显式使用 PC4 的内部运控接口。该接口配置与 CycloneDDS 配置分离，不能因为 DDS 改用 WiFi 就把 Unitree SDK 也切换到 WiFi。

### 6.3 断网停止策略

本阶段采用调试优先的安全策略：如果 Web 与客户端的心跳/连接断开，导航任务停止。

实现要求：

- Web 前端定期发送心跳，PC4 后端维护最后一次有效连接时间。
- 超过设定超时后，后端取消当前导航并触发停止运动。
- Web 进程异常退出时，导航子进程和运控进程必须有清理路径。
- daemon 收到 SIGTERM/SIGINT 或无有效控制命令超时后发送 `StopMove`。
- 该机制不能依赖浏览器单独执行 JavaScript，PC4 后端必须独立判断超时。

具体超时时间在实施阶段通过低速静态测试确定，初始建议为 2 秒心跳失联阈值，并设置足够的日志信息用于复现。

## 7. RViz 远程可视化设计

RViz 运行在 AI PC，通过 WiFi DDS 加入与 PC4 相同的 ROS_DOMAIN_ID。

RViz 需要能够：

- 订阅 `/map`、点云、TF、里程计、路径、局部/全局代价地图等必要话题。
- 发布 `/initialpose`。
- 通过 Nav2 action 发送导航目标点。

为避免 WiFi 被高带宽点云占满，初期只开放必要显示项；必要时降低点云频率、体素大小或 QoS 数据量。RViz 断开不能影响 PC4 内部 ROS2、Nav2 或运控进程。

DDS 配置要求：

- PC4 使用可用的 WiFi 接口参与 DDS 发现和数据传输。
- AI PC 使用其 Jarvis WiFi 接口参与 DDS。
- 两端 ROS_DOMAIN_ID、RMW 实现和 DDS 发现策略一致。
- 不再把 `enp43s0`、`enp45s0` 等旧有线网卡写死为唯一接口。
- 若自动接口选择不稳定，再使用单独的 PC4/AI PC 配置文件明确指定 Jarvis WiFi 接口。

## 8. systemd 服务与启动顺序

建议拆成两个服务：

### 8.1 `h1-2-web.service`

- `After=network-online.target`。
- 启动前 source ROS2、Unitree ROS2 和本工程 install 环境。
- 设置 `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`、`CYCLONEDDS_URI`。
- 运行 Web GUI，监听 `0.0.0.0:8765`。
- `Restart=on-failure`。
- 默认不启动 `run_ai_slam_nav.sh`。

### 8.2 导航进程

导航暂不作为开机 systemd 服务，而是由 Web 后端按需启动。这样可以避免传感器、地图和底层网络尚未准备好时自动拉起一整套导航。

若后续需要无人值守运行，可再增加独立的导航服务，但必须先保留明确的停止、看门狗和人工接管机制。

## 9. 迁移步骤

### 阶段 0：PC4 环境盘点

1. 确认 PC4 的 Ubuntu、ROS 2 Humble、Python 和编译器版本。
2. 确认 `/home/unitree/unitree_ros2` 和 Unitree SDK 是否存在。
3. 确认 PC4 WiFi 地址、`eth0` 地址以及到内部运控地址的连通性。
4. 确认磁盘、内存、CPU 和传感器设备权限。
5. 确认 PC4 可以使用 `systemd` 用户服务或系统服务。

### 阶段 1：迁移主链路

1. 复制源码、`maps/`、任务 JSON、配置 YAML 和 DDS 配置，不复制旧机器的 `build/`、`install/`、`log/`。
2. 修复 Unitree SDK 路径为可配置值。
3. 在 PC4 重新执行 `rosdep` 和 `colcon build`。
4. 先单独验证 Unitree SDK 与底板/导航板通信。
5. 验证本机启动运控 daemon、ROS 速度桥和停止命令。
6. 验证在线 SLAM/Nav2，不接入 Web 远程访问。
7. 安装并启动 Web systemd 服务。
8. 从 AI PC 浏览器访问 Web 并启动/停止导航。

### 阶段 2：远程 RViz

1. 配置 AI PC 与 PC4 的 DDS 参数。
2. 先只验证 `/tf`、`/odom`、`/map` 等低带宽话题。
3. 验证 RViz 设置初始位姿。
4. 验证 RViz 发送单个低速导航目标。
5. 验证 WiFi 断开后导航在 PC4 停止。
6. 调整点云显示和 QoS，避免影响导航。

### 阶段 3：相机、语音和视觉

在主链路和远程 RViz 连续稳定后，再单独迁移：

- 相机预览和图像订阅。
- 语音串口服务。
- YOLO/视觉手势识别。
- 挥手、握手等动作服务。

每项功能单独验收，避免与导航、WiFi DDS 和底层运控同时变更。

## 10. 验收标准

### 构建与启动

- PC4 可以从全新 shell source 环境并发现所有工程包。
- `colcon build` 成功且不依赖 `/home/llx` 路径。
- Web 服务重启后仍能监听 `0.0.0.0:8765`。
- PC4 重启后 Web 自动恢复，导航不会未经网页操作自动启动。

### 本机运控

- PC4 能通过 `eth0` 与内部运控设备通信。
- 低速前进、转向和停止命令成功。
- 运动请求不出现持续的 3104 超时。
- daemon、ROS 桥和 Web 停止时机器人最终处于停止状态。

### 在线导航

- 在线 SLAM 能正常产生传感器数据、里程计、TF 和地图。
- Nav2 能够接收目标并规划路径。
- Web 能启动、查看日志和停止导航。
- AI PC RViz 能设置初始位姿和发送导航目标。

### 断网与恢复

- 断开 AI PC WiFi 或关闭浏览器后，PC4 在超时内停止导航。
- RViz 退出不会让 PC4 本机进程崩溃。
- WiFi 恢复后 Web 可重新访问，且不会自动恢复之前未完成的运动目标。
- PC4 重启后服务状态和日志可通过 `systemctl`、`journalctl` 查询。

## 11. 风险与取舍

### WiFi 地址变化

PC4 的 WiFi 地址变化会影响浏览器和 DDS 配置。优先在 Jarvis 网络中为 PC4 保留固定 DHCP 租约或静态地址 `192.168.8.6`。

### WiFi DDS 带宽不足

远程 RViz 只作为调试和操作界面，不承担本机导航计算。点云显示需要限流；WiFi 质量差时可以关闭高带宽显示，但不能影响 PC4 的本机导航。

### SDK/系统版本不一致

PC4 上的 Unitree SDK、ROS2 和系统 ABI 必须与现有机器人运行环境匹配。不能直接复制 AI PC 的 `build/` 或 `install/` 产物作为最终部署结果。

### 无认证 Web

本阶段按用户选择不增加登录认证，默认 Jarvis 是可信网络。后续若机器人接入更大范围网络，应增加认证、来源限制和更严格的运动控制授权。

## 12. 设计结论

采用“运行时全部在 PC4、AI PC 浏览器和 RViz 远程接入”的方案。运动控制链路留在机器人内部，WiFi 仅承载 Web 和 RViz DDS；Web 开机自动启动，导航按网页操作启动；相机、语音和视觉手势延后迁移；WiFi/浏览器失联时立即停止导航。

