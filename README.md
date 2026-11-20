# H1-2 Navigation Workspace

<p align="center"><img src="docs/assets/h1-2-navigation-logo.svg" alt="H1-2 Navigation" width="440"></p>

这是一个面向 H1-2 的 ROS 2 Humble 工作空间，包含 MID360/FAST-LIO2 建图、Nav2 在线与固定地图导航、路径与记忆点工具，以及可选的 Unitree 或 ONNX Policy 速度控制后端。

## 技术栈

![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-14%2F17%2F20-00599C?logo=cplusplus&logoColor=white)
![Nav2](https://img.shields.io/badge/Nav2-robot_navigation-4A90D9)
![FAST-LIO2](https://img.shields.io/badge/FAST--LIO2-LiDAR--Inertial-36A3D9)
![PCL](https://img.shields.io/badge/PCL-point_clouds-5B8C5A)
![ONNX Runtime](https://img.shields.io/badge/ONNX_Runtime-inference-005CED?logo=onnx&logoColor=white)
![Cyclone DDS](https://img.shields.io/badge/Cyclone_DDS-middleware-6C7A89)
![Fast DDS](https://img.shields.io/badge/Fast_DDS-middleware-6C7A89)
![colcon](https://img.shields.io/badge/colcon-build-4B5563)
![pytest](https://img.shields.io/badge/pytest-tests-0A9EDC?logo=pytest&logoColor=white)

徽章中的 C++ 版本对应不同子包的标准设置；Unitree SDK 2 和 ONNX Runtime 是相关运动后端的依赖。

## 快速开始

需要 Ubuntu、ROS 2 Humble 和工作空间依赖。首次安装依赖并构建：

```bash
cd ~/h1_nav2_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

默认入口启动在线建图和 Nav2：

```bash
./run_ai_slam_nav.sh
```

该模式沿用 FAST-LIO2 的 `camera_init` 坐标系。RViz 固定坐标系应设为 `camera_init`，可使用 Nav2 Goal 发送目标。停止当前会话：

```bash
./stop_ai_slam_nav.sh
```

启动器支持 `mode=online`（默认）和兼容别名 `mode=mapping`。完整参数可查看：

```bash
./run_ai_slam_nav.sh --help
```

## 固定地图导航

固定地图流程需要来自同一次在线建图的 2D 地图（YAML/PGM）和 3D 点云地图（PCD）。在线建图时分别保存：

```bash
./run_ai_slam_nav.sh mode=save_map map_prefix=maps/h1_2_lab
./run_ai_slam_nav.sh mode=save_pcd_map pcd_prefix=maps/h1_2_lab
```

使用保存的地图启动导航：

```bash
./run_ai_slam_nav.sh mode=navigation \
  map="$PWD/maps/h1_2_lab.yaml" \
  localization=global_gicp \
  pcd="$PWD/maps/h1_2_lab.pcd"
```

`global_gicp` 使用点云地图进行全局初始化和后续定位；`icp` 是需要初始位姿的局部定位选项。首次使用固定地图时，先在 RViz 设置初始位姿，再发送导航目标。保存地图命令只请求保存当前数据，不会停止在线建图。

## 运动控制后端

启动器默认使用 `unitree` 后端。在线或固定地图模式可在启动时选择 `policy`。Policy 启动要求操作人员已按现场流程核实调试模式和独占控制，并显式确认；下面的确认参数只是人工声明：

```bash
./run_ai_slam_nav.sh mode=navigation motion_backend=policy \
  confirm_debug_mode=true confirm_exclusive_control=true \
  map="$PWD/maps/h1_2_lab.yaml" \
  localization=global_gicp \
  pcd="$PWD/maps/h1_2_lab.pcd"
```

在线模式也可通过 `motion_backend=policy` 选择 Policy 后端。Policy bridge 单独启动入口为 `mode=policy_bridge_only`，不会启动 Nav2 或路线。Policy 硬件输出默认受授权和状态门控保护；启动参数、现场准备与验证要求见[硬件适配说明](docs/h1_2-policy-hardware-adaptation.md)及[Policy bridge 文档](src/h1_2_policy_bridge/README.md)。这些软件入口不代表真机动作已通过验证，也不证明控制权独占或机器人处于安全状态。开始任何硬件控制前，须按现场流程核实当前状态、支撑和控制权。

Nav2 的速度链路为 `/cmd_vel` → 平滑器 `/cmd_vel_final` → 选定的运动后端。Policy 也可通过启动参数启用遥控器速度仲裁；它仍由同一个 Policy bridge 输出控制命令。

## 工作空间内容

| 包 | 用途 |
| --- | --- |
| `h1_2_slam_bringup` | Livox、FAST-LIO2、在线/固定地图导航及 RViz 启动入口 |
| `FAST_LIO_ROS2` | FAST-LIO2 ROS 2 实现，安装后的 ROS 包名为 `fast_lio2` |
| `livox_ros_driver2` | Livox ROS 2 驱动与消息定义 |
| `h1_2_nav_tools`、`h1_2_nav_msgs` | 导航辅助节点、记忆点和路线接口、Web 工具 |
| `h1_2_icp_localization` | 固定地图 3D ICP 定位 |
| `h1_2_loco_bridge`、`h1_2_policy_bridge` | Unitree 与 ONNX Policy 速度控制后端 |
| `h1_2_description` | H1-2 URDF 与模型资源 |
| `h1_2_vision`、`h1_2_voice_control` | 视觉检测与语音命令组件 |
| `h1_2_bag_benchmark` | rosbag 建图与定位回归工具 |

直接连接 MID360 并在本机启动 Livox 驱动时，还需安装 Livox-SDK2。只通过 DDS 接收已有 Livox 话题时，可不安装 SDK2；具体构建条件见 [`livox_ros_driver2`](src/livox_ros_driver2/README.md)。

## 文档索引

- [ONNX 导航计划与验证记录](docs/h1_2-onnx-navigation/plan.md)
- [硬件验证清单](docs/h1_2-onnx-navigation/hardware-validation-checklist.md)
- [Policy 硬件适配说明](docs/h1_2-policy-hardware-adaptation.md)
- [导航故障记录](docs/h1_2-policy-troubleshooting.md)
- [人工策略试验工具](tools/h1_2_policy_hardware/README.md)
- [测试目录说明](test/README.md)

## 开发检查

运行离线静态测试：

```bash
python3 -m pytest -q test/static
```

ROS 包测试需先构建，再按目标包执行，例如：

```bash
colcon test --packages-select h1_2_policy_bridge
colcon test-result --verbose
```

本仓库包含第三方源码及硬件相关工具。测试、离线仿真和输出禁用检查不等同于真机控制验证；硬件操作应遵循对应功能文档及现场准入流程。
