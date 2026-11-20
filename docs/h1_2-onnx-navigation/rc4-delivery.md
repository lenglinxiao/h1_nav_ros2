# RC4 交付核验

时间：2026-11-10 17:35（Asia/Shanghai）  
目标：PC4 `unitree-h1-2-pc4`，ROS 2 Humble，ROS domain 0。  
Release：`/home/unitree/h1_nav2_ros2_ws/releases/onnx-navigation-20261110-rc4`

## 构建与配置检查

- 源码归档 `source.tar.gz` SHA256：`0bd15eb646585b37c42b9a7796f86ea09b18f01f6343c59d07e6da86729598b1`。归档包含五个功能包、硬件工具（不含历史 evidence）、运行脚本、地图、配置和功能文档。
- 在独立 RC4 前缀构建 `h1_2_policy_interfaces`、`h1_2_nav_msgs`、`h1_2_policy_bridge`、`h1_2_nav_tools`、`h1_2_slam_bringup`：5/5 完成。Policy Bridge 在加载 ROS Humble 与 PC4 主工作区依赖后 CTest 6/6 通过；首次未加载 ROS 环境的 CTest 中 launch 参数检查因找不到 Python `launch` 失败，加载环境后重跑通过。
- README 所列 `python3 tools/h1_2_policy_hardware/test_acceptance_offline.py` 在 RC4 源码上 17/17 通过。
- 只读 monitor 已编译但未运行，SHA256：`deb334fe27531c75eecf3dbb91363967e34610af6ea5c96b501558c126c7ecc5`。
- Policy executable SHA256：`3ece6216cb15404df54fbac53fc0447f035e3aad08a374db9509aff5e603e0bc`；Owner Python SHA256：`2a80e8896b3aa782d7977fbb96a26ab9fbc15c6b8850dd63ebf830a6fc4a8862`。模型 SHA256：`3090836d0ce454a35161982b60dde20e0fd87627150e06126aabd1e6bbfabae7`；Owner guard 所用 URDF `/home/unitree/h1_nav2_ros2_ws/src/h1_2_description/h1_2.urdf` SHA256：`504b0cc9ea5311044876f4be4e3148fbc285aa1439e33b19d93c357a7468066c`。完整 release 校验项列于目录 `SHA256SUMS`。
- `ros2 launch h1_2_nav_tools policy_nav_session.launch.py --show-args` 通过。
- 使用 RC4 executable、PC4 模型/URDF 和 RC4 monitor 执行 `policy_owner.py --check` 通过；实际配置保持 `enable_hardware_output=false`、`continuous_policy_navigation=false`、`enable_navigation_commands=false`，未配置 envelope。该命令只做配置和文件检查，不启动 Owner 子进程。
- 另用 `/tmp` 临时 mock envelope 对导航启用配置执行 `policy_owner.py --check`，检查通过后临时文件已删除。此项仅验证参数配置路径；不是现场 envelope，不保存在 release，不代表现场准入。

## 当前现场状态与边界

- 核验时间 `2026-11-10T17:35:57+08:00`。只读 ROS 图仍为 `/QtServer`、`/bms_ctr_node`、`/dog_control_pub`、`/h1_2_nav_web_gui`、`/rsLidarCheak`、`/switch_node`；`ros2 action list` 为空。Owner systemd 状态为 `inactive`。
- 8765 Web GUI 仍由主工作区 `install/h1_2_nav_tools` 启动；RC4 GUI 未安装或替换该服务。因而此刻用户不能通过当前网页选择新 RC4 ONNX 后端并启动 Nav2。
- RC4 只在独立 release 目录构建。没有覆盖主工作区 `src`、`install`、地图或现有服务；没有启动 Owner、Policy Bridge、monitor 或 Nav2，没有授权控制输出或发送运动命令。
- 构建核验之后已另做 RC4 monitor 6秒只读采样并复核地图；具体数据以[硬件核对清单最新样本](hardware-validation-checklist.md)为准。唯一 writer 仍未映射到 host/PID，地图 PGM/PCD 与开发机版本差异仍未解决；现场速度 envelope 尚无批准证据。故此交付达到“可以开始部署前只读核对与人工准入步骤”的程度，**尚未达到可以直接启动 ONNX 运控/Nav2 行走测试**。

## 本机复核命令

在 PC4：

```bash
cd /home/unitree/h1_nav2_ros2_ws/releases/onnx-navigation-20261110-rc4
sha256sum -c SHA256SUMS
source /opt/ros/humble/setup.bash
source /home/unitree/h1_nav2_ros2_ws/install/setup.bash
source install/setup.bash
ros2 launch h1_2_nav_tools policy_nav_session.launch.py --show-args
```

Owner `--check` 需在逐项审查并导出实际配置后单独执行；不要把临时 mock envelope 复制成现场配置。现场状态变化后，先按[板端核对清单](hardware-validation-checklist.md)重新检查 writer、地图、急停和授权范围。
