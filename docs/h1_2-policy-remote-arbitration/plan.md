# H1-2 Policy 速度与遥控器共存

## 目标

让 ONNX Policy bridge 继续作为唯一 `rt/lowcmd` writer，同时读取 H1-2 `LowState.wireless_remote`，把遥控器摇杆速度与导航速度在同一进程内仲裁。

## 范围

- 增加纯逻辑速度源仲裁：遥控器摇杆接管时优先，遥控器回中时使用新鲜导航速度，输入超时或非法输入归零。
- 使用 SDK 已有的 40 字节遥控器布局；保留当前导航限速和训练保护。
- 增加仿真/集成测试和状态日志，确认不会创建第二个 LowCmd writer。

## 非目标

- 不启动旧 Unitree Loco bridge 或第二个 `rt/lowcmd` 发布者。
- 不猜测 H1-2 固件的 FSM 按键交接语义，不自动调用未经确认的 `ReleaseMode`。
- 不在本轮直接进行真机运动或宣称遥控交接已验收。

## 步骤

1. 固定遥控器原始数据解码和速度源优先级，先写纯逻辑测试。
2. 在 Policy bridge 的 LowState 回调中保存遥控器状态，在唯一 `tick()` 中选择 ONNX 速度输入。
3. 暴露只读仲裁状态，更新 launch/README 和网页显示。
4. 运行 C++ 集成/纯逻辑测试、编译和静态检查；部署代码后只读确认 Web/launch，不自动启动控制或运动。

## 验收标准

- 新鲜遥控器摇杆覆盖导航速度；遥控器回中后导航仍受原有 readiness 和 0.4 秒 watchdog 门控。
- 遥控器没有有效输入时，导航速度仍按现有 0.4 秒 watchdog 工作。
- NaN、非法或过期输入输出零，不改变 LowCmd writer 数量。
- 所有测试在 FakeIo/仿真完成；真机动作和固件模式交接另行现场验证。

## 风险

遥控器 40 字节布局已由 SDK 确认；摇杆 yaw 符号按 SDK state-machine 示例使用 `-rx`，但仍需现场只读确认 H1-2 实机方向。H1-2 具体按钮组合未确认，因此不绑定 FSM 切换键、不自动调用 `ReleaseMode`，也不把该功能描述成遥控器模式交接。

## 2026-11-17 实现状态

- 已完成：`enable_remote_velocity` 启动参数；Policy bridge 在唯一 LowCmd writer 内解码 `LowState.wireless_remote`，非中立摇杆优先于导航速度，回中后恢复原导航门控；网页默认勾选“ONNX 与遥控器速度共存”，并显示当前速度源。
- 已完成：没有启动旧 Unitree Loco bridge 或第二个 `rt/lowcmd` writer；纯 C++ 仲裁测试覆盖解码、摇杆优先、回中回退、禁用、超时/非法输入和限速。
- 已验证：两包构建成功；remote arbitration、hardware、launch parameter、train guard、core、ONNX regression 测试通过。node integration 测试在本机 DDS 锁竞争/长时间无输出，未计为通过。
- 未验证：真机摇杆方向与速度响应、按钮/FSM 语义、遥控器模式交接和摔倒后的物理安全。当前不能用这项实现替代现场支撑、急停和撤权流程。

## 2026-11-18 PC4 代码部署

- 已部署至 `/home/unitree/h1_nav2_ros2_ws` 主工作区；板端原文件备份为 `/home/unitree/h1_nav2_ros2_ws/backup_remote_velocity_coexist_20261118/source-before.tar.gz`，SHA256 `b613ffe51053ef78e7814e753d12e046a78c8066dad463c815192cfe5db15a8c`。
- `h1_2_policy_bridge`、`h1_2_nav_tools` 在 PC4 构建成功。Policy bridge CTest（排除 node_integration）7/7 通过，包含遥控仲裁、launch 参数、训练保护、ONNX core 与回归。
- 重启了 `h1-2-web.service`；重启前确认其 cgroup 只有 Web 进程，无 Policy/Nav2/route runner。重启后 service active、HTTP 8765 返回 200，页面包含共存选项，配置 API 返回 `enable_remote_velocity=true`。安装的 Policy bridge 二进制 SHA256 `c5355598cfc88a7c9a5ae5956995d934285953ec35da1778439c6f9efc729cfd`。
- 仅部署并验证软件；未启动 Policy bridge/Nav2、未授权 LowCmd、未发送运动命令。真机摇杆方向、速度响应、FSM 按钮交接和摔倒事故安全审查仍未验证；当前部署不构成实机控制验收。
