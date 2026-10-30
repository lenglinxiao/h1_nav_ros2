# H1_2 策略接入：第一阶段规格

## 已批准范围

保留现有 Nav2 定位、感知和规划，增加可选策略运动后端。第一阶段仅进行离线 ONNX 推理和隔离仿真闭环；禁止连接实机、自动释放或切换 161 原运控控制权。真机阶段须另行授权。Unitree 后端与策略后端互斥，策略独占 27 关节并保留右臂胸前 ArmHold 姿态；冲突手势暂停。

主导航 launch 的 `motion_backend` 为 `unitree`（默认）或 `policy`；其他值报错。Unitree 保留 `enable_sport_bridge`、`auto_start_loco` 原语义。policy 模式不启动旧 loco daemon/bridge，不接受非空 `gesture_route`，且要求 `policy_network_interface=lo`、`policy_dds_domain=42`。主导航将 `/cmd_vel_final`、`policy_model_path`（默认 `/home/llx/policy.onnx`）、以上接口和 domain 传给 `h1_2_policy_bridge` 的 `policy_bridge.launch.py`，其参数名分别为 `cmd_vel_topic`、`model_path`、`network_interface`、`dds_domain`。策略不得自动激活：准备和策略接管需手动状态门控，启动顺序为 Passive → FixStand → ArmHoldStand → ArmHold；任何状态可返回 Passive。启动不释放 161。

沿用既有 Nav2 配置：前进 0～0.20 m/s、横向 0、最大 yaw 0.50 rad/s；桥接限幅 vx ±0.20 m/s、vy 0、wz ±0.50 rad/s。导航速度输入超过 0.4 秒未更新时归零，策略继续站立；底层故障处理独立于速度超时。此超时不同于旧 daemon 的看门狗和命令持续时间。

## 模型及部署契约

模型 `/home/llx/policy.onnx`：MD5 `b27c034920b671ffb33c920a58798d0a`，SHA-256 `3090836d0ce454a35161982b60dde20e0fd87627150e06126aabd1e6bbfabae7`。输入 `obs` 为 float32 `[1,92]`，输出 `actions` 为 float32 `[1,27]`。参数依据为 `/home/llx/.codex/attachments/c35c98bd-a4c9-40ed-92fc-d83b96c53cd9/已粘贴的文本.txt`，SHA-256 `f2d699d27246ce35eca0303f0468210ce8998d05d4a128ed16a9c787a3e1fd8f`；观测、关节参数、相位、状态机和静态向量分别见附件第 2、4、6、7、9 节。附件称归一化已嵌入模型，外部不再归一化；该声明尚需独立验证。

观测依次为机体系角速度、重力投影、速度指令、相位正余弦、27 个关节相对默认角、27 个关节速度、上一步原始动作，首步动作为零。循环 50 Hz，相位周期 0.6 s；速度指令范数小于 0.1 时相位观测置零而内部相位继续。目标角为默认角加 scale 乘 action；PD 力矩按附件限幅。用户确认附件零点补偿适用于当前原机。

## 已知差异和验收边界

附件腕部顺序为 pitch-roll，而 SDK/MJCF 为 roll-pitch；必须按名称显式映射，训练时真实顺序尚未证实。附件踝 pitch 限幅 ±40 N·m 与参考 MJCF ±60 N·m 不同；附件显式 PD 力矩计算及裁剪与参考位置/PD 命令的执行时序不能假定等价，训练时序待核实。参考 `unitree_rl_mjlab` 不是已确认的训练来源。

附件声称静态向量和 sim2sim 通过，这些属于附件报告而非本次验收结果。静态回归只能验证模型数值，不能证明关节物理语义；仿真稳定也不能单独证明映射正确。离线模型回归、隔离闭环、161 控制权释放与恢复、164 实时性以及实机行为均须按实际运行结果分别记录；在完成验证前不得称其已通过。

## 验收记录

- 主导航后端选择离线测试：4 项通过，覆盖默认 Unitree、策略互斥与隔离校验、非法后端拒绝，以及加载实际策略 launch 并核对四个转发参数。测试只解析 launch，未启动节点或硬件。
- 主导航、策略 launch 和后端测试 Python 语法检查通过；主导航 `ros2 launch ... --show-args` 解析通过。
- 静态 ONNX Runtime 回归由两个执行者独立运行，报告最大绝对差 `0.000457704`；差值本身不代表物理关节映射已验证。
- 最终隔离仿真通过真实 ROS → DDS domain 42/lo → MuJoCo 链路，使用唯一策略进程，每次自由物理运行至少 30 秒；准备阶段理想固定、接管后释放，不代表真实吊带。每个 2 ms 物理步发布状态（500 Hz），策略命令保持 50 Hz；此前 50 Hz 状态导致准备振荡并被 readiness 拒绝，故改用此频率。
- 零指令：骨盆高度 0.942187～0.949963 m，无跌倒或辅助，CRC 错误及非法命令均为 0；2 ms 采样下左脚最长无接触 16 ms、右脚 0，踝最大抬高 0，xy 漂移分别 12.36、9.87 mm。这些结果不等同严格零迈步。前进指令 0.20 m/s 时机体系平均 vx 0.1541、vy -0.0386 m/s；转向指令 0.50 rad/s 时平均 wz 0.4468 rad/s，均存在跟踪偏差。
- SDK 节点使用同 UID 排他锁；仿真脚本直接执行并清理进程组，以防双 publisher。受重复 publisher 污染的旧数据已弃用。持久结果：[零指令](../tools/h1_2_policy_sim/results/dds_zero.json)、[前进](../tools/h1_2_policy_sim/results/dds_forward.json)、[转向](../tools/h1_2_policy_sim/results/dds_turn.json)；操作命令及条件见[仿真说明](../tools/h1_2_policy_sim/README.md)。原生直接运行的自由物理结果与上述 DDS 结果独立。
- DDS 假状态 smoke 在修正 directexec 脚本后复验：启动时输出 0 条命令，stale 场景 149 条、nonfinite 场景 127 条；CRC、有限力矩和零 kp/kd 检查通过，故障后停止发送。结束时 fuser 检查排他锁无持有者。假状态测试不构成 DDS 物理闭环。当前 DDS 依赖仅支持 Fast DDS，并依赖运行时 RPATH 配置。实机、控制权切换与恢复未执行、未验收。
