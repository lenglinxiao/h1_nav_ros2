# H1_2 离线仿真

独立进程，不启动 ROS、DDS、网卡或窗口，不连接机器人。仿真复用工作区公共 Controller 的阶段、策略、观测和力矩实现，并以名称映射关节。附件指定腕关节 pitch-roll 顺序；真实训练顺序尚未验证。模型和 ONNX 从本机读取，第三方文件不修改。每 20 ms 更新策略与显式 PD 力矩，并在物理步之间保持力矩；XML 物理步长必须整除 20 ms。关节和 actuator 力矩上限在内存中设置，包括踝 pitch ±40 N·m。

默认 `--prep ideal_fixed_base` 是理想化准备：初始为附件 FixStand 姿态，0–3 秒固定自由基座在高度 1.03 m，3–6 秒随 ArmHoldStand 插值把基座高度降到 0.95 m，姿态保持竖直。各关节仍由共享控制器驱动，物理计算和足地接触持续运行；6 秒满足控制器条件后完全撤除基座固定，接管后的物理和重力均自由运行。此方式只是构造策略评估初态，不能视作真实吊带、人工准备或接管验证。这里 0.95 m 是 MuJoCo pelvis 自由关节高度，附件的站高测量基准未提供，不能确认两者一致。`--prep upward_force` 则使基座全程自由，在准备时施加等于 60% 自重的恒定竖直力；它也只是吊带近似，没有姿态或水平约束。

共享状态机检查准备时间、双膝和重力方向。人为目视确认和真实吊带状态不能由仿真实现。程序仅在允许接管后施加指定速度；高度低于 0.6 m 或 roll/pitch 超过 1 rad 时停止。此停止条件不是通过标准。`--duration` 是接管后需要运行的模拟时间，不包含准备时间；15 秒内未接管会停止。进程成功退出仅表示仿真执行完毕。JSON 的 `evaluation_status` 明确区分 `completed_without_fall`、`active_incomplete` 和 `preparation_failed`；这仅描述本次运行完成状态，不代表通过所有验收标准。

```bash
cmake -S /home/llx/h1_nav2_ros2_ws/tools/h1_2_policy_sim -B /tmp/h1_2_policy_sim_build
cmake --build /tmp/h1_2_policy_sim_build -j2
python3 /home/llx/h1_nav2_ros2_ws/tools/h1_2_policy_sim/test_contract.py /tmp/h1_2_policy_sim_build/h1_2_policy_sim
timeout 30 /tmp/h1_2_policy_sim_build/h1_2_policy_sim --duration 30 --output /tmp/h1_sim_ideal
timeout 30 /tmp/h1_2_policy_sim_build/h1_2_policy_sim --duration 30 --vx 0.2 --output /tmp/h1_sim_forward
timeout 30 /tmp/h1_2_policy_sim_build/h1_2_policy_sim --duration 30 --wz 0.5 --output /tmp/h1_sim_turn
timeout 30 /tmp/h1_2_policy_sim_build/h1_2_policy_sim --duration 30 --prep upward_force --output /tmp/h1_sim_upward
```

输出目录包含 summary.json 和 trace.csv。轨迹以 50 Hz 记录阶段、时间、pelvis 高度、姿态、世界坐标及骨盆机体系水平速度、IMU 机体系角速度、饱和关节数、跌倒、基座约束、外加竖直力、接触数、膝角和踝位置。`--model` 和 `--xml` 可指定其他路径；需重新核实模型和资源。

## 本次结果

2026-10-30，本机 ORT 1.22.0、MuJoCo 3.3.6，物理 dt 0.002 s；ONNX SHA-256 `3090836d0ce454a35161982b60dde20e0fd87627150e06126aabd1e6bbfabae7`。理想准备模式在 t=6 s、pelvis 高 0.95 m、双膝 0.8924/0.8993 rad、接触数 2 时接管，随后撤除约束。各项运行到 t=36 s，即自由物理运行 30 s：

| 指令 | 跌倒 | pelvis 高度范围 m | 全程饱和关节采样数 | 备注 |
|---|---|---|---|---|
| 零速度 | 否 | 0.94686–0.95029 | 0 | 左右踝相对接管时的水平最大位移约 1.28/0.97 cm |
| vx=0.2 m/s | 否 | 0.94858–0.96593 | 0 | 骨盆机体系平均前向 0.157 m/s、横向 -0.041 m/s；前向达到指令约 78%，存在侧向漂移 |
| wz=0.5 rad/s | 否 | 0.94718–0.96541 | 3 | IMU 机体系 z 角速度均值约 0.443 rad/s，达到指令约 89% |

高度范围为 ArmHold 样本，三个理想准备运行的高度均处于 0.95±0.03 m；表中饱和计数包含准备及接管后全部采样，三个自由阶段分别为 0、0、3。JSON 另有自由阶段时长、最大姿态角、机体系平均速度、踝位移和饱和计数。零速度未计算严格的足部离地与迈步判据，因此踝位置漂移不能证明零迈步。向上恒力模式在 FixStand t=2.00 s 前倾 pitch 1.020 rad、pelvis 高 0.667 m，未接管即触发停止。理想初始化后的表现不能证明手动启动、训练关节语义或实机安全。

## ROS/DDS 真实物理闭环

`h1_2_dds_physics` 只使用 lo 和 Unitree DDS domain 42；它把 MuJoCo 状态发给 ROS 策略节点，并接收 CRC 校验通过的 LowCmd 驱动物理。`run_dds_physics.py` 用 ROS domain 43、FastDDS 和 ROS_LOCALHOST_ONLY=1 启动节点及桥；只有对应 Trigger 成功后才切换理想准备支撑，ArmHold 成功后释放基座，随后持续发布 `/cmd_vel_final`。运行结束清理其启动的进程。DT_RPATH 指向 SDK 匹配的 CycloneDDS 库。

本机普通沙箱禁止接口枚举与 UDP 套接字，运行需允许本机环回通信。参考仿真桥以 1 ms 周期更新，当前模型物理步为 2 ms，桥以 500 Hz 发状态、节点以 50 Hz 控制；日志为 50 Hz。ROS 工作区已构建时可运行：

```bash
source /opt/ros/humble/setup.bash
source /home/llx/h1_nav2_ros2_ws/install/setup.bash
timeout 50 python3 /home/llx/h1_nav2_ros2_ws/tools/h1_2_policy_sim/run_dds_physics.py --duration 30 --output /tmp/h1_dds_zero
```

直行和转向可分别加 `--vx 0.2` 或 `--wz 0.5`，输出独立目录。ROS 日志位于 `/tmp/h1_sim_ros_logs`。离线 evaluator 要求自由物理时长达目标、无跌倒和支撑、命令及状态频率正常、CRC 正确、命令未超时、样本有限；失败退出非零。50 Hz 状态发送曾发生准备阶段膝角振荡而拒绝接管，该失败保留在 `/tmp/h1_dds_zero_clean2/trace.csv`；不能据此断言振荡的唯一原因。

本机最终单节点、500 Hz 状态测试的持久摘要：[`dds_zero.json`](results/dds_zero.json)、[`dds_forward.json`](results/dds_forward.json)、[`dds_turn.json`](results/dds_turn.json)。三次自由物理阶段均超过 30 s，低层命令约 50 Hz、状态约 500 Hz，接管后无固定支撑和外力，无 CRC 错误、坏命令或跌倒；三次 pelvis 高度均在 0.95±0.03 m。前进时机体系平均 vx 0.154 m/s、vy -0.039 m/s；转向时 IMU 机体系平均 wz 0.447 rad/s。两项跟踪值是实测结果，不隐含未定义的误差验收阈值。

零指令补测在每个 2 ms 物理步检查左右 `ankle_roll_link` 与 floor 的 MuJoCo 接触，按 20 ms 记录累计统计：左足最长连续无接触 16 ms、右足 0 ms，踝部相对接管位置最大上升左右均为 0 m，最大水平位移左 12.36 mm、右 9.87 mm。足部接触不始终连续；由于未定义接触间隙、滚动/滑动和足部抬起的迈步判定阈值，**附件“30 s 不迈步”尚不能判定通过**。前进与转向测试时未记录该足部指标。足部统计只覆盖指定脚踝实体的 floor 接触，时间分辨率为 2 ms，不代表实机足部接触行为。原始轨迹暂存 `/tmp/h1_dds_500_zero_feet/trace.csv`、`/tmp/h1_dds_500_forward/trace.csv` 和 `/tmp/h1_dds_500_turn/trace.csv`。
