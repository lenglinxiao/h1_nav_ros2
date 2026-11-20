# ONNX 手动测试网页

- 状态：已部署并启动网页；现场动作验收待执行
- 目标：为既有受保护的 ONNX 人工短测提供仅本机访问的网页入口，操作者可选择方向、速度和时长，并通过网页完成 runner 的现场确认与撤权交接。
- 范围：复用 `run_acceptance.py` 的策略准备、monitor、保护和 ArmHold；速度限于 `|vx| <= 0.12 m/s`、`vy = 0`、`|wz| <= 0.1 rad/s`、单次 `0.5–3.0 s`；版本和训练 URDF 从运行环境自动识别；服务默认绑定 `127.0.0.1`。
- 非目标：网页直接发布 LowCmd、Nav2 测试、扩展速度包络、自动执行现场准备、远程无令牌访问、自动 RELEASE/撤权。
- 步骤：
  1. 为 runner 增加受限速度分量参数，并自动记录源码和 URDF 身份。
  2. 新增 localhost Web 服务，使用 PTY 驱动原 runner 的交互确认，不绕过其保护流程。
  3. 增加前端方向/速度/时长选择、状态显示、姿态确认和支撑后撤权操作。
  4. 覆盖参数边界、生命周期/API 与离线 UI 测试；运行相关测试和语法检查。
- 验收标准：无显式网页确认不得启动 runner；无效/越界参数在启动前拒绝；单实例、单会话；runner 异常或浏览器控制中断不自动重试；普通停止路径只在操作者明确确认支撑后撤权；页面展示软件观测与现场观察的区别。
- 依赖与风险：硬件测试必须在已确认调试模式、独占控制、支撑/姿态满足现场流程时由操作者启动。反向及右转方向尚无现场验证，本工具只提供受限的人工试验入口，不代表已验收。

## 操作步骤

本流程使用独立的 ONNX 手动速度页 `8781`，不是原导航网页 `8765`，也不是 Nav2 路线测试页 `8770`。动作会启动受保护的 Policy runner 并向机器人发送短时速度；开始前必须现场确认控制权唯一、调试模式正确、机器人和双臂有可靠支撑且急停可用。历史现场验证覆盖前进和左转；后退和右转仍属待验证方向。

1. 在 PC4 现场终端确认没有另一场手动测试会话或 Policy 输出会话占用控制权，并完成急停、支撑、调试模式和独占控制检查。若 writer 来源/控制权不清楚，或机器人无法可靠支撑，停止在这里，不启动测试。
2. 在 PC4 启动已部署的手动测试页。RC2 部署路径示例：

   ```bash
   cd /home/unitree/h1_nav2_ros2_ws/releases/onnx-manual-test-20261112-rc2
   source /opt/ros/humble/setup.bash
   python3 tools/h1_2_policy_hardware/manual_test_web.py
   ```

   保持该终端和网页服务运行。服务只绑定 PC4 的 `127.0.0.1:8781`。
3. 在自己的电脑另开终端建立 SSH 隧道：

   ```bash
   ssh -N -L 8781:127.0.0.1:8781 unitree@192.168.123.164
   ```

   浏览器打开 `http://127.0.0.1:8781`。若本机 8781 已被占用，可将本地端口改为 8782，例如 `-L 8782:127.0.0.1:8781`，然后访问 `http://127.0.0.1:8782`。
4. 先选方向、速度和时长。线速度范围 `0.01–0.12 m/s`，角速度范围 `0.01–0.1 rad/s`，单次 `0.5–3.0 s`，横向速度固定为零。首次/保守试验应使用较低速度和较短时间，并由现场人员持续观察；不要把网页上限当作推荐值。
5. 仅在现场条件真实满足时勾选调试模式、独占控制、支撑/姿态三项确认，点击“准备测试”。等待 runner 完成准备；出现姿态确认提示后，现场确认姿态和支撑条件仍满足，再点击“确认姿态并继续”。测试期间随时观察机器人，出现异常立即按现场急停流程处理，不要依赖网页按钮或计时器作为急停。
6. 单次脉冲结束后确认机器人已归零并处于 ArmHold。状态为“零速 ArmHold，可执行下一次”时，才可更改参数并点“执行下一次”；每次请求单独执行，不会排队。任何异常、保护触发、状态不完整或动作结果不确定时，不再点下一次。
7. 结束整场测试时，先由现场人员可靠支撑机器人和双臂，再点击“结束测试：已支撑，确认撤权”。等待 runner 退出/撤权完成，并查看运行记录；只有状态/日志确认流程结束后，才关闭浏览器隧道和 PC4 服务终端。关闭浏览器本身不会停止 runner，也不能代替撤权。

若页面显示 `awaiting_revoke`、`RELEASE` 被保护阻断或训练保护进入 Passive，保持现场支撑，不重启会话、不重复动作；按页面提示先完成可靠支撑，再明确确认撤权。若网页断开，PC4 runner 可能仍在运行，需从 PC4 服务页面/API或终端日志核实状态，不能把断开当作停止。

## 进度与验证

### 2026-11-12 前后运动感觉异常的日志核查

- 现场状态：此前训练保护会话 `acceptance-20261112T143541` 已写出 summary，`trained_guard_handoff=operator_confirmed_REVOKE`；随后网页启动了新会话 `acceptance-20261112T150320`（runner PID `15730`、Policy PID `15755`）。最新只读 API 为 `ready_next`，会话仍 active，网页当前显示左转 `0.1 rad/s、3 s`。runner仍持有repeat事件，退出前完整 `repeat_walks` summary 未落盘，不能从已保存记录准确列出每次方向与触发时刻。
- 新会话节点日志：Policy joint 10 的位置PD torque projection 在累计50帧时报告 `raw_q=0.384703`、`projected_q=0.142301`、`sample_tau=40`，之后当前 trace 中该关节速度绝对峰值 `6.4673 rad/s`。Policy顺序的joint 10对应右踝pitch；训练速度保护中对应SDK index 10限值为8 rad/s，因此未因该速度触发训练保护。该投影会显著改变请求目标，可能影响动作表现，但现有数据未把这段投影/峰值与前进或后退某次脉冲精确关联，不能作为其方向错误的确认。
- 控制映射代码确认 forward 使用 `vx=+speed`、backward 使用 `vx=-speed`。软件映射本身没有发现符号反转；仍需完整session summary及每轮时间窗trace，再结合现场动作观察判断实际方向、步态与参数是否匹配。
- 处理：当前会话保持 `ready_next`，暂不继续提交动作；确认可靠支撑后由现场操作者结束并REVOKE，以便summary落盘。已做只读日志/API/进程检查，没有改代码或发送命令。

### 2026-11-12 训练保护中止记录

- 现象：CONFIRM 后网页显示 `awaiting_revoke`，用户认为流程直接退出。
- 已确认：PC4 runner PID `7470` 与 Policy PID `7499` 当时仍运行；网页 API 为 `active=true`、`exit_code=null`、`state=awaiting_revoke`。节点日志记录训练保护因 SDK index 17 `left_wrist_roll_joint` 速度 `0.506213665 rad/s` 超过 profile 限值 `0.5 rad/s` 而锁存；保护请求 Passive 原生阻尼，并等待物理交接。保护触发后的 status 是 `policy_cmd=0,0,0`、`navigation_ready=false`。最终 summary 随后确认前进命令 `[0.12, 0, 0]` 的 `manual_nonzero_published=true`、`manual_policy_nonzero_observed=true`，但 `manual_policy_zero_in_armhold_observed=false`；因此有非零输出证据，却没有完整0.5秒窗口结束及归零的证据，实际动作可能在脉冲中途被保护截断。保护后的零状态不能反推之前从未发过非零命令。
- 与此前成功试验的差异：本次记录实际触发了关节速度门限；既有成功记录没有该次 SDK17 越限证据。现有遥测只能确认关节速度门限触发，不能确认造成左腕滚转速度峰值的机械/控制原因，也不能据此归因于 RC2 网页重复测试改动。
- 处理：不在锁存会话内重试或重启。确认机器人及双臂已有可靠外部支撑后，按页面要求完成 `REVOKE` 交接；Passive 和零 `policy_cmd` 本身不证明机械支撑。summary 已落盘；物理运动方向、完整时长和峰值的机械原因仍需现场观察与 trace 关联。
- 验证范围：PC4 只读核对进程、网页 API、runner node.log 和 trace；未发运动命令、重启进程或撤权。

### 2026-11-12 连续测试修正（本地实现完成）

- 问题：网页只启用单次 runner，脉冲后进入等待 REVOKE，导致每次测试都要退出并重新准备。
- 处理：复用原 runner 的 ArmHold repeat 循环，新增仅网页协议启用的有界方向/速度/时长请求；每次完成并确认策略非零输出及归零后才允许下一次，不排队自动执行。撤权只结束整场会话；故障或租约取消不开放继续。普通终端 `WALK` 保持兼容。
- 验收：同一 Policy/runner 进程两次不同参数脉冲、归零、无再次准备/撤权；旧 WALK 兼容，越界与运行中请求拒绝，撤权和故障保护不变。
- 现场只读证据：PC4 网页状态 active/awaiting_revoke，runner PID3787、Policy PID3812；日志观察前进0.12m/s、0.5秒非零及ArmHold归零。物理迈步/站稳尚无本轮用户确认。不重启或修改该活动实例，不发下一次动作或撤权。
- 本地 fake workflow 验证：同一 runner 中左转 `(0,0,0.05)` 1秒和前进 `(0.08,0,0)` 0.5秒均观察到非零/归零，FixStand 与 ArmHoldStand 各只执行一次，最后单独 REVOKE；越界、非 idle 与故障观测不可继续。关联测试套件96 passed（76条既有 NumPy reload warning），py_compile 和 diff check通过。
- RC1 网页不会热更新，新 repeat 流程尚未部署。部署时需先只读确认没有活动 runner；活动 runner 不覆盖、不重启、不撤权。
- 部署完成：独立 RC2 `/home/unitree/h1_nav2_ros2_ws/releases/onnx-manual-test-20261112-rc2`，网页 SHA256 `9eb4defd4fa6291da72bff39714f86c07183b385dcc8f0bb919cd9e5c6515f96`，runner SHA256 `d17c10dd103452ab191e7f842e85cb42a81fef442c0397a1e51aa950dd1476cb`，repeat input SHA256 `a8beafeafa1f09a5e3b3ab66d863bf0044d8faa8cbdfce60a9c27307dbd8192a`。部署前确认无网页/runner/Policy 进程。PC4 编译成功；网页 PID `6092` 监听 loopback 8781，首页HTTP200（8441 bytes），API idle。没有启动 runner或运动动作。用户可在单次速度完成并归零后点击“执行下一次”更换参数；只有整场结束才点“结束测试：已支撑，确认撤权”。

- [x] 查明现有人工 runner 与 Web GUI 的扩展边界。
- [x] runner 参数及运行身份自动记录。
- [x] localhost 测试 Web 与受限交互控制。
- [x] 软件测试、文档和会话交接。
- 验证：`source /opt/ros/humble/setup.bash && python3 -m pytest -q tools/h1_2_policy_hardware/test_manual_pulse.py tools/h1_2_policy_hardware/test_manual_test_web.py tools/h1_2_policy_hardware/test_acceptance_offline.py tools/h1_2_policy_hardware/test_acceptance_workflow.py`：89 passed（75 条 NumPy reload warning）；`python3 -m py_compile` 涉及的三个 Python 文件、`bash -n run_ai_slam_nav.sh stop_ai_slam_nav.sh`、`git diff --check` 均通过。CLI dry-run 确认 `backward 0.08 m/s` 映射为 `(-0.08, 0, 0)`，未启动 ROS/硬件。
- 2026-11-11 板端部署：独立 release `/home/unitree/h1_nav2_ros2_ws/releases/onnx-manual-test-20261111-rc1`；网页 SHA256 `aad2c5bcc0a15ed877d47fa44f97eb45451e237436ec7db369a3aa0b25937497`，runner SHA256 `06312dd6cb5c6d4722dc77a00657a0e370f0d106a56ede207c8e0e083fca9982`，关键 helper 与开发机 hash 一致。PC4 ROS Humble Python 编译与训练 profile/手动参数 dry-run 成功，后退示例 `0.08 m/s` 映射为 `(-0.08,0,0)`。网页进程 PID `3509` 正监听 `127.0.0.1:8781`，首页 HTTP 200（7757 bytes），`/api/status` 为 `idle`。旧 GUI、主工作区和 Owner 未改变；没有启动 Policy/runner、授权输出或发送运动命令。后退和右转现场行为仍未验证。
- 2026-11-12 刷新：首次检查时开发机有线接口 `NO-CARRIER`，PC4 8781 已退出。链路恢复后重新启动独立 release 网页，当前 PID `7450`，绑定 `127.0.0.1:8781`，首页 HTTP 200（7757 bytes），status `idle`。Owner `inactive`，ROS 图仅有既有 `/h1_2_nav_web_gui`、无 action。只读 monitor 7秒：3490 LowState、3486 LowCmd、CRC0、age `0.001140396`秒、mode6/PR0、`policy_commands=0`、`unknown_commands=3486`；该旧 monitor schema 未提供 `lowcmd_writers` 字段，故来源/控制权仍未知。没有启动 Policy/runner、授权输出或发送运动命令。网页功能可访问，实机动作准入未确认。
