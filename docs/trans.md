# H1-2 ONNX 运动控制复现与 robot_nav 集成指引（给外部 AI，2026-10-29）

> **背景**：内部部署代码无法外传。本文是完整规格，让外部 AI 基于宇树官方公开仓库从零复现 ONNX 运动控制器，并把它接入 robot_nav 替换宇树官方 LocoClient 运动控制接口。
> **底座**（全部公开）：
> - `unitree_rl_mjlab`（github.com/unitreerobotics/unitree_rl_mjlab）——含 pt→onnx 导出与 C++ onnxruntime 部署示例
> - `unitree_mujoco`（github.com/unitreerobotics/unitree_mujoco）——官方 sim2sim 仿真器
> - `unitree_sdk2`——DDS 通信（rt/lowcmd、rt/lowstate、rt/wirelesscontroller）
>
> **本文 = 官方基线之上的全部定制清单 + 精确数值 + 集成接口契约**。凡本文未提到的部分，按官方仓库原样即可。

## 0. 交付目标与材料

### 0.1 两个独立交付物

```
现状：Nav2 /cmd_vel → loco_bridge(UDP) → 官方 LocoClient（机器人内置黑盒）─┐
                                                                        ├→ 机器人关节
目标：Nav2 /cmd_vel → rl_bridge(DDS) → ONNX 控制器（自建，rt/lowcmd 直驱）─┘
```

1. **ONNX 控制器**（对标内部 h1_2_ctrl）——50Hz 实时推理，输入速度指令，输出 rt/lowcmd
2. **导航桥接**（对标替换 h1_2_loco_bridge）——/cmd_vel → 控制器指令 topic

### 0.2 内部提供的材料（非代码）

| 材料 | 用途 |
|---|---|
| `policy.onnx`（[1,92]→[1,27]） | 行走策略本体 |
| **配套参数数值表**（= 本文第 2 节全部数值） | default_joint_pos / action scale / PD / 观测顺序 |
| 本文**附录 10（训练配置摘要）** | 理解策略能力边界（指令量程、DR、奖励）——第 5.1 节安全包络的依据都在里面 |

> ⚠️ **模型与参数严格配套**：onnx 换版本时四处必须同步换——default_joint_pos / action offset / ArmHoldStand qs / PD。只换 onnx 不换参数 = 观测/动作分布外，必摔。

## 1. 架构总览

两个独立进程，DDS（CycloneDDS，回环口 lo）互联，与实机完全同构：

```
┌────────────────────┐   rt/lowcmd (50Hz)    ┌────────────────────┐
│  unitree_mujoco     │ ────────────────────▶ │  ONNX 控制器        │
│  （仿真器，含吊带/  │ ◀──────────────────── │  （实机同款二进制） │
│   CSV 记录/键盘）   │   rt/lowstate (500Hz) │                    │
└────────────────────┘ ◀── rt/wirelesscontroller（摇杆/键盘） ──┘
```

控制器主循环 50Hz：lowstate → 拼装 92 维观测 → onnx 推理 → 27 维动作 → 关节目标 + PD → lowcmd。FSM 管模式切换。

**sim2sim 不可跳过**：验证"部署链路会不会把一个好策略跑坏"。训练侧 play.py 用训练同源代码，测不出 C++ 重实现的观测拼装/摇杆换算/状态机 bug；实机之前必须过 sim2sim（内部实修过三个这类 bug：gait_phase 站立不清零、摇杆换算错、FixStand 右臂突变）。

## 2. ONNX 控制器复现规格

### 2.1 FSM 状态机

| 状态 | id | 类型 | 进入按键（键盘/遥控器） | 说明 |
|---|---|---|---|---|
| Passive | 1 | 纯阻尼 | （开机默认） | kd 制动，电机模式 1 |
| FixStand | 2 | 位置伺服 | `1`（LT+up） | 收拢站姿 |
| ArmHoldStand | 3 | 位置伺服 | `2`（RT+X） | **强化增益**保持策略目标站姿 |
| ArmHold | 4 | RLBase | `3`（RT+A） | 策略接管，50Hz |
| 急停回 Passive | | | `0`（LT+B） | 任意状态 |

位置伺服过渡：ts = [0, 3]（3s 插值到目标 qs）。

**增益表（27 关节顺序：左腿6、右腿6、腰1、左臂7、右臂7；H1_2 全身 = 12 腿 + 1 腰 + 7×2 臂）**：

| 状态 | kp | kd |
|---|---|---|
| Passive | 0（纯 kd） | 腿 6.3,6.3,6.3,10.1,1.3,1.3 / 腰 6.3 / 臂 1.3,1.3,0.5×5 |
| FixStand | 腿 98.7,98.7,98.7,157.7,19.7,19.7 / 腰 98.7 / 臂 19.7,19.7,7.9×5 | 同 Passive kd |
| ArmHoldStand | **腿 98.7,98.7,98.7,157.7,100,100 / 腰 98.7 / 臂 100,100,60,120,40×5** | **腿 6.3,6.3,6.3,10.1,4,4 / 腰 6.3 / 臂 6,6,3,4,2×5** |

> ArmHoldStand 必须强化：默认增益下踝（kp 19.7）和左臂（7.9）静态下沉太大（前倾 ~6.6°），策略在接管时刻会看到训练分布外的观测。

**目标姿态 qs**：
- FixStand（收拢）：腿 `[0,-0.2,0,0.5,-0.3,0]×2`，腰 0，双臂 `[0.28,0,0,0.52,0,0,0]`
- ArmHoldStand（= 策略 default_joint_pos + 静态下沉预偏置，**预偏置需在吊带 ~60% 体重支撑下仿真标定**）：
  ```
  腿 [0,-0.45,0,0.90,-0.45,0, 0,-0.45,0,0.90,-0.45,0]，腰 0
  左臂 [0.270,0.001,0.000,0.483,-0.003,-0.030,-0.002]
  右臂 [-0.239,-0.192,0.941,-0.633,0.004,-0.076,0.026]
  ```
  （腿为本项目站姿的名义值；若站姿不同，腿按策略 default_joint_pos 换，臂部预偏置可沿用）

### 2.2 观测拼装（92 维，顺序固定，全部 scale=1，无 clip）

| # | 项 | 维 | 来源/说明 |
|---|---|---|---|
| 0-2 | base_ang_vel | 3 | IMU 角速度（rad/s） |
| 3-5 | projected_gravity | 3 | 重力在基座系投影 |
| 6-8 | command | 3 | 指令换算后的 vx,vy,wz（见 2.4/2.6） |
| 9-10 | gait_phase | 2 | sin/cos(2π·phase)，**period 0.6s，相位时钟恒走；‖cmd‖<0.1 时输出 (0,0)** |
| 11-37 | joint_pos_rel | 27 | 关节角 − default_joint_pos |
| 38-64 | joint_vel_rel | 27 | 关节速度 |
| 65-91 | last_action | 27 | 上一步 onnx 原始输出 |

> gait_phase 站立清零是训练行为，漏掉会引入分布外观测。

### 2.3 动作映射

```
joint_target = offset + scale × act      （act 为 onnx 原始输出，27 维，未 clip）
offset = default_joint_pos
```

**default_joint_pos（27 维，站姿名义值）**：
```
腿 [0,-0.45,0,0.90,-0.45,0]×2，腰 0，
左臂 [0.28,0,0,0.52,0,0,0]，右臂 [-0.2,-0.23,0.9,-0.55,0,0,0]
```

**scale（27 维，= 0.25×力矩上限/kp）**：
```
腿 [0.3333,0.3333,0.3333,0.375,0.25,0.25]×2，腰 0.1667，
臂 [0.5076,0.5076,0.5696,0.5696,0.5696,0.5696,0.5696]×2
```
（髋 yaw/pitch/roll 1/3、膝 0.375、踝 0.25、腰 1/6、肩pitch/roll 0.5076、肩yaw/肘/腕 0.5696）

**策略模式 PD（ArmHold 状态用，与训练 ActuatorCfg 一致）**：
```
stiffness: 腿 [150,150,150,200,40,40]×2，腰 300，臂 [19.7,19.7,7.9×5]×2
damping:   腿 [2,2,2,4,2,2]×2，腰 6，臂 [1.3,1.3,0.5×5]×2
```
> 踝 40 N·m 必须与训练力矩上限一致，否则踝行为分裂（内部曾因 88 vs 40 出过系统性后仰事故）。

### 2.4 摇杆→指令换算（关键，易错）

```cpp
死区: |stick| < 0.05 → 0
换算: cmd = stick > 0 ? stick × upper : stick × |lower|    // 正负向分别缩放
站立判据: hypot(vx,vy,wz) < 0.1 → 视为站立（gait_phase 清零）
```

**量程（非对称！）**：`lin_vel_x [-0.5, 1.0]`、`lin_vel_y [-0.5, 0.5]`、`ang_vel_z [-1.0, 1.0]`。

> 负向乘 0.5 是**有意设计**（满杆后退 = -0.5 m/s、满杆前进 = +1.0 m/s）。忘了非对称处理会让倒走指令翻倍。内部排查时曾因口径混淆（摇杆值 vs 策略侧指令）得出错误结论——**所有指标一律以策略侧指令为准**。

### 2.5 模型

onnx：输入 float32[1,92]，输出 float32[1,27]，onnxruntime CPU 推理，50Hz。pt→onnx 用官方 unitree_rl_mjlab 的导出脚本（torch.onnx.export，actor MLP，固定 batch=1，opset ≥ 17）。

### 2.6 导航接口三件套（接入 robot_nav 必须新增）

复现上述控制器吃"遥控器摇杆"。接 Nav2 需要程序化接口，**加在摇杆换算之后**（不要让桥接侧反演死区/非对称缩放，那是坑）：

**① 外部速度指令 topic**
- DDS topic：如 `rt/cmd_vel_ext`（三浮点 vx/vy/wz，单位 m/s、m/s、rad/s）
- 语义：**策略侧指令口径**，控制器内 clamp 到 2.4 节量程
- **watchdog：>0.5s 无新消息 → 指令归零**（策略零指令=站立，天然对标 LocoClient 的 timeout→StopMove）
- 优先级：外部指令新鲜时覆盖摇杆；遥控器始终保留（手动兜底 + 急停）

**② FSM 模式控制 topic**
- 如 `rt/mode_cmd`（int：0=Passive 1=FixStand 2=ArmHoldStand 3=ArmHold）
- **互锁必须做**：ArmHold 只允许从"ArmHoldStand 且已稳定 ≥3s"进入（关节角停在 default_joint_pos 附近，偏差 <0.07 rad）；非法转移拒绝并回 NACK
- 急停：任何时候可回 Passive（纯阻尼）

**③ 状态反馈 topic**
- 如 `rt/ctrl_state`（当前 FSM 状态 + 就绪标志 + 实际基座速度），桥接侧转 ROS2 发布
- 导航侧用它 gate：**ArmHold 未就绪不接受 Nav2 目标点**

## 3. 仿真器定制（unitree_mujoco 之上）

1. **场景 XML**：官方 scene_h1_2.xml → 加 DEX5-1 五指手 + **右手 0.25 kg 负载**（对齐训练物理）。生成方式：脚本从官方 XML 替换手部 body/mesh 并加质量。
2. **弹性吊带（保护用，评测期定时释放）**：
   - 刚度 200 N/m、阻尼 200、**零力高度 0.90 m**（吊带力 = k×(0.90 − z)，行走期骨盆 ~0.95 时零力）
   - 跟随模式：锚点 xy 实时跟随骨盆（只提供竖直吊力，无水平回拉），锚点 z = 3.0 m
   - 定时释放：t=20s 开始、t=22s 完全松开（评测协议）
3. **CSV 记录**：100Hz dump，字段 `t,lx,ly,rx,ry,x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,wall`。分析用：跟踪率、俯仰 `2·atan2(qy,qw)`、yaw、骨盆 z。
4. **wait_for_ctrl**：收到第一条 lowcmd 前不步进物理（消除 DDS 连接延迟造成的零力矩自由落体窗口）。
5. **键盘摇杆模式**（无需手柄）：`w/s` = ly ±0.1/按、`a/d` = lx、`q/e` = rx、空格归零、`1/2/3/0` = 状态机按键（2.1 表）。摇杆数据经 `rt/wirelesscontroller` DDS 走全链路。
6. **渲染**：必须 `LIBGL_ALWAYS_SOFTWARE=1`（软件渲染防 GPU 驱动崩溃；也方便无显卡服务器跑）。

## 4. sim2sim 评测协议与基准

**调度**（定时自动切换）：`FixStand: 8s → ArmHoldStand: 10s → ArmHold: 95s`，共 113s；策略 t=18s 接管，吊带 20→22s 释放。**指令在策略接管并站稳后再给**（站立 ≥30s 后，模拟实机操作）。

**判据**：
- 摔倒 = 骨盆 z < 0.75 m；有效行走 = 全程 z > 0.90（吊带零力，即无保护）
- 指标：跟踪率（|实际 vx 均值 / 策略侧指令|）、俯仰最低、yaw 漂移、骨盆 z 最低
- 公平性：`taskset` 绑核避开后台负载（墙钟/仿真倍率偏离 1 超 3% 的跑作废）

**内部基准（前进/站立全过）**：

| 场景 | 指令（策略侧） | 结果 |
|---|---|---|
| 纯站立 | 0 | 40.8s 零迈步、站高 0.95、俯仰 ≤±1° |
| 前进 | +0.3 / +0.5 / +1.0，各 ×2 跑 | 113s 全程零摔，跟踪 87~96% |
| 倒走 | -0.10 / -0.15 / -0.25 / -0.5（含站稳后突加 -0.5 ×2） | 全部不摔；-0.5 稳态 -0.34（67%），俯仰最低 -2.3°，yaw +0.7°/s |

复现后若前进/站立达到同量级（零摔、跟踪 >85%），链路即判定等价。倒走表现与模型训练强相关，只作参考项。

## 5. robot_nav 集成改动清单

| # | 改动 | 说明 |
|---|---|---|
| 1 | 新包 `h1_2_rl_bridge`（或改造 loco_bridge） | 订阅 `/cmd_vel_nav`（平滑后），发布 `rt/cmd_vel_ext`；订阅 `rt/ctrl_state` 转 `/h1_2/rl_state`；**退役 UDP daemon 与 LocoClient 调用** |
| 2 | Nav2 velocity smoother 包络重调 | 见 5.1 数值——**这是安全边界，不是调参项** |
| 3 | 启动编排（launch/脚本） | 开机序列：Passive→FixStand(8s)→ArmHoldStand(3s 稳定)→ArmHold，轮询 ctrl_state 确认就绪后再起 Nav2 |
| 4 | 行为树/导航入口加就绪检查 | rl_state 不就绪时拒绝新目标 |
| 5 | DDS 网络配置 | 控制器跑机器人本机（或同 LAN x86），Nav 主机与它同 CycloneDDS 域（livox 话题已走通跨机 DDS，链路现成） |

### 5.1 velocity smoother 数值（初始包络）

| 量 | 上限 | 依据 |
|---|---|---|
| vx 前进 | 0.5 m/s | sim2sim 验收过 0.5/1.0，导航从保守起步 |
| **vx 后退** | **-0.3 m/s（硬上限）** | 倒走在训练分布边界（训练只采样到 -0.5），实机曾摔倒，调查未结案；**倒退导航需求出现前不许放开** |
| vy | ±0.3 m/s | 训练范围 ±0.5 收窄 |
| wz | ±0.8 rad/s | 训练 ±1.0 收窄（倒走伴 0.7°/s 航向漂移，转向余量留足） |
| 加速度 | 前进 ≤0.5 m/s²，wz ≤1.0 rad/s² | 满杆突加是实机摔倒触发方式之一，导航指令必须斜坡化 |
| **指令下限** | **‖cmd‖ < 0.1 策略不响应（站立）** | 训练 stand_still 奖励死区。**Nav2 接近目标点的微调速度若低于 0.1 m/s 机器人不会动**——goal tolerance、到位判定、行为树超时必须按此设计（不要试图用更小的速度做精细位置伺服） |

## 6. 验证路径（顺序不可换）

```
① 控制器单元 sim2sim（unitree_mujoco，按第 4 节协议）
   —— 前进 0.3/0.5 各 2 跑零摔 + 跟踪 >85% 才算控制器合格
② 桥接联调 sim2sim —— /cmd_vel 由测试脚本发（模拟 Nav2），整链验证：
   watchdog（停发指令 1s 机器人站住）、模式切换互锁、状态反馈
③ 实机低速 —— 0.2 m/s 以下、空旷场地、遥控器急停在手、观测黑匣子常开
④ 实机 + Nav2 —— 先遥控器跟随验证，再上自主导航
```

## 7. 踩坑清单（复现必读）

1. **onnx 推理逐项对齐训练**：观测顺序、scale、default_joint_pos、动作 scale/offset、PD——任何一处不一致都表现为"sim 里好好的，sim2sim/实机里摔"，极难从摔倒行为倒查。
2. **gait_phase 站立清零**、**负向指令减半**、**踝力矩上限 40** 是内部实修过的三个链路 bug，复现时优先自检。
3. **"play.py 能跑就行"是错的**：play.py 与部署 C++ 是两套代码，链路 bug 只有 sim2sim 能测出来。
4. DDS：同机用 lo；CycloneDDS 建议 noshm 配置；WSL2 下 DDS 互连有环境问题，建议原生 Linux。
5. `LIBGL_ALWAYS_SOFTWARE=1`；后台负载用 taskset 绑核，否则实时性破坏数据有效性。
6. **口径纪律**：报跟踪率写清楚摇杆值还是策略侧指令（负向差 2 倍）；分析脚本的指令重建必须复刻 2.4 非对称换算。
7. ArmHoldStand 静态下沉预偏置必须在"吊带部分承重"工况下标定，全重/零重标出的都会偏。
8. 接管瞬态：ArmHold 接管时关节角必须已停在 default_joint_pos 附近（偏差 < 训练关节偏移 DR ±0.07 rad），否则接管即分布外。

## 8. 自检清单（按序打勾）

- [ ] onnx 输入输出 shape = [1,92] / [1,27]
- [ ] 92 维观测逐段核对（含 gait_phase 站立清零）
- [ ] 动作 target = default_joint_pos + scale×act，踝目标范围合理（|target| < 关节极限）
- [ ] 摇杆：死区 0.05、负向 ×0.5、站立判据 0.1
- [ ] 四处联动一致：default_joint_pos / action offset / ArmHoldStand qs / PD
- [ ] 仿真器：软件渲染、wait_for_ctrl、吊带参数、CSV 字段
- [ ] 导航接口三件套：cmd_vel_ext watchdog / mode_cmd 互锁 / ctrl_state 反馈
- [ ] velocity smoother 包络按 5.1 收紧（后退 -0.3 硬上限）
- [ ] 跑通"纯站立 40s 零迈步"再跑行走
- [ ] 前进 0.3/0.5 各 2 跑零摔 + 跟踪 >85% → 控制器合格
- [ ] 桥接联调三件套全过 → 实机

## 9. 工作量参考（供排期）

| 项 | 估时 |
|---|---|
| ONNX 控制器复现（含 sim2sim ①） | 3-5 天 |
| 桥接包 + 接口三件套（②联调） | 2-3 天 |
| robot_nav 集成 + 启动编排 | 1-2 天 |
| 实机验收 ③④ | 1-2 天 |
| 合计 | 7-12 天 |

## 10. 附录：训练配置摘要（理解策略能力边界）

> 来源：训练 run 的 env.yaml（1899 行解析式 dump）人工提炼。完整原文备查。相对训练基线的改动见训练仓库 diff。

### 10.0 最关键的发现（倒走安全上限的直接依据）

- **指令课程 stage2（lin_vel_x [-0.8, 1.2]）在 120k 步才解锁，本 run 只训到 30k——整个训练期间倒走指令实际采样范围是 stage1 的 [-0.5, 1.0]**
- 实机摔倒时的策略侧指令 **-0.47 恰好贴在训练采样边界 -0.5 上**（边界处采样密度最低、策略最少见）
- 训练环境 diagnose：-0.3 → act_vx -0.22（73%，界内表现正常）；sim2sim -0.5 → -0.34（67%）；实机 -0.47 → 摔倒
- **这就是 5.1 节后退硬上限 -0.3 的完整证据链，不许放开**

### 10.1 基本设置

| 项 | 值 |
|---|---|
| num_envs / seed / episode_length | 4096 / 42 / 20 s |
| sim timestep / decimation | 0.005 s / 4（策略 50 Hz） |
| 求解器 | MuJoCo implicitfast + newton（10 iter），pyramidal cone，impratio 1.0 |
| 地形 | 平面（terrain_type: plane） |
| 机器人 | h1_2 + DEX5-1 手 + 右手负载 |

### 10.2 指令与课程（commands.twist）

| 项 | 值 |
|---|---|
| 重采样间隔 | 3~8 s |
| 采样范围（基础 cfg） | lin_vel_x **[-1.0, 2.0]**，lin_vel_y [-1,1]，ang_vel_z [-1,1]，heading ±π |
| 课程 stage0（step 0，**实际生效**） | lin_vel_x **[-0.5, 1.0]**，lin_vel_y [-0.5,0.5]，ang_vel_z [-1,1] |
| 课程 stage1（step 120000，**未及解锁**） | lin_vel_x [-0.8, 1.2]，lin_vel_y [-0.6,0.6] |
| heading_command | true（stiffness 0.5） |
| 站立环境占比 | rel_standing_envs 0.15 |
| heading/原地转占比 | rel_heading_envs 0.5 / rel_turn_in_place_envs 0.1 |
| heading 漂移收敛 | settle 1.0 s，aligned 0.2 rad |

### 10.3 观测（actor，92 维，顺序）

| 项 | 维 | 噪声 U(-n,n) |
|---|---|---|
| base_ang_vel（imu） | 3 | ±0.2 |
| projected_gravity | 3 | ±0.05 |
| command（twist） | 3 | 无 |
| phase（period 0.6 s，站立时清零） | 2 | 无 |
| joint_pos rel（**含 encoder bias**） | 27 | ±0.01 |
| joint_vel rel | 27 | ±1.5 |
| last_action | 27 | 无 |

critic 观测同 actor（joint_pos 不带 bias）。

### 10.4 动作

joint_pos 目标 = default_joint_pos + scale × act + offset；`use_default_offset: true`。

| 关节 | scale |
|---|---|
| hip_yaw / hip_pitch / hip_roll | 1/3 |
| torso | 1/6 |
| knee | 0.375 |
| ankle_pitch / ankle_roll | 0.25 |
| shoulder_pitch / roll | 0.5076 |
| shoulder_yaw / elbow / wrist | 0.5696 |

### 10.5 奖励（权重与关键参数）

| 项 | 权重 | 关键参数 |
|---|---|---|
| track_linear_velocity | 1.0 | std 0.5 |
| track_angular_velocity | 1.0 | std 0.707 |
| heading_drift | 0.5 | std 0.25 |
| pose | 1.0 | 站立 std 0.05；行走/奔跑按关节分档（hip_pitch/knee 0.5→0.25 等，阈值 0.1/1.5），**不含右臂** |
| foot_gait | 0.5 | period 0.6，offset 0/0.5，threshold 0.56，cmd_threshold 0.1 |
| right_arm_hold_coarse / fine | 0.5 / 1.0 | std 0.3 / 0.1 |
| body_orientation_l2 | -1.0 | torso_link |
| foot_clearance | -1.0 | 目标 0.1 m |
| foot_slip | -0.25 | |
| stand_still | -1.0 | cmd_threshold 0.1（**<0.1 判站立——5.1 节指令下限的依据**） |
| self_collisions | -1.0 | 力阈值 10 |
| action_rate_l2 | -0.05 | （本 run 20k 后曾爆炸后自恢复，见训练曲线） |
| joint_pos_limits | -10.0 | |
| is_terminated | -200.0 | |
| body_ang_vel | -0.05 | |
| angular_momentum | -0.025 | |
| joint_acc_l2 | -2.5e-7 | |
| soft_landing | -0.001 | |

终止：time_out + fell_over（limit_angle 1.222 rad ≈ 70°）。

### 10.6 域随机化（events）

| 项 | 模式 | 范围 |
|---|---|---|
| reset_base | reset | yaw ±3.14，roll ±0.1，**pitch ±0.14**，xy ±0.5 m，速度各轴 ±0.3 |
| reset_robot_joints | reset | 关节位置偏移 **±0.07** |
| push_robot | interval 5-6s | vx/vy ±0.5，vz ±0.4，roll/pitch ±0.52，yaw ±0.78 |
| foot_friction | startup | 0.3~1.6（14 个脚底 geom 共享） |
| encoder_bias | startup | **±0.03**（注：实机已知恒定零偏右踝 -0.025、髋yaw ±0.027 在此包络内） |
| base_com | startup | torso x [-0.1,+0.05]，y ±0.05，z ±0.05 |
| hand_payload | startup | 右手 +0~0.5 kg |
| torso_mass | startup | -1~+3 kg |
| pd_gains | startup | kp/kd ×0.8~1.2 |
| action_delay | reset | **滞后 0~4 步（0~80 ms @50Hz）** |
| reset_joint_position_target | reset | 任务自定义 |

### 10.7 执行器（训练 ActuatorCfg）

- 增益分组（stiffness/damping/effort_limit）：150/2.0/200、300/6.0/200、200/4.0/300、**40/2.0/40（踝，与实机 40 N·m 对齐——2.3 节 PD 的来源）**、19.7/1.3/40（臂 coarse）、7.9/0.5/18（臂 fine）；armature 0.025
- 执行器模型延迟：delay_target position，lag 0~4 步

### 10.8 初始姿态（default_joint_pos）

- 腿：hip_pitch -0.45，knee 0.9，ankle_pitch -0.45（stance095 站姿，骨盆 ~0.95 m）
- 右臂（hold 目标）：shoulder_pitch -0.2，roll -0.23，yaw 0.9，elbow -0.55
- 左臂：shoulder_pitch 0.28，elbow 0.52
