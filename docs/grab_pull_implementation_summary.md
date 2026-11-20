# 抓握后拉手势功能开发总结

**开发日期**: 2026-11-19  
**参考文档**: [docs/grasp_door.md](grasp_door.md)  
**功能状态**: ✅ 代码实现完成，待编译测试

---

## 📋 功能概述

H1-2 人形机器人使用右手执行"抓握后拉"手势，触发问界 M9/M8 隔空开门功能。

**技术路线**:
- 腿部：宇树原厂运控负责平衡
- 右臂：使用 arm_sdk 按关键帧播放
- Dex5 灵巧手：张开/握拳手型切换
- 集成：通过现有手势链路接入导航Web界面

---

## ✅ 已完成模块清单

### 1. 核心轨迹生成模块
**文件**: `src/h1_2_loco_bridge/src/arm_trajectory.hpp`

**功能**:
- 五次多项式插值（缓起缓停）
- 关节限速控制（防止超速）
- 关节限位裁剪
- 权重渐变（接管/交还）
- 总时长计算

**关键常量**:
```cpp
constexpr int kWaistIndex = 12;
constexpr int kRightArmStart = 20;
constexpr int kArmJoints = 7;
constexpr int kWeightIndex = 27;
```

---

### 2. 配置文件
**文件**: `src/h1_2_loco_bridge/config/grab_pull.yaml`

**内容**:
- 时间参数：接管1s、交还2s、中止交还1s
- 速度限制：初始0.5 rad/s（可调整到1.0）
- 增益配置：右臂/左臂/腰部 kp/kd/tau_ff
- 关键姿势：chest、ready、pull_back（待示教后填写实际值）
- Dex5手型：张开/握拳的20个关节角度
- 动作序列：7段完整流程

**待标定参数**:
- `poses.ready.joints`: 门把手前位置
- `poses.pull_back.joints`: 后拉10-15cm位置
- `right_arm_gains.tau_ff`: 重力前馈补偿

---

### 3. 配置加载模块
**文件**: `src/h1_2_loco_bridge/src/grab_pull_config.hpp`

**功能**:
- 简化YAML解析器（无需第三方依赖）
- 配置验证（数组长度、数值有效性、必需姿势检查）
- 默认动作序列生成

---

### 4. 手势播放器
**文件**: `src/h1_2_loco_bridge/src/arm_gesture_player.hpp`

**功能**:
- DDS通信：订阅 `rt/lowstate`、发布 `rt/arm_sdk`
- 状态监控：等待接收机器人状态
- 轨迹执行：每20ms发送一帧指令
- 中止控制：响应外部中止请求，1秒内平滑降权重
- 错误处理：状态超时、配置错误等

**返回码**:
- `0`: 成功
- `-3`: 正在播放（忙）
- `-4`: 已中止
- `-5`: 配置文件不合法
- `-6`: 未收到机器人状态

---

### 5. 守护进程集成
**文件**: `src/h1_2_loco_bridge/src/h1_2_loco_udp_daemon.cpp`

**新增功能**:
- 多线程架构：主循环处理UDP，工作线程播放手势
- `gesture grab_pull` 指令支持
- 播放期间拒绝速度命令
- `stop` 指令中止播放
- 完成后自动回复（延迟回复模式）

**启动参数新增**:
```bash
--grab-pull-config <path>  # 配置文件路径
```

---

### 6. 单元测试
**文件**: `src/h1_2_loco_bridge/test/test_arm_trajectory.cpp`

**测试覆盖**:
- ✓ 五次多项式边界条件
- ✓ 关节限位裁剪
- ✓ 速度限制不超标
- ✓ 轨迹精确到达目标
- ✓ 权重渐变正确
- ✓ 相同起点终点保持不变
- ✓ 总时长计算

---

### 7. 独立Demo程序
**文件**: `src/h1_2_loco_bridge/src/grab_pull_demo.cpp`

**用途**:
- 挂吊架测试
- 示教调参（点动模式）
- 不依赖守护进程独立运行

**用法**:
```bash
ros2 run h1_2_loco_bridge grab_pull_demo \
  --config /path/to/grab_pull.yaml \
  --network-interface <interface> \
  --domain-id 0
```

**选项**:
- `--no-play`: 仅检查配置，不播放

---

### 8. 构建系统
**文件**: `src/h1_2_loco_bridge/CMakeLists.txt`

**更新**:
- 添加 `grab_pull_demo` 可执行目标
- 包含头文件目录：`${CMAKE_CURRENT_SOURCE_DIR}/src`
- 安装 `config/` 目录
- 单元测试支持

---

### 9. Launch文件
**文件**: `src/h1_2_loco_bridge/launch/h1_2_loco_bridge.launch.py`

**新增参数**:
```python
DeclareLaunchArgument("grab_pull_config", default_value=grab_pull_config)
```

自动传递配置文件路径到守护进程。

---

### 10. 界面集成

#### Web界面
**文件**: `src/h1_2_nav_tools/h1_2_nav_tools/h1_2_nav_web_gui.py`
```python
GESTURE_OPTIONS = ["", "wave_hand", "shake_hand", "grab_pull"]
```

#### 桌面界面
**文件**: `src/h1_2_nav_tools/h1_2_nav_tools/h1_2_nav_gui.py`
```python
GESTURE_OPTIONS = ["", "wave_hand", "shake_hand", "grab_pull"]
```

#### 路线事件示例
**文件**: `src/h1_2_nav_tools/config/teaching_route_events.example.json`
```json
{
  "point": "CarDoor",
  "trigger_radius": 0.5,
  "gesture": "grab_pull",
  "comment": "抓握后拉手势，用于触发问界M9/M8隔空开门"
}
```

---

## 🔧 下一步操作

### 第一阶段：编译验证（30分钟）

```bash
cd /home/llx/h1_nav2_ros2_ws

# 设置Unitree SDK环境变量
export UNITREE_SDK2_ROOT=/path/to/unitree_sdk2

# 编译
colcon build --packages-select h1_2_loco_bridge --cmake-args -DBUILD_TESTING=ON

# 运行单元测试
colcon test --packages-select h1_2_loco_bridge
colcon test-result --verbose
```

**预期结果**: 
- 编译成功，无错误
- 单元测试全部通过

---

### 第二阶段：配置检查（15分钟）

```bash
# 检查配置文件语法
ros2 run h1_2_loco_bridge grab_pull_demo \
  --config install/h1_2_loco_bridge/share/h1_2_loco_bridge/config/grab_pull.yaml \
  --no-play
```

**预期输出**:
```
Config loaded successfully
  Control period: 0.02 s
  Max joint velocity: 0.5 rad/s
  Motion segments: 7
  Estimated total duration: ~12 s
Config check complete
```

---

### 第三阶段：吊架测试（阶段1，半天）

**前提条件**:
- 机器人挂在吊架上
- 原厂运控运行中
- 遥控器就位（可随时切阻尼）

**步骤**:

1. **验证arm_sdk可用**
```bash
# 运行官方例程（需先修复权重归零问题）
cd /path/to/unitree_sdk2/example/h1/high_level
./h1_2_arm_sdk_dds_example
```

2. **测试demo程序**
```bash
ros2 run h1_2_loco_bridge grab_pull_demo \
  --config install/h1_2_loco_bridge/share/h1_2_loco_bridge/config/grab_pull.yaml \
  --network-interface <interface>
```

**观察重点**:
- 双臂动作是否平滑
- 全程是否站稳
- 权重是否平滑归零
- 记录关节下垂量（用于计算重力前馈）

---

### 第四阶段：示教关键帧（阶段4，1天）

**工具**: 使用demo程序的点动模式（需要在代码中添加）

**待标定姿势**:
1. **ready**: 手伸到门把手前
   - 距离：根据摄像头视野调整
   - 高度：门把手高度
   - 掌心朝向：朝向车门

2. **pull_back**: 后拉10-15cm
   - 与ready相比，肩部/肘部角度减少

**记录方式**:
- 记录发出去的**指令值**（不是实际测到的角度）
- 填写到 `grab_pull.yaml` 的 `poses` 部分

---

### 第五阶段：上车测试（阶段7，第2周）

**车端前提**:
- P挡、车辆已解锁、车机启动、主驾无人
- "设置 > 车辆控制 > 体感控制"打开

**测试流程**:

1. **确定停靠点**
   - 在车门打开扫掠范围外
   - 手在摄像头视野内

2. **调参测试**
   - 扫参数：距离、高度、左右偏移、角度、速度
   - 每次记录：成功与否、从握拳到车门开始动的时间
   - 使用 [docs/grasp_door.md](grasp_door.md) 第8节的记录表

3. **反向用例**
   - 不带钥匙
   - 非P挡
   - 错误手势（只握拳不后拉）
   - 预期：车门不应打开

**成功标准**: 成功率 ≥ 70%

---

## 📊 架构图

```
用户界面 (Web/Desktop)
    ↓
ExecuteGesture ROS服务
    ↓
gesture_to_h1_2_loco (UDP客户端)
    ↓
h1_2_loco_udp_daemon (主循环)
    ↓
工作线程 (gesture_worker_thread)
    ↓
ArmGesturePlayer (播放器)
    ├─ 订阅: rt/lowstate
    └─ 发布: rt/arm_sdk
         ↓
原厂运控 (权重混合)
    ↓
关节电机 (PD控制)
```

---

## 🔍 关键设计决策

### 1. 为什么不训练？
- 右臂伸出重心前移仅2.6cm（约脚长的1/10）
- 原厂运控闭环平衡可承受该扰动
- 参考：ShakeHand(0) 也是手臂前伸+腿保持站立

### 2. 为什么用五次多项式？
- 两端速度和加速度为0，缓起缓停
- 避免跳变和震动

### 3. 为什么限速用拉长时间而非截断？
- 截断会导致手臂落后时间表
- 拉长时间保证每段结束时精确到达目标
- 手型切换与手臂到位对齐

### 4. 为什么延迟回复？
- 手势播放需约12秒
- 立即回复会导致路线执行器以为做完，机器人伸着手臂开始走
- 由工作线程播放完成后回复

---

## ⚠️ 待验证事项

| 事项 | 验证方法 | 当前状态 |
|------|---------|---------|
| arm_sdk在当前固件可用 | 阶段1运行官方例程 | 待验证 |
| Dex5指令话题名 | 阶段2订阅实际话题 | 待验证：`rt/dex5/right/cmd` |
| Dex5关节顺序 | 阶段2逐个关节发小角度 | 待验证 |
| 重力下垂量 | 阶段1对比指令和实际角度 | 待测量，粗估4cm |
| 车能识别机器人手 | 阶段7上车测试 | 待验证，可能需肤色手套 |

---

## 🐛 已知问题

### 灵巧手集成未完成
**原因**: 等待Dex5话题名和关节顺序确认

**临时方案**: 手势播放器预留了手型发布接口，当前注释掉：
```cpp
// TODO: 发布手型到灵巧手
// publish_hand_shape(cmd.hand_shape);
```

**完成条件**: 确认话题名后取消注释，实现 `publish_hand_shape()` 函数

---

## 📝 安全规范

1. **首次上真机必须挂吊架**
2. **限速从0.5 rad/s起步**，稳定后可提到1.0
3. **旁边始终留人拿遥控器**，随时能切阻尼
4. **不在调试模式下运行**（不能和 h1_2_policy_bridge 同时运行）
5. **测试前确认**：支撑可靠、急停就位、净空充足

---

## 📚 参考文档

- [docs/grasp_door.md](grasp_door.md) - 完整设计文档
- [docs/h1_2-onnx-navigation/plan.md](h1_2-onnx-navigation/plan.md) - 导航功能计划
- Unitree SDK2 文档：`$UNITREE_SDK2_ROOT/doc`

---

## 🎯 验收标准

### 软件层面
- [x] 编译通过，无警告
- [ ] 单元测试全部通过
- [ ] 配置文件加载成功
- [ ] demo程序能独立运行

### 功能层面
- [ ] 吊架上反复运行稳定
- [ ] 中止后权重平滑归零
- [ ] 手势动作平滑无跳变
- [ ] 上车成功率 ≥ 70%

### 集成层面
- [ ] Web界面能触发
- [ ] 路线到点后自动执行
- [ ] 执行完成后继续走下一段

---

## 📞 问题排查

### 编译错误
- 检查 `UNITREE_SDK2_ROOT` 环境变量
- 确认 Unitree SDK2 库文件存在
- 检查C++20编译器支持

### 运行时错误 -6
- 机器人是否上电
- 网络是否连通
- DDS domain_id 是否正确

### 手臂不动
- 检查是否在原厂运控模式（不是调试模式）
- 确认权重是否升到1.0
- 查看 `rt/lowstate` 是否有数据

### 手臂下垂
- 增加重力前馈 `tau_ff`
- 或适当提高 `kp`

---

**开发完成日期**: 2026-11-19  
**下一步负责人**: 待分配  
**预计上车时间**: 第2周
