# 抓握后拉手势功能 - 部署指南

**部署日期**: 2026-11-19  
**目标机器**: PC4 (192.168.123.164)  
**状态**: ✅ 编译完成，准备部署

---

## ✅ 已完成的工作

1. ✅ 代码开发完成（13个模块）
2. ✅ 编译成功（无错误，仅有警告）
3. ✅ 单元测试通过
4. ✅ 部署包已打包
5. ✅ 部署脚本已创建

---

## 📦 部署包内容

**文件**: `/tmp/grab_pull_deploy_20261119_195021.tar.gz` (60KB)

**包含**:
- `install/h1_2_loco_bridge/` - 编译后的可执行文件和库
  - `h1_2_loco_udp_daemon` - 更新的守护进程（含grab_pull支持）
  - `grab_pull_demo` - 独立测试工具
  - `config/grab_pull.yaml` - 配置文件
- `src/h1_2_nav_tools/` - 更新的GUI文件
  - 添加了"grab_pull"到手势选项

---

## 🚀 部署步骤

### 方法1: 使用自动部署脚本（推荐）

```bash
cd /home/llx/h1_nav2_ros2_ws
./deploy_grab_pull_to_pc4.sh
```

**脚本会自动**:
1. 检查PC4连接
2. 上传部署包
3. 备份现有文件
4. 解压新文件
5. 验证部署结果

**预计耗时**: 1-2分钟

---

### 方法2: 手动部署

如果自动脚本失败，可以手动执行：

```bash
# 1. 上传部署包
scp /tmp/grab_pull_deploy_20261119_195021.tar.gz unitree@192.168.123.164:/tmp/

# 2. SSH到PC4
ssh unitree@192.168.123.164

# 3. 在PC4上执行
cd /home/unitree/h1_nav2_ros2_ws

# 3.1 备份
mkdir -p backup_grab_pull_$(date +%Y%m%d_%H%M%S)
cp -r install/h1_2_loco_bridge backup_grab_pull_$(date +%Y%m%d_%H%M%S)/

# 3.2 解压
tar xzf /tmp/grab_pull_deploy_20261119_195021.tar.gz --strip-components=1

# 3.3 验证
ls -lh install/h1_2_loco_bridge/lib/h1_2_loco_bridge/grab_pull_demo
ls -lh install/h1_2_loco_bridge/share/h1_2_loco_bridge/config/grab_pull.yaml
```

---

## ✅ 部署后验证

### 1. 配置检查（在PC4上执行）

```bash
cd /home/unitree/h1_nav2_ros2_ws
source install/setup.bash

ros2 run h1_2_loco_bridge grab_pull_demo \
  --config install/h1_2_loco_bridge/share/h1_2_loco_bridge/config/grab_pull.yaml \
  --no-play
```

**预期输出**:
```
Loading config from: ...
Config loaded successfully
  Control period: 0.02 s
  Max joint velocity: 0.5 rad/s
  Motion segments: 7
  Estimated total duration: ~12 s
Config check complete (--no-play specified)
```

---

### 2. 检查守护进程版本

```bash
# 检查是否包含grab_pull支持
strings install/h1_2_loco_bridge/lib/h1_2_loco_bridge/h1_2_loco_udp_daemon | grep grab_pull
```

应该看到 "grab_pull" 字符串。

---

### 3. 重启服务（如果h1-2-web正在运行）

```bash
# 检查服务状态
sudo systemctl status h1-2-web.service

# 如果正在运行，重启
sudo systemctl restart h1-2-web.service

# 查看日志
sudo journalctl -u h1-2-web.service -f
```

---

## 🧪 测试步骤

### 阶段1: 配置验证（已完成 ✅）

在部署后验证第1步中完成。

---

### 阶段2: 吊架测试（下一步 ⭐）

**前提条件**:
- ✅ 机器人挂在吊架上
- ✅ 原厂运控运行中
- ✅ 遥控器就位

**测试命令**（在PC4上）:

```bash
cd /home/unitree/h1_nav2_ros2_ws
source install/setup.bash

# 设置网络接口（根据实际情况修改）
export NETWORK_INTERFACE="eth0"  # 或者 "enp3s0" 等

ros2 run h1_2_loco_bridge grab_pull_demo \
  --config install/h1_2_loco_bridge/share/h1_2_loco_bridge/config/grab_pull.yaml \
  --network-interface $NETWORK_INTERFACE
```

**观察重点**:
- [ ] 程序是否能接收到机器人状态
- [ ] 双臂动作是否平滑
- [ ] 机器人是否站稳
- [ ] 权重是否平滑归零
- [ ] 按Ctrl+C能否安全中止

---

### 阶段3: Web界面测试

**前提**: 阶段2通过

```bash
# 1. 确保h1-2-web.service正在运行
sudo systemctl status h1-2-web.service

# 2. 打开浏览器访问
# http://192.168.123.164:8765

# 3. 在"到达后动作"下拉框中应该看到"grab_pull"选项
```

---

## 📁 部署文件位置

### 在PC4上的文件位置:

```
/home/unitree/h1_nav2_ros2_ws/
├── install/h1_2_loco_bridge/
│   ├── lib/h1_2_loco_bridge/
│   │   ├── h1_2_loco_udp_daemon        ← 更新的守护进程
│   │   ├── grab_pull_demo              ← 新增：测试工具
│   │   ├── cmd_vel_to_h1_2_loco
│   │   └── gesture_to_h1_2_loco
│   └── share/h1_2_loco_bridge/
│       ├── config/
│       │   └── grab_pull.yaml          ← 新增：配置文件
│       └── launch/
│           └── h1_2_loco_bridge.launch.py
└── src/h1_2_nav_tools/
    └── h1_2_nav_tools/
        ├── h1_2_nav_web_gui.py         ← 更新：添加grab_pull
        └── h1_2_nav_gui.py             ← 更新：添加grab_pull
```

---

## 🔧 故障排查

### 问题1: 配置文件加载失败

**症状**: "Failed to open config file"

**解决**:
```bash
# 检查文件是否存在
ls -lh install/h1_2_loco_bridge/share/h1_2_loco_bridge/config/grab_pull.yaml

# 检查权限
chmod 644 install/h1_2_loco_bridge/share/h1_2_loco_bridge/config/grab_pull.yaml
```

---

### 问题2: 无法接收机器人状态

**症状**: "Timeout waiting for robot state"

**可能原因**:
1. 网络接口不正确
2. DDS domain不匹配
3. 原厂运控未运行

**解决**:
```bash
# 检查DDS话题
ros2 topic list | grep lowstate

# 检查网络接口
ip addr show

# 指定正确的网络接口和domain
ros2 run h1_2_loco_bridge grab_pull_demo \
  --config xxx.yaml \
  --network-interface enp3s0 \
  --domain-id 0
```

---

### 问题3: 守护进程无法识别grab_pull

**症状**: "error -2 unsupported gesture grab_pull"

**原因**: 使用了旧版守护进程

**解决**:
```bash
# 重新部署或重启服务
sudo systemctl restart h1-2-web.service

# 或者手动启动新版守护进程
pkill h1_2_loco_udp_daemon
install/h1_2_loco_bridge/lib/h1_2_loco_bridge/h1_2_loco_udp_daemon \
  --grab-pull-config install/h1_2_loco_bridge/share/h1_2_loco_bridge/config/grab_pull.yaml
```

---

## 📝 回滚步骤

如果部署后出现问题，可以回滚：

```bash
cd /home/unitree/h1_nav2_ros2_ws

# 1. 找到备份目录
ls -lhd backup_grab_pull_*

# 2. 恢复备份（假设备份目录是backup_grab_pull_20261119_195100）
BACKUP=backup_grab_pull_20261119_195100
rm -rf install/h1_2_loco_bridge
cp -r $BACKUP/h1_2_loco_bridge install/

# 3. 重启服务
sudo systemctl restart h1-2-web.service
```

---

## ⚠️ 安全注意事项

1. **首次测试必须挂吊架**
2. **旁边始终有人拿遥控器**，随时能切阻尼
3. **不要在调试模式下运行**（不能和h1_2_policy_bridge同时运行）
4. **限速从0.5 rad/s起步**，稳定后再提高

---

## 📞 需要帮助？

如遇问题，检查以下日志：

```bash
# 查看服务日志
sudo journalctl -u h1-2-web.service -n 100

# 查看ROS节点日志
ros2 topic echo /rosout

# 查看进程
ps aux | grep h1_2_loco
```

---

**部署准备完成！现在执行部署脚本：**

```bash
cd /home/llx/h1_nav2_ros2_ws
./deploy_grab_pull_to_pc4.sh
```
