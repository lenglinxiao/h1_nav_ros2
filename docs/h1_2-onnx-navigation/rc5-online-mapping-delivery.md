# RC5 在线建图后端交付核验

时间：2026-11-11（Asia/Shanghai）  
目标：PC4 `unitree-h1-2-pc4`，ROS 2 Humble。  
Release：`/home/unitree/h1_nav2_ros2_ws/releases/onnx-navigation-20261111-rc5`

## 交付范围

RC5 包含启动时选择 `motion_backend=policy` 的 camera-init 在线建图入口。FAST-LIO、`camera_init` 坐标系、实时动态占据栅格、Nav2 参数/目标输入保持现有路径；Policy 模式将 Nav2 会话和速度输出交给 Policy session，不启动 Unitree locomotion bridge 或原 smoother。默认仍为 Unitree。只在独立 release 构建；没有覆盖主工作区、地图或现有 GUI 服务。

## 验证证据

- 本机有线接口 `enp43s0` 为 `192.168.123.99/24`，路由直达 PC4；SSH 返回主机 `unitree-h1-2-pc4`，PC4 时间 `2026-11-11T10:15:35+08:00`。
- 源码包 SHA256：`9ce45f010a0c5b3bbf29474b3d3e68a5196b0c431f8e2cdbc62db38a444ff9e1`。
- 首次构建尝试缺少板端 ONNX Runtime 路径而失败；按 RC4 的 CMake cache 指向板端依赖后重跑成功：五个包 `h1_2_policy_interfaces`、`h1_2_nav_msgs`、`h1_2_policy_bridge`、`h1_2_nav_tools`、`h1_2_slam_bringup` 均完成。
- Release overlay 下 `h1_2_nav2_camera_init_navigation.launch.py --show-args` 与 `policy_nav_session.launch.py --show-args` 通过；前者列出 `motion_backend`，默认 `unitree`。离线验收 `test_acceptance_offline.py`：17/17 通过。
- camera-init launch SHA256：`59595c5f68091d3033199d3556c75acebf0b7e6d3ef720b81273c4d495293104`；session manager executable SHA256：`e857052e7f61dbabd104579ff8551c9eb959a14477e0e2f8b70bdd0c44b4e1ab`。
- 现场只读 monitor 6 秒：2991 LowState、2990 LowCmd、CRC 错误 0、状态 age `0.000235824` 秒、mode 6 / PR 0。唯一活跃 LowCmd writer GUID `01100565e7bb65be7fc3ec2c00000203`（2990 samples，last_seen_age `0.000042731` 秒），`policy_commands=0`、`unknown_commands=2990`。当前 host/PID 和控制权未知。telemetry SHA256 `d1fd32f8ac584822ed9f74676b022a562c05a3809a4b4e155060fee4491d6aec`；trace SHA256 `8432cedfbb18305ce673d2993945d6848f4455131ff8f9e12158576923613368`。monitor 在 `/tmp/h1-rc5-readonly/`，仅订阅状态和命令，无发布。
- 2026-11-11 行走短测准入复核：PC4 Owner systemd 为 `inactive`，ROS graph 无 Policy/coordinator/Nav2 action；当前旧 GUI 与用户启动的 Livox 节点保留。新鲜6秒monitor 2987 LowState/2987 LowCmd、CRC0、age `0.000539063`秒、mode6/PR0；唯一 writer 仍为 `01100565e7bb65be7fc3ec2c00000203`（2987 samples，age `0.000021113`秒），`unknown_commands=2987`、`policy_commands=0`。telemetry SHA256 `c0ec70e238b6c958c7f880215fa373678c1f0e96646d78c43a7eba0057455600`，trace SHA256 `93538bf7c7e29d35d54398c7b31cc9066c868f2b026243d1a38afe7492e3af05`，DDS dump `/tmp/h1-shorttest-preflight/dds.json`。
- `ros2 topic info -v /lowcmd` 将 endpoint 描述为 `_CREATED_BY_BARE_DDS_APP_`，GID 对应上述 writer。只读 `ddsls -a -j -r 3` 将 publication GUID 映射到 participant `01100565-e7bb-65be-7fc3-ec2c000001c1`；同一 participant 还发布 `rt/SymState`。这提示它是 bare DDS participant，但不提供 host/PID，无法确认部署位置或实际控制权。没有停止/切换它。
- 用户提供PC4 sudo授权后，在 `eth0` 仅捕获UDP/7400发现流量，`snaplen=64`、最多60包、35秒，无数据负载；60包捕获、69包匹配、kernel drop为0。RTPS头部 GUID前缀 `01100565e7bb65be7fc3ec2c` 与LowCmd participant完全匹配，源IP为 `192.168.123.161`。PC4 `/home/unitree/.ssh/known_hosts` 中已登记的`.161` Ed25519/ECDSA key与现场key匹配，SSH主机身份验证通过；但 `unitree@192.168.123.161` 返回 `Permission denied (publickey,password)`，无法读取进程表。由此确认participant发现流量来自`.161`，具体PID/控制程序与实际控制权仍待该机管理员确认。PC4 monitor和`ros2 topic info`仍显示LowCmd writer活跃。当前Owner inactive；缺少进程/控制权归属及已验收速度包络，未启动控制栈或发送目标。
- 使用 RC5 policy executable、当前模型和 URDF、RC5 monitor 与 coordinator 在输出和导航全关闭配置下执行 Owner `--check`，通过；该检查不启动子进程。实际模型 SHA256 `3090836d0ce454a35161982b60dde20e0fd87627150e06126aabd1e6bbfabae7`、URDF SHA256 `504b0cc9ea5311044876f4be4e3148fbc285aa1439e33b19d93c357a7468066c`、policy executable SHA256 `923a2f0d6bc621dfc25c2055f8b9d4f70cab535700b698e9b0cb1aaf38330c34`、monitor SHA256 `deb334fe27531c75eecf3dbb91363967e34610af6ea5c96b501558c126c7ecc5`。不代表Owner graph准入或现场速度包络有效。
- 2026-11-11 限时启动在线 camera-init，参数 `motion_backend=policy start_livox_driver:=true`，点云保存路径设为 `/tmp/h1-rc5-readonly/online-test.pcd`。FAST-LIO、实时占据栅格、Livox 驱动、在线入口节点均能启动；Unitree bridge/smoother 未启动。Livox 日志报 `bind failed`，原因是安装配置的 host IP 为 `192.168.123.104`，PC4 当前 `eth0` 是 `192.168.123.164`。仅生成临时 JSON（host IP 改为当前 eth0），SHA256 `3bc84051c6549a0e2324f779c5cbe10ac268ba9d97bd153b3c4c5cf377c6ba4b`；driver 初始化成功，但 30 秒内无 `/livox/lidar` publisher，`192.168.123.120` 无邻居，未形成在线点云或地图。驱动对 SIGINT 无响应，限时停止升级至 SIGKILL；复核所有本次启动节点与驱动均已退出，原有 ROS 图恢复。临时文件在 `/tmp/h1-rc5-readonly/`，没有修改安装配置或地图。
- 用户开启雷达后发现其已有驱动由 `/home/unitree/ros2_ws` 运行，节点 `/livox_lidar_publisher`；没有启动第二个驱动。只读频率约为 `/livox/lidar` 8 Hz、`/livox/imu` 200 Hz。
- 随后限时30秒启动RC5 camera-init，参数 `motion_backend=policy start_livox_driver:=false`，点云保存路径 `/tmp/h1-rc5-readonly/online-test.pcd`。FAST-LIO收到至少30帧点云，示例一帧1622点、704点通过z滤波；动态栅格发布至少45帧 `/map`，每帧1600×1600、分辨率0.05m。首帧2394 occupied/30166 free cells，后续数量随输入更新。Unitree bridge/smoother未启动。`/slam_odom` 与 `camera_init -> body` TF 未在本窗口独立确认。
- Policy session manager因Owner/coordinator未启动，在15秒后报readiness timeout；没有Owner、Policy/Nav2 session启动或运动命令。限时结束后本次FAST-LIO、栅格和入口节点均退出；用户原有Livox driver仍运行。现有8765 GUI仍由主工作区旧版运行。
- 之前RC5附带Livox launch的试启报告host IP `.104` bind failure；一次临时改为`.164`的独立试启仍未发现设备且无数据。这两次额外驱动进程均已结束。所有临时配置/日志在PC4 `/tmp/h1-rc5-readonly/`，未改变用户原有`/home/unitree/ros2_ws`驱动、安装配置或地图。

## 当前结论

RC5在线建图链路已完成短时真机数据验证：Livox点云进入FAST-LIO，动态栅格连续发布`/map`。这证明在线建图入口的点云到栅格路径工作；`/slam_odom`、TF和长时稳定性仍未验证。Owner静态配置检查通过，但未启动。LowCmd participant现已通过RTPS发现头部映射到源IP `192.168.123.161`，known_hosts key校验匹配；但SSH认证被拒，具体PID/控制程序和实际控制权仍未确认，且缺少已验收速度包络，因此短测仍未准入、当前不发送Nav2 goal。下一步需由`.161`管理员核实对应进程和控制权，再完成现场速度包络与Owner graph验收。
