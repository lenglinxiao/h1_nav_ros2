# 2026-11-05 板端新接口隔离通信验证

用户选择只验证通信、不让电机动。在 `192.168.123.164` 的正式安装上运行实际 `policy_navigation_gate`，测试 `ROS_DOMAIN_ID=189`、`ROS_LOCALHOST_ONLY=1`，公开/私有 action 和 ready/alive 均映射至 `/h1_comm_probe_1105`。测试进程的ROS域和回环参数从 `/proc/<pid>/environ` 核实；开始前隔离域节点列表为空。后端是独立进程的假 Nav2，ready由测试客户端注入；探针不导入宇树SDK，不创建 `rt/lowcmd` publisher，不启动策略节点或真实Nav2，不调用电机授权或动作服务。

## 实际发现与修复

- 已部署 gate 将异步 `execute` 经普通 lambda 注册，真实rclpy将协程当返回值，导致已接受目标后崩溃。失败证据 `gate_callback_failure.log`。已改成真正 async 回调；新增注册回调层回归，开发机gate测试15/15通过。
- 原总launch只映射 action 基名。在板端域190用独立 fake C++ rclcpp_action server核对，基名映射后5个派生接口仍在旧名（`result.txt`）；分别映射 send_goal/get_result/cancel_goal/feedback/status 后均在私有名（`explicit_result.txt`），跨进程客户端发现 private=true、old public=false（`client_result.txt`），假目标按后端设定拒绝。总launch现为两种action分别映射5个派生接口；开发机导航launch测试9/9通过。
- gate退出时曾因SIGINT后重复shutdown打印异常，已改用安全的try_shutdown；最终实测gate和fake backend退出码均0。

修复后，正式src、持久release src及正式install各自一致：

- `policy_navigation_gate.py` SHA256 `d78fdd48001d06c2e33553f5438a4cdf42fbaf1b3d09a5e6f3a9820170317ae8`
- `h1_2_nav2_navigation.launch.py` SHA256 `a5dd4932453c0eff8128215bd88684c927472b933e644a08f965a59622219b7b`
- 两个测试文件亦已同步正式src和release src。修改前文件备份在板端 `/home/unitree/h1_nav2_ros2_ws/staging_policy_switch_20261105_5IcSXL/comm_fix_backup/`。

## 最终跨进程测试

`driver_final.log`：遗留假目标达到取消终态后 gate 才alive；两类action未ready时均拒绝；ready后两类均转发成功并各收到14条反馈和成功结果；ready=false和ready心跳超时均取消执行中的假目标；最终测试通过，子进程均正常退出0。`gate.log`、`backend.log`为该次原始日志；检查未见探针、fake backend或gate残留。测试脚本 `gate_probe.py` 保留实际使用的唯一前缀重映射。

`driver_first_error.log`至`driver_fifth_error.log`保留测试脚本初始化或隔离映射不完整导致的失败；`driver_base_only.log`记录基名映射不足的独立对照。测试断言和现场部署原始记录均保留，未覆盖旧部署证据。

本次验证的是本机跨进程 ROS action 与 gate 协议。测试的 ready、目标和后端均为模拟，**未验证真实 Nav2 运行、真实机器人电机动作、导航效果或停机物理行为**；此前实机退出超速问题仍未解决。
