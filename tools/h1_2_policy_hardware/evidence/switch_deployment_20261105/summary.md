# 2026-11-05 板端 ONNX 导航切换部署

**后续更新：**当日隔离通信验证中修复了 gate 异步回调和 launch action 派生端点重映射，当前板端 gate 与 launch 哈希已变化；其最新哈希、备份和验证见 [通信验证记录](../interface_comms_20261105/summary.md)。下文记录的是最初部署状态和当时证据。

目标：`unitree@192.168.123.164`，hostname `unitree-h1-2-pc4`，x86_64；板端 eth0、Unitree DDS domain 0。用户明确要求部署；本轮没有授权LowCmd，没有执行准备动作或导航。

- 上传补丁只含 `h1_2_policy_bridge`、`h1_2_nav_tools`、`h1_2_slam_bringup` 的12个必要文件；包 SHA256 `3d1deb22c209174ada94f691a9de6c73a763e212cb5790c9bb004628fe645a34`。
- 持久release：`/home/unitree/h1_nav2_ros2_ws/staging_policy_switch_20261105_5IcSXL/`。正式 `install/` 的上述三个包是指向该目录 `install/` 对应包的符号链接；不能删除release目录。正式 `src/` 三包与release源码同步；部署说明随后同步更新，不改变程序。
- 完整旧源码和安装备份：release下的 `promoted_old/`（原正式目录）与 `backup/`（独立副本）。回滚脚本：`/home/unitree/h1_nav2_ros2_ws/staging_policy_switch_20261105_5IcSXL/rollback.sh`；**回滚前先停止相关节点**。本轮未执行回滚。
- 正式节点库 SHA256 `79992ca558bf9e5e4249ac00ff47f3ef70cb53c016ef97acd8b68f1411e83074`；核心库 `a658903526d0900747993ed4709aada53e5317267f1bd697946482fa8c2a5c72`。模型 SHA256 `3090836d0ce454a35161982b60dde20e0fd87627150e06126aabd1e6bbfabae7`。
- 隔离构建3包通过；`node_integration`（假IO）和 `launch_parameters` 2/2通过；gate单元/行为测试14/14通过；导航launch 9/9通过。导航launch首跑因隔离目录没有同级机器人描述文件而失败；链接到板端已有描述包后重跑通过，首次记录保留在 `python_tests.txt`、重跑见 `navigation_tests_retry.txt`。
- 隔离版本只读status/shadow：`state=valid`、`authorized=false`、`publisher_present=false`、`write_count=0`，5个输出/导航开关均false；就绪及gate话题存在，3条shadow日志，节点退出0。
- 正式安装用板端常规 `install/setup.bash` 解析，三个包前缀、Python模块、动态库与源文件校验正确；正式节点只读status同样 `state=valid`、`authorized=false`、`publisher_present=false`、`write_count=0`、导航和连续运行开关false，节点退出0，检查未见相关残留进程。

未执行sim2sim、实机电机动作、导航或控制权切换。实机策略闭环和之前的退出超速问题仍未验证，不可据此声称行走已成功或安全。
