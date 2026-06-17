# 测试目录说明

这里按运行风险划分测试类型，避免离线测试误触发 ROS、Nav2 或实机链路。

- static/：离线静态测试，只读取仓库文件；不能启动 shell 脚本、ROS、Nav2、传感器或机器人硬件。
- ros_integration/：后续 ROS 图集成测试，只允许使用本地 mock 节点。
- manual/：后续实机侧人工验收清单。

Todo 1 的测试放在 static/，只用于锁定在线/离线工作流约定。
