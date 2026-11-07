# 训练统计保护：本地验证

- 用户确认：速度及位置表 hip_pitch/hip_roll 均互换后映射当前 SDK；原始文字保存在 docs/train.md。
- 纯保护：构建 h1_policy_trained_guard_test，ctest -R '^trained_guard$' 通过。Release 下 -UNDEBUG 保留测试断言。
- 集成：ctest --test-dir /home/llx/h1_nav2_ros2_ws/build/h1_2_policy_bridge -R '^(core|node_integration)$' --output-on-failure，2/2 通过，6.76秒。
- Python：source /opt/ros/humble/setup.bash；在 tools/h1_2_policy_hardware 执行 PYTHONWARNINGS=ignore python3 -m unittest -q test_train_guard_profile test_acceptance_workflow，36项通过，13.178秒。
- 校验内容包括 SDK 名称/索引、URDF交集、3个新样本去抖、warm-up、姿态持续时间、真实Passive命令及runner人工交接；不等于实机执行或厂家认证。
- 首次集成命令误用了相对build路径，未运行该项测试；后使用上述绝对路径得到2/2结果。
- 两次 SSH 连接192.168.123.164超时；未部署新版本、未创建实机LowCmd发布者、未执行新配置。
- 对应源码hash见 source_sha256.json，生成的有效SDK保护配置见 profile.json；原始训练统计未重新计算。
