# Guarded release 历史轨迹离线回放（2026-11-05）

从工作区根目录运行：

```bash
cmake --build build/h1_2_policy_bridge --target h1_policy_core -j2
g++ -std=c++17 -I src/h1_2_policy_bridge/include tools/h1_2_policy_hardware/evidence/guarded_release_replay_20261105/replay.cpp -L build/h1_2_policy_bridge -lh1_policy_core -Wl,-rpath,"$PWD/build/h1_2_policy_bridge" -o /tmp/h1_guarded_release_replay
/tmp/h1_guarded_release_replay tools/h1_2_policy_hardware/evidence/onnx_once_20261105/acceptance-20261105T150959/trace.csv 3498.224512651
/tmp/h1_guarded_release_replay tools/h1_2_policy_hardware/evidence/arm_prepare_transition_20261104/trace.csv 3573.869384
```

回放直接链接本地构建的真实 C++ `GuardedRelease`。取释放前最后一条同时有状态和命令的采样，使用其中的 `cmd_q/kp/kd/tau` 和 `q/dq` 初始化；之后按 CSV 每条采样的 monotonic 相对时间和 27 维 `q/dq` 输入。CSV 关节顺序为 SDK 顺序，调用 `from_sdk` 转为 policy 顺序。初始命令是监测到的最后快照，不能单凭该快照证明节点最后一次成功写入的实际命令。以下关节编号使用从 0 开始的 SDK 编号，速度单位 rad/s。

| 轨迹 | 释放前快照 monotonic | 首次 blocked：monotonic / 相对秒 / 关节 / dq | 历史首次速度绝对值 ≥ 2：monotonic / 相对秒 / 关节 / dq |
|---|---:|---|---|
| 2026-11-05 | 3498.214570930 | 3500.420315150 / 2.195802499 / 16 / +0.555232883 | 3501.118331040 / 2.893818389 / 23 / +2.071487665 |
| 2026-11-04 | 3573.860266310 | 3575.865044480 / 1.995660480 / 25 / +0.655009806 | 3576.756100490 / 2.886716490 / 23 / +2.066033602 |

两次 blocked 是以历史速度作为输入得到的离线状态机结果。历史轨迹由旧控制过程产生，回放不改变真实机器人的运动，也没有模拟新的闭环响应；不能据此预测新控制的轨迹或证明其能防止超速。采样间发生的事件也无法从 CSV 精确还原。0.1 和 0.5 rad/s 为工程阈值，不构成安全证明。
