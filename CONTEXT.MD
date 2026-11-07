2026-11-07 导航开发规划：常驻策略生命周期、唯一速度/心跳仲裁、域与训练配置贯通、GUI 停止隔离、已验限速和分层验收拆为[六项任务](docs/h1_2-onnx-navigation/development-plan.md)；[设计](docs/h1_2-onnx-navigation/design.md)待审阅。当前私有 ROS43 localhost 实例不能假定热接，现场及授权见[任务](tasks/20261106-trained-policy-guard.md)，未执行新操作。

2026-11-07 无外部悬吊/防摔支撑：用户确认站稳、净空、可急停，原PTY一次WALK后返回ArmHold idle；前后status均ArmHold、ready、零命令、guard未触发，同PID/GUID。窗口中runner检查模式；repeat内存事件未落盘；用户确认已迈步、结束后独立站稳，未切模式、重启、撤权或再次触发，本次授权已用；不推及长时或导航安全。[任务](tasks/20261106-trained-policy-guard.md)。

2026-11-07 重复测试：用户授权后向现有PTY27864输入一次WALK；前后status均ArmHold、ready且零命令、guard未触发，PID和writer不变，约3.7秒后再次idle。runner在脉冲中检查ArmHold；repeat事件仍在内存，非零观测未独立读取，用户确认再次迈步、结束后站稳；第一轮同进程 repeat 已实机确认，未调用模式切换服务、重启或撤权，本次授权已使用。[任务](tasks/20261106-trained-policy-guard.md)。

2026-11-07 17:10：训练接管角差调整已部署并通过未授权状态检查；一次0.12 m/s、3秒试验程序报告非零及随后零命令，用户确认持续迈步、归零后站稳，重复 WALK 尚未实机触发。PTY27864保持ArmHold idle，不自动重试或撤权。[任务](tasks/20261106-trained-policy-guard.md)。

2026-11-07 部署前历史：训练配置 ArmHoldStand→ArmHold 的默认角差及膝目标差改为诊断，legacy 膝门限与运行期保护保留；Fake IO 和 core 定向测试通过。未部署、未进行实机动作；现场最近报告 PTY95666 保持 ArmHoldStand，状态易变。[计划](docs/h1_2-armhold-repeat-walk/plan.md) [任务](tasks/20261106-trained-policy-guard.md)。

2026-11-07 历史现场负载补充：用户确认支撑主要用于防摔，重量仍由双脚承担；不能据此断言带载PD静差或控制权为偏差根因。当时PTY95666保持ArmHoldStand，SDK3/4/9偏差约0.266–0.276 rad，WALK入口不可用，本次未运动；不绕过门限、撤权或重启。[任务](tasks/20261106-trained-policy-guard.md)。

2026-11-07 历史：用户确认可靠支撑允许更新重启，旧26347撤权确认且退出0；repeat入口已部署，本次初始试验在SDK4 prearm误差0.273897超过0.1时拒绝，未发布非零速度。PTY95666保持ArmHoldStand，用户确认支撑可靠且机器人稳住；2秒trace中SDK3/4/9命令目标为训练默认、实际偏差持续约0.266–0.276rad，具体原因未知。等待安全交接指示，不自动撤权/release；repeat idle未启用。[任务](tasks/20261106-trained-policy-guard.md)。

2026-11-07 历史：用户明确没有吊带，要求保持现有 ArmHold 再测试行走；PTY26347 保持，本次新行走未执行。禁止切模式、撤权、release、重启或新 publisher 抢发。本地重复行走入口定向6项通过但未部署、未实机验证，当前会话不能使用；详见[功能计划](docs/h1_2-armhold-repeat-walk/plan.md)和[任务](tasks/20261106-trained-policy-guard.md)。

2026-11-07 历史：单次原地左转 `(0,0,+0.1)` 3秒窗口runner报告非零及归零，用户确认已向左转动、归零后站稳；仅此单次参数通过，右转、更大角度、长时和导航转向待验。PTY26347持续零ArmHold，不自动撤权。前次PTY72345已撤权确认、节点和监测器退出0。[转向计划](docs/h1_2-onnx-turn-test/plan.md)。

2026-11-07 最新：控制链路恢复，当日现场条件确认后一次0.12 m/s、3秒窗口runner报告完成，观察到非零及随后零policy_cmd；当前PTY72345保持ArmHold，用户现场确认持续迈步、归零后站稳；本次单次授权已使用，不自动撤权或release。[任务](tasks/20261106-trained-policy-guard.md)。

2026-11-07 当前：用户要求更长时间行走，拟单次0.12 m/s、3秒；今日未执行。enp43s0有192.168.123.99/24且路由正确，但NO-CARRIER，提权只读SSH到板端仍超时；此前3秒结果及当前现场状态未知。恢复物理连接后先只读核对记录、进程与版本，再依据当天现场确认推进。[当前任务](tasks/20261106-trained-policy-guard.md)。

2026-11-07 交接校正：执行记录显示旧38895已撤权、3秒版本已部署并启动试验；该次结果与当前现场状态未知。下方“尚未部署/38895保持”是未更新的历史表述，待核对保存证据。分阶段验收见[测试计划](docs/h1_2-onnx-navigation/test-plan.md)。

2026-11-07：已将 ONNX 动作基线通过后的导航联调顺序整理为[功能开发计划](docs/h1_2-onnx-navigation/plan.md)；现有接口已完成隔离通信验证，实机导航仍待验收。现场状态及授权以[当前任务](tasks/20261106-trained-policy-guard.md)和执行时复核为准。

2026-11-06 最新方向：用户要求更久测试，拟单次0.12 m/s、3秒后归零；本地时长上限和定向测试已完成，尚未部署。此前1.5秒试验租约中止，不能称完整；PTY38895继续保持，等待现场支撑及安全交接确认。[任务](tasks/20261106-trained-policy-guard.md)。

2026-11-06 最新：一次1.5秒速度试验期间 ready lease expired，已观测非零和随后零命令，不能证明完整1.5秒；当前PTY session 38895保持ArmHold，用户确认迈步后停稳，不自动撤权。原因及证据见 [任务](tasks/20261106-trained-policy-guard.md)。

2026-11-06 最新：用户将训练配置预接管门限由0.07改为严格小于0.1 rad；定向测试2/2通过并已部署。一次0.12 m/s脉冲窗口日志确认非零命令及随后零指令 ArmHold，用户现场确认已迈步且归零后站稳；一次脉冲授权已使用，当时 PTY session 61909 保持，之后按用户确认已撤权结束。详见 [当前任务](tasks/20261106-trained-policy-guard.md)。

2026-11-06 此前试验（已撤权结束）：第二次训练配置试验在 SDK10 右踝 pitch 预接管偏差 0.0731309 rad 超 0.07 门限时拒绝，未发布非零速度；当时曾保持 ArmHoldStand，随后按用户确认撤权，PTY session 85198 已结束。此前 SDK24 准备目标冲突已修复部署，本地 core/node 定向 2/2 通过；SDK10 持续误差原因未确认。实时状态及证据详见 [任务](tasks/20261106-trained-policy-guard.md)。

2026-11-06 会话交接入口：[SESSION.md](SESSION.md) 索引当前任务，[tasks/README.md](tasks/README.md) 提供任务模板；任务文件是状态事实源，[使用说明](docs/session-handoff.md) 记录边界。本文件保留可复用的长期结论和历史证据；现场状态以实时复核为准。

2026-11-06 12:04 现场复测中用户报告看到腿部迈步、试验后静止稳定；日志确认非零输入，左膝超 3 rad/s 后撤权，发令后持续零速站立全流程被中止，0.5 秒窗口是否完成无法确认，持续行走尚未通过验证。[证据](tools/h1_2_policy_hardware/evidence/manual_walk_trial_20261106T120400/summary.md)。

2026-11-06 短速度试验已观察策略非零输入，但右膝 SDK9 超过 3 rad/s 后撤权，未证实迈步或当前物理静止；详见[试验聚合](tools/h1_2_policy_hardware/evidence/manual_walk_trial_20261106T115543/summary.md)及[故障记录](docs/h1_2-policy-troubleshooting.md)。

2026-11-05 21:36 分组门限试验：已部署准备期全身 2、pulse 后腿 3/上身 2 rad/s，保护未关闭。板端编译、15 项假数据测试及 3 秒只读监测通过；现场 CONFIRM 后膝位置复核失败，未调用 arm_hold、未发脉冲。ArmHoldStand 端点左/右膝 0.963871/0.913728 rad，撤权前邻近 trace 0.970870/0.918026，左膝越 0.97 上界；精确触发帧未保存，根因未定。累计峰 1.374758 rad/s、撤权确认、node/monitor 退出 0、无目标进程；现场报告静止稳定、支撑仍在。未实机进入行走，不能称分组行走验证通过。证据 `tools/h1_2_policy_hardware/evidence/grouped_walk_trial_20261105T213652/summary.md`。

2026-11-05 21:20 手动行走尝试：ArmHoldStand 已观察 3.109 秒，ArmHold 已 ready，非零 policy_cmd=0.12,0,0 后 arm_hold 阶段 SDK9 实测 dq 首超 2 为 2.16938、最高 2.22022 rad/s，原有速度保护中止并确认撤权；node/monitor 退出 0，电机撤权后行为未验证。与新增工程 precheck 不同，根因未确定。精简证据见 `tools/h1_2_policy_hardware/evidence/manual_walk_trial_20261105T212015/summary.md`。

2026-11-05 20:52 零策略步数准备后 Hold，现场再次确认支撑并允许 RELEASE，卸载 2.621146 秒因右肘 SDK23 速度 0.505055 rad/s 阻断，q=-0.266930，最后命令 q=-0.570109、kp=15.9541、kd=4；全程观测峰 1.362759 rad/s。现场确认静止支撑后授权 REVOKE，末状态 authorized=false、publisher=false、write_count=2413，节点/监测器退出 0，后续无目标进程；needs_operator_check，原生阻尼和撤权后电机行为未验证。现场确认支撑仍失败，不能断言支撑即可解决或归咎模型；不要无改动重试。证据 `tools/h1_2_policy_hardware/evidence/release_recheck_20261105T205236/`，详见 `docs/h1_2-policy-troubleshooting.md`。

2026-11-05 用户授权一次准备及短时零速 ONNX 接管；随后只读监测器生存 37.055 秒并退出 0，采集 18527 条状态、0 条命令。实际进入 FixStand 和 ArmHoldStand，但现场姿态确认在 60 秒超时后到达，程序已进入 Hold，记录中 `arm_hold` 服务调用为 0，未发生 ONNX 实际接管。用户明确确认机器人与双臂已实际外部支撑并授权释放；3 秒受控释放中 SDK23 右肘观测到速度超 2 rad/s，首次累计 2.07149、撤权后观察窗口累计峰 2.21439 rad/s；未到已验证最终阻尼。撤权已确认，之后两次 status 均 `authorized=false`、`publisher_present=false`、`write_count=11283` 不变；节点和监测器退出 0，runner 退出 1，最终 `needs_operator_check`。用户随后明确报告已静止、现场稳定且支撑仍在；暂停动作，不自动重试。证据与可复现离线分析见 `tools/h1_2_policy_hardware/evidence/onnx_once_20261105/summary.md`。此次位置/增益变化与 11-04 右肘退出超速相似，不能据时间相关断言根因。释放期间 status 查询很频繁，采样中 LowCmd 约 50 Hz，不能推断其因果。

2026-11-05 用户授权只修复板端监测器生命周期并做只读>35秒存活验证。已备份板端旧源码、runner、二进制至 `/home/unitree/h1_nav2_ros2_ws/backup_monitor_lifecycle_20261105/`；旧源码/runner/二进制 SHA256 分别为 `e7e7c93b526824defa6c0cbae188bb0fb5358cc76e7b82017f5cddc2929f2dbe`、`63ec60ec5314320d0cee2416d7a0b43c62e6fd05eef882fb7cc35940289b9e1e`、`4914244f8a6370029aa174a6e73ef61972e1d5029d0fc67d8f271a3cc151168e`。已将本地 schema3/`lifetime=until_stopped` prepare_monitor.cpp、原版 runner 与 process_guard.py 同步并以板端 SDK 编译；新源码 SHA `9be1d6421f086dd713a22405d63453c582ec1515288a7794e882cd90ecf53809`，runner SHA `7a9539859ee384838b77a02a18e7bc4f82e4d63410a3b4bdd33a9d7f3a25a808`，guard SHA `a045dcfcb0429a5d25378fbd26f5aba9de9195c4f98ff152149e0f0ed7e4bedb`，新二进制 SHA `e0d1ab78010955c70c84a4e6b0a4109dde772318c0484139725f3305df46001d`。只读启动后约3秒收到1501条 `rt/lowcmd`（同窗1501条 LowState），按要求立即中止>35秒验证；约10.55秒快照5276条，bad_crc=0、tau_max=0、native_damping=false、peak_dq=0.06685 rad/s。给本次监测器发SIGINT后终检PID不存在、无匹配残留；跨SSH中止未取得wait退出码，不能声称正常退出已验证。没有定位或终止未知LowCmd publisher；该发现不证明来源、所有权或物理执行，未运行runner、未发控制命令。板端原始和本地副本证据在 `tools/h1_2_policy_hardware/evidence/monitor_lifecycle_20261105/`。因发现现存LowCmd流，首次生命周期>35秒验证中止；随后在一个纯订阅窗口内完成 37.055 秒生存验证，期间 0 条 LowCmd、监测器退出 0；其后用户确认进入调试状态且原运控已退出。此前命令流来源仍未确定，0条窗口不证明长久独占。

2026-11-05 用户要求只验证新接口通信、不让电机动。板端域189/localhost真实gate与独立假Nav2、客户端跨进程验证发现并修复两个实际问题：gate的普通lambda未await异步回调导致接收目标后崩溃，导航总launch只映射action基名而未映射五个派生接口。域190 fake C++ server确认基名映射无效、显式五端点有效。正式src/release src/install已同步修复并备份；gate 15/15、导航launch 9/9，本机跨进程隔离fake ready/fake Nav2验证旧目标清理、双action拒绝/转发/反馈/结果、ready失效取消通过，测试进程退出0、无残留。探针不启SDK或LowCmd，不能证明真实导航、策略闭环或物理退出安全。gate现SHA256 d78fdd48001d06c2e33553f5438a4cdf42fbaf1b3d09a5e6f3a9820170317ae8，launch现SHA256 a5dd4932453c0eff8128215bd88684c927472b933e644a08f965a59622219b7b；证据 `tools/h1_2_policy_hardware/evidence/interface_comms_20261105/`。

2026-11-05 用户明确要求部署后，ONNX导航切换三个包已部署板端 192.168.123.164；正式 src 同步新版，正式 install 三包链接到持久目录 `/home/unitree/h1_nav2_ros2_ws/staging_policy_switch_20261105_5IcSXL/install/`，不能删除该目录。旧 src/install 保留于同目录 `promoted_old/` 及 `backup/`，回滚脚本 `rollback.sh`。补丁包 SHA256 `3d1deb22c209174ada94f691a9de6c73a763e212cb5790c9bb004628fe645a34`；正式策略节点库 SHA256 `79992ca558bf9e5e4249ac00ff47f3ef70cb53c016ef97acd8b68f1411e83074`。隔离构建3包通过，假IO/策略launch 2/2、gate 14/14、导航launch 9/9（首次因隔离目录缺机器人描述路径失败，链接既有描述包后通过）。隔离新版节点只读状态+shadow、正式安装节点只读status均通过：state=valid、authorized=false、publisher_present=false、write_count=0，未运行动作；过程证据见 `tools/h1_2_policy_hardware/evidence/switch_deployment_20261105/`。DDS运动控制、导航链及此前退出超速仍未验证，不能称已可安全行走。

2026-11-04 ONNX 导航切换本地实现：新增硬件导航显式开关与连续模式，默认关闭，保留诊断预算、输出授权、状态保护和互斥；ArmHold 过渡完成且状态有效才输出 policy_ready，ready 之前缓存速度不用，命令超时归零。代理心跳失效时速度归零；代理启动及 action 后台服务重连先请求取消旧目标、等待已知目标终态，恢复后要求新速度。参考 /home/llx/unitree_rl_mjlab/deploy 的 q/kp/kd、dq=tau=0，硬件导航采用 position_pd，当前50 Hz输出及硬件过渡/目标投影并非 deploy 1 kHz FSM原样。用户要求跳过sim2sim，本轮仅构建和假IO、离线接口测试；既有准备退出超速、实际电机响应与完整策略控制尚未真机验证，不能宣称已经可安全导航。

2026-11-04 已将减少无效验证、在授权与现场条件满足时优先执行受控真机操作的用户偏好写入根目录 AGENTS.md。

2026-11-04 新版 Hold 第三次准备动作：FixStand 与 ArmHoldStand 后进入 Hold，未输入 RELEASE、未进入 ONNX；等待期间监测器约 30 秒自行退出，runner 因观测丢失急撤权，结果 needs_operator_check，撤权后遥测不完整，不可称安全通过。撤权后两次 status 均 publisher=false、authorized=false、write_count=1432，节点退出 0、监测器退出 0，进程检查无残留；监测有效窗口累计速度峰 1.513606 rad/s，不覆盖撤权后。板端及本地 prepare_monitor.cpp 第 57 行硬编码 30 秒，trace 末时刻 29.9916 秒，与正常退出相符；Hold 可无限等待，与该上限冲突。需修复并验证监测生命周期，禁止未经新授权的实机动作；现场状态等待用户确认。原始记录 `tools/h1_2_policy_hardware/evidence/hold_action_20261104/`。

2026-11-04 Hold 修复版部署板端完成：position_pd 正常 passive 或预算耗尽保持最后成功命令于 Hold，不自动释放；只有现场明确确认机器人及双臂均已外部支撑并输入 RELEASE，才请求受控退出。部署包 SHA256 bfc228e9bf627572a799091b8f1603642a54b64273120683ead3e241a5d20e4c；隔离构建 CTest 5/5、工具离线 24/24，通过严格输出禁用、零预算 status/shadow 只读检查，未运行任何动作服务。证据见 tools/h1_2_policy_hardware/evidence/hold_deployment_20261104/，备份 /home/unitree/h1_nav2_ros2_ws/backup_policy_review_eHdui98Y。实机抬臂尚未跑通，本次未再动作，先前退出 2.391 rad/s 的问题尚未证实解决；用户未观察到右臂实际运动。

2026-11-04 完整准备实机复测失败：用户授权一次 FixStand→ArmHoldStand、预算 0，未进入 ONNX。受控退出期间监测器首次累计超 2 rad/s（2.120575），最终观察累计峰 2.391101；退出未达到已验证最终阻尼。撤权后两次 publisher=false、write_count=468 不变，node 退出 0、monitor -2，无相关进程残留。用户随后确认机器人已静止、现场稳定；停止动作，不重试。右肘 SDK23 在增益 kp 从 120 降至约 5.6 期间 q 从 -0.557 变化至 0.081 rad，首次超限时仍处于 Stopping；时间相关不证明根因，采样与命令异步。三秒渐变尚未解决或证明真机退出安全。离线复现脚本及数据见 `tools/h1_2_policy_hardware/evidence/arm_prepare_transition_20261104/`。

2026-11-04 用户明确确认机器人 192.168.123.164 并授权传输部署后，完成 position_pd 修复版板端隔离构建和安装：CTest 5/5、验收工具离线 21/21。首次只读 status 在启动约 24 ms 时 state=invalid，严格检查失败并恢复旧版；增加保持 output 禁用的 5 秒有界状态等待后重试通过：status 前后 state=valid、authorized=false、publisher_present=false、write_count=0，shadow false→true，节点正常退出 0，未调用授权/动作/撤权服务。两次原始证据见 tools/h1_2_policy_hardware/evidence/deployment_20261104/；最终备份 /home/unitree/h1_nav2_ros2_ws/backup_policy_review_rxSyoSEO，旧 install 保留为 install/h1_2_policy_bridge.old.100761。最终核心库/节点库分别 SHA256 b4ade40de90f8faec123441bb3d12f3a70fa0a5da851b6d094a16df37c5da5ec、1a1b9056a8e51e1a089ebb4b3b53264b14a5cc47ead0705dab1f047765fdbc9d，均与 staging 一致；部署包 SHA256 63f868440c2bb76ece934da3999ccd4104e9bfb91ab2693c138874b990b23efd。部署与只读检查不证明实际电机运动安全。

2026-11-04 本地 position_pd 受控进入/退出及验收监视修改已完成；本地完整包构建、CTest 5/5 和验收工具离线测试 21 项通过（均非真机验证）。计划以备份和隔离构建部署板端，并执行输出禁用、零预算的 status/shadow 只读检查；部署尚未进行。传输源码包至 unitree@192.168.123.164 的提权 scp 被自动审批拒绝，理由是目的地归属和向该地址外传源码的授权未明确；需要用户明确授权该目的地及源码传输后再处理，不可绕过拒绝。

2026-11-04 板端只读 shadow：通过 192.168.123.164 连接，启动 hardware profile、output/两项确认均 false、预算 0 的现有节点；开始与约 3 秒结束 status 均为 state=valid、authorized=false、publisher_present=false、write_count=0，shadow 从 false 变 true。节点正常退出 0，查询无残留目标进程；未调用授权、模式或撤权服务。保存 3 条约每秒 obs/action/tau 原始日志和参数、哈希至 `tools/h1_2_policy_hardware/evidence/readonly_shadow_20261104/`。按记录离线换算 SDK 23 右肘目标约 -11.44 rad、当前 q 约 1.139 rad，软件 PD 未裁剪约 -99.4 Nm、裁剪 -18 Nm；position_pd 不受该软件裁剪。当前为 Passive 姿态且 shadow 存在前次 action 历史，不能当作 ArmHoldStand 实际接管首帧，但此输出不能直接用于实机。此前完整准备退出 11.654 rad/s 异常仍未解决；本次没有实机动作。

# 项目上下文

2026-11-04 用户提供板端当前连接地址 `192.168.123.164`；正在只读核实，连接状态尚未确认。此前 `192.168.8.6` 不通。

2026-11-04 仅离线核对 ONNX：`/home/llx/policy.onnx` MD5 与指南一致，ORT 元数据为空，无法从模型自证关节顺序或增益；`core.cpp` 的观测顺序、默认角、策略 kp/kd/scale 与指南一致，但腕 pitch/roll 依据 SDK 作交换。历史 ArmHoldStand 末态 q/dq 已保存，缺 IMU；按 gyro=0、gravity=(0.04,0,-1)、零指令及首帧历史为零假设推理，SDK 23 目标 -1.17223 rad，超本地 URDF 下限 -0.95 rad；指南参考观测也产生约 -1.226 rad 目标。虚拟 PD 目标超 URDF 限位是适配风险，不能单独证明模型错误或固件限位；native position PD 未应用软件力矩截断。ArmHoldStand 肘增益 120/4 切至策略 7.9/0.5。离线固定状态及前序 action 的试算见 `tools/h1_2_policy_hardware/evidence/policy_offline_audit.json` 和对应 C++；不代表实际完整观测、物理闭环或安全。历史完整准备撤权后超速仍未查清，本轮 SSH 检查返回 No route to host，未启动节点或执行动作。

2026-11-03 用户再次请求短动作并确认现场准备后，执行一次 FixStand-only position PD：3.101 秒，右肘 SDK 23 由 1.16701 到 0.896526 rad，距目标 0.52 rad 误差 0.376526 rad，超过本次 0.1 rad 诊断门槛而中止；未进入后续姿态或 ONNX。含清理及撤权后约 0.5 秒的最高累计速度 0.975203 rad/s，未观测超 2，观测完整；Passive 后撤权确认，最后 write_count=184 不变，节点退出 0、监测器 -2，无残留。最后遥测最大速度约 0.0421 rad/s，只代表该时刻。板端记录 `acceptance-20261103T214718`，本地只读副本 `/tmp/h1_fixstand_20261103T214718/`；等待现场反馈，不重试。此前完整准备退场超速问题仍未解除，本次窗口内结果不能证明窗口外安全。

紧接此前一次同范围短测试（板端 `acceptance-20261103T214459`，本地 `/tmp/h1_fixstand_20261103T214459/`）最大误差 0.429415 rad，观察窗口累计峰 1.138204 rad/s，观察完整并确认撤权、节点退出；同样未进入 ONNX。用户最新一次即时短动作请求已按上述 21:47:18 记录执行一次；两次短测试未观测超速不能外推完整准备退场已安全，此前 11.654 rad/s 异常仍待解决。

2026-11-03 紧急：一次获授权的 position PD 完整准备以预算 0 运行了 FixStand 和 ArmHoldStand，未进入 ONNX；程序随后请求 Passive 并撤权，确认发布器关闭且 write_count=340 不变，节点退出 0、监测器 -2，无残留。ArmHoldStand 末累计速度峰约 1.415 rad/s，但 Passive 清理窗口使用 `monitor_motion=False`，因此不执行 2 rad/s 超速检查；最终 Passive 快照峰为 6.522 rad/s，后续 telemetry 累计峰 11.654 rad/s，trace 约 10 ms 快照最大 11.184 rad/s（SDK 25 右腕 pitch），发生在撤权请求之后。该结果不能以进程 exit 0 判为安全通过；撤权后真实电机行为及现场状态须确认，立即暂停动作，不重试。板端原始记录 `/home/unitree/h1_nav2_ros2_ws/third_party/h1_policy_runtime/integration-record/acceptance-20261103T212658/`，本地副本 `/tmp/h1_phase2_20261103T212658/`。

用户已两次明确请求再次运行，并表示上次没看清；现场稳定情况尚未得到直接答复。鉴于实际记录存在收尾超速，先排查并修正验收报告缺陷，暂停原样重跑；该修正不能保证机械退场安全。撤权后 trace 中命令是最后接收的旧快照，不能据此认定撤权后继续发送。

本地验收脚本已修正报告缺陷：清理与撤权后观测到累计速度超 2 rad/s、或授权后观测不完整时，最终结果为 `needs_operator_check`，仍独立尝试撤权；撤权后只读观察 0.5 秒并记录窗口，不保证窗口之后安全。历史原始 summary 原样副本及独立分析位于 `tools/h1_2_policy_hardware/evidence/`。此修正仅离线测试，未部署或实机复验，不改变机械退出动作或保证其安全。

2026-11-03 较低幅度 FixStand-only 替代方案仅已暂存脚本 `/home/unitree/h1_nav2_ros2_ws/backup_policy_20261103_Fy7CEI/cleanup_review_PeoQVI/run_acceptance.py`（SHA-256 `efa24f66347af4f3809ecf4366560ddcddacad961d2dc879ba601ac79532944d`），dry-run 显示预算 0、0.1 rad 诊断门槛和撤权后只读观察 0.5 秒；未覆盖既有正式脚本或运行节点。用户此前请求重复完整准备，因收尾异常主动暂停原样重跑；现场当前稳定情况及用户是否同意此替代范围尚待确认，未进行新动作。

2026-11-03 用户补充：最近一次 position PD FixStand 准备阶段动作平稳，无抖动、甩动、碰撞或异响。这是现场观察，不代表 ONNX 或导航已通过；任何后续实机动作须另有具体授权。

2026-11-03 本地验收脚本已调整：完整准备流程可不设 FixStand 位置误差门槛，仅记录误差和最差轴；若显式设置则严格执行，FixStand-only 仍要求显式正有限门槛。摘要标记门槛是否应用；2 rad/s、状态/CRC、预算及策略姿态门槛不变。该调整仅完成离线模拟测试，未正式部署或实机执行；历史 FixStand 末态估算下一姿态右肘目标位移约 -1.676 rad、3 秒目标斜率约 -0.559 rad/s，非当前测量或实际峰速，须单独审阅及授权。

第二阶段脚本已仅暂存到板端 `/home/unitree/h1_nav2_ros2_ws/backup_policy_20261103_Fy7CEI/phase2_preparation_Wo9opN/run_acceptance.py`，SHA-256 `f2f566aa5d839d6080cf545918c2525aeceded98b00e514edfdac4c5e6368c87`；板端 dry-run 显示两个准备阶段、预算 0、FixStand 位置门槛未应用且仅记录误差。未执行控制或覆盖正式脚本；后续实际动作涉及更大右肘位移与更高增益，必须取得新的明确授权，上次平稳反馈不构成授权。

2026-11-03 最新：用户确认一次受限 position PD FixStand 准备阶段现场“动了”；ONNX 策略接管和导航仍未验证，不能称完整复现成功。该次 3.18 秒后 SDK 23 右肘位置误差 0.523235 rad 超过本次新设 0.1 rad 诊断门槛，程序返回 Passive 并确认撤权，未请求 ArmHoldStand/ArmHold；0.1 rad 不是原策略验收要求，不等于位置 PD 无效。累计速度峰 1.41699 rad/s，节点退出 0、监测器退出 -2，检查无残留。板端原始记录 `/home/unitree/h1_nav2_ros2_ws/third_party/h1_policy_runtime/integration-record/acceptance-20261103T211112/`，本地只读副本 `/tmp/h1_position_pd_20261103T211112/`。这与上次 torque 模式现场无明显动作的报告形成差异，但初始条件不同，不能认定因果或放宽保护；不自动重试。

2026-11-03：根据用户确认同台真机曾以 native PD 运行，现有 bridge 已调整为允许硬件选择 `position_pd`，此前仅允许仿真的限制属于历史状态。默认仍为 `torque`，硬件授权、状态门控、预算和速度限制不变；native PD 不受原软件力矩裁剪的实时约束，旧部署配置不可得，不能声称复现或已验证。该修订已部署板端，假 IO 和离线测试通过，尚未进行实机动作测试。

2026-11-03 板端旧版备份在 `/home/unitree/h1_nav2_ros2_ws/backup_policy_20261103_Fy7CEI/prior.tar.gz`；部署源码包 SHA-256 为 `fe90be2232278419c5bd4043abd114e1367e202f3b4cb4e7ab14b0909696017c`。板端构建、5 项包测试及 10 项验收工具离线测试通过，FixStand-only position PD dry-run 不创建 ROS 连接。首次只读验收因远程 shell 引号错误退出 127，未取得状态；修正封装后输出禁用的只读节点验收已通过：首尾两次 status 均为 hardware、Passive、position_pd、software_torque_clamp=false、authorized=false、shadow=false、publisher_present=false、write_count=0、state=valid；节点正常退出码 0，进程检查无残留。板端证据在 `/home/unitree/h1_nav2_ros2_ws/backup_policy_20261103_Fy7CEI/readonly_evidence/`。部署及只读通过不等于运动通过；未执行授权、模式切换或新的实机动作。

## 当前进展：现有桥接器的位置 PD 输出对照

- 用户确认：同一台 H1-2 曾在 mjlab deploy 使用该 policy 成功，只有模型/配置被替换，C++ 控制代码未修改；成功配套配置已无文件。范围仅参考其 DDS 控制逻辑改进现有 bridge，不引入 mjlab 部署程序。
- 现有 bridge 新增启动只读参数 `joint_command_mode=torque|position_pd`，默认 `torque`。两者共享单次观测/ONNX/相位推进、目标角和增益；位置 PD 输出 SDK 顺序的 q/kp/kd、dq=0、tau=0。原状态机、授权、预算、CRC 和超时机制保留。
- `position_pd` 现可显式用于 hardware，但仍须通过原有输出授权、状态和预算门控；默认仍为 `torque`。HG LowCmd 无逐电机力矩限幅字段，不能声称电机侧 PD 保留原软件力矩裁剪；配套历史参数也无法核实。未将新增节点部署至板端，未再进行任何真机动作，真机现场无动作与遥测变化的矛盾仍未解决。
- 本地包构建成功，CTest 的 node_integration/hardware/launch_parameters/core/onnx_regression 五项通过；新增 Fake IO 覆盖 Passive、两侧腕映射及真实 ONNX 动作到位置 PD 字段。ONNX 最大回归误差 0.000457704。Fake IO 测试受沙箱 socket 限制，不代表实际 ROS/DDS 通信验证。
- 实际端到端隔离仿真在 lo / DDS domain 42 / ROS domain 43 通过：现有桥接节点进入 ArmHold，以新位置 PD 输出运行约 3.58 秒；LowCmd 约 50 Hz、LowState 约 500 Hz，CRC/命令错误均为 0，高度 0.944–0.949 m，仿真力矩夹限计数 0。记录 `/tmp/h1_position_pd_bridge_check_allowed/`。仿真驱动需将同一命令模式同时传给物理桥和策略节点；该传参缺口已修正。上述结果仅证明隔离仿真路径，不证明固件限矩或真机接管。
- 用户当前无需继续按键或启动程序；不再追问丢失配置。新增路径尚未真机启用，导航硬件速度固定零的既定限制仍在，不能宣称真机导航已完成。
- 2026-11-03 受限真机入口已完成本地验证：5 项包测试和 10 项验收工具离线测试通过。验收工具必须在授权前读回并核对 `joint_command_mode` 与软件限矩状态，旧节点或模式不匹配会拒绝授权。首次计划只做位置 PD 的 FixStand-only（预算 0），不进入 ONNX；尚未执行。访问已记录板端地址 `192.168.8.6` 返回 `No route to host`，等待用户连接机器人网络，不能把本地编译当作板端已更新。

## 2026-11-02：策略接入真机无明显动作

- 用户反馈真机完全不动；参考记录为 `/media/llx/Data1/冷林啸个人知识库/AI/机器人导航/06 机器人运控策略更改.md`。最新准备阶段验收已发布 LowCmd，但进入 ArmHoldStand 约 0.06 秒后，SDK 19 左腕 yaw 的 `dq=-2.13684 rad/s` 触发验收脚本的 2 rad/s 保护，随后返回 Passive 并撤权；现场未观察到明显动作。该次策略预算为零，ONNX 尚未控制实机关节。发布和接收命令帧不能证明实机控制权或电机响应。
- SDK 19 未参与代码中腕部 pitch/roll 顺序交换。当前 hardware 模式固定使用零速度，主导航 policy 入口仅接隔离仿真，因此真机导航指令接入仍是后续阻塞。不要依据目前证据推断速度异常的具体原因或放宽保护阈值。
- 待核实线索：FixStand 切换 ArmHoldStand 时左腕 yaw 的 kp 从 7.9 立即变为 40、kd 从 0.5 立即变为 2；目标位置虽有 3 秒插值，增益没有插值。该转换仅要求 FixStand 持续至少 3 秒，没有稳定性判定。需用当次 q/dq 和 LowCmd 时序核对切换时是否发生力矩突变；目前不能认定为中止根因。
- 原始记录位于板端 `/home/unitree/h1_nav2_ros2_ws/third_party/h1_policy_runtime/integration-record/acceptance-20261102T115729`；初次连接返回 `No route to host`，随后网络恢复，已只读取回摘要、日志、轨迹、监测器源码及当次脚本至 `/tmp/h1_acceptance_readonly/`。当前工作区后续验收脚本与当次板端节点版本不同，不能混同；摘要中部署提交号由操作者填写，未独立验证。
- 原始摘要确认策略预算为 0；FixStand 末态右肘 SDK 23 为 1.11323 rad、目标 0.52 rad，左肘为 0.93606 rad、目标 0.52 rad，双膝为 0.75749/0.73428 rad、目标 0.5 rad。关节遥测发生变化，但其原因尚不明确。当次脚本不调用 Release，授权回执也说明释放未获机器验证，不能据此确认控制权。
- 用户补充上次没有手动释放原厂控制权或切换低层模式；目前用户明确报告通过遥控器 L2+R2 进入调试状态，与本地 `/home/llx/unitree_rl_mjlab/README_zh.md` 的描述一致。当前调试状态与独占低层控制的关系仍待验证；不能将未调用 Release 认定为上次无动作的唯一或确定原因，也不能自动套用 B2 的 ReleaseMode API。
- 2026-11-02 16:45:44（板端时间）执行一次已核查的只读 GetFsmId probe，捕获 `AnyCast error`，未取得返回码或 FSM 值；不能据此判断当前模式。同期按策略节点、旧 loco 桥和准备监测器等指定名称筛选进程未发现匹配，仅为当前限定范围快照，不能证明不存在其他控制源或解释历史现象。
- 轨迹采样中 SDK 19 只有一个点的速度绝对值超过 2 rad/s；`peak_dq` 是监测器累计峰值，不能视为每个采样时刻的即时速度。记录缺少逐关节命令和反馈力矩、电机模式及统一时间和阶段标签，不能确定增益切换与异常的因果关系，也不能将其认定为代码缺陷。后续先验证实际控制权和准备阶段响应，再验证 ONNX 策略，最后接入导航。
- 已找到自有 ArmHold 模型协议说明 `/home/llx/.codex/attachments/c35c98bd-a4c9-40ed-92fc-d83b96c53cd9/已粘贴的文本.txt`：指南建议 50 Hz 软件 PD，当前准备参数和相位处理与指南相符；原始训练及部署源码尚未取得。`unitree_rl_mjlab` 是另一套参考，其 1 kHz 差异不能认定为当前代码缺陷。2026-11-02 只读实算板端模型 `/home/unitree/h1_nav2_ros2_ws/models/h1_2_armhold/policy.onnx` 的 SHA-256 为 `3090836d0ce454a35161982b60dde20e0fd87627150e06126aabd1e6bbfabae7`，与预期一致。
- 验收工具已增加 FixStand 单阶段选项和显式位置误差门槛，并将逐关节状态、命令及同机单调时间记录加入新版监测器；保留 2 rad/s 保护。监测数据是异步快照，不证明控制权。改动仅完成本机离线测试与监测器编译，尚未部署、选择误差阈值或执行实机验收，须先审查具体测试方案。
- 新监测器写轨迹失败时退出；状态与命令为同一 Linux 主机约 10 ms 的异步最新值快照。FixStand 未到位时摘要保留误差和最差 SDK 关节。日志不可写及写入失败已在 DDS 初始化前离线验证；离线端到端模拟已通过，具体实机方案仍待审查。
- 板端已暂存新版脚本及监测器源码于 `/tmp/h1_fixstand_20261102_GE0EQd`，并备份旧监测器到该目录 `h1_prepare_monitor.previous` 后安装新版。板端编译、日志失败检查和 FixStand-only dry-run 已通过，未执行实机动作；板端工作区没有 `.git`，旧摘要记录的部署来源提交由操作者填写，不能独立证明节点二进制来源。
- 随后经现场授权执行一次 FixStand-only 诊断（策略预算 0、误差门槛 0.1 rad），FixStand 3.11 秒后右肘 SDK 23 为 1.12265 rad、目标 0.52 rad，最大误差 0.60265 rad，因超限中止；未进入 ArmHoldStand 或 ONNX。监测累计速度峰值 1.9864 rad/s，程序返回 Passive 并确认撤权、停止写入。该结果不证明撤流后的电机行为或控制权。原始记录为板端 `acceptance-20261102T170058`，只读副本在 `/tmp/h1_fixstand_result_20261102T170058/`；状态和命令是异步快照，不能直接据此认定命令被执行。
- 本次膝、肘的 `tau_est` 与 `cmd_tau` 同号且幅值接近，切回 Passive 后估计力矩下降并伴随遥测关节角变化；但用户现场明确反馈 17:00:58 试验没有任何变化，与 CSV 角度变化矛盾，不能将遥测关联认定为实际机械响应。0.1 rad 是本轮新设的诊断门槛，原 ArmHold 指南仅要求 3 秒插值，未达该门槛不能认定为原流程缺陷；低 kp 下负载平衡仅为待验证假设。响应图位于 `/tmp/h1_fixstand_result_20261102T170058/response.png`。
- 暂停全部动作。当前只读 DDS 发现窗口有一个 HG `rt/lowstate` writer，参与者前缀 `01108860ddcc47e3eb27f07b` 的发现多播来源为 `192.168.123.161`；当前未发现 `rt/lowcmd` writer 或策略控制节点，本机和板端进程名筛选未发现已知仿真器。这些是当前快照，不能证明历史来源、现场机器人实体或不存在其他隐藏来源，不能声称已发现仿真。此前只读 GetFsmId 抛异常，当前模式仍无法确认；用户对按 L2+R2 前的零力矩状态不确定。
- 用户不确定按 L2+R2 前是否已处于零力矩；仅有按键动作不能确认调试状态。Unitree 官方 RL 实物部署说明覆盖 H1_2，要求吊装启动后等待零力矩、确认该状态后按 L2+R2，但未给出零力矩的独立判定或从不明状态切换的方法；应查该台机器人手册或向厂商确认状态指示，不套用其他型号按键。板端 SDK 的 H1 GetFsmId 在检查 RPC 返回码之前即解析响应，解析可能先抛 AnyCast；本次未得到原始返回码和响应，无法判断异常原因或据此证明 H1-2 接口不兼容。
- 用户随后明确表示已完成模式操作且当前已是调试状态，作为现场报告记录，不要求重复按键，也不能把缺少机器验证当作模式错误的证据。现有软件仍不能独立确认：`mode_machine=6` 用于型号匹配，`mode_pr=0` 表示踝关节 PR 映射，最近 FSM 查询超时。下一步围绕 LowCmd 实际执行与现场观察、遥测之间的矛盾排查，不因模式无法独立判定而反复循环；未进行新的实机动作。
- 已编写 `tools/h1_2_policy_hardware/get_fsm_readonly.cpp`，固定只查询 H1 loco GET_FSM_ID，先输出原始返回码和响应，再解析成功响应。2026-11-02 17:13:15 板端经审查只运行一次，返回 `3104`、原始响应为空，退出码 1；板端 SDK 将 3104 定义为 RPC 调用超时。旧版无条件解析空响应可能解释此前 AnyCast，但无法证实之前的返回码，也不能据此判定机器人模式或 H1-2 兼容性。结果和程序保存在板端 `/tmp/h1_fsm_readonly_20261102_7TZ4DQ/`，未重试。
- 修正验收脚本此前无条件声明调试和独占控制已确认的问题：`--execute` 现在要求显式人工确认两项，dry-run 默认传 false；这些标志不是机器验证。当前模式及控制权仍未确认，且现场反馈与遥测矛盾，不再进行实机试验，直至按适用 H1-2 流程核实实际状态。此修订仅本地离线验证，未重新部署板端。
- 用户报告同一个 policy 曾在 `unitree_rl_mjlab/deploy` 跑通，尚待确认是真机还是仿真、实际启动命令和版本。本地 checkout 所带模型 SHA-256 为 `04d58ad85246e6522768fb81d97a2ffea318666199f308ccca6831af4761392c`，不同于当前模型 `3090836d0ce454a35161982b60dde20e0fd87627150e06126aabd1e6bbfabae7`，不能将本地模型视为历史成功版本。用户明确纠正范围：只参考 mjlab 的 DDS 真机控制逻辑来完善现有 bridge，保留现有 ONNX，不引入 deploy 系统或迁移架构；先前复用 deploy 的主线建议废弃。本地参考实现为 1 kHz 发送、50 Hz 策略及电机侧 PD，当前桥为 50 Hz 显式力矩；这些差异不能直接判为缺陷。未获授权前不做新的实机动作。
- 本轮静态 DDS 审查中，HG 类型、`rt/lowstate`/`rt/lowcmd`、硬件 eth0/domain 0、`mode_machine=6`、`mode_pr=0`、电机命令 mode 1、默认 q/dq 为 0、按消息大小计算 CRC 以及 Write 失败处理均未发现可证协议缺陷。参考实现保护共享命令消息的锁与现有 bridge 每次构造局部 LowCmd 的模型不同，不能机械照搬。参考电机侧 PD 和高频发送与自有指南的 50 Hz 软件 PD 存在差异，但不足以证明需要更改频率或控制模式。硬件导航速度仍固定为零，说明导航未接通，不能解释准备阶段不动。已询问用户成功 deploy 是仅换模型配置还是也改过 C++ 力矩或 ArmHold，等待答复；不再重复追问启动命令或引入部署系统。本轮未改控制代码、未进行实机动作。
- 上述只读调查之外，仅执行了所记载的一次受限 FixStand 诊断；未执行策略接管或导航。
- 已下载并视觉核对宇树官方 H1-2 Remote Control User Manual V1.0（2025-04-18）图像页：R3 第 9、10 页本地分别为 `/tmp/h1_2_remote_page9.png`、`/tmp/h1_2_remote_page10.png`；官方原图分别为 https://doc-cdn.unitree.com/static/2025/4/18/26100da7079c4e109e8eaa999533ac36_2482x3509.png 和 https://doc-cdn.unitree.com/static/2025/4/18/282e5f1d1d5b45d5b3afe372d8bddca3_2481x3509.png ，R1 相应内容重复于第 15、16 页。第 10 页表格列一般情况下 L2+B 进入阻尼、L2+R2 进入 Debug Mode，在阻尼模式下 L2+Y 进入零力矩；第 9 页状态图显示阻尼通过 L2+R2 进入 Develop，文字称 Debug Mode，第 10 页对应操作表称 Develop mode。第 9 页描述零力矩状态摆动无阻尼感，阻尼状态有明显阻尼感；RL 实机指南仍要求吊装启动并等待零力矩后再按 L2+R2。手册第 9 页提及 L2+A 用于确认调试模式，但第 10 页明确它会执行位置控制和诊断动作，不能作为无动作只读查询；当前现场模式未确认，保持暂停所有动作，不根据这些资料自行发送控制或按键。

## 2026-11-05 guarded release v3 deployment completed

Policy-only release promoted at `/home/unitree/h1_nav2_ros2_ws/releases/guarded_release_v3_20261105`; paired runner/source synchronized, backup and rollback in `backup/pre_promote`. Board fake-IO core/node tests 2/2 passed. Restored missing colcon package index; ordinary workspace setup resolves new release. Output-disabled status confirmed v3, valid state, authorized=false, publisher_present=false, write_count=0; node exited 0. No real action performed; physical release safety and ONNX takeover remain unverified. Evidence: `tools/h1_2_policy_hardware/evidence/guarded_release_deploy_20261105/`.

## 2026-11-05 bounded zero-command ONNX trial (acceptance-20261105T202400)

Operator confirmed posture, then arm_hold service accepted. 155-frame / 3.1-second budget ended in Hold; node logged policy budget exhaustion. Recorded cumulative peak dq 1.203186393 rad/s, no >2-rad/s event and no incomplete observation. After explicit support/release confirmation, guarded release blocked approximately 0.59 seconds after request; native damping not verified. Operator confirmed static supported physical handoff and explicitly authorized REVOKE. Revocation confirmed, final publisher=false/authorized=false/write_count=7043; node and monitor exited 0. Overall needs_operator_check, not a full pass or walking validation. No walking command or retry. Evidence: tools/h1_2_policy_hardware/evidence/onnx_bounded_20261105T202400/.

## 2026-11-05 release diagnosis and observed right-elbow drop

See docs/h1_2-policy-troubleshooting.md for concise issue/cause/fix/verification. Added persistent blocked-release diagnostics without changing trajectory/gains/thresholds, local and board core/node tests 2/2 passed; promoted releases/guarded_release_diagnostics_20261105, output-disabled check passed. Trial acceptance-20261105T204047: awk console buffering delayed confirmation, timed out, no arm_hold event/no ONNX; operator-approved supported release from preparation Hold blocked at 2.625s, SDK23 dq=.500691, q=-.270177, last q=-.569702 kp15.7943 kd4. Operator observed right-elbow drop, then confirmed static supported handoff and authorized REVOKE; revocation confirmed, children exit0, no target processes. Demonstrates failure does not require ONNX. Support and load conditions during release remain unverified; a later trial blocked despite operator-confirmed support. Physical fixture details and reliable native damping handoff remain unverified. Do not change model contract or loosen guards. Fixed terminal flow with --brief-summary, removed awk filtering, clarified actual arm weight-bearing requirement; 3 targeted tests pass. Full runner tests initially35/36, parent-death standalone rerun passes; initial failure unexplained, do not claim repaired. Runner synced board with backup, no further action.

## 2026-11-05 corrected user walking-test scope

User explicitly clarified actual use: keep ArmHold policy/arm control active, briefly command speed, then zero velocity and continue policy standing. Do not automatically lower arms/release gains. Supported-release failures are a separate exit-path issue, not evidence that continuous ONNX walking fails and not an automatic prerequisite for this different authorized test. Existing finite acceptance runner cannot satisfy continuous standing; isolated manual trial entry requires implementation/verification before any walking command. Keep state/speed/exclusivity/revocation protections. No new robot action yet.

## 2026-11-05 manual walking entry and aborted zero-command test

Implemented isolated manual trial in run_acceptance.py with manual_pulse.py: --continuous-walk-trial enables existing continuous hardware node with unique cmd_vel topic and remapped private manual heartbeat in ROS43 localhost; no production Nav2 gate spoof or production cmd topic. One vx=.12 m/s .5s monotonic publishing window then 0 and continue monitored ArmHold until explicit physically-supported REVOKE. Existing .4s command watchdog means no hard realtime physical .5s stop claim. Original diagnostic flow retained. Installed scripts with backup in releases/manual_walk_trial_20261105.
Trial acceptance-20261105T210530: after onsiteCONFIRM ArmHold entry finished~3.079s, navigation_ready true and policy_cmd0; new .5rad/s stationary gate immediately aborted on arm transient (nearby SDK16 left elbow readings .722/1.037/.459/.481/.576), peak1.426<2, no forward velocity sent. Revocation confirmed, children exit0; user confirmed static/stable/support retained. The .5 gate is new engineering precheck, not original2 overspeed or ONNX walking failure. Fixed to zero-command settle up to10s, requiring3s consecutive sampled speeds<.5, resetting on motion (including after prior completion). Helper5+manualfake4 tests passed local and board9/9; staged update installed, no further action. Docs: docs/h1_2-policy-troubleshooting.md. Full board trace copy denied automatic review for internal data/destination authorization; only necessary remote aggregate calculated, original stays board, aggregate stored evidence/manual_walk_trial_20261105T210530/aggregate.txt. Do not retry full-copy denied action or claim heartbeat never lost.

## 2026-11-05 manual walking trial fresh-frame abort

Record acceptance-20261105T211405: zero-command ArmHold entered; second stationary gate watch(.1) aborted before pulse. Initial status RPC took 81.032 ms while watch deadline had already started; status write_count 2292 to 2293, below existing minimum 2. Failed window received/command age were not saved. Nearby trace command gap max 20.1 ms; this sampling does not establish exact failed snapshots. All status policy_cmd=0 and limited_frames=0, peak dq 1.332 rad/s, no recorded 2 rad/s event. Revoke confirmed, children exit0; user confirmed stable and supported. Fault observed only after revoke. No walking validation or automatic retry. Local fix starts observation after initial snapshots, retains count threshold and records failed-window metrics; targeted fake-IO and existing manual tests 7/7 passed. Not deployed or field-tested. Aggregate: tools/h1_2_policy_hardware/evidence/manual_walk_trial_20261105T211405/aggregate.txt; docs/h1_2-policy-troubleshooting.md.

- 21:14 watch-window timing fix promoted: board 7/7 targeted fake-IO tests passed after loading ROS, runner backed up at releases/manual_walk_trial_20261105/watch_window_fix/run_acceptance.before_fix.py. Count threshold unchanged; start window only after initial snapshots and record precise failed-window counts. No subsequent physical retry; user confirmed stationary/stable/supported after revocation.

- 2rad/s threshold audit: trial212015 DID show policy_cmd=.12 before SDK9 right-knee dq2.169/peak2.220. Unified2 is project acceptance engineering limit applied into walking, not established motor capability/safe limit. Reference deploy has no found measured-dq grouping protection config; URDF velocity and trans observationclip are not authorization thresholds. No threshold edit/retry; need concrete verified hardware policy-stage limit rather than invent a number.

## 2026-11-06 user-supplied trained guard profile

Full training statistics stored docs/train.md. User explicitly confirmed BOTH speed/position hip_pitch/hip_roll labels swap to match current SDK1/7 pitch and2/8 roll. Implemented opt-in train_20261106 profile: current values, per-joint 3 fresh50Hz samples, warm2s speedx2, actualq soft bounds intersect URDF, all27 prearm |q-default|<.07, basegyro2.5, roll/pitch>.2 for.5s; node latch sends true native Passive damping without revoking, runner observes damping and waits operator REVOKE. Legacy and normal zero-velocity ArmHold behavior retained; no new act>3 cutoff. Pureguard passed, core/node2/2, Python profile/workflow36 passed. Source/effectiveprofile SHA saved evidence/train_guard_20261106/. Not deployed or run: boardSSH twice timeout, local enp43s0 UP but NO-CARRIER, route goes Wi-Fi. Do not claim board new guards active; do not start robot from historical site confirmation. Next task tasks/20261106-trained-policy-guard.md.

## 2026-11-06 LowCmd gate provenance correction

The initial any-LowCmd abort is already in eac4a99 (2026-11-02 acceptance runner), not added by the current train profile. The monitor's total conflates vendor humanoid traffic around 500 Hz (metadata source `.161`) with this process's output; publication/source metadata does not establish execution ownership. The 145101 preflight stopped before node spawn, with 301 total commands in 0.612 s. Preserve base processes and modes; do not stop the base or switch modes to clear this gate. The trained release was already deployed and passed board readonly verification.

User has now freshly confirmed field readiness and this bounded action: `vx=0.12 m/s` for `0.5 s`, then zero velocity and continue ArmHold, with no release. This is confirmation of intended scope, not evidence the action ran. Implementation is adding the policy writer's own GUID via atomic write to the path in `H1_POLICY_WRITER_GUID_FILE`, plus per-source monitor fields. This change is not yet verified; keep the action result pending until implementation and evidence are available.
