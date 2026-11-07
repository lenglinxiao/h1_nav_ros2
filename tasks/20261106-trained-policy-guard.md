# 任务：20261106-trained-policy-guard

- 状态：进行中（用户确认持续迈步并站稳，ArmHold idle）
- 更新时间：2026-11-07 Asia/Shanghai
- 负责人 / 认领：root；保护类、节点集成、runner代理已冻结交付；后续写入需重新协调。
- 目标：保存用户训练结论，按其数据落实保护配置，必要验证后执行原定短速度试验。
- 范围：docs/train.md、纯C++保护类、Node及runner、对应测试；不修改模型/PD增益，不扩展导航速度。
- 依赖：[前期试验审查](20261106-policy-walk-gates.md)；可连接的板端与执行时有效的现场条件。
- 功能计划：[ONNX 运动与导航开发计划](../docs/h1_2-onnx-navigation/plan.md)；导航接入的[六项开发任务](../docs/h1_2-onnx-navigation/development-plan.md)与[设计提案](../docs/h1_2-onnx-navigation/design.md)待审阅，不改变当前现场授权。
- 2026-11-07 交接校正：本会话执行记录显示旧 38895 已撤权且节点/监测器退出0，3秒脚本已部署并启动试验（PTY19495）；该试验结果未取得，当前现场状态未知。下列旧状态存在冲突，以后续核实为准；测试计划见 [test-plan](../docs/h1_2-onnx-navigation/test-plan.md)。
- 2026-11-07 最新转向：用户授权一次固定左转 `(vx,vy,wz)=(0,0,0.1)`、3秒；runner报告窗口结束、预期非零和随后全零命令，未报告取消。用户确认已向左转动、归零后站稳；PTY26347零ArmHold保持，本次授权已使用，不自动撤权/release。记录 acceptance-20261107T113404，功能计划见 [转向测试](../docs/h1_2-onnx-turn-test/plan.md)。
- 2026-11-07 前次直行记录（已撤权）：现场已确认支撑、净空、急停、调试及接管条件；控制口载波恢复且SSH可达。一次授权0.12 m/s、3秒窗口runner报告结束，观察到非零及随后零 policy_cmd，无取消提示；用户现场确认持续迈步、归零后站稳。PTY72345随后撤权确认，节点及监测器退出0。记录板端 acceptance-20261107T112810。此前3秒记录162248无summary，不能判断其结果；旧现场状态如下为历史，不适用于当前。
- 现场负载：用户明确支撑主要用于防摔，重量仍由双脚承担；不得将可靠支撑理解为已卸载。该事实不证明偏差原因。
- 当前摘要：用户确认能承托身体与双臂并许可一次更新重启及0.12 m/s 3秒测试。旧95666记录 revocation_confirmed=true、节点/监测器退出0；新节点已部署，未授权状态核对通过。记录 acceptance-20261107T171020，PTY27864 报告一次窗口结束、非零及随后ArmHold零命令，现ArmHold idle；用户确认无外部悬吊/防摔支撑下本次迈步并独立站稳，未发REVOKE。
- 当前问题：用户确认支撑可靠、机器人稳住；trace 77.38–79.37秒SDK3/9实际约1.16561/1.17553，目标均0.9，SDK4实际约-0.71598、目标-0.45，误差持续约0.266–0.276 rad。实际命令目标等于训练默认，具体物理原因未确认；保留保护、不无改动重试。
- 下一步：保持当前ArmHold，保持现有 ArmHold；每次新 WALK 需新指令，不自动撤权、release或发送WALK。详情见 [功能计划](../docs/h1_2-armhold-repeat-walk/plan.md)。
- 授权边界：本次可靠支撑许可仅覆盖已执行的撤权、部署重启及一次初始测试；不得泛化。此前无吊带约束仍须按当下实际支撑条件审慎处理。

## 已实现

- 逐关节当前值/实际位置，连续3个新鲜策略观测去抖，同tick不重复计数；不复用累计峰值。
- ArmHold接受后前2秒关节和基座速度门限×2；姿态及位置限位不放宽。
- 历史板端版本要求全27维预接管最大默认角误差<0.1；2026-11-07 已部署为仅记录。运行期软边界仍取用户表与指定URDF交集。
- 基座角速度范数2.5；由完整projected gravity计算roll/pitch，任一绝对值>0.2持续0.5秒触发。
- 同一控制tick直接请求Passive原生阻尼，保持授权持续发包并锁存；runner观测native damping并等待人工REVOKE，不因模式变化立刻撤权。状态/CRC/写入等原故障路径保留。
- legacy默认保持，不改模型、动作scale/offset或PD参数；未新增act>3急停条件。

## 验证

- 纯保护：trained_guard单项通过，Release下显式保留断言。
- 节点/控制核心：绝对build路径下ctest -R '^(core|node_integration)$'，2/2通过，6.76秒。
- Python：ROS Humble环境下test_train_guard_profile与test_acceptance_workflow，36项通过，13.178秒。
- 对应代码hash、有效SDK配置、URDF hash及命令记录：tools/h1_2_policy_hardware/evidence/train_guard_20261106/。
- 配置hash：f47bae87fe47d4743dea5b748acc79ff3d4f6c69df339fc34e662d92b8436284；原训练统计未重新计算。
- 历史源码包：/tmp/h1_train_guard_20261106.tar.gz；已被后续训练准备目标及预接管门限部署替代，不代表当前板端版本。

## 外部条件

- 历史现场曾确认支撑、净空、急停、调试及接管；最近 PTY27864 报告 ArmHold idle，现场状态易变，以当下反馈为准。

## 交接日志

- 2026-11-06 本次：用户确认支撑、急停、调试及接管条件并授权一次 0.12 m/s、0.5 秒。来源归因隔离域197/lo双writer测试：有效自身125帧、故意坏CRC 10帧归因正确；自身停止后计数不变、帧龄增长，同时外源计数从167增至284。板端 `/tmp/h1_guid_isolated_197/trace.csv`；不能由此证明现场控制权。profile hash 为 f47bae87fe47d4743dea5b748acc79ff3d4f6c69df339fc34e662d92b8436284。实际 FixStand、ArmHoldStand 后，训练 prearm 因 SDK24 偏差 0.0838258 超过 0.07 拒绝，未发布非零速度，首次随后经用户确认撤权，node/monitor 均退出0。

- 2026-11-06：保存完整训练结论、澄清髋映射，完成本地保护路径实现和必要验证；因控制口无物理链路，尚未部署/执行。连接恢复后不能把此前离线状态当成现场已就绪。

- 2026-11-06后续：连接恢复后正式包切换至releases/train_guard_20261106/install/h1_2_policy_bridge，备份在backup/；readonly_result.json及installed_sha256.json为板端证据。用户明确禁止关闭底板进程或切换模式。Git确认旧门禁来自11月2日eac4a99；145101预检因混合LowCmd总计数而停止，未启动策略节点。此次来源归因修复仍在实现，未验证；本次确认的0.12 m/s、0.5秒动作尚未执行。

- 2026-11-06 后续：训练专用准备目标改为已校验 config.defaults，经 SDK 顺序转换，legacy 和 PD 不变；本地 core/node 集成 2/2 通过。板端备份于 releases/train_stand_fix_20261106，已部署 executable SHA256 eb8874e04fadfbd14291d0d730d7aa04c4581a8b592436fd79943f55696a9b44、core SHA256 5e640936faab699322dd0864692ee6aeb4acd5722f6d1a1bb8abc58164c232aa。第二次 SDK10 拒绝，未发布非零速度，记录 acceptance-20261106T155348；按用户指示继续保持、不撤权。

- 2026-11-06：用户指定预接管门限严格小于0.1 rad；仅训练保护门限更改，膝门限、速度/姿态/位置保护不变。trained_guard和node_integration 2/2通过（6.35秒），含0.099通过及等值拒绝。板端备份 releases/prearm_010_20261106/backup.tar，header SHA256 fc8af0703c91e34e47deee8418cd47fcddb3ae1f88dfa94c8f142ee9296b0524，可执行程序 SHA256 600eeb58b03e8da5a94c1d3b7887b19a3b83cffda4ad12c154b6a7a6c5776133。一次脉冲日志报告 nonzero=True、zero in ArmHold=True；用户确认已迈步、归零后站稳，持续保持；一次脉冲授权已使用。

- 最新只读监测：板端记录 `third_party/h1_policy_runtime/integration-record/acceptance-20261106T160641/`；约89.3秒时状态44644帧、自身命令4398帧、自身命令帧龄接近0、状态及自身命令CRC错误均0；累计最大关节速度 SDK3 5.69848 rad/s，累计峰值不等于训练保护三连续新样本触发判定。独立ROS status服务查询不可见超时，不能凭该查询独立确认guard状态。终端无新告警；用户未授权此次撤权，保持PTY61909。

- 2026-11-06：用户授权0.12 m/s单次1.5秒。ManualPulse时长范围0.5–1.5、默认0.5；11项单测及3项runner定向测试通过。板端脚本备份 releases/manual_pulse_15_20261106/backup.tar；manual_pulse SHA256 be0718818082b7520033311364a0c07fe1acb2214990f1ec761ae8ff87aa6549、runner SHA256 2f518f0b672512882cc237f3e773ff7f4812754254aeb2f6c832f5a8467d5272。记录 acceptance-20261106T161329，非零命令已观察、零指令已观察，ready lease expired中止，窗口是否完成未证实。

- 1.5秒取消分析：watch状态续租以响应结束为基点再等待约0.1秒，间隔还包括下一次同步RPC耗时；watch入口初始状态不续租。fake runner注入一次81ms状态延迟，在1.5秒参数下复现 ready lease expired（定向1/1通过）；现场未记录对应延迟，具体原因未证实。用户确认迈步后停稳，PTY38895保持；尚未修改续租调度。

- 本地待部署修复：watch 对有效状态响应续租，ArmHold按请求开始时间20ms调度，租约保持150ms；其他模式保留0.1秒状态间隔。fake在一次81ms延迟下：50ms请求间隔产生0.151秒续租间隔而取消，20ms版为0.121秒且完成；超过150ms及ready丢失仍取消。定向16项通过，本地runner SHA256 a99a8d71bdf8a06a0d608496bb7003134a8af21a0fcdb9c422da019839bc187c，尚未部署；模拟不能证明现场RPC延迟。等待现场支撑及撤权/单次重试确认，PTY38895继续保持。

- 用户要求更久测试，拟单次0.12 m/s、3秒后归零；当前试验部分脉冲不等于完整1.5秒。仅本地将可配置时长上限由1.5扩至3秒（默认0.5不变）；3秒结束归零、2秒后ready/租约中断、超过3拒绝及runner 3秒流程定向3项通过。尚未部署、尚未获得当前支撑撤权确认，PTY38895继续保持。

- 2026-11-07：链路从NO-CARRIER恢复，SSH可达；当前模型3090836d0ce454a35161982b60dde20e0fd87627150e06126aabd1e6bbfabae7、URDF e5dae04b4f16590d34a64d8bb9a0bcc2a24072f79b05ec4371d49866fb7228bd、profile f47bae87fe47d4743dea5b748acc79ff3d4f6c69df339fc34e662d92b8436284与既有证据一致。runner d0bf1a034a4c111b3ceb16d3809881bff1580faecffd9a48a272f3b1cb66e773、manual 7f3bcee7b03f0ad7d15cab08622501f535e5c23ef9108b1e2423c68f0b8efa08、节点 600eeb58b03e8da5a94c1d3b7887b19a3b83cffda4ad12c154b6a7a6c5776133；source commit为dirty基准非完整版本。3秒窗口结束，非零及零命令已观察，用户现场确认持续迈步、归零后站稳。

- 2026-11-07 转向：固定 `--manual-motion turn_left --manual-pulse-seconds 3.0`，向量(0,0,0.1)，runner观察预期非零与归零，用户确认已向左转动、归零后站稳；右转、更大角度、长时及导航转向未验。记录 acceptance-20261107T113404。定向测试15项及新增2项通过；板端manual SHA256 5c1e033a1ec2fd7990576b0bcce4b3a8455dfdd8143d00a89c72cd4e39876edc、runner SHA256 41f700cc1c83bce899f7701d6b9e4acc492075c95b569c639fbb259d6bdf6821，备份 releases/manual_turn_20261107/backup.tar。旧直行PTY72345已确认撤权、节点/监测器退出0。

- 2026-11-07 repeat部署：备份板端 releases/repeat_walk_20261107/backup.tar；runner SHA256 07796ab1d3c70bb96fcb463563ade924719870c2978a9dff21997c5b4b9e7702，manual 5c1e033a1ec2fd7990576b0bcce4b3a8455dfdd8143d00a89c72cd4e39876edc，repeat_input d83f034c4a7de8b9ace8860bc2b4edadddc1933f48af7ee29c4bea9455aa57cb。记录 acceptance-20261107T115142：SDK4 prearm误差0.273897，未发布非零命令；当时 PTY95666 保持 ArmHoldStand，后收到 SIGINT 且撤权确认、节点和监测器退出0。

- 2026-11-07 本地：按用户要求训练配置取消 ArmHoldStand→ArmHold 默认角误差及膝目标差拦截，保留诊断、legacy 门限和运行期保护；Fake IO/core 定向测试通过，未部署。先前 PTY95666 状态为现场报告，易变须复核；本地操作不触碰该进程。无本次保存的 hash。

- 2026-11-07 17:10：训练接管角差诊断修改部署，备份板端 releases/entry_diagnostic_20261107/backup.tar，SHA256 c1f5a5f2a68feb398c2eb155a579a584c5f423a175f8ba4e6afead755cd95cef；源及产物hash保存 releases/entry_diagnostic_20261107/sha256.txt。未授权节点显示 entry_angle_gate=diagnostic_only、state=valid、authorized=false、publisher_present=false、write_count=0，退出0。runner 报告一次 0.12 m/s 3秒窗口、非零及随后零 policy_cmd，用户确认持续迈步且归零后站稳；PTY27864保持ArmHold idle，记录 acceptance-20261107T171020。一次试验授权已使用。

- 2026-11-07 重复 WALK：用户明确授权同一 ArmHold 下测试，PTY27864 输入一次 WALK；前后 status 均 ArmHold、ready=true、policy_cmd=0、guard未触发，runner/node/monitor PID 10257/10335/10326 不变，writer GUID `011053416022fa9b8f51f8a700000403`。输入耗时约3.7秒后再次显示 idle；代码在脉冲中以 watch 校验 ArmHold，但本次 repeat_walks 内存记录须退出后才能读取，非零观测尚未独立确认；用户确认再次迈步、结束后站稳，第一轮同进程 repeat 已实机确认；未调用模式切换服务、重启、撤权或再次触发，本次授权已使用。记录 acceptance-20261107T171020。

- 2026-11-07 无外部悬吊/防摔支撑：用户确认独立站稳、净空、可急停并授权一次WALK；原PTY27864发送一次后返回ArmHold idle。前后status为ArmHold、ready、零命令、guard未触发，PID及GUID不变；窗口内runner检查ArmHold，repeat内存事件尚未落盘；用户确认已迈步、结束后独立站稳。未撤权、切模式、重启或再次WALK；PTY27864 idle，本次授权已用，不推及长时或导航安全。记录 acceptance-20261107T171020。
