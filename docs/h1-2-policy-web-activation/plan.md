# 原导航网页 Policy 准备控制

## 2026-11-16 WebSocket 心跳端口错配

- 现象：网页提示“浏览器心跳未连接或已超时；Policy 控制请求未发送”。独立Web实际监听8775/8776，但 `/api/config` 中持久化旧端口8766覆盖运行端口，页面连到旧WebSocket服务。
- 修复：读取持久化设置后始终以当前实例端口填充heartbeat配置；新增保存值8766不能覆盖运行值8776的回归测试。
- 验证：相关测试80 passed，Python compile与scoped diff check通过；PC4构建成功。独立8775服务active，配置API报告8776，WebSocket握手收到 `{"type":"ready"}`；原8765服务未重启。
- 现场边界：当前只读API报告Nav2与Policy bridge均未运行；没有授权、FSM或路线操作。强制刷新 `http://192.168.123.164:8775/` 可恢复网页心跳，但不会启动机器人控制。

## 2026-11-16 拆分 ONNX 控制与导航启动

- 用户确认的流程：先启动 ONNX 控制并完成 `Passive → FixStand → ArmHoldStand → ArmHold`；随后单独启动导航栈；路线/巡航仍由现有路线按钮显式开始。
- 范围：Policy bridge 可独立启动和跟踪；ArmHold 控制准备不依赖 Nav2；导航启动必须验证现有 bridge 已授权、处于 ArmHold、LowState/publisher 有效，并复用 bridge。自动路线启动保持关闭。
- 非目标：不改变 bridge 控制协议、FSM 步骤、速度限制或真机准入；不操作/重启当前现场控制栈，不发送真机控制命令。
- 验收：启动 ONNX 只创建 bridge；导航启动前缺少 ArmHold 时服务端拒绝；满足 ArmHold 后启动导航时不重复 bridge；启动 Nav2 不触发路线；UI 清楚区分控制就绪、导航栈运行和巡航运行。
- 步骤：1) 已完成：脚本支持 `mode=policy_bridge_only`，Web 独立跟踪 bridge 并拆分启动/API/UI；2) 已完成：77 项定向测试、Python/shell/JS 语法及 scoped diff 检查通过；3) 已完成：部署 PC4 主工作区并在独立端口启动新 Web，原 `8765/8766` service 未重启。
- 安全边界：部署前只读现场状态为 Policy `Passive / authorized=true / publisher_present=true / navigation_ready=false`，路线 idle；bridge/Nav2/FAST-LIO/route runner 属于原 Web user service 的 `KillMode=control-group`。为避免中断现场控制栈，新 Web 运行于独立 transient user service `h1-2-web-policy-split.service`，监听 `8775/8776`。本轮没有发送 Policy/FSM/route 控制请求。
- 验证：`python3 -m pytest -q test/static/test_h1_2_nav_web_gui.py test/unit/test_web_task_profiles.py test/unit/test_policy_web_client.py`：77 passed；Python 编译、shell/JS 语法与 `git diff --check` 通过。PC4 `h1_2_nav_tools` 构建成功，`http://127.0.0.1:8775/` 返回 200，新 UI 包含“启动 ONNX 控制桥 / 启动导航栈 / 开始巡航”；只读 `/api/status` 可显示 bridge、Nav2 和 route idle。板端 SHA256：GUI `89816d17aa135ee938272327517ef0ab0051a3eb1ac571514dcb514592312f1a`，helper `4bbf2d3b64890e8e05ff22518fad6fe30997277ea60688a87856a7da9e32b595`，脚本 `4169090b56394bcd5f1a320a8d6456a7793c25be67f69827967a2e6b4e2d400f`。备份位于 `/home/unitree/h1_nav2_ros2_ws/backup_policy_nav_split_20261116/`。
- 实机接续：页面当前看到的桥是已授权的 `Passive`，不是全新未授权状态；请勿重复授权。进入任何 FSM 姿态步骤前，需按现场测试方案重新核实支撑、急停与 LowCmd writer 控制权。当前没有 ArmHold 或巡航验证结果。

## 2026-11-16 首次网页心跳误判修复

- 现象：新版独立 Web 服务启动时，浏览器尚未建立第一次 heartbeat，`_heartbeat_age()` 返回 infinity；看门狗把未连接误判为超时并锁存 Policy 准备，页面提示“网页心跳曾超时；请显式重启导航实例”。
- 修复：看门狗在收到首次 heartbeat 前不进行断连超时判定；收到过 heartbeat 后，原有超时保护保持不变。
- 验证：新增首次 heartbeat 前不锁存、不 stop 导航/route 的回归测试；定向测试78 passed，Python/JS语法、scoped diff检查通过。PC4仅重建 `h1_2_nav_tools` 并重启独立 `h1-2-web-policy-split.service`；原 `h1-2-web.service` 和机器人控制进程未重启。
- 现场只读复核：新版 HTTP 200，API 可读 Policy `Passive / authorized=true / publisher_present=true`、route idle；新 Web log 没有启动即超时记录。未执行 FSM 或路线命令。新版 GUI SHA256 `8cdaef895fbcafa2824d179dc633add50560dd28b0f752b0cc83442d65ab80a8`。

## 2026-11-16 ONNX 控制栏整理

- 现象：ONNX 状态、向导和一般主控制按钮混排；用户难以区分选后端、启动导航栈、准备运动控制和开始巡航。
- 调整：ONNX 面板明确显示“导航与 Policy 桥接 / ONNX 运动控制 / 巡航任务”三段状态；顶部保留唯一“启动 ONNX 模式”入口；后端选项仅显示配置提示。准备浮窗按“启动前现场确认 / 准备进度 / 当前下一步”组织，窄视口按视口宽度约束。
- 边界：不改变服务端确认、桥接状态和动作门控；不自动授权、转换姿态或开始巡航。
- 验证：GUI 静态测试 34 passed，Python 编译、内嵌 JavaScript `node --check` 与 `git diff --check` 通过。桌面 Chrome mock 截图已检查；窄屏截图仍受本地 headless Chrome 实际 CSS viewport 与截图窗口尺寸不一致影响，需真实浏览器复核。
- 部署：已部署 PC4 原导航网页，仅替换 `h1_2_nav_web_gui.py` 并构建 `h1_2_nav_tools`；备份目录 `/home/unitree/h1_nav2_ros2_ws/backup_onnx_ui_20261116/`。远端源码 SHA256 `e51f0841f8793fb09365c619463fcc4b3faa005eea4420fdbef090a0f0f5a454`。服务 active、8765 HTTP 200，HTML 有三段 ONNX 状态。窄屏仍需真实浏览器复核。

## 2026-11-16 未授权启动恢复入口

- 目标：网页等待 Policy status 时，提供可理解的重试与安全取消路径，避免卡在等待态。
- 范围：增加显式刷新状态；增加“取消未授权启动”，仅在 bridge `Passive`、`authorized=false`、`publisher_present=false`、无转场且路线 idle 时允许停止可验证的导航/bridge 专属进程组。
- 非目标：重置已授权 Policy、发送 Passive/ArmHold 或站立动作、释放/撤权、发送路线目标。
- 验收：未知状态、服务不可用、路线非 idle、publisher 存在、授权或进程标记不匹配时拒绝取消；满足全部门控时才停止启动栈并清理其启动标记。
- 状态：已实现并部署。GUI 测试 34 passed，Python/JS 语法检查与 diff 检查通过；PC4 `h1_2_nav_tools` build 成功，Web service active、8765 HTTP 200，页面含重试/取消入口。旧运行栈在 Web service 重启时由 systemd cgroup 清理，重启前 bridge 是 Passive、未授权且无 publisher；现在 Nav2/Policy 均停止、旧 PID 标记已清理。取消接口自身未在活动硬件栈上触发，因此目标现场的取消流程尚未端到端验证。

## 2026-11-16 Policy 启动向导 UX 调整

- 目标：让 ONNX 后端的选择与启动有明确区别，并在单个浮窗内逐步完成在线/离线导航栈启动、输出授权和站立准备。
- 范围：在原网页增加“启动 ONNX 模式”入口；浮窗按当前导航/Policy 状态呈现下一步；保留现有服务端现场确认与桥接状态门控。Policy 启动强制关闭自动巡航，进入 ArmHold 后才可从原网页单独开始路线。
- 非目标：改变 Policy/LowCmd 协议、放宽现场准入、自动确认动作、自动开始路线或改动手动脉冲测试网页。
- 步骤：1) 已完成：改造原网页入口、阶段浮窗和状态呈现；2) 已完成：补充 Policy 启动参数约束并通过定向测试；3) 已完成：部署 PC4 网页并核验页面更新。
- 验收：仅选中 ONNX 不启动任何进程；显式按“启动 ONNX 模式”才启动 Policy 导航栈；用户可在浮窗内辨认尚未启动、Passive 待授权、FixStand、ArmHoldStand、ArmHold及异常/等待状态；服务端仍逐步拒绝跳步；任何Policy启动都不会自动发路线目标。
- 验证边界：本地测试验证网页交互结构、导航参数和现有门控；不通过自动化测试触发真机控制服务。
- 2026-11-16 部署完成：PC4 `192.168.123.164` 在线；部署前 `h1-2-web.service` active，无 Nav2/Policy/route runner。先备份远端 GUI 到 `/home/unitree/h1_nav2_ros2_ws/backup_onnx_ui_20261116/`，仅复制 GUI 文件，远端 `colcon build --base-paths src --packages-select h1_2_nav_tools --symlink-install --parallel-workers 2` 成功。重启服务后 active，8765/8766 监听，首页 HTTP 200 并包含 `policy_overview_stack/control/route`。远端新 GUI SHA256 `e51f0841f8793fb09365c619463fcc4b3faa005eea4420fdbef090a0f0f5a454`；旧文件 SHA256 `fd7274eda3b05d061b7c35b29419f41e31de1eecfdc6993187f982abd214e9bb`。未启动导航/Policy、未授权、未发路线目标或运动命令。

## 目标

在现有 `h1_2_nav_web_gui.py` 加入 ONNX Policy bridge 的分阶段准备控制，使操作人员可在网页查看状态并逐步完成输出授权、FixStand、ArmHoldStand 和 ArmHold，然后使用原网页路线测试。

## 范围与边界

- 只控制当前已运行的在线/导航 Policy bridge；不新增导航后端、不自动启动或重启机器人栈。
- Policy 模式的 bridge、Nav2 与 route runner 使用 Fast DDS，网页现有 ROS 节点可能使用 Cyclone DDS；Policy 路线状态/启动/停止、在线 TF readiness 和点位保存需使用隔离的 Fast DDS 客户端。Unitree 数据路径保持不变。
- 每次服务调用前由服务器检查当前 bridge 状态、预期前置模式、导航实例/后端和浏览器心跳；禁止跳步和重复授权。
- 页面要求操作员确认 Debug、独占控制、其他 LowCmd writer 已核实/交接、现场支撑与急停准备；明确这些人工确认不构成 DDS writer 自动核验。
- 不自动巡航、不发送速度目标、不做 RELEASE/REVOKE；真机动作只在另行完成现场核验后由操作员点击单步命令。

## 步骤

1. 已完成：增加隔离 Fast DDS ROS 客户端，为 bridge、在线位姿、点位及 route runner 提供请求/响应；Unitree ROS 路径不变。
2. 已完成：原网页加入分阶段、服务端校验的准备 API 和单步控制面板；路线仍需 bridge readiness 与定位就绪。
3. 已完成：增加 API 门控与 Fast DDS 客户端测试；定向测试、隔离构建和 bridge CTest 通过。
4. 已完成：PC4 SSH 恢复后，仅部署 `h1_2_nav_tools` 并重启原 Web 服务。当前页面已更新；未启动/授权 Policy 或发送路线目标。

## 验收标准

- 未连接 Policy、非 Policy 导航、状态未知/过期、错误模式、转场未完成、心跳失效或缺少确认时，API 拒绝调用 bridge 服务。
- 每个状态转换只能按 `Passive → FixStand → ArmHoldStand → ArmHold` 单步请求；不能从网页跳过中间态。
- Policy 模式网页可通过 Fast DDS 完成 route status、在线 TF readiness、camera_init 点位保存及路线启动/停止调用。
- 输出授权前显示 LowCmd writer 需独立确认；代码不把勾选框当成 writer 身份证明。
- 单测覆盖成功顺序及所有拒绝门控；静态/编译检查通过。
- 本地实现与软件验证完成，并已部署到 PC4；未启动/授权真机控制或发送导航目标。

## 风险与依赖

- bridge status 不提供 DDS writer 身份；控制权仍需现场监测器/管理员独立核验。
- 网页服务当前面向既有导航使用方式；新增控制命令需依赖单步确认、服务端门控和现有浏览器心跳，不能视为物理急停或电机安全证明。
- 部署虽已完成，当前 LowCmd writer 来源/控制权仍未刷新确认；使用网页准备步骤前需现场核实。

## 验证与交付

- `python3 -m pytest -q test/unit/test_web_task_profiles.py test/unit/test_policy_web_client.py test/static/test_h1_2_nav_web_gui.py`：72 passed。
- ROS Humble overlay 下隔离构建 `h1_2_nav_tools h1_2_policy_bridge` 成功；bridge CTest 7/7 passed。
- Python `py_compile`、内嵌 JavaScript `node --check`、`git diff --check` 通过。
- PC4 备份目录：`/home/unitree/h1_nav2_ros2_ws/backup_policy_web_activation_20261114T172108`；仅替换 Web GUI 与新增 Fast DDS client 两个 Python 文件，远端主工作区包构建成功。
- PC4 `h1-2-web.service` 重启后为 active，`http://127.0.0.1:8765/` 返回 HTTP 200；HTML 包含 `ONNX Policy 准备` 面板。空确认 POST 返回“请先完成页面上的现场准入确认”，在此门控处拒绝。
- 部署时进程检查只见 Web 服务，没有 Policy bridge、Nav2 或 route runner。未调用 Policy/FSM/route ROS 控制服务，没有授权输出、启动路线或执行运动。
- PC4 当前源码 SHA256：GUI `fd7274eda3b05d061b7c35b29419f41e31de1eecfdc6993187f982abd214e9bb`；Fast DDS client `f888f9bb1f8ad069fb791eeba0d8404973166e079812d8f1eba76db73ab31af6`。

## 计划调整

- 2026-11-14：原网页目前没有输出授权或站立准备操作；用户要求把启动 ONNX Policy 控制的手段放到原网页。本计划增加网页内人工分阶段入口，保持所有现场授权、训练保护和路线 readiness 门控；实现过程不执行真机控制。
- 2026-11-14：发现 Policy 栈与原网页 ROS 客户端分别使用 Fast DDS/Cyclone DDS。为满足网页端到端导航测试目标，扩展为 Policy 模式专用 Fast DDS 请求客户端；网页原 Cyclone ROS 节点和 Unitree 路径保持不变。`authorize_output` 已强制校验 bridge 启动参数，因此不扩展 bridge 状态格式，部署仅需更新导航工具包。
- 2026-11-14：PC4 SSH 恢复后完成主工作区部署并重启 Web 服务。部署前只读检查显示没有 Policy/Nav2/route runner；没有调用控制服务。LowCmd writer 身份尚未在本次部署中核验，真机测试仍需单独准入。
