# 离线模式 3D ICP 定位架构方案

## 背景

当前 H1-2 工作空间有两条主链路：

- 在线模式：`mode=online` 启动 `h1_2_nav2_camera_init_navigation.launch.py`，使用 FAST-LIO2 的 `camera_init -> body` 里程计和在线生成的 2D 栅格给 Nav2 规划。
- 离线模式：`mode=navigation` 启动 `h1_2_nav2_navigation.launch.py`，加载固定 2D 地图，并通过 AMCL 发布 `map -> camera_init`，Nav2 继续负责全局规划、局部避障和速度输出。

由于当前雷达是 3D LiDAR，AMCL 基于 2D LaserScan 与 2D 栅格的定位方式不适合作为长期离线定位主链路。参考 `Dailywatero/WK` 的设计，离线模式应改为用当前 3D 点云和离线 3D PCD 地图做 ICP 配准，规划仍交给 Nav2。

## 设计目标

- 在线模式不受影响：`mode=online`、在线建图、在线 Nav2、地图保存流程保持独立。
- 离线模式替换定位后端：离线固定地图导航不再依赖 AMCL，改由 3D ICP 发布 `map -> camera_init`。
- Nav2 继续负责规划：保留 Nav2 map server、planner、controller、costmap、recovery、goal/action、记忆点巡航。
- 复用现有 frame 合同：保持 `map -> camera_init -> body`，减少对 Nav2、记忆点和运动桥接的扰动。
- 支持手动或自动重定位：可由 RViz 初始位姿、`START` 记忆点或脚本参数触发 ICP 初始配准。
- 可回退：保留 AMCL 作为调试/应急后端，但默认推荐离线使用 ICP。

## WK 可参考点

WK 中的离线定位链路核心是：

- `localizer_node` 内部运行 FAST-LIO 局部里程计，持续发布局部 `local -> body`。
- `/slam_reloc` 服务接收 PCD 地图路径和初始位姿，触发当前点云到 PCD 地图的重定位。
- `IcpLocalizer` 使用粗/精两级 voxel 分辨率：
  - 粗配准使用较低点云密度快速搜索。
  - 精配准使用更细点云继续优化。
- 初始重定位可用多候选位姿搜索，在初始点附近按 XY 和 yaw 生成候选，选择 fitness score 最好的结果。
- 定位成功后计算并持续更新全局到局部里程计的 offset，本质上就是发布 `map -> local`。

迁移到本仓库时，不建议直接照搬 ROS1 launch 和导航栈。应保留本仓库的 ROS2/Nav2 结构，只迁移 ICP 定位思想和必要算法。

## 推荐方案

推荐采用“新增 ROS2 ICP 定位包 + 离线 launch 后端切换”的方式。

### 核心链路

```text
离线 3D PCD 地图
        |
        v
h1_2_icp_localization
        ^
        |
/cloud_registered_body + /slam_odom
        |
FAST-LIO2: camera_init -> body

h1_2_icp_localization 发布:
  map -> camera_init

Nav2 继续使用:
  map_server 加载 h1_2_lab.yaml/pgm
  global planner 使用 map
  local costmap 使用 /scan 动态避障
  controller 输出 /cmd_vel
```

### Frame 合同

- `map`：离线固定地图坐标系，2D 栅格和 3D PCD 必须来自同一次建图或同一坐标基准。
- `camera_init`：FAST-LIO2 离线运行时的局部里程计坐标系。
- `body`：机器人机体坐标系。
- ICP 节点只负责发布 `map -> camera_init`。
- FAST-LIO2 继续发布或提供 `camera_init -> body` 和 `/slam_odom`。
- Nav2、记忆点、巡航目标继续使用 `map`。

### 离线启动行为

离线入口扩展为：

```bash
./run_ai_slam_nav.sh mode=navigation \
  map=$PWD/maps/h1_2_lab.yaml \
  pcd=$PWD/maps/h1_2_lab.pcd \
  localization=icp
```

默认建议：

- `mode=online` 默认不变，不加载 ICP。
- `mode=navigation` 默认使用 `localization=icp`。
- `localization=amcl` 保留为回退选项。

### ICP 节点职责

新增 `h1_2_icp_localization`，建议先用 C++/PCL 实现：

- 订阅 `/cloud_registered_body` 或经过 frame adapter 后的 body 点云。
- 订阅 `/slam_odom`，读取当前 `camera_init -> body` 里程计。
- 加载离线 PCD 地图，生成 rough/refine 两份体素地图并计算法向量。
- 提供 `/icp_relocalize` 服务：
  - 输入：PCD 路径、初始 x/y/z/roll/pitch/yaw、搜索范围参数。
  - 输出：是否收敛、fitness score、最终位姿。
- 运行低频跟踪 ICP：
  - 初次重定位使用多候选搜索。
  - 后续用上一帧 `map -> camera_init` 与当前里程计组合得到初始猜测。
- 发布 `map -> camera_init` TF。
- 发布状态话题 `/icp_localization/status`，包含 `initialized`、`converged`、`score`、`last_success_time`。

### 质量门槛

建议初始参数：

- `rough_resolution`: `0.5`
- `refine_resolution`: `0.2`
- `rough_iter`: `10`
- `refine_iter`: `5`
- `score_threshold`: `0.15` 起步，实测后按场地调整。
- 初始搜索：
  - `xy_offset`: `1.0`
  - `xy_steps`: `[-3, -2, -1, 0, 1, 2, 3]`
  - `yaw_resolution`: `0.5`
  - `yaw_offset`: 先从 `1` 或 `2` 调试，定位稳定后缩小。

失败处理：

- 初始 ICP 未收敛时，不发布新的 `map -> camera_init`。
- 跟踪 ICP 单次失败时，短时间保留上一帧 TF，但状态置为 degraded。
- 连续失败超过阈值时，阻止自动巡航启动或暂停路线执行。
- fitness score 超阈值时视为失败，不能把错误位姿喂给 Nav2。

## 备选方案

### 方案 A：新增独立 ICP 定位包

这是推荐方案。

优点：

- 在线模式完全不动。
- 与 Nav2、记忆点、运动桥接边界清楚。
- 可以保留 AMCL 作为回退后端。
- 后续可单独测试 ICP 收敛、TF 发布和状态话题。

缺点：

- 需要新增 C++ ROS2 包和服务接口。
- 需要把 WK 的 ICP 思路移植到 ROS2。

### 方案 B：直接改 FAST_LIO_ROS2，内置离线定位

优点：

- 点云、IMU、里程计都在一个包内，数据路径短。
- 与 WK 的 `localizer_node` 更接近。

缺点：

- 会污染上游 FAST-LIO2 包，后续升级和排错成本高。
- 容易影响在线模式。
- 功能边界不如独立包清晰。

### 方案 C：继续 AMCL，仅调参和改 2D 投影

优点：

- 改动最少。
- 现有离线流程能继续跑。

缺点：

- 没有解决 3D LiDAR 在 3D 场景中定位信息被压扁的问题。
- 长走廊、多层结构、几何相似区域仍容易退化。
- 不符合当前“3D 雷达离线定位”的目标。

## 实施计划

### 阶段 1：文档和入口隔离

- 新增本架构文档。
- 新增项目级规则：技术架构需求必须沉淀到 Markdown。
- 不修改在线 launch。

### 阶段 2：3D 地图资产

- 明确 `maps/h1_2_lab.yaml/pgm` 与 `maps/h1_2_lab.pcd` 的生成关系。
- 为在线模式增加可选的 3D PCD 保存流程，或复用 FAST-LIO2 `/map_save` 服务。
- 文档要求 2D 栅格和 3D PCD 必须来自同一建图坐标系。

### 阶段 3：ICP 定位节点

- 新增 `h1_2_icp_localization` C++ 包。
- 实现 PCD 加载、体素滤波、法向量计算、粗/精 ICP。
- 实现多候选初始重定位。
- 发布 `map -> camera_init`。
- 提供状态话题和重定位服务。

### 阶段 4：离线 Nav2 launch 集成

- `h1_2_nav2_navigation.launch.py` 增加 `localization_backend` 参数。
- `localization_backend:=icp` 时不启动 AMCL，启动 ICP 节点。
- `localization_backend:=amcl` 时保留旧链路。
- `run_ai_slam_nav.sh` 增加 `pcd=` 与 `localization=` 参数。

### 阶段 5：记忆点和巡航联动

- `auto_initial_pose` 的 AMCL 等待逻辑扩展为可等待 ICP 状态。
- 自动巡航启动前要求：
  - `map -> camera_init` 可用。
  - ICP 状态 initialized。
  - 最近一次 score 低于阈值。

### 阶段 6：验证

离线静态验证：

- `mode=online` 不引用 ICP 包、PCD 参数或 ICP launch。
- `mode=navigation localization=icp` 不启动 AMCL。
- `mode=navigation localization=amcl` 保持旧行为。
- ICP launch 参数包含 map、pcd、frames、score 阈值。

实机验证：

- 在线建图并保存 2D 栅格和 3D PCD。
- 离线启动 ICP 定位，手动给初始位姿。
- RViz 中确认 PCD 地图、当前点云、机器人模型配准。
- 下发短距离 Nav2 goal，观察局部避障和速度输出。
- 保存 START/A/B/C 记忆点后执行循环巡航。

## 不做的事

- 不把在线模式切到固定 PCD 地图定位。
- 不替换 Nav2 planner/controller。
- 不把 AMCL 立即删除。
- 不把 3D ICP 的结果直接发布成 `/cmd_vel`。
- 不让 ICP 失败时继续自动巡航。

## 结论

当前最合适的路线是：在线模式继续作为建图、调试和临时导航入口；离线模式新增 3D ICP 定位后端，用 PCD 地图和实时 3D 点云发布 `map -> camera_init`；Nav2 继续基于固定 2D 栅格做规划和避障。这样既符合 3D LiDAR 的定位能力，也能最大限度保留现有在线功能和 Nav2 工作流。
