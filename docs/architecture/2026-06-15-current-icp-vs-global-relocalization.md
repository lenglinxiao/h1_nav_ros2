# 当前 ICP 定位思路与全局重定位方案对比

## 背景

当前 H1-2 离线模式使用先验 3D PCD 地图做 3D ICP 定位，并继续使用 Nav2 基于 2D 栅格地图规划。用户参考 `/home/llx/Lidar_nav2_ws` 中 “3.5 全局重定位：三种方案”，希望明确当前实现属于哪一类，以及后续是否需要切换到 KISS-Matcher、small_gicp 或 NDT/NDP。

## 当前实现定位

当前实现是 `h1_2_icp_localization`，核心节点为：

- `src/h1_2_icp_localization/src/icp_localization_node.cpp`
- `src/h1_2_icp_localization/src/icp_localizer.cpp`
- launch 入口：`src/h1_2_slam_bringup/launch/h1_2_icp_localization.launch.py`
- 离线集成入口：`src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py`

当前算法更接近 “纯 ICP / small_gicp 已知初值重定位” 这一类，但和参考项目里描述的 “ICP 只在开机一刻进行重定位，后续不维护 map -> odom” 不完全一样。

本项目当前 ICP 是：

1. 通过 `START` 或指定记忆点提供初始位姿。
2. 在初始位姿附近做多候选粗配准。
3. 粗配准成功后做精配准。
4. 计算并发布 `map -> camera_init`。
5. 后续定时继续 ICP 跟踪，持续维护 `map -> camera_init`。

所以它不是一次性 ICP，而是“初值辅助的连续 ICP 跟踪”。

## 数据流

离线 ICP 模式下，TF 和数据流应为：

```text
map
 └── camera_init
      └── body
           └── livox_frame / pelvis ...
```

输入：

- 先验 3D 地图：`maps/h1_2_lab.pcd`
- 当前 3D 点云：`/cloud_registered_body`
- FAST-LIO 里程计：`/slam_odom`
- 初始位姿：`maps/memory_points.json` 中的 `START`

输出：

- ICP 状态：`/icp_localization/status`
- 全局定位 TF：`map -> camera_init`
- Nav2 规划仍使用 2D 栅格：`maps/h1_2_lab.yaml`

## 算法步骤

### 1. 地图加载

节点加载 PCD 地图后，会做两级体素降采样：

- rough map：较粗分辨率，默认 `0.5m`
- refine map：较细分辨率，默认 `0.2m`

如果 PCD 不带 normal，会对地图点云估计 normal。当前 ICP 实际使用带 normal 的点类型做匹配。

### 2. 初始位姿来源

`auto_initial_pose` 会读取 `START` 记忆点，并调用：

```text
icp_relocalize
```

这个服务把 `START` 作为初始猜测传给 ICP。

### 3. 多候选粗搜索

为了避免初值稍微不准导致 ICP 直接收敛失败，当前实现会在初值附近展开候选：

- XY 平移搜索：默认 `xy_offset=1.0`，`xy_steps=3`
- yaw 搜索：默认 `yaw_offset=1`，`yaw_resolution=0.5`

也就是在初始位姿附近尝试多个 `(x, y, yaw)` 候选，先跑 rough ICP，选 score 最好的候选。

### 4. 精配准

粗配准得到较好的初始结果后，再用 refine 分辨率做精配准。

判断条件：

- ICP 必须 `hasConverged`
- fitness score 必须小于阈值，默认 `0.15`

现场曾看到：

```text
converged: true
score: 0.006
```

这说明匹配本身质量很好。

### 5. 发布全局定位

ICP 得到的是当前点云在 `map` 下的位姿。节点结合当前 FAST-LIO odom 位姿，计算：

```text
map_to_odom = icp_aligned_pose * odom_pose.inverse()
```

在本项目中 `odom_frame` 是 `camera_init`，因此最终发布：

```text
map -> camera_init
```

### 6. 连续维护

初始化成功后，节点每秒按当前 `map_to_odom * odom_pose` 作为初值继续对当前点云做 ICP，并更新 `map -> camera_init`。

这比“一次性 ICP”更强，因为它会持续修正 FAST-LIO 漂移；但它仍依赖局部初值，不具备 KISS-Matcher 这种无初值全局搜索能力。

## 与参考三种方案对比

| 方案 | 是否需要初值 | 是否连续维护 map -> odom | 抗初值错误能力 | 当前项目状态 |
|---|---:|---:|---:|---|
| KISS-Matcher + small_gicp | 不强依赖 | 是 | 强 | 未接入 |
| 纯 small_gicp | 需要大致初值 | 是 | 中 | 未接入 |
| 传统一次性 ICP | 需要大致初值 | 否 | 弱 | 当前不是这一种 |
| 当前 H1-2 ICP | 需要 START 附近初值 | 是，维护 `map -> camera_init` | 中偏弱 | 已接入 |

## 当前方案适用边界

适合：

- 机器人离 `START` 或指定初始点不远。
- 离线地图与 PCD 来自同一次建图坐标系。
- 场景结构稳定，3D 点云和 PCD 有足够几何约束。
- 需要先把离线 Nav2 流程跑通。

不适合：

- 开机位置完全未知。
- `START` 给错很多。
- 大范围重定位。
- 场景强重复、走廊高度相似、局部 ICP 容易落入错误极小值。

## 推荐路线

短期继续使用当前 ICP，不建议现在切换 NDT/NDP 或 KISS-Matcher。当前现场问题主要集中在 frame、TF 时间戳和路线点坐标解释，不是 ICP 匹配质量本身。

中期建议把定位 backend 抽象成：

```text
localization=icp
localization=small_gicp
localization=kiss_gicp
localization=ndt
```

其中：

- `icp`：当前默认，适合 START 初值可靠的离线演示。
- `small_gicp`：作为当前 ICP 的性能和鲁棒性升级。
- `kiss_gicp`：作为无初值或初值很差时的全局重定位。
- `ndt`：可作为结构化室内场景的备选，但需要单独调 voxel/grid 参数。

## 注意事项

ICP 离线模式下正式导航坐标系必须是 `map`，不能因为 RViz 在 `camera_init` 下显示正常就把 Nav2 全局坐标改成 `camera_init`。

`camera_init` 只是 FAST-LIO 局部里程计坐标，`map` 才是离线栅格地图、PCD 地图、Nav2 全局规划和记忆点应该对齐的坐标。
