# 人工验收

后续实机侧验收清单放在这里。

人工验收可以覆盖地图保存、初始位姿设置、记忆点记录、路线循环、动态障碍物处理，以及停止/恢复行为。

## ICP 离线定位验收

1. 在线模式启动后完整走一遍演示区域：
   `./run_ai_slam_nav.sh mode=online`
2. 在同一次在线建图中保存 2D 栅格和 3D PCD：
   `./run_ai_slam_nav.sh mode=save_map map_prefix=maps/h1_2_lab`
   `./run_ai_slam_nav.sh mode=save_pcd_map pcd_prefix=maps/h1_2_lab`
3. 确认文件同时存在：
   `maps/h1_2_lab.yaml`
   `maps/h1_2_lab.pgm`
   `maps/h1_2_lab.pcd`
4. 启动 ICP 离线导航：
   `./run_ai_slam_nav.sh mode=navigation map=$PWD/maps/h1_2_lab.yaml localization=icp pcd=$PWD/maps/h1_2_lab.pcd`
5. 在 RViz 中确认当前点云和 PCD 地图大致重合，TF 树存在 `map -> camera_init -> body`。
6. 查看 `/icp_localization/status`，确认 `initialized=true`、`converged=true`，且 score 不高于当前 `max_icp_score`。
7. 发送一个短距离 `Nav2 Goal`，确认 Nav2 规划、局部避障和 `/cmd_vel_nav` 输出正常。
8. 保存 START/A/B/C 记忆点后启动路线，确认 ICP 未就绪时不会自动巡航，ICP 就绪后才开始执行。
