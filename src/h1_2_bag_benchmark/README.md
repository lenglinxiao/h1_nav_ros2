# H1-2 建图与定位 Bag 回归测试

这个包只测试 FAST-LIO 建图和 ICP/GICP 定位，不启动 Livox 驱动、Nav2 或 Unitree 运动桥。

## 1. 构建

```bash
cd /home/llx/h1_nav2_ros2_ws
colcon build --packages-select h1_2_slam_bringup h1_2_bag_benchmark --symlink-install
source install/setup.bash
```

## 2. 录制标准实机数据

先正常启动 Livox 和 FAST-LIO。开始录包后，再设置初始位姿并走完标准路线：

```bash
ros2 launch h1_2_bag_benchmark record_slam_bag.launch.py \
  output:=/home/llx/h1_nav2_ros2_ws/datasets/bags/lab_loop_01
```

录制 `/livox/lidar`、`/livox/imu`、`/slam_odom`、定位状态、TF和初始位姿。结束时按 `Ctrl+C`，然后检查：

```bash
ros2 bag info /home/llx/h1_nav2_ros2_ws/datasets/bags/lab_loop_01
```

## 3. 重放建图

```bash
ros2 launch h1_2_bag_benchmark replay_mapping_bag.launch.py \
  bag:=/home/llx/h1_nav2_ros2_ws/datasets/bags/lab_loop_01 \
  output_dir:=/home/llx/h1_nav2_ros2_ws/datasets/results/lab_loop_01_mapping
```

输出：

- `replayed_map.pcd`：重新生成的FAST-LIO点云地图；
- `trajectory.csv`：`camera_init`坐标系中的轨迹；
- `summary.json`：轨迹长度、闭环误差、采样率、最大位姿步跳等指标。

第一次运行的 `trajectory.csv` 可以作为回归基线。后续运行增加：

```bash
reference_csv:=/absolute/path/to/baseline/trajectory.csv
```

没有独立真值时，这只表示重复性和版本退化，不表示绝对建图精度。

比较新旧两份已经处于同一坐标系的PCD地图：

```bash
ros2 run h1_2_bag_benchmark pcd_map_compare \
  --reference /absolute/path/to/baseline_map.pcd \
  --actual /home/llx/h1_nav2_ros2_ws/datasets/results/lab_loop_01_mapping/replayed_map.pcd \
  --output /home/llx/h1_nav2_ros2_ws/datasets/results/lab_loop_01_mapping/map_comparison.json \
  --overlap-distance-m 0.20
```

输出双向最近邻RMSE、P95、最大误差和重叠率。工具故意不做自动配准：若地图坐标系发生错误，不应该通过后处理悄悄掩盖。

## 4. 重放定位

全局GICP：

```bash
ros2 launch h1_2_bag_benchmark replay_localization_bag.launch.py \
  bag:=/home/llx/h1_nav2_ros2_ws/datasets/bags/lab_loop_01 \
  pcd_map:=/home/llx/h1_nav2_ros2_ws/maps/h1_2_lab.pcd \
  backend:=global_gicp \
  output_dir:=/home/llx/h1_nav2_ros2_ws/datasets/results/lab_loop_01_global_gicp
```

旧ICP后端使用 `backend:=icp`。ICP需要bag中包含有效的 `/initialpose`；因此录制时必须先启动record，再设置初始位姿。

定位报告增加以下指标：

- 首次收敛时间；
- 收敛样本占比；
- degraded占比；
- score均值、P95和最大值；
- `map -> camera_init -> body` 全局轨迹；
- 相对参考轨迹的位置和航向误差。

## 5. 设置自动失败阈值

默认负数表示只采集、不设门槛。积累基线后可设置：

```bash
max_pose_step_m:=0.30 \
max_position_regression_p95_m:=0.15 \
max_degraded_ratio:=0.05 \
min_converged_ratio:=0.95
```

最终结果查看 `summary.json` 的 `passed` 和 `failures` 字段。建议先以 `rate:=1.0` 回放；只有确认机器能实时处理Livox数据后再提高回放倍率。

回放结束后默认再等待 `drain_delay_sec:=5.0`，让FAST-LIO处理完队列并保存PCD。点云计算落后时应增大这个值，不要设为0。
