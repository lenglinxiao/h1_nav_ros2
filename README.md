# h1_2_nav2_ros2_ws

这个工程把当前 `H1-2Nav2D` 里和雷达、SLAM、导航/路径示教直接相关的部分整理成了 ROS2 工作空间。当前上位机是 ROS2 Foxy；如果换 Humble，把下面命令里的 `foxy` 改成 `humble` 即可。

## 包内容

- `livox_ros_driver2`: 原项目里的 Livox ROS2 驱动和 Livox `CustomMsg` 消息定义；上位机通过 DDS 接收机器人话题时不需要本机直连雷达驱动。
- `FAST_LIO_ROS2` / `fast_lio2`: ROS2 版 FAST-LIO2。ROS 包名、可执行文件和启动入口都按 FAST-LIO2 命名。
- `h1_2_slam_bringup`: MID360 雷达启动、FAST-LIO2 建图/里程计、以及保留的 `slam_toolbox` 2D 备用启动入口。
- `path_to_waypoint`: 订阅 `/teach_path` 和 `/slam_odom`，按原项目逻辑做路径采样、曲率前瞻，发布 `/way_point`。
- `play_teaching_path`: 从文本文件读取示教路径，循环发布 `/teach_path`。
- `record_teaching_path`: 订阅 `/slam_odom`，通过 TF 转到 `map` 后按距离间隔录制路径。
- `local2map_odom`: 把里程计位姿转换到 `map`，发布 `/odom_in_map`。
- `velocity_smoother_ema`: 对 `/cmd_vel` 做指数滑动平均，默认发布 `/cmd_vel_smooth`。
- `costmap_clear_on_initialpose`: 收到 `/initialpose` 后调用 Nav2 的 global/local costmap 清图服务。

## 编译

先安装 ROS2 侧依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-foxy-navigation2 \
  ros-foxy-nav2-bringup \
  ros-foxy-slam-toolbox \
  ros-foxy-pointcloud-to-laserscan \
  ros-foxy-pcl-ros \
  ros-foxy-pcl-conversions \
  ros-foxy-rclcpp-components \
  libpcl-dev \
  libapr1-dev
```

如果上位机直接连接 MID360 并启动 `livox_mid360.launch.py`，Livox 驱动还需要 Livox-SDK2，并确认这些文件存在。仅通过 DDS 订阅机器人已经发布的 `/livox/lidar` 和 `/livox/imu` 时，可以不装 SDK2；工程会只编译 Livox 消息接口。

```bash
ls /usr/local/lib/liblivox_lidar_sdk_shared.so
ls /usr/local/include/livox_lidar_api.h
ls /usr/local/include/livox_lidar_def.h
```

然后编译工作空间：

```bash
cd ~/h1_2_nav2_ros2_ws
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

如果只想先编译我迁移的导航辅助节点，不编译雷达驱动：

```bash
colcon build --symlink-install --packages-select h1_2_nav_tools h1_2_slam_bringup
```

## 雷达

启动 MID360：

```bash
ros2 launch h1_2_slam_bringup livox_mid360.launch.py
```

默认输出：

- `/livox/lidar` (`livox_ros_driver2/msg/CustomMsg`)
- `/livox/imu` (`sensor_msgs/Imu`)
- frame: `livox_frame`

如果雷达配置文件需要改 IP/广播码，编辑：

```bash
~/h1_2_nav2_ros2_ws/src/livox_ros_driver2/config/MID360_config.json
```

如果你要输出标准 PointCloud2 给其他 2D 包使用，启动时改：

```bash
ros2 launch h1_2_slam_bringup livox_mid360.launch.py xfer_format:=0
```

## FAST-LIO2 建图/里程计

默认上位机不启动本机 Livox 驱动，只订阅 DDS 中已有的 `/livox/lidar` 和 `/livox/imu`，然后运行 Fast-LIO2：

```bash
ros2 launch h1_2_slam_bringup fastlio2_mid360_mapping.launch.py
```

如果上位机要直接连 MID360，再打开驱动：

```bash
ros2 launch h1_2_slam_bringup fastlio2_mid360_mapping.launch.py start_livox_driver:=true
```

主要输出：

- `/slam_odom`：由 FAST-LIO2 的 `/Odometry` remap 而来
- `/cloud_registered`
- `/cloud_registered_body`
- `/Laser_map`
- `/path`
- TF: `camera_init -> body`

FAST-LIO2 参数文件：

```bash
~/h1_2_nav2_ros2_ws/src/FAST_LIO_ROS2/config/mid360.yaml
```

关键参数通常需要按机器人实物调整：

- `common.lid_topic`: 默认 `/livox/lidar`
- `common.imu_topic`: 默认 `/livox/imu`
- `mapping.extrinsic_T`
- `mapping.extrinsic_R`
- `mapping.extrinsic_est_en`
- `common.time_sync_en`

## FAST-LIO2 + 示教导航辅助

```bash
ros2 launch h1_2_slam_bringup fastlio2_mid360_navigation.launch.py \
  path_file:=/home/$USER/h1_2_nav2_ros2_ws/src/h1_2_nav_tools/path/teaching_paths.txt
```

这个启动文件把示教路径 frame 设为 `camera_init`，并让 `path_to_waypoint` 订阅 `/slam_odom`。如果你后续有 `map -> camera_init` 或自己的定位 frame，可以通过 `target_frame:=map` 切回 `map`。

## 2D SLAM 备用链路

启动雷达、点云转激光、`slam_toolbox`：

```bash
ros2 launch h1_2_slam_bringup lidar_slam_mapping.launch.py
```

主要链路：

- `/livox/lidar` -> `pointcloud_to_laserscan` -> `/scan`
- `/scan` -> `slam_toolbox` -> `/map` 和 `map->odom`

雷达安装位姿默认是 `base_link -> livox_frame: x=0 y=0 z=0.25`。如果实际安装位置不同，启动时传参：

```bash
ros2 launch h1_2_slam_bringup lidar_slam_mapping.launch.py \
  livox_x:=0.10 livox_y:=0.0 livox_z:=0.32 livox_yaw:=0.0
```

## 录制示教路径

```bash
ros2 launch h1_2_nav_tools record_teaching_path.launch.py \
  file_path:=/home/$USER/h1_2_nav2_ros2_ws/src/h1_2_nav_tools/path/teaching_paths.txt \
  odom_topic:=/slam_odom
```

控制台命令：

- `s`: 开始录制
- `e`: 结束当前路径
- `w`: 写入文件
- `q`: 退出

## 播放路径并发布前瞻点

```bash
ros2 launch h1_2_nav_tools teaching_navigation.launch.py \
  path_file:=/home/$USER/h1_2_nav2_ros2_ws/src/h1_2_nav_tools/path/teaching_paths.txt \
  odom_topic:=/slam_odom
```

如果要连同雷达和 SLAM 一起启动：

```bash
ros2 launch h1_2_slam_bringup lidar_slam_navigation.launch.py \
  path_file:=/home/$USER/h1_2_nav2_ros2_ws/src/h1_2_nav_tools/path/teaching_paths.txt
```

主要话题：

- 输入：`/slam_odom` (`nav_msgs/Odometry`)
- 输入：`/teach_path` (`nav_msgs/Path`)
- 输出：`/way_point` (`geometry_msgs/PointStamped`)
- 可视化输出：`/path_with_stairs` (`nav_msgs/Path`)

## 接机器人控制

这个工程保留了原项目的 `/way_point` 输出。远端机器人如果已有跟踪 `/way_point` 的控制节点，可以直接接上。如果机器人使用 Nav2 的标准目标接口，则需要另写一个 `/way_point` 到 `NavigateToPose` action 的桥接节点，或者把现有底盘控制节点改为订阅 `/way_point`。

`velocity_smoother_ema` 默认把 `/cmd_vel` 平滑到 `/cmd_vel_smooth`。如果机器人底盘只听 `/cmd_vel`，请在启动时改参数：

```bash
ros2 run h1_2_nav_tools velocity_smoother_ema --ros-args \
  -p raw_cmd_topic:=/cmd_vel_raw \
  -p cmd_topic:=/cmd_vel
```

## 从原项目迁移时注意

原项目里的 `move_base`、TEB、`costmap_2d` 配置属于 ROS1 导航栈，不能直接在 ROS2 Humble 编译运行。ROS2 侧应启动 Nav2，然后使用本工程的示教路径、前瞻点和辅助节点。

原项目里的 `fastlio2` 是 ROS1/catkin 版，包含大量 `ros::NodeHandle`、ROS1 service、ROS1 Livox 自定义消息接口，不能通过复制目录在 ROS2 里直接编译。这个工程现在接入的是 ROS2 版 FAST-LIO2：`FAST_LIO_ROS2` 包名安装后是 `fast_lio2`，可执行文件是 `fastlio2_mapping`。
