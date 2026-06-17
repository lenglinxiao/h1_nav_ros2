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

## Nav2 在线建图和导航

默认推荐保留原来的 `camera_init` SLAM 体系：FAST-LIO2 继续发布
`camera_init -> body`，`/cloud_registered` 和最终 `/map` 都在 `camera_init`。
Nav2 只接管规划和速度控制，不再重建一套 `map -> odom -> base_link`。
RViz 的 `Fixed Frame` 继续使用 `camera_init`。

```bash
ros2 launch h1_2_slam_bringup h1_2_nav2_camera_init_navigation.launch.py
```

一键脚本等价用法：

```bash
./run_ai_slam_nav.sh
```

或者显式写：

```bash
./run_ai_slam_nav.sh mode=online
```

`mode=mapping` 也会进入 `camera_init` 在线建图 + Nav2，保留它只是为了兼容旧习惯：

```bash
./run_ai_slam_nav.sh mode=mapping
```

启动后在 RViz 里把 `Fixed Frame` 设为 `camera_init`，可以直接用普通
2D goal 发布 `/goal_pose`，本工程的 `goal_pose_to_nav2` 会转成 Nav2 的
`NavigateToPose` action。也可以使用 RViz 的 `Nav2 Goal`。如果想保存当前
在线建出的地图：

推荐直接用配好的 RViz：

```bash
ros2 launch h1_2_slam_bringup h1_2_nav2_rviz.launch.py
```

这份配置会显示 `/map`、`/cloud_registered`、`/plan` 和 `/slam_odom`，
其中 Odometry 的 `Keep` 已固定为 `1`，只显示当前位姿。

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/h1_2_lab
```


停止：

```bash
./stop_ai_slam_nav.sh
```

如果以后要用已经保存好的固定地图导航，也保留离线定位入口：

```bash
./run_ai_slam_nav.sh mode=navigation map=$PWD/maps/h1_2_lab.yaml
```

对应的 launch 是：

```bash
ros2 launch h1_2_slam_bringup h1_2_nav2_navigation.launch.py \
  map:=$PWD/maps/h1_2_lab.yaml
```

离线定位导航启动后，在 RViz 里先用 `2D Pose Estimate` 给 `/initialpose`，
再用 2D goal 或 `Nav2 Goal` 给目标。Nav2 输出 `/cmd_vel`，本工程会先平滑到
`/cmd_vel_nav`，再交给 `h1_2_loco_bridge`。`camera_init` 在线建图 + Nav2 模式通常不需要
手动给初始位姿。

## 离线地图演示工作流

在线建图 + 在线导航仍然是默认入口，继续用于调试雷达、现场建图、临时目标点导航和保存地图：

```bash
./run_ai_slam_nav.sh mode=online
```

如果要做“预设点长期复用、后续一条指令循环巡航”的演示，先在在线模式下走完整个演示区域，然后保存固定地图：

```bash
mkdir -p maps
./run_ai_slam_nav.sh mode=save_map map_prefix=maps/h1_2_lab
./run_ai_slam_nav.sh mode=save_pcd_map pcd_prefix=maps/h1_2_lab
```

`mode=save_map` 只封装 `nav2_map_server` 的 `map_saver_cli`，用于把当前已经发布的 `/map` 保存成 `maps/h1_2_lab.yaml` 和 `maps/h1_2_lab.pgm`。它不会停止正在运行的在线建图/导航；通常在另一个终端里执行。

`mode=save_pcd_map` 只封装 FAST-LIO2 的 `/map_save` 服务，用于把当前在线建图累计的 3D 点云保存成 `maps/h1_2_lab.pcd`。这份 PCD 必须和 `maps/h1_2_lab.yaml`、`maps/h1_2_lab.pgm` 来自同一次在线建图，确保 2D 栅格和 3D PCD 处于同一坐标系。

之后每次演示推荐加载同一份 2D 栅格和 3D PCD，用 `global_gicp` 做全局初始化、GICP 跟踪和失败恢复：

```bash
./run_ai_slam_nav.sh mode=navigation \
  map=$PWD/maps/h1_2_lab.yaml \
  localization=global_gicp \
  pcd=$PWD/maps/h1_2_lab.pcd
```

旧版局部 ICP 后端仍可用于对比测试。它依赖 RViz 或记忆点给定初始位姿，不会做全局粗定位：

```bash
./run_ai_slam_nav.sh mode=navigation \
  map=$PWD/maps/h1_2_lab.yaml \
  localization=icp \
  pcd=$PWD/maps/h1_2_lab.pcd
```

`global_gicp` 会先订阅 `/cloud_registered_body` 和 `/slam_odom`，在 PCD 地图上全局初始化，然后持续发布 `map -> camera_init`。如果记忆点文件里有 `START`，自动初始位姿节点也会发布 `/initialpose` 辅助收敛；需要手动干预时仍可在 RViz 里用 `2D Pose Estimate`。完成定位后再发送普通目标点；后续新增记忆点和循环巡航模块时，也应在这个 `map` 坐标系下保存 A/B/C 等演示点。

`global_gicp` 常用调参入口：

```bash
./run_ai_slam_nav.sh mode=navigation \
  map=$PWD/maps/h1_2_lab.yaml \
  localization=global_gicp \
  pcd=$PWD/maps/h1_2_lab.pcd
```

如果现场初始化慢或失败，可以直接用 launch 调这些参数：

```bash
ros2 launch h1_2_slam_bringup h1_2_nav2_navigation.launch.py \
  map:=$PWD/maps/h1_2_lab.yaml \
  localization_backend:=global_gicp \
  pcd_map:=$PWD/maps/h1_2_lab.pcd \
  global_voxel_resolution:=0.35 \
  tracking_voxel_resolution:=0.20 \
  global_score_threshold:=0.5 \
  gicp_score_threshold:=0.35 \
  global_max_accumulated_points:=120000
```

这条离线演示流程不会替换在线模式：在线模式负责建图和调试，离线模式负责固定地图下的可重复演示。

## 可选 3D 重定位依赖

KISS-Matcher 和 small_gicp 不作为默认硬依赖。新机器使用前，先准备源码，把已有 `third_party` 目录拷贝到工作区，目录里只保留源码和 `.git` 信息，不包含 build 或 install 产物：

```text
third_party/KISS-Matcher
third_party/ROBIN
third_party/pmc
third_party/xenium
third_party/small_gicp
```

然后在新环境本地编译安装：

```bash
./scripts/install_relocalization_deps.sh
```

脚本不会 clone 或 fetch，也不会访问网络；它只检查 `third_party/` 中已有源码是否完整，然后编译并安装到被忽略的本工作区路径：

```text
install/kiss_matcher
install/small_gicp
```

推荐拷贝时排除构建缓存：

```bash
rsync -a --exclude '*/build/' --exclude '*/install/' third_party/ <new-machine-workspace>/third_party/
```

安装后可显式验证 H1 包能发现这两个依赖：

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select h1_2_icp_localization \
  --cmake-args -DH1_USE_KISS_MATCHER=ON -DH1_USE_SMALL_GICP=ON
```

不加 `H1_USE_KISS_MATCHER=ON` 和 `H1_USE_SMALL_GICP=ON` 时，工程不会强制依赖这两个库，仍会使用当前 PCL 后端。

离线导航启动并完成初始定位后，可以保存和查看记忆点：

```bash
ros2 run h1_2_nav_tools memory_point_cli save A
ros2 run h1_2_nav_tools memory_point_cli list
```

配置路线后可以启动或停止循环巡航：

```bash
ros2 run h1_2_nav_tools route_runner_cli start
ros2 run h1_2_nav_tools route_runner_cli stop
ros2 run h1_2_nav_tools route_runner_cli status
```

常用调参：

```bash
ros2 launch h1_2_slam_bringup h1_2_nav2_navigation.launch.py \
  map:=$PWD/maps/h1_2_lab.yaml \
  max_linear_speed:=0.15 \
  max_angular_speed:=0.4 \
  odom_yaw_offset:=3.141592653589793
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

这个工程保留了原项目的 `/way_point` 输出。远端机器人如果已有跟踪 `/way_point` 的控制节点，可以直接接上。当前 Nav2 链路也提供了 `/goal_pose` 到 `NavigateToPose` action 的桥接节点，方便继续使用普通 2D goal 工作流。

`velocity_smoother_ema` 默认把 `/cmd_vel` 平滑到 `/cmd_vel_smooth`。如果机器人底盘只听 `/cmd_vel`，请在启动时改参数：

```bash
ros2 run h1_2_nav_tools velocity_smoother_ema --ros-args \
  -p raw_cmd_topic:=/cmd_vel_raw \
  -p cmd_topic:=/cmd_vel
```

## 从原项目迁移时注意

原项目里的 `move_base`、TEB、`costmap_2d` 配置属于 ROS1 导航栈，不能直接在 ROS2 Humble 编译运行。ROS2 侧应启动 Nav2，然后使用本工程的示教路径、前瞻点和辅助节点。

原项目里的 `fastlio2` 是 ROS1/catkin 版，包含大量 `ros::NodeHandle`、ROS1 service、ROS1 Livox 自定义消息接口，不能通过复制目录在 ROS2 里直接编译。这个工程现在接入的是 ROS2 版 FAST-LIO2：`FAST_LIO_ROS2` 包名安装后是 `fast_lio2`，可执行文件是 `fastlio2_mapping`。
