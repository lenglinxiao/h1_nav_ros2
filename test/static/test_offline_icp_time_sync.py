from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_fastlio_can_stamp_published_outputs_with_ros_time_for_offline_icp():
    source = read("src/FAST_LIO_ROS2/src/laserMapping.cpp")

    assert 'declare_parameter<bool>("publish.use_ros_time_stamps", false)' in source
    assert 'get_parameter_or<bool>("publish.use_ros_time_stamps", use_ros_time_stamps, false)' in source
    assert "output_ros_stamp()" in source
    assert "use_ros_time_stamps ? rclcpp::Clock(RCL_SYSTEM_TIME).now() : get_ros_time(lidar_end_time)" in source
    assert "odomAftMapped.header.stamp = output_ros_stamp()" in source
    assert "trans.header.stamp = odomAftMapped.header.stamp" in source


def test_fastlio_launch_exposes_ros_time_stamp_switch_defaulting_off():
    source = read("src/h1_2_slam_bringup/launch/fastlio2_mid360_mapping.launch.py")

    assert 'DeclareLaunchArgument("use_ros_time_stamps", default_value="false")' in source
    assert '"publish.use_ros_time_stamps": ParameterValue(' in source
    assert 'LaunchConfiguration("use_ros_time_stamps"), value_type=bool' in source


def test_offline_icp_enables_fastlio_ros_time_stamps_without_touching_amcl_or_online():
    offline = read("src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py")
    online = read("src/h1_2_slam_bringup/launch/h1_2_nav2_camera_init_navigation.launch.py")

    assert '"use_ros_time_stamps": PythonExpression(' in offline
    assert "' == 'icp' or '" in offline
    assert "' == 'global_gicp'" in offline
    assert "publish.use_ros_time_stamps" not in online
    assert "use_ros_time_stamps" not in online


def test_nav_scan_converter_stamps_scans_with_current_time_for_nav2_tf():
    converter = read("src/h1_2_nav_tools/h1_2_nav_tools/simple_pointcloud_to_laserscan.py")
    params = read("src/h1_2_slam_bringup/config/pointcloud_to_laserscan_body.yaml")

    assert 'self.declare_parameter("stamp_with_current_time", True)' in converter
    assert "scan.header.stamp = now.to_msg() if self.stamp_with_current_time else msg.header.stamp" in converter
    assert 'self.declare_parameter("stale_stamp_warn_sec", 1.0)' in converter
    assert "Input cloud stamp is" in converter
    assert "stamp_with_current_time: true" in params
    assert "stale_stamp_warn_sec: 1.0" in params
