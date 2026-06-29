from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / "src" / "h1_2_icp_localization"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_icp_localization_package_declares_ros2_and_pcl_dependencies():
    package_xml = read(PKG / "package.xml")
    cmake = read(PKG / "CMakeLists.txt")

    for dependency in [
        "rclcpp",
        "sensor_msgs",
        "nav_msgs",
        "geometry_msgs",
        "std_msgs",
        "tf2_ros",
        "pcl_conversions",
        "PCL",
        "rosidl_default_generators",
    ]:
        assert dependency in package_xml or dependency in cmake

    assert "icp_localization_node" in cmake
    assert "rosidl_generate_interfaces" in cmake
    assert "srv/Relocalize.srv" in cmake
    assert "msg/LocalizationStatus.msg" in cmake


def test_icp_localizer_core_uses_two_stage_point_to_plane_icp():
    header = read(PKG / "include" / "h1_2_icp_localization" / "icp_localizer.hpp")
    source = read(PKG / "src" / "icp_localizer.cpp")

    assert "class IcpLocalizer" in header
    assert "loadMap" in header
    assert "align" in header
    assert "multiAlign" in header
    assert "pcl::IterativeClosestPointWithNormals" in header
    assert "voxel_rough_filter_" in header
    assert "voxel_refine_filter_" in header
    assert "NormalEstimation" in source
    assert "icp_rough_.align" in source
    assert "icp_refine_.align" in source
    assert "score_threshold_" in source


def test_icp_localization_node_subscribes_cloud_and_odom_and_publishes_map_to_odom_tf():
    node = read(PKG / "src" / "icp_localization_node.cpp")

    assert "create_subscription<sensor_msgs::msg::PointCloud2>" in node
    assert "create_subscription<nav_msgs::msg::Odometry>" in node
    assert "create_service<h1_2_icp_localization::srv::Relocalize>" in node
    assert "create_publisher<h1_2_icp_localization::msg::LocalizationStatus>" in node
    assert "tf_broadcaster_" in node
    assert "map_frame" in node
    assert "odom_frame" in node
    assert "base_frame" in node
    assert "map_to_odom" in node


def test_global_relocalization_node_keeps_relocalize_service_compatibility():
    node = read(PKG / "src" / "global_relocalization_node.cpp")

    assert '#include "h1_2_icp_localization/srv/relocalize.hpp"' in node
    assert "create_service<h1_2_icp_localization::srv::Relocalize>" in node
    assert '"icp_relocalize"' in node
    assert "onRelocalize" in node
    assert "request->pcd_path" in node
    assert "forceGlobalRecovery" in node


def test_global_relocalization_node_snapshots_map_resources_for_runtime_reload():
    node = read(PKG / "src" / "global_relocalization_node.cpp")

    assert "std::mutex map_mutex_" in node
    assert "struct MapSnapshot" in node
    assert "snapshotMap()" in node
    assert "publishMapSnapshot" in node
    assert "const MapSnapshot map = snapshotMap()" in node
    assert "tryGlobalAlignment(source, map, map_to_base, score)" in node
    assert "tryGicp(source, map, initial_guess, map_to_base, score)" in node


def test_global_relocalization_tf_is_future_dated_for_nav2_transform_tolerance():
    node = read(PKG / "src" / "global_relocalization_node.cpp")

    assert 'declare_parameter<double>("transform_tolerance_sec", 0.5)' in node
    assert "rclcpp::Duration::from_seconds(transform_tolerance_sec_)" in node
    assert "stamp = now() + rclcpp::Duration::from_seconds(transform_tolerance_sec_)" in node
    assert "stamp = latest_odom_stamp_" not in node
    assert "Publishing TF %s -> %s" in node
    assert "Stopped publishing TF %s -> %s" in node
    assert "tf_is_publishing_" in node


def test_icp_cmake_exposes_optional_kiss_matcher_dependency_without_requiring_it():
    cmake = read(PKG / "CMakeLists.txt")
    gitignore = read(ROOT / ".gitignore")

    assert "option(H1_USE_KISS_MATCHER" in cmake
    assert 'H1_USE_KISS_MATCHER "Build optional KISS-Matcher backend" OFF' in cmake
    assert "H1_KISS_MATCHER_PREFIX" in cmake
    assert "H1_WORKSPACE_DIR" in cmake
    assert "list(PREPEND CMAKE_PREFIX_PATH" in cmake
    assert "install/kiss_matcher" in cmake
    assert "find_package(robin QUIET)" in cmake
    assert "find_package(TBB QUIET)" in cmake
    assert "find_package(kiss_matcher QUIET)" in cmake
    assert "H1_HAVE_KISS_MATCHER" in cmake

    assert "third_party/**/build/" in gitignore
    assert "third_party/**/build_*/" in gitignore
    assert ".codex_tmp/" in gitignore


def test_icp_cmake_exposes_optional_small_gicp_dependency_without_requiring_it():
    cmake = read(PKG / "CMakeLists.txt")
    gitignore = read(ROOT / ".gitignore")

    assert "option(H1_USE_SMALL_GICP" in cmake
    assert 'H1_USE_SMALL_GICP "Build optional small_gicp backend" OFF' in cmake
    assert "H1_SMALL_GICP_PREFIX" in cmake
    assert "install/small_gicp" in cmake
    assert "find_package(small_gicp QUIET)" in cmake
    assert "H1_HAVE_SMALL_GICP" in cmake
    assert "small_gicp was not found" in cmake

    assert "third_party/**/build/" in gitignore
    assert "third_party/**/build_*/" in gitignore


def test_global_relocalization_node_uses_kiss_matcher_and_small_gicp_when_enabled():
    node = read(PKG / "src" / "global_relocalization_node.cpp")
    cmake = read(PKG / "CMakeLists.txt")

    assert "#ifdef H1_HAVE_KISS_MATCHER" in node
    assert "#include <kiss_matcher/KISSMatcher.hpp>" in node
    assert "kiss_matcher::KISSMatcherConfig" in node
    assert "matcher.estimate(source_points, target_points)" in node
    assert "matcher.getNumFinalInliers()" in node
    assert '"KISSMatcher global initialization succeeded"' in node
    assert '"KISSMatcher global recovery succeeded"' in node

    assert "#ifdef H1_HAVE_SMALL_GICP" in node
    assert "#include <small_gicp/registration/registration.hpp>" in node
    assert "small_gicp::Registration<small_gicp::GICPFactor" in node
    assert "registration.align(*map.small_gicp_target" in node
    assert '"small_gicp tracking succeeded"' in node

    assert "target_compile_definitions(global_relocalization_node PRIVATE H1_HAVE_KISS_MATCHER)" in cmake
    assert "target_compile_definitions(global_relocalization_node PRIVATE H1_HAVE_SMALL_GICP)" in cmake
    assert "kiss_matcher::kiss_matcher_core" in cmake
    assert "small_gicp::small_gicp" in cmake


def test_global_relocalization_launch_keeps_kiss_runtime_opt_in():
    global_launch = read(
        ROOT
        / "src"
        / "h1_2_slam_bringup"
        / "launch"
        / "h1_2_global_relocalization.launch.py"
    )
    navigation_launch = read(
        ROOT
        / "src"
        / "h1_2_slam_bringup"
        / "launch"
        / "h1_2_nav2_navigation.launch.py"
    )

    assert 'DeclareLaunchArgument("use_kiss_matcher", default_value="false")' in global_launch
    assert '"use_kiss_matcher": ParameterValue(' in global_launch
    assert 'DeclareLaunchArgument("use_small_gicp", default_value="true")' in global_launch
    assert '"use_small_gicp": ParameterValue(' in global_launch

    assert 'DeclareLaunchArgument("global_use_kiss_matcher", default_value="false")' in navigation_launch
    assert '"use_kiss_matcher": LaunchConfiguration("global_use_kiss_matcher")' in navigation_launch
    assert 'DeclareLaunchArgument("global_use_small_gicp", default_value="true")' in navigation_launch
    assert '"use_small_gicp": LaunchConfiguration("global_use_small_gicp")' in navigation_launch


def test_icp_interfaces_expose_relocalize_request_and_status_score():
    relocalize = read(PKG / "srv" / "Relocalize.srv")
    status = read(PKG / "msg" / "LocalizationStatus.msg")

    assert "string pcd_path" in relocalize
    assert "geometry_msgs/Pose initial_pose" in relocalize
    assert "bool success" in relocalize
    assert "float64 score" in relocalize
    assert "bool initialized" in status
    assert "bool converged" in status
    assert "float64 score" in status
