from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BRINGUP = ROOT / "src" / "h1_2_slam_bringup" / "launch"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_offline_navigation_launch_has_only_pcd_localization_backend_paths():
    source = read(BRINGUP / "h1_2_nav2_navigation.launch.py")

    assert "localization_backend" in source
    assert "PythonExpression" in source
    assert "icp_backend_condition" in source
    assert "global_gicp_backend_condition" in source
    assert "navigation_launch.py" in source
    assert "h1_2_icp_localization.launch.py" in source
    assert "h1_2_global_relocalization.launch.py" in source
    assert 'condition=icp_backend_condition' in source
    assert 'condition=global_gicp_backend_condition' in source
    assert "amcl_backend_condition" not in source
    assert "bringup_launch.py" not in source
    assert "nav2_amcl" not in source


def test_icp_backend_launch_receives_pcd_map_frames_cloud_and_odom_topics():
    source = read(BRINGUP / "h1_2_nav2_navigation.launch.py")

    assert '"pcd_map": LaunchConfiguration("pcd_map")' in source
    assert '"map_frame": "map"' in source
    assert '"odom_frame": LaunchConfiguration("odom_frame")' in source
    assert '"base_frame": LaunchConfiguration("base_frame")' in source
    assert '"cloud_topic": LaunchConfiguration("fastlio_body_cloud_topic")' in source
    assert '"odom_topic": LaunchConfiguration("input_odom_topic")' in source


def test_icp_localization_launch_starts_icp_node_with_expected_parameters():
    source = read(BRINGUP / "h1_2_icp_localization.launch.py")

    assert 'package="h1_2_icp_localization"' in source
    assert 'executable="icp_localization_node"' in source
    assert 'DeclareLaunchArgument("pcd_map"' in source
    assert '"pcd_path": LaunchConfiguration("pcd_map")' in source
    assert '"publish_tf": True' in source
    assert '"status_topic": "/icp_localization/status"' in source


def test_global_relocalization_launch_exposes_recovery_stability_parameters():
    source = read(BRINGUP / "h1_2_global_relocalization.launch.py")

    assert 'executable="global_relocalization_node"' in source
    assert 'DeclareLaunchArgument("min_input_points"' in source
    assert 'DeclareLaunchArgument("max_accumulated_points"' in source
    assert '"min_input_points": ParameterValue(' in source
    assert '"max_accumulated_points": ParameterValue(' in source


def test_online_launch_does_not_reference_icp_backend():
    online = read(BRINGUP / "h1_2_nav2_camera_init_navigation.launch.py")

    assert "h1_2_icp_localization" not in online
    assert "localization_backend" not in online
    assert "pcd_map" not in online


def test_bringup_declares_runtime_dependency_on_icp_localization_package():
    package_xml = read(ROOT / "src" / "h1_2_slam_bringup" / "package.xml")

    assert "<exec_depend>h1_2_icp_localization</exec_depend>" in package_xml
