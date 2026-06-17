from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_repo_file(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_auto_initial_pose_can_initialize_and_wait_for_icp_localization():
    node = read_repo_file("src/h1_2_nav_tools/h1_2_nav_tools/auto_initial_pose.py")

    assert "LocalizationStatus" in node
    assert "Relocalize" in node
    assert "localization_backend" in node
    assert "icp_status_topic" in node
    assert "icp_relocalize_service" in node
    assert "icp_pcd_path" in node
    assert "max_icp_score" in node
    assert "on_icp_status" in node
    assert "request_icp_relocalize" in node
    assert "self.localization_backend == \"icp\"" in node
    assert "icp_status_is_ready" in node
    assert "msg.initialized and msg.converged and msg.score <= self.max_icp_score" in node
    assert "self.stable_icp_samples >= self.required_stable_samples" in node


def test_offline_launch_passes_icp_gate_parameters_to_auto_initial_pose():
    launch = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py")

    assert 'DeclareLaunchArgument("wait_for_icp_status"' in launch
    assert 'DeclareLaunchArgument("max_icp_score"' in launch
    assert '"localization_backend": LaunchConfiguration("localization_backend")' in launch
    assert '"icp_status_topic": "/icp_localization/status"' in launch
    assert '"icp_relocalize_service": "icp_relocalize"' in launch
    assert '"icp_pcd_path": LaunchConfiguration("pcd_map")' in launch
    assert '"wait_for_icp_status": ParameterValue(LaunchConfiguration("wait_for_icp_status"), value_type=bool)' in launch
    assert '"max_icp_score": ParameterValue(LaunchConfiguration("max_icp_score"), value_type=float)' in launch


def test_nav_tools_declares_runtime_dependency_on_icp_interfaces():
    package_xml = read_repo_file("src/h1_2_nav_tools/package.xml")

    assert "<exec_depend>h1_2_icp_localization</exec_depend>" in package_xml
