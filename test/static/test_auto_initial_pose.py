from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_repo_file(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_auto_initial_pose_node_reads_start_point_and_publishes_initialpose():
    node = read_repo_file("src/h1_2_nav_tools/h1_2_nav_tools/auto_initial_pose.py")

    assert "class AutoInitialPose" in node
    assert "points_file" in node
    assert "start_point_name" in node
    assert '"START"' in node
    assert "PoseWithCovarianceStamped" in node
    assert "output_topic" in node
    assert '"/initialpose"' in node
    assert "odom_topic" in node
    assert "latest_odom_stamp" in node


def test_auto_initial_pose_can_wait_for_localization_before_starting_route():
    node = read_repo_file("src/h1_2_nav_tools/h1_2_nav_tools/auto_initial_pose.py")

    assert "wait_for_tf_target_frame" in node
    assert "wait_for_tf_source_frame" in node
    assert "lookup_transform" in node
    assert "auto_start_route" in node
    assert "start_route_service" in node
    assert "StartRoute" in node


def test_auto_initial_pose_is_installed_as_console_script():
    setup = read_repo_file("src/h1_2_nav_tools/setup.py")

    assert "auto_initial_pose = h1_2_nav_tools.auto_initial_pose:main" in setup


def test_offline_navigation_launch_wires_auto_initial_pose_without_touching_online_launch():
    offline_launch = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py")
    online_launch = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_slam_navigation.launch.py")

    assert 'DeclareLaunchArgument("auto_initial_pose"' in offline_launch
    assert 'DeclareLaunchArgument("initial_pose_name"' in offline_launch
    assert 'DeclareLaunchArgument("auto_start_route"' in offline_launch
    assert 'executable="auto_initial_pose"' in offline_launch
    assert '"points_file": LaunchConfiguration("memory_points_file")' in offline_launch
    assert '"start_point_name": LaunchConfiguration("initial_pose_name")' in offline_launch
    assert '"auto_start_route": ParameterValue(LaunchConfiguration("auto_start_route"), value_type=bool)' in offline_launch
    assert "auto_initial_pose" not in online_launch


def test_route_runner_refuses_to_start_until_map_tf_is_available():
    runner = read_repo_file("src/h1_2_nav_tools/h1_2_nav_tools/memory_route_runner.py")

    assert "required_tf_target_frame" in runner
    assert "required_tf_source_frame" in runner
    assert "tf_buffer" in runner
    assert "lookup_transform" in runner
    assert "Localization TF is not available" in runner


def test_navigation_script_exposes_auto_initial_pose_demo_arguments():
    script = read_repo_file("run_ai_slam_nav.sh")

    assert "INITIAL_POSE_NAME" in script
    assert "AUTO_START_ROUTE" in script
    assert "MEMORY_ROUTE" in script
    assert "initial_pose=*" in script
    assert "auto_start_route=*" in script
    assert "memory_route=*" in script
    assert 'initial_pose_name:="${INITIAL_POSE_NAME}"' in script
    assert 'auto_start_route:="${AUTO_START_ROUTE}"' in script
    assert 'memory_route:="${MEMORY_ROUTE}"' in script


def test_auto_initial_pose_starts_route_only_after_publishing_and_settling():
    node = read_repo_file("src/h1_2_nav_tools/h1_2_nav_tools/auto_initial_pose.py")
    launch = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py")

    assert "route_start_delay_sec" in node
    assert "initial_pose_published" in node
    assert "localization_ready_since" in node
    assert "can_start_route" in node
    assert "self.publish_count > 0" in node
    assert "elapsed >= self.route_start_delay_sec" in node
    assert 'DeclareLaunchArgument("route_start_delay_sec"' in launch
    assert '"route_start_delay_sec": ParameterValue(LaunchConfiguration("route_start_delay_sec"), value_type=float)' in launch


def test_auto_initial_pose_uses_icp_status_not_amcl_pose_before_auto_route_start():
    node = read_repo_file("src/h1_2_nav_tools/h1_2_nav_tools/auto_initial_pose.py")
    launch = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py")

    assert "required_stable_samples" in node
    assert "localization_timeout_sec" in node
    assert "localization_quality_ready" in node
    assert "latest_icp_status" in node
    assert "wait_for_icp_status" in node
    assert "max_icp_score" in node
    assert "amcl_pose_topic" not in node
    assert '"/amcl_pose"' not in node
    assert "wait_for_amcl_pose" not in node
    assert "stable_amcl_samples" not in node
    assert "on_amcl_pose" not in node
    assert "amcl_pose_is_stable" not in node
    assert 'DeclareLaunchArgument("wait_for_amcl_pose"' not in launch
    assert 'DeclareLaunchArgument("required_stable_samples"' in launch
    assert 'DeclareLaunchArgument("localization_timeout_sec"' in launch
    assert '"icp_status_topic": "/icp_localization/status"' in launch


def test_auto_initial_pose_republishes_start_pose_multiple_times_quickly():
    node = read_repo_file("src/h1_2_nav_tools/h1_2_nav_tools/auto_initial_pose.py")
    launch = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py")

    assert "publish_period_sec" in node
    assert "max_publish_count" in node
    assert "Published auto initial pose" in node
    assert 'DeclareLaunchArgument("initial_pose_publish_count"' in launch
    assert 'DeclareLaunchArgument("initial_pose_publish_period_sec"' in launch
    assert '"max_publish_count": ParameterValue(LaunchConfiguration("initial_pose_publish_count"), value_type=int)' in launch
    assert '"publish_period_sec": ParameterValue(LaunchConfiguration("initial_pose_publish_period_sec"), value_type=float)' in launch


def test_auto_initial_pose_publishes_all_initial_pose_samples_before_waiting_for_quality():
    node = read_repo_file("src/h1_2_nav_tools/h1_2_nav_tools/auto_initial_pose.py")

    assert "if self.publish_count < self.max_publish_count:" in node
    assert "self.first_localization_ready_time = None" in node


def test_auto_initial_pose_warns_with_latest_icp_status_when_not_ready():
    node = read_repo_file("src/h1_2_nav_tools/h1_2_nav_tools/auto_initial_pose.py")

    assert "latest_icp_status_summary" in node
    assert "latest ICP status" in node
    assert "no /icp_localization/status received" in node


def test_auto_initial_pose_default_icp_gate_is_lenient_for_real_global_gicp():
    launch = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py")

    assert 'DeclareLaunchArgument("required_stable_samples", default_value="1")' in launch
    assert 'DeclareLaunchArgument("max_icp_score", default_value="0.15")' in launch
