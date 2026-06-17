from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_repo_file(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_offline_rviz_uses_raw_slam_odom_for_visual_arrow():
    rviz = read_repo_file("src/h1_2_nav_tools/rviz/h1_2_nav2_offline.rviz")

    assert "Name: Odometry" in rviz
    assert "Value: /slam_odom" in rviz
    assert "Value: /odom" not in rviz


def test_offline_rviz_keeps_map_on_navigation_plane():
    rviz = read_repo_file("src/h1_2_nav_tools/rviz/h1_2_nav2_offline.rviz")

    assert "Name: Map" in rviz
    assert "Z: -1.2" not in rviz


def test_offline_launch_raises_robot_visual_model_to_ground_plane():
    launch = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py")
    online_launch = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_camera_init_navigation.launch.py")

    assert 'DeclareLaunchArgument("robot_pelvis_z", default_value="1.03")' in launch
    assert 'DeclareLaunchArgument("robot_pelvis_z", default_value="-0.22")' in online_launch


def test_online_rviz_still_uses_camera_init_frame_and_slam_odom():
    rviz = read_repo_file("src/h1_2_nav_tools/rviz/h1_2_nav2_camera_init.rviz")

    assert "Fixed Frame: camera_init" in rviz
    assert "Value: /slam_odom" in rviz


def test_offline_rviz_uses_visual_slam_odom_arrow_above_ground():
    rviz = read_repo_file("src/h1_2_nav_tools/rviz/h1_2_nav2_offline.rviz")
    launch = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py")
    setup = read_repo_file("src/h1_2_nav_tools/setup.py")

    assert "Value: /slam_odom_visual" in rviz
    assert 'executable="odometry_z_offset"' in launch
    assert '"input_topic": LaunchConfiguration("input_odom_topic")' in launch
    assert '"output_topic": "/slam_odom_visual"' in launch
    assert '"z_offset": ParameterValue(LaunchConfiguration("robot_pelvis_z"), value_type=float)' in launch
    assert "odometry_z_offset = h1_2_nav_tools.odometry_z_offset:main" in setup
