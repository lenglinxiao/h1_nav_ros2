from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BRINGUP = ROOT / "src" / "h1_2_slam_bringup" / "launch"
RUNNER = ROOT / "src" / "h1_2_nav_tools" / "h1_2_nav_tools" / "memory_route_runner.py"


def read(path):
    return path.read_text(encoding="utf-8")


def test_offline_navigation_launch_starts_memory_nodes_in_map_frame():
    source = read(BRINGUP / "h1_2_nav2_navigation.launch.py")

    assert 'DeclareLaunchArgument("memory_points_file"' in source
    assert 'DeclareLaunchArgument("memory_route"' in source
    assert 'executable="memory_point_manager"' in source
    assert 'executable="memory_route_runner"' in source
    assert '"odom_topic": LaunchConfiguration("input_odom_topic")' in source
    assert '"target_frame": "map"' in source
    assert '"route_names": LaunchConfiguration("memory_route")' in source


def test_checked_in_demo_memory_points_are_in_offline_map_frame():
    points = read(ROOT / "maps" / "memory_points.json")

    assert '"frame_id": "map"' in points
    assert '"frame_id": "camera_init"' not in points



def test_online_navigation_launch_starts_memory_nodes_in_camera_init_frame():
    source = read(BRINGUP / "h1_2_nav2_camera_init_navigation.launch.py")

    assert 'DeclareLaunchArgument("memory_points_file"' in source
    assert 'DeclareLaunchArgument("memory_route"' in source
    assert 'executable="memory_point_manager"' in source
    assert 'executable="memory_route_runner"' in source
    assert '"odom_topic": LaunchConfiguration("odom_topic")' in source
    assert '"target_frame": LaunchConfiguration("global_frame")' in source
    assert '"route_names": LaunchConfiguration("memory_route")' in source


def test_route_runner_accepts_comma_separated_launch_route():
    source = read(RUNNER)

    assert "def parse_route_names" in source
    assert 'value.split(",")' in source
    assert "self.parse_route_names" in source


def test_offline_navigation_starts_loco_ros_shim_for_cmd_vel_nav():
    launch = (ROOT / "src" / "h1_2_slam_bringup" / "launch" / "h1_2_nav2_navigation.launch.py").read_text(encoding="utf-8")

    assert '"cmd_topic": "/cmd_vel_nav"' in launch
    assert '"start_ros_shim": "true"' in launch
    assert '"auto_start": LaunchConfiguration("auto_start_loco")' in launch


def test_offline_memory_point_manager_saves_from_raw_slam_odom_not_yaw_offset_odom():
    source = read(BRINGUP / "h1_2_nav2_navigation.launch.py")

    manager_index = source.index('executable="memory_point_manager"')
    manager_block = source[manager_index: source.index('executable="memory_point_markers"')]
    assert '"odom_topic": LaunchConfiguration("input_odom_topic")' in manager_block
    assert '"odom_topic": LaunchConfiguration("output_odom_topic")' not in manager_block
