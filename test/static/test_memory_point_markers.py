from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_repo_file(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_memory_point_marker_node_publishes_marker_array_from_points_file():
    node = read_repo_file("src/h1_2_nav_tools/h1_2_nav_tools/memory_point_markers.py")

    assert "class MemoryPointMarkers" in node
    assert "MarkerArray" in node
    assert "visualization_msgs.msg" in node
    assert "points_file" in node
    assert "maps/memory_points.json" in node
    assert "marker_topic" in node
    assert '"/memory_points_markers"' in node
    assert "Marker.SPHERE" in node
    assert "Marker.TEXT_VIEW_FACING" in node
    assert "marker.header.stamp = Time()" in node
    assert "delete_all.action = Marker.DELETEALL" not in node


def test_memory_point_marker_console_script_is_registered():
    setup = read_repo_file("src/h1_2_nav_tools/setup.py")

    assert "memory_point_markers = h1_2_nav_tools.memory_point_markers:main" in setup


def test_offline_navigation_launch_starts_memory_point_markers():
    launch = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py")

    assert 'executable="memory_point_markers"' in launch
    assert '"points_file": LaunchConfiguration("memory_points_file")' in launch
    assert '"marker_topic": "/memory_points_markers"' in launch


def test_offline_rviz_displays_memory_point_markers():
    rviz = read_repo_file("src/h1_2_nav_tools/rviz/h1_2_nav2_offline.rviz")

    assert "Name: MemoryPoints" in rviz
    assert "Class: rviz_default_plugins/MarkerArray" in rviz
    assert "Value: /memory_points_markers" in rviz
