from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]


def read_repo_file(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_online_launch_keeps_navigation_map_and_enables_low_bandwidth_preview():
    launch = read_repo_file(
        "src/h1_2_slam_bringup/launch/h1_2_nav2_camera_init_navigation.launch.py"
    )
    grid_node = launch.split('executable="pointcloud_to_occupancy_grid"', 1)[1].split(
        'IncludeLaunchDescription(', 1
    )[0]

    assert 'DeclareLaunchArgument("map_topic", default_value="/map")' in launch
    assert '"map_topic": LaunchConfiguration("map_topic")' in grid_node
    assert '"publish_hz": 3.0' in grid_node
    assert '"preview_map_topic": "/map_preview"' in grid_node
    assert '"preview_publish_hz": 0.5' in grid_node
    assert '"preview_downsample_factor": 2' in grid_node


def test_online_rviz_subscribes_to_preview_map_and_its_matching_updates():
    rviz = read_repo_file("src/h1_2_nav_tools/rviz/h1_2_nav2_camera_init.rviz")
    map_display = rviz.split("Name: Map", 1)[1].split("- Alpha: 1", 1)[0]

    assert "Value: /map_preview" in map_display
    assert "Value: /map_preview_updates" in map_display
    assert "Value: /map\n" not in map_display
    assert "Value: /map_updates" not in map_display


def test_map_preview_wiring_does_not_change_offline_rviz_or_add_missing_numpy_dependency():
    package_xml = ET.fromstring(read_repo_file("src/h1_2_nav_tools/package.xml"))
    dependencies = {element.text for element in package_xml.findall("exec_depend")}
    offline_rviz = read_repo_file("src/h1_2_nav_tools/rviz/h1_2_nav2_offline.rviz")

    assert "python3-numpy" in dependencies
    assert re.search(r"Value: /map\s", offline_rviz)
    assert "/map_preview" not in offline_rviz
