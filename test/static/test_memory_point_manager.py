from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MANAGER = ROOT / "src" / "h1_2_nav_tools" / "h1_2_nav_tools" / "memory_point_manager.py"
SETUP = ROOT / "src" / "h1_2_nav_tools" / "setup.py"


def test_memory_point_manager_registers_expected_services():
    source = MANAGER.read_text(encoding="utf-8")

    assert "class MemoryPointManager(Node):" in source
    assert "SaveMemoryPoint" in source
    assert "DeleteMemoryPoint" in source
    assert "ListMemoryPoints" in source
    assert "save_memory_point" in source
    assert "delete_memory_point" in source
    assert "list_memory_points" in source


def test_memory_point_manager_persists_json_points():
    source = MANAGER.read_text(encoding="utf-8")

    assert "points_file" in source
    assert "maps/memory_points.json" in source
    assert "json.dumps" in source
    assert "temp_path.replace(self.points_file)" in source
    assert "frame_id" in source
    assert "position" in source
    assert "orientation" in source


def test_memory_point_manager_uses_latest_odom_and_target_frame():
    source = MANAGER.read_text(encoding="utf-8")

    assert "Odometry" in source
    assert "/slam_odom" in source
    assert "target_frame" in source
    assert "lookup_transform" in source
    assert "transform_pose" in source


def test_memory_point_manager_console_script_is_registered():
    setup = SETUP.read_text(encoding="utf-8")

    assert "memory_point_manager = h1_2_nav_tools.memory_point_manager:main" in setup
