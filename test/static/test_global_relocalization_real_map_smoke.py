from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_real_map_smoke_replays_a_crop_from_the_h1_lab_pcd():
    source = read("test/ros_integration/global_relocalization_real_map_smoke.py")

    assert 'MAP_PATH = ROOT / "maps" / "h1_2_lab.pcd"' in source
    assert '"global_relocalization_node"' in source
    assert "crop_points_near_pose" in source
    assert "GICP tracking succeeded" in source
