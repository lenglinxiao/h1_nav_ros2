from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_repo_file(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_online_navigation_remains_the_default_entrypoint():
    script = read_repo_file("run_ai_slam_nav.sh")

    assert 'MODE="online"' in script
    assert "mode=online" in script
    assert "mode=mapping" in script
    assert "mode=save_map" in script


def test_offline_navigation_entrypoint_requires_a_map_file():
    script = read_repo_file("run_ai_slam_nav.sh")

    assert 'MODE}" == "navigation"' in script
    assert "Navigation mode requires map=/absolute/path/to/map.yaml" in script
    assert "Map file does not exist" in script
    assert "h1_2_nav2_navigation.launch.py" in script


def test_save_map_entrypoint_wraps_map_saver_without_replacing_online_mode():
    script = read_repo_file("run_ai_slam_nav.sh")

    assert "MAP_SAVE_PREFIX" in script
    assert "Save-map mode requires map_prefix=maps/h1_2_lab" in script
    assert 'ros2 run nav2_map_server map_saver_cli -f "${MAP_SAVE_PREFIX}"' in script
    assert script.index('if [[ "${MODE}" == "save_map" ]]') < script.index('exec 9>"${LOCK_FILE}"')
    assert 'pkill -f "nav2_"' not in script


def test_readme_documents_fixed_map_navigation_workflow():
    readme = read_repo_file("README.md")

    assert "## 离线地图演示工作流" in readme
    assert "在线建图 + 在线导航仍然是默认入口" in readme
    assert "./run_ai_slam_nav.sh mode=save_map map_prefix=maps/h1_2_lab" in readme
    assert "map_saver_cli" in readme
    assert "localization=global_gicp" in readme
    assert "2D Pose Estimate" in readme
    assert "这条离线演示流程不会替换在线模式" in readme
