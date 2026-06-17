from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_repo_file(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_navigation_script_exposes_non_destructive_pcd_map_save_mode():
    script = read_repo_file("run_ai_slam_nav.sh")

    assert "mode=save_pcd_map" in script
    assert "PCD_SAVE_PREFIX" in script
    assert "pcd_prefix=*" in script
    assert "Save-PCD-map mode requires pcd_prefix=maps/h1_2_lab" in script
    assert 'ros2 service call /map_save std_srvs/srv/Trigger "{}"' in script
    assert 'if [[ "${MODE}" == "save_pcd_map" ]]' in script
    assert script.index('if [[ "${MODE}" == "save_pcd_map" ]]') < script.index('exec 9>"${LOCK_FILE}"')


def test_online_mode_passes_fastlio_pcd_save_path_without_replacing_online_workflow():
    script = read_repo_file("run_ai_slam_nav.sh")
    launch = read_repo_file("src/h1_2_slam_bringup/launch/fastlio2_mid360_mapping.launch.py")
    online_launch = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_camera_init_navigation.launch.py")
    offline_launch = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py")

    assert 'PCD_SAVE_PREFIX="maps/h1_2_lab"' in script
    assert 'map_file_path:="${PCD_SAVE_PREFIX}.pcd"' in script
    assert 'DeclareLaunchArgument("map_file_path", default_value="./test.pcd")' in launch
    assert '"map_file_path": LaunchConfiguration("map_file_path")' in launch
    assert 'DeclareLaunchArgument("map_file_path", default_value="./test.pcd")' in online_launch
    assert 'DeclareLaunchArgument("map_file_path", default_value="./test.pcd")' in offline_launch


def test_readme_documents_2d_and_3d_maps_share_the_same_build_session():
    readme = read_repo_file("README.md")

    assert "./run_ai_slam_nav.sh mode=save_pcd_map pcd_prefix=maps/h1_2_lab" in readme
    assert "maps/h1_2_lab.pcd" in readme
    assert "同一次在线建图" in readme
    assert "同一坐标系" in readme


def test_manual_checklist_includes_icp_acceptance_steps():
    manual = read_repo_file("test/manual/README.md")

    assert "ICP 离线定位验收" in manual
    assert "maps/h1_2_lab.yaml" in manual
    assert "maps/h1_2_lab.pcd" in manual
    assert "/icp_localization/status" in manual
    assert "map -> camera_init" in manual
    assert "Nav2 Goal" in manual
