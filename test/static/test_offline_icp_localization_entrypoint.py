from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_repo_file(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_navigation_script_exposes_icp_localization_arguments():
    script = read_repo_file("run_ai_slam_nav.sh")

    assert 'LOCALIZATION_BACKEND="global_gicp"' in script
    assert 'PCD_MAP_FILE=""' in script
    assert "localization=*" in script
    assert "pcd=*" in script
    assert "global_gicp" in script
    assert "localization=amcl" not in script
    assert '"localization_backend:=${LOCALIZATION_BACKEND}"' in script
    assert '"pcd_map:=${PCD_MAP_FILE}"' in script


def test_navigation_script_selects_motion_backend_at_startup():
    script = read_repo_file("run_ai_slam_nav.sh")

    assert 'MOTION_BACKEND="unitree"' in script
    assert "motion_backend=*)" in script
    assert '"motion_backend:=${MOTION_BACKEND}"' in script
    assert '"${MOTION_BACKEND}" != "unitree"' in script
    assert '"${MOTION_BACKEND}" != "policy"' in script
    assert "policy motion backend is only supported with mode=navigation" in script
    assert "motion_backend=policy" in script


def test_offline_navigation_launch_accepts_backend_and_pcd_without_touching_online_launch():
    offline = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py")
    online = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_camera_init_navigation.launch.py")

    assert 'DeclareLaunchArgument("localization_backend", default_value="global_gicp")' in offline
    assert 'DeclareLaunchArgument("pcd_map", default_value="")' in offline
    assert "localization_backend" not in online
    assert "pcd_map" not in online


def test_readme_documents_pcd_localization_without_amcl_fallback():
    readme = read_repo_file("README.md")

    assert "localization=global_gicp" in readme
    assert "localization=icp" in readme
    assert "pcd=$PWD/maps/h1_2_lab.pcd" in readme
    assert "localization=amcl" not in readme
    assert "AMCL" not in readme
