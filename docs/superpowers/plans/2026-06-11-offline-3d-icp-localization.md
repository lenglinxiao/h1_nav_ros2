# Offline 3D ICP Localization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a 3D ICP localization path for offline fixed-map navigation while keeping online FAST-LIO2 + Nav2 behavior unchanged.

**Architecture:** Keep the existing online `camera_init` workflow untouched. For offline navigation, introduce a localization backend switch so AMCL remains available while ICP can become the default once the ROS2 ICP node exists. ICP will publish `map -> camera_init`; Nav2 will keep using the fixed 2D map for planning and costmaps.

**Tech Stack:** ROS2 Foxy launch, Nav2, FAST-LIO2, PCL ICP, Python static tests, Bash entry script.

---

## Todo Overview

- [ ] Task 1: Add offline ICP CLI and launch parameters without changing online mode.
- [ ] Task 2: Split offline Nav2 launch so AMCL and ICP localization backends are selectable.
- [ ] Task 3: Add `h1_2_icp_localization` ROS2 package with PCD loading and rough/refine ICP.
- [ ] Task 4: Add ICP status gating for auto route startup.
- [ ] Task 5: Add 3D PCD save workflow tied to the same coordinate base as the 2D map.
- [ ] Task 6: Add static, build, and manual verification coverage.

### Task 1: Offline ICP CLI and Launch Parameters

**Files:**
- Modify: `run_ai_slam_nav.sh`
- Modify: `src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py`
- Modify: `README.md`
- Test: `test/static/test_offline_icp_localization_entrypoint.py`

- [ ] **Step 1: Write the failing static test**

```python
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_repo_file(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_navigation_script_exposes_icp_localization_arguments():
    script = read_repo_file("run_ai_slam_nav.sh")

    assert 'LOCALIZATION_BACKEND="amcl"' in script
    assert 'PCD_MAP_FILE=""' in script
    assert "localization=*" in script
    assert "pcd=*" in script
    assert 'localization_backend:="${LOCALIZATION_BACKEND}"' in script
    assert 'pcd_map:="${PCD_MAP_FILE}"' in script


def test_offline_navigation_launch_accepts_backend_and_pcd_without_touching_online_launch():
    offline = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py")
    online = read_repo_file("src/h1_2_slam_bringup/launch/h1_2_nav2_camera_init_navigation.launch.py")

    assert 'DeclareLaunchArgument("localization_backend", default_value="amcl")' in offline
    assert 'DeclareLaunchArgument("pcd_map", default_value="")' in offline
    assert "localization_backend" not in online
    assert "pcd_map" not in online


def test_readme_documents_icp_as_planned_offline_backend_with_amcl_fallback():
    readme = read_repo_file("README.md")

    assert "localization=icp" in readme
    assert "pcd=$PWD/maps/h1_2_lab.pcd" in readme
    assert "localization=amcl" in readme
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest test/static/test_offline_icp_localization_entrypoint.py -q`

Expected: FAIL because the script and launch file do not yet expose `localization_backend` or `pcd_map`.

- [ ] **Step 3: Add minimal entrypoint support**

Add `LOCALIZATION_BACKEND` and `PCD_MAP_FILE` variables to `run_ai_slam_nav.sh`, parse `localization=*` and `pcd=*`, document them in usage, and forward them to `h1_2_nav2_navigation.launch.py`.

Add these launch arguments to `h1_2_nav2_navigation.launch.py`:

```python
DeclareLaunchArgument("localization_backend", default_value="amcl"),
DeclareLaunchArgument("pcd_map", default_value=""),
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest test/static/test_offline_icp_localization_entrypoint.py -q`

Expected: PASS.

### Task 2: Backend Selection in Offline Launch

**Files:**
- Modify: `src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py`
- Create: `src/h1_2_slam_bringup/launch/h1_2_icp_localization.launch.py`
- Test: `test/static/test_offline_icp_backend_switch.py`

- [ ] **Step 1: Write static tests for backend split**

Test that `localization_backend:=amcl` preserves the current Nav2 bringup path, and `localization_backend:=icp` has a separate ICP localization launch path that is not referenced by online launch files.

- [ ] **Step 2: Implement launch-time backend split**

Use launch conditions to keep AMCL in the old path and introduce an ICP-only include. The ICP include must receive `pcd_map`, `map_frame:=map`, `odom_frame:=camera_init`, `base_frame:=body`, `/cloud_registered_body`, and `/slam_odom`.

- [ ] **Step 3: Verify**

Run the new static test and the existing offline workflow tests.

### Task 3: ROS2 ICP Localization Package

**Files:**
- Create: `src/h1_2_icp_localization/package.xml`
- Create: `src/h1_2_icp_localization/CMakeLists.txt`
- Create: `src/h1_2_icp_localization/include/h1_2_icp_localization/icp_localizer.hpp`
- Create: `src/h1_2_icp_localization/src/icp_localizer.cpp`
- Create: `src/h1_2_icp_localization/src/icp_localization_node.cpp`
- Create: `src/h1_2_icp_localization/srv/Relocalize.srv`
- Test: `test/static/test_icp_package_structure.py`

- [ ] **Step 1: Add structure tests**

Check that the package exists, depends on `rclcpp`, `sensor_msgs`, `nav_msgs`, `tf2_ros`, `pcl_conversions`, `pcl_ros`, and exposes an executable.

- [ ] **Step 2: Port the PCL ICP core**

Implement PCD loading, rough/refine voxel filters, normal estimation, rough ICP, refine ICP, score thresholding, and multi-candidate initial pose search.

- [ ] **Step 3: Implement ROS2 node wrapper**

Subscribe to body cloud and FAST-LIO odom, provide a relocalization service, publish TF `map -> camera_init`, and publish localization status.

- [ ] **Step 4: Verify build**

Run: `colcon build --symlink-install --packages-select h1_2_icp_localization`

Expected: build exits 0.

### Task 4: Route Startup Gating

**Files:**
- Modify: `src/h1_2_nav_tools/h1_2_nav_tools/auto_initial_pose.py`
- Modify: `src/h1_2_slam_bringup/launch/h1_2_nav2_navigation.launch.py`
- Test: `test/static/test_auto_route_icp_status_gate.py`

- [ ] **Step 1: Write tests for ICP status gate parameters**

Check that auto route startup can wait for either AMCL covariance or ICP status based on `localization_backend`.

- [ ] **Step 2: Add ICP status wait support**

Subscribe to `/icp_localization/status`, require initialized and score below threshold before starting route.

- [ ] **Step 3: Verify**

Run route gating static tests.

### Task 5: 3D Map Save Workflow

**Files:**
- Modify: `run_ai_slam_nav.sh`
- Modify: `README.md`
- Test: `test/static/test_pcd_map_save_workflow.py`

- [ ] **Step 1: Write tests for `mode=save_pcd_map`**

Check that PCD save mode does not stop online mode and documents that PCD and 2D map must share the same coordinate base.

- [ ] **Step 2: Implement wrapper around FAST-LIO2 `/map_save`**

Add a non-destructive script mode that calls the FAST-LIO2 map save service and stores the PCD under `maps/`.

- [ ] **Step 3: Verify**

Run static workflow tests.

### Task 6: Verification Coverage

**Files:**
- Modify: `test/static/*`
- Modify: `test/manual/README.md`

- [ ] **Step 1: Add manual ICP acceptance checklist**

Document RViz alignment checks, short-goal Nav2 checks, and memory route checks.

- [ ] **Step 2: Run static test suite**

Run: `pytest test/static -q`

Expected: all static tests pass.

- [ ] **Step 3: Run targeted build**

Run: `colcon build --symlink-install --packages-select h1_2_icp_localization h1_2_slam_bringup h1_2_nav_tools`

Expected: build exits 0 after the ICP package is implemented.
