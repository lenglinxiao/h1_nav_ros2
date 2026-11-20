"""Offline URI validation; never start ROS or RViz."""
import importlib.util
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location(
    "rviz_launcher", Path(__file__).resolve().parents[2] / "scripts/ai_pc_open_rviz.py"
)
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


@pytest.mark.parametrize("mode", ["online", "offline"])
def test_valid_mode(mode):
    assert launcher.parse_mode(f"h1-rviz://open?mode={mode}") == mode


def test_close_uri_is_explicit_and_has_no_options():
    assert launcher.parse_action("h1-rviz://close") == ("close", None, None)


def test_close_only_accepts_the_managed_rviz_launch_command():
    assert launcher.is_managed_rviz_command([
        b"ros2", b"launch", b"h1_2_slam_bringup", b"h1_2_nav2_rviz.launch.py",
        b"mode:=online",
    ])
    assert launcher.is_managed_rviz_command([
        b"/usr/bin/python3", b"/opt/ros/humble/bin/ros2", b"launch",
        b"h1_2_slam_bringup", b"h1_2_nav2_rviz.launch.py", b"mode:=online",
    ])
    assert not launcher.is_managed_rviz_command([
        b"ros2", b"launch", b"other_package", b"h1_2_nav2_rviz.launch.py",
    ])
    assert not launcher.is_managed_rviz_command([
        b"python3", b"-c", b"import os; os.kill(1, 9)",
    ])


def test_legacy_rviz_discovery_requires_one_exact_isolated_launch_group():
    command = [
        b"/usr/bin/python3", b"/opt/ros/humble/bin/ros2", b"launch",
        b"h1_2_slam_bringup", b"h1_2_nav2_rviz.launch.py",
    ]
    assert launcher.select_unique_managed_rviz_pid([(123, 123, 123, command)]) == 123
    assert launcher.select_unique_managed_rviz_pid([
        (123, 123, 123, command), (124, 124, 124, command),
    ]) is None
    assert launcher.select_unique_managed_rviz_pid([(123, 10, 10, command)]) is None


@pytest.mark.parametrize("backend,rmw", [
    ("unitree", "rmw_cyclonedds_cpp"),
    ("policy", "rmw_fastrtps_cpp"),
])
def test_backend_selection(backend, rmw):
    assert launcher.parse_options(f"h1-rviz://open?mode=online&backend={backend}") == ("online", backend)
    script = (Path(__file__).resolve().parents[2] / "scripts/ai_pc_start_rviz.sh").read_text()
    assert rmw in script
    if backend == "policy":
        assert 'FASTRTPS_DEFAULT_PROFILES_FILE="${WORKSPACE_DIR}/config/fastdds_ai_pc_policy.xml"' in script
        profile = Path(__file__).resolve().parents[2] / "config/fastdds_ai_pc_policy.xml"
        assert "192.168.123.99" in profile.read_text()


@pytest.mark.parametrize("uri", [
    "h1-rviz://open?mode=online&command=touch%20/tmp/unwanted",
    "h1-rviz://open?mode=online;touch /tmp/unwanted",
    "h1-rviz://open?mode=$(touch /tmp/unwanted)",
    "h1-rviz://open?mode=offline#extra",
    "h1-rviz://close?mode=online",
    "https://example.com/?mode=online",
    "h1-rviz://open?mode=online&mode=offline",
    "h1-rviz://open?mode=online&backend=unexpected",
    "h1-rviz://open?mode=online&backend=policy&backend=unitree",
    "h1-rviz://open",
])
def test_rejects_untrusted_uri(uri):
    with pytest.raises(ValueError):
        launcher.parse_mode(uri)
