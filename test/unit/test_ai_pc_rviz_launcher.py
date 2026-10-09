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


@pytest.mark.parametrize("uri", [
    "h1-rviz://open?mode=online&command=touch%20/tmp/unwanted",
    "h1-rviz://open?mode=online;touch /tmp/unwanted",
    "h1-rviz://open?mode=$(touch /tmp/unwanted)",
    "h1-rviz://open?mode=offline#extra",
    "h1-rviz://close?mode=online",
    "https://example.com/?mode=online",
    "h1-rviz://open?mode=online&mode=offline",
    "h1-rviz://open",
])
def test_rejects_untrusted_uri(uri):
    with pytest.raises(ValueError):
        launcher.parse_mode(uri)
