from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / "src" / "h1_2_nav_tools" / "h1_2_nav_tools"
SETUP = ROOT / "src" / "h1_2_nav_tools" / "setup.py"


def test_memory_point_cli_supports_save_delete_list_services():
    source = (TOOLS / "memory_point_cli.py").read_text(encoding="utf-8")

    assert "SaveMemoryPoint" in source
    assert "DeleteMemoryPoint" in source
    assert "ListMemoryPoints" in source
    assert "save_memory_point" in source
    assert "delete_memory_point" in source
    assert "list_memory_points" in source
    assert "--overwrite" in source


def test_route_runner_cli_supports_start_stop_status_services():
    source = (TOOLS / "route_runner_cli.py").read_text(encoding="utf-8")

    assert "StartRoute" in source
    assert "StopRoute" in source
    assert "RouteStatus" in source
    assert "start_memory_route" in source
    assert "stop_memory_route" in source
    assert "memory_route_status" in source


def test_cli_tools_are_registered_as_console_scripts():
    setup = SETUP.read_text(encoding="utf-8")

    assert "memory_point_cli = h1_2_nav_tools.memory_point_cli:main" in setup
    assert "route_runner_cli = h1_2_nav_tools.route_runner_cli:main" in setup
