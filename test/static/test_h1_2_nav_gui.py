from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / "src" / "h1_2_nav_tools"
GUI = TOOLS / "h1_2_nav_tools" / "h1_2_nav_gui.py"


def test_h1_2_nav_gui_console_script_is_registered():
    setup = (TOOLS / "setup.py").read_text(encoding="utf-8")

    assert "h1_2_nav_gui = h1_2_nav_tools.h1_2_nav_gui:main" in setup


def test_h1_2_nav_gui_declares_pyqt5_runtime_dependency():
    package_xml = (TOOLS / "package.xml").read_text(encoding="utf-8")

    assert "<exec_depend>python3-pyqt5</exec_depend>" in package_xml


def test_h1_2_nav_gui_manages_navigation_and_rviz_processes():
    source = GUI.read_text(encoding="utf-8")

    assert "QProcess" in source
    assert "run_ai_slam_nav.sh" in source
    assert "stop_ai_slam_nav.sh" in source
    assert "h1_2_nav2_rviz.launch.py" in source
    assert "rviz_mode_combo" in source
    assert "mode:={rviz_mode}" in source
    assert "voice_process" in source
    assert "h1_2_voice_control h1_2_voice_control.launch.py" in source
    assert "Start Voice Control" in source
    assert "Stop Voice Control" in source


def test_h1_2_nav_gui_generates_route_and_gesture_arguments():
    source = GUI.read_text(encoding="utf-8")

    assert "memory_route=" in source
    assert "gesture_route=" in source
    assert "build_memory_route" in source
    assert "build_gesture_route" in source
    assert "wave_hand" in source
    assert "shake_hand" in source


def test_h1_2_nav_gui_uses_route_and_gesture_services():
    source = GUI.read_text(encoding="utf-8")

    assert "StartRoute" in source
    assert "StopRoute" in source
    assert "RouteStatus" in source
    assert "ExecuteGesture" in source
    assert "start_memory_route" in source
    assert "stop_memory_route" in source
    assert "memory_route_status" in source
    assert "/h1_2_loco/execute_gesture" in source
