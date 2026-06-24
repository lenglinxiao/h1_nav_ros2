from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / "src" / "h1_2_nav_tools"
WEB_GUI = TOOLS / "h1_2_nav_tools" / "h1_2_nav_web_gui.py"
RUN_SCRIPT = ROOT / "run_ai_slam_nav.sh"


def test_h1_2_nav_web_gui_console_script_is_registered():
    setup = (TOOLS / "setup.py").read_text(encoding="utf-8")

    assert "h1_2_nav_web_gui = h1_2_nav_tools.h1_2_nav_web_gui:main" in setup


def test_h1_2_nav_web_gui_serves_localhost_console():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "ThreadingHTTPServer" in source
    assert "127.0.0.1" in source
    assert "8765" in source
    assert "bind_http_server" in source
    assert "EADDRINUSE" in source
    assert "--port-tries" in source
    assert "INDEX_HTML" in source


def test_h1_2_nav_web_gui_uses_yinwang_team_logo():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "引望" in source
    assert "工具装备团队" in source
    assert "logo-rule" in source
    assert "logo-sub" in source
    assert "H1-2 Navigation Console" not in source
    assert "Online/offline modes" not in source


def test_h1_2_nav_web_gui_enforces_single_instance_by_default():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "find_existing_web_gui_pids" in source
    assert "stop_existing_web_guis" in source
    assert "ancestor_pids" in source
    assert "protected_pids" in source
    assert "--no-single-instance" in source
    assert "SIGTERM" in source
    assert "SIGKILL" in source


def test_h1_2_nav_web_gui_manages_navigation_and_rviz_processes():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "run_ai_slam_nav.sh" in source
    assert "stop_ai_slam_nav.sh" in source
    assert "h1_2_nav2_rviz.launch.py" in source
    assert "mode:={rviz_mode}" in source
    assert "rviz_mode" in source
    assert "跟随 Run Mode" in source
    assert "subprocess.Popen" in source
    assert "voice_process" in source
    assert "voice_bridge_process" in source
    assert '"h1_2_voice_control", "h1_2_voice_control.launch.py"' in source
    assert "h1_2_voice_action_bridge.launch.py" in source
    assert "voice_remote_host" in source
    assert "Voice SSH" in source
    assert "ssh" in source


def test_h1_2_nav_web_gui_uses_ros_services():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "StartRoute" in source
    assert "StopRoute" in source
    assert "RouteStatus" in source
    assert "ExecuteGesture" in source
    assert "start_memory_route" in source
    assert "stop_memory_route" in source
    assert "memory_route_status" in source
    assert "/h1_2_loco/execute_gesture" in source


def test_h1_2_nav_web_gui_exposes_expected_api_routes():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "/api/navigation/start" in source
    assert "/api/navigation/stop_all" in source
    assert "/api/rviz/open" in source
    assert "/api/rviz/close" in source
    assert "/api/route/start" in source
    assert "/api/route/stop" in source
    assert "/api/gesture" in source
    assert "/api/voice/start" in source
    assert "/api/voice/stop" in source
    assert "/api/logs" in source


def test_h1_2_nav_web_gui_can_start_voice_control_on_robot_over_ssh():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "H1_2_VOICE_REMOTE" in source
    assert "H1_2_VOICE_WORKSPACE" in source
    assert "H1_2_VOICE_ROS_SETUP" in source
    assert "def normalize_ssh_target" in source
    assert "def normalize_voice_workspace" in source
    assert "def quote_remote_path" in source
    assert 'text.startswith("ssh ")' in source
    assert "def _voice_start_command" in source
    assert "def _voice_stop_command" in source
    assert "bash -lc" in source
    assert '"~/h1_nav2_ros2_ws", "/home/unitree/h1_nav2_ros2_ws"' in source
    assert '"~/ros2_ws"' in source
    assert "cd {quote_remote_path(workspace)}" in source
    assert "pkill -f 'h1_2_voice_control.launch.py|h1_2_voice_control'" in source
    assert "ros2 launch h1_2_voice_control h1_2_voice_control.launch.py" in source


def test_h1_2_nav_web_gui_filters_polling_get_requests_from_log():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert 'if source == "http":' in source
    assert "def log_message(self, fmt, *args):\n        return" in source
    assert 'if (entry.source === "http") continue;' in source


def test_h1_2_nav_web_gui_does_not_force_log_autoscroll_during_selection():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "logsAutoFollow" in source
    assert "logsSelecting" in source
    assert "logsHasSelection" in source
    assert "setupLogScrollLock" in source
    assert "followLogsIfAllowed(shouldFollow)" in source


def test_h1_2_nav_web_gui_supports_online_offline_and_save_modes():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "run_mode" in source
    assert 'value="online"' in source
    assert 'value="navigation"' in source
    assert "save_map" in source
    assert "save_pcd_map" in source
    assert "map_prefix" in source
    assert "pcd_prefix" in source


def test_run_script_still_exposes_online_and_offline_modes():
    script = RUN_SCRIPT.read_text(encoding="utf-8")

    assert "mode=online" in script
    assert "mode=navigation" in script
    assert "mode=save_map" in script
    assert "mode=save_pcd_map" in script
