from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / "src" / "h1_2_nav_tools"
WEB_GUI = TOOLS / "h1_2_nav_tools" / "h1_2_nav_web_gui.py"
CAMERA_STREAM = TOOLS / "h1_2_nav_tools" / "camera_stream.py"
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
    assert "start_teaching_route" in source
    assert "stop_teaching_route" in source
    assert "teaching_route_status" in source
    assert "start_teaching_path_recording" in source
    assert "stop_and_save_teaching_path" in source
    assert "/h1_2_loco/execute_gesture" in source


def test_h1_2_nav_web_gui_exposes_expected_api_routes():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "/api/navigation/start" in source
    assert "/api/navigation/stop_all" in source
    assert "/api/rviz/open" in source
    assert "/api/rviz/close" in source
    assert "/api/route/start" in source
    assert "/api/route/stop" in source
    assert "/api/teaching_route/process/start" in source
    assert "/api/teaching_route/process/stop" in source
    assert "/api/teaching_route/start" in source
    assert "/api/teaching_route/stop" in source
    assert "/api/teaching_record/process/start" in source
    assert "/api/teaching_record/process/stop" in source
    assert "/api/teaching_record/start" in source
    assert "/api/teaching_record/stop_save" in source
    assert "/api/gesture" in source
    assert "/api/voice/start" in source
    assert "/api/voice/stop" in source
    assert "/api/logs" in source
    assert "/api/camera/status" in source
    assert "/api/camera/reconnect" in source
    assert "/api/camera/rgb.mjpg" in source
    assert "/api/camera/depth.mjpg" in source
    assert "/api/vision/status" in source
    assert "/api/vision/start" in source
    assert "/api/vision/stop" in source
    assert "/api/vision/annotated.mjpg" in source


def test_h1_2_nav_web_gui_subscribes_to_configurable_realsense_topics():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "/camera/camera/color/image_raw" in source
    assert "/camera/camera/aligned_depth_to_color/image_raw" in source
    assert "/camera/camera/color/camera_info" in source
    assert "camera_rgb_topic" in source
    assert "camera_depth_topic" in source
    assert "camera_info_topic" in source
    assert "QoSReliabilityPolicy.BEST_EFFORT" in source
    assert "QoSHistoryPolicy.KEEP_LAST" in source
    assert "depth=1" in source
    assert "create_subscription" in source
    assert "destroy_subscription" in source


def test_h1_2_nav_web_gui_keeps_camera_processing_off_until_requested():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert '"camera_preview_enabled": False' in source
    assert "self.camera_preview_enabled = False" in source
    assert "def start_camera_preview" in source
    assert "def stop_camera_preview" in source
    assert '"/api/camera/start"' in source
    assert '"/api/camera/stop"' in source
    assert "开启相机预览" in source
    assert "关闭相机预览" in source
    assert 'if not self.camera_preview_enabled:' in source


def test_h1_2_nav_web_gui_has_camera_preview_panel():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "Camera Preview" in source
    assert 'id="camera_rgb"' in source
    assert 'id="camera_depth"' in source
    assert 'id="camera_state"' in source
    assert "重新连接" in source
    assert "等待相机话题" in source
    assert "refreshCameraStatus" in source
    assert 'id="vision_annotated"' in source
    assert 'id="vision_state"' in source
    assert "YOLO Detection" in source
    assert "启动识别" in source
    assert "停止识别" in source


def test_h1_2_nav_web_gui_manages_remote_yolo_without_stopping_camera():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "vision_process" in source
    assert "vision_remote_host" in source
    assert "vision_workspace" in source
    assert "vision_ros_setup" in source
    assert "vision_annotated_topic" in source
    assert "ros2 launch h1_2_vision yolo_detector.launch.py" in source
    assert "pkill -f 'yolo_detector.launch.py|h1_2_vision/yolo_detector|h1_2_vision.*yolo_detector'" in source
    assert "realsense2_camera" not in source[source.index("def _vision_stop_command"):source.index("def open_rviz")]


def test_camera_stream_helpers_are_kept_ros_independent():
    source = CAMERA_STREAM.read_text(encoding="utf-8")

    assert "def encode_rgb_jpeg" in source
    assert "def encode_depth_jpeg" in source
    assert "class CameraFrameStore" in source
    assert "import rclpy" not in source
    assert "from sensor_msgs" not in source


def test_h1_2_nav_tools_declares_camera_preview_dependencies():
    package_xml = (TOOLS / "package.xml").read_text(encoding="utf-8")

    assert "<exec_depend>cv_bridge</exec_depend>" in package_xml
    assert "<exec_depend>python3-opencv</exec_depend>" in package_xml
    assert "<exec_depend>python3-numpy</exec_depend>" in package_xml


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


def test_h1_2_nav_web_gui_defaults_to_manual_offline_route_start():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert '"auto_start_route": False' in source
    assert "payload.get('auto_start_route', False)" in source
    assert 'config.auto_start_route === true' in source


def test_h1_2_nav_web_gui_exposes_teaching_route_controls():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert '"teaching_path_file"' in source
    assert '"teaching_events_file"' in source
    assert '"teaching_loop": True' in source
    assert '"nav2_params_file": ""' in source
    assert 'id="nav2_params_file"' in source
    assert "params_file=" in source
    assert 'id="teaching_path_file"' in source
    assert 'id="teaching_events_file"' in source
    assert "启动示教录制" in source
    assert "开始录制" in source
    assert "结束并保存" in source
    assert "record_teaching_path.launch.py" in source
    assert "path_topic:=/teaching_route_path" in source
    assert "interactive_console:=false" in source
    assert 'id="teaching_loop"' in source
    assert "启动示教节点" in source
    assert "停止示教节点" in source
    assert "开始示教巡航" in source
    assert "停止示教巡航" in source
    assert "startTeachingRoute" in source
    assert "start_teaching_route_button" in source
    assert "teaching_route_runner.launch.py" in source
    assert "teaching_route_process" in source
    assert "processes.teaching_route" in source
    assert "data.teaching_route" in source


def test_h1_2_nav_web_gui_keeps_start_out_of_navigation_route():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "def normalize_memory_route" in source
    assert 'name != "START"' in source
    assert 'config["memory_route"] = normalize_memory_route' in source
    assert "memory_route={normalize_memory_route" in source
    assert 'point.name === "START" ? "disabled" : ""' in source
    assert 'if (name === "START") return;' in source


def test_h1_2_nav_web_gui_migrates_legacy_automatic_route_config():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert 'CONFIG_WORKFLOW_VERSION = 2' in source
    assert 'saved.get("workflow_version", 1) < CONFIG_WORKFLOW_VERSION' in source
    assert 'saved["auto_start_route"] = False' in source
    assert 'config["workflow_version"] = CONFIG_WORKFLOW_VERSION' in source


def test_h1_2_nav_web_gui_gates_route_start_on_stable_localization():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "LocalizationStatus" in source
    assert '"/icp_localization/status"' in source
    assert "LOCALIZATION_REQUIRED_STABLE_SAMPLES = 5" in source
    assert "LOCALIZATION_MAX_SCORE = 0.15" in source
    assert '"manual initial pose accepted"' in source
    assert "localization_status" in source
    assert "定位稳定，可开始巡航" in source
    assert "定位未稳定" in source
    assert 'id="localization_indicator"' in source
    assert "localization-indicator waiting" in source
    assert ".localization-indicator.ready" in source
    assert 'localizationIndicator.className = `localization-indicator ${indicatorState}`' in source
    assert 'id="start_route_button"' in source
    assert "startRoute()" in source
    assert "startButton.disabled = !localization.ready" in source
    assert "LOCALIZATION_STATUS_TIMEOUT_SEC = 10.0" in source


def test_h1_2_nav_web_gui_uses_cli_ros_dds_environment_before_rclpy_init():
    source = WEB_GUI.read_text(encoding="utf-8")

    assert "def configure_ros_environment" in source
    assert 'os.environ["ROS_LOCALHOST_ONLY"] = "0"' in source
    assert 'os.environ.setdefault("ROS_DOMAIN_ID", "0")' in source
    assert 'os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"' in source
    assert 'os.environ.pop("CYCLONEDDS_URI", None)' in source
    assert 'os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)' in source
    assert source.index("configure_ros_environment()") < source.index("rclpy.init(args=None)")


def test_run_script_still_exposes_online_and_offline_modes():
    script = RUN_SCRIPT.read_text(encoding="utf-8")

    assert "mode=online" in script
    assert "mode=navigation" in script
    assert "mode=save_map" in script
    assert "mode=save_pcd_map" in script
    assert "params_file=*" in script
    assert '"params_file:=${NAV2_PARAMS_FILE}"' in script
