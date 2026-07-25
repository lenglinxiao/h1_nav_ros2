import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / "src" / "h1_2_nav_tools"
RUNNER = TOOLS / "h1_2_nav_tools" / "teaching_route_runner.py"
GUARD = TOOLS / "h1_2_nav_tools" / "route_activity_guard.py"
MEMORY_RUNNER = TOOLS / "h1_2_nav_tools" / "memory_route_runner.py"
SETUP = TOOLS / "setup.py"
LAUNCH = TOOLS / "launch" / "teaching_route_runner.launch.py"
RPP = TOOLS / "config" / "nav2_h1_2_rpp.yaml"
EVENTS = TOOLS / "config" / "teaching_route_events.example.json"
RECORDER = TOOLS / "h1_2_nav_tools" / "record_teaching_path.py"
RECORDER_LAUNCH = TOOLS / "launch" / "record_teaching_path.launch.py"
OFFLINE_RVIZ = TOOLS / "rviz" / "h1_2_nav2_offline.rviz"
CAMERA_INIT_RVIZ = TOOLS / "rviz" / "h1_2_nav2_camera_init.rviz"


def read(path):
    return path.read_text(encoding="utf-8")


def test_teaching_route_runner_registers_follow_path_and_route_services():
    source = read(RUNNER)
    setup = read(SETUP)
    launch = read(LAUNCH)

    assert "class TeachingRouteRunner(Node):" in source
    assert "FollowPath" in source
    assert "ActionClient(self, FollowPath" in source
    assert "TransformListener(self.tf_buffer, self)" in source
    assert 'self.declare_parameter("control_mode", "direct")' in source
    assert 'self.declare_parameter("cmd_topic", "/cmd_vel")' in source
    assert "def direct_tracking_step(self):" in source
    assert "find_nearest_pose_index" in source
    assert "signed_lateral_error" in source
    assert "Direct teaching tracker is off path" in source
    assert "StartRoute" in source
    assert "StopRoute" in source
    assert "RouteStatus" in source
    assert 'self.declare_parameter("start_service", "start_teaching_route")' in source
    assert 'self.declare_parameter("stop_service", "stop_teaching_route")' in source
    assert 'self.declare_parameter("status_service", "teaching_route_status")' in source
    assert 'self.declare_parameter("enable_start_pose_guard", True)' in source
    assert "def validate_start_pose(self, path_msg):" in source
    assert "Teaching route start pose mismatch" in source
    assert 'DeclareLaunchArgument("enable_start_pose_guard", default_value="true")' in launch
    assert 'DeclareLaunchArgument("start_xy_tolerance", default_value="0.5")' in launch
    assert 'DeclareLaunchArgument("control_mode", default_value="direct")' in launch
    assert 'DeclareLaunchArgument("tracking_lookahead_dist", default_value="0.25")' in launch
    assert 'DeclareLaunchArgument("tracking_linear_speed", default_value="0.08")' in launch
    assert "teaching_route_runner = h1_2_nav_tools.teaching_route_runner:main" in setup
    assert 'executable="teaching_route_runner"' in launch


def test_teaching_route_runner_loads_path_events_and_splits_by_memory_points():
    source = read(RUNNER)
    events = json.loads(read(EVENTS))

    assert "load_path_file" in source
    assert "load_route_events" in source
    assert "load_memory_points" in source
    assert "project_event" in source
    assert "build_segments" in source
    assert "nearest_index" in source
    assert "trigger_radius" in source
    assert events["events"][0]["point"] == "A"
    assert events["events"][0]["gesture"] == "wave_hand"
    assert events["events"][0]["voice_code"] == 52


def test_teaching_route_runner_executes_stop_gesture_and_voice_at_events():
    source = read(RUNNER)

    assert "ExecuteGesture" in source
    assert 'self.declare_parameter("gesture_service_name", "/h1_2_loco/execute_gesture")' in source
    assert 'self.declare_parameter("voice_feedback_topic", "/h1_2/voice_feedback")' in source
    assert 'self.declare_parameter("stop_cmd_topic", "/cmd_vel")' in source
    assert "publish_stop_command" in source
    assert "publish_voice_code" in source
    assert "self.voice_pub.publish" in source
    assert "self.gesture_client.call_async" in source
    assert 'self.state = "gesturing"' in source


def test_teaching_route_runner_publishes_the_saved_path_for_rviz():
    source = read(RUNNER)
    launch = read(LAUNCH)

    assert 'self.declare_parameter("visualization_path_topic", "/teaching_route_path")' in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "def publish_teaching_path(self, path_msg):" in source
    assert "self.teaching_path_pub.publish(path_msg)" in source
    assert "self.publish_teaching_path(teaching_path)" in source
    assert "self.publish_teaching_path(path_msg)" in source
    assert 'DeclareLaunchArgument("visualization_path_topic", default_value="/teaching_route_path")' in launch

    for rviz_path in (OFFLINE_RVIZ, CAMERA_INIT_RVIZ):
        rviz = read(rviz_path)
        assert "Name: TeachingRoute" in rviz
        assert "Value: /teaching_route_path" in rviz
        assert "Color: 255; 140; 0" in rviz


def test_teaching_route_uses_low_speed_rpp_for_path_tracking():
    rpp = read(RPP)

    assert 'plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"' in rpp
    assert "desired_linear_vel: 0.10" in rpp
    assert "lookahead_dist: 0.25" in rpp
    assert "min_lookahead_dist: 0.15" in rpp
    assert "max_lookahead_dist: 0.35" in rpp
    assert "min_approach_linear_velocity: 0.06" in rpp
    assert "regulated_linear_scaling_min_speed: 0.06" in rpp


def test_teaching_path_recorder_exposes_web_controllable_record_and_save_services():
    source = read(RECORDER)
    launch = read(RECORDER_LAUNCH)

    assert "from std_srvs.srv import Trigger" in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert 'self.create_service(Trigger, "start_teaching_path_recording", self.start_recording_callback)' in source
    assert 'self.create_service(Trigger, "stop_and_save_teaching_path", self.stop_and_save_callback)' in source
    assert "def start_recording(self, replace=False):" in source
    assert "self.start_recording(replace=True)" in source
    assert "self.publish_current_path()" in source
    assert "def stop_and_save_callback" in source
    assert "interactive_console" in source
    assert 'self.declare_parameter("required_tf_source_frame", "camera_init")' in source
    assert "def required_tf_available(self):" in source
    assert "Wait for offline localization before recording." in source
    assert 'self.declare_parameter("distance_threshold", 0.1)' in source
    assert 'DeclareLaunchArgument("distance_threshold", default_value="0.1")' in launch
    assert 'DeclareLaunchArgument("interactive_console", default_value="true")' in launch
    assert 'DeclareLaunchArgument("path_topic", default_value="/teaching_path")' in launch
    assert '"path_topic": LaunchConfiguration("path_topic")' in launch


def test_route_runners_use_peer_status_guard_to_avoid_concurrent_control():
    guard = read(GUARD)
    teaching = read(RUNNER)
    memory = read(MEMORY_RUNNER)

    assert "class RouteActivityGuard" in guard
    assert 'INACTIVE_ROUTE_STATES = {"", "idle", "blocked"}' in guard
    assert "wait_for_service" in guard
    assert "status check timed out; refusing to start route" in guard
    assert "is active: state=" in guard
    assert 'self.declare_parameter("peer_status_service", "memory_route_status")' in teaching
    assert 'self.declare_parameter("peer_status_service", "teaching_route_status")' in memory
    assert "self.route_guard.check_peer_inactive()" in teaching
    assert "self.route_guard.check_peer_inactive()" in memory
    assert "MultiThreadedExecutor(num_threads=2)" in teaching
    assert "MultiThreadedExecutor(num_threads=2)" in memory


def test_rpp_config_is_additive_and_uses_regulated_pure_pursuit():
    rpp = read(RPP)
    dwb = read(TOOLS / "config" / "nav2_h1_2.yaml")

    assert "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController" in rpp
    assert 'controller_plugins: ["FollowPath"]' in rpp
    assert "desired_linear_vel: 0.10" in rpp
    assert "required_movement_radius: 0.1" in rpp
    assert "movement_time_allowance: 20.0" in rpp
    assert "min_approach_linear_velocity: 0.06" in rpp
    assert "regulated_linear_scaling_min_speed: 0.06" in rpp
    assert "use_cost_regulated_linear_velocity_scaling: false" in rpp
    assert "use_rotate_to_heading: false" in rpp
    assert "lookahead_dist: 0.25" in rpp
    assert "min_lookahead_dist: 0.15" in rpp
    assert "max_lookahead_dist: 0.35" in rpp
    assert "dwb_core::DWBLocalPlanner" in dwb
    assert "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController" not in dwb


def test_dwb_terminal_rotation_uses_a_practical_speed_and_timeout():
    dwb = read(TOOLS / "config" / "nav2_h1_2.yaml")

    assert "movement_time_allowance: 20.0" in dwb
    assert "min_speed_theta: 0.0" in dwb
