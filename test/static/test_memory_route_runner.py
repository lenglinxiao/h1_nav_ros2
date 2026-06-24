from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
RUNNER = ROOT / "src" / "h1_2_nav_tools" / "h1_2_nav_tools" / "memory_route_runner.py"
SETUP = ROOT / "src" / "h1_2_nav_tools" / "setup.py"


def test_memory_route_runner_registers_route_services_and_action_client():
    source = RUNNER.read_text(encoding="utf-8")

    assert "class MemoryRouteRunner(Node):" in source
    assert "StartRoute" in source
    assert "StopRoute" in source
    assert "RouteStatus" in source
    assert "NavigateToPose" in source
    assert "ActionClient" in source


def test_memory_route_runner_loads_points_and_route_names():
    source = RUNNER.read_text(encoding="utf-8")

    assert "points_file" in source
    assert "maps/memory_points.json" in source
    assert "route_names" in source
    assert "[\"A\", \"B\", \"C\"]" in source
    assert "json.loads" in source
    assert "record_to_pose" in source


def test_memory_route_runner_uses_latest_tf_for_long_running_goals():
    source = RUNNER.read_text(encoding="utf-8")

    assert "from builtin_interfaces.msg import Time" in source
    assert "pose.header.stamp = Time()" in source
    assert "pose.header.stamp = self.get_clock().now().to_msg()" not in source


def test_memory_route_runner_loops_successful_goals_and_retries_failures():
    source = RUNNER.read_text(encoding="utf-8")

    assert "GoalStatus.STATUS_SUCCEEDED" in source
    assert "self.current_index = (self.current_index + 1) % len(self.route_names)" in source
    assert "blocked_retry_wait_sec" in source
    assert "max_retries_per_goal" in source
    assert "handle_goal_failure" in source
    assert 'self.state = "blocked"' in source


def test_memory_route_runner_stop_cancels_current_goal():
    source = RUNNER.read_text(encoding="utf-8")

    assert "stop_requested" in source
    assert "cancel_goal_async" in source
    assert 'self.state = "stopping"' in source


def test_memory_route_runner_console_script_is_registered():
    setup = SETUP.read_text(encoding="utf-8")

    assert "memory_route_runner = h1_2_nav_tools.memory_route_runner:main" in setup


def test_memory_route_runner_can_return_to_start_before_looping_route():
    source = RUNNER.read_text(encoding="utf-8")
    launch = (ROOT / "src" / "h1_2_slam_bringup" / "launch" / "h1_2_nav2_navigation.launch.py").read_text(encoding="utf-8")
    script = (ROOT / "run_ai_slam_nav.sh").read_text(encoding="utf-8")

    assert 'self.declare_parameter("return_to_start", False)' in source
    assert 'self.declare_parameter("start_point_name", "START")' in source
    assert "self.returning_to_start" in source
    assert "next_goal_name" in source
    assert "if self.returning_to_start:" in source
    assert "self.returning_to_start = False" in source
    assert "self.current_index = 0" in source
    assert 'f"Route contains unknown memory points: {self.start_point_name}"' in source
    assert 'DeclareLaunchArgument("return_to_start"' in launch
    assert '"return_to_start": ParameterValue(LaunchConfiguration("return_to_start"), value_type=bool)' in launch
    assert '"start_point_name": LaunchConfiguration("initial_pose_name")' in launch
    assert "RETURN_TO_START" in script
    assert "return_to_start=*" in script
    assert '"return_to_start:=${RETURN_TO_START}"' in script


def test_memory_route_runner_executes_configured_gestures_after_arrival():
    source = RUNNER.read_text(encoding="utf-8")
    launch = (ROOT / "src" / "h1_2_slam_bringup" / "launch" / "h1_2_nav2_navigation.launch.py").read_text(encoding="utf-8")
    script = (ROOT / "run_ai_slam_nav.sh").read_text(encoding="utf-8")

    assert "ExecuteGesture" in source
    assert 'self.declare_parameter("gesture_route", "")' in source
    assert 'self.state = "gesturing"' in source
    assert "parse_gesture_route" in source
    assert "handle_goal_arrival" in source
    assert "execute_arrival_gesture" in source
    assert "finish_goal_arrival" in source
    assert "call_async" in source
    assert 'DeclareLaunchArgument("gesture_route"' in launch
    assert '"gesture_route": LaunchConfiguration("gesture_route")' in launch
    assert "GESTURE_ROUTE" in script
    assert "gesture_route=*" in script
    assert 'if [[ -n "${GESTURE_ROUTE}" ]]' in script
    assert 'NAV_LAUNCH_ARGS+=("gesture_route:=${GESTURE_ROUTE}")' in script
