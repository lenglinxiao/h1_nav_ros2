"""Test the browser-loss watchdog with process and ROS stand-ins only."""
import ast
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from types import MethodType
from unittest.mock import Mock


SOURCE = Path(__file__).resolve().parents[2] / "src/h1_2_nav_tools/h1_2_nav_tools/h1_2_nav_web_gui.py"


def method_from(class_name, method_name, namespace=None):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    target_class = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name)
    method = next(node for node in target_class.body if isinstance(node, ast.FunctionDef) and node.name == method_name)
    module = ast.Module(body=[method], type_ignores=[])
    global_namespace = {"HEARTBEAT_TIMEOUT_SEC": 2.0, "threading": threading, "time": time}
    global_namespace.update(namespace or {})
    exec(compile(module, str(SOURCE), "exec"), global_namespace)
    return global_namespace[method_name]


class OneTimeoutThenStop:
    def __init__(self):
        self.calls = 0

    def wait(self, _interval):
        self.calls += 1
        return self.calls > 1


def test_watchdog_stops_navigation_after_stale_heartbeat_without_ros_first():
    events = []
    app = SimpleNamespace()
    app.watchdog_stop = OneTimeoutThenStop()
    app.processes = Mock()
    app.processes.status.return_value = {"navigation": {"running": True}}
    app.processes.stop_navigation.side_effect = lambda reason: events.append(("process", reason))
    app.ros = Mock()
    app.ros.route_stop.side_effect = lambda: events.append(("route", None))
    app.ros.teaching_route_stop.side_effect = lambda: events.append(("teaching", None))
    app.logs = Mock()
    app.heartbeat_lock = threading.Lock()
    app.last_heartbeat = 0.0
    app.heartbeat_expired = False

    app._heartbeat_age = MethodType(method_from("WebGuiApp", "_heartbeat_age"), app)
    method_from("WebGuiApp", "_watch_heartbeat")(app)

    assert events[0] == ("process", "browser heartbeat timeout")
    assert events[1:] == [("route", None), ("teaching", None)]
    assert app.heartbeat_expired is True


def test_watchdog_ignores_fresh_heartbeat_and_absent_navigation():
    for navigation_running, last_heartbeat in ((True, time.monotonic()), (False, 0.0)):
        app = SimpleNamespace(
            watchdog_stop=OneTimeoutThenStop(),
            processes=Mock(),
            ros=Mock(),
            logs=Mock(),
            heartbeat_lock=threading.Lock(),
            last_heartbeat=last_heartbeat,
            heartbeat_expired=False,
        )
        app.processes.status.return_value = {"navigation": {"running": navigation_running}}
        app._heartbeat_age = MethodType(method_from("WebGuiApp", "_heartbeat_age"), app)

        method_from("WebGuiApp", "_watch_heartbeat")(app)

        app.processes.stop_navigation.assert_not_called()
        app.ros.route_stop.assert_not_called()
        assert app.heartbeat_expired is False


def test_ui_only_shutdown_never_stops_robot_processes():
    app = SimpleNamespace(
        ui_only=True,
        watchdog_stop=Mock(),
        processes=Mock(),
        ros=Mock(),
    )

    method_from("WebGuiApp", "shutdown")(app)

    app.processes.shutdown.assert_not_called()
    app.ros.shutdown.assert_called_once_with()


def test_expired_heartbeat_rejects_motion_action():
    class Handler:
        path = "/api/gesture"
        app = SimpleNamespace(ui_only=False, pc4_profile=False, heartbeat_expired=True, logs=Mock())
        _read_json = lambda self: {}
        _send_json = lambda self, data: setattr(self, "response", data)

    handler = Handler()
    method_from("RequestHandler", "do_POST", {"urlparse": __import__("urllib.parse", fromlist=["urlparse"]).urlparse})(handler)

    assert handler.response["success"] is False
    assert "心跳曾超时" in handler.response["message"]


def test_invalid_navigation_attempt_does_not_clear_expiry_but_success_does():
    app = SimpleNamespace(
        task_lock=threading.RLock(),
        heartbeat_lock=threading.Lock(),
        last_heartbeat=time.monotonic(),
        heartbeat_expired=True,
        processes=Mock(),
        pc4_profile=False,
        validate_navigation=Mock(return_value=(False, "invalid")),
        navigation_args=Mock(return_value=["mode=save_map"]),
        save_config=Mock(),
    )
    app._heartbeat_age = MethodType(method_from("WebGuiApp", "_heartbeat_age"), app)
    start_navigation = method_from("WebGuiApp", "start_navigation_task")

    assert start_navigation(app, {"run_mode": "save_map"})["success"] is False
    assert app.heartbeat_expired is True

    app.validate_navigation.return_value = (True, "")
    app.processes.start_navigation.return_value = {"success": True}
    assert start_navigation(app, {"run_mode": "save_map"})["success"] is True
    assert app.heartbeat_expired is True

    assert start_navigation(app, {"run_mode": "online"})["success"] is True
    assert app.heartbeat_expired is False
