"""Verify the Web GUI isolates Policy requests from its Cyclone DDS node."""

import ast
import json
import os
import subprocess
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock


SOURCE = Path(__file__).resolve().parents[2] / "src/h1_2_nav_tools/h1_2_nav_tools/h1_2_nav_web_gui.py"


def bridge_class():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "RosServiceBridge")
    namespace = {"os": os, "sys": sys, "json": json, "subprocess": subprocess, "threading": threading}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace["RosServiceBridge"]


def test_policy_service_subprocess_uses_fastdds_and_structured_input(monkeypatch):
    bridge_type = bridge_class()
    bridge = bridge_type.__new__(bridge_type)
    bridge.policy_service_lock = threading.Lock()
    monkeypatch.setenv("CYCLONEDDS_URI", "file:///tmp/cyclone.xml")
    monkeypatch.setenv("FASTRTPS_DEFAULT_PROFILES_FILE", "/tmp/old-fastdds.xml")
    completed = subprocess.CompletedProcess([], 0, '{"success":true,"message":"ok"}\n', "")
    run = Mock(return_value=completed)
    monkeypatch.setattr(subprocess, "run", run)

    result = bridge._call_policy_service("arm_hold", {"manual": False})

    assert result == {"success": True, "message": "ok"}
    command = run.call_args.args[0]
    options = run.call_args.kwargs
    assert command[-1] == "h1_2_nav_tools.policy_web_client"
    assert options["env"]["RMW_IMPLEMENTATION"] == "rmw_fastrtps_cpp"
    assert options["env"]["ROS_DOMAIN_ID"] == "0"
    assert "CYCLONEDDS_URI" not in options["env"]
    assert "FASTRTPS_DEFAULT_PROFILES_FILE" not in options["env"]
    assert json.loads(options["input"]) == {"operation": "arm_hold", "manual": False}


def test_policy_service_subprocess_fails_closed_on_invalid_response(monkeypatch):
    bridge_type = bridge_class()
    bridge = bridge_type.__new__(bridge_type)
    bridge.policy_service_lock = threading.Lock()
    run = Mock(return_value=subprocess.CompletedProcess([], 1, "", "service failed"))
    monkeypatch.setattr(subprocess, "run", run)

    result = bridge._call_policy_service("policy_status")

    assert result["success"] is False
    assert "service failed" in result["message"]


def test_policy_client_waits_five_seconds_for_cold_fastdds_service_discovery():
    client_path = SOURCE.parent / "policy_web_client.py"
    tree = ast.parse(client_path.read_text(encoding="utf-8"))
    call = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "_call")
    timeout = next(
        node.value for node in tree.body
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "SERVICE_DISCOVERY_TIMEOUT_SEC" for target in node.targets)
    )
    namespace = {
        "time": __import__("time"),
        "SERVICE_DISCOVERY_TIMEOUT_SEC": ast.literal_eval(timeout),
        "SERVICE_RESPONSE_TIMEOUT_SEC": 4.0,
    }
    exec(compile(ast.Module(body=[call], type_ignores=[]), str(client_path), "exec"), namespace)

    response = type("Response", (), {"success": True, "message": "idle"})()
    future = Mock()
    future.done.return_value = True
    future.result.return_value = response
    client = Mock(wait_for_service=Mock(return_value=True), call_async=Mock(return_value=future))
    node = Mock(create_client=Mock(return_value=client))
    result = namespace["_call"](node, Mock(), object, "/memory_route_status", object())

    assert client.wait_for_service.call_args.kwargs["timeout_sec"] == 5.0
    assert result == {"success": True, "message": "idle"}


def test_policy_client_allows_four_seconds_for_service_response():
    client_path = SOURCE.parent / "policy_web_client.py"
    tree = ast.parse(client_path.read_text(encoding="utf-8"))
    call = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "_call")
    constants = {
        target.id: ast.literal_eval(node.value)
        for node in tree.body if isinstance(node, ast.Assign)
        for target in node.targets if isinstance(target, ast.Name)
        if target.id in ("SERVICE_DISCOVERY_TIMEOUT_SEC", "SERVICE_RESPONSE_TIMEOUT_SEC")
    }
    spins = {"count": 0}

    class Clock:
        now = 0.0

        @classmethod
        def monotonic(cls):
            cls.now += 0.1
            return cls.now

    future = Mock()
    future.done.side_effect = lambda: spins["count"] >= 20
    future.result.return_value = SimpleNamespace(success=True, message="delayed response")
    client = Mock(wait_for_service=Mock(return_value=True), call_async=Mock(return_value=future))
    node = Mock(create_client=Mock(return_value=client))
    executor = Mock()
    executor.spin_once.side_effect = lambda **kwargs: spins.update(count=spins["count"] + 1)
    namespace = {
        "time": SimpleNamespace(monotonic=Clock.monotonic),
        **constants,
    }
    exec(compile(ast.Module(body=[call], type_ignores=[]), str(client_path), "exec"), namespace)

    result = namespace["_call"](node, executor, object, "/memory_route_status", object())

    assert constants["SERVICE_RESPONSE_TIMEOUT_SEC"] == 4.0
    assert spins["count"] == 20
    assert result == {"success": True, "message": "delayed response"}
