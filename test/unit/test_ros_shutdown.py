import ast
from pathlib import Path
from types import SimpleNamespace

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
NODE_PACKAGE = REPO_ROOT / "src" / "h1_2_nav_tools" / "h1_2_nav_tools"
NODE_FILES = (
    "goal_pose_to_nav2.py",
    "memory_route_runner.py",
    "velocity_smoother_ema.py",
    "memory_point_manager.py",
    "pointcloud_to_occupancy_grid.py",
)


class ExternalShutdownException(Exception):
    pass


class FakeNode:
    def __init__(self):
        self.destroyed = False

    def destroy_node(self):
        self.destroyed = True


def main_function(filename):
    source = (NODE_PACKAGE / filename).read_text()
    module = ast.parse(source, filename=filename)
    return next(
        node for node in module.body
        if isinstance(node, ast.FunctionDef) and node.name == "main"
    )


@pytest.mark.parametrize("filename", NODE_FILES)
@pytest.mark.parametrize("spin_result", ("external_shutdown", "keyboard_interrupt", "runtime_error"))
def test_main_shutdown_is_safe_and_runtime_errors_propagate(filename, spin_result):
    state = {"shutdown": False, "shutdown_calls": 0, "node": None}

    def spin():
        if spin_result == "external_shutdown":
            # The executor's context may already have been shut down by ROS.
            state["shutdown"] = True
            raise ExternalShutdownException()
        if spin_result == "keyboard_interrupt":
            raise KeyboardInterrupt()
        raise ValueError("unexpected runtime failure")

    def try_shutdown():
        state["shutdown_calls"] += 1
        state["shutdown"] = True

    def make_node():
        state["node"] = FakeNode()
        return state["node"]

    class FakeExecutor:
        def __init__(self, **kwargs):
            self.removed = False

        def add_node(self, node):
            pass

        def spin(self):
            spin()

        def remove_node(self, node):
            self.removed = True

    namespace = {
        "rclpy": SimpleNamespace(init=lambda **kwargs: None, spin=lambda node: spin(), try_shutdown=try_shutdown),
        "ExternalShutdownException": ExternalShutdownException,
        "GoalPoseToNav2": make_node,
        "MemoryRouteRunner": make_node,
        "VelocitySmootherEma": make_node,
        "MemoryPointManager": make_node,
        "PointCloudToOccupancyGrid": make_node,
        "MultiThreadedExecutor": FakeExecutor,
    }
    exec(compile(ast.Module(body=[main_function(filename)], type_ignores=[]), filename, "exec"), namespace)

    if spin_result == "runtime_error":
        with pytest.raises(ValueError, match="unexpected runtime failure"):
            namespace["main"]()
    else:
        assert namespace["main"]() is None

    assert state["node"].destroyed
    assert state["shutdown_calls"] == 1
