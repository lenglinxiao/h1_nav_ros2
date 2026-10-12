import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time


REPO_ROOT = Path(__file__).resolve().parents[2]
NODE_FILE = REPO_ROOT / "src/h1_2_nav_tools/h1_2_nav_tools/pointcloud_to_occupancy_grid.py"
SPEC = importlib.util.spec_from_file_location("pointcloud_to_occupancy_grid", NODE_FILE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_downsample_is_conservative_and_pads_partial_blocks():
    # 3x3 -> 2x2; one occupied cell survives, mixed unknown/free stays unknown,
    # and blocks touching the padded edge stay unknown.
    values = [0, 0, 0, 0, 100, -1, 0, 0, 0]
    reduced, width, height = MODULE.downsample_occupancy_grid(values, 3, 3, 2)
    assert (width, height) == (2, 2)
    assert list(reduced) == [100, -1, -1, -1]
    assert values == [0, 0, 0, 0, 100, -1, 0, 0, 0]


def test_downsample_marks_free_only_when_every_cell_is_free():
    reduced, width, height = MODULE.downsample_occupancy_grid([0, 0, 0, 0], 2, 2, 2)
    assert (width, height, list(reduced)) == (1, 1, [0])
    reduced, _, _ = MODULE.downsample_occupancy_grid([0, -1, 0, 0], 2, 2, 2)
    assert list(reduced) == [-1]


@pytest.mark.parametrize(
    "data,width,height,factor",
    [([], 0, 1, 2), ([0], 1, 1, 0), ([0], 2, 1, 2)],
)
def test_downsample_rejects_invalid_geometry(data, width, height, factor):
    with pytest.raises(ValueError):
        MODULE.downsample_occupancy_grid(data, width, height, factor)


class FakePublisher:
    def __init__(self, subscriptions=1):
        self.subscriptions = subscriptions
        self.messages = []

    def get_subscription_count(self):
        return self.subscriptions

    def publish(self, msg):
        self.messages.append(msg)


class FakeLogger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


def preview_node(subscriptions=1):
    node = MODULE.PointCloudToOccupancyGrid.__new__(MODULE.PointCloudToOccupancyGrid)
    node.preview_pub = FakePublisher(subscriptions)
    node.has_cloud = True
    node.latest_grid = [0, 100, -1, 0]
    node.width = node.height = 2
    node.resolution = 0.05
    node.preview_downsample_factor = 2
    node.preview_period = 2.0
    node.last_preview_publish_monotonic = None
    node.unknown_value = -1
    node.frame_id = "camera_init"
    node.origin_x = -20.0
    node.origin_y = -20.0
    node.map_z = -1.2
    node.logger = FakeLogger()
    node.get_logger = lambda: node.logger
    stamp = Time()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp))
    return node


def test_preview_waits_for_subscriber_and_rate_limits_with_monotonic(monkeypatch):
    ticks = iter((10.0, 10.5, 12.0))
    monkeypatch.setattr(MODULE, "time", SimpleNamespace(monotonic=lambda: next(ticks)))
    node = preview_node(subscriptions=0)
    raw_grid = list(node.latest_grid)

    node.publish_preview_map()
    assert node.preview_pub.messages == []
    node.preview_pub.subscriptions = 1
    node.publish_preview_map()
    node.publish_preview_map()
    assert node.logger.errors == []
    assert len(node.preview_pub.messages) == 1
    node.publish_preview_map()
    assert len(node.preview_pub.messages) == 2

    first = node.preview_pub.messages[0]
    assert (first.info.width, first.info.height) == (1, 1)
    assert first.info.resolution == pytest.approx(0.10)
    assert first.info.origin.position.x == -20.0
    assert first.info.origin.position.y == -20.0
    assert first.info.origin.position.z == -1.2
    assert first.header.frame_id == "camera_init"
    assert list(first.data) == [100]
    assert node.latest_grid == raw_grid
