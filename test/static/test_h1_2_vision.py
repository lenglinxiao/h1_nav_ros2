import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
VISION = ROOT / "src" / "h1_2_vision"
sys.path.insert(0, str(VISION))

from h1_2_vision.detector_core import (  # noqa: E402
    Detection,
    LatestFrameSlot,
    RateLimiter,
    draw_detections,
    parse_yolo_result,
)


def test_h1_2_vision_package_and_launch_contract():
    package_xml = (VISION / "package.xml").read_text(encoding="utf-8")
    setup = (VISION / "setup.py").read_text(encoding="utf-8")
    launch = (VISION / "launch" / "yolo_detector.launch.py").read_text(encoding="utf-8")
    node = (VISION / "h1_2_vision" / "yolo_detector.py").read_text(encoding="utf-8")

    assert "<name>h1_2_vision</name>" in package_xml
    assert "<exec_depend>vision_msgs</exec_depend>" in package_xml
    assert "yolo_detector = h1_2_vision.yolo_detector:main" in setup
    assert "/camera/camera/color/image_raw" in launch
    assert "/h1_2/vision/annotated_image" in launch
    assert "/h1_2/vision/detections" in launch
    assert "~/ros2_ws/models/yolov8n.pt" in launch
    assert '"confidence_threshold", default_value="0.40"' in launch
    assert '"iou_threshold", default_value="0.45"' in launch
    assert '"image_size", default_value="640"' in launch
    assert '"max_inference_rate", default_value="10.0"' in launch
    assert '"device", default_value="cpu"' in launch
    assert "from ultralytics import YOLO" not in node
    assert 'import_module("ultralytics")' in node


def test_latest_frame_slot_replaces_old_frames():
    slot = LatestFrameSlot()

    slot.put("first")
    slot.put("second")

    assert slot.take() == "second"
    assert slot.take() is None


def test_rate_limiter_rejects_frames_before_period():
    limiter = RateLimiter(10.0)

    assert limiter.ready(1.0)
    limiter.mark(1.0)
    assert not limiter.ready(1.05)
    assert limiter.ready(1.10)


class FakeTensor:
    def __init__(self, values):
        self.values = np.asarray(values)

    def cpu(self):
        return self

    def numpy(self):
        return self.values


class FakeBoxes:
    xyxy = FakeTensor([[10.0, 20.0, 110.0, 220.0]])
    conf = FakeTensor([0.87])
    cls = FakeTensor([0])


class FakeResult:
    boxes = FakeBoxes()
    names = {0: "person"}


def test_parse_yolo_result_and_draw_label():
    detections = parse_yolo_result(FakeResult())

    assert detections == [Detection("person", 0.87, 10.0, 20.0, 110.0, 220.0)]

    image = np.zeros((240, 320, 3), dtype=np.uint8)
    annotated = draw_detections(image, detections)

    assert annotated.shape == image.shape
    assert int(annotated.sum()) > 0


def test_parse_empty_yolo_result():
    class EmptyResult:
        boxes = None
        names = {}

    assert parse_yolo_result(EmptyResult()) == []
