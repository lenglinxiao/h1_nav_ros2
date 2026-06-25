import sys
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "h1_2_nav_tools"))

from h1_2_nav_tools.camera_stream import (  # noqa: E402
    CameraFrameStore,
    encode_depth_jpeg,
    encode_rgb_jpeg,
)


def decode_jpeg(data):
    return cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)


def test_encode_rgb8_and_bgr8_as_jpeg():
    rgb = np.zeros((8, 10, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255

    rgb_jpeg = decode_jpeg(encode_rgb_jpeg(rgb, "rgb8"))
    bgr_jpeg = decode_jpeg(encode_rgb_jpeg(rgb[:, :, ::-1], "bgr8"))

    assert rgb_jpeg.shape == (8, 10, 3)
    assert bgr_jpeg.shape == (8, 10, 3)
    assert int(rgb_jpeg[4, 5, 2]) > 200
    assert int(bgr_jpeg[4, 5, 2]) > 200


def test_encode_16uc1_depth_as_colorized_jpeg():
    depth_mm = np.repeat(
        np.repeat(
            np.array([[0, 300, 1000], [2500, 5000, 6000]], dtype=np.uint16),
            20,
            axis=0,
        ),
        20,
        axis=1,
    )

    image = decode_jpeg(encode_depth_jpeg(depth_mm, "16UC1", 0.3, 5.0))

    assert image.shape == (40, 60, 3)
    assert int(image[10, 10].max()) < 20
    assert int(image[30, 50].max()) < 20
    assert int(image[10, 50].max()) > 20


def test_encode_32fc1_depth_as_colorized_jpeg():
    depth_m = np.repeat(
        np.repeat(
            np.array([[np.nan, 0.3, 1.0], [2.5, 5.0, np.inf]], dtype=np.float32),
            20,
            axis=0,
        ),
        20,
        axis=1,
    )

    image = decode_jpeg(encode_depth_jpeg(depth_m, "32FC1", 0.3, 5.0))

    assert image.shape == (40, 60, 3)
    assert int(image[10, 10].max()) < 20
    assert int(image[30, 50].max()) < 20
    assert int(image[10, 50].max()) > 20


def test_frame_store_reports_waiting_then_online_and_stale():
    clock = [100.0]
    store = CameraFrameStore(stale_after=2.0, clock=lambda: clock[0])

    waiting = store.status("/rgb", "/depth", "/info")
    assert waiting["state"] == "waiting"
    assert waiting["message"] == "等待相机话题"

    store.update_rgb(b"rgb", 640, 480, "rgb8")
    store.update_depth(b"depth", 640, 480, "16UC1")
    online = store.status("/rgb", "/depth", "/info")
    assert online["state"] == "online"
    assert online["rgb"]["resolution"] == "640x480"
    assert online["depth"]["encoding"] == "16UC1"

    clock[0] += 3.0
    stale = store.status("/rgb", "/depth", "/info")
    assert stale["state"] == "stale"
    assert stale["message"] == "相机话题已超时"


def test_frame_store_reports_annotated_stream_fps():
    clock = [10.0]
    store = CameraFrameStore(stale_after=2.0, clock=lambda: clock[0])

    store.update_annotated(b"first", 640, 480, "bgr8")
    clock[0] += 0.1
    store.update_annotated(b"second", 640, 480, "bgr8")

    status = store.stream_status("annotated")
    assert status["online"]
    assert status["fps"] == 10.0
