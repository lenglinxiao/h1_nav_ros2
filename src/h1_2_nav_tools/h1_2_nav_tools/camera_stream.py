import threading
import time
from collections import deque

import cv2
import numpy as np


JPEG_QUALITY = 75


def _encode_jpeg(image):
    ok, encoded = cv2.imencode(
        ".jpg",
        image,
        [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
    )
    if not ok:
        raise ValueError("OpenCV failed to encode JPEG")
    return encoded.tobytes()


def encode_rgb_jpeg(image, encoding):
    encoding = str(encoding or "").lower()
    if encoding == "rgb8":
        bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif encoding == "bgr8":
        bgr = image
    elif encoding == "rgba8":
        bgr = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    elif encoding == "bgra8":
        bgr = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    elif encoding == "mono8":
        bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        raise ValueError(f"unsupported RGB encoding: {encoding}")
    return _encode_jpeg(np.ascontiguousarray(bgr))


def encode_depth_jpeg(image, encoding, min_depth=0.3, max_depth=5.0):
    encoding = str(encoding or "").upper()
    if max_depth <= min_depth:
        raise ValueError("max_depth must be greater than min_depth")
    if encoding in ("16UC1", "MONO16"):
        depth_m = image.astype(np.float32) * 0.001
    elif encoding == "32FC1":
        depth_m = image.astype(np.float32)
    else:
        raise ValueError(f"unsupported depth encoding: {encoding}")

    valid = np.isfinite(depth_m) & (depth_m >= min_depth) & (depth_m <= max_depth)
    normalized = np.zeros(depth_m.shape, dtype=np.uint8)
    normalized[valid] = np.clip(
        (depth_m[valid] - min_depth) * 255.0 / (max_depth - min_depth),
        0,
        255,
    ).astype(np.uint8)
    colorized = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    colorized[~valid] = 0
    return _encode_jpeg(colorized)


class CameraFrameStore:
    def __init__(self, stale_after=2.0, clock=time.monotonic):
        self.stale_after = float(stale_after)
        self.clock = clock
        self.condition = threading.Condition()
        self.frames = {
            "rgb": self._empty_frame(),
            "depth": self._empty_frame(),
            "annotated": self._empty_frame(),
        }
        self.camera_info = {
            "received": False,
            "width": 0,
            "height": 0,
            "last_frame": None,
        }
        self.last_error = ""

    @staticmethod
    def _empty_frame():
        return {
            "jpeg": None,
            "sequence": 0,
            "width": 0,
            "height": 0,
            "encoding": "",
            "last_frame": None,
            "timestamps": deque(maxlen=30),
        }

    def update_rgb(self, jpeg, width, height, encoding):
        self._update("rgb", jpeg, width, height, encoding)

    def update_depth(self, jpeg, width, height, encoding):
        self._update("depth", jpeg, width, height, encoding)

    def update_annotated(self, jpeg, width, height, encoding):
        self._update("annotated", jpeg, width, height, encoding)

    def _update(self, stream, jpeg, width, height, encoding):
        with self.condition:
            frame = self.frames[stream]
            now = self.clock()
            frame.update(
                {
                    "jpeg": jpeg,
                    "sequence": frame["sequence"] + 1,
                    "width": int(width),
                    "height": int(height),
                    "encoding": str(encoding),
                    "last_frame": now,
                }
            )
            frame["timestamps"].append(now)
            self.last_error = ""
            self.condition.notify_all()

    def update_camera_info(self, width, height):
        with self.condition:
            self.camera_info.update(
                {
                    "received": True,
                    "width": int(width),
                    "height": int(height),
                    "last_frame": self.clock(),
                }
            )

    def set_error(self, message):
        with self.condition:
            self.last_error = str(message)

    def wait_for_frame(self, stream, after_sequence=0, timeout=1.0):
        deadline = self.clock() + timeout
        with self.condition:
            while self.frames[stream]["sequence"] <= after_sequence:
                remaining = deadline - self.clock()
                if remaining <= 0:
                    return None
                self.condition.wait(remaining)
            return dict(self.frames[stream])

    def status(self, rgb_topic, depth_topic, info_topic):
        now = self.clock()
        with self.condition:
            rgb = self._stream_status(self.frames["rgb"], now)
            depth = self._stream_status(self.frames["depth"], now)
            info = dict(self.camera_info)
            if not rgb["received"] and not depth["received"]:
                state = "waiting"
                message = self.last_error or "等待相机话题"
            elif rgb["online"] and depth["online"]:
                state = "online"
                message = "RGB 和深度画面在线"
            elif rgb["online"] or depth["online"]:
                state = "partial"
                message = "仅收到部分相机话题"
            else:
                state = "stale"
                message = self.last_error or "相机话题已超时"
            return {
                "state": state,
                "message": message,
                "rgb_topic": rgb_topic,
                "depth_topic": depth_topic,
                "info_topic": info_topic,
                "rgb": rgb,
                "depth": depth,
                "camera_info": info,
            }

    def stream_status(self, stream):
        with self.condition:
            return self._stream_status(self.frames[stream], self.clock())

    def _stream_status(self, frame, now):
        received = frame["last_frame"] is not None
        age = None if not received else max(0.0, now - frame["last_frame"])
        timestamps = frame["timestamps"]
        fps = 0.0
        if len(timestamps) >= 2 and timestamps[-1] > timestamps[0]:
            fps = (len(timestamps) - 1) / (timestamps[-1] - timestamps[0])
        return {
            "received": received,
            "online": received and age <= self.stale_after,
            "age_sec": None if age is None else round(age, 3),
            "resolution": (
                f"{frame['width']}x{frame['height']}"
                if frame["width"] and frame["height"]
                else "-"
            ),
            "encoding": frame["encoding"] or "-",
            "sequence": frame["sequence"],
            "fps": round(fps, 1),
        }
