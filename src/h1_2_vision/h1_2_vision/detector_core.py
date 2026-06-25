import threading
from dataclasses import dataclass

import cv2


@dataclass(frozen=True)
class Detection:
    class_name: str
    confidence: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float


class LatestFrameSlot:
    def __init__(self):
        self.lock = threading.Lock()
        self.value = None

    def put(self, value):
        with self.lock:
            self.value = value

    def take(self):
        with self.lock:
            value = self.value
            self.value = None
            return value


class RateLimiter:
    def __init__(self, max_rate):
        self.period = 0.0 if max_rate <= 0.0 else 1.0 / float(max_rate)
        self.last_run = None

    def ready(self, now):
        return self.last_run is None or now - self.last_run + 1e-9 >= self.period

    def remaining(self, now):
        if self.last_run is None:
            return 0.0
        return max(0.0, self.period - (now - self.last_run))

    def mark(self, now):
        self.last_run = float(now)


def parse_yolo_result(result):
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return []
    xyxy = boxes.xyxy.cpu().numpy()
    confidences = boxes.conf.cpu().numpy()
    class_ids = boxes.cls.cpu().numpy()
    names = getattr(result, "names", {})
    detections = []
    for coordinates, confidence, class_id in zip(xyxy, confidences, class_ids):
        class_index = int(class_id)
        class_name = str(names.get(class_index, class_index))
        detections.append(
            Detection(
                class_name=class_name,
                confidence=float(confidence),
                x_min=float(coordinates[0]),
                y_min=float(coordinates[1]),
                x_max=float(coordinates[2]),
                y_max=float(coordinates[3]),
            )
        )
    return detections


def draw_detections(image, detections):
    annotated = image.copy()
    for detection in detections:
        x_min = max(0, int(round(detection.x_min)))
        y_min = max(0, int(round(detection.y_min)))
        x_max = min(annotated.shape[1] - 1, int(round(detection.x_max)))
        y_max = min(annotated.shape[0] - 1, int(round(detection.y_max)))
        label = f"{detection.class_name} {detection.confidence:.2f}"
        cv2.rectangle(annotated, (x_min, y_min), (x_max, y_max), (0, 220, 0), 2)
        text_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        text_top = max(0, y_min - text_size[1] - baseline - 4)
        cv2.rectangle(
            annotated,
            (x_min, text_top),
            (min(annotated.shape[1] - 1, x_min + text_size[0] + 6), y_min),
            (0, 220, 0),
            -1,
        )
        cv2.putText(
            annotated,
            label,
            (x_min + 3, max(text_size[1], y_min - baseline - 2)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )
    return annotated
