import os
import threading
import time
from importlib import import_module
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from h1_2_vision.detector_core import (
    LatestFrameSlot,
    RateLimiter,
    draw_detections,
    parse_yolo_result,
)


class YoloDetector(Node):
    def __init__(self):
        super().__init__("h1_2_yolo_detector")
        self.declare_parameter("input_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("annotated_topic", "/h1_2/vision/annotated_image")
        self.declare_parameter("detections_topic", "/h1_2/vision/detections")
        self.declare_parameter("model_path", "~/ros2_ws/models/yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.40)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("max_inference_rate", 10.0)
        self.declare_parameter("device", "cpu")

        self.input_topic = self.get_parameter("input_topic").value
        self.annotated_topic = self.get_parameter("annotated_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.model_path = Path(
            os.path.expandvars(os.path.expanduser(self.get_parameter("model_path").value))
        )
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.max_inference_rate = float(self.get_parameter("max_inference_rate").value)
        self.device = str(self.get_parameter("device").value)

        if not self.model_path.is_file():
            raise RuntimeError(f"YOLO model file does not exist: {self.model_path}")

        try:
            yolo_class = import_module("ultralytics").YOLO
        except Exception as exc:
            raise RuntimeError(
                "Ultralytics YOLO is unavailable. Install torch and ultralytics on the robot."
            ) from exc

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = yolo_class(str(self.model_path))
        self.bridge = CvBridge()
        self.latest_frame = LatestFrameSlot()
        self.frame_ready = threading.Event()
        self.stopping = threading.Event()
        self.rate_limiter = RateLimiter(self.max_inference_rate)

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.image_subscription = self.create_subscription(
            Image,
            self.input_topic,
            self._on_image,
            sensor_qos,
        )
        self.annotated_publisher = self.create_publisher(Image, self.annotated_topic, 1)
        self.detections_publisher = self.create_publisher(
            Detection2DArray,
            self.detections_topic,
            10,
        )
        self.worker = threading.Thread(target=self._inference_loop, daemon=True)
        self.worker.start()
        self.get_logger().info(
            f"YOLO ready input={self.input_topic} max_rate={self.max_inference_rate:.1f}Hz "
            f"device={self.device}"
        )

    def destroy_node(self):
        self.stopping.set()
        self.frame_ready.set()
        if hasattr(self, "worker"):
            self.worker.join(timeout=2.0)
        return super().destroy_node()

    def _on_image(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert input image: {exc}")
            return
        self.latest_frame.put((image, message.header))
        self.frame_ready.set()

    def _inference_loop(self):
        while not self.stopping.is_set():
            if not self.frame_ready.wait(timeout=0.1):
                continue
            now = time.monotonic()
            if not self.rate_limiter.ready(now):
                self.stopping.wait(min(0.1, self.rate_limiter.remaining(now)))
                continue
            self.frame_ready.clear()
            item = self.latest_frame.take()
            if item is None:
                continue
            self.rate_limiter.mark(now)
            image, header = item
            try:
                results = self.model.predict(
                    source=image,
                    conf=self.confidence_threshold,
                    iou=self.iou_threshold,
                    imgsz=self.image_size,
                    device=self.device,
                    verbose=False,
                )
                result = results[0]
                detections = parse_yolo_result(result)
                self._publish_results(image, header, detections)
            except Exception as exc:
                self.get_logger().error(f"YOLO inference failed: {exc}")

    def _publish_results(self, image, header, detections):
        annotated = draw_detections(image, detections)
        annotated_message = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_message.header = header
        self.annotated_publisher.publish(annotated_message)

        detection_array = Detection2DArray()
        detection_array.header = header
        for detection in detections:
            message = Detection2D()
            message.header = header
            self._set_bbox(message, detection)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = detection.class_name
            hypothesis.hypothesis.score = detection.confidence
            message.results.append(hypothesis)
            detection_array.detections.append(message)
        self.detections_publisher.publish(detection_array)

    @staticmethod
    def _set_bbox(message, detection):
        center_x = (detection.x_min + detection.x_max) * 0.5
        center_y = (detection.y_min + detection.y_max) * 0.5
        center = message.bbox.center
        if hasattr(center, "position"):
            center.position.x = center_x
            center.position.y = center_y
        else:
            center.x = center_x
            center.y = center_y
        message.bbox.size_x = detection.x_max - detection.x_min
        message.bbox.size_y = detection.y_max - detection.y_min


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDetector()
        rclpy.spin(node)
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f"h1_2_vision startup failed: {exc}", flush=True)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
