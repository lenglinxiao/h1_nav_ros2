import math
import threading
from pathlib import Path as FilePath

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .transform_utils import transform_pose


class RecordTeachingPath(Node):
    def __init__(self):
        super().__init__("record_teaching_path")
        self.declare_parameter("file_path", "teaching_paths.txt")
        self.declare_parameter("distance_threshold", 0.3)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("odom_topic", "/slam_odom")
        self.declare_parameter("path_topic", "/teaching_path")
        self.declare_parameter("interactive_console", True)

        self.file_path = self.get_parameter("file_path").value
        self.distance_threshold = float(self.get_parameter("distance_threshold").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.lock = threading.Lock()
        self.current_path = []
        self.all_paths = []
        self.is_recording = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10)
        self.path_pub = self.create_publisher(
            Path,
            self.get_parameter("path_topic").value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_service(Trigger, "start_teaching_path_recording", self.start_recording_callback)
        self.create_service(Trigger, "stop_and_save_teaching_path", self.stop_and_save_callback)
        self.print_hint()

    def odom_callback(self, msg):
        if not self.is_recording:
            return

        raw_pose = PoseStamped()
        raw_pose.header = msg.header
        raw_pose.pose = msg.pose.pose

        try:
            transform = self.tf_buffer.lookup_transform(self.frame_id, raw_pose.header.frame_id, rclpy.time.Time())
            pose = transform_pose(raw_pose, transform)
        except Exception as exc:
            self.get_logger().warn(f"TF transform failed: {exc}", throttle_duration_sec=2.0)
            return

        with self.lock:
            if not self.current_path:
                self.current_path.append(pose)
            else:
                last = self.current_path[-1].pose.position
                now = pose.pose.position
                if math.dist((last.x, last.y, last.z), (now.x, now.y, now.z)) >= self.distance_threshold:
                    self.current_path.append(pose)
            self.publish_current_path()

    def publish_current_path(self):
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = list(self.current_path)
        self.path_pub.publish(msg)

    def start_recording(self, replace=False):
        with self.lock:
            if replace:
                self.all_paths = []
            self.current_path = []
            self.is_recording = True
        self.publish_current_path()
        self.get_logger().info("Recording started")

    def stop_recording(self):
        with self.lock:
            self.is_recording = False
            if self.current_path:
                self.all_paths.append(list(self.current_path))
                self.get_logger().info(f"Recording stopped, {len(self.current_path)} points buffered")
            else:
                self.get_logger().warn("No path data recorded")
            self.current_path = []

    def save_to_file(self):
        with self.lock:
            if not self.all_paths:
                message = "No teaching path data recorded; file was not changed."
                self.get_logger().warn(message)
                return False, message
            try:
                output_path = FilePath(self.file_path).expanduser()
                output_path.parent.mkdir(parents=True, exist_ok=True)
                with output_path.open("w", encoding="utf-8") as stream:
                    for recorded_path in self.all_paths:
                        for pose in recorded_path:
                            p = pose.pose.position
                            q = pose.pose.orientation
                            stream.write(f"{p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n")
                        stream.write("EOP\n")
                message = f"Teaching path saved: {output_path}"
                self.get_logger().info(message)
                return True, message
            except OSError as exc:
                message = f"Failed to write file {self.file_path}: {exc}"
                self.get_logger().error(message)
                return False, message

    def start_recording_callback(self, request, response):
        del request
        self.start_recording(replace=True)
        response.success = True
        response.message = "Teaching path recording started."
        return response

    def stop_and_save_callback(self, request, response):
        del request
        self.stop_recording()
        response.success, response.message = self.save_to_file()
        return response

    @staticmethod
    def print_hint():
        print("\n--- Path Recorder Console ---")
        print("[s] Start Recording")
        print("[e] End Recording")
        print("[w] Write to File")
        print("[q] Quit")
        print("-----------------------------")

    def console_loop(self):
        while rclpy.ok():
            try:
                command = input("Enter command: ").strip().lower()
            except EOFError:
                return
            if command == "s":
                self.start_recording()
            elif command == "e":
                self.stop_recording()
            elif command == "w":
                self.save_to_file()
            elif command == "q":
                rclpy.shutdown()
                return
            else:
                print("Unknown command")


def main():
    rclpy.init()
    node = RecordTeachingPath()
    try:
        if node.get_parameter("interactive_console").value:
            spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
            spin_thread.start()
            node.console_loop()
        else:
            rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
