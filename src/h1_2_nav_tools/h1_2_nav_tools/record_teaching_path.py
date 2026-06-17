import math
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
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
        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 1)
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

    def start_recording(self):
        with self.lock:
            self.current_path = []
            self.is_recording = True
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
            try:
                with open(self.file_path, "w", encoding="utf-8") as stream:
                    for path in self.all_paths:
                        for pose in path:
                            p = pose.pose.position
                            q = pose.pose.orientation
                            stream.write(f"{p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n")
                        stream.write("EOP\n")
                self.get_logger().info(f"Paths written to file: {self.file_path}")
            except OSError as exc:
                self.get_logger().error(f"Failed to write file {self.file_path}: {exc}")

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
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.console_loop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
