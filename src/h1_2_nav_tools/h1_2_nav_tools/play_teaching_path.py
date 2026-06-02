from pathlib import Path as FilePath

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class PlayTeachingPath(Node):
    def __init__(self):
        super().__init__("play_teaching_path")
        self.declare_parameter("path_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("path_topic", "/teach_path")
        self.declare_parameter("publish_period", 0.5)

        self.frame_id = self.get_parameter("frame_id").value
        self.pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        self.path_msg.poses = self.load_path(self.get_parameter("path_file").value)

        period = float(self.get_parameter("publish_period").value)
        self.create_timer(max(period, 0.05), self.timer_callback)
        self.get_logger().info(f"Loaded teaching path with {len(self.path_msg.poses)} poses")

    def load_path(self, path_file):
        if not path_file:
            self.get_logger().warn("Parameter path_file is empty")
            return []

        poses = []
        try:
            with FilePath(path_file).open("r", encoding="utf-8") as stream:
                for line in stream:
                    line = line.strip()
                    if not line or "EOP" in line:
                        continue
                    parts = [float(value) for value in line.split()]
                    if len(parts) != 7:
                        self.get_logger().warn(f"Invalid path line skipped: {line}")
                        continue
                    pose = PoseStamped()
                    pose.header.frame_id = self.frame_id
                    pose.pose.position.x = parts[0]
                    pose.pose.position.y = parts[1]
                    pose.pose.position.z = parts[2]
                    pose.pose.orientation.x = parts[3]
                    pose.pose.orientation.y = parts[4]
                    pose.pose.orientation.z = parts[5]
                    pose.pose.orientation.w = parts[6]
                    poses.append(pose)
        except OSError as exc:
            self.get_logger().error(f"Failed to load path file {path_file}: {exc}")
        except ValueError as exc:
            self.get_logger().error(f"Failed to parse path file {path_file}: {exc}")
        return poses

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = stamp
        for pose in self.path_msg.poses:
            pose.header.stamp = stamp
        self.pub.publish(self.path_msg)


def main():
    rclpy.init()
    node = PlayTeachingPath()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
