import json
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class MemoryPointMarkers(Node):
    def __init__(self):
        super().__init__("memory_point_markers")
        self.declare_parameter("points_file", "maps/memory_points.json")
        self.declare_parameter("marker_topic", "/memory_points_markers")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("marker_z", 0.08)
        self.declare_parameter("text_z", 0.45)

        self.points_file = Path(self.get_parameter("points_file").value).expanduser()
        self.marker_topic = self.get_parameter("marker_topic").value
        self.marker_z = float(self.get_parameter("marker_z").value)
        self.text_z = float(self.get_parameter("text_z").value)
        period = max(0.2, float(self.get_parameter("publish_period_sec").value))

        self.pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_timer(period, self.publish_markers)
        self.get_logger().info(f"Publishing memory point markers from {self.points_file} on {self.marker_topic}")

    def publish_markers(self):
        marker_array = MarkerArray()

        data = self.load_points()
        frame_id = data.get("frame_id", "map")
        points = data.get("points", {})
        if not isinstance(points, dict):
            self.pub.publish(marker_array)
            return

        for index, name in enumerate(sorted(points.keys())):
            record = points[name]
            position = record.get("position", {})
            orientation = record.get("orientation", {})
            marker_array.markers.append(self.make_sphere(index * 2, name, frame_id, position))
            marker_array.markers.append(self.make_text(index * 2 + 1, name, frame_id, position))
            marker_array.markers.append(self.make_arrow(index + 1000, name, frame_id, position, orientation))

        self.pub.publish(marker_array)

    def load_points(self):
        try:
            return json.loads(self.points_file.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Failed to load memory points {self.points_file}: {exc}", throttle_duration_sec=5.0)
            return {"frame_id": "map", "points": {}}

    def make_sphere(self, marker_id, name, frame_id, position):
        marker = self.base_marker(marker_id, name, frame_id, Marker.SPHERE)
        marker.pose.position.x = float(position.get("x", 0.0))
        marker.pose.position.y = float(position.get("y", 0.0))
        marker.pose.position.z = self.marker_z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.08
        marker.color.r, marker.color.g, marker.color.b = self.color_for_name(name)
        marker.color.a = 0.95
        return marker

    def make_text(self, marker_id, name, frame_id, position):
        marker = self.base_marker(marker_id, name + "_label", frame_id, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = float(position.get("x", 0.0))
        marker.pose.position.y = float(position.get("y", 0.0))
        marker.pose.position.z = self.text_z
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.28
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = name
        return marker

    def make_arrow(self, marker_id, name, frame_id, position, orientation):
        marker = self.base_marker(marker_id, name + "_heading", frame_id, Marker.ARROW)
        marker.pose.position.x = float(position.get("x", 0.0))
        marker.pose.position.y = float(position.get("y", 0.0))
        marker.pose.position.z = self.marker_z + 0.06
        marker.pose.orientation.x = float(orientation.get("x", 0.0))
        marker.pose.orientation.y = float(orientation.get("y", 0.0))
        marker.pose.orientation.z = float(orientation.get("z", 0.0))
        marker.pose.orientation.w = float(orientation.get("w", 1.0))
        marker.scale.x = 0.45
        marker.scale.y = 0.06
        marker.scale.z = 0.06
        marker.color.r = 1.0
        marker.color.g = 0.9
        marker.color.b = 0.1
        marker.color.a = 0.9
        return marker

    def base_marker(self, marker_id, ns, frame_id, marker_type):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = Time()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        return marker

    @staticmethod
    def color_for_name(name):
        if name == "START":
            return 0.1, 0.9, 0.2
        palette = [
            (0.1, 0.55, 1.0),
            (1.0, 0.35, 0.25),
            (0.75, 0.35, 1.0),
            (1.0, 0.75, 0.1),
        ]
        return palette[sum(ord(ch) for ch in name) % len(palette)]


def main(args=None):
    rclpy.init(args=args)
    node = MemoryPointMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
