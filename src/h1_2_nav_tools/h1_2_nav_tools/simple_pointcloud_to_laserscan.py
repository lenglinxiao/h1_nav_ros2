import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2


class SimplePointCloudToLaserScan(Node):
    def __init__(self):
        super().__init__("simple_pointcloud_to_laserscan")
        self.declare_parameter("cloud_topic", "/cloud_nav_body")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("target_frame", "body")
        self.declare_parameter("min_height", -0.90)
        self.declare_parameter("max_height", 0.80)
        self.declare_parameter("angle_min", -3.14159)
        self.declare_parameter("angle_max", 3.14159)
        self.declare_parameter("angle_increment", 0.0087)
        self.declare_parameter("scan_time", 0.1)
        self.declare_parameter("range_min", 0.2)
        self.declare_parameter("range_max", 30.0)
        self.declare_parameter("use_inf", True)
        self.declare_parameter("inf_epsilon", 1.0)

        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.angle_increment = float(self.get_parameter("angle_increment").value)
        self.scan_time = float(self.get_parameter("scan_time").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.use_inf = bool(self.get_parameter("use_inf").value)
        self.inf_epsilon = float(self.get_parameter("inf_epsilon").value)
        self.beam_count = int(math.ceil((self.angle_max - self.angle_min) / self.angle_increment))

        scan_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        scan_qos.durability = DurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(LaserScan, self.scan_topic, scan_qos)
        self.sub = self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, 10)
        self.get_logger().info(
            f"Converting {self.cloud_topic} to {self.scan_topic} in frame {self.target_frame}, "
            f"z [{self.min_height}, {self.max_height}], range [{self.range_min}, {self.range_max}]"
        )

    def on_cloud(self, msg):
        offsets = {field.name: field.offset for field in msg.fields}
        if not {"x", "y", "z"}.issubset(offsets):
            self.get_logger().warn("PointCloud2 is missing x/y/z fields")
            return

        fill = math.inf if self.use_inf else self.range_max + self.inf_epsilon
        ranges = [fill] * self.beam_count
        ox, oy, oz = offsets["x"], offsets["y"], offsets["z"]
        endian = ">" if msg.is_bigendian else "<"
        total = msg.width * msg.height

        for index in range(total):
            base = index * msg.point_step
            x = struct.unpack_from(endian + "f", msg.data, base + ox)[0]
            y = struct.unpack_from(endian + "f", msg.data, base + oy)[0]
            z = struct.unpack_from(endian + "f", msg.data, base + oz)[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if z < self.min_height or z > self.max_height:
                continue
            distance = math.hypot(x, y)
            if distance < self.range_min or distance > self.range_max:
                continue
            angle = math.atan2(y, x)
            if angle < self.angle_min or angle > self.angle_max:
                continue
            beam = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= beam < self.beam_count and distance < ranges[beam]:
                ranges[beam] = distance

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.target_frame or msg.header.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        self.pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = SimplePointCloudToLaserScan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
