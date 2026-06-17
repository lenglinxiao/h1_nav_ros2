import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from .transform_utils import transform_pose


class LocalToMapOdom(Node):
    def __init__(self):
        super().__init__("local2map_odom")
        self.declare_parameter("odom_topic", "/slam_odom")
        self.declare_parameter("pose_topic", "/odom_in_map")
        self.declare_parameter("target_frame", "map")
        self.target_frame = self.get_parameter("target_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, self.get_parameter("pose_topic").value, 10)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10)

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, pose.header.frame_id, rclpy.time.Time())
            self.pub.publish(transform_pose(pose, transform))
        except Exception as exc:
            self.get_logger().warn(f"Transform to {self.target_frame} failed: {exc}", throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = LocalToMapOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
