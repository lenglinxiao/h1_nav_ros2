import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


class FastLioOdomAdapter(Node):
    def __init__(self):
        super().__init__("fastlio_odom_adapter")
        self.declare_parameter("input_odom_topic", "/slam_odom")
        self.declare_parameter("output_odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("yaw_offset", math.pi)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("planarize", True)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.yaw_offset = float(self.get_parameter("yaw_offset").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.planarize = bool(self.get_parameter("planarize").value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter("output_odom_topic").value, 20
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("input_odom_topic").value,
            self.odom_callback,
            20,
        )

    def odom_callback(self, msg):
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose = msg.pose
        out.twist = msg.twist

        out.pose.pose.position.z = 0.0 if self.planarize else msg.pose.pose.position.z
        yaw = yaw_from_quaternion(msg.pose.pose.orientation) + self.yaw_offset
        q = quaternion_from_yaw(yaw)
        out.pose.pose.orientation.x = q["x"]
        out.pose.pose.orientation.y = q["y"]
        out.pose.pose.orientation.z = q["z"]
        out.pose.pose.orientation.w = q["w"]

        self.odom_pub.publish(out)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header = out.header
            tf_msg.child_frame_id = out.child_frame_id
            tf_msg.transform.translation.x = out.pose.pose.position.x
            tf_msg.transform.translation.y = out.pose.pose.position.y
            tf_msg.transform.translation.z = out.pose.pose.position.z
            tf_msg.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = FastLioOdomAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
