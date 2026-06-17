import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class InitialPoseTimeAdapter(Node):
    def __init__(self):
        super().__init__("initialpose_time_adapter")
        self.declare_parameter("input_topic", "/rviz_initialpose")
        self.declare_parameter("output_topic", "/initialpose")
        self.declare_parameter("odom_topic", "/slam_odom")
        self.declare_parameter("global_frame", "map")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.global_frame = self.get_parameter("global_frame").value
        self.latest_odom_stamp = None

        self.pub = self.create_publisher(PoseWithCovarianceStamped, self.output_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, self.input_topic, self.on_pose, 10)
        self.get_logger().info(
            f"Adapting initial pose {self.input_topic} -> {self.output_topic} using timestamps from {self.odom_topic}"
        )

    def on_odom(self, msg):
        self.latest_odom_stamp = msg.header.stamp

    def on_pose(self, msg):
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = self.global_frame
        if self.latest_odom_stamp is not None:
            out.header.stamp = self.latest_odom_stamp
        else:
            out.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().warn("No odom timestamp received yet; using node clock for initial pose")
        out.pose = msg.pose
        self.pub.publish(out)
        self.get_logger().info(
            f"Republished initial pose at stamp {out.header.stamp.sec}.{out.header.stamp.nanosec:09d}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseTimeAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
