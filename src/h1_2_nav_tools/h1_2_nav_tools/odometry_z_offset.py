import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdometryZOffset(Node):
    def __init__(self):
        super().__init__("odometry_z_offset")
        self.declare_parameter("input_topic", "/slam_odom")
        self.declare_parameter("output_topic", "/slam_odom_visual")
        self.declare_parameter("z_offset", 0.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.z_offset = float(self.get_parameter("z_offset").value)

        self.pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.sub = self.create_subscription(Odometry, self.input_topic, self.on_odom, 10)
        self.get_logger().info(
            f"Visual odometry z offset: {self.input_topic} -> {self.output_topic}, z_offset={self.z_offset}"
        )

    def on_odom(self, msg):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        out.pose.pose.position.z = msg.pose.pose.position.z + self.z_offset
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryZOffset()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
