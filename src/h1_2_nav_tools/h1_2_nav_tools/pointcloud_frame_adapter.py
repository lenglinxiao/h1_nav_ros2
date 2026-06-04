import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointCloudFrameAdapter(Node):
    def __init__(self):
        super().__init__("pointcloud_frame_adapter")
        self.declare_parameter("input_topic", "/cloud_registered_body")
        self.declare_parameter("output_topic", "/cloud_nav_body")
        self.declare_parameter("target_frame", "base_link")

        self.target_frame = self.get_parameter("target_frame").value
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("output_topic").value, 10)
        self.create_subscription(
            PointCloud2,
            self.get_parameter("input_topic").value,
            self.cloud_callback,
            10,
        )

    def cloud_callback(self, msg):
        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = self.target_frame
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = msg.data
        out.is_dense = msg.is_dense
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PointCloudFrameAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
