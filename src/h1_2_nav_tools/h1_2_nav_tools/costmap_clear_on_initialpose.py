import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class CostmapClearOnInitialPose(Node):
    def __init__(self):
        super().__init__("costmap_clear_on_initialpose")
        try:
            from nav2_msgs.srv import ClearEntireCostmap
        except ModuleNotFoundError:
            self.get_logger().error(
                "nav2_msgs is not installed. Install Nav2, for example: "
                "sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup"
            )
            raise

        self.clear_srv_type = ClearEntireCostmap
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter(
            "clear_services",
            ["/global_costmap/clear_entirely_global_costmap", "/local_costmap/clear_entirely_local_costmap"],
        )
        self.clients = [
            self.create_client(self.clear_srv_type, name)
            for name in self.get_parameter("clear_services").get_parameter_value().string_array_value
        ]
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("initialpose_topic").value,
            self.initialpose_callback,
            1,
        )

    def initialpose_callback(self, _msg):
        for client in self.clients:
            if not client.wait_for_service(timeout_sec=0.2):
                self.get_logger().warn(f"Costmap clear service unavailable: {client.srv_name}")
                continue
            future = client.call_async(self.clear_srv_type.Request())
            future.add_done_callback(lambda done, name=client.srv_name: self.log_result(done, name))

    def log_result(self, future, service_name):
        try:
            future.result()
            self.get_logger().info(f"Cleared costmap via {service_name}")
        except Exception as exc:
            self.get_logger().error(f"Failed to clear costmap via {service_name}: {exc}")


def main():
    rclpy.init()
    node = CostmapClearOnInitialPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
