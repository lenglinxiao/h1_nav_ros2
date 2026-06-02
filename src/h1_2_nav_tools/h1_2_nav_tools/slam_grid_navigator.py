import socket

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from h1_2_nav_tools.pointcloud_to_occupancy_grid import PointCloudToOccupancyGrid
from h1_2_nav_tools.simple_grid_navigator import SimpleGridNavigator
from h1_2_nav_tools.velocity_smoother_ema import VelocitySmootherEma


class UdpH12LocoBridge(Node):
    def __init__(self):
        super().__init__("udp_h1_2_loco_bridge")
        self.declare_parameter("udp_bridge_enabled", True)
        self.declare_parameter("cmd_topic", "/cmd_vel_nav")
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 39001)
        self.declare_parameter("command_duration", 0.5)
        self.declare_parameter("max_x", 0.2)
        self.declare_parameter("max_y", 0.0)
        self.declare_parameter("max_yaw", 0.5)

        self.enabled = self.as_bool(self.get_parameter("udp_bridge_enabled").value)
        self.udp_host = self.get_parameter("udp_host").value
        self.udp_port = int(self.get_parameter("udp_port").value)
        self.command_duration = float(self.get_parameter("command_duration").value)
        self.max_x = float(self.get_parameter("max_x").value)
        self.max_y = float(self.get_parameter("max_y").value)
        self.max_yaw = float(self.get_parameter("max_yaw").value)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(Twist, self.get_parameter("cmd_topic").value, self.cmd_callback, 10)
        self.get_logger().info(
            f"UDP H1-2 loco bridge {'enabled' if self.enabled else 'disabled'}: "
            f"{self.get_parameter('cmd_topic').value} -> {self.udp_host}:{self.udp_port}"
        )

    def cmd_callback(self, msg: Twist):
        if not self.enabled:
            return
        vx = self.clamp(msg.linear.x, -self.max_x, self.max_x)
        vy = self.clamp(msg.linear.y, -self.max_y, self.max_y)
        wz = self.clamp(msg.angular.z, -self.max_yaw, self.max_yaw)
        packet = f"vel {vx:.6f} {vy:.6f} {wz:.6f} {self.command_duration:.3f}"
        self.sock.sendto(packet.encode("ascii"), (self.udp_host, self.udp_port))

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    @staticmethod
    def as_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ("1", "true", "yes", "on")
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    nodes = [
        PointCloudToOccupancyGrid(),
        SimpleGridNavigator(),
        VelocitySmootherEma(),
        UdpH12LocoBridge(),
    ]
    executor = MultiThreadedExecutor(num_threads=3)
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
