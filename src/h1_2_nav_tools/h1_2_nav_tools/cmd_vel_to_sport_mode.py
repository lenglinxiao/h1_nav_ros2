import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_go.msg import SportModeCmd


class CmdVelToSportMode(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_sport_mode")
        self.declare_parameter("cmd_topic", "/cmd_vel_nav")
        self.declare_parameter("sport_topic", "/ros_sport_mode_cmd")
        self.declare_parameter("enabled", False)
        self.declare_parameter("mode", 1)
        self.declare_parameter("gait_type", 1)
        self.declare_parameter("speed_level", 1)
        self.declare_parameter("max_x", 0.25)
        self.declare_parameter("max_y", 0.0)
        self.declare_parameter("max_yaw", 0.6)
        self.declare_parameter("timeout", 0.35)
        self.declare_parameter("publish_rate", 30.0)

        self.enabled = self.as_bool(self.get_parameter("enabled").value)
        self.mode = int(self.get_parameter("mode").value)
        self.gait_type = int(self.get_parameter("gait_type").value)
        self.speed_level = int(self.get_parameter("speed_level").value)
        self.max_x = float(self.get_parameter("max_x").value)
        self.max_y = float(self.get_parameter("max_y").value)
        self.max_yaw = float(self.get_parameter("max_yaw").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(SportModeCmd, self.get_parameter("sport_topic").value, qos)
        self.create_subscription(Twist, self.get_parameter("cmd_topic").value, self.cmd_callback, 10)
        self.create_timer(1.0 / max(float(self.get_parameter("publish_rate").value), 1.0), self.publish)

        self.get_logger().info(
            f"cmd_vel bridge {'ENABLED' if self.enabled else 'disabled'}: "
            f"{self.get_parameter('cmd_topic').value} -> {self.get_parameter('sport_topic').value}"
        )

    def cmd_callback(self, msg: Twist):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def publish(self):
        out = SportModeCmd()
        out.mode = self.mode if self.enabled else 0
        out.gait_type = self.gait_type
        out.speed_level = self.speed_level

        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if self.enabled and age <= self.timeout:
            out.velocity[0] = self.clamp(self.cmd.linear.x, -self.max_x, self.max_x)
            out.velocity[1] = self.clamp(self.cmd.linear.y, -self.max_y, self.max_y)
            out.yaw_speed = self.clamp(self.cmd.angular.z, -self.max_yaw, self.max_yaw)
        else:
            out.velocity[0] = 0.0
            out.velocity[1] = 0.0
            out.yaw_speed = 0.0
        self.pub.publish(out)

    @staticmethod
    def clamp(value: float, low: float, high: float):
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
    node = CmdVelToSportMode()
    try:
        rclpy.spin(node)
    finally:
        node.enabled = True
        node.cmd = Twist()
        for _ in range(3):
            node.publish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
