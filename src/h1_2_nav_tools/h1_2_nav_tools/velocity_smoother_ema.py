import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class VelocitySmootherEma(Node):
    def __init__(self):
        super().__init__("velocity_smoother_ema")
        self.declare_parameter("alpha_v", 0.4)
        self.declare_parameter("alpha_w", 0.4)
        self.declare_parameter("raw_cmd_topic", "/cmd_vel")
        self.declare_parameter("cmd_topic", "/cmd_vel_smooth")
        self.declare_parameter("cmd_rate", 30.0)
        self.declare_parameter("stop_counter", 3)

        self.alpha_v = float(self.get_parameter("alpha_v").value)
        self.alpha_w = float(self.get_parameter("alpha_w").value)
        self.stop_counter_limit = int(self.get_parameter("stop_counter").value)
        self.stop_counter = 0
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.previous_w = 0.0
        self.cmd = Twist()

        self.create_subscription(Twist, self.get_parameter("raw_cmd_topic").value, self.twist_callback, 10)
        self.pub = self.create_publisher(Twist, self.get_parameter("cmd_topic").value, 10)
        rate = max(float(self.get_parameter("cmd_rate").value), 1.0)
        self.create_timer(1.0 / rate, self.update)

    def twist_callback(self, msg):
        self.cmd = msg
        self.stop_counter = self.stop_counter_limit

    def update(self):
        if self.stop_counter <= 0:
            self.cmd.linear.x = 0.0
            self.cmd.linear.y = 0.0
            self.cmd.angular.z = 0.0
        else:
            self.stop_counter -= 1

        sx = self.alpha_v * self.cmd.linear.x + (1.0 - self.alpha_v) * self.previous_x
        sy = self.alpha_v * self.cmd.linear.y + (1.0 - self.alpha_v) * self.previous_y
        sw = self.alpha_w * self.cmd.angular.z + (1.0 - self.alpha_w) * self.previous_w

        out = Twist()
        out.linear = self.cmd.linear
        out.angular = self.cmd.angular
        out.linear.x = sx
        out.linear.y = sy
        out.angular.z = sw

        self.previous_x = sx
        self.previous_y = sy
        self.previous_w = sw
        self.pub.publish(out)


def main():
    rclpy.init()
    node = VelocitySmootherEma()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
