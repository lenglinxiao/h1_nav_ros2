import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class VelocitySmootherEma(Node):
    def __init__(self):
        super().__init__("velocity_smoother_ema")
        self.declare_parameter("alpha_v", 0.4)
        self.declare_parameter("alpha_w", 0.4)
        self.declare_parameter("raw_cmd_topic", "/cmd_vel")
        self.declare_parameter("priority_cmd_topic", "/cmd_vel_voice")
        self.declare_parameter("cmd_topic", "/cmd_vel_smooth")
        self.declare_parameter("cmd_rate", 30.0)
        self.declare_parameter("stop_counter", 3)
        self.declare_parameter("priority_timeout_sec", 0.6)
        self.declare_parameter("min_linear_speed", 0.0)
        self.declare_parameter("min_linear_speed_max_angular", 0.12)

        self.alpha_v = float(self.get_parameter("alpha_v").value)
        self.alpha_w = float(self.get_parameter("alpha_w").value)
        self.stop_counter_limit = int(self.get_parameter("stop_counter").value)
        self.priority_timeout_sec = float(self.get_parameter("priority_timeout_sec").value)
        self.min_linear_speed = max(0.0, float(self.get_parameter("min_linear_speed").value))
        self.min_linear_speed_max_angular = max(0.0, float(self.get_parameter("min_linear_speed_max_angular").value))
        self.stop_counter = 0
        self.priority_stop_counter = 0
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.previous_w = 0.0
        self.cmd = Twist()
        self.priority_cmd = Twist()
        self.last_priority_time = None

        self.create_subscription(Twist, self.get_parameter("raw_cmd_topic").value, self.twist_callback, 10)
        self.create_subscription(
            Twist, self.get_parameter("priority_cmd_topic").value, self.priority_twist_callback, 10
        )
        self.pub = self.create_publisher(Twist, self.get_parameter("cmd_topic").value, 10)
        rate = max(float(self.get_parameter("cmd_rate").value), 1.0)
        self.create_timer(1.0 / rate, self.update)

    def twist_callback(self, msg):
        self.cmd = msg
        if self.is_zero_twist(msg):
            self.stop_counter = 0
            self.reset_previous()
            self.publish_zero()
            return
        self.stop_counter = self.stop_counter_limit

    def priority_twist_callback(self, msg):
        self.priority_cmd = msg
        if self.is_zero_twist(msg):
            self.priority_stop_counter = 0
            self.last_priority_time = None
            self.reset_previous()
            self.publish_zero()
            return
        self.priority_stop_counter = self.stop_counter_limit
        self.last_priority_time = self.get_clock().now()

    def update(self):
        active_cmd = self.cmd
        if self.priority_active():
            active_cmd = self.priority_cmd
            self.priority_stop_counter -= 1
        elif self.stop_counter <= 0:
            self.cmd.linear.x = 0.0
            self.cmd.linear.y = 0.0
            self.cmd.angular.z = 0.0
        else:
            self.stop_counter -= 1

        if not self.priority_active():
            self.priority_cmd.linear.x = 0.0
            self.priority_cmd.linear.y = 0.0
            self.priority_cmd.angular.z = 0.0

        if self.is_zero_twist(active_cmd):
            self.reset_previous()
            self.publish_zero()
            return

        sx = self.alpha_v * active_cmd.linear.x + (1.0 - self.alpha_v) * self.previous_x
        sy = self.alpha_v * active_cmd.linear.y + (1.0 - self.alpha_v) * self.previous_y
        sw = self.alpha_w * active_cmd.angular.z + (1.0 - self.alpha_w) * self.previous_w
        should_raise_linear_speed = (
            self.min_linear_speed > 0.0
            and abs(active_cmd.linear.x) > 1e-4
            and abs(sx) < self.min_linear_speed
            and abs(sw) <= self.min_linear_speed_max_angular
        )
        if should_raise_linear_speed:
            sx = self.min_linear_speed if active_cmd.linear.x > 0.0 else -self.min_linear_speed

        out = Twist()
        out.linear = active_cmd.linear
        out.angular = active_cmd.angular
        out.linear.x = sx
        out.linear.y = sy
        out.angular.z = sw

        self.previous_x = sx
        self.previous_y = sy
        self.previous_w = sw
        self.pub.publish(out)

    def priority_active(self):
        if self.priority_stop_counter <= 0 or self.last_priority_time is None:
            return False
        age = (self.get_clock().now() - self.last_priority_time).nanoseconds / 1e9
        return age <= self.priority_timeout_sec

    @staticmethod
    def is_zero_twist(msg):
        return (
            abs(msg.linear.x) < 1e-6
            and abs(msg.linear.y) < 1e-6
            and abs(msg.angular.z) < 1e-6
        )

    def reset_previous(self):
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.previous_w = 0.0

    def publish_zero(self):
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.angular.z = 0.0
        out = Twist()
        self.pub.publish(out)

def main():
    rclpy.init()
    node = VelocitySmootherEma()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
