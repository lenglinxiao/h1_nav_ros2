import rclpy
from geometry_msgs.msg import Twist
from h1_2_nav_msgs.srv import ExecuteGesture
from rclpy.node import Node
from std_msgs.msg import Int32

from h1_2_voice_control.h1_2_voice_control import (
    DEFAULT_CODE_BACKWARD,
    DEFAULT_CODE_FORWARD,
    DEFAULT_CODE_LEFT,
    DEFAULT_CODE_RIGHT,
    DEFAULT_CODE_STOP,
    DEFAULT_CODE_WAVE,
    parse_code_list,
)


class H12VoiceActionBridge(Node):
    def __init__(self):
        super().__init__("h1_2_voice_action_bridge")
        self.declare_parameter("voice_command_topic", "/h1_2/voice_command")
        self.declare_parameter("voice_feedback_topic", "/h1_2/voice_feedback")
        self.declare_parameter("cmd_topic", "/cmd_vel_voice")
        self.declare_parameter("gesture_service_name", "/h1_2_loco/execute_gesture")
        self.declare_parameter("forward_speed", 0.12)
        self.declare_parameter("backward_speed", -0.08)
        self.declare_parameter("turn_yaw_speed", 0.30)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("continuous_move", True)
        self.declare_parameter("code_stop", DEFAULT_CODE_STOP)
        self.declare_parameter("code_forward", DEFAULT_CODE_FORWARD)
        self.declare_parameter("code_backward", DEFAULT_CODE_BACKWARD)
        self.declare_parameter("code_left", DEFAULT_CODE_LEFT)
        self.declare_parameter("code_right", DEFAULT_CODE_RIGHT)
        self.declare_parameter("code_wave", DEFAULT_CODE_WAVE)

        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.backward_speed = float(self.get_parameter("backward_speed").value)
        self.turn_yaw_speed = float(self.get_parameter("turn_yaw_speed").value)
        self.continuous_move = bool(self.get_parameter("continuous_move").value)
        self.code_stop = set(parse_code_list(self.get_parameter("code_stop").value))
        self.code_forward = set(parse_code_list(self.get_parameter("code_forward").value))
        self.code_backward = set(parse_code_list(self.get_parameter("code_backward").value))
        self.code_left = set(parse_code_list(self.get_parameter("code_left").value))
        self.code_right = set(parse_code_list(self.get_parameter("code_right").value))
        self.code_wave = set(parse_code_list(self.get_parameter("code_wave").value))

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_topic").value, 10)
        self.feedback_pub = self.create_publisher(
            Int32, self.get_parameter("voice_feedback_topic").value, 10
        )
        self.voice_sub = self.create_subscription(
            Int32, self.get_parameter("voice_command_topic").value, self.handle_voice_command, 10
        )
        self.gesture_client = self.create_client(
            ExecuteGesture, self.get_parameter("gesture_service_name").value
        )

        self.current_cmd = Twist()
        self.zero_burst_count = 0

        rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self.create_timer(1.0 / rate, self.publish_current_command)
        self.get_logger().info(
            f"H1-2 voice action bridge ready: "
            f"voice_command_topic={self.get_parameter('voice_command_topic').value}, "
            f"cmd_topic={self.get_parameter('cmd_topic').value}, "
            f"gesture_service={self.get_parameter('gesture_service_name').value}"
        )

    def handle_voice_command(self, message):
        code = int(message.data)
        if code in self.code_stop:
            self.stop_motion()
            self.get_logger().info(f"voice code={code}: stop")
            self.publish_feedback(code)
        elif code in self.code_forward:
            self.set_motion(self.forward_speed, 0.0)
            self.get_logger().info(f"voice code={code}: forward x={self.forward_speed:.3f}")
            self.publish_feedback(code)
        elif code in self.code_backward:
            self.set_motion(self.backward_speed, 0.0)
            self.get_logger().info(f"voice code={code}: backward x={self.backward_speed:.3f}")
            self.publish_feedback(code)
        elif code in self.code_left:
            self.set_motion(0.0, self.turn_yaw_speed)
            self.get_logger().info(f"voice code={code}: left yaw={self.turn_yaw_speed:.3f}")
            self.publish_feedback(code)
        elif code in self.code_right:
            self.set_motion(0.0, -self.turn_yaw_speed)
            self.get_logger().info(f"voice code={code}: right yaw={-self.turn_yaw_speed:.3f}")
            self.publish_feedback(code)
        elif code in self.code_wave:
            self.stop_motion()
            self.get_logger().info(f"voice code={code}: wave_hand")
            self.execute_wave_hand(code)
        else:
            self.get_logger().warn(f"unknown voice code={code}")

    def publish_feedback(self, code):
        message = Int32()
        message.data = int(code)
        self.feedback_pub.publish(message)

    def set_motion(self, linear_x, angular_z):
        self.current_cmd = Twist()
        self.current_cmd.linear.x = float(linear_x)
        self.current_cmd.angular.z = float(angular_z)
        self.zero_burst_count = 0
        if not self.continuous_move:
            self.zero_burst_count = 1

    def stop_motion(self):
        self.current_cmd = Twist()
        self.zero_burst_count = 5

    def publish_current_command(self):
        self.cmd_pub.publish(self.current_cmd)
        if self.zero_burst_count > 0:
            self.zero_burst_count -= 1
            if self.zero_burst_count == 0:
                self.current_cmd = Twist()

    def execute_wave_hand(self, feedback_code):
        if not self.gesture_client.service_is_ready():
            self.get_logger().warn(
                f"Gesture service is not available: {self.get_parameter('gesture_service_name').value}"
            )
            return
        request = ExecuteGesture.Request()
        request.name = "wave_hand"
        future = self.gesture_client.call_async(request)
        future.add_done_callback(lambda done: self.gesture_done(done, feedback_code))

    def gesture_done(self, future, feedback_code):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f"wave_hand gesture failed: {exc}")
            return
        if not result.success:
            self.get_logger().warn(
                f"wave_hand gesture returned sdk_ret={result.sdk_ret}: {result.message}"
            )
            return
        self.publish_feedback(feedback_code)


def main():
    rclpy.init()
    node = H12VoiceActionBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
