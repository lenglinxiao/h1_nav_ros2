import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from h1_2_nav_msgs.srv import ExecuteGesture, ListMemoryPoints
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Int32

from h1_2_voice_control.h1_2_voice_control import (
    DEFAULT_CODE_FORWARD,
    DEFAULT_CODE_NAV_A,
    DEFAULT_CODE_NAV_B,
    DEFAULT_CODE_NAV_C,
    DEFAULT_CODE_NAV_START,
    DEFAULT_CODE_REACH_HAND,
    DEFAULT_CODE_RETRACT_HAND,
    DEFAULT_CODE_RIGHT,
    DEFAULT_CODE_SQUAT,
    DEFAULT_CODE_STAND,
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
        self.declare_parameter("memory_points_service_name", "list_memory_points")
        self.declare_parameter("navigation_action_name", "navigate_to_pose")
        self.declare_parameter("forward_speed", 0.12)
        self.declare_parameter("turn_yaw_speed", 0.30)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("continuous_move", True)
        self.declare_parameter("code_stop", DEFAULT_CODE_STOP)
        self.declare_parameter("code_forward", DEFAULT_CODE_FORWARD)
        self.declare_parameter("code_squat", DEFAULT_CODE_SQUAT)
        self.declare_parameter("code_stand", DEFAULT_CODE_STAND)
        self.declare_parameter("code_right", DEFAULT_CODE_RIGHT)
        self.declare_parameter("code_nav_a", DEFAULT_CODE_NAV_A)
        self.declare_parameter("code_nav_b", DEFAULT_CODE_NAV_B)
        self.declare_parameter("code_nav_c", DEFAULT_CODE_NAV_C)
        self.declare_parameter("code_nav_start", DEFAULT_CODE_NAV_START)
        self.declare_parameter("code_reach_hand", DEFAULT_CODE_REACH_HAND)
        self.declare_parameter("code_retract_hand", DEFAULT_CODE_RETRACT_HAND)
        self.declare_parameter("code_wave", DEFAULT_CODE_WAVE)

        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.turn_yaw_speed = float(self.get_parameter("turn_yaw_speed").value)
        self.continuous_move = bool(self.get_parameter("continuous_move").value)
        self.code_stop = set(parse_code_list(self.get_parameter("code_stop").value))
        self.code_forward = set(parse_code_list(self.get_parameter("code_forward").value))
        self.code_squat = set(parse_code_list(self.get_parameter("code_squat").value))
        self.code_stand = set(parse_code_list(self.get_parameter("code_stand").value))
        self.code_right = set(parse_code_list(self.get_parameter("code_right").value))
        self.code_nav_a = set(parse_code_list(self.get_parameter("code_nav_a").value))
        self.code_nav_b = set(parse_code_list(self.get_parameter("code_nav_b").value))
        self.code_nav_c = set(parse_code_list(self.get_parameter("code_nav_c").value))
        self.code_nav_start = set(parse_code_list(self.get_parameter("code_nav_start").value))
        self.code_reach_hand = set(parse_code_list(self.get_parameter("code_reach_hand").value))
        self.code_retract_hand = set(parse_code_list(self.get_parameter("code_retract_hand").value))
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
        self.memory_points_client = self.create_client(
            ListMemoryPoints, self.get_parameter("memory_points_service_name").value
        )
        self.action_client = ActionClient(
            self, NavigateToPose, self.get_parameter("navigation_action_name").value
        )

        self.current_cmd = Twist()
        self.zero_burst_count = 0
        self.active_nav_goal_handle = None
        self.nav_sequence = 0

        rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self.create_timer(1.0 / rate, self.publish_current_command)
        self.get_logger().info(
            f"H1-2 voice action bridge ready: "
            f"voice_command_topic={self.get_parameter('voice_command_topic').value}, "
            f"cmd_topic={self.get_parameter('cmd_topic').value}, "
            f"gesture_service={self.get_parameter('gesture_service_name').value}, "
            f"navigation_action={self.get_parameter('navigation_action_name').value}"
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
        elif code in self.code_right:
            self.set_motion(0.0, -self.turn_yaw_speed)
            self.get_logger().info(f"voice code={code}: right yaw={-self.turn_yaw_speed:.3f}")
            self.publish_feedback(code)
        elif code in self.code_squat:
            self.execute_gesture("low_stand", code)
        elif code in self.code_stand:
            self.execute_gesture("high_stand", code)
        elif code in self.code_reach_hand:
            self.execute_gesture("reach_hand", code)
        elif code in self.code_retract_hand:
            self.execute_gesture("retract_hand", code)
        elif code in self.code_wave:
            self.execute_gesture("wave_hand", code)
        elif code in self.code_nav_a:
            self.start_navigation("A", code)
        elif code in self.code_nav_b:
            self.start_navigation("B", code)
        elif code in self.code_nav_c:
            self.start_navigation("C", code)
        elif code in self.code_nav_start:
            self.start_navigation("START", code)
        else:
            self.get_logger().warn(f"unknown voice code={code}")

    def publish_feedback(self, code):
        message = Int32()
        message.data = int(code)
        self.feedback_pub.publish(message)

    def set_motion(self, linear_x, angular_z):
        self.cancel_navigation()
        self.current_cmd = Twist()
        self.current_cmd.linear.x = float(linear_x)
        self.current_cmd.angular.z = float(angular_z)
        self.zero_burst_count = 0
        if not self.continuous_move:
            self.zero_burst_count = 1

    def stop_motion(self):
        self.cancel_navigation()
        self.current_cmd = Twist()
        self.zero_burst_count = 5

    def publish_current_command(self):
        self.cmd_pub.publish(self.current_cmd)
        if self.zero_burst_count > 0:
            self.zero_burst_count -= 1
            if self.zero_burst_count == 0:
                self.current_cmd = Twist()

    def execute_gesture(self, name, feedback_code):
        self.stop_motion()
        if not self.gesture_client.service_is_ready():
            self.get_logger().warn(
                f"Gesture service is not available: {self.get_parameter('gesture_service_name').value}"
            )
            return
        request = ExecuteGesture.Request()
        request.name = name
        self.get_logger().info(f"voice code={feedback_code}: gesture={name}")
        future = self.gesture_client.call_async(request)
        future.add_done_callback(lambda done: self.gesture_done(done, name, feedback_code))

    def gesture_done(self, future, name, feedback_code):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f"{name} gesture failed: {exc}")
            return
        if not result.success:
            self.get_logger().warn(
                f"{name} gesture returned sdk_ret={result.sdk_ret}: {result.message}"
            )
            return
        self.publish_feedback(feedback_code)

    def cancel_navigation(self):
        self.nav_sequence += 1
        if self.active_nav_goal_handle is not None:
            self.active_nav_goal_handle.cancel_goal_async()
            self.active_nav_goal_handle = None

    def start_navigation(self, point_name, feedback_code):
        self.stop_motion()
        if not self.memory_points_client.service_is_ready():
            self.get_logger().warn(
                f"Memory point service is not available: "
                f"{self.get_parameter('memory_points_service_name').value}"
            )
            return
        if not self.action_client.server_is_ready():
            self.get_logger().warn(
                f"Nav2 action server is not available: "
                f"{self.get_parameter('navigation_action_name').value}"
            )
            return
        self.nav_sequence += 1
        request_id = self.nav_sequence
        future = self.memory_points_client.call_async(ListMemoryPoints.Request())
        future.add_done_callback(
            lambda done: self.memory_points_done(
                done, point_name, feedback_code, request_id
            )
        )
        self.get_logger().info(f"voice code={feedback_code}: navigation point={point_name}")

    def memory_points_done(self, future, point_name, feedback_code, request_id):
        if request_id != self.nav_sequence:
            return
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Failed to list memory points: {exc}")
            return
        if not result.success or point_name not in result.names:
            self.get_logger().warn(
                f"Memory point is unavailable: {point_name}; {result.message}"
            )
            return
        pose = result.poses[result.names.index(point_name)]
        pose.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self.navigation_goal_response(
                done, point_name, feedback_code, request_id
            )
        )

    def navigation_goal_response(self, future, point_name, feedback_code, request_id):
        if request_id != self.nav_sequence:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Nav2 rejected voice navigation goal: {point_name}")
            return
        self.active_nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self.navigation_done(
                done, point_name, feedback_code, request_id
            )
        )

    def navigation_done(self, future, point_name, feedback_code, request_id):
        if request_id != self.nav_sequence:
            return
        self.active_nav_goal_handle = None
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Voice navigation to {point_name} failed: {exc}")
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f"Voice navigation to {point_name} ended with status={result.status}"
            )
            return
        self.get_logger().info(f"Voice navigation reached memory point: {point_name}")
        self.publish_feedback(feedback_code)


def main():
    rclpy.init()
    node = H12VoiceActionBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
