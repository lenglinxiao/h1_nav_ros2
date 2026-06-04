import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalPoseToNav2(Node):
    def __init__(self):
        super().__init__("goal_pose_to_nav2")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("server_timeout_sec", 2.0)

        self.server_timeout_sec = float(self.get_parameter("server_timeout_sec").value)
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter("action_name").value,
        )
        self.active_goal_handle = None
        self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self.goal_callback,
            10,
        )

    def goal_callback(self, msg):
        if not self.action_client.wait_for_server(timeout_sec=self.server_timeout_sec):
            self.get_logger().warn(
                "Nav2 NavigateToPose action server is not available yet.",
                throttle_duration_sec=2.0,
            )
            return

        if self.active_goal_handle is not None:
            self.get_logger().info("Canceling previous Nav2 goal before sending a new one.")
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.get_logger().info(
            f"Sending Nav2 goal from {msg.header.frame_id}: "
            f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
        )
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal was rejected.")
            return
        self.active_goal_handle = goal_handle
        self.get_logger().info("Nav2 goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        status_name = self.status_name(result.status)
        self.get_logger().info(f"Nav2 goal finished with status: {status_name}")
        self.active_goal_handle = None

    @staticmethod
    def status_name(status):
        names = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        return names.get(status, str(status))


def main():
    rclpy.init()
    node = GoalPoseToNav2()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
