import json
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from h1_2_nav_msgs.srv import ExecuteGesture, RouteStatus, StartRoute, StopRoute
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class MemoryRouteRunner(Node):
    def __init__(self):
        super().__init__("memory_route_runner")
        self.declare_parameter("points_file", "maps/memory_points.json")
        self.declare_parameter("route_names", ["A", "B", "C"])
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("start_service", "start_memory_route")
        self.declare_parameter("stop_service", "stop_memory_route")
        self.declare_parameter("status_service", "memory_route_status")
        self.declare_parameter("server_timeout_sec", 2.0)
        self.declare_parameter("arrival_wait_sec", 2.0)
        self.declare_parameter("blocked_retry_wait_sec", 5.0)
        self.declare_parameter("max_retries_per_goal", 3)
        self.declare_parameter("return_to_start", False)
        self.declare_parameter("start_point_name", "START")
        self.declare_parameter("required_tf_target_frame", "map")
        self.declare_parameter("required_tf_source_frame", "camera_init")
        self.declare_parameter("required_tf_timeout_sec", 0.1)
        self.declare_parameter("gesture_route", "")
        self.declare_parameter("gesture_service_name", "/h1_2_loco/execute_gesture")
        self.declare_parameter("gesture_timeout_sec", 20.0)

        self.points_file = Path(self.get_parameter("points_file").value).expanduser()
        self.route_names = self.parse_route_names(self.get_parameter("route_names").value)
        self.server_timeout_sec = float(self.get_parameter("server_timeout_sec").value)
        self.arrival_wait_sec = max(0.0, float(self.get_parameter("arrival_wait_sec").value))
        self.blocked_retry_wait_sec = max(0.0, float(self.get_parameter("blocked_retry_wait_sec").value))
        self.max_retries_per_goal = max(0, int(self.get_parameter("max_retries_per_goal").value))
        self.return_to_start = bool(self.get_parameter("return_to_start").value)
        self.start_point_name = self.get_parameter("start_point_name").value
        self.required_tf_target_frame = self.get_parameter("required_tf_target_frame").value
        self.required_tf_source_frame = self.get_parameter("required_tf_source_frame").value
        self.required_tf_timeout_sec = max(0.0, float(self.get_parameter("required_tf_timeout_sec").value))
        self.gesture_route = self.parse_gesture_route(self.get_parameter("gesture_route").value)
        self.gesture_service_name = self.get_parameter("gesture_service_name").value
        self.gesture_timeout_sec = max(0.0, float(self.get_parameter("gesture_timeout_sec").value))

        self.action_client = ActionClient(self, NavigateToPose, self.get_parameter("action_name").value)
        self.gesture_client = self.create_client(ExecuteGesture, self.gesture_service_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.active_goal_handle = None
        self.active_gesture_future = None
        self.state = "idle"
        self.current_index = 0
        self.current_goal = ""
        self.returning_to_start = False
        self.retry_count = 0
        self.points = {}
        self.frame_id = "map"
        self.pending_timer = None
        self.stop_requested = False
        self.last_message = "Route runner is idle."

        self.create_service(StartRoute, self.get_parameter("start_service").value, self.start_callback)
        self.create_service(StopRoute, self.get_parameter("stop_service").value, self.stop_callback)
        self.create_service(RouteStatus, self.get_parameter("status_service").value, self.status_callback)
        self.get_logger().info(
            f"Memory route runner ready: file={self.points_file}, route={','.join(self.route_names)}, "
            f"gesture_route={self.gesture_route}"
        )

    def start_callback(self, request, response):
        del request
        if self.state in ("running", "waiting", "retrying", "gesturing", "stopping"):
            response.success = False
            response.message = f"Route is already active: {self.state}"
            return response
        if not self.route_names:
            response.success = False
            response.message = "Route names are empty."
            return response
        if not self.action_client.wait_for_server(timeout_sec=self.server_timeout_sec):
            response.success = False
            response.message = "Nav2 NavigateToPose action server is not available."
            return response
        if self.gesture_route and not self.gesture_client.wait_for_service(timeout_sec=self.server_timeout_sec):
            response.success = False
            response.message = f"Gesture service is not available: {self.gesture_service_name}"
            return response
        if not self.localization_tf_available():
            response.success = False
            response.message = (
                f"Localization TF is not available: "
                f"{self.required_tf_target_frame}->{self.required_tf_source_frame}"
            )
            return response
        if not self.load_points():
            response.success = False
            response.message = self.last_message
            return response
        missing = [name for name in self.route_names if name not in self.points]
        if missing:
            response.success = False
            response.message = f"Route contains unknown memory points: {', '.join(missing)}"
            return response
        if self.return_to_start and self.start_point_name not in self.points:
            response.success = False
            response.message = f"Route contains unknown memory points: {self.start_point_name}"
            return response

        self.stop_requested = False
        self.current_index = 0
        self.returning_to_start = self.return_to_start
        self.retry_count = 0
        self.active_gesture_future = None
        self.state = "running"
        self.last_message = "Route started."
        self.send_current_goal()
        response.success = True
        response.message = self.last_message
        return response

    def stop_callback(self, request, response):
        del request
        self.stop_requested = True
        self.cancel_pending_timer()
        self.active_gesture_future = None
        if self.active_goal_handle is not None:
            self.state = "stopping"
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None
        else:
            self.state = "idle"
        self.current_goal = ""
        self.last_message = "Route stop requested."
        response.success = True
        response.message = self.last_message
        return response

    def status_callback(self, request, response):
        del request
        response.success = True
        response.state = self.state
        response.current_goal = self.current_goal
        response.current_index = int(self.current_index)
        response.message = self.last_message
        return response

    def localization_tf_available(self):
        if not self.required_tf_target_frame or not self.required_tf_source_frame:
            return True
        try:
            self.tf_buffer.lookup_transform(
                self.required_tf_target_frame,
                self.required_tf_source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.required_tf_timeout_sec),
            )
            return True
        except Exception:
            return False

    @staticmethod
    def parse_route_names(value):
        if isinstance(value, str):
            return [name.strip() for name in value.split(",") if name.strip()]
        return [str(name).strip() for name in value if str(name).strip()]

    @staticmethod
    def parse_gesture_route(value):
        if isinstance(value, dict):
            return {
                str(name).strip(): str(gesture).strip()
                for name, gesture in value.items()
                if str(name).strip() and str(gesture).strip()
            }
        if not isinstance(value, str):
            return {}
        gesture_route = {}
        for item in value.split(","):
            item = item.strip()
            if not item:
                continue
            if ":" not in item:
                continue
            name, gesture = item.split(":", 1)
            name = name.strip()
            gesture = gesture.strip()
            if name and gesture:
                gesture_route[name] = gesture
        return gesture_route

    def load_points(self):
        try:
            data = json.loads(self.points_file.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self.state = "blocked"
            self.last_message = f"Failed to load memory points file {self.points_file}: {exc}"
            self.get_logger().error(self.last_message)
            return False
        self.frame_id = data.get("frame_id", "map")
        points = data.get("points", {})
        if not isinstance(points, dict):
            self.state = "blocked"
            self.last_message = "Memory points file has invalid points object."
            return False
        self.points = points
        return True


    def next_goal_name(self):
        if self.returning_to_start:
            return self.start_point_name
        return self.route_names[self.current_index]

    def send_current_goal(self):
        if self.stop_requested:
            self.state = "idle"
            self.current_goal = ""
            self.last_message = "Route stopped."
            return
        if not self.route_names:
            self.state = "blocked"
            self.last_message = "Route names are empty."
            return
        name = self.next_goal_name()
        record = self.points.get(name)
        if record is None:
            self.state = "blocked"
            self.current_goal = name
            self.last_message = f"Memory point not found: {name}"
            self.get_logger().error(self.last_message)
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.record_to_pose(record)
        self.current_goal = name
        self.state = "running"
        self.last_message = f"Sending route goal: {name}"
        self.get_logger().info(self.last_message)
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        if self.stop_requested:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.handle_goal_failure("Nav2 rejected route goal.")
            return
        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.active_goal_handle = None
        result = future.result()
        if self.stop_requested or result.status == GoalStatus.STATUS_CANCELED:
            self.state = "idle"
            self.current_goal = ""
            self.last_message = "Route stopped."
            return
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.retry_count = 0
            self.handle_goal_arrival()
            return
        status_name = self.status_name(result.status)
        self.handle_goal_failure(f"Nav2 goal {self.current_goal} finished with status: {status_name}")

    def handle_goal_arrival(self):
        gesture = self.gesture_route.get(self.current_goal, "")
        if gesture:
            self.execute_arrival_gesture(self.current_goal, gesture)
            return
        self.finish_goal_arrival(f"Reached {self.current_goal}. Waiting before next goal.")

    def execute_arrival_gesture(self, goal_name, gesture):
        if self.stop_requested:
            self.state = "idle"
            self.current_goal = ""
            self.last_message = "Route stopped."
            return
        if not self.gesture_client.service_is_ready():
            self.get_logger().warn(f"Gesture service is not ready; skipping gesture {gesture} at {goal_name}.")
            self.finish_goal_arrival(f"Reached {goal_name}; gesture service unavailable; continuing.")
            return

        request = ExecuteGesture.Request()
        request.name = gesture
        self.state = "gesturing"
        self.last_message = f"Reached {goal_name}. Executing gesture: {gesture}"
        self.get_logger().info(self.last_message)
        future = self.gesture_client.call_async(request)
        self.active_gesture_future = future
        future.add_done_callback(lambda done: self.gesture_result_callback(done, goal_name, gesture))
        self.schedule_timer(
            self.gesture_timeout_sec,
            lambda: self.gesture_timeout_callback(future, goal_name, gesture),
        )

    def gesture_result_callback(self, future, goal_name, gesture):
        if future is not self.active_gesture_future:
            return
        self.active_gesture_future = None
        self.cancel_pending_timer()
        if self.stop_requested:
            self.state = "idle"
            self.current_goal = ""
            self.last_message = "Route stopped."
            return
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Gesture {gesture} at {goal_name} failed: {exc}")
            self.finish_goal_arrival(f"Reached {goal_name}; gesture {gesture} failed; continuing.")
            return

        if result.success:
            self.finish_goal_arrival(f"Reached {goal_name}; gesture {gesture} completed.")
        else:
            self.get_logger().warn(
                f"Gesture {gesture} at {goal_name} returned sdk_ret={result.sdk_ret}: {result.message}"
            )
            self.finish_goal_arrival(f"Reached {goal_name}; gesture {gesture} failed; continuing.")

    def gesture_timeout_callback(self, future, goal_name, gesture):
        if future is not self.active_gesture_future:
            return
        self.active_gesture_future = None
        self.get_logger().warn(f"Gesture {gesture} at {goal_name} timed out; continuing route.")
        self.finish_goal_arrival(f"Reached {goal_name}; gesture {gesture} timed out; continuing.")

    def finish_goal_arrival(self, message):
        if self.stop_requested:
            self.state = "idle"
            self.current_goal = ""
            self.last_message = "Route stopped."
            return
        if self.returning_to_start:
            self.returning_to_start = False
            self.current_index = 0
        else:
            self.current_index = (self.current_index + 1) % len(self.route_names)
        self.state = "waiting"
        self.last_message = message
        self.schedule_timer(self.arrival_wait_sec, self.send_current_goal)

    def handle_goal_failure(self, message):
        self.get_logger().warn(message)
        if self.retry_count >= self.max_retries_per_goal:
            self.state = "blocked"
            self.last_message = f"{message}; max retries reached."
            return
        self.retry_count += 1
        self.state = "retrying"
        self.last_message = f"{message}; retry {self.retry_count}/{self.max_retries_per_goal}."
        self.schedule_timer(self.blocked_retry_wait_sec, self.send_current_goal)

    def schedule_timer(self, delay_sec, callback):
        self.cancel_pending_timer()
        if delay_sec <= 0.0:
            callback()
            return

        def timer_callback():
            self.cancel_pending_timer()
            callback()

        self.pending_timer = self.create_timer(delay_sec, timer_callback)

    def cancel_pending_timer(self):
        if self.pending_timer is not None:
            self.pending_timer.cancel()
            self.pending_timer = None

    def record_to_pose(self, record):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = Time()
        position = record.get("position", {})
        orientation = record.get("orientation", {})
        pose.pose.position.x = float(position.get("x", 0.0))
        pose.pose.position.y = float(position.get("y", 0.0))
        pose.pose.position.z = float(position.get("z", 0.0))
        pose.pose.orientation.x = float(orientation.get("x", 0.0))
        pose.pose.orientation.y = float(orientation.get("y", 0.0))
        pose.pose.orientation.z = float(orientation.get("z", 0.0))
        pose.pose.orientation.w = float(orientation.get("w", 1.0))
        return pose

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
    node = MemoryRouteRunner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
