import json
import math
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from h1_2_icp_localization.msg import LocalizationStatus
from h1_2_nav_msgs.srv import ExecuteGesture, RouteStatus, StartRoute, StopRoute
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from .route_activity_guard import RouteActivityGuard


class MemoryRouteRunner(Node):
    def __init__(self):
        super().__init__("memory_route_runner")
        self.declare_parameter("points_file", "maps/memory_points.json")
        self.declare_parameter("route_names", "A,B,C")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("start_service", "start_memory_route")
        self.declare_parameter("stop_service", "stop_memory_route")
        self.declare_parameter("status_service", "memory_route_status")
        self.declare_parameter("server_timeout_sec", 2.0)
        self.declare_parameter("arrival_wait_sec", 2.0)
        self.declare_parameter("blocked_retry_wait_sec", 5.0)
        self.declare_parameter("max_retries_per_goal", 3)
        self.declare_parameter("return_to_start", False)
        self.declare_parameter("start_at_first_route_point", False)
        self.declare_parameter("start_point_name", "START")
        self.declare_parameter("required_tf_target_frame", "map")
        self.declare_parameter("required_tf_source_frame", "camera_init")
        self.declare_parameter("required_tf_timeout_sec", 0.1)
        self.declare_parameter("required_tf_max_age_sec", 0.25)
        self.declare_parameter("required_tf_max_future_sec", 0.25)
        self.declare_parameter("require_localization_status", False)
        self.declare_parameter("localization_status_topic", "/icp_localization/status")
        self.declare_parameter("max_localization_score", 0.15)
        self.declare_parameter("required_stable_samples", 3)
        self.declare_parameter("localization_status_max_age_sec", 1.5)
        self.declare_parameter("gesture_route", "")
        self.declare_parameter("gesture_service_name", "/h1_2_loco/execute_gesture")
        self.declare_parameter("gesture_timeout_sec", 20.0)
        self.declare_parameter("gesture_hold_sec", 3.0)
        self.declare_parameter("turn_route", "")
        self.declare_parameter("turn_angle_rad", math.pi)
        self.declare_parameter("spin_action_name", "spin")
        self.declare_parameter("spin_time_allowance_sec", 15.0)
        self.declare_parameter("peer_status_service", "teaching_route_status")
        self.declare_parameter("route_guard_timeout_sec", 0.5)

        self.points_file = Path(self.get_parameter("points_file").value).expanduser()
        self.route_names = self.parse_route_names(self.get_parameter("route_names").value)
        self.server_timeout_sec = float(self.get_parameter("server_timeout_sec").value)
        self.arrival_wait_sec = max(0.0, float(self.get_parameter("arrival_wait_sec").value))
        self.blocked_retry_wait_sec = max(0.0, float(self.get_parameter("blocked_retry_wait_sec").value))
        self.max_retries_per_goal = max(0, int(self.get_parameter("max_retries_per_goal").value))
        self.return_to_start = bool(self.get_parameter("return_to_start").value)
        self.start_at_first_route_point = bool(
            self.get_parameter("start_at_first_route_point").value
        )
        self.start_point_name = self.get_parameter("start_point_name").value
        self.required_tf_target_frame = self.get_parameter("required_tf_target_frame").value
        self.required_tf_source_frame = self.get_parameter("required_tf_source_frame").value
        self.required_tf_timeout_sec = max(0.0, float(self.get_parameter("required_tf_timeout_sec").value))
        self.required_tf_max_age_sec = max(0.0, float(self.get_parameter("required_tf_max_age_sec").value))
        self.required_tf_max_future_sec = max(
            0.0, float(self.get_parameter("required_tf_max_future_sec").value)
        )
        self.require_localization_status = bool(
            self.get_parameter("require_localization_status").value
        )
        self.max_localization_score = float(self.get_parameter("max_localization_score").value)
        self.required_stable_samples = max(
            1, int(self.get_parameter("required_stable_samples").value)
        )
        self.localization_status_max_age_sec = max(
            0.0, float(self.get_parameter("localization_status_max_age_sec").value)
        )
        self.gesture_route = self.parse_gesture_route(self.get_parameter("gesture_route").value)
        self.gesture_service_name = self.get_parameter("gesture_service_name").value
        self.gesture_timeout_sec = max(0.0, float(self.get_parameter("gesture_timeout_sec").value))
        self.gesture_hold_sec = max(0.0, float(self.get_parameter("gesture_hold_sec").value))
        self.turn_route = self.parse_turn_route(self.get_parameter("turn_route").value)
        self.turn_angle_rad = abs(float(self.get_parameter("turn_angle_rad").value))
        self.spin_time_allowance_sec = max(0.0, float(
            self.get_parameter("spin_time_allowance_sec").value
        ))

        self.action_client = ActionClient(self, NavigateToPose, self.get_parameter("action_name").value)
        self.spin_action_client = ActionClient(
            self, Spin, self.get_parameter("spin_action_name").value
        )
        self.gesture_client = self.create_client(ExecuteGesture, self.gesture_service_name)
        self.route_guard = RouteActivityGuard(
            self,
            self.get_parameter("peer_status_service").value,
            "teaching route",
            self.get_parameter("route_guard_timeout_sec").value,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.localization_status_sub = self.create_subscription(
            LocalizationStatus,
            self.get_parameter("localization_status_topic").value,
            self.localization_status_callback,
            10,
        )
        self.active_goal_handle = None
        self.active_spin_goal_handle = None
        self.active_gesture_future = None
        self.pending_gestures = []
        self.pending_gestures = []
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
        self.last_tf_error = ""
        self.last_localization_status = None
        self.last_localization_status_received = None
        self.localization_stable_samples = 0

        self.create_service(StartRoute, self.get_parameter("start_service").value, self.start_callback)
        self.create_service(StopRoute, self.get_parameter("stop_service").value, self.stop_callback)
        self.create_service(RouteStatus, self.get_parameter("status_service").value, self.status_callback)
        self.get_logger().info(
            f"Memory route runner ready: file={self.points_file}, route={','.join(self.route_names)}, "
            f"gesture_route={self.gesture_route}, turn_route={self.turn_route}"
        )

    def start_callback(self, request, response):
        del request
        if self.state in ("running", "waiting", "retrying", "gesturing", "turning", "stopping"):
            response.success = False
            response.message = f"Route is already active: {self.state}"
            return response
        allowed, guard_message = self.route_guard.check_peer_inactive()
        if not allowed:
            response.success = False
            response.message = guard_message
            return response
        if guard_message:
            self.get_logger().info(guard_message)
        if not self.route_names:
            response.success = False
            response.message = "Route names are empty."
            return response
        if not self.action_client.wait_for_server(timeout_sec=self.server_timeout_sec):
            response.success = False
            response.message = "Nav2 NavigateToPose action server is not available."
            return response
        if self.turn_route and not self.spin_action_client.wait_for_server(
            timeout_sec=self.server_timeout_sec
        ):
            response.success = False
            response.message = "Nav2 Spin action server is not available."
            return response
        if self.gesture_route and not self.gesture_client.wait_for_service(timeout_sec=self.server_timeout_sec):
            response.success = False
            response.message = f"Gesture service is not available: {self.gesture_service_name}"
            return response
        localization_ready, localization_message = self.localization_ready()
        if not localization_ready:
            response.success = False
            response.message = localization_message
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
        self.current_index = (
            1 if self.start_at_first_route_point and not self.return_to_start
            and len(self.route_names) > 1 else 0
        )
        self.returning_to_start = self.return_to_start
        self.retry_count = 0
        self.active_gesture_future = None
        self.pending_gestures = []
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
        canceled_action = False
        if self.active_goal_handle is not None:
            self.state = "stopping"
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None
            canceled_action = True
        if self.active_spin_goal_handle is not None:
            self.state = "stopping"
            self.active_spin_goal_handle.cancel_goal_async()
            self.active_spin_goal_handle = None
            canceled_action = True
        if not canceled_action:
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
            self.last_tf_error = ""
            return True
        try:
            transform = self.tf_buffer.lookup_transform(
                self.required_tf_target_frame,
                self.required_tf_source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.required_tf_timeout_sec),
            )
            stamp = rclpy.time.Time.from_msg(transform.header.stamp)
            if stamp.nanoseconds <= 0:
                self.last_tf_error = "Localization TF has a zero timestamp."
                return False
            age_sec = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if self.required_tf_max_age_sec > 0.0 and age_sec > self.required_tf_max_age_sec:
                self.last_tf_error = (
                    f"Localization TF is stale: age={age_sec:.3f}s, "
                    f"limit={self.required_tf_max_age_sec:.3f}s."
                )
                return False
            if age_sec < -self.required_tf_max_future_sec:
                self.last_tf_error = (
                    f"Localization TF is too far in the future: offset={-age_sec:.3f}s, "
                    f"limit={self.required_tf_max_future_sec:.3f}s."
                )
                return False
            self.last_tf_error = ""
            return True
        except Exception as exc:
            self.last_tf_error = (
                f"Localization TF is not available: "
                f"{self.required_tf_target_frame}->{self.required_tf_source_frame}: {exc}"
            )
            return False

    def localization_status_callback(self, msg):
        self.last_localization_status = msg
        self.last_localization_status_received = self.get_clock().now()
        valid = (
            msg.map_loaded
            and msg.initialized
            and msg.converged
            and not msg.degraded
            and math.isfinite(msg.score)
            and msg.score <= self.max_localization_score
        )
        if valid:
            self.localization_stable_samples = min(
                self.localization_stable_samples + 1,
                self.required_stable_samples,
            )
        else:
            self.localization_stable_samples = 0

    def localization_ready(self):
        if not self.localization_tf_available():
            return False, self.last_tf_error
        if not self.require_localization_status:
            return True, ""
        if self.last_localization_status is None or self.last_localization_status_received is None:
            return False, "Localization status has not been received."

        age_sec = (
            self.get_clock().now() - self.last_localization_status_received
        ).nanoseconds / 1e9
        if (
            self.localization_status_max_age_sec > 0.0
            and age_sec > self.localization_status_max_age_sec
        ):
            return False, (
                f"Localization status is stale: age={age_sec:.3f}s, "
                f"limit={self.localization_status_max_age_sec:.3f}s."
            )

        status = self.last_localization_status
        if not status.map_loaded:
            return False, "Localization map is not loaded."
        if not status.initialized:
            return False, "Localization is not initialized."
        if not status.converged or status.degraded:
            return False, f"Localization is not healthy: {status.message}"
        if not math.isfinite(status.score) or status.score > self.max_localization_score:
            return False, (
                f"Localization score is too high: score={status.score:.3f}, "
                f"limit={self.max_localization_score:.3f}."
            )
        if self.localization_stable_samples < self.required_stable_samples:
            return False, (
                f"Localization is not stable yet: "
                f"{self.localization_stable_samples}/{self.required_stable_samples} samples."
            )
        return True, ""

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

    @staticmethod
    def parse_turn_route(value):
        if not isinstance(value, str):
            return {}
        turn_route = {}
        for item in value.split(","):
            if ":" not in item:
                continue
            name, direction = (part.strip() for part in item.split(":", 1))
            direction = direction.lower()
            if name and direction in ("cw", "ccw"):
                turn_route[name] = direction
        return turn_route

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
        localization_ready, localization_message = self.localization_ready()
        if not localization_ready:
            self.handle_goal_failure(localization_message)
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
        # Route points may have been recorded while the robot was facing an
        # unrelated direction. For navigation, align the goal pose with the
        # travel direction; an explicit turn_route is applied after arrival.
        if not self.returning_to_start:
            self.set_route_arrival_orientation(goal.pose, name)
        self.current_goal = name
        self.state = "running"
        self.last_message = f"Sending route goal: {name}"
        self.get_logger().info(self.last_message)
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def set_route_arrival_orientation(self, pose, goal_name):
        if len(self.route_names) < 2:
            return
        previous_name = self.route_names[(self.current_index - 1) % len(self.route_names)]
        previous_record = self.points.get(previous_name)
        goal_record = self.points.get(goal_name)
        if previous_record is None or goal_record is None:
            return
        previous_position = previous_record.get("position", {})
        goal_position = goal_record.get("position", {})
        dx = float(goal_position.get("x", 0.0)) - float(previous_position.get("x", 0.0))
        dy = float(goal_position.get("y", 0.0)) - float(previous_position.get("y", 0.0))
        if math.hypot(dx, dy) < 1e-6:
            return
        arrival_yaw = math.atan2(dy, dx)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(arrival_yaw / 2.0)
        pose.pose.orientation.w = math.cos(arrival_yaw / 2.0)

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
            gestures = [item.strip() for item in gesture.split("+") if item.strip()]
            self.pending_gestures = gestures[1:]
            gesture = gestures[0] if gestures else ""
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

        gesture = gesture.strip()
        if not gesture:
            self.finish_goal_arrival(f"Reached {goal_name}. Waiting before next goal.")
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
            self.pending_gestures = []
            self.get_logger().warn(f"Gesture {gesture} at {goal_name} failed: {exc}")
            self.finish_goal_arrival(f"Reached {goal_name}; gesture {gesture} failed; continuing.")
            return

        if result.success:
            if self.pending_gestures:
                next_gesture = self.pending_gestures.pop(0)
                self.schedule_timer(
                    self.gesture_hold_sec,
                    lambda: self.execute_arrival_gesture(goal_name, next_gesture),
                )
            else:
                self.finish_goal_arrival(f"Reached {goal_name}; gesture {gesture} completed.")
        else:
            self.pending_gestures = []
            self.get_logger().warn(
                f"Gesture {gesture} at {goal_name} returned sdk_ret={result.sdk_ret}: {result.message}"
            )
            self.finish_goal_arrival(f"Reached {goal_name}; gesture {gesture} failed; continuing.")

    def gesture_timeout_callback(self, future, goal_name, gesture):
        if future is not self.active_gesture_future:
            return
        self.active_gesture_future = None
        self.pending_gestures = []
        self.get_logger().warn(f"Gesture {gesture} at {goal_name} timed out; continuing route.")
        self.finish_goal_arrival(f"Reached {goal_name}; gesture {gesture} timed out; continuing.")

    def finish_goal_arrival(self, message):
        if self.stop_requested:
            self.state = "idle"
            self.current_goal = ""
            self.last_message = "Route stopped."
            return
        direction = self.turn_route.get(self.current_goal, "")
        if direction:
            self.execute_arrival_turn(self.current_goal, direction)
            return
        self.advance_route_after_arrival(message)

    def execute_arrival_turn(self, goal_name, direction):
        goal = Spin.Goal()
        goal.target_yaw = self.turn_angle_rad if direction == "ccw" else -self.turn_angle_rad
        goal.time_allowance.sec = int(self.spin_time_allowance_sec)
        goal.time_allowance.nanosec = int(
            (self.spin_time_allowance_sec - int(self.spin_time_allowance_sec)) * 1e9
        )
        self.state = "turning"
        self.last_message = f"Reached {goal_name}. Turning {direction} before next goal."
        self.get_logger().info(self.last_message)
        future = self.spin_action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self.spin_response_callback(done, goal_name, direction)
        )

    def spin_response_callback(self, future, goal_name, direction):
        if self.stop_requested:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = "blocked"
            self.last_message = f"Nav2 rejected {direction} turn at {goal_name}."
            self.get_logger().error(self.last_message)
            return
        self.active_spin_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self.spin_result_callback(done, goal_name, direction)
        )

    def spin_result_callback(self, future, goal_name, direction):
        self.active_spin_goal_handle = None
        result = future.result()
        if self.stop_requested or result.status == GoalStatus.STATUS_CANCELED:
            self.state = "idle"
            self.current_goal = ""
            self.last_message = "Route stopped."
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.state = "blocked"
            self.last_message = (
                f"{direction} turn at {goal_name} failed: {self.status_name(result.status)}"
            )
            self.get_logger().error(self.last_message)
            return
        self.advance_route_after_arrival(
            f"Reached {goal_name}; {direction} turn completed."
        )

    def advance_route_after_arrival(self, message):
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
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
