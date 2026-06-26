import json
import math
from pathlib import Path as FilePath

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from h1_2_nav_msgs.srv import ExecuteGesture, RouteStatus, StartRoute, StopRoute
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32

from .route_activity_guard import RouteActivityGuard


class TeachingRouteRunner(Node):
    def __init__(self):
        super().__init__("teaching_route_runner")
        self.declare_parameter("path_file", "")
        self.declare_parameter("events_file", "")
        self.declare_parameter("memory_points_file", "maps/memory_points.json")
        self.declare_parameter("follow_path_action", "follow_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("start_service", "start_teaching_route")
        self.declare_parameter("stop_service", "stop_teaching_route")
        self.declare_parameter("status_service", "teaching_route_status")
        self.declare_parameter("peer_status_service", "memory_route_status")
        self.declare_parameter("server_timeout_sec", 2.0)
        self.declare_parameter("route_guard_timeout_sec", 0.5)
        self.declare_parameter("loop", True)
        self.declare_parameter("default_trigger_radius", 0.45)
        self.declare_parameter("gesture_service_name", "/h1_2_loco/execute_gesture")
        self.declare_parameter("gesture_timeout_sec", 20.0)
        self.declare_parameter("voice_feedback_topic", "/h1_2/voice_feedback")
        self.declare_parameter("stop_cmd_topic", "/cmd_vel")
        self.declare_parameter("event_wait_sec", 0.5)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("visualization_path_topic", "/teaching_route_path")

        self.path_file_value = str(self.get_parameter("path_file").value or "").strip()
        self.path_file = FilePath(self.path_file_value).expanduser()
        self.events_file = str(self.get_parameter("events_file").value or "").strip()
        self.memory_points_file = FilePath(self.get_parameter("memory_points_file").value).expanduser()
        self.controller_id = self.get_parameter("controller_id").value
        self.server_timeout_sec = float(self.get_parameter("server_timeout_sec").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.default_trigger_radius = max(0.0, float(self.get_parameter("default_trigger_radius").value))
        self.gesture_timeout_sec = max(0.0, float(self.get_parameter("gesture_timeout_sec").value))
        self.event_wait_sec = max(0.0, float(self.get_parameter("event_wait_sec").value))
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.state = "idle"
        self.current_goal = ""
        self.current_segment_index = 0
        self.last_message = "Teaching route runner is idle."
        self.stop_requested = False
        self.active_goal_handle = None
        self.active_gesture_future = None
        self.pending_timer = None
        self.segments = []

        self.action_client = ActionClient(self, FollowPath, self.get_parameter("follow_path_action").value)
        self.gesture_client = self.create_client(ExecuteGesture, self.get_parameter("gesture_service_name").value)
        self.voice_pub = self.create_publisher(Int32, self.get_parameter("voice_feedback_topic").value, 10)
        self.stop_cmd_pub = self.create_publisher(Twist, self.get_parameter("stop_cmd_topic").value, 10)
        self.teaching_path_pub = self.create_publisher(
            Path,
            self.get_parameter("visualization_path_topic").value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.route_guard = RouteActivityGuard(
            self,
            self.get_parameter("peer_status_service").value,
            "memory route",
            self.get_parameter("route_guard_timeout_sec").value,
        )

        self.create_service(StartRoute, self.get_parameter("start_service").value, self.start_callback)
        self.create_service(StopRoute, self.get_parameter("stop_service").value, self.stop_callback)
        self.create_service(RouteStatus, self.get_parameter("status_service").value, self.status_callback)

        teaching_path = self.load_path_file()
        if teaching_path is not None:
            self.publish_teaching_path(teaching_path)

        self.get_logger().info(
            f"Teaching route runner ready: path_file={self.path_file}, "
            f"events_file={self.events_file or '<none>'}, controller_id={self.controller_id}"
        )

    def start_callback(self, request, response):
        del request
        if self.state in ("running", "waiting", "gesturing", "stopping"):
            response.success = False
            response.message = f"Teaching route is already active: {self.state}"
            return response

        allowed, guard_message = self.route_guard.check_peer_inactive()
        if not allowed:
            response.success = False
            response.message = guard_message
            return response
        if guard_message:
            self.get_logger().info(guard_message)

        if not self.action_client.wait_for_server(timeout_sec=self.server_timeout_sec):
            response.success = False
            response.message = "Nav2 FollowPath action server is not available."
            return response

        path_msg = self.load_path_file()
        if path_msg is None:
            response.success = False
            response.message = self.last_message
            return response
        self.publish_teaching_path(path_msg)

        events = self.load_route_events(path_msg)
        if events is None:
            response.success = False
            response.message = self.last_message
            return response

        if any(event.get("gesture") for event in events):
            if not self.gesture_client.wait_for_service(timeout_sec=self.server_timeout_sec):
                response.success = False
                response.message = f"Gesture service is not available: {self.get_parameter('gesture_service_name').value}"
                return response

        self.segments = self.build_segments(path_msg, events)
        if not self.segments:
            response.success = False
            response.message = "Teaching path does not contain enough poses to follow."
            return response

        self.stop_requested = False
        self.current_segment_index = 0
        self.active_goal_handle = None
        self.active_gesture_future = None
        self.state = "running"
        self.last_message = "Teaching route started."
        self.send_current_segment()
        response.success = True
        response.message = self.last_message
        return response

    def stop_callback(self, request, response):
        del request
        self.stop_requested = True
        self.cancel_pending_timer()
        self.active_gesture_future = None
        self.publish_stop_command()
        if self.active_goal_handle is not None:
            self.state = "stopping"
            self.active_goal_handle.cancel_goal_async()
            self.active_goal_handle = None
        else:
            self.state = "idle"
        self.current_goal = ""
        self.last_message = "Teaching route stop requested."
        response.success = True
        response.message = self.last_message
        return response

    def status_callback(self, request, response):
        del request
        response.success = True
        response.state = self.state
        response.current_goal = self.current_goal
        response.current_index = int(self.current_segment_index)
        response.message = self.last_message
        return response

    def load_path_file(self):
        if not self.path_file_value:
            self.state = "blocked"
            self.last_message = "Parameter path_file is empty."
            return None
        poses = []
        try:
            with self.path_file.open("r", encoding="utf-8") as stream:
                for line in stream:
                    line = line.strip()
                    if not line or "EOP" in line:
                        continue
                    parts = [float(value) for value in line.split()]
                    if len(parts) != 7:
                        self.get_logger().warn(f"Invalid teaching path line skipped: {line}")
                        continue
                    pose = PoseStamped()
                    pose.header.frame_id = self.frame_id
                    pose.pose.position.x = parts[0]
                    pose.pose.position.y = parts[1]
                    pose.pose.position.z = parts[2]
                    pose.pose.orientation.x = parts[3]
                    pose.pose.orientation.y = parts[4]
                    pose.pose.orientation.z = parts[5]
                    pose.pose.orientation.w = parts[6]
                    poses.append(pose)
        except OSError as exc:
            self.state = "blocked"
            self.last_message = f"Failed to load teaching path file {self.path_file}: {exc}"
            self.get_logger().error(self.last_message)
            return None
        except ValueError as exc:
            self.state = "blocked"
            self.last_message = f"Failed to parse teaching path file {self.path_file}: {exc}"
            self.get_logger().error(self.last_message)
            return None

        if len(poses) < 2:
            self.state = "blocked"
            self.last_message = f"Teaching path needs at least 2 poses: {self.path_file}"
            return None

        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.poses = poses
        return path_msg

    def publish_teaching_path(self, path_msg):
        stamp = self.get_clock().now().to_msg()
        path_msg.header.stamp = stamp
        for pose in path_msg.poses:
            pose.header.frame_id = self.frame_id
            pose.header.stamp = stamp
        self.teaching_path_pub.publish(path_msg)

    def load_route_events(self, path_msg):
        if not self.events_file:
            return []
        try:
            data = json.loads(FilePath(self.events_file).expanduser().read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self.state = "blocked"
            self.last_message = f"Failed to load teaching route events file {self.events_file}: {exc}"
            self.get_logger().error(self.last_message)
            return None
        raw_events = data.get("events", [])
        if not isinstance(raw_events, list):
            self.state = "blocked"
            self.last_message = "Teaching route events file has invalid events list."
            return None
        if not raw_events:
            return []
        points = self.load_memory_points()
        if points is None:
            return None
        events = []
        for raw in raw_events:
            event = self.project_event(raw, points, path_msg)
            if event is None:
                return None
            events.append(event)
        events.sort(key=lambda item: item["path_index"])
        return events

    def load_memory_points(self):
        try:
            data = json.loads(self.memory_points_file.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self.state = "blocked"
            self.last_message = f"Failed to load memory points file {self.memory_points_file}: {exc}"
            self.get_logger().error(self.last_message)
            return None
        points = data.get("points", {})
        if not isinstance(points, dict):
            self.state = "blocked"
            self.last_message = "Memory points file has invalid points object."
            return None
        return points

    def project_event(self, raw, points, path_msg):
        if not isinstance(raw, dict):
            self.state = "blocked"
            self.last_message = "Teaching route event must be an object."
            return None
        point_name = str(raw.get("point", "")).strip()
        if not point_name or point_name not in points:
            self.state = "blocked"
            self.last_message = f"Teaching route event contains unknown memory point: {point_name}"
            return None

        record = points[point_name]
        position = record.get("position", {})
        target_x = float(position.get("x", 0.0))
        target_y = float(position.get("y", 0.0))
        nearest_index = min(
            range(len(path_msg.poses)),
            key=lambda index: self.pose_xy_distance(path_msg.poses[index], target_x, target_y),
        )
        nearest_distance = self.pose_xy_distance(path_msg.poses[nearest_index], target_x, target_y)
        trigger_radius = float(raw.get("trigger_radius", self.default_trigger_radius))
        if nearest_distance > trigger_radius:
            self.state = "blocked"
            self.last_message = (
                f"Memory point {point_name} is too far from teaching path: "
                f"{nearest_distance:.2f}m > {trigger_radius:.2f}m"
            )
            return None

        return {
            "point": point_name,
            "path_index": nearest_index,
            "gesture": str(raw.get("gesture", "")).strip(),
            "voice_code": raw.get("voice_code", None),
        }

    @staticmethod
    def pose_xy_distance(pose_stamped, x, y):
        p = pose_stamped.pose.position
        return math.hypot(p.x - x, p.y - y)

    def build_segments(self, path_msg, events):
        segments = []
        start_index = 0
        for event in events:
            end_index = int(event["path_index"])
            if end_index <= start_index:
                end_index = min(start_index + 1, len(path_msg.poses) - 1)
            if end_index > start_index:
                segments.append(
                    {
                        "path": self.slice_path(path_msg, start_index, end_index),
                        "event": event,
                    }
                )
                start_index = end_index
        if start_index < len(path_msg.poses) - 1:
            segments.append(
                {
                    "path": self.slice_path(path_msg, start_index, len(path_msg.poses) - 1),
                    "event": None,
                }
            )
        return segments

    def slice_path(self, path_msg, start_index, end_index):
        sliced = Path()
        sliced.header.frame_id = path_msg.header.frame_id
        sliced.poses = list(path_msg.poses[start_index : end_index + 1])
        return sliced

    def send_current_segment(self):
        if self.stop_requested:
            self.state = "idle"
            self.current_goal = ""
            self.last_message = "Teaching route stopped."
            return
        if not self.segments:
            self.state = "blocked"
            self.last_message = "Teaching route has no path segments."
            return

        segment = self.segments[self.current_segment_index]
        path_msg = segment["path"]
        stamp = self.get_clock().now().to_msg()
        path_msg.header.stamp = stamp
        for pose in path_msg.poses:
            pose.header.stamp = stamp
        event = segment.get("event")
        self.current_goal = event["point"] if event else f"segment_{self.current_segment_index}"
        self.state = "running"
        self.last_message = f"Sending teaching path segment: {self.current_goal}"
        self.get_logger().info(self.last_message)

        goal = FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = self.controller_id
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        if self.stop_requested:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = "blocked"
            self.last_message = "Nav2 rejected teaching path segment."
            self.get_logger().warn(self.last_message)
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
            self.publish_stop_command()
            self.last_message = "Teaching route stopped."
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.state = "blocked"
            self.last_message = f"Teaching path segment failed with status: {self.status_name(result.status)}"
            self.get_logger().warn(self.last_message)
            self.publish_stop_command()
            return
        self.handle_segment_arrival()

    def handle_segment_arrival(self):
        segment = self.segments[self.current_segment_index]
        event = segment.get("event")
        if event:
            self.execute_event(event)
            return
        self.finish_segment_arrival("Teaching path segment completed.")

    def execute_event(self, event):
        if self.stop_requested:
            self.state = "idle"
            self.current_goal = ""
            self.last_message = "Teaching route stopped."
            return
        self.publish_stop_command()
        self.publish_voice_code(event.get("voice_code"))
        gesture = event.get("gesture", "")
        if not gesture:
            self.state = "waiting"
            self.last_message = f"Reached {event['point']}; waiting before next segment."
            self.schedule_timer(self.event_wait_sec, lambda: self.finish_segment_arrival(self.last_message))
            return
        if not self.gesture_client.service_is_ready():
            self.get_logger().warn(f"Gesture service is not ready; skipping gesture {gesture} at {event['point']}.")
            self.finish_segment_arrival(f"Reached {event['point']}; gesture service unavailable; continuing.")
            return

        request = ExecuteGesture.Request()
        request.name = gesture
        self.state = "gesturing"
        self.last_message = f"Reached {event['point']}. Executing gesture: {gesture}"
        self.get_logger().info(self.last_message)
        future = self.gesture_client.call_async(request)
        self.active_gesture_future = future
        future.add_done_callback(lambda done: self.gesture_result_callback(done, event, gesture))
        self.schedule_timer(
            self.gesture_timeout_sec,
            lambda: self.gesture_timeout_callback(future, event, gesture),
        )

    def publish_voice_code(self, voice_code):
        if voice_code is None or voice_code == "":
            return
        try:
            code = int(voice_code)
        except (TypeError, ValueError):
            self.get_logger().warn(f"Invalid voice_code skipped: {voice_code}")
            return
        message = Int32()
        message.data = code
        self.voice_pub.publish(message)

    def gesture_result_callback(self, future, event, gesture):
        if future is not self.active_gesture_future:
            return
        self.active_gesture_future = None
        self.cancel_pending_timer()
        if self.stop_requested:
            self.state = "idle"
            self.current_goal = ""
            self.last_message = "Teaching route stopped."
            return
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Gesture {gesture} at {event['point']} failed: {exc}")
            self.finish_segment_arrival(f"Reached {event['point']}; gesture {gesture} failed; continuing.")
            return
        if result.success:
            self.finish_segment_arrival(f"Reached {event['point']}; gesture {gesture} completed.")
        else:
            self.get_logger().warn(
                f"Gesture {gesture} at {event['point']} returned sdk_ret={result.sdk_ret}: {result.message}"
            )
            self.finish_segment_arrival(f"Reached {event['point']}; gesture {gesture} failed; continuing.")

    def gesture_timeout_callback(self, future, event, gesture):
        if future is not self.active_gesture_future:
            return
        self.active_gesture_future = None
        self.get_logger().warn(f"Gesture {gesture} at {event['point']} timed out; continuing route.")
        self.finish_segment_arrival(f"Reached {event['point']}; gesture {gesture} timed out; continuing.")

    def finish_segment_arrival(self, message):
        if self.stop_requested:
            self.state = "idle"
            self.current_goal = ""
            self.last_message = "Teaching route stopped."
            return
        next_index = self.current_segment_index + 1
        if next_index >= len(self.segments):
            if not self.loop:
                self.state = "idle"
                self.current_goal = ""
                self.last_message = "Teaching route completed."
                self.publish_stop_command()
                return
            next_index = 0
        self.current_segment_index = next_index
        self.state = "waiting"
        self.last_message = message
        self.schedule_timer(self.event_wait_sec, self.send_current_segment)

    def publish_stop_command(self):
        self.stop_cmd_pub.publish(Twist())

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
    node = TeachingRouteRunner()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
