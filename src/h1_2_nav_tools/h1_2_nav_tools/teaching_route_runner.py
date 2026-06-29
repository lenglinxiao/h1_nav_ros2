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
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformListener

from .route_activity_guard import RouteActivityGuard


class TeachingRouteRunner(Node):
    def __init__(self):
        super().__init__("teaching_route_runner")
        self.declare_parameter("path_file", "")
        self.declare_parameter("events_file", "")
        self.declare_parameter("memory_points_file", "maps/memory_points.json")
        self.declare_parameter("follow_path_action", "follow_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("control_mode", "direct")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("control_frequency", 20.0)
        self.declare_parameter("tracking_linear_speed", 0.08)
        self.declare_parameter("tracking_lookahead_dist", 0.25)
        self.declare_parameter("tracking_goal_tolerance", 0.18)
        self.declare_parameter("max_tracking_error", 0.6)
        self.declare_parameter("heading_gain", 1.4)
        self.declare_parameter("lateral_gain", 0.8)
        self.declare_parameter("max_tracking_angular_speed", 0.35)
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
        self.declare_parameter("robot_frame", "body")
        self.declare_parameter("enable_start_pose_guard", True)
        self.declare_parameter("start_xy_tolerance", 0.5)
        self.declare_parameter("start_yaw_tolerance", 1.0)
        self.declare_parameter("start_tf_timeout_sec", 0.2)
        self.declare_parameter("visualization_path_topic", "/teaching_route_path")

        self.path_file_value = str(self.get_parameter("path_file").value or "").strip()
        self.path_file = FilePath(self.path_file_value).expanduser()
        self.events_file = str(self.get_parameter("events_file").value or "").strip()
        self.memory_points_file = FilePath(self.get_parameter("memory_points_file").value).expanduser()
        self.controller_id = self.get_parameter("controller_id").value
        self.control_mode = str(self.get_parameter("control_mode").value).strip().lower()
        self.server_timeout_sec = float(self.get_parameter("server_timeout_sec").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.default_trigger_radius = max(0.0, float(self.get_parameter("default_trigger_radius").value))
        self.gesture_timeout_sec = max(0.0, float(self.get_parameter("gesture_timeout_sec").value))
        self.event_wait_sec = max(0.0, float(self.get_parameter("event_wait_sec").value))
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.enable_start_pose_guard = bool(self.get_parameter("enable_start_pose_guard").value)
        self.start_xy_tolerance = max(0.0, float(self.get_parameter("start_xy_tolerance").value))
        self.start_yaw_tolerance = max(0.0, float(self.get_parameter("start_yaw_tolerance").value))
        self.start_tf_timeout_sec = max(0.0, float(self.get_parameter("start_tf_timeout_sec").value))
        self.control_frequency = max(1.0, float(self.get_parameter("control_frequency").value))
        self.tracking_linear_speed = max(0.0, float(self.get_parameter("tracking_linear_speed").value))
        self.tracking_lookahead_dist = max(0.05, float(self.get_parameter("tracking_lookahead_dist").value))
        self.tracking_goal_tolerance = max(0.02, float(self.get_parameter("tracking_goal_tolerance").value))
        self.max_tracking_error = max(0.05, float(self.get_parameter("max_tracking_error").value))
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.lateral_gain = float(self.get_parameter("lateral_gain").value)
        self.max_tracking_angular_speed = max(0.05, float(self.get_parameter("max_tracking_angular_speed").value))

        self.state = "idle"
        self.current_goal = ""
        self.current_segment_index = 0
        self.last_message = "Teaching route runner is idle."
        self.stop_requested = False
        self.active_goal_handle = None
        self.active_gesture_future = None
        self.pending_timer = None
        self.control_timer = None
        self.current_tracking_path = None
        self.tracking_index = 0
        self.segments = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, FollowPath, self.get_parameter("follow_path_action").value)
        self.gesture_client = self.create_client(ExecuteGesture, self.get_parameter("gesture_service_name").value)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_topic").value, 10)
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
            f"events_file={self.events_file or '<none>'}, control_mode={self.control_mode}, "
            f"controller_id={self.controller_id}"
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

        if self.control_mode not in ("direct", "nav2"):
            response.success = False
            response.message = f"Unsupported teaching control_mode: {self.control_mode}"
            return response

        if self.control_mode == "nav2" and not self.action_client.wait_for_server(timeout_sec=self.server_timeout_sec):
            response.success = False
            response.message = "Nav2 FollowPath action server is not available."
            return response

        path_msg = self.load_path_file()
        if path_msg is None:
            response.success = False
            response.message = self.last_message
            return response
        self.publish_teaching_path(path_msg)

        if not self.validate_start_pose(path_msg):
            response.success = False
            response.message = self.last_message
            return response

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
        self.stop_direct_tracking()
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

    def validate_start_pose(self, path_msg):
        if not self.enable_start_pose_guard:
            return True
        start = path_msg.poses[0].pose
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.start_tf_timeout_sec),
            )
        except Exception as exc:
            self.state = "blocked"
            self.last_message = (
                f"Cannot verify teaching route start pose: TF {self.frame_id}->{self.robot_frame} "
                f"is unavailable: {exc}"
            )
            self.get_logger().warn(self.last_message)
            return False

        current = transform.transform.translation
        xy_error = math.hypot(current.x - start.position.x, current.y - start.position.y)
        current_yaw = self.yaw_from_quaternion(transform.transform.rotation)
        start_yaw = self.yaw_from_quaternion(start.orientation)
        yaw_error = abs(self.normalize_angle(current_yaw - start_yaw))
        if xy_error <= self.start_xy_tolerance and yaw_error <= self.start_yaw_tolerance:
            return True

        self.state = "blocked"
        self.last_message = (
            "Teaching route start pose mismatch: "
            f"current=({current.x:.2f},{current.y:.2f},{current_yaw:.2f}), "
            f"path_start=({start.position.x:.2f},{start.position.y:.2f},{start_yaw:.2f}), "
            f"error=({xy_error:.2f}m,{yaw_error:.2f}rad), "
            f"limit=({self.start_xy_tolerance:.2f}m,{self.start_yaw_tolerance:.2f}rad)."
        )
        self.get_logger().warn(self.last_message)
        return False

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

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

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

        if self.control_mode == "direct":
            self.start_direct_tracking(path_msg)
            return

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

    def start_direct_tracking(self, path_msg):
        self.stop_direct_tracking(publish_stop=False)
        self.current_tracking_path = path_msg
        self.tracking_index = 0
        self.control_timer = self.create_timer(1.0 / self.control_frequency, self.direct_tracking_step)

    def stop_direct_tracking(self, publish_stop=True):
        if self.control_timer is not None:
            self.control_timer.cancel()
            self.control_timer = None
        self.current_tracking_path = None
        self.tracking_index = 0
        if publish_stop:
            self.publish_stop_command()

    def direct_tracking_step(self):
        if self.stop_requested:
            self.stop_direct_tracking()
            self.state = "idle"
            self.current_goal = ""
            self.last_message = "Teaching route stopped."
            return
        if self.current_tracking_path is None or len(self.current_tracking_path.poses) < 2:
            self.state = "blocked"
            self.last_message = "Direct teaching tracker has no valid path."
            self.get_logger().warn(self.last_message)
            self.stop_direct_tracking()
            return

        robot_pose = self.lookup_robot_pose()
        if robot_pose is None:
            self.state = "blocked"
            self.last_message = f"Direct teaching tracker lost TF {self.frame_id}->{self.robot_frame}."
            self.get_logger().warn(self.last_message)
            self.stop_direct_tracking()
            return
        px, py, yaw = robot_pose
        poses = self.current_tracking_path.poses
        nearest_index, nearest_error = self.find_nearest_pose_index(poses, px, py)
        self.tracking_index = max(self.tracking_index, nearest_index)

        if nearest_error > self.max_tracking_error:
            self.state = "blocked"
            self.last_message = (
                f"Direct teaching tracker is off path: error={nearest_error:.2f}m "
                f"> {self.max_tracking_error:.2f}m at index {nearest_index}."
            )
            self.get_logger().warn(self.last_message)
            self.stop_direct_tracking()
            return

        goal = poses[-1].pose.position
        goal_error = math.hypot(goal.x - px, goal.y - py)
        if goal_error <= self.tracking_goal_tolerance and self.tracking_index >= len(poses) - 2:
            self.stop_direct_tracking()
            self.handle_segment_arrival()
            return

        target_index = self.find_lookahead_index(poses, self.tracking_index)
        target = poses[target_index].pose.position
        target_yaw = math.atan2(target.y - py, target.x - px)
        heading_error = self.normalize_angle(target_yaw - yaw)
        lateral_error = self.signed_lateral_error(poses, self.tracking_index, px, py)
        angular = self.heading_gain * heading_error - self.lateral_gain * lateral_error
        angular = self.clamp(angular, -self.max_tracking_angular_speed, self.max_tracking_angular_speed)

        speed_scale = max(0.25, 1.0 - min(abs(heading_error), 1.2) / 1.2)
        lateral_scale = max(0.35, 1.0 - min(abs(lateral_error), self.max_tracking_error) / self.max_tracking_error)
        cmd = Twist()
        cmd.linear.x = self.tracking_linear_speed * speed_scale * lateral_scale
        if abs(heading_error) > 1.2:
            cmd.linear.x = 0.0
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def lookup_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.start_tf_timeout_sec),
            )
        except Exception:
            return None
        translation = transform.transform.translation
        yaw = self.yaw_from_quaternion(transform.transform.rotation)
        return translation.x, translation.y, yaw

    def find_nearest_pose_index(self, poses, px, py):
        start = max(0, self.tracking_index - 5)
        best_index = start
        best_distance = float("inf")
        for index in range(start, len(poses)):
            point = poses[index].pose.position
            distance = math.hypot(point.x - px, point.y - py)
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index, best_distance

    def find_lookahead_index(self, poses, start_index):
        distance = 0.0
        previous = poses[start_index].pose.position
        for index in range(start_index + 1, len(poses)):
            point = poses[index].pose.position
            distance += math.hypot(point.x - previous.x, point.y - previous.y)
            if distance >= self.tracking_lookahead_dist:
                return index
            previous = point
        return len(poses) - 1

    def signed_lateral_error(self, poses, index, px, py):
        start = poses[max(0, index - 1)].pose.position
        end = poses[min(len(poses) - 1, index + 1)].pose.position
        dx = end.x - start.x
        dy = end.y - start.y
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return 0.0
        nearest = poses[index].pose.position
        return (dx * (py - nearest.y) - dy * (px - nearest.x)) / norm

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
        self.cmd_pub.publish(Twist())
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

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))


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
