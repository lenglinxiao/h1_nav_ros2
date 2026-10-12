import json
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from h1_2_nav_msgs.srv import DeleteMemoryPoint, ListMemoryPoints, SaveMemoryPoint
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from .transform_utils import transform_pose


class MemoryPointManager(Node):
    STORAGE_FRAME = "map"
    IDENTITY_FRAME_ALIASES = {"map", "camera_init"}

    def __init__(self):
        super().__init__("memory_point_manager")
        self.declare_parameter("points_file", "maps/memory_points.json")
        self.declare_parameter("odom_topic", "/slam_odom")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("save_service", "save_memory_point")
        self.declare_parameter("delete_service", "delete_memory_point")
        self.declare_parameter("list_service", "list_memory_points")

        self.points_file = Path(self.get_parameter("points_file").value).expanduser()
        requested_target_frame = self.get_parameter("target_frame").value
        self.target_frame = self.STORAGE_FRAME
        if requested_target_frame != self.target_frame:
            self.get_logger().warn(
                f"Ignoring target_frame={requested_target_frame}; "
                f"memory points are always stored in {self.target_frame}."
            )
        self.latest_pose = None
        self.points = {}
        self.storage_error = None

        self.load_points()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10)
        self.create_service(SaveMemoryPoint, self.get_parameter("save_service").value, self.save_callback)
        self.create_service(DeleteMemoryPoint, self.get_parameter("delete_service").value, self.delete_callback)
        self.create_service(ListMemoryPoints, self.get_parameter("list_service").value, self.list_callback)
        self.get_logger().info(
            f"Memory point manager ready: file={self.points_file}, target_frame={self.target_frame}"
        )

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.latest_pose = pose

    def save_callback(self, request, response):
        name = request.name.strip()
        if not name:
            response.success = False
            response.message = "Memory point name is empty."
            return response
        if self.storage_error:
            response.success = False
            response.message = self.storage_error
            return response
        if name in self.points and not request.overwrite:
            response.success = False
            response.message = f"Memory point already exists: {name}"
            return response
        pose = self.current_pose_in_target_frame()
        if pose is None:
            response.success = False
            response.message = "No current robot pose is available in target frame."
            return response

        self.points[name] = self.pose_to_record(pose)
        try:
            self.write_points()
        except OSError as exc:
            response.success = False
            response.message = f"Failed to write memory points: {exc}"
            return response

        response.success = True
        response.message = f"Saved memory point: {name}"
        response.pose = pose
        return response

    def delete_callback(self, request, response):
        name = request.name.strip()
        if not name:
            response.success = False
            response.message = "Memory point name is empty."
            return response
        if self.storage_error:
            response.success = False
            response.message = self.storage_error
            return response
        if name not in self.points:
            response.success = False
            response.message = f"Memory point does not exist: {name}"
            return response

        del self.points[name]
        try:
            self.write_points()
        except OSError as exc:
            response.success = False
            response.message = f"Failed to write memory points: {exc}"
            return response

        response.success = True
        response.message = f"Deleted memory point: {name}"
        return response

    def list_callback(self, request, response):
        del request
        if self.storage_error:
            response.success = False
            response.message = self.storage_error
            return response

        names = sorted(self.points.keys())
        response.success = True
        response.message = f"Loaded {len(names)} memory points."
        response.names = names
        response.poses = [self.record_to_pose(self.points[name]) for name in names]
        return response

    def current_pose_in_target_frame(self):
        if self.latest_pose is None:
            return None
        if self.latest_pose.header.frame_id in self.IDENTITY_FRAME_ALIASES:
            pose = PoseStamped()
            pose.header.stamp = self.latest_pose.header.stamp
            pose.header.frame_id = self.target_frame
            pose.pose = self.latest_pose.pose
            return pose
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.latest_pose.header.frame_id,
                rclpy.time.Time(),
            )
            return transform_pose(self.latest_pose, transform)
        except Exception as exc:
            self.get_logger().warn(f"Transform to {self.target_frame} failed: {exc}", throttle_duration_sec=2.0)
            return None

    def load_points(self):
        if not self.points_file.exists():
            self.points = {}
            return
        try:
            data = json.loads(self.points_file.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self.storage_error = f"Failed to load memory points file {self.points_file}: {exc}"
            self.get_logger().error(self.storage_error)
            self.points = {}
            return

        frame_id = data.get("frame_id")
        if frame_id and frame_id != self.target_frame:
            self.storage_error = (
                f"Memory points file frame_id={frame_id} differs from "
                f"target_frame={self.target_frame}; refusing to mix coordinate frames."
            )
            self.get_logger().error(self.storage_error)
            self.points = {}
            return
        points = data.get("points", {})
        self.points = points if isinstance(points, dict) else {}

    def write_points(self):
        self.points_file.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "frame_id": self.target_frame,
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "points": self.points,
        }
        temp_path = self.points_file.with_suffix(self.points_file.suffix + ".tmp")
        temp_path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")
        temp_path.replace(self.points_file)

    @staticmethod
    def pose_to_record(pose):
        p = pose.pose.position
        q = pose.pose.orientation
        return {
            "created_at": datetime.now(timezone.utc).isoformat(),
            "position": {"x": p.x, "y": p.y, "z": p.z},
            "orientation": {"x": q.x, "y": q.y, "z": q.z, "w": q.w},
        }

    def record_to_pose(self, record):
        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.header.stamp = self.get_clock().now().to_msg()
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


def main():
    rclpy.init()
    node = None
    try:
        node = MemoryPointManager()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
