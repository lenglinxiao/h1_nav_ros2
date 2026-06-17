import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from h1_2_icp_localization.msg import LocalizationStatus
from h1_2_icp_localization.srv import Relocalize
from h1_2_nav_msgs.srv import StartRoute
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class AutoInitialPose(Node):
    def __init__(self):
        super().__init__("auto_initial_pose")
        self.declare_parameter("points_file", "maps/memory_points.json")
        self.declare_parameter("start_point_name", "START")
        self.declare_parameter("output_topic", "/initialpose")
        self.declare_parameter("odom_topic", "/slam_odom")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("max_publish_count", 10)
        self.declare_parameter("wait_for_tf_target_frame", "map")
        self.declare_parameter("wait_for_tf_source_frame", "camera_init")
        self.declare_parameter("auto_start_route", False)
        self.declare_parameter("route_start_delay_sec", 3.0)
        self.declare_parameter("start_route_service", "start_memory_route")
        self.declare_parameter("required_stable_samples", 5)
        self.declare_parameter("localization_timeout_sec", 15.0)
        self.declare_parameter("localization_backend", "global_gicp")
        self.declare_parameter("icp_status_topic", "/icp_localization/status")
        self.declare_parameter("icp_relocalize_service", "icp_relocalize")
        self.declare_parameter("icp_pcd_path", "")
        self.declare_parameter("wait_for_icp_status", True)
        self.declare_parameter("max_icp_score", 0.15)

        self.points_file = Path(self.get_parameter("points_file").value).expanduser()
        self.start_point_name = self.get_parameter("start_point_name").value
        self.output_topic = self.get_parameter("output_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.publish_period_sec = max(0.1, float(self.get_parameter("publish_period_sec").value))
        self.max_publish_count = max(1, int(self.get_parameter("max_publish_count").value))
        self.wait_for_tf_target_frame = self.get_parameter("wait_for_tf_target_frame").value
        self.wait_for_tf_source_frame = self.get_parameter("wait_for_tf_source_frame").value
        self.auto_start_route = bool(self.get_parameter("auto_start_route").value)
        self.route_start_delay_sec = max(0.0, float(self.get_parameter("route_start_delay_sec").value))
        self.start_route_service = self.get_parameter("start_route_service").value
        self.required_stable_samples = max(1, int(self.get_parameter("required_stable_samples").value))
        self.localization_timeout_sec = max(0.0, float(self.get_parameter("localization_timeout_sec").value))
        self.localization_backend = self.get_parameter("localization_backend").value
        self.icp_status_topic = self.get_parameter("icp_status_topic").value
        self.icp_relocalize_service = self.get_parameter("icp_relocalize_service").value
        self.icp_pcd_path = self.get_parameter("icp_pcd_path").value
        self.wait_for_icp_status = bool(self.get_parameter("wait_for_icp_status").value)
        self.max_icp_score = max(0.0, float(self.get_parameter("max_icp_score").value))

        self.latest_odom_stamp = None
        self.publish_count = 0
        self.initial_pose_published = False
        self.localization_ready_since = None
        self.route_start_requested = False
        self.stable_icp_samples = 0
        self.first_localization_ready_time = None
        self.latest_icp_status = None
        self.icp_relocalize_in_flight = False
        self.start_pose = self.load_start_pose()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, self.output_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(LocalizationStatus, self.icp_status_topic, self.on_icp_status, 10)
        self.start_route_client = self.create_client(StartRoute, self.start_route_service)
        self.icp_relocalize_client = self.create_client(Relocalize, self.icp_relocalize_service)
        self.timer = self.create_timer(self.publish_period_sec, self.on_timer)
        self.get_logger().info(
            f"Auto initial pose ready: point={self.start_point_name}, file={self.points_file}, output={self.output_topic}"
        )

    def on_odom(self, msg):
        self.latest_odom_stamp = msg.header.stamp

    def on_icp_status(self, msg):
        self.latest_icp_status = msg
        if self.icp_status_is_ready(msg):
            self.stable_icp_samples += 1
            if self.stable_icp_samples == self.required_stable_samples:
                self.get_logger().info(
                    f"ICP localization status is stable ({self.stable_icp_samples}/{self.required_stable_samples})."
                )
        else:
            if self.stable_icp_samples > 0:
                self.get_logger().warn("ICP localization status became unstable; waiting again before route start.")
                self.stable_icp_samples = 0

    def load_start_pose(self):
        try:
            data = json.loads(self.points_file.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Failed to load memory points file {self.points_file}: {exc}")
            return None

        points = data.get("points", {})
        record = points.get(self.start_point_name) if isinstance(points, dict) else None
        if record is None:
            self.get_logger().error(f"Initial pose memory point does not exist: {self.start_point_name}")
            return None

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = data.get("frame_id", "map")
        position = record.get("position", {})
        orientation = record.get("orientation", {})
        pose.pose.pose.position.x = float(position.get("x", 0.0))
        pose.pose.pose.position.y = float(position.get("y", 0.0))
        pose.pose.pose.position.z = float(position.get("z", 0.0))
        pose.pose.pose.orientation.x = float(orientation.get("x", 0.0))
        pose.pose.pose.orientation.y = float(orientation.get("y", 0.0))
        pose.pose.pose.orientation.z = float(orientation.get("z", 0.0))
        pose.pose.pose.orientation.w = float(orientation.get("w", 1.0))
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.06853891945200942
        return pose

    def on_timer(self):
        if self.start_pose is None:
            return

        if self.publish_count < self.max_publish_count:
            self.publish_initial_pose()
            return

        if not self.localization_tf_available():
            self.localization_ready_since = None
            if self.publish_count < self.max_publish_count:
                self.publish_initial_pose()
            else:
                self.get_logger().warn(
                    "Auto initial pose was published, but localization TF is still unavailable.",
                    throttle_duration_sec=5.0,
                )
            return

        if self.localization_ready_since is None:
            self.localization_ready_since = self.get_clock().now()
            self.get_logger().info("Localization TF is available; waiting before route start.")
            return

        if self.first_localization_ready_time is None:
            self.first_localization_ready_time = self.localization_ready_since

        if not self.localization_quality_ready():
            self.warn_if_localization_timeout()
            return

        if self.auto_start_route and not self.route_start_requested and self.can_start_route():
            self.request_route_start()

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header = self.start_pose.header
        msg.header.stamp = self.latest_odom_stamp if self.latest_odom_stamp is not None else self.get_clock().now().to_msg()
        msg.pose = self.start_pose.pose
        self.pub.publish(msg)
        self.publish_count += 1
        self.initial_pose_published = True
        self.stable_icp_samples = 0
        if self.localization_backend == "icp":
            self.request_icp_relocalize()
        self.localization_ready_since = None
        self.first_localization_ready_time = None
        self.get_logger().info(
            f"Published auto initial pose {self.start_point_name} ({self.publish_count}/{self.max_publish_count})"
        )

    def can_start_route(self):
        if not self.initial_pose_published or self.localization_ready_since is None:
            return False
        if not self.publish_count > 0:
            return False
        if not self.localization_quality_ready():
            return False
        elapsed = (self.get_clock().now() - self.localization_ready_since).nanoseconds * 1e-9
        return elapsed >= self.route_start_delay_sec

    def icp_status_is_ready(self, msg):
        return msg.initialized and msg.converged and msg.score <= self.max_icp_score

    def localization_quality_ready(self):
        if self.localization_backend in ("icp", "global_gicp"):
            if not self.wait_for_icp_status:
                return True
            if self.latest_icp_status is None:
                return False
            return self.stable_icp_samples >= self.required_stable_samples
        return True

    def warn_if_localization_timeout(self):
        if self.first_localization_ready_time is None or self.localization_timeout_sec <= 0.0:
            return
        elapsed = (self.get_clock().now() - self.first_localization_ready_time).nanoseconds * 1e-9
        if elapsed >= self.localization_timeout_sec:
            self.get_logger().warn(
                "PCD localization is not ready enough; memory route will not auto-start yet. "
                f"latest ICP status: {self.latest_icp_status_summary()}. "
                "Place the robot near START or publish a better initial pose.",
                throttle_duration_sec=5.0,
            )

    def latest_icp_status_summary(self):
        if self.latest_icp_status is None:
            return "no /icp_localization/status received"
        return (
            f"initialized={self.latest_icp_status.initialized}, "
            f"converged={self.latest_icp_status.converged}, "
            f"degraded={self.latest_icp_status.degraded}, "
            f"score={self.latest_icp_status.score:.3f}, "
            f"limit<={self.max_icp_score:.3f}, "
            f"stable_samples={self.stable_icp_samples}/{self.required_stable_samples}, "
            f"message={self.latest_icp_status.message}"
        )

    def localization_tf_available(self):
        if not self.wait_for_tf_target_frame or not self.wait_for_tf_source_frame:
            return True
        try:
            self.tf_buffer.lookup_transform(
                self.wait_for_tf_target_frame,
                self.wait_for_tf_source_frame,
                rclpy.time.Time(),
            )
            return True
        except Exception:
            return False

    def request_route_start(self):
        if not self.start_route_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(f"Start route service is not available: {self.start_route_service}")
            return
        self.route_start_requested = True
        future = self.start_route_client.call_async(StartRoute.Request())
        future.add_done_callback(self.on_route_start_response)
        self.get_logger().info("Localization TF is available; requested memory route start.")

    def request_icp_relocalize(self):
        if self.icp_relocalize_in_flight:
            return
        if self.start_pose is None:
            return
        if not self.icp_relocalize_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(f"ICP relocalize service is not available: {self.icp_relocalize_service}")
            return
        request = Relocalize.Request()
        request.pcd_path = self.icp_pcd_path
        request.initial_pose = self.start_pose.pose.pose
        request.use_multi_align = True
        self.icp_relocalize_in_flight = True
        future = self.icp_relocalize_client.call_async(request)
        future.add_done_callback(self.on_icp_relocalize_response)
        self.get_logger().info("Requested ICP relocalization from auto initial pose.")

    def on_icp_relocalize_response(self, future):
        self.icp_relocalize_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to call ICP relocalize: {exc}")
            return
        if response.success:
            self.get_logger().info(f"ICP relocalize accepted: score={response.score:.3f}")
        else:
            self.get_logger().warn(f"ICP relocalize failed: {response.message}")

    def on_route_start_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to start memory route: {exc}")
            return
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)


def main(args=None):
    rclpy.init(args=args)
    node = AutoInitialPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
