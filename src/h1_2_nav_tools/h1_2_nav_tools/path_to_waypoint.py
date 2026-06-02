import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from .transform_utils import transform_pose


class PathToWaypoint(Node):
    def __init__(self):
        super().__init__("path_to_waypoint")
        self.declare_parameter("odom_topic", "/slam_odom")
        self.declare_parameter("path_topic", "/teach_path")
        self.declare_parameter("waypoint_topic", "/way_point")
        self.declare_parameter("visual_path_topic", "/path_with_stairs")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("distance_tolerance", 1.9)
        self.declare_parameter("distance_step", 0.1)
        self.declare_parameter("min_lookahead_distance", 1.0)
        self.declare_parameter("max_lookahead_distance", 2.0)
        self.declare_parameter("high_curvature_threshold", 0.5)
        self.declare_parameter("publish_rate", 50.0)

        self.target_frame = self.get_parameter("target_frame").value
        self.distance_tolerance = float(self.get_parameter("distance_tolerance").value)
        self.distance_step = float(self.get_parameter("distance_step").value)
        self.min_lookahead = float(self.get_parameter("min_lookahead_distance").value)
        self.max_lookahead = float(self.get_parameter("max_lookahead_distance").value)
        self.high_curvature_threshold = float(self.get_parameter("high_curvature_threshold").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_z = 0.0
        self.current_stamp = self.get_clock().now().to_msg()
        self.waypoints = []
        self.last_path_signature = None
        self.curvatures = []
        self.path_index = 0
        self.goal = None
        self.need_publish_goal = False

        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10)
        self.create_subscription(Path, self.get_parameter("path_topic").value, self.path_callback, 10)
        self.waypoint_pub = self.create_publisher(PointStamped, self.get_parameter("waypoint_topic").value, 10)
        self.visual_path_pub = self.create_publisher(Path, self.get_parameter("visual_path_topic").value, 1)

        period = 1.0 / max(float(self.get_parameter("publish_rate").value), 1.0)
        self.create_timer(period, self.update)

    def odom_callback(self, msg):
        local_pose = PoseStamped()
        local_pose.header = msg.header
        local_pose.pose = msg.pose.pose
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, local_pose.header.frame_id, rclpy.time.Time())
            map_pose = transform_pose(local_pose, transform)
        except TransformException as exc:
            self.get_logger().warn(f"TF transform failed: {exc}", throttle_duration_sec=2.0)
            return

        self.current_stamp = map_pose.header.stamp
        self.vehicle_x = map_pose.pose.position.x
        self.vehicle_y = map_pose.pose.position.y
        self.vehicle_z = map_pose.pose.position.z

    def path_callback(self, msg):
        if not msg.poses:
            return

        last = msg.poses[-1].pose.position
        signature = (len(msg.poses), round(last.x, 4), round(last.y, 4), round(last.z, 4))
        if signature == self.last_path_signature:
            return
        self.last_path_signature = signature

        sampled = []
        temp = msg.poses[0].pose.position
        temp_x, temp_y, temp_z = temp.x, temp.y, temp.z
        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position
            dist = math.dist((p.x, p.y, p.z), (temp_x, temp_y, temp_z))
            if dist > self.distance_step:
                sampled.append(pose_stamped)
                temp_x, temp_y, temp_z = p.x, p.y, p.z
        sampled.append(msg.poses[-1])

        self.waypoints = sampled
        self.curvatures = [self.calculate_curvature(sampled, i) for i in range(len(sampled))]
        self.path_index = 0
        self.goal = self.waypoints[0]
        self.need_publish_goal = True

        visual = Path()
        visual.header.frame_id = self.target_frame
        visual.header.stamp = self.get_clock().now().to_msg()
        visual.poses = list(self.waypoints)
        self.visual_path_pub.publish(visual)
        self.get_logger().info(f"Loaded path with {len(msg.poses)} poses, sampled to {len(self.waypoints)} waypoints")

    @staticmethod
    def calculate_curvature(path, index):
        if index == 0 or index >= len(path) - 1:
            return 0.0
        p1 = path[index - 1].pose.position
        p2 = path[index].pose.position
        p3 = path[index + 1].pose.position
        dx1, dy1 = p2.x - p1.x, p2.y - p1.y
        dx2, dy2 = p3.x - p2.x, p3.y - p2.y
        len1 = math.hypot(dx1, dy1)
        len2 = math.hypot(dx2, dy2)
        if len1 < 1e-6 or len2 < 1e-6:
            return 0.0
        dx1, dy1 = dx1 / len1, dy1 / len1
        dx2, dy2 = dx2 / len2, dy2 / len2
        dot = max(-1.0, min(1.0, dx1 * dx2 + dy1 * dy2))
        return math.acos(dot) / max((len1 + len2) * 0.5, 1e-6)

    def lookahead_distance(self, curvature):
        if curvature < self.high_curvature_threshold:
            return self.max_lookahead
        factor = min(curvature / self.high_curvature_threshold, 2.0)
        distance = self.max_lookahead - (self.max_lookahead - self.min_lookahead) * (factor - 1.0)
        return max(distance, self.min_lookahead)

    def update(self):
        if not self.waypoints:
            return

        closest_index = min(
            range(len(self.waypoints)),
            key=lambda i: math.dist(
                (
                    self.waypoints[i].pose.position.x,
                    self.waypoints[i].pose.position.y,
                    self.waypoints[i].pose.position.z,
                ),
                (self.vehicle_x, self.vehicle_y, self.vehicle_z),
            ),
        )

        start = min(closest_index, self.path_index)
        end = max(closest_index, self.path_index)
        segment_curvature = max(self.curvatures[start : end + 1] or [0.0])
        target_distance = self.lookahead_distance(segment_curvature)

        accum = 0.0
        target_index = closest_index
        for i in range(closest_index + 1, len(self.waypoints)):
            a = self.waypoints[i - 1].pose.position
            b = self.waypoints[i].pose.position
            accum += math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
            if accum >= target_distance:
                target_index = i
                break
        if accum < target_distance:
            target_index = len(self.waypoints) - 1

        if target_index != self.path_index:
            self.path_index = target_index
            self.goal = self.waypoints[target_index]
            self.need_publish_goal = True

        if self.need_publish_goal and self.goal is not None:
            msg = PointStamped()
            msg.header.frame_id = self.target_frame
            msg.header.stamp = self.current_stamp
            msg.point = self.goal.pose.position
            self.waypoint_pub.publish(msg)
            self.need_publish_goal = False


def main():
    rclpy.init()
    node = PathToWaypoint()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
