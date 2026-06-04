import heapq
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


GridCell = Tuple[int, int]


class SimpleGridNavigator(Node):
    def __init__(self):
        super().__init__("simple_grid_navigator")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/upper_fastlio2/Odometry")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("raw_cmd_topic", "")
        self.declare_parameter("cmd_topic", "/cmd_vel_nav")
        self.declare_parameter("path_topic", "/nav_path")
        self.declare_parameter("status_topic", "/nav_status")
        self.declare_parameter("global_frame", "")
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("replan_period", 1.0)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("lookahead_distance", 0.7)
        self.declare_parameter("min_linear_speed", 0.06)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 0.6)
        self.declare_parameter("yaw_gain", 1.2)
        self.declare_parameter("slow_yaw_error", 0.8)
        self.declare_parameter("stop_yaw_error", 1.1)
        self.declare_parameter("rotate_to_path_yaw", 0.35)
        self.declare_parameter("walk_yaw_error", 0.18)
        self.declare_parameter("walk_max_angular_speed", 0.12)
        self.declare_parameter("odom_yaw_offset", 0.0)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("allow_unknown", False)
        self.declare_parameter("nearest_free_radius", 12)
        self.declare_parameter("start_nearest_free_radius", 6)
        self.declare_parameter("goal_nearest_free_radius", 12)
        self.declare_parameter("search_limit", 90000)
        self.declare_parameter("data_timeout", 1.0)

        self.global_frame = str(self.get_parameter("global_frame").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.replan_period = float(self.get_parameter("replan_period").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.min_linear_speed = float(self.get_parameter("min_linear_speed").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.slow_yaw_error = float(self.get_parameter("slow_yaw_error").value)
        self.stop_yaw_error = float(self.get_parameter("stop_yaw_error").value)
        self.rotate_to_path_yaw = float(self.get_parameter("rotate_to_path_yaw").value)
        self.walk_yaw_error = float(self.get_parameter("walk_yaw_error").value)
        self.walk_max_angular_speed = float(self.get_parameter("walk_max_angular_speed").value)
        self.odom_yaw_offset = float(self.get_parameter("odom_yaw_offset").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.allow_unknown = self.as_bool(self.get_parameter("allow_unknown").value)
        self.nearest_free_radius = int(self.get_parameter("nearest_free_radius").value)
        self.start_nearest_free_radius = int(self.get_parameter("start_nearest_free_radius").value)
        self.goal_nearest_free_radius = int(self.get_parameter("goal_nearest_free_radius").value)
        self.search_limit = int(self.get_parameter("search_limit").value)
        self.data_timeout = float(self.get_parameter("data_timeout").value)

        self.map_msg: Optional[OccupancyGrid] = None
        self.goal: Optional[PoseStamped] = None
        self.pose: Optional[PoseStamped] = None
        self.path: List[Tuple[float, float]] = []
        self.last_plan_time = self.get_clock().now()
        self.last_map_time = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()
        self.active = False
        self.last_status_text = ""

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value, self.map_callback, 1)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 20)
        self.create_subscription(PoseStamped, self.get_parameter("goal_topic").value, self.goal_callback, 5)
        self.create_subscription(PointStamped, self.get_parameter("clicked_point_topic").value, self.clicked_point_callback, 5)

        raw_cmd_topic = str(self.get_parameter("raw_cmd_topic").value)
        if not raw_cmd_topic:
            raw_cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.cmd_pub = self.create_publisher(Twist, raw_cmd_topic, 10)
        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 1)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, status_qos)

        self.create_timer(1.0 / max(self.control_rate, 1.0), self.update)
        self.get_logger().info(
            f"Simple navigator ready: goal {self.get_parameter('goal_topic').value}, "
            f"clicked point {self.get_parameter('clicked_point_topic').value}, "
            f"map {self.get_parameter('map_topic').value}, cmd {raw_cmd_topic}"
        )

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.last_map_time = self.get_clock().now()
        if not self.global_frame:
            self.global_frame = msg.header.frame_id

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.pose = self.transform_pose_to_global(pose)
        self.apply_odom_yaw_offset(self.pose)
        self.last_odom_time = self.get_clock().now()

    def goal_callback(self, msg: PoseStamped):
        self.set_goal(msg, "goal")

    def clicked_point_callback(self, msg: PointStamped):
        goal = PoseStamped()
        goal.header = msg.header
        goal.pose.position.x = msg.point.x
        goal.pose.position.y = msg.point.y
        goal.pose.position.z = msg.point.z
        goal.pose.orientation.w = 1.0
        self.set_goal(goal, "clicked point")

    def set_goal(self, msg: PoseStamped, source: str):
        self.goal = self.transform_pose_to_global(msg)
        self.active = True
        self.path = []
        self.publish_status(
            f"Received {source} ({self.goal.pose.position.x:.2f}, {self.goal.pose.position.y:.2f})"
        )
        self.plan_path(force=True)

    def update(self):
        if not self.active:
            return
        if self.map_msg is None or self.pose is None or self.goal is None:
            self.stop("Waiting for map, odom, or goal")
            return
        if self.data_is_stale():
            self.stop("Waiting for fresh map/odom")
            return

        px = self.pose.pose.position.x
        py = self.pose.pose.position.y
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        if math.hypot(gx - px, gy - py) <= self.goal_tolerance:
            self.active = False
            self.path = []
            self.publish_path()
            self.stop("Goal reached")
            return

        now = self.get_clock().now()
        if not self.path or (now - self.last_plan_time).nanoseconds * 1e-9 >= self.replan_period:
            self.plan_path(force=False)

        if not self.path:
            self.stop("No valid path")
            return

        target = self.select_reachable_lookahead(px, py)
        if target is None:
            self.stop("No path target")
            return

        yaw = self.yaw_from_pose(self.pose)
        target_yaw = math.atan2(target[1] - py, target[0] - px)
        yaw_error = self.normalize_angle(target_yaw - yaw)

        cmd = Twist()
        if abs(yaw_error) > self.rotate_to_path_yaw:
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, self.yaw_gain * yaw_error))
        else:
            speed_scale = max(0.0, 1.0 - abs(yaw_error) / max(self.walk_yaw_error, 0.01))
            min_scale = self.min_linear_speed / max(self.max_linear_speed, 0.01)
            cmd.linear.x = self.max_linear_speed * min(1.0, max(min_scale, speed_scale))
            correction = max(-self.walk_max_angular_speed, min(self.walk_max_angular_speed, self.yaw_gain * yaw_error))
            cmd.angular.z = correction
        self.cmd_pub.publish(cmd)

    def plan_path(self, force: bool):
        if self.map_msg is None or self.pose is None or self.goal is None:
            return
        if not force and self.path and self.path_is_still_free():
            self.last_plan_time = self.get_clock().now()
            return

        start = self.world_to_cell(self.pose.pose.position.x, self.pose.pose.position.y)
        goal = self.world_to_cell(self.goal.pose.position.x, self.goal.pose.position.y)
        if start is None or goal is None:
            self.path = []
            self.publish_status("Start or goal outside map")
            return

        start = self.nearest_free(start, self.start_nearest_free_radius)
        goal = self.nearest_free(goal, self.goal_nearest_free_radius)
        if start is None or goal is None:
            self.path = []
            self.publish_status("Start or goal is not near free space")
            return

        plan_start_time = self.get_clock().now()
        cells = self.astar(start, goal)
        self.last_plan_time = self.get_clock().now()
        plan_time = (self.last_plan_time - plan_start_time).nanoseconds * 1e-9
        if not cells:
            self.path = []
            self.publish_status(f"A* failed after {plan_time:.2f}s")
            self.publish_path()
            return

        self.path = [self.cell_to_world(c) for c in self.smooth_path(cells)]
        self.publish_path()
        self.publish_status(f"Planned path with {len(self.path)} points in {plan_time:.2f}s")

    def astar(self, start: GridCell, goal: GridCell) -> List[GridCell]:
        open_heap = [(0.0, start)]
        came_from: Dict[GridCell, GridCell] = {}
        cost_so_far: Dict[GridCell, float] = {start: 0.0}
        visited = 0

        while open_heap and visited < self.search_limit:
            _, current = heapq.heappop(open_heap)
            visited += 1
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for nb, step_cost in self.neighbors(current):
                new_cost = cost_so_far[current] + step_cost
                if nb not in cost_so_far or new_cost < cost_so_far[nb]:
                    cost_so_far[nb] = new_cost
                    priority = new_cost + math.hypot(goal[0] - nb[0], goal[1] - nb[1])
                    heapq.heappush(open_heap, (priority, nb))
                    came_from[nb] = current
        return []

    def neighbors(self, cell: GridCell):
        for dx, dy, cost in (
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414),
        ):
            nb = (cell[0] + dx, cell[1] + dy)
            if self.is_free(nb):
                yield nb, cost

    def reconstruct_path(self, came_from: Dict[GridCell, GridCell], current: GridCell):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def smooth_path(self, cells: List[GridCell]):
        if len(cells) <= 2:
            return cells
        out = [cells[0]]
        anchor = cells[0]
        for i in range(2, len(cells)):
            if not self.line_is_free(anchor, cells[i]):
                anchor = cells[i - 1]
                out.append(anchor)
        out.append(cells[-1])
        return out

    def path_is_still_free(self):
        for x, y in self.path:
            cell = self.world_to_cell(x, y)
            if cell is None or not self.is_free(cell):
                return False
        return True

    def line_is_free(self, a: GridCell, b: GridCell):
        x0, y0 = a
        x1, y1 = b
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            if not self.is_free((x, y)):
                return False
            if x == x1 and y == y1:
                return True
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

    def nearest_free(self, cell: GridCell, radius_limit: Optional[int] = None) -> Optional[GridCell]:
        if self.is_free(cell):
            return cell
        max_radius = self.nearest_free_radius if radius_limit is None else max(0, int(radius_limit))
        for radius in range(1, max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if self.is_free(candidate):
                        return candidate
        return None

    def is_free(self, cell: GridCell):
        if self.map_msg is None:
            return False
        i, j = cell
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        if i < 0 or i >= width or j < 0 or j >= height:
            return False
        value = self.map_msg.data[j * width + i]
        if value < 0:
            return self.allow_unknown
        return value < self.occupied_threshold

    def world_to_cell(self, x: float, y: float) -> Optional[GridCell]:
        if self.map_msg is None:
            return None
        info = self.map_msg.info
        i = int((x - info.origin.position.x) / info.resolution)
        j = int((y - info.origin.position.y) / info.resolution)
        if 0 <= i < info.width and 0 <= j < info.height:
            return i, j
        return None

    def cell_to_world(self, cell: GridCell):
        info = self.map_msg.info
        return (
            info.origin.position.x + (cell[0] + 0.5) * info.resolution,
            info.origin.position.y + (cell[1] + 0.5) * info.resolution,
        )

    def select_lookahead(self, px: float, py: float):
        if not self.path:
            return None
        closest = min(range(len(self.path)), key=lambda idx: math.hypot(self.path[idx][0] - px, self.path[idx][1] - py))
        for point in self.path[closest:]:
            if math.hypot(point[0] - px, point[1] - py) >= self.lookahead_distance:
                return point
        return self.path[-1]

    def select_reachable_lookahead(self, px: float, py: float):
        if not self.path:
            return None
        robot_cell = self.world_to_cell(px, py)
        if robot_cell is None:
            return None
        robot_cell = self.nearest_free(robot_cell, self.start_nearest_free_radius) or robot_cell

        closest = min(range(len(self.path)), key=lambda idx: math.hypot(self.path[idx][0] - px, self.path[idx][1] - py))
        fallback = self.path[-1]
        for point in self.path[closest:]:
            if math.hypot(point[0] - px, point[1] - py) < self.lookahead_distance:
                fallback = point
                continue
            target_cell = self.world_to_cell(point[0], point[1])
            if target_cell is not None and self.line_is_free(robot_cell, target_cell):
                return point
        target_cell = self.world_to_cell(fallback[0], fallback[1])
        if target_cell is not None and self.line_is_free(robot_cell, target_cell):
            return fallback
        return None

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = self.global_frame or (self.map_msg.header.frame_id if self.map_msg else "camera_init")
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def stop(self, reason: str):
        self.cmd_pub.publish(Twist())
        self.publish_status(reason)

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        if text != self.last_status_text:
            self.get_logger().info(text)
            self.last_status_text = text

    def data_is_stale(self):
        now = self.get_clock().now()
        if (now - self.last_odom_time).nanoseconds * 1e-9 > self.data_timeout:
            return True
        if (now - self.last_map_time).nanoseconds * 1e-9 > max(self.data_timeout, 2.0):
            return True
        return False

    def transform_pose_to_global(self, pose: PoseStamped):
        target_frame = self.global_frame or (self.map_msg.header.frame_id if self.map_msg else pose.header.frame_id)
        if not target_frame or not pose.header.frame_id or pose.header.frame_id == target_frame:
            pose.header.frame_id = target_frame or pose.header.frame_id
            return pose
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, pose.header.frame_id, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(
                f"TF unavailable {pose.header.frame_id}->{target_frame}, using raw pose: {exc}",
                throttle_duration_sec=2.0,
            )
            return pose
        return self.apply_2d_transform(pose, transform)

    @classmethod
    def apply_2d_transform(cls, pose: PoseStamped, transform):
        out = PoseStamped()
        out.header.stamp = pose.header.stamp
        out.header.frame_id = transform.header.frame_id
        tf_yaw = cls.yaw_from_quaternion(transform.transform.rotation)
        cos_yaw = math.cos(tf_yaw)
        sin_yaw = math.sin(tf_yaw)
        x = pose.pose.position.x
        y = pose.pose.position.y
        out.pose.position.x = transform.transform.translation.x + cos_yaw * x - sin_yaw * y
        out.pose.position.y = transform.transform.translation.y + sin_yaw * x + cos_yaw * y
        out.pose.position.z = pose.pose.position.z + transform.transform.translation.z
        yaw = cls.yaw_from_pose(pose) + tf_yaw
        out.pose.orientation.z = math.sin(yaw * 0.5)
        out.pose.orientation.w = math.cos(yaw * 0.5)
        return out

    @staticmethod
    def yaw_from_pose(pose: PoseStamped):
        return SimpleGridNavigator.yaw_from_quaternion(pose.pose.orientation)

    def apply_odom_yaw_offset(self, pose: PoseStamped):
        if abs(self.odom_yaw_offset) < 1e-9:
            return
        yaw = self.normalize_angle(self.yaw_from_pose(pose) + self.odom_yaw_offset)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def normalize_angle(angle: float):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def as_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ("1", "true", "yes", "on")
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleGridNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
