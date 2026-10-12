import math
import struct
import time
from array import array
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


FLOAT32 = PointField.FLOAT32


def downsample_occupancy_grid(data, width: int, height: int, factor: int, unknown_value: int = -1):
    """Conservatively reduce a row-major OccupancyGrid, padding partial blocks as unknown."""
    if width <= 0 or height <= 0 or factor <= 0:
        raise ValueError("width, height, and factor must be positive")
    if len(data) != width * height:
        raise ValueError("occupancy data length does not match grid dimensions")
    out_width = (width + factor - 1) // factor
    out_height = (height + factor - 1) // factor
    padded = np.full((out_height * factor, out_width * factor), unknown_value, dtype=np.int16)
    padded[:height, :width] = np.asarray(data, dtype=np.int16).reshape((height, width))
    blocks = padded.reshape(out_height, factor, out_width, factor).transpose(0, 2, 1, 3)
    occupied = np.any(blocks >= 100, axis=(2, 3))
    free = np.all(blocks == 0, axis=(2, 3))
    reduced = np.full((out_height, out_width), unknown_value, dtype=np.int8)
    reduced[free] = 0
    reduced[occupied] = 100
    return array("b", reduced.tobytes()), out_width, out_height


class PointCloudToOccupancyGrid(Node):
    def __init__(self):
        super().__init__("pointcloud_to_occupancy_grid")

        self.declare_parameter("cloud_topic", "/upper_fastlio2/Laser_map")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("frame_id", "camera_init")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("width_m", 40.0)
        self.declare_parameter("height_m", 40.0)
        self.declare_parameter("origin_x", -20.0)
        self.declare_parameter("origin_y", -20.0)
        self.declare_parameter("map_z", -1.20)
        self.declare_parameter("min_z", -0.35)
        self.declare_parameter("max_z", 1.50)
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("preview_map_topic", "")
        self.declare_parameter("preview_publish_hz", 0.5)
        self.declare_parameter("preview_downsample_factor", 2)
        self.declare_parameter("point_stride", 1)
        self.declare_parameter("inflation_radius", 0.10)
        self.declare_parameter("clear_robot_radius", 0.0)
        self.declare_parameter("cloud_qos_reliable", False)
        self.declare_parameter("unknown_value", -1)
        self.declare_parameter("accumulate_map", True)
        self.declare_parameter("occupancy_hit_increment", 3)
        self.declare_parameter("occupancy_miss_decrement", 1)
        self.declare_parameter("occupancy_threshold", 3)
        self.declare_parameter("occupancy_max_score", 20)
        self.declare_parameter("occupied_decay_after_misses", 6)
        self.declare_parameter("occupied_decay_decrement", 2)
        self.declare_parameter("raytrace_free", False)
        self.declare_parameter("raytrace_clears_occupied", False)
        self.declare_parameter("filter_floating_points", False)
        self.declare_parameter("support_min_z", 0.05)
        self.declare_parameter("support_max_z", 0.30)
        self.declare_parameter("floating_neighbor_radius", 1)
        self.declare_parameter("sensor_origin_x", 0.0)
        self.declare_parameter("sensor_origin_y", 0.0)
        self.declare_parameter("sensor_origin_from_odom", True)
        self.declare_parameter("odom_topic", "/upper_fastlio2/Odometry")

        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.map_topic = self.get_parameter("map_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.resolution = float(self.get_parameter("resolution").value)
        self.width_m = float(self.get_parameter("width_m").value)
        self.height_m = float(self.get_parameter("height_m").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        self.map_z = float(self.get_parameter("map_z").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.preview_map_topic = str(self.get_parameter("preview_map_topic").value).strip()
        self.preview_publish_hz = float(self.get_parameter("preview_publish_hz").value)
        self.preview_downsample_factor = int(self.get_parameter("preview_downsample_factor").value)
        if not math.isfinite(self.preview_publish_hz) or self.preview_publish_hz <= 0:
            raise ValueError("preview_publish_hz must be finite and greater than zero")
        if self.preview_downsample_factor <= 0:
            raise ValueError("preview_downsample_factor must be greater than zero")
        self.preview_period = 1.0 / self.preview_publish_hz
        self.last_preview_publish_monotonic = None
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))
        self.inflation_radius = max(0.0, float(self.get_parameter("inflation_radius").value))
        self.clear_robot_radius = max(0.0, float(self.get_parameter("clear_robot_radius").value))
        self.cloud_qos_reliable = self.as_bool(self.get_parameter("cloud_qos_reliable").value)
        self.unknown_value = int(self.get_parameter("unknown_value").value)
        if self.unknown_value < -1 or self.unknown_value > 100:
            self.unknown_value = -1
        self.accumulate_map = self.as_bool(self.get_parameter("accumulate_map").value)
        self.occupancy_hit_increment = max(1, int(self.get_parameter("occupancy_hit_increment").value))
        self.occupancy_miss_decrement = max(1, int(self.get_parameter("occupancy_miss_decrement").value))
        self.occupancy_threshold = max(1, int(self.get_parameter("occupancy_threshold").value))
        self.occupancy_max_score = max(self.occupancy_threshold, int(self.get_parameter("occupancy_max_score").value))
        self.occupied_decay_after_misses = max(0, int(self.get_parameter("occupied_decay_after_misses").value))
        self.occupied_decay_decrement = max(1, int(self.get_parameter("occupied_decay_decrement").value))
        self.raytrace_free = bool(self.get_parameter("raytrace_free").value)
        self.raytrace_clears_occupied = self.as_bool(self.get_parameter("raytrace_clears_occupied").value)
        self.filter_floating_points = self.as_bool(self.get_parameter("filter_floating_points").value)
        self.support_min_z = float(self.get_parameter("support_min_z").value)
        self.support_max_z = float(self.get_parameter("support_max_z").value)
        self.floating_neighbor_radius = max(0, int(self.get_parameter("floating_neighbor_radius").value))
        self.sensor_origin_x = float(self.get_parameter("sensor_origin_x").value)
        self.sensor_origin_y = float(self.get_parameter("sensor_origin_y").value)
        self.sensor_origin_from_odom = self.as_bool(self.get_parameter("sensor_origin_from_odom").value)

        self.width = max(1, int(math.ceil(self.width_m / self.resolution)))
        self.height = max(1, int(math.ceil(self.height_m / self.resolution)))
        self.inflation_cells = int(math.ceil(self.inflation_radius / self.resolution))
        self.clear_robot_cells = int(math.ceil(self.clear_robot_radius / self.resolution))

        self.latest_grid = [self.unknown_value] * (self.width * self.height)
        self.occupancy_scores = [0] * (self.width * self.height)
        self.occupied_miss_counts = [0] * (self.width * self.height)
        self.has_cloud = False

        cloud_qos = QoSProfile(depth=1)
        cloud_qos.reliability = (
            ReliabilityPolicy.RELIABLE if self.cloud_qos_reliable else ReliabilityPolicy.BEST_EFFORT
        )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.sub = self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_callback, cloud_qos)
        self.odom_sub = None
        if self.sensor_origin_from_odom:
            self.odom_sub = self.create_subscription(
                Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 20
            )
        self.pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self.preview_pub = None
        if self.preview_map_topic:
            preview_qos = QoSProfile(depth=1)
            preview_qos.reliability = ReliabilityPolicy.RELIABLE
            preview_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.preview_pub = self.create_publisher(OccupancyGrid, self.preview_map_topic, preview_qos)
        self.timer = self.create_timer(1.0 / max(self.publish_hz, 0.1), self.publish_map)
        self.cloud_count = 0
        self.publish_count = 0
        self.last_occupied_count = 0
        self.last_free_count = 0

        self.get_logger().info(
            f"Converting {self.cloud_topic} to {self.map_topic}: "
            f"{self.width}x{self.height} @ {self.resolution:.3f} m, "
            f"filter z [{self.min_z:.2f}, {self.max_z:.2f}], map_z {self.map_z:.2f}, "
            f"frame {self.frame_id}, "
            f"cloud_qos {'reliable' if self.cloud_qos_reliable else 'best_effort'}, "
            f"unknown_value {self.unknown_value}, "
            f"accumulate_map {self.accumulate_map}, "
            f"occupancy score hit +{self.occupancy_hit_increment}, "
            f"miss -{self.occupancy_miss_decrement}, threshold {self.occupancy_threshold}, "
            f"occupied decay after {self.occupied_decay_after_misses} misses "
            f"-{self.occupied_decay_decrement}, "
            f"raytrace_free {self.raytrace_free}, "
            f"raytrace_clears_occupied {self.raytrace_clears_occupied}, "
            f"filter_floating_points {self.filter_floating_points}, "
            f"support z [{self.support_min_z:.2f}, {self.support_max_z:.2f}], "
            f"inflation_radius {self.inflation_radius:.2f}, "
            f"clear_robot_radius {self.clear_robot_radius:.2f}"
        )

    def odom_callback(self, msg: Odometry):
        self.sensor_origin_x = msg.pose.pose.position.x
        self.sensor_origin_y = msg.pose.pose.position.y

    def field_offsets(self, msg: PointCloud2) -> Optional[Tuple[int, int, int]]:
        offsets = {}
        for field in msg.fields:
            if field.name in ("x", "y", "z") and field.datatype == FLOAT32:
                offsets[field.name] = field.offset
        if {"x", "y", "z"} <= offsets.keys():
            return offsets["x"], offsets["y"], offsets["z"]
        self.get_logger().warn("PointCloud2 does not contain float32 x/y/z fields", throttle_duration_sec=2.0)
        return None

    def cloud_callback(self, msg: PointCloud2):
        try:
            offsets = self.field_offsets(msg)
            if offsets is None or msg.point_step <= 0:
                return

            x_off, y_off, z_off = offsets
            if not self.accumulate_map:
                self.occupancy_scores = [0] * (self.width * self.height)
                self.occupied_miss_counts = [0] * (self.width * self.height)
            grid = (
                list(self.latest_grid)
                if self.accumulate_map and self.has_cloud
                else [self.unknown_value] * (self.width * self.height)
            )
            total_points = msg.width * msg.height
            data = msg.data
            occupied_count = 0
            free_count = 0
            decayed_count = 0
            origin_i = int((self.sensor_origin_x - self.origin_x) / self.resolution)
            origin_j = int((self.sensor_origin_y - self.origin_y) / self.resolution)
            candidate_points = []
            support_cells = set()
            hit_indices = set()
            accepted_points = 0

            for point_index in range(0, total_points, self.point_stride):
                base = point_index * msg.point_step
                try:
                    x = struct.unpack_from("<f", data, base + x_off)[0]
                    y = struct.unpack_from("<f", data, base + y_off)[0]
                    z = struct.unpack_from("<f", data, base + z_off)[0]
                except struct.error:
                    break

                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                i = int((x - self.origin_x) / self.resolution)
                j = int((y - self.origin_y) / self.resolution)
                if not (0 <= i < self.width and 0 <= j < self.height):
                    continue

                if self.support_min_z <= z <= self.support_max_z:
                    support_cells.add((i, j))
                if self.min_z <= z <= self.max_z:
                    candidate_points.append((i, j))

            for i, j in candidate_points:
                if self.filter_floating_points and not self.has_support_cell(i, j, support_cells):
                    continue
                accepted_points += 1
                if self.raytrace_free and 0 <= origin_i < self.width and 0 <= origin_j < self.height:
                    free_count += self.mark_free_ray(grid, origin_i, origin_j, i, j)
                occupied_count += self.mark_occupied(grid, i, j, hit_indices)

            decayed_count += self.decay_unseen_occupied(grid, hit_indices)
            free_count += self.clear_robot_footprint(grid, origin_i, origin_j)
            self.rebuild_grid_from_scores(grid)
            self.latest_grid = grid
            self.last_occupied_count = occupied_count
            self.last_free_count = free_count
            self.has_cloud = True
            self.cloud_count += 1
            if self.cloud_count == 1 or self.cloud_count % 30 == 0:
                self.get_logger().info(
                    f"Received cloud #{self.cloud_count}: {total_points} points, "
                    f"{len(candidate_points)} z-filtered points, {accepted_points} accepted points, "
                    f"{occupied_count} newly occupied cells, {free_count} free cells, "
                    f"{decayed_count} decayed occupied cells before publish"
                )
        except Exception as exc:
            self.get_logger().error(f"Failed to convert point cloud: {exc}")

    def has_support_cell(self, i: int, j: int, support_cells):
        radius = self.floating_neighbor_radius
        for dj in range(-radius, radius + 1):
            for di in range(-radius, radius + 1):
                if (i + di, j + dj) in support_cells:
                    return True
        return False

    def mark_free_ray(self, grid, x0: int, y0: int, x1: int, y1: int):
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x = x0
        y = y0
        marked = 0

        while True:
            if x == x1 and y == y1:
                break
            index = y * self.width + x
            if self.occupancy_scores[index] >= self.occupancy_threshold:
                if self.raytrace_clears_occupied:
                    self.occupancy_scores[index] = max(
                        0,
                        self.occupancy_scores[index] - self.occupancy_miss_decrement,
                    )
                    if self.occupancy_scores[index] < self.occupancy_threshold:
                        grid[index] = 0
                        marked += 1
            else:
                self.occupancy_scores[index] = max(
                    0,
                    self.occupancy_scores[index] - self.occupancy_miss_decrement,
                )
                if grid[index] != 0:
                    grid[index] = 0
                    marked += 1

            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
            if not (0 <= x < self.width and 0 <= y < self.height):
                break

        return marked

    def mark_occupied(self, grid, i: int, j: int, hit_indices):
        radius = self.inflation_cells
        if radius <= 0:
            index = j * self.width + i
            hit_indices.add(index)
            self.occupied_miss_counts[index] = 0
            before = self.occupancy_scores[index]
            self.occupancy_scores[index] = min(
                self.occupancy_max_score,
                self.occupancy_scores[index] + self.occupancy_hit_increment,
            )
            if before < self.occupancy_threshold <= self.occupancy_scores[index]:
                return 1
            return 0
        radius_sq = radius * radius
        marked = 0
        for dj in range(-radius, radius + 1):
            for di in range(-radius, radius + 1):
                if di * di + dj * dj > radius_sq:
                    continue
                ii = i + di
                jj = j + dj
                if 0 <= ii < self.width and 0 <= jj < self.height:
                    index = jj * self.width + ii
                    hit_indices.add(index)
                    self.occupied_miss_counts[index] = 0
                    before = self.occupancy_scores[index]
                    self.occupancy_scores[index] = min(
                        self.occupancy_max_score,
                        self.occupancy_scores[index] + self.occupancy_hit_increment,
                    )
                    if before < self.occupancy_threshold <= self.occupancy_scores[index]:
                        marked += 1
        return marked

    def decay_unseen_occupied(self, grid, hit_indices):
        if self.occupied_decay_after_misses <= 0:
            return 0
        decayed = 0
        for index, score in enumerate(self.occupancy_scores):
            if index in hit_indices or score < self.occupancy_threshold:
                if score < self.occupancy_threshold:
                    self.occupied_miss_counts[index] = 0
                continue
            self.occupied_miss_counts[index] += 1
            if self.occupied_miss_counts[index] < self.occupied_decay_after_misses:
                continue
            before = self.occupancy_scores[index]
            self.occupancy_scores[index] = max(
                0,
                self.occupancy_scores[index] - self.occupied_decay_decrement,
            )
            if before >= self.occupancy_threshold > self.occupancy_scores[index]:
                grid[index] = 0 if self.raytrace_free else self.unknown_value
                self.occupied_miss_counts[index] = 0
                decayed += 1
        return decayed

    def rebuild_grid_from_scores(self, grid):
        for index, score in enumerate(self.occupancy_scores):
            if score >= self.occupancy_threshold:
                grid[index] = 100
            elif grid[index] == 0:
                continue
            elif score == 0:
                grid[index] = self.unknown_value
            else:
                grid[index] = 0

    def clear_robot_footprint(self, grid, center_i: int, center_j: int):
        radius = self.clear_robot_cells
        if radius <= 0 or not (0 <= center_i < self.width and 0 <= center_j < self.height):
            return 0
        radius_sq = radius * radius
        cleared = 0
        for dj in range(-radius, radius + 1):
            for di in range(-radius, radius + 1):
                if di * di + dj * dj > radius_sq:
                    continue
                ii = center_i + di
                jj = center_j + dj
                if 0 <= ii < self.width and 0 <= jj < self.height:
                    index = jj * self.width + ii
                    if grid[index] != 0 or self.occupancy_scores[index] != 0:
                        self.occupancy_scores[index] = 0
                        self.occupied_miss_counts[index] = 0
                        grid[index] = 0
                        cleared += 1
        return cleared

    def publish_map(self):
        if not self.has_cloud:
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = self.map_z
        msg.info.origin.orientation.w = 1.0
        msg.data = self.latest_grid
        self.pub.publish(msg)
        self.publish_preview_map()
        self.publish_count += 1
        if self.publish_count == 1 or self.publish_count % max(1, int(self.publish_hz * 5.0)) == 0:
            self.get_logger().info(
                f"Published map #{self.publish_count}: {self.width}x{self.height}, "
                f"occupied cells {self.last_occupied_count}, free cells {self.last_free_count}"
            )

    def publish_preview_map(self):
        if self.preview_pub is None or not self.has_cloud:
            return
        try:
            if self.preview_pub.get_subscription_count() <= 0:
                return
            now = time.monotonic()
            if (self.last_preview_publish_monotonic is not None and
                    now - self.last_preview_publish_monotonic < self.preview_period):
                return
            preview_data, preview_width, preview_height = downsample_occupancy_grid(
                self.latest_grid, self.width, self.height,
                self.preview_downsample_factor, self.unknown_value,
            )
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.info.map_load_time = msg.header.stamp
            msg.info.resolution = self.resolution * self.preview_downsample_factor
            msg.info.width = preview_width
            msg.info.height = preview_height
            msg.info.origin.position.x = self.origin_x
            msg.info.origin.position.y = self.origin_y
            msg.info.origin.position.z = self.map_z
            msg.info.origin.orientation.w = 1.0
            msg.data = preview_data
            self.preview_pub.publish(msg)
            self.last_preview_publish_monotonic = now
        except Exception as exc:
            self.get_logger().error(f"Failed to publish preview map: {exc}")

    @staticmethod
    def as_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ("1", "true", "yes", "on")
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PointCloudToOccupancyGrid()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
