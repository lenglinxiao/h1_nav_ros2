#!/usr/bin/env python3
"""Smoke test global relocalization against the real H1 lab PCD map.

The test crops a local scan from maps/h1_2_lab.pcd, transforms it into a
synthetic body frame, starts global_relocalization_node, and waits for global
initialization followed by GICP tracking. It still uses mock ROS topics, but
the geometry comes from the real saved map instead of a toy point set.
"""

import math
import os
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from h1_2_icp_localization.msg import LocalizationStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


ROOT = Path(__file__).resolve().parents[2]
MAP_PATH = ROOT / "maps" / "h1_2_lab.pcd"


def read_binary_pcd_xyzi(path):
    raw = path.read_bytes()
    header_end = raw.index(b"DATA binary\n") + len(b"DATA binary\n")
    header = raw[:header_end].decode("ascii", errors="replace")
    fields = []
    sizes = []
    counts = []
    point_count = 0
    for line in header.splitlines():
        if line.startswith("FIELDS "):
            fields = line.split()[1:]
        elif line.startswith("SIZE "):
            sizes = [int(value) for value in line.split()[1:]]
        elif line.startswith("COUNT "):
            counts = [int(value) for value in line.split()[1:]]
        elif line.startswith("POINTS "):
            point_count = int(line.split()[1])

    if not fields or not sizes or not counts or point_count <= 0:
        raise RuntimeError(f"Unsupported PCD header in {path}")

    offsets = {}
    offset = 0
    for field, size, count in zip(fields, sizes, counts):
        offsets[field] = offset
        offset += size * count
    point_step = offset
    required = ("x", "y", "z", "intensity")
    if any(field not in offsets for field in required):
        raise RuntimeError(f"PCD missing required fields {required}: {path}")

    data = raw[header_end:]
    points = []
    for index in range(point_count):
        base = index * point_step
        x = struct.unpack_from("<f", data, base + offsets["x"])[0]
        y = struct.unpack_from("<f", data, base + offsets["y"])[0]
        z = struct.unpack_from("<f", data, base + offsets["z"])[0]
        intensity = struct.unpack_from("<f", data, base + offsets["intensity"])[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((x, y, z, intensity))
    return points


def choose_pose(points):
    # Choose a stable dense-ish pose near the middle of the map footprint.
    xs = sorted(point[0] for point in points)
    ys = sorted(point[1] for point in points)
    return xs[len(xs) // 2], ys[len(ys) // 2], 0.0


def crop_points_near_pose(points, pose, radius=7.5, max_points=5500):
    px, py, yaw = pose
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    radius2 = radius * radius
    cropped = []
    for x, y, z, intensity in points:
        dx = x - px
        dy = y - py
        if dx * dx + dy * dy > radius2:
            continue
        body_x = cos_yaw * dx + sin_yaw * dy
        body_y = -sin_yaw * dx + cos_yaw * dy
        cropped.append((body_x, body_y, z, intensity))
        if len(cropped) >= max_points:
            break
    if len(cropped) < 1000:
        raise RuntimeError(f"Real-map crop is too sparse: {len(cropped)} points")
    return cropped


def make_cloud(points, stamp):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = "body"
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 32
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    data = bytearray(msg.row_step)
    for index, point in enumerate(points):
        x, y, z, intensity = point
        offset = index * msg.point_step
        struct.pack_into("<fff", data, offset, x, y, z)
        struct.pack_into("<f", data, offset + 16, intensity)
    msg.data = bytes(data)
    return msg


def make_odom(stamp):
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = "camera_init"
    msg.child_frame_id = "body"
    msg.pose.pose.orientation.w = 1.0
    return msg


class RealMapSmokeDriver(Node):
    def __init__(self, points):
        super().__init__("global_relocalization_real_map_smoke_driver")
        self.points = points
        self.status = None
        self.odom_pub = self.create_publisher(Odometry, "/slam_odom", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/cloud_registered_body", 10)
        self.create_subscription(LocalizationStatus, "/icp_localization/status", self.on_status, 10)
        self.timer = self.create_timer(0.1, self.publish_inputs)

    def on_status(self, msg):
        self.status = msg

    def publish_inputs(self):
        stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(make_odom(stamp))
        self.cloud_pub.publish(make_cloud(self.points, stamp))


def run_node():
    env = os.environ.copy()
    env.setdefault("ROS_LOG_DIR", "/tmp/h1_ros_log")
    return subprocess.Popen(
        [
            "ros2",
            "run",
            "h1_2_icp_localization",
            "global_relocalization_node",
            "--ros-args",
            "-p",
            f"pcd_path:={MAP_PATH}",
            "-p",
            "recovery_min_points:=1000",
            "-p",
            "min_input_points:=500",
            "-p",
            "max_accumulated_points:=30000",
            "-p",
            "global_voxel_resolution:=0.45",
            "-p",
            "tracking_voxel_resolution:=0.25",
            "-p",
            "normal_radius:=1.0",
            "-p",
            "feature_radius:=2.0",
            "-p",
            "global_score_threshold:=2.5",
            "-p",
            "gicp_score_threshold:=1.5",
            "-p",
            "gicp_max_correspondence_distance:=2.0",
            "-p",
            "registration_rate_hz:=1.0",
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def main():
    if not MAP_PATH.exists():
        print(f"real map does not exist: {MAP_PATH}", file=sys.stderr)
        return 2

    map_points = read_binary_pcd_xyzi(MAP_PATH)
    scan_points = crop_points_near_pose(map_points, choose_pose(map_points))
    process = run_node()
    rclpy.init()
    driver = RealMapSmokeDriver(scan_points)
    deadline = time.monotonic() + 30.0
    saw_global_init = False
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(driver, timeout_sec=0.1)
            if not driver.status:
                continue
            if driver.status.initialized and driver.status.converged:
                if "global initialization succeeded" in driver.status.message:
                    saw_global_init = True
                if saw_global_init and "GICP tracking succeeded" in driver.status.message:
                    print(
                        "real-map global relocalization smoke passed: "
                        f"score={driver.status.score:.3f}, points={len(scan_points)}"
                    )
                    return 0

        print("real-map global relocalization smoke failed", file=sys.stderr)
        if driver.status:
            print(
                f"last status initialized={driver.status.initialized} "
                f"converged={driver.status.converged} score={driver.status.score:.3f} "
                f"message={driver.status.message}",
                file=sys.stderr,
            )
        output = process.stdout.read() if process.stdout else ""
        if output:
            print(output[-4000:], file=sys.stderr)
        return 1
    finally:
        driver.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()


if __name__ == "__main__":
    sys.exit(main())
