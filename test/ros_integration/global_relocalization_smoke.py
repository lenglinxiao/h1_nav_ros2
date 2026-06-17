#!/usr/bin/env python3
"""Smoke test for the H1 global relocalization node with synthetic data.

This starts the C++ node, publishes a deterministic body-frame cloud and odom,
and waits for /icp_localization/status to report a converged initialization.
It does not require Livox hardware, Nav2, or robot networking.
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


MAP_PATH = Path("/tmp/h1_global_relocalization_smoke_map.pcd")


def generate_points():
    points = []
    for ix in range(-7, 8):
        for iy in range(-5, 6):
            x = ix * 0.35
            y = iy * 0.31
            z = 0.25 * math.sin(ix * 0.7) + 0.18 * math.cos(iy * 0.5)
            intensity = float((ix + 10) * (iy + 8) % 255)
            points.append((x, y, z, intensity))
    for i in range(90):
        angle = i * 0.31
        radius = 1.2 + 0.02 * i
        points.append((radius * math.cos(angle), radius * math.sin(angle), 0.45 + 0.003 * i, 80.0))
    return points


def write_pcd(path, points):
    with path.open("w", encoding="ascii") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
        stream.write("VERSION 0.7\n")
        stream.write("FIELDS x y z intensity\n")
        stream.write("SIZE 4 4 4 4\n")
        stream.write("TYPE F F F F\n")
        stream.write("COUNT 1 1 1 1\n")
        stream.write(f"WIDTH {len(points)}\n")
        stream.write("HEIGHT 1\n")
        stream.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        stream.write(f"POINTS {len(points)}\n")
        stream.write("DATA ascii\n")
        for x, y, z, intensity in points:
            stream.write(f"{x:.6f} {y:.6f} {z:.6f} {intensity:.3f}\n")


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


class SmokeDriver(Node):
    def __init__(self, points):
        super().__init__("global_relocalization_smoke_driver")
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
            "recovery_min_points:=200",
            "-p",
            "global_voxel_resolution:=0.25",
            "-p",
            "tracking_voxel_resolution:=0.20",
            "-p",
            "global_score_threshold:=2.0",
            "-p",
            "gicp_score_threshold:=1.0",
            "-p",
            "registration_rate_hz:=2.0",
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def main():
    points = generate_points()
    write_pcd(MAP_PATH, points)
    process = run_node()
    rclpy.init()
    driver = SmokeDriver(points)
    deadline = time.monotonic() + 12.0
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
                        "global relocalization smoke passed: "
                        f"score={driver.status.score:.3f}, message={driver.status.message}"
                    )
                    return 0
        print("global relocalization smoke failed: did not reach tracking success", file=sys.stderr)
        if driver.status:
            print(
                f"last status initialized={driver.status.initialized} "
                f"converged={driver.status.converged} score={driver.status.score:.3f} "
                f"message={driver.status.message}",
                file=sys.stderr,
            )
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
