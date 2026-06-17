#!/usr/bin/env python3
"""Smoke test the global relocalization compatibility service."""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from h1_2_icp_localization.srv import Relocalize
from nav_msgs.msg import Odometry
from rclpy.node import Node


ROOT = Path(__file__).resolve().parents[2]
MAP_PATH = ROOT / "maps" / "h1_2_lab.pcd"


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
            "registration_rate_hz:=1.0",
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


class ServiceDriver(Node):
    def __init__(self):
        super().__init__("global_relocalization_service_smoke_driver")
        self.odom_pub = self.create_publisher(Odometry, "/slam_odom", 10)
        self.client = self.create_client(Relocalize, "icp_relocalize")
        self.timer = self.create_timer(0.05, self.publish_odom)

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_init"
        msg.child_frame_id = "body"
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)


def main():
    if not MAP_PATH.exists():
        print(f"real map does not exist: {MAP_PATH}", file=sys.stderr)
        return 2

    process = run_node()
    rclpy.init()
    driver = ServiceDriver()
    try:
        if not driver.client.wait_for_service(timeout_sec=8.0):
            print("icp_relocalize service did not become available", file=sys.stderr)
            return 1

        # Let the node receive at least one odom sample before using a manual pose.
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(driver, timeout_sec=0.05)

        request = Relocalize.Request()
        request.initial_pose.position.x = 1.0
        request.initial_pose.position.y = 2.0
        request.initial_pose.orientation.w = 1.0
        future = driver.client.call_async(request)
        deadline = time.monotonic() + 8.0
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(driver, timeout_sec=0.05)

        if not future.done():
            print("icp_relocalize service call timed out", file=sys.stderr)
            return 1
        response = future.result()
        if not response.success or response.message != "manual relocalize pose accepted":
            print(
                "unexpected service response: "
                f"success={response.success}, score={response.score:.3f}, message={response.message}",
                file=sys.stderr,
            )
            return 1
        print(
            "global relocalization service smoke passed: "
            f"score={response.score:.3f}, message={response.message}"
        )
        return 0
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
