import csv
import json
import math
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from h1_2_icp_localization.msg import LocalizationStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from .metrics import (
    PoseSample,
    compare_trajectories,
    compose_se2,
    summarize_localization,
    summarize_trajectory,
)


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_trajectory(path: Path) -> List[PoseSample]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        return [
            PoseSample(
                stamp=float(row["stamp"]),
                x=float(row["x"]),
                y=float(row["y"]),
                yaw=float(row["yaw_rad"]),
            )
            for row in reader
        ]


class SlamBagEvaluator(Node):
    def __init__(self):
        super().__init__("slam_bag_evaluator")
        self.declare_parameter("mode", "mapping")
        self.declare_parameter("output_dir", "/tmp/h1_2_bag_benchmark")
        self.declare_parameter("reference_csv", "")
        self.declare_parameter("odom_topic", "/slam_odom")
        self.declare_parameter("status_topic", "/icp_localization/status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "camera_init")
        self.declare_parameter("max_time_difference_sec", 0.05)
        self.declare_parameter("max_endpoint_error_m", -1.0)
        self.declare_parameter("max_pose_step_m", -1.0)
        self.declare_parameter("max_position_regression_p95_m", -1.0)
        self.declare_parameter("max_degraded_ratio", -1.0)
        self.declare_parameter("min_converged_ratio", -1.0)

        self.mode = str(self.get_parameter("mode").value)
        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.reference_csv = str(self.get_parameter("reference_csv").value)
        self.map_frame = str(self.get_parameter("map_frame").value).lstrip("/")
        self.odom_frame = str(self.get_parameter("odom_frame").value).lstrip("/")
        self.local_samples: List[PoseSample] = []
        self.global_samples: List[PoseSample] = []
        self.statuses: List[dict] = []
        self.map_to_odom: Optional[Tuple[float, float, float]] = None
        self.report_written = False

        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.on_odom,
            100,
        )
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        self.create_subscription(
            LocalizationStatus,
            str(self.get_parameter("status_topic").value),
            self.on_status,
            20,
        )
        self.get_logger().info(
            f"Evaluating {self.mode} bag output in {self.output_dir}"
        )

    def on_tf(self, message: TFMessage):
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if parent == self.map_frame and child == self.odom_frame:
                translation = transform.transform.translation
                self.map_to_odom = (
                    float(translation.x),
                    float(translation.y),
                    yaw_from_quaternion(transform.transform.rotation),
                )

    def on_odom(self, message: Odometry):
        pose = message.pose.pose
        stamp = stamp_to_seconds(message.header.stamp)
        sample = PoseSample(
            stamp=stamp,
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=yaw_from_quaternion(pose.orientation),
        )
        self.local_samples.append(sample)
        if self.map_to_odom is not None:
            x, y, yaw = compose_se2(
                self.map_to_odom,
                (sample.x, sample.y, sample.yaw),
            )
            self.global_samples.append(PoseSample(stamp, x, y, yaw))

    def on_status(self, message: LocalizationStatus):
        stamp = stamp_to_seconds(message.header.stamp)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1.0e-9
        self.statuses.append(
            {
                "stamp": stamp,
                "converged": bool(message.converged),
                "degraded": bool(message.degraded),
                "score": float(message.score),
                "message": message.message,
            }
        )

    def selected_samples(self) -> List[PoseSample]:
        if self.mode == "localization" and self.global_samples:
            return self.global_samples
        return self.local_samples

    def write_report(self):
        if self.report_written:
            return
        self.report_written = True
        self.output_dir.mkdir(parents=True, exist_ok=True)
        samples = self.selected_samples()
        trajectory_path = self.output_dir / "trajectory.csv"
        with trajectory_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["stamp", "x", "y", "yaw_rad"])
            for sample in samples:
                writer.writerow([f"{sample.stamp:.9f}", sample.x, sample.y, sample.yaw])

        summary = {
            "mode": self.mode,
            "trajectory_frame": self.map_frame if self.global_samples and self.mode == "localization" else self.odom_frame,
            "measurement_scope": (
                "absolute only when reference_csv is independent ground truth; otherwise regression/repeatability"
            ),
            "trajectory": summarize_trajectory(samples),
            "localization": summarize_localization(self.statuses),
        }

        reference_path = Path(self.reference_csv).expanduser() if self.reference_csv else None
        if reference_path is not None and reference_path.is_file():
            summary["reference"] = compare_trajectories(
                load_trajectory(reference_path),
                samples,
                float(self.get_parameter("max_time_difference_sec").value),
            )

        failures = []
        if len(samples) < 2:
            failures.append("fewer than two trajectory samples")
        if self.mode == "localization" and not any(status["converged"] for status in self.statuses):
            failures.append("localization never converged")
        self.check_maximum(
            failures,
            "endpoint error",
            summary["trajectory"]["endpoint_error_m"],
            "max_endpoint_error_m",
        )
        self.check_maximum(
            failures,
            "pose step",
            summary["trajectory"]["max_step_m"],
            "max_pose_step_m",
        )
        self.check_maximum(
            failures,
            "degraded ratio",
            summary["localization"]["degraded_ratio"],
            "max_degraded_ratio",
        )
        if "reference" in summary:
            self.check_maximum(
                failures,
                "position regression p95",
                summary["reference"]["position_error_p95_m"],
                "max_position_regression_p95_m",
            )
        minimum_converged = float(self.get_parameter("min_converged_ratio").value)
        if minimum_converged >= 0.0 and summary["localization"]["converged_ratio"] < minimum_converged:
            failures.append(
                f"converged ratio {summary['localization']['converged_ratio']:.6f} < {minimum_converged:.6f}"
            )
        summary["passed"] = not failures
        summary["failures"] = failures

        report_path = self.output_dir / "summary.json"
        report_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        self.get_logger().info(f"Benchmark report written to {report_path}")

    def check_maximum(self, failures, label, actual, parameter_name):
        maximum = float(self.get_parameter(parameter_name).value)
        if maximum >= 0.0 and actual > maximum:
            failures.append(f"{label} {actual:.6f} > {maximum:.6f}")


def main(args=None):
    rclpy.init(args=args)
    node = SlamBagEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
