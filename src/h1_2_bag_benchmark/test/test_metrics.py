import math

import pytest
import numpy as np

from h1_2_bag_benchmark.metrics import (
    PoseSample,
    compare_trajectories,
    compose_se2,
    summarize_localization,
    summarize_trajectory,
)
from h1_2_bag_benchmark.pcd_compare import compare_point_clouds, load_pcd_xyz


def test_square_loop_reports_length_and_endpoint_error():
    samples = [
        PoseSample(0.0, 0.0, 0.0, 0.0),
        PoseSample(1.0, 1.0, 0.0, 0.0),
        PoseSample(2.0, 1.0, 1.0, math.pi / 2.0),
        PoseSample(3.0, 0.0, 1.0, math.pi),
        PoseSample(4.0, 0.0, 0.0, -math.pi / 2.0),
    ]

    summary = summarize_trajectory(samples)

    assert summary["sample_count"] == 5
    assert summary["duration_sec"] == pytest.approx(4.0)
    assert summary["path_length_m"] == pytest.approx(4.0)
    assert summary["endpoint_error_m"] == pytest.approx(0.0)
    assert summary["max_step_m"] == pytest.approx(1.0)


def test_trajectory_comparison_uses_timestamps_and_reports_regression():
    reference = [
        PoseSample(10.0, 0.0, 0.0, 0.0),
        PoseSample(11.0, 1.0, 0.0, 0.0),
        PoseSample(12.0, 2.0, 0.0, 0.0),
    ]
    actual = [
        PoseSample(10.01, 0.1, 0.0, math.radians(1.0)),
        PoseSample(11.01, 1.1, 0.0, math.radians(1.0)),
        PoseSample(12.01, 2.1, 0.0, math.radians(1.0)),
    ]

    summary = compare_trajectories(reference, actual, max_time_difference_sec=0.05)

    assert summary["associated_sample_count"] == 3
    assert summary["position_error_rmse_m"] == pytest.approx(0.1)
    assert summary["position_error_p95_m"] == pytest.approx(0.1)
    assert summary["yaw_error_p95_deg"] == pytest.approx(1.0)


def test_localization_summary_tracks_convergence_degradation_and_score():
    statuses = [
        {"stamp": 2.0, "converged": False, "degraded": False, "score": 0.5},
        {"stamp": 3.0, "converged": True, "degraded": False, "score": 0.2},
        {"stamp": 4.0, "converged": True, "degraded": True, "score": 0.3},
        {"stamp": 5.0, "converged": True, "degraded": False, "score": 0.1},
    ]

    summary = summarize_localization(statuses)

    assert summary["status_sample_count"] == 4
    assert summary["time_to_first_convergence_sec"] == pytest.approx(1.0)
    assert summary["converged_ratio"] == pytest.approx(0.75)
    assert summary["degraded_ratio"] == pytest.approx(0.25)
    assert summary["score_p95"] == pytest.approx(0.47)


def test_compose_se2_applies_map_to_odom_before_odom_to_body():
    x, y, yaw = compose_se2((1.0, 2.0, math.pi / 2.0), (2.0, 0.0, math.pi / 2.0))

    assert x == pytest.approx(1.0)
    assert y == pytest.approx(4.0)
    assert yaw == pytest.approx(math.pi)


def test_point_cloud_comparison_reports_symmetric_nearest_neighbor_error():
    reference = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    actual = np.array([[0.1, 0.0, 0.0], [1.1, 0.0, 0.0]])

    summary = compare_point_clouds(reference, actual, overlap_distance_m=0.15)

    assert summary["symmetric_rmse_m"] == pytest.approx(0.1)
    assert summary["symmetric_p95_m"] == pytest.approx(0.1)
    assert summary["symmetric_overlap_ratio"] == pytest.approx(1.0)


def test_ascii_pcd_loader_reads_xyz_and_ignores_other_fields(tmp_path):
    pcd = tmp_path / "map.pcd"
    pcd.write_text(
        "# .PCD v0.7\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "WIDTH 2\n"
        "HEIGHT 1\n"
        "POINTS 2\n"
        "DATA ascii\n"
        "1 2 3 10\n"
        "4 5 6 20\n",
        encoding="ascii",
    )

    points = load_pcd_xyz(pcd)

    assert points.tolist() == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
