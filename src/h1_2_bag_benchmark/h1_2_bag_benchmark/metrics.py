from bisect import bisect_left
from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Sequence, Tuple


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    x: float
    y: float
    yaw: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_se2(
    parent_to_child: Tuple[float, float, float],
    child_to_body: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    px, py, pyaw = parent_to_child
    cx, cy, cyaw = child_to_body
    cosine = math.cos(pyaw)
    sine = math.sin(pyaw)
    return (
        px + cosine * cx - sine * cy,
        py + sine * cx + cosine * cy,
        normalize_angle(pyaw + cyaw),
    )


def percentile(values: Sequence[float], percent: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    weight = position - lower
    return float(ordered[lower] * (1.0 - weight) + ordered[upper] * weight)


def summarize_trajectory(samples: Sequence[PoseSample]) -> Dict[str, float]:
    if not samples:
        return {
            "sample_count": 0,
            "duration_sec": 0.0,
            "sample_rate_hz": 0.0,
            "path_length_m": 0.0,
            "endpoint_error_m": 0.0,
            "endpoint_yaw_error_deg": 0.0,
            "max_step_m": 0.0,
            "step_p95_m": 0.0,
            "max_speed_mps": 0.0,
        }

    steps: List[float] = []
    speeds: List[float] = []
    for previous, current in zip(samples, samples[1:]):
        step = math.hypot(current.x - previous.x, current.y - previous.y)
        steps.append(step)
        delta_time = current.stamp - previous.stamp
        if delta_time > 0.0:
            speeds.append(step / delta_time)

    duration = max(0.0, samples[-1].stamp - samples[0].stamp)
    return {
        "sample_count": len(samples),
        "duration_sec": duration,
        "sample_rate_hz": (len(samples) - 1) / duration if duration > 0.0 else 0.0,
        "path_length_m": sum(steps),
        "endpoint_error_m": math.hypot(
            samples[-1].x - samples[0].x,
            samples[-1].y - samples[0].y,
        ),
        "endpoint_yaw_error_deg": math.degrees(
            abs(normalize_angle(samples[-1].yaw - samples[0].yaw))
        ),
        "max_step_m": max(steps, default=0.0),
        "step_p95_m": percentile(steps, 95.0),
        "max_speed_mps": max(speeds, default=0.0),
    }


def compare_trajectories(
    reference: Sequence[PoseSample],
    actual: Sequence[PoseSample],
    max_time_difference_sec: float = 0.05,
) -> Dict[str, float]:
    if not reference or not actual:
        return {
            "associated_sample_count": 0,
            "position_error_rmse_m": 0.0,
            "position_error_p95_m": 0.0,
            "position_error_max_m": 0.0,
            "yaw_error_p95_deg": 0.0,
            "yaw_error_max_deg": 0.0,
        }

    stamps = [sample.stamp for sample in reference]
    position_errors: List[float] = []
    yaw_errors: List[float] = []
    for sample in actual:
        index = bisect_left(stamps, sample.stamp)
        candidates = []
        if index < len(reference):
            candidates.append(reference[index])
        if index > 0:
            candidates.append(reference[index - 1])
        if not candidates:
            continue
        match = min(candidates, key=lambda candidate: abs(candidate.stamp - sample.stamp))
        if abs(match.stamp - sample.stamp) > max_time_difference_sec:
            continue
        position_errors.append(math.hypot(sample.x - match.x, sample.y - match.y))
        yaw_errors.append(math.degrees(abs(normalize_angle(sample.yaw - match.yaw))))

    if not position_errors:
        return compare_trajectories([], [])
    return {
        "associated_sample_count": len(position_errors),
        "position_error_rmse_m": math.sqrt(
            sum(error * error for error in position_errors) / len(position_errors)
        ),
        "position_error_p95_m": percentile(position_errors, 95.0),
        "position_error_max_m": max(position_errors),
        "yaw_error_p95_deg": percentile(yaw_errors, 95.0),
        "yaw_error_max_deg": max(yaw_errors),
    }


def summarize_localization(statuses: Iterable[dict]) -> Dict[str, float]:
    status_list = list(statuses)
    if not status_list:
        return {
            "status_sample_count": 0,
            "time_to_first_convergence_sec": -1.0,
            "converged_ratio": 0.0,
            "degraded_ratio": 0.0,
            "score_mean": 0.0,
            "score_p95": 0.0,
            "score_max": 0.0,
        }

    first_stamp = status_list[0]["stamp"]
    first_converged_stamp = next(
        (status["stamp"] for status in status_list if status["converged"]),
        None,
    )
    scores = [float(status["score"]) for status in status_list if math.isfinite(status["score"])]
    return {
        "status_sample_count": len(status_list),
        "time_to_first_convergence_sec": (
            first_converged_stamp - first_stamp if first_converged_stamp is not None else -1.0
        ),
        "converged_ratio": sum(bool(status["converged"]) for status in status_list) / len(status_list),
        "degraded_ratio": sum(bool(status["degraded"]) for status in status_list) / len(status_list),
        "score_mean": sum(scores) / len(scores) if scores else 0.0,
        "score_p95": percentile(scores, 95.0),
        "score_max": max(scores, default=0.0),
    }
