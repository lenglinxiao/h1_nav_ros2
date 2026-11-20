import math


def goal_readiness_error(goal_frame, expected_frame, transform_age_sec, max_age_sec):
    if not goal_frame:
        return "goal frame is empty"
    if goal_frame != expected_frame:
        return f"goal frame {goal_frame} does not match {expected_frame}"
    if transform_age_sec is None:
        return f"{expected_frame} to body transform is unavailable"
    if not math.isfinite(transform_age_sec) or transform_age_sec < 0.0:
        return f"{expected_frame} to body transform timestamp is invalid"
    if transform_age_sec > max_age_sec:
        return f"{expected_frame} to body transform is stale ({transform_age_sec:.2f}s)"
    return None
