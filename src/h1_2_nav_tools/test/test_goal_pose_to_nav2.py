from h1_2_nav_tools.goal_readiness import goal_readiness_error


def test_goal_requires_expected_frame():
    assert goal_readiness_error("", "camera_init", 0.1, 0.5) == "goal frame is empty"
    assert goal_readiness_error("map", "camera_init", 0.1, 0.5) == (
        "goal frame map does not match camera_init"
    )


def test_goal_requires_fresh_robot_transform():
    assert goal_readiness_error("camera_init", "camera_init", None, 0.5) == (
        "camera_init to body transform is unavailable"
    )
    assert goal_readiness_error("camera_init", "camera_init", 2.52, 0.5) == (
        "camera_init to body transform is stale (2.52s)"
    )
    assert goal_readiness_error("camera_init", "camera_init", 0.1, 0.5) is None


def test_goal_rejects_invalid_or_future_transform_timestamps():
    assert goal_readiness_error("map", "map", float("nan"), 0.5) == (
        "map to body transform timestamp is invalid"
    )
    assert goal_readiness_error("map", "map", -0.2, 0.5) == (
        "map to body transform timestamp is invalid"
    )
