"""Unit checks for the coordinator's token and command predicates."""
import math

from h1_2_nav_tools.policy_motion_coordinator_node import valid_command
from geometry_msgs.msg import Twist


def test_rejects_nonfinite_and_nonplanar_commands():
    cmd = Twist()
    assert valid_command(cmd)
    cmd.linear.x = math.nan
    assert not valid_command(cmd)
    cmd.linear.x = 0.0
    cmd.angular.x = 1.0
    assert not valid_command(cmd)


def test_one_goal_slot_across_action_types_and_old_cleanup_cannot_release_new():
    from h1_2_nav_tools.policy_motion_coordinator import MotionCoordinator
    from h1_2_nav_tools.policy_motion_coordinator_node import PolicyMotionCoordinator
    from rclpy.action import GoalResponse

    node = PolicyMotionCoordinator.__new__(PolicyMotionCoordinator)
    node.core = MotionCoordinator('a' * 32, nav_verified=True)
    op = node.core.begin_prepare('nav', node.core.token)
    node.core.finish_prepare(op.operation, isolated=True, zero_barrier=True,
                             node_ready=True, backend_ready=True)
    node.core.activate(op.token)
    node.goal = None
    node._valid_session = lambda session: True
    first, second = {}, {}
    assert node._goal(first, object()) == GoalResponse.ACCEPT
    assert node._goal(second, object()) == GoalResponse.REJECT
    assert node.goal is first


def test_old_velocity_is_rejected_and_invalid_values_fail_closed():
    from h1_2_nav_tools.policy_motion_coordinator import MotionCoordinator
    from h1_2_nav_tools.policy_motion_coordinator_node import PolicyMotionCoordinator

    node = PolicyMotionCoordinator.__new__(PolicyMotionCoordinator)
    node.core = MotionCoordinator('a' * 32)
    node.command = Twist()
    node.last_input = None
    active = object()
    node._valid_session = lambda session: session is active
    reasons = []
    node._invalidate = reasons.append
    msg = Twist()
    msg.linear.x = 2.0
    node._velocity(object(), msg)
    assert node.command.linear.x == 0.0
    msg.linear.x = math.inf
    node._velocity(active, msg)
    assert reasons == ['INVALID_COMMAND']
