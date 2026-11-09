"""Localhost-only integration tests with two fake Nav2 action servers."""
import threading
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from h1_2_policy_interfaces.srv import ActivateMotionSource, ClaimPolicyInput, PrepareMotionSource
from h1_2_nav_tools.policy_motion_coordinator_node import PolicyMotionCoordinator, token_msg
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient, ActionServer
from rclpy.action import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def wait(predicate, timeout=4):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('timed out')


def call(client, request):
    wait(client.service_is_ready)
    future = client.call_async(request)
    wait(future.done)
    return future.result()


def test_services_and_both_fake_nav_actions():
    rclpy.init()
    coordinator = PolicyMotionCoordinator(owner_boot_id='a' * 32, nav_verified=True)
    fake = Node('test_fake_nav')
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(coordinator)
    executor.add_node(fake)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    entities = []
    started, release = threading.Event(), threading.Event()
    try:
        claim = fake.create_client(ClaimPolicyInput, '/h1_2/policy/confirm_input_claim')
        prepare = fake.create_client(PrepareMotionSource, '/h1_2/policy/prepare_source')
        activate = fake.create_client(ActivateMotionSource, '/h1_2/policy/activate_source')
        hold = fake.create_client(Trigger, '/h1_2/policy/hold')
        request = ClaimPolicyInput.Request()
        request.owner_boot_id = 'a' * 32
        gids = coordinator.publisher_gids()
        request.cmd_writer_gid, request.heartbeat_writer_gid = list(gids[0]), list(gids[1])
        assert call(claim, request).success
        req = PrepareMotionSource.Request()
        req.source, req.expected_token = 'nav', token_msg(coordinator.core.token)
        prepared = call(prepare, req)
        assert prepared.success
        backend = prepared.action_namespace.rsplit('/', 1)[0] + '/backend'
        group = ReentrantCallbackGroup()
        blocking = {'value': False}
        for name, typ in (('navigate_to_pose', NavigateToPose), ('navigate_through_poses', NavigateThroughPoses)):
            def execute(handle, action_type=typ):
                if blocking['value']:
                    started.set()
                    release.wait(3)
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.succeed()
                return action_type.Result()
            entities.append(ActionServer(fake, typ, backend + '/' + name, execute_callback=execute,
                                         cancel_callback=lambda _: CancelResponse.ACCEPT,
                                         callback_group=group))
        ready = fake.create_publisher(Bool, '/h1_2/policy_ready', 10)
        timer = fake.create_timer(.05, lambda: ready.publish(Bool(data=True)))
        wait(lambda: coordinator.ready.ready(time.monotonic()))
        wait(lambda: all(c.server_is_ready() for c in coordinator.session['clients'].values()))
        activation = ActivateMotionSource.Request()
        activation.token = prepared.token
        assert call(activate, activation).success
        for name, typ in (('navigate_to_pose', NavigateToPose), ('navigate_through_poses', NavigateThroughPoses)):
            client = ActionClient(fake, typ, prepared.action_namespace + '/' + name, callback_group=group)
            entities.append(client)
            wait(client.server_is_ready)
            future = client.send_goal_async(typ.Goal())
            wait(future.done)
            handle = future.result()
            assert handle.accepted
            result = handle.get_result_async()
            wait(result.done)
            assert result.result().status == GoalStatus.STATUS_SUCCEEDED
        blocking['value'] = True
        client = entities[-1]
        goal = client.send_goal_async(NavigateThroughPoses.Goal())
        wait(goal.done)
        handle = goal.result()
        assert handle.accepted
        assert started.wait(3)
        canceled = handle.cancel_goal_async()
        wait(canceled.done)
        wait(lambda: coordinator.session['backend'] is not None)
        wait(lambda: coordinator.session['backend'].status == GoalStatus.STATUS_CANCELING)
        release.set()
        result = handle.get_result_async()
        wait(result.done)
        assert result.result().status == GoalStatus.STATUS_CANCELED

        started.clear()
        release.clear()
        goal = client.send_goal_async(NavigateThroughPoses.Goal())
        wait(goal.done)
        handle = goal.result()
        assert handle.accepted
        assert started.wait(3)
        assert call(hold, Trigger.Request()).success
        release.set()
        result = handle.get_result_async()
        wait(result.done)
        assert result.result().status == GoalStatus.STATUS_ABORTED
        stale = ActivateMotionSource.Request()
        stale.token = prepared.token
        assert not call(activate, stale).success
        fake.destroy_timer(timer)
    finally:
        release.set()
        for entity in entities:
            entity.destroy()
        executor.shutdown()
        thread.join(timeout=2)
        fake.destroy_node()
        coordinator.destroy_node()
        rclpy.shutdown()


def test_duplicate_output_writer_forces_hold_and_drops_claim():
    rclpy.init()
    coordinator = PolicyMotionCoordinator(owner_boot_id='b' * 32)
    fake = Node('test_duplicate_writer')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(coordinator)
    executor.add_node(fake)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        claim = fake.create_client(ClaimPolicyInput, '/h1_2/policy/confirm_input_claim')
        request = ClaimPolicyInput.Request()
        request.owner_boot_id = 'b' * 32
        gids = coordinator.publisher_gids()
        request.cmd_writer_gid, request.heartbeat_writer_gid = list(gids[0]), list(gids[1])
        assert call(claim, request).success
        assert coordinator.claimed

        rogue = fake.create_publisher(Twist, '/h1_2/policy/cmd_vel', 10)
        wait(lambda: not coordinator.claimed and coordinator.core.state == 'hold')

        assert coordinator.publisher_gids()[0] == gids[0]
        assert not coordinator.core.accepts(coordinator.core.token)
        rogue.publish(Twist())
        assert coordinator.command.linear.x == 0.0
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        fake.destroy_node()
        coordinator.destroy_node()
        rclpy.shutdown()


def test_ready_loss_invalidates_session_without_automatic_reactivation():
    rclpy.init()
    coordinator = PolicyMotionCoordinator(owner_boot_id='c' * 32, nav_verified=True)
    fake = Node('test_ready_loss')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(coordinator)
    executor.add_node(fake)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        claim = fake.create_client(ClaimPolicyInput, '/h1_2/policy/confirm_input_claim')
        prepare = fake.create_client(PrepareMotionSource, '/h1_2/policy/prepare_source')
        activate = fake.create_client(ActivateMotionSource, '/h1_2/policy/activate_source')
        ready_pub = fake.create_publisher(Bool, '/h1_2/policy_ready', 10)
        request = ClaimPolicyInput.Request()
        request.owner_boot_id = 'c' * 32
        gids = coordinator.publisher_gids()
        request.cmd_writer_gid, request.heartbeat_writer_gid = list(gids[0]), list(gids[1])
        assert call(claim, request).success
        ready_pub.publish(Bool(data=True))
        wait(lambda: coordinator.ready.ready(time.monotonic()))

        prep = PrepareMotionSource.Request()
        prep.source, prep.expected_token = 'manual', token_msg(coordinator.core.token)
        prepared = call(prepare, prep)
        assert prepared.success
        activation = ActivateMotionSource.Request()
        activation.token = prepared.token
        assert call(activate, activation).success

        ready_pub.publish(Bool(data=False))
        wait(lambda: coordinator.core.state == 'hold')
        ready_pub.publish(Bool(data=True))
        wait(lambda: coordinator.ready.ready(time.monotonic()))
        assert coordinator.core.state == 'hold'
        stale = ActivateMotionSource.Request()
        stale.token = prepared.token
        assert not call(activate, stale).success
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        fake.destroy_node()
        coordinator.destroy_node()
        rclpy.shutdown()


def test_hold_before_backend_acceptance_cancels_late_goal():
    rclpy.init()
    coordinator = PolicyMotionCoordinator(owner_boot_id='d' * 32, nav_verified=True)
    fake = Node('test_late_backend_acceptance')
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(coordinator)
    executor.add_node(fake)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    entities = []
    acceptance_started, allow_acceptance = threading.Event(), threading.Event()
    try:
        claim = fake.create_client(ClaimPolicyInput, '/h1_2/policy/confirm_input_claim')
        prepare = fake.create_client(PrepareMotionSource, '/h1_2/policy/prepare_source')
        activate = fake.create_client(ActivateMotionSource, '/h1_2/policy/activate_source')
        hold = fake.create_client(Trigger, '/h1_2/policy/hold')
        request = ClaimPolicyInput.Request()
        request.owner_boot_id = 'd' * 32
        gids = coordinator.publisher_gids()
        request.cmd_writer_gid, request.heartbeat_writer_gid = list(gids[0]), list(gids[1])
        assert call(claim, request).success
        ready = fake.create_publisher(Bool, '/h1_2/policy_ready', 10)
        timer = fake.create_timer(.05, lambda: ready.publish(Bool(data=True)))

        prep = PrepareMotionSource.Request()
        prep.source, prep.expected_token = 'nav', token_msg(coordinator.core.token)
        prepared = call(prepare, prep)
        assert prepared.success
        backend = prepared.action_namespace.rsplit('/', 1)[0] + '/backend'

        def goal_callback(_):
            acceptance_started.set()
            assert allow_acceptance.wait(3)
            return GoalResponse.ACCEPT

        def execute(handle):
            end = time.monotonic() + 3
            while not handle.is_cancel_requested and time.monotonic() < end:
                time.sleep(.01)
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.succeed()
            return NavigateToPose.Result()

        backend_server = ActionServer(
            fake, NavigateToPose, backend + '/navigate_to_pose',
            execute_callback=execute, goal_callback=goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        entities.append(backend_server)
        through_server = ActionServer(
            fake, NavigateThroughPoses, backend + '/navigate_through_poses',
            execute_callback=lambda handle: NavigateThroughPoses.Result(),
            callback_group=ReentrantCallbackGroup())
        entities.append(through_server)
        wait(lambda: coordinator.ready.ready(time.monotonic()))
        wait(lambda: all(client.server_is_ready()
                         for client in coordinator.session['clients'].values()))
        activation = ActivateMotionSource.Request()
        activation.token = prepared.token
        assert call(activate, activation).success

        client = ActionClient(fake, NavigateToPose,
                              prepared.action_namespace + '/navigate_to_pose')
        entities.append(client)
        wait(client.server_is_ready)
        frontend_future = client.send_goal_async(NavigateToPose.Goal())
        wait(acceptance_started.is_set)
        assert call(hold, Trigger.Request()).success
        allow_acceptance.set()
        wait(frontend_future.done)
        frontend = frontend_future.result()
        assert frontend.accepted
        result = frontend.get_result_async()
        wait(result.done)
        assert result.result().status == GoalStatus.STATUS_ABORTED
        wait(lambda: coordinator.core.state == 'hold')
        timer.cancel()
    finally:
        allow_acceptance.set()
        for entity in entities:
            entity.destroy()
        executor.shutdown()
        thread.join(timeout=2)
        fake.destroy_node()
        coordinator.destroy_node()
        rclpy.shutdown()
