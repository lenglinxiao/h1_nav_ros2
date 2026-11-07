from h1_2_nav_tools.policy_navigation_gate import Readiness


def test_ready_requires_recent_true_heartbeat():
    state = Readiness(0.2)
    assert not state.ready(1.0)
    state.update(True, 1.0)
    assert state.ready(1.19)
    assert not state.ready(1.201)
    state.update(False, 1.21)
    assert not state.ready(1.21)


def test_interrupted_goal_stays_invalid_after_recovery():
    state = Readiness(0.2)
    state.update(True, 1.0)
    generation = state.generation
    state.update(False, 1.01)
    state.update(True, 1.02)
    assert state.ready(1.02)
    assert state.generation != generation


def test_timeout_invalidates_pending_goal_even_if_heartbeat_recovers():
    state = Readiness(0.2)
    state.update(True, 1.0)
    generation = state.generation
    assert not state.ready(1.21)
    state.update(True, 1.22)
    assert state.generation != generation

import asyncio
import inspect
from concurrent.futures import Future
import threading
import time
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import GoalResponse

from h1_2_nav_tools.policy_navigation_gate import PolicyNavigationGate


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeBackend:
    accepted = True

    def __init__(self):
        self.cancels = 0
        self.result = asyncio.get_running_loop().create_future()

    def cancel_goal_async(self):
        self.cancels += 1

    def get_result_async(self):
        return self.result


class FakeClient:
    def __init__(self):
        self.sent = []
        self.pending = asyncio.get_running_loop().create_future()

    def server_is_ready(self):
        return True

    def send_goal_async(self, request, feedback_callback):
        self.sent.append(request)
        return self.pending


class FakeCleanupClient:
    def __init__(self):
        self.available = True
        self.requests = []

    def service_is_ready(self):
        return self.available

    def call_async(self, request):
        future = Future()
        self.requests.append(future)
        return future


class FakeGoal:
    def __init__(self):
        self.request = object()
        self.is_cancel_requested = False
        self.outcome = None

    def abort(self):
        self.outcome = 'aborted'

    def succeed(self):
        self.outcome = 'succeeded'

    def canceled(self):
        self.outcome = 'canceled'

    def publish_feedback(self, feedback):
        pass


def gate_with_client():
    # Exercise the actual callbacks without starting DDS or a ROS executor.
    gate = object.__new__(PolicyNavigationGate)
    gate.state = Readiness(0.2)
    gate.lock = threading.RLock()
    gate.active = {}
    gate.accepted = {}
    gate.cancel_sent = set()
    gate.cancel_requested = set()
    gate.alive_publisher = FakePublisher()
    gate.cleanup_clients = {name: FakeCleanupClient() for name in
                            ('navigate_to_pose', 'navigate_through_poses')}
    gate.cleanup_futures = {}
    gate.cleanup_pending = {}
    gate.cleanup_deadline = None
    gate.cleanup_settle_at = None
    gate.cleanup_complete = True
    gate.backend_status = {name: {} for name in gate.cleanup_clients}
    client = FakeClient()
    gate.action_clients = {'navigate_to_pose': client, 'navigate_through_poses': client}
    return gate, client


def finish(backend, status):
    backend.result.set_result(SimpleNamespace(status=status, result=object()))


def test_action_server_execute_callback_is_awaitable():
    async def scenario():
        gate, client = gate_with_client()
        goal = FakeGoal()
        gate.state.update(True, time.monotonic())
        assert gate.accept_goal('navigate_to_pose', goal.request) == GoalResponse.ACCEPT
        callback = gate.make_execute_callback('navigate_to_pose', NavigateToPose)
        assert inspect.iscoroutinefunction(callback)
        task = asyncio.create_task(callback(goal))
        await asyncio.sleep(0)
        backend = FakeBackend()
        client.pending.set_result(backend)
        await asyncio.sleep(0)
        finish(backend, GoalStatus.STATUS_SUCCEEDED)
        await task
        assert goal.outcome == 'succeeded'
    asyncio.run(scenario())


@pytest.mark.parametrize('name,action', [
    ('navigate_to_pose', NavigateToPose),
    ('navigate_through_poses', NavigateThroughPoses),
])
def test_reject_unready_then_forward_ready_success(name, action):
    async def scenario():
        gate, client = gate_with_client()
        goal = FakeGoal()
        assert gate.accept_goal(name, goal.request) == GoalResponse.REJECT
        gate.state.update(True, time.monotonic())
        assert gate.accept_goal(name, goal.request) == GoalResponse.ACCEPT
        task = asyncio.create_task(gate.execute(goal, name, action))
        await asyncio.sleep(0)
        assert client.sent == [goal.request]
        backend = FakeBackend()
        client.pending.set_result(backend)
        await asyncio.sleep(0)
        finish(backend, GoalStatus.STATUS_SUCCEEDED)
        await task
        assert goal.outcome == 'succeeded'
    asyncio.run(scenario())


def test_lost_readiness_cancels_active_once_and_aborts():
    async def scenario():
        gate, client = gate_with_client()
        goal = FakeGoal()
        gate.state.update(True, time.monotonic())
        assert gate.accept_goal('navigate_to_pose', goal.request) == GoalResponse.ACCEPT
        task = asyncio.create_task(gate.execute(goal, 'navigate_to_pose', NavigateToPose))
        await asyncio.sleep(0)
        backend = FakeBackend()
        client.pending.set_result(backend)
        await asyncio.sleep(0)
        gate.on_ready(SimpleNamespace(data=False))
        gate.check_ready()
        assert backend.cancels == 1
        finish(backend, GoalStatus.STATUS_CANCELED)
        await task
        assert goal.outcome == 'aborted'
    asyncio.run(scenario())


def test_lost_readiness_while_backend_acceptance_pending_cancels():
    async def scenario():
        gate, client = gate_with_client()
        goal = FakeGoal()
        gate.state.update(True, time.monotonic())
        assert gate.accept_goal('navigate_to_pose', goal.request) == GoalResponse.ACCEPT
        task = asyncio.create_task(gate.execute(goal, 'navigate_to_pose', NavigateToPose))
        await asyncio.sleep(0)
        gate.on_ready(SimpleNamespace(data=False))
        backend = FakeBackend()
        client.pending.set_result(backend)
        await asyncio.sleep(0)
        gate.check_ready()
        assert backend.cancels == 1
        finish(backend, GoalStatus.STATUS_CANCELED)
        await task
        assert goal.outcome == 'aborted'
    asyncio.run(scenario())


def test_interruption_between_accept_and_execute_never_sends_goal():
    async def scenario():
        gate, client = gate_with_client()
        goal = FakeGoal()
        gate.state.update(True, time.monotonic())
        assert gate.accept_goal('navigate_to_pose', goal.request) == GoalResponse.ACCEPT
        gate.on_ready(SimpleNamespace(data=False))
        gate.on_ready(SimpleNamespace(data=True))
        await gate.execute(goal, 'navigate_to_pose', NavigateToPose)
        assert not client.sent
        assert goal.outcome == 'aborted'
    asyncio.run(scenario())


def test_client_cancel_is_forwarded_once():
    async def scenario():
        gate, client = gate_with_client()
        goal = FakeGoal()
        gate.state.update(True, time.monotonic())
        assert gate.accept_goal('navigate_to_pose', goal.request) == GoalResponse.ACCEPT
        task = asyncio.create_task(gate.execute(goal, 'navigate_to_pose', NavigateToPose))
        await asyncio.sleep(0)
        backend = FakeBackend()
        client.pending.set_result(backend)
        await asyncio.sleep(0)
        gate.cancel_goal(goal)
        goal.is_cancel_requested = True
        gate.on_ready(SimpleNamespace(data=False))
        assert backend.cancels == 1
        finish(backend, GoalStatus.STATUS_CANCELED)
        await task
        assert goal.outcome == 'canceled'
    asyncio.run(scenario())


def test_client_cancel_during_backend_acceptance_is_forwarded():
    async def scenario():
        gate, client = gate_with_client()
        goal = FakeGoal()
        gate.state.update(True, time.monotonic())
        assert gate.accept_goal('navigate_to_pose', goal.request) == GoalResponse.ACCEPT
        task = asyncio.create_task(gate.execute(goal, 'navigate_to_pose', NavigateToPose))
        await asyncio.sleep(0)
        gate.cancel_goal(goal)
        backend = FakeBackend()
        client.pending.set_result(backend)
        await asyncio.sleep(0)
        assert backend.cancels == 1
        goal.is_cancel_requested = True
        finish(backend, GoalStatus.STATUS_CANCELED)
        await task
        assert goal.outcome == 'canceled'
    asyncio.run(scenario())


def test_timer_publishes_alive_even_when_policy_not_ready():
    async def scenario():
        gate, _ = gate_with_client()
        gate.check_ready()
        assert len(gate.alive_publisher.messages) == 1
        assert gate.alive_publisher.messages[0].data is True
    asyncio.run(scenario())


def test_recovery_waits_for_cancel_response_and_terminal_goal_status():
    async def scenario():
        gate, _ = gate_with_client()
        gate.cleanup_complete = False
        gate.check_ready()
        assert not gate.alive_publisher.messages[-1].data
        uid = bytes([1] * 16)
        goal = SimpleNamespace(goal_id=SimpleNamespace(uuid=uid))
        for name, client in gate.cleanup_clients.items():
            client.requests[-1].set_result(SimpleNamespace(
                return_code=0, goals_canceling=[goal] if name == 'navigate_to_pose' else []))
        gate.check_ready()
        assert not gate.alive_publisher.messages[-1].data
        gate.cleanup_settle_at = time.monotonic() - 1
        gate.on_backend_status('navigate_to_pose', SimpleNamespace(status_list=[
            SimpleNamespace(goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=uid)),
                            status=GoalStatus.STATUS_CANCELING)]))
        gate.check_ready()
        assert not gate.alive_publisher.messages[-1].data
        gate.on_backend_status('navigate_to_pose', SimpleNamespace(status_list=[
            SimpleNamespace(goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=uid)),
                            status=GoalStatus.STATUS_CANCELED)]))
        gate.check_ready()
        assert gate.alive_publisher.messages[-1].data
    asyncio.run(scenario())


def test_empty_cancel_response_does_not_ignore_known_active_goal():
    async def scenario():
        gate, _ = gate_with_client()
        gate.cleanup_complete = False
        uid = bytes([2] * 16)
        gate.backend_status['navigate_to_pose'][uid] = GoalStatus.STATUS_EXECUTING
        gate.check_ready()
        for client in gate.cleanup_clients.values():
            client.requests[-1].set_result(SimpleNamespace(return_code=0, goals_canceling=[]))
        gate.check_ready()
        gate.cleanup_settle_at = time.monotonic() - 1
        gate.check_ready()
        assert not gate.alive_publisher.messages[-1].data
        gate.backend_status['navigate_to_pose'][uid] = GoalStatus.STATUS_ABORTED
        gate.check_ready()
        assert gate.alive_publisher.messages[-1].data
    asyncio.run(scenario())

def test_backend_disconnect_revokes_alive_and_repeats_cleanup():
    async def scenario():
        gate, _ = gate_with_client()
        gate.state.update(True, time.monotonic())
        client = gate.cleanup_clients['navigate_to_pose']
        client.available = False
        gate.check_ready()
        assert not gate.alive_publisher.messages[-1].data
        assert not gate.state.ready(time.monotonic())
        client.available = True
        gate.check_ready()
        assert not gate.alive_publisher.messages[-1].data
        assert len(client.requests) == 1
    asyncio.run(scenario())
