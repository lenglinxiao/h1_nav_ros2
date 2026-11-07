"""Fail-closed action proxy for navigation while the policy is ready."""

import threading
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from std_msgs.msg import Bool


class Readiness:
    def __init__(self, timeout):
        self.timeout = timeout
        self.last_true = None
        self.generation = 0

    def ready(self, now):
        if self.last_true is not None and now - self.last_true >= self.timeout:
            self.last_true = None
            self.generation += 1
        return self.last_true is not None

    def update(self, value, now):
        self.ready(now)
        if value:
            self.last_true = now
        elif self.last_true is not None:
            self.last_true = None
            self.generation += 1


class PolicyNavigationGate(Node):
    def __init__(self):
        super().__init__('policy_navigation_gate')
        self.declare_parameter('ready_timeout', 0.2)
        timeout = float(self.get_parameter('ready_timeout').value)
        if timeout <= 0.0:
            raise ValueError('ready_timeout must be positive')
        self.state = Readiness(timeout)
        self.lock = threading.RLock()
        self.active = {}
        self.accepted = {}
        self.cancel_sent = set()
        self.cancel_requested = set()
        group = ReentrantCallbackGroup()
        self.create_subscription(Bool, '/h1_2/policy_ready', self.on_ready, 10,
                                 callback_group=group)
        self.action_clients = {}
        self.cleanup_clients = {}
        self.cleanup_futures = {}
        self.cleanup_pending = {}
        self.cleanup_deadline = None
        self.cleanup_settle_at = None
        self.cleanup_complete = False
        self.backend_status = {}
        self.servers = []
        for action_type, name in ((NavigateToPose, 'navigate_to_pose'),
                                  (NavigateThroughPoses, 'navigate_through_poses')):
            client = ActionClient(self, action_type, '/h1_2/nav2/' + name,
                                  callback_group=group)
            self.action_clients[name] = client
            self.cleanup_clients[name] = self.create_client(
                CancelGoal, '/h1_2/nav2/' + name + '/_action/cancel_goal', callback_group=group)
            self.backend_status[name] = {}
            self.create_subscription(
                GoalStatusArray, '/h1_2/nav2/' + name + '/_action/status',
                lambda msg, n=name: self.on_backend_status(n, msg),
                qos_profile_action_status_default, callback_group=group)
            self.servers.append(ActionServer(
                self, action_type, '/' + name,
                execute_callback=self.make_execute_callback(name, action_type),
                goal_callback=lambda request, n=name: self.accept_goal(n, request),
                cancel_callback=self.cancel_goal, callback_group=group))
        self.alive_publisher = self.create_publisher(Bool, '/h1_2/policy_gate_alive', 10)
        self.create_timer(0.02, self.check_ready, callback_group=group)

    def make_execute_callback(self, name, action_type):
        async def callback(handle):
            return await self.execute(handle, name, action_type)
        return callback

    def on_ready(self, msg):
        with self.lock:
            self.state.update(msg.data, time.monotonic())
            self.cancel_invalid_locked()

    def check_ready(self):
        with self.lock:
            self.update_cleanup_locked(time.monotonic())
            alive = Bool()
            alive.data = self.cleanup_complete
            self.alive_publisher.publish(alive)
            if not self.cleanup_complete:
                self.state.update(False, time.monotonic())
            self.state.ready(time.monotonic())
            self.cancel_invalid_locked()

    def on_backend_status(self, name, msg):
        with self.lock:
            self.backend_status[name] = {
                bytes(item.goal_info.goal_id.uuid): item.status for item in msg.status_list
            }

    def update_cleanup_locked(self, now):
        # Startup/reconnection must cancel any goal orphaned by an earlier proxy.
        # A cancel response only acknowledges the request; wait for terminal status.
        if not all(client.service_is_ready() for client in self.cleanup_clients.values()):
            self.cleanup_complete = False
            self.cleanup_futures.clear()
            self.cleanup_pending.clear()
            self.cleanup_deadline = None
            self.cleanup_settle_at = None
            return
        if self.cleanup_complete:
            return
        if self.cleanup_deadline is None:
            self.cleanup_futures = {name: client.call_async(CancelGoal.Request())
                                    for name, client in self.cleanup_clients.items()}
            self.cleanup_deadline = now + 1.0
            return
        if self.cleanup_futures:
            if not all(f.done() for f in self.cleanup_futures.values()):
                if now >= self.cleanup_deadline:
                    self.cleanup_futures.clear()
                    self.cleanup_deadline = None
                return
            try:
                responses = {name: future.result() for name, future in self.cleanup_futures.items()}
                if any(response.return_code != CancelGoal.Response.ERROR_NONE
                       for response in responses.values()):
                    raise RuntimeError('backend rejected cancel-all')
                self.cleanup_pending = {
                    name: {bytes(goal.goal_id.uuid) for goal in response.goals_canceling}
                    for name, response in responses.items()
                }
                self.cleanup_futures.clear()
                # Allow the transient-local action status to arrive before arming.
                self.cleanup_settle_at = now + 0.2
            except Exception as exc:
                self.get_logger().warning('Navigation cleanup failed: %s' % exc)
                self.cleanup_futures.clear()
                self.cleanup_deadline = None
                return
        terminal = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED}
        no_active_goal = all(all(status in terminal for status in statuses.values())
                             for statuses in self.backend_status.values())
        if (self.cleanup_settle_at is not None and now >= self.cleanup_settle_at and
                no_active_goal and
                all(all(self.backend_status[name].get(uuid) in terminal for uuid in uuids)
                    for name, uuids in self.cleanup_pending.items())):
            self.cleanup_complete = True

    def cancel_invalid_locked(self):
        for key, (handle, generation) in list(self.active.items()):
            if generation != self.state.generation:
                self.cancel_backend_locked(key, handle)

    def cancel_backend_locked(self, key, handle):
        if key not in self.cancel_sent:
            self.cancel_sent.add(key)
            handle.cancel_goal_async()

    def accept_goal(self, name, request):
        with self.lock:
            ready = self.state.ready(time.monotonic())
            self.cancel_invalid_locked()
            if ready and self.cleanup_complete and self.action_clients[name].server_is_ready():
                self.accepted[id(request)] = self.state.generation
                return GoalResponse.ACCEPT
            return GoalResponse.REJECT

    def cancel_goal(self, goal_handle):
        with self.lock:
            self.cancel_requested.add(id(goal_handle))
            entry = self.active.get(id(goal_handle))
            if entry is not None:
                self.cancel_backend_locked(id(goal_handle), entry[0])
        return CancelResponse.ACCEPT

    async def execute(self, goal_handle, name, action_type):
        with self.lock:
            generation = self.accepted.pop(id(goal_handle.request), None)
            if (not self.state.ready(time.monotonic()) or
                    generation is None or generation != self.state.generation):
                goal_handle.abort()
                return action_type.Result()
        try:
            client = self.action_clients[name]
            future = client.send_goal_async(
                goal_handle.request,
                feedback_callback=lambda feedback: goal_handle.publish_feedback(feedback.feedback))
            backend = await future
            if not backend.accepted:
                goal_handle.abort()
                return action_type.Result()
            with self.lock:
                self.active[id(goal_handle)] = (backend, generation)
                ready = self.state.ready(time.monotonic())
                if (not ready or generation != self.state.generation or
                        id(goal_handle) in self.cancel_requested or goal_handle.is_cancel_requested):
                    self.cancel_backend_locked(id(goal_handle), backend)
            response = await backend.get_result_async()
            with self.lock:
                self.active.pop(id(goal_handle), None)
                self.cancel_sent.discard(id(goal_handle))
                self.cancel_requested.discard(id(goal_handle))
                ready = self.state.ready(time.monotonic())
                valid = ready and generation == self.state.generation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif valid and response.status == GoalStatus.STATUS_SUCCEEDED:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return response.result
        except Exception as exc:
            with self.lock:
                self.active.pop(id(goal_handle), None)
                self.cancel_sent.discard(id(goal_handle))
                self.cancel_requested.discard(id(goal_handle))
            self.get_logger().error('Navigation proxy failed: %s' % exc)
            goal_handle.abort()
            return action_type.Result()


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNavigationGate()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
