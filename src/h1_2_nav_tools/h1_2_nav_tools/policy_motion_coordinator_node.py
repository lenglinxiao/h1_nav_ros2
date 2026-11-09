"""Token-scoped policy input coordinator and Nav2 action proxy."""
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from h1_2_policy_interfaces.msg import PolicySourceState, SessionToken as TokenMsg
from h1_2_policy_interfaces.srv import ActivateMotionSource, ClaimPolicyInput, PrepareMotionSource
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .policy_motion_coordinator import MotionCoordinator, SessionToken
from .policy_navigation_gate import Readiness


def valid_command(msg):
    values = (msg.linear.x, msg.linear.y, msg.linear.z,
              msg.angular.x, msg.angular.y, msg.angular.z)
    return all(math.isfinite(x) for x in values) and all(
        x == 0.0 for x in (msg.linear.z, msg.angular.x, msg.angular.y))


def token_from_msg(msg):
    return SessionToken(msg.owner_boot_id, msg.coordinator_boot_id, msg.epoch)


def token_msg(token):
    msg = TokenMsg()
    msg.owner_boot_id, msg.coordinator_boot_id, msg.epoch = (
        token.owner_boot_id, token.coordinator_boot_id, token.epoch)
    return msg


class PolicyMotionCoordinator(Node):
    def __init__(self, nav_verified=False, owner_boot_id=None):
        super().__init__('policy_motion_coordinator')
        self.declare_parameter('owner_boot_id', '')
        owner = owner_boot_id if owner_boot_id is not None else self.get_parameter('owner_boot_id').value
        self.core = MotionCoordinator(owner, nav_verified)
        self.group = ReentrantCallbackGroup()
        self.ready = Readiness(0.2)
        self.claimed = False
        self.last_input = None
        self.command = Twist()
        self.session = None
        self.retired = []
        self.goal = None
        self.reason = 'STARTUP'
        self.cmd_pub = self.create_publisher(Twist, '/h1_2/policy/cmd_vel', 10)
        self.alive_pub = self.create_publisher(Bool, '/h1_2/policy_gate_alive', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(PolicySourceState, '/h1_2/policy/source_state', qos)
        self.create_subscription(Bool, '/h1_2/policy_ready', self._on_ready, 10, callback_group=self.group)
        self.create_service(PrepareMotionSource, '/h1_2/policy/prepare_source', self._prepare, callback_group=self.group)
        self.create_service(ActivateMotionSource, '/h1_2/policy/activate_source', self._activate, callback_group=self.group)
        self.create_service(Trigger, '/h1_2/policy/hold', self._hold_service, callback_group=self.group)
        self.create_service(ClaimPolicyInput, '/h1_2/policy/confirm_input_claim', self._claim, callback_group=self.group)
        self.create_timer(0.02, self._tick, callback_group=self.group)
        self._state('STARTUP')

    def publisher_gids(self):
        def gid(topic):
            endpoints = self.get_publishers_info_by_topic(topic)
            own = [e for e in endpoints if e.node_name == self.get_name() and e.node_namespace == self.get_namespace()]
            return bytes(own[0].endpoint_gid) if len(own) == 1 else None
        return gid('/h1_2/policy/cmd_vel'), gid('/h1_2/policy_gate_alive')

    def _conflict(self):
        return any(self.count_publishers(t) > 1 for t in ('/h1_2/policy/cmd_vel', '/h1_2/policy_gate_alive'))

    def _state(self, reason):
        self.reason = reason
        msg = PolicySourceState()
        msg.token = token_msg(self.core.token)
        msg.state, msg.reason = self.core.state, reason
        self.state_pub.publish(msg)

    def _zero(self):
        self.command = Twist()
        self.cmd_pub.publish(self.command)

    def _invalidate(self, reason):
        self.core.hold()
        self.last_input = None
        try:
            self._zero()
        finally:
            self._close_session()
            self._state(reason)

    def _close_session(self):
        session, self.session = self.session, None
        if session is None:
            return
        if self.goal is session:
            self.goal = None
        self.destroy_subscription(session['sub'])
        # Keep action entities while an execute callback is pending; old closures reject the token.
        session['closed'] = True
        if session.get('backend') is not None:
            self._request_cancel(session)
        if not session.get('busy'):
            self._destroy_session(session)

    def _destroy_session(self, session):
        if session.get('destroyed'):
            return
        session['destroyed'] = True
        for server in session['servers']:
            server.destroy()
        for client in session['clients'].values():
            client.destroy()

    def _claim(self, request, response):
        with self.core.lock:
            gids = self.publisher_gids()
            good = (request.owner_boot_id == self.core.token.owner_boot_id and all(gids)
                    and bytes(request.cmd_writer_gid) == gids[0]
                    and bytes(request.heartbeat_writer_gid) == gids[1] and not self._conflict())
            if not good:
                self.claimed = False
                self._invalidate('CLAIM_MISMATCH')
            else:
                self.claimed = True
            response.success, response.reason = good, '' if good else 'CLAIM_MISMATCH'
            return response

    def _on_ready(self, msg):
        with self.core.lock:
            self.ready.update(msg.data, time.monotonic())
            if not self.ready.ready(time.monotonic()) and self.core.state in ('manual', 'nav'):
                self._invalidate('READY_LOST')

    def _prepare(self, req, res):
        with self.core.lock:
            transition = self.core.begin_prepare(req.source, token_from_msg(req.expected_token))
            if not transition.success:
                res.reason = transition.reason
                return res
            try:
                self._zero()
                self._close_session()
                transition = self.core.finish_prepare(transition.operation, isolated=True, zero_barrier=True,
                                                      node_ready=False, backend_ready=False)
                if transition.success:
                    self._new_session(transition.token, transition.cmd_topic, transition.action_namespace)
                    res.token = token_msg(transition.token)
                    res.cmd_topic, res.action_namespace = transition.cmd_topic, transition.action_namespace
                res.success, res.reason = transition.success, transition.reason
                self._state('PREPARED' if transition.success else transition.reason)
            except Exception as exc:
                self.get_logger().error('Prepare failed: %s' % exc)
                self._invalidate('PREPARE_FAILED')
                res.success, res.reason = False, 'PREPARE_FAILED'
            return res

    def _activate(self, req, res):
        with self.core.lock:
            session = self.session
            backend_ready = bool(session and (session['source'] == 'manual' or
                all(client.server_is_ready() for client in session['clients'].values())))
            good = self.claimed and not self._conflict() and self.ready.ready(time.monotonic())
            transition = self.core.activate(token_from_msg(req.token), node_ready=good, backend_ready=backend_ready)
            res.success, res.reason = transition.success, transition.reason
            if transition.success:
                self.last_input = time.monotonic() if self.core.state == 'manual' else None
                self._state('ACTIVE')
            return res

    def _hold_service(self, req, res):
        with self.core.lock:
            try:
                self._invalidate('REQUESTED_HOLD')
                res.success = True
            except Exception as exc:
                self.get_logger().error('Hold failed: %s' % exc)
                res.success, res.message = False, str(exc)
            return res

    def _new_session(self, token, topic, frontend):
        session = {'token': token, 'source': self.core._source, 'servers': [], 'clients': {},
                   'busy': False, 'closed': False, 'backend': None}
        session['sub'] = self.create_subscription(Twist, topic,
            lambda msg: self._velocity(session, msg), 10, callback_group=self.group)
        backend = frontend.rsplit('/', 1)[0] + '/backend'
        for name, typ in (('navigate_to_pose', NavigateToPose), ('navigate_through_poses', NavigateThroughPoses)):
            session['clients'][name] = ActionClient(self, typ, backend + '/' + name, callback_group=self.group)
            session['servers'].append(ActionServer(self, typ, frontend + '/' + name,
                execute_callback=self._execute_callback(session, name, typ),
                goal_callback=lambda request, s=session: self._goal(s, request),
                cancel_callback=lambda handle, s=session: self._cancel(s, handle), callback_group=self.group))
        self.session = session

    def _execute_callback(self, session, name, typ):
        async def execute(handle):
            return await self._execute(session, name, typ, handle)
        return execute

    def _valid_session(self, session):
        return (self.session is session and self.core.accepts(session['token']) and self.claimed
                and not self._conflict() and self.ready.ready(time.monotonic()))

    def _velocity(self, session, msg):
        with self.core.lock:
            if not self._valid_session(session) or (self.core.state == 'nav' and not session['busy']):
                return
            if not valid_command(msg):
                self._invalidate('INVALID_COMMAND')
                return
            self.command = msg
            self.last_input = time.monotonic()

    def _goal(self, session, request):
        with self.core.lock:
            if not self._valid_session(session) or self.core.state != 'nav' or self.goal is not None:
                return GoalResponse.REJECT
            self.goal = session
            session['busy'] = True
            self.last_input = time.monotonic()
            return GoalResponse.ACCEPT

    def _cancel(self, session, handle):
        with self.core.lock:
            if session['backend'] is not None:
                self._request_cancel(session)
            return CancelResponse.ACCEPT

    def _request_cancel(self, session):
        try:
            future = session['backend'].cancel_goal_async()
            future.add_done_callback(lambda done: self._cancel_done(session, done))
        except Exception as exc:
            self._cancel_failed(session, exc)

    def _cancel_done(self, session, future):
        with self.core.lock:
            try:
                response = future.result()
                if response.return_code != 0:
                    raise RuntimeError('backend rejected cancellation')
            except Exception as exc:
                self._cancel_failed(session, exc)

    def _cancel_failed(self, session, exc):
        self.get_logger().error('Backend cancel failed: %s' % exc)
        if self.session is session:
            self._invalidate('BACKEND_CANCEL_FAILED')

    async def _execute(self, session, name, typ, handle):
        try:
            with self.core.lock:
                if not self._valid_session(session):
                    handle.abort()
                    return typ.Result()
                client = session['clients'][name]
                future = client.send_goal_async(handle.request,
                    feedback_callback=lambda feedback: self._feedback(session, handle, feedback))
            backend = await future
            with self.core.lock:
                session['backend'] = backend
                if not backend.accepted:
                    handle.abort()
                    return typ.Result()
                if not self._valid_session(session) or handle.is_cancel_requested:
                    self._request_cancel(session)
            result = await backend.get_result_async()
            with self.core.lock:
                if not self._valid_session(session):
                    handle.abort()
                    return typ.Result()
                if handle.is_cancel_requested and result.status == GoalStatus.STATUS_CANCELED:
                    handle.canceled()
                elif not handle.is_cancel_requested and result.status == GoalStatus.STATUS_SUCCEEDED:
                    handle.succeed()
                else:
                    handle.abort()
                return result.result
        except Exception as exc:
            with self.core.lock:
                self.get_logger().error('Backend action failed: %s' % exc)
                if self.session is session:
                    self._invalidate('BACKEND_ACTION_FAILED')
                handle.abort()
            return typ.Result()
        finally:
            with self.core.lock:
                session['busy'] = False
                if self.goal is session:
                    self.goal = None
                    self.last_input = None
                    self.command = Twist()
                if session['closed']:
                    # rclpy sends the frontend result after this callback returns.
                    # Keep the server alive briefly so that response can be delivered.
                    self.retired.append((time.monotonic() + 1.0, session))

    def _feedback(self, session, handle, feedback):
        with self.core.lock:
            if self._valid_session(session):
                handle.publish_feedback(feedback.feedback)

    def _tick(self):
        with self.core.lock:
            try:
                now = time.monotonic()
                retained = []
                for deadline, session in self.retired:
                    if now >= deadline:
                        self._destroy_session(session)
                    else:
                        retained.append((deadline, session))
                self.retired = retained
                conflict = self._conflict()
                if conflict and (self.core.state != 'hold' or self.claimed):
                    self.claimed = False
                    self._invalidate('DUPLICATE_PUBLISHER')
                elif self.core.state in ('manual', 'nav'):
                    now = time.monotonic()
                    if not self.ready.ready(now):
                        self._invalidate('READY_LOST')
                    elif self.last_input is not None and now - self.last_input >= 0.4:
                        self._invalidate('INPUT_TIMEOUT')
                alive = Bool()
                alive.data = self.claimed and not conflict
                if self.claimed or conflict:
                    self.alive_pub.publish(alive)
                self.cmd_pub.publish(self.command if alive.data else Twist())
            except Exception as exc:
                self.get_logger().error('Output failed: %s' % exc)
                self.core.hold()
                self.claimed = False
                self._close_session()
                self._state('OUTPUT_FAILED')


def main(args=None):
    rclpy.init(args=args)
    node = PolicyMotionCoordinator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
