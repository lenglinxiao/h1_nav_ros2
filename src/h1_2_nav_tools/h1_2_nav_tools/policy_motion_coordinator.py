"""ROS-independent, lock-protected policy session admission state."""
from dataclasses import dataclass
import re
import threading
import uuid


@dataclass(frozen=True)
class SessionToken:
    owner_boot_id: str
    coordinator_boot_id: str
    epoch: int


def _valid_token(token):
    return (type(token) is SessionToken and type(token.owner_boot_id) is str
            and re.fullmatch(r"[0-9a-f]{32}", token.owner_boot_id, flags=re.ASCII) is not None
            and type(token.coordinator_boot_id) is str
            and re.fullmatch(r"[0-9a-f]{32}", token.coordinator_boot_id, flags=re.ASCII) is not None
            and type(token.epoch) is int and 0 <= token.epoch < 2**64)


@dataclass(frozen=True)
class PrepareOperation:
    generation: int
    token: SessionToken


@dataclass(frozen=True)
class Transition:
    success: bool
    reason: str = ''
    token: SessionToken | None = None
    operation: PrepareOperation | None = None
    cmd_topic: str = ''
    action_namespace: str = ''


class MotionCoordinator:
    def __init__(self, owner_boot_id, nav_verified=False):
        if not isinstance(owner_boot_id, str) or not re.fullmatch(r'[0-9a-f]{32}', owner_boot_id, flags=re.ASCII):
            raise ValueError('owner_boot_id must be 32 lowercase hexadecimal characters')
        if type(nav_verified) is not bool:
            raise TypeError('nav_verified must be a bool')
        self.lock = threading.RLock()
        self._nav_verified = nav_verified
        self._token = SessionToken(owner_boot_id, uuid.uuid4().hex, 0)
        self._generation = 0
        self._state = 'hold'
        self._source = None
        self._ready = False
        self._node_ready = False
        self._backend_ready = False
        self._operation = None

    @property
    def token(self):
        with self.lock:
            return self._token

    @property
    def state(self):
        with self.lock:
            return self._state

    @property
    def generation(self):
        with self.lock:
            return self._generation

    def begin_prepare(self, source, expected_token):
        with self.lock:
            if not _valid_token(expected_token) or expected_token != self._token:
                return Transition(False, 'TOKEN_MISMATCH')
            if type(source) is not str or source not in ('manual', 'nav'):
                return Transition(False, 'INVALID_SOURCE')
            if source == 'nav' and not self._nav_verified:
                return Transition(False, 'NAV_NOT_VERIFIED')
            self._advance()
            self._state = 'switching'
            self._source = source
            self._ready = False
            self._operation = PrepareOperation(self._generation, self._token)
            return Transition(True, token=self._token, operation=self._operation)

    def finish_prepare(self, operation, *, isolated, zero_barrier, node_ready, backend_ready):
        with self.lock:
            if (type(operation) is not PrepareOperation or not _valid_token(operation.token)
                    or type(operation.generation) is not int or operation != self._operation
                    or operation.generation != self._generation or operation.token != self._token
                    or self._state != 'switching'):
                return Transition(False, 'STALE_PREPARE')
            # A completed attempt is consumed, including failure, to prevent replay.
            self._operation = None
            if not all(value is True for value in (isolated, zero_barrier)):
                self._ready = False
                return Transition(False, 'BARRIER_FAILED', token=self._token)
            self._ready = True
            self._node_ready = node_ready is True
            self._backend_ready = backend_ready is True
            t = self._token
            base = f'/h1_2/policy/session/o_{t.owner_boot_id}/c_{t.coordinator_boot_id}/e_{t.epoch}'
            return Transition(True, token=t, cmd_topic=base + '/cmd_vel', action_namespace=base + '/frontend')

    def activate(self, token, *, node_ready=None, backend_ready=None):
        with self.lock:
            if not _valid_token(token) or token != self._token:
                return Transition(False, 'TOKEN_MISMATCH')
            if self._state != 'switching' or not self._ready:
                return Transition(False, 'NOT_READY')
            if ((node_ready is not None and type(node_ready) is not bool)
                    or (backend_ready is not None and type(backend_ready) is not bool)):
                return Transition(False, 'INVALID_READINESS')
            if node_ready is not None:
                self._node_ready = node_ready
            if backend_ready is not None:
                self._backend_ready = backend_ready
            if not self._node_ready or not self._backend_ready:
                return Transition(False, 'NOT_READY')
            self._state = self._source
            self._ready = False
            return Transition(True, token=self._token)

    def hold(self):
        with self.lock:
            self._advance()
            self._state = 'hold'
            self._source = None
            self._ready = False
            self._operation = None
            return self._token

    def accepts(self, token):
        with self.lock:
            return _valid_token(token) and token == self._token and self._state in ('manual', 'nav')

    def _advance(self):
        self._generation += 1
        t = self._token
        self._token = SessionToken(t.owner_boot_id, t.coordinator_boot_id, t.epoch + 1)
