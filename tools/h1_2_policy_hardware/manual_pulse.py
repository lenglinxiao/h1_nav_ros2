"""Bounded, single-use manual command state; timestamps are monotonic seconds."""
import math

class ManualPulse:
    def __init__(self, duration=.5, motion='forward', speed=None):
        if not math.isfinite(duration) or not .5 <= duration <= 3.0:
            raise ValueError('Pulse duration must be between 0.5 and 3.0 seconds')
        self.duration = duration
        if motion not in ('forward', 'backward', 'turn_left', 'turn_right'):
            raise ValueError('Unknown manual motion')
        default_speed = .1 if motion.startswith('turn_') else .12
        speed = default_speed if speed is None else speed
        maximum = .1 if motion.startswith('turn_') else .12
        if not math.isfinite(speed) or not 0.0 < speed <= maximum:
            raise ValueError('Manual speed must be positive and within the motion limit')
        sign = -1.0 if motion in ('backward', 'turn_right') else 1.0
        self.command = (sign * speed, 0., 0.) if motion in ('forward', 'backward') else (0., 0., sign * speed)
        self.lease_until = 0.0
        self.ready = False
        self.started = None
        self.fault = None
        self.cancel_reason = None

    def authorize(self, now, ready):
        if self.started is not None:
            if now >= self.lease_until and self.lease_until < self.started + self.duration:
                self.cancel("ready lease expired")
            elif now < self.started + self.duration and not ready:
                self.cancel("readiness lost")
        self.lease_until = now + .15
        self.ready = ready

    def start(self, now):
        if self.started is not None or self.cancel_reason or not self.ready or now >= self.lease_until:
            raise RuntimeError('Pulse requires a fresh ready lease and can only start once')
        self.started = now

    def cancel(self, reason):
        if self.cancel_reason is None:
            self.cancel_reason = reason

    def output(self, now, telemetry_ok):
        if self.started is not None and now >= self.lease_until and self.lease_until < self.started + self.duration:
            self.cancel("ready lease expired")
        if self.fault or not telemetry_ok or now >= self.lease_until:
            return None
        return self.command if (not self.cancel_reason and self.ready and self.started is not None and
                                self.started <= now < self.started + self.duration) else (0., 0., 0.)

class ManualSettle:
    """Require continuous quiet samples, bounded by an independent deadline."""
    def __init__(self, now):
        self.deadline = now + 10.0
        self.quiet_since = None
        self.complete = False

    def observe(self, now, max_dq):
        if now >= self.deadline:
            raise RuntimeError('Timed out waiting for 3 seconds of stationary telemetry')
        if max_dq > 2.0:
            raise RuntimeError('Preparation joint speed exceeded 2 rad/s')
        if max_dq >= .5:
            self.quiet_since = None
            self.complete = False
        elif self.quiet_since is None:
            self.quiet_since = now
        elif now - self.quiet_since >= 3.0:
            self.complete = True
        return self.complete
