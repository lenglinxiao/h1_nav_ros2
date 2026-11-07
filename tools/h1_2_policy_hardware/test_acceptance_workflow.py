#!/usr/bin/env python3
"""Exercise the real acceptance workflow offline with isolated I/O fakes."""
import contextlib
import io
import json
import runpy
import signal
import sys
import tempfile
import types
import unittest
import repeat_input
from pathlib import Path
from unittest.mock import patch

SCRIPT = Path(__file__).with_name('run_acceptance.py')

class FakeProcess:
    def __init__(self, name, case):
        self.name, self.case, self.returncode, self.pid = name, case, None, 12345
    def poll(self):
        if self.name == 'monitor' and self.case.monitor_exited:
            return 1
        if self.name == 'node' and self.case.node_exited:
            return 1
        return self.returncode

class TtyInput(io.StringIO):
    def isatty(self): return True
    def fileno(self): return 0

class Case:
    def __init__(self, failure=None):
        self.failure = failure
        self.services = []
        self.mode = 'Passive'
        self.authorized = False
        self.writes = 0
        self.clock = 0.0
        self.monitor_exited = False
        self.node_exited = False
        self.samples = 0
        self.failed = False
        self.had_fix = False
        self.revoke_time = None
        self.stopping_since = None
        self.command_mode = 'position_pd'
        self.stop_samples = 0
        self.hold_since = None
        self.release_time = None
        self.post_release_status_count = 0
        self.manual = False
        self.last_cmd = 0.0
        self.last_vector = (0., 0., 0.)
        self.published_vectors = []
        self.published = []
        self.timer = None
        self.arm_hold_started = None
        self.trained = False
        self.tripped = False
        self.guard_hash = None
        self.guard_observed_before_revoke = False
        self.guid = 'a' * 32
        self.repeat_started = False
    def now(self):
        self.clock += .005
        return self.clock
    def sample(self):
        self.samples += 1
        if self.mode == 'Hold' and self.hold_since is None: self.hold_since = self.clock
        if self.mode == 'Stopping':
            self.stop_samples += 1
            stop_delay = 8 if self.failure == 'slow_release' else 3
            if self.failure != 'stop_stuck' and self.clock - self.stopping_since > stop_delay:
                self.mode = 'Passive'
        if self.failure == 'post_read_failure' and self.mode == 'Fault':
            raise RuntimeError('Telemetry invalid or stale')
        if self.failure == 'cleanup_read_failure' and self.had_fix and self.mode == 'Passive':
            raise RuntimeError('Telemetry invalid or stale')
        if self.failure == 'hold_hup' and self.mode == 'Hold':
            signal.raise_signal(signal.SIGHUP)
        if self.failure == 'hold_interrupt' and self.mode == 'Hold':
            raise RuntimeError('Preparation interrupted')
        if self.failure == 'interrupt' and self.mode == 'FixStand':
            raise RuntimeError('Preparation interrupted')
        if self.authorized and self.failure != 'own_stalled':
            self.writes += 2
        q = [0, -.2, 0, .5, -.3, 0, 0, -.2, 0, .5, -.3, 0, 0, .28, 0, 0, .52, 0, 0, 0, .28, 0, 0, .52, 0, 0, 0]
        if self.failure == 'position' and self.mode == 'FixStand': q[23] = 1.2
        if (self.manual or self.failure in ('onsite', 'stop_stuck', 'stop_speed', 'policy_success')) and self.mode == 'ArmHoldStand': q[3] = q[9] = .9
        elapsed = self.clock - self.arm_hold_started if self.arm_hold_started is not None else 0
        peak = 2.1 if self.failure == 'manual_speed' and self.mode == 'ArmHold' and elapsed > 3.2 else 0
        peak = 2.1 if self.failure == 'speed' and self.mode == 'FixStand' else peak
        if self.failure == 'stop_speed' and self.mode == 'Stopping': peak = 2.1
        if self.failure == 'hold_speed' and self.mode == 'Hold': peak = 2.1
        if self.failure == 'blocked_speed' and self.mode == 'Hold' and self.release_time is not None: peak = 2.1
        if self.failure == 'cleanup_speed' and self.had_fix and self.mode == 'Passive': peak = 2.1
        if self.failure == 'post_speed' and self.mode == 'Fault' and self.clock - self.revoke_time > .1: peak = 2.1
        if self.failure == 'monitor' and self.mode == 'FixStand': self.monitor_exited = True
        dq = [0]*27
        if self.mode == 'ArmHold' and (self.failure == 'manual_unsettled' or self.failure == 'manual_transient' and 3.1 < elapsed < 3.8): dq[16] = .6
        if self.trained and self.tripped and self.mode == 'Passive': self.guard_observed_before_revoke = True
        return dict(peak_dq_by_joint=[4.0 if self.trained else peak] * 27, schema=2 if self.failure == 'schema' else 3, lifetime='limited' if self.failure == 'lifetime' else 'until_stopped', states=100, commands=self.writes + 1000 + self.samples, bad_crc=0, age=0, command_age=0, policy_writer_guid=('b' * 32 if self.failure == 'wrong_guid' else self.guid if self.authorized else ''), policy_commands=self.writes, policy_command_age=(1 if self.failure == 'own_stale' else 0), policy_bad_crc=(1 if self.failure == 'own_crc' else 0), policy_finite=True, policy_native_damping=self.mode == 'Passive' and self.failure != 'own_not_damping' and not (self.failure == 'no_final_damping' and self.release_time is not None),
                    finite=True, machine=6, pr=0, native_damping=self.mode == 'Passive' and not (self.failure == 'no_final_damping' and self.release_time is not None),
                    peak_dq=peak, q=q, dq=dq)
    def call(self, name):
        self.services.append(name)
        if name == 'authorize_output':
            self.authorized = True
            (self.record / 'policy_writer_guid').write_text(self.guid)
            if self.failure == 'authorization_timeout': raise RuntimeError('Service timeout: authorize_output')
        elif name == 'fix_stand':
            self.mode = 'FixStand'
            self.had_fix = True
            if self.failure == 'node_exit': self.node_exited = True
            if self.failure == 'service_timeout': raise RuntimeError('Service timeout: fix_stand')
        elif name == 'arm_hold_stand':
            self.mode = 'ArmHoldStand'
        elif name == 'arm_hold' and self.manual and self.failure == 'arm_rejected':
            return types.SimpleNamespace(success=False, message='transition rejected')
        elif name == 'arm_hold' and self.manual:
            self.mode = 'ArmHold'
            self.arm_hold_started = self.clock
        elif name == 'arm_hold':
            log = ('policy budget exhausted; holding last output; external support required' if self.command_mode == 'position_pd' else 'policy budget exhausted; exiting control; motor behavior not verified')
            (self.record / 'node.log').write_text(log)
            self.mode = 'Hold' if self.command_mode == 'position_pd' else 'Passive'
            self.stopping_since = self.clock
        elif name == 'passive':
            if self.failure == 'passive_failure': raise RuntimeError('Passive unavailable')
            if self.command_mode == 'position_pd' and self.mode != 'Passive':
                self.mode = 'Hold'
            else:
                self.mode = 'Passive'
        elif name == 'release_supported':
            self.release_time = self.clock
            self.post_release_status_count = 0
            if self.failure in ('release_blocked', 'blocked_delay', 'blocked_speed', 'blocked_state_invalid', 'blocked_status_failure'): self.mode = 'Hold'
            else:
                self.mode = 'Stopping'
                self.stopping_since = self.clock
        elif name == 'revoke_output':
            if self.failure == 'revoke_failure': raise RuntimeError('Revoke unavailable')
            self.authorized = False
            self.mode = 'Fault'
            self.revoke_time = self.clock
            if self.failure == 'post_monitor_exit': self.monitor_exited = True
        if name == 'status':
            if self.failure == 'repeat_status_fault' and self.repeat_started and any(self.last_vector):
                raise RuntimeError('repeat status fault')
            if self.trained and not self.tripped and self.failure == 'guard_' + self.mode:
                self.tripped = True
                self.mode = 'Passive'
            if self.failure in ('manual_slow_status', 'manual_expired_status') and self.mode == 'ArmHold' and self.clock - self.arm_hold_started > 3.2 and not self.failed:
                self.clock += .16 if self.failure == 'manual_expired_status' else .081
                self.failed = True
            if self.release_time is not None and self.failure in ('blocked_state_invalid', 'blocked_status_failure'):
                self.post_release_status_count += 1
                if self.post_release_status_count > 1 and self.failure == 'blocked_status_failure':
                    raise RuntimeError('Service timeout: status')
            if self.authorized: self.writes += 2
            state = 'invalid' if self.failure == 'blocked_state_invalid' and self.post_release_status_count > 1 else 'valid'
            if self.manual and self.failure == 'manual_failure' and self.mode == 'ArmHold': raise RuntimeError('Manual injected status failure')
            ready = not (self.manual and self.mode == 'ArmHold' and (self.failure == 'ready_timeout' or self.failure == 'ready_lost' and any(self.last_vector)))
            msg = f'arm_hold_knee_tolerance=0.07 navigation_ready={str(ready).lower()} policy_cmd={self.last_vector[0]},{self.last_vector[1]},{self.last_vector[2]} mode={self.mode} state={state} authorized={str(self.authorized).lower()} publisher_present={str(self.authorized).lower()} write_count={self.writes}'
            if self.failure != 'old_node':
                cmd_mode = 'torque' if self.failure == 'wrong_mode' else self.command_mode
                clamp = 'true' if (self.failure == 'wrong_clamp' or self.command_mode == 'torque') else 'false'
                msg += f' joint_command_mode={cmd_mode} software_torque_clamp={clamp}'
                transition = 'exit' if self.mode == 'Stopping' else ('hold' if self.mode == 'Hold' else 'none')
                projection = 'false' if self.failure == 'projection_disabled' else 'true'
                protocol = 'hold_supported_release_v2' if self.failure == 'old_protocol' else 'hold_supported_release_v3'
                blocked = 'true' if self.failure in ('release_blocked', 'blocked_delay', 'blocked_speed', 'blocked_state_invalid', 'blocked_status_failure') and self.mode == 'Hold' else 'false'
                msg += f' position_pd_sample_torque_projection={projection} output_transition={transition} controlled_stop_supported=false exit_protocol={protocol} supported_release_required=true release_blocked={blocked} limited_frames=0'
            if self.trained:
                msg += f' protection_profile=train_20261106 train_guard_profile_hash={self.guard_hash} trained_guard_tripped={str(self.tripped).lower()}'
            return types.SimpleNamespace(success=True, message=msg)
        blocked_failures = ('release_blocked', 'blocked_delay', 'blocked_speed', 'blocked_state_invalid', 'blocked_status_failure')
        return types.SimpleNamespace(success=self.failure not in blocked_failures or name != 'release_supported',
                                     message='release_blocked=true' if name == 'release_supported' and self.failure in blocked_failures else 'accepted')

class WorkflowTests(unittest.TestCase):
    def run_case(self, failure=None, full=False, limit=None, policy_steps=0, command_mode='position_pd', confirm=False, brief=False, manual=False, before_start=False, trained=False, pulse_seconds=.5, motion='forward', repeat=False):
        case = Case(failure)
        case.command_mode = command_mode
        case.manual = manual
        case.trained = trained
        if trained:
            from train_guard_profile import build_profile
            urdf = SCRIPT.parents[2] / 'src/h1_2_description/h1_2_handless.urdf'
            case.guard_hash = build_profile(urdf)['profile_hash']
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = SCRIPT.read_text()
            source = source.replace("ws = Path('/home/unitree/h1_nav2_ros2_ws')", f'ws = Path({tmp!r})')
            source = source.replace("for entry in Path('/proc').iterdir():", 'for entry in ():')
            source = source.replace("monitor=spawn([str(ws/'tools/hardware-preflight/h1_prepare_monitor'),str(telemetry_path),str(record/'trace.csv'),str(writer_guid_path)],'monitor')", "monitor=spawn([], 'monitor'); telemetry_path.touch()")
            # Only replace external I/O boundaries; status, watch, main workflow and finally remain real.
            marker = '\ntry:\n    monitor=spawn('
            inject = '''\n_case.record = record\nread_sample = _case.sample\ncall = _case.call\ndef spawn(command, name): return FakeProcess(name, _case)\n'''
            if marker not in source: self.fail('workflow injection point changed')
            source = source.replace(marker, inject + marker, 1)
            script = root / 'isolated_runner.py'
            script.write_text(source)
            ros = types.ModuleType('rclpy')
            ros.init = lambda **kw: None
            ros.ok = lambda: False
            def create_node(*a, **kw):
                def publisher(msg_type, topic, depth):
                    def publish(msg):
                        if topic.endswith('/private_cmd_vel'):
                            case.last_cmd = msg.linear.x
                            case.last_vector = (msg.linear.x, msg.linear.y, msg.angular.z)
                            case.published_vectors.append((case.clock, case.last_vector))
                            case.published.append((case.clock, msg.linear.x))
                    return types.SimpleNamespace(publish=publish)
                def timer(period, callback): case.timer = callback
                return types.SimpleNamespace(destroy_node=lambda: None, create_publisher=publisher, create_timer=timer)
            ros.create_node = create_node
            ros.spin_once = lambda *a, **kw: case.timer() if case.timer else None
            srv = types.ModuleType('std_srvs.srv'); srv.Trigger = object
            mods = {'rclpy': ros, 'std_srvs': types.ModuleType('std_srvs'), 'std_srvs.srv': srv}
            argv = [str(script), '--source-commit', 'test', '--execute',
                    '--confirm-debug-mode', '--confirm-exclusive-control', '--joint-command-mode', command_mode]
            if brief: argv += ['--brief-summary']
            if manual: argv += ['--continuous-walk-trial', '--confirm-supported-upright']
            if pulse_seconds != .5: argv += ['--manual-pulse-seconds', str(pulse_seconds)]
            if motion != 'forward': argv += ['--manual-motion', motion]
            if repeat: argv += ['--allow-repeat-walk']
            if trained: argv += ['--protection-profile', 'train_20261106', '--train-urdf', str(urdf)]
            if before_start: argv += ['--onsite-confirmed-before-start']
            output = io.StringIO()
            if not full: argv += ['--fixstand-only', '--fixstand-max-error', '.2']
            elif limit is not None: argv += ['--fixstand-max-error', str(limit)]
            if policy_steps: argv += ['--policy-steps', str(policy_steps), '--confirm-supported-upright']
            class FakeRepeatInput:
                def __init__(self, fd): self.inputs = iter(['WALK', 'WALK', 'REVOKE'])
                def poll(self, accept_walk):
                    if not accept_walk and failure == 'repeat_busy_revoke' and case.repeat_started and any(case.last_vector):
                        return 'REVOKE'
                    if accept_walk:
                        answer = next(self.inputs)
                        if answer == 'WALK': case.repeat_started = True
                        return answer
                    return None
                def discard_partial(self): pass
                def close(self): pass
            with patch.object(repeat_input, 'RepeatInput', FakeRepeatInput), patch.dict(sys.modules, mods), patch.object(sys, 'argv', argv), \
                 patch('time.monotonic', side_effect=case.now), patch('time.sleep', return_value=None), \
                 patch('os.killpg', return_value=None), patch('select.select', side_effect=lambda *a: ([], [], []) if failure == 'hold_delay' and case.mode == 'Hold' and case.clock - case.hold_since < 31 or failure == 'blocked_delay' and case.mode == 'Hold' and case.release_time is not None and case.clock - case.release_time < 5 else ([sys.stdin], [], [])), \
                 patch.object(sys, 'stdin', TtyInput(('REVOKE\n' if before_start else 'CONFIRM\nREVOKE\n' if manual else 'CONFIRM\n' if confirm else '') + ('INVALID\nRELEASE\n' if failure == 'invalid_release' else 'RELEASE\nREVOKE\n' if failure in ('release_blocked', 'blocked_delay') else 'RELEASE\n' if failure == 'blocked_speed' else '' if failure in ('hold_eof', 'hold_speed', 'hold_interrupt') else 'RELEASE\n'))), contextlib.redirect_stdout(output):
                # Owned fake processes stop without touching the operating system.
                FakeProcess.wait = lambda self, timeout=None: setattr(self, 'returncode', 0)
                saved_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT)}
                try:
                    with self.assertRaises(SystemExit) as raised:
                        runpy.run_path(str(script), init_globals={'_case': case, 'FakeProcess': FakeProcess}, run_name='__main__')
                finally:
                    for sig, handler in saved_handlers.items():
                        signal.signal(sig, handler)
            summaries = list(root.glob('third_party/h1_policy_runtime/integration-record/*/summary.json'))
            self.assertEqual(len(summaries), 1)
            result = json.loads(summaries[0].read_text())
            result['_exit_code'] = raised.exception.code
            case.output = output.getvalue()
            return case, result

    def test_background_stream_does_not_block_own_progress(self):
        case, result = self.run_case()
        self.assertEqual(result['result'], 'fixstand_only_completed')
        self.assertGreater(result['initial']['commands'], 0)

    def test_background_stream_cannot_replace_own_evidence(self):
        for failure in ('wrong_guid', 'own_stale', 'own_stalled', 'own_crc', 'own_not_damping'):
            with self.subTest(failure=failure):
                case, result = self.run_case(failure)
                self.assertNotEqual(result['result'], 'fixstand_only_completed')
                self.assertNotIn('fix_stand', case.services)

    def test_trained_guard_handoff_monitors_damping_before_revoke(self):
        for stage in ('FixStand', 'ArmHoldStand', 'ArmHold'):
            with self.subTest(stage=stage):
                case, result = self.run_case('guard_' + stage, full=True, manual=True, before_start=True, trained=True)
                self.assertTrue(case.guard_observed_before_revoke)
                self.assertEqual(result['trained_guard_handoff'], 'operator_confirmed_REVOKE')
                self.assertTrue(result['revocation_confirmed'])
                self.assertNotIn('release_supported', case.services)
                self.assertIsNone(result['first_observed_over_limit'])

    def test_trained_monitor_fault_still_revokes(self):
        case, result = self.run_case('monitor', full=True, manual=True, before_start=True, trained=True)
        self.assertIn('revoke_output', case.services)
        self.assertEqual(result['result'], 'needs_operator_check')

    def test_trained_cumulative_peak_does_not_trigger_legacy_gate(self):
        case, result = self.run_case(None, full=True, manual=True, before_start=True, trained=True)
        self.assertEqual(result['result'], 'manual_trial_completed_physical_behavior_unconfirmed')
        self.assertIsNone(result['first_observed_over_limit'])
        self.assertNotIn('release_supported', case.services)

    def test_run_case_restores_signal_handlers(self):
        signals = (signal.SIGTERM, signal.SIGHUP, signal.SIGINT)
        before = {sig: signal.getsignal(sig) for sig in signals}
        self.run_case(None, full=True, manual=True)
        self.assertEqual({sig: signal.getsignal(sig) for sig in signals}, before)

    def test_preconfirmed_site_skips_prompt(self):
        case, result = self.run_case(full=True, manual=True, before_start=True)
        self.assertEqual(result['onsite'], 'operator_confirmed_before_start')
        self.assertNotIn('Type CONFIRM to continue', case.output)
        self.assertIn('revoke_output', case.services)

    def test_arm_rejection_holds_preparation_until_handoff(self):
        case, result = self.run_case('arm_rejected', full=True, manual=True, before_start=True)
        self.assertEqual(result['result'], 'manual_trial_canceled_no_policy')
        self.assertEqual(result['manual_pulse_state'], 'nonzero_not_published')
        self.assertNotIn(.12, [v for _, v in case.published])
        self.assertNotIn('release_supported', case.services)
        self.assertNotIn('passive', case.services)
        self.assertIn('revoke_output', case.services)

    def test_ready_timeout_waits_for_handoff_without_pulse(self):
        case, result = self.run_case('ready_timeout', full=True, manual=True, before_start=True)
        self.assertEqual(result['result'], 'manual_trial_canceled_no_pulse')
        self.assertNotIn(.12, [v for _, v in case.published])
        self.assertNotIn('release_supported', case.services)
        self.assertIn('revoke_output', case.services)

    def test_readiness_loss_cancels_without_replay(self):
        case, result = self.run_case('ready_lost', full=True, manual=True, before_start=True)
        self.assertEqual(result['manual_cancel_reason'], 'readiness lost')
        speeds = [v for _, v in case.published]
        self.assertIn(.12, speeds)
        self.assertEqual(speeds[-1], 0.)
        self.assertNotIn('release_supported', case.services)
        self.assertIn('revoke_output', case.services)

    def test_manual_initial_status_delay_preserves_observation(self):
        case, result = self.run_case('manual_slow_status', full=True, manual=True)
        self.assertTrue(case.failed)
        self.assertEqual(result['result'], 'manual_trial_completed_physical_behavior_unconfirmed')

    def test_long_manual_delayed_status_preserves_lease(self):
        case, result = self.run_case('manual_slow_status', full=True, manual=True, pulse_seconds=1.5)
        self.assertTrue(case.failed)
        self.assertEqual(result['result'], 'manual_trial_completed_physical_behavior_unconfirmed')

    def test_long_manual_expired_status_still_cancels(self):
        case, result = self.run_case('manual_expired_status', full=True, manual=True, pulse_seconds=1.5)
        self.assertTrue(case.failed)
        self.assertEqual(result['manual_cancel_reason'], 'ready lease expired')

    def test_manual_trial_does_not_require_extra_quiet_window(self):
        for failure in ('manual_transient', 'manual_unsettled'):
            case, result = self.run_case(failure, full=True, manual=True)
            self.assertEqual(result['result'], 'manual_trial_completed_physical_behavior_unconfirmed')
            first_pulse = next(t for t, v in case.published if v == .12)
            self.assertGreaterEqual(first_pulse - case.arm_hold_started, 3.1)
            self.assertLess(first_pulse - case.arm_hold_started, 6.8)
            self.assertIn('revoke_output', case.services)
            self.assertNotIn('release_supported', case.services)

    def test_manual_settle_speed_abort(self):
        case, result = self.run_case('manual_speed', full=True, manual=True)
        self.assertIn('speed', result['error'].lower())
        self.assertNotIn(.12, [v for _, v in case.published])
        self.assertIn('revoke_output', case.services)
        self.assertNotIn('release_supported', case.services)

    def test_manual_trial_returns_to_zero_and_revoke_without_release(self):
        case, result = self.run_case(full=True, manual=True)
        self.assertEqual(result['result'], 'manual_trial_completed_physical_behavior_unconfirmed')
        self.assertTrue(result['manual_policy_nonzero_observed'])
        self.assertTrue(result['manual_policy_zero_in_armhold_observed'])
        self.assertTrue(result['revocation_confirmed'])
        self.assertIn(.12, [v for _, v in case.published])
        self.assertEqual(case.published[-1][1], 0.)
        self.assertNotIn('passive', case.services)
        self.assertNotIn('release_supported', case.services)
        self.assertIn('Pulse window elapsed; zero command continues', case.output)
        self.assertNotIn('Manual velocity canceled: None', case.output)

    def test_long_manual_trial_observes_nonzero_then_zero(self):
        case, result = self.run_case(full=True, manual=True, pulse_seconds=1.5)
        self.assertEqual(result['manual_pulse_seconds'], 1.5)
        self.assertTrue(result['manual_policy_nonzero_observed'])
        self.assertTrue(result['manual_policy_zero_in_armhold_observed'])
        nonzero = [t for t, value in case.published if value == .12]
        self.assertGreater(nonzero[-1] - nonzero[0], 1.3)
        self.assertEqual(case.published[-1][1], 0.)

    def test_three_second_trial_observes_nonzero_then_zero(self):
        case, result = self.run_case(full=True, manual=True, pulse_seconds=3.)
        self.assertEqual(result['manual_pulse_seconds'], 3.)
        self.assertTrue(result['manual_policy_nonzero_observed'])
        self.assertTrue(result['manual_policy_zero_in_armhold_observed'])
        nonzero = [t for t, value in case.published if value == .12]
        self.assertGreater(nonzero[-1] - nonzero[0], 2.8)
        self.assertEqual(case.published[-1][1], 0.)

    def test_left_turn_and_ready_loss(self):
        for failure in (None, 'ready_lost'):
            case, result = self.run_case(failure, full=True, manual=True, pulse_seconds=3., motion='turn_left')
            vectors = [value for _, value in case.published_vectors]
            self.assertIn((0., 0., .1), vectors)
            self.assertEqual(vectors[-1], (0., 0., 0.))
            self.assertEqual(result['manual_motion'], 'turn_left')
            if failure:
                self.assertEqual(result['manual_cancel_reason'], 'readiness lost')
                last = len(vectors) - 1 - vectors[::-1].index((0., 0., .1))
                self.assertTrue(all(not any(v) for v in vectors[last + 1:]))
            else:
                self.assertTrue(result['manual_policy_nonzero_observed'])
                self.assertTrue(result['manual_policy_zero_in_armhold_observed'])

    def test_repeat_walks_in_same_armhold(self):
        case, result = self.run_case(full=True, manual=True, repeat=True)
        self.assertEqual(len(result['repeat_walks']), 2)
        for item in result['repeat_walks']:
            self.assertEqual(item['command'], [.12, 0., 0.])
            self.assertTrue(item['nonzero_observed'] and item['zero_observed'])
            self.assertIsNone(item['cancel_reason'])
        self.assertEqual(case.services.count('arm_hold'), 1)

    def test_repeat_busy_revoke_and_status_fault(self):
        case, result = self.run_case('repeat_busy_revoke', full=True, manual=True, repeat=True)
        self.assertTrue(result['revocation_confirmed'])
        self.assertEqual(len(result['repeat_walks']), 1)
        self.assertEqual(result['repeat_walks'][0]['cancel_reason'], 'operator requested REVOKE')
        case, result = self.run_case('repeat_status_fault', full=True, manual=True, repeat=True)
        item = result['repeat_walks'][0]
        self.assertIn('repeat status fault', item['error'])
        self.assertIn('end', item)
        self.assertTrue(result['manual_policy_nonzero_observed'])

    def test_manual_failure_revokes_without_release(self):
        case, result = self.run_case('manual_failure', full=True, manual=True)
        self.assertEqual(result['result'], 'needs_operator_check')
        self.assertIn('revoke_output', case.services)
        self.assertNotIn('release_supported', case.services)
        self.assertNotIn('passive', case.services)

    def test_brief_summary_keeps_full_record(self):
        case, result = self.run_case(brief=True)
        printed = json.loads(case.output[case.output.rfind('\n{') + 1:])
        self.assertEqual(printed['result'], result['result'])
        self.assertIn('events', result)
        self.assertNotIn('events', printed)
        self.assertIn('record', printed)
        self.assertIn('Body sling is not arm support', case.output)

    def test_success_and_cleanup(self):
        c, result = self.run_case()
        self.assertEqual(result['result'], 'fixstand_only_completed')
        self.assertEqual(result['_exit_code'], 0)
        self.assertFalse(result['observation_incomplete'])
        self.assertEqual(result['joint_command_mode'], 'position_pd')
        self.assertIn('fix_stand', c.services); self.assertIn('passive', c.services); self.assertIn('revoke_output', c.services)
        self.assertFalse({'arm_hold_stand', 'arm_hold'} & set(c.services))
        self.assertTrue(result['revocation_confirmed'])
        window = result['post_revoke_observation_window']
        self.assertGreaterEqual(window['end_monotonic'] - window['start_monotonic'], .5)

    def test_old_or_mismatched_node_rejected_before_authorization(self):
        for failure in ('old_node', 'old_protocol', 'projection_disabled', 'wrong_mode', 'wrong_clamp'):
            with self.subTest(failure=failure):
                case, result = self.run_case(failure)
                self.assertEqual(result['result'], 'aborted')
                self.assertNotIn('authorize_output', case.services)

    def test_full_preparation_without_position_gate(self):
        case, result = self.run_case('position', full=True)
        self.assertEqual(result['result'], 'preparation_executed_no_policy')
        self.assertFalse(result['position_gate_applied'])
        self.assertIsNone(result['fixstand_max_error_limit'])
        self.assertGreater(result['fixstand_max_error'], .2)
        self.assertIn('arm_hold_stand', case.services)
        self.assertNotIn('arm_hold', case.services)
        self.assertTrue(result['revocation_confirmed'])

    def test_explicit_full_preparation_gate_rejects_excess_error(self):
        case, result = self.run_case('position', full=True, limit=.2)
        self.assertTrue(result['position_gate_applied'])
        self.assertEqual(result['result'], 'aborted')
        self.assertNotIn('arm_hold_stand', case.services)
        self.assertTrue(result['revocation_confirmed'])

    def test_no_position_gate_preserves_policy_readiness_and_confirmation(self):
        case, result = self.run_case(full=True, policy_steps=1)
        self.assertIn('Knee position gate failed', result['error'])
        self.assertNotIn('arm_hold', case.services)
        case, result = self.run_case('onsite', full=True, policy_steps=1)
        self.assertIn('confirmation withheld', result['error'])
        self.assertNotIn('arm_hold', case.services)

    def test_faults_revoke(self):
        for failure in ('position', 'speed', 'authorization_timeout', 'passive_failure', 'monitor', 'service_timeout'):
            with self.subTest(failure=failure):
                c, result = self.run_case(failure)
                self.assertNotEqual(result['result'], 'fixstand_only_completed')
                self.assertIn('revoke_output', c.services)
                self.assertFalse({'arm_hold_stand', 'arm_hold'} & set(c.services))
                self.assertTrue('error' in result or 'exit_error' in result)

    def test_old_schema_rejected_before_authorization(self):
        for failure in ('schema', 'lifetime'):
            with self.subTest(failure=failure):
                c, result = self.run_case(failure)
                self.assertNotIn('authorize_output', c.services)
                self.assertEqual(result['result'], 'aborted')

    def test_cleanup_and_post_revoke_anomalies_do_not_prevent_revocation(self):
        for failure, phase in (('cleanup_speed', 'cleanup_exit'), ('post_speed', 'revoke_observation_unverified'),
                               ('post_read_failure', None), ('post_monitor_exit', None),
                               ('cleanup_read_failure', None)):
            with self.subTest(failure=failure):
                case, result = self.run_case(failure)
                self.assertIn('revoke_output', case.services)
                self.assertEqual(result['preparation_result'], 'fixstand_only_completed')
                self.assertEqual(result['result'], 'needs_operator_check')
                self.assertNotEqual(result['_exit_code'], 0)
                if phase:
                    self.assertEqual(result['first_observed_over_limit']['phase'], phase)
                else:
                    self.assertTrue(result['observation_incomplete'])

    def test_controlled_stop_completion_and_emergency_revoke(self):
        case, result = self.run_case('policy_success', full=True, policy_steps=1, confirm=True)
        self.assertEqual(result['result'], 'policy_budget_completed_unconfirmed_physical_behavior')
        self.assertGreater(case.stop_samples, 1)
        self.assertTrue(result['revocation_confirmed'])
        for failure in ('stop_speed', 'stop_stuck'):
            with self.subTest(failure=failure):
                case, result = self.run_case(failure, full=True, policy_steps=1, confirm=True)
                self.assertEqual(result['result'], 'needs_operator_check')
                self.assertIn('revoke_output', case.services)
                self.assertNotIn('passive', case.services[case.services.index('arm_hold') + 1:])

    def test_cleanup_stop_timeout_still_revokes(self):
        case, result = self.run_case('stop_stuck')
        self.assertEqual(result['preparation_result'], 'fixstand_only_completed')
        self.assertEqual(result['result'], 'needs_operator_check')
        self.assertIn('Timed out waiting for Passive', result['exit_error'])
        self.assertIn('revoke_output', case.services)

    def test_v3_release_waits_beyond_legacy_timeout(self):
        case, result = self.run_case('slow_release')
        self.assertEqual(result['result'], 'fixstand_only_completed')
        self.assertGreater(case.clock - case.release_time, 4.5)
        self.assertTrue(result['revocation_confirmed'])

    def test_normal_release_records_failed_final_damping_and_revokes(self):
        case, result = self.run_case('no_final_damping')
        self.assertEqual(result['result'], 'needs_operator_check')
        self.assertFalse(result['final_damping']['policy_native_damping'])
        self.assertIn('Native damping not observed', result['exit_error'])
        self.assertIn('revoke_output', case.services)

    def test_blocked_release_waits_for_explicit_operator_revoke(self):
        for failure in ('release_blocked', 'blocked_delay'):
            with self.subTest(failure=failure):
                case, result = self.run_case(failure)
                self.assertEqual(result['result'], 'needs_operator_check')
                self.assertEqual(result['release_blocked_operator_confirmation'], 'REVOKE')
                self.assertEqual(case.services.count('release_supported'), 1)
                self.assertEqual(case.services.count('revoke_output'), 1)
                self.assertTrue(result['revocation_confirmed'])
                if failure == 'blocked_delay': self.assertGreaterEqual(case.clock - case.release_time, 5)

    def test_blocked_release_speed_fault_still_revokes_immediately(self):
        case, result = self.run_case('blocked_speed')
        self.assertEqual(result['result'], 'needs_operator_check')
        self.assertIn('speed exceeded limit', result['exit_error'])
        self.assertNotIn('release_blocked_operator_confirmation', result)
        self.assertIn('revoke_output', case.services)

    def test_blocked_wait_status_and_service_failures_still_revoke(self):
        for failure, expected in (('blocked_state_invalid', 'Controller state invalid'),
                                  ('blocked_status_failure', 'Service timeout: status')):
            with self.subTest(failure=failure):
                case, result = self.run_case(failure)
                self.assertEqual(result['result'], 'needs_operator_check')
                self.assertIn(expected, result['exit_error'])
                self.assertNotIn('release_blocked_operator_confirmation', result)
                self.assertIn('revoke_output', case.services)

    def test_hold_requires_release_and_waits_without_auto_release(self):
        for failure in ('hold_delay', 'invalid_release'):
            with self.subTest(failure=failure):
                case, result = self.run_case(failure)
                self.assertEqual(result['result'], 'fixstand_only_completed')
                self.assertLess(case.services.index('passive'), case.services.index('release_supported'))
                self.assertLess(case.services.index('release_supported'), case.services.index('revoke_output'))
                if failure == 'hold_delay': self.assertGreaterEqual(case.release_time - case.hold_since, 31)
                self.assertIn('supported_release', result)

    def test_hold_failure_revokes_without_release(self):
        for failure in ('hold_eof', 'hold_speed', 'hold_interrupt', 'hold_hup'):
            with self.subTest(failure=failure):
                case, result = self.run_case(failure)
                self.assertEqual(result['result'], 'needs_operator_check')
                self.assertNotIn('release_supported', case.services)
                self.assertIn('revoke_output', case.services)

    def test_torque_workflow_remains_compatible(self):
        case, result = self.run_case('policy_success', full=True, policy_steps=1, confirm=True, command_mode='torque')
        self.assertEqual(result['result'], 'policy_budget_completed_unconfirmed_physical_behavior')
        self.assertTrue(result['revocation_confirmed'])

    def test_revoke_failure_keeps_observation_unverified(self):
        case, result = self.run_case('revoke_failure')
        self.assertIn('revoke_output', case.services)
        self.assertEqual(result['result'], 'needs_operator_check')
        self.assertNotEqual(result['_exit_code'], 0)
        self.assertNotIn('revocation_confirmed', result)
        self.assertEqual(result['last_observation']['phase'], 'cleanup_unverified')
        self.assertIsNotNone(result['node_returncode'])
        self.assertIsNotNone(result['monitor_returncode'])

    def test_interrupt_and_exited_node_do_not_claim_success(self):
        case, result = self.run_case('interrupt')
        self.assertIn('revoke_output', case.services)
        self.assertNotEqual(result['_exit_code'], 0)
        case, result = self.run_case('node_exit')
        self.assertEqual(result['result'], 'needs_operator_check')
        self.assertEqual(result['cleanup_status'], 'unknown_process_exited_or_unavailable')
        self.assertNotEqual(result['_exit_code'], 0)

if __name__ == '__main__': unittest.main()
