#!/usr/bin/env python3
"""Offline checks; never imports ROS or starts the monitor."""
import ast
import math
import os
import time
import re
import signal
import tempfile
from pathlib import Path
import subprocess
import sys
import unittest
from types import SimpleNamespace

SCRIPT = Path(__file__).with_name('run_acceptance.py')
SOURCE = SCRIPT.read_text()
TREE = ast.parse(SOURCE)

class AcceptanceTests(unittest.TestCase):
    def run_cli(self, *args):
        return subprocess.run([sys.executable, str(SCRIPT), '--source-commit', 'test', *args],
                              capture_output=True, text=True)

    def test_preconfirmed_flag_rejects_unexecuted_or_missing_requirements(self):
        for flags in ((), ('--continuous-walk-trial', '--joint-command-mode', 'position_pd', '--confirm-supported-upright'),
                      ('--execute', '--confirm-debug-mode', '--confirm-exclusive-control')):
            result = self.run_cli('--onsite-confirmed-before-start', *flags)
            self.assertEqual(result.returncode, 2)
            self.assertIn('--onsite-confirmed-before-start', result.stderr)

    def test_extended_policy_budget_requires_bounded_position_pd(self):
        supported = ('--joint-command-mode', 'position_pd', '--confirm-supported-upright')
        result = self.run_cli(*supported, '--policy-steps', '155', '--max-policy-seconds', '3.1')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('max_policy_seconds:=3.1', result.stdout)
        for args in (
            (*supported, '--policy-steps', '155'),
            (*supported, '--policy-steps', '155', '--max-policy-seconds', '3.11'),
            (*supported, '--policy-steps', '155', '--max-policy-seconds', 'nan'),
            ('--confirm-supported-upright', '--policy-steps', '155', '--max-policy-seconds', '3.1'),
            (*supported, '--policy-steps', '156', '--max-policy-seconds', '3.1'),
        ):
            self.assertEqual(self.run_cli(*args).returncode, 2, str(args))

    def test_time_bound_requires_node_to_report_matching_value(self):
        node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'status')
        base = ('state=valid mode=Passive joint_command_mode=position_pd '
                'software_torque_clamp=false position_pd_sample_torque_projection=true '
                'output_transition=none controlled_stop_supported=false '
                'exit_protocol=hold_supported_release_v3 supported_release_required=true release_blocked=false arm_hold_knee_tolerance=0.07')
        args = SimpleNamespace(joint_command_mode='position_pd', max_policy_seconds=3.1,
                               fixstand_only=False, policy_steps=155, arm_hold_knee_tolerance=.07)
        scope = {'args': args, 'math': math, 'train_profile': None}
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(SCRIPT), 'exec'), scope)
        for reported in ('', ' max_policy_seconds=0', ' max_policy_seconds=nan'):
            scope['call'] = lambda name, message=base + reported: SimpleNamespace(success=True, message=message)
            with self.assertRaisesRegex(RuntimeError, 'time bound'): scope['status']()
        scope['call'] = lambda name: SimpleNamespace(success=True, message=base + ' max_policy_seconds=3.1')
        self.assertEqual(scope['status']()['max_policy_seconds'], '3.1')

    def test_knee_tolerance_is_bounded_and_trial_only(self):
        trial = ('--continuous-walk-trial', '--joint-command-mode', 'position_pd', '--confirm-supported-upright')
        ok = self.run_cli(*trial, '--arm-hold-knee-tolerance', '0.10')
        self.assertEqual(ok.returncode, 0, ok.stderr)
        self.assertIn('arm_hold_knee_tolerance:=0.1', ok.stdout)
        for args in (('--arm-hold-knee-tolerance', '0.1'),
                     (*trial, '--arm-hold-knee-tolerance', '0.101'),
                     (*trial, '--arm-hold-knee-tolerance', 'nan')):
            self.assertEqual(self.run_cli(*args).returncode, 2)

    def test_requires_explicit_finite_positive_limit_before_ros_import(self):
        for args in [('--execute', '--fixstand-only'), ('--fixstand-max-error', 'nan'),
                     ('--fixstand-max-error', '0'), ('--fixstand-max-error', 'inf')]:
            result = self.run_cli(*args, '--confirm-debug-mode', '--confirm-exclusive-control')
            self.assertEqual(result.returncode, 2)
            self.assertIn('--fixstand-max-error', result.stderr)

    def test_execute_requires_both_human_assertions_before_ros_import(self):
        for flags in ((), ('--confirm-debug-mode',), ('--confirm-exclusive-control',)):
            result = self.run_cli('--execute', '--fixstand-max-error', '.1', *flags)
            self.assertEqual(result.returncode, 2)
            self.assertIn('requires --confirm-debug-mode and --confirm-exclusive-control', result.stderr)
            self.assertNotIn('rclpy', result.stderr)

    def test_fixstand_only_rejects_policy_budget(self):
        self.assertEqual(self.run_cli('--fixstand-only', '--policy-steps', '1',
                                      '--confirm-supported-upright').returncode, 2)
        result = self.run_cli('--fixstand-only', '--fixstand-max-error', '.1')
        self.assertEqual(result.returncode, 0)
        self.assertNotIn('arm_hold_stand (3.1 s)', result.stdout)
        self.assertIn('operator_confirmed_debug_mode:=false', result.stdout)
        self.assertIn('operator_confirmed_exclusive_control:=false', result.stdout)
        self.assertIn('neither is machine verified', result.stdout)
        self.assertIn('observe telemetry for 0.5 s', result.stdout)
        self.assertIn('incomplete observation makes the result non-success', result.stdout)

    def test_position_pd_dry_run_and_invalid_mode(self):
        result = self.run_cli('--fixstand-only', '--fixstand-max-error', '.1', '--joint-command-mode', 'position_pd')
        self.assertEqual(result.returncode, 0)
        self.assertIn('joint_command_mode:=position_pd', result.stdout)
        self.assertIn('not applied to native PD', result.stdout)
        self.assertEqual(self.run_cli('--joint-command-mode', 'invalid').returncode, 2)

    def test_blocked_release_prompt_does_not_imply_position_support(self):
        prompt = next(line for line in SOURCE.splitlines() if "Release is blocked;" in line)
        self.assertIn('Release is blocked;', prompt)
        self.assertIn('Hold preserves the last output; kp may already be zero, so position support is not guaranteed.', prompt)

    def test_position_pd_execute_requires_tty_before_ros(self):
        result = self.run_cli('--execute', '--joint-command-mode', 'position_pd',
                              '--confirm-debug-mode', '--confirm-exclusive-control')
        self.assertEqual(result.returncode, 2)
        self.assertIn('requires an interactive terminal', result.stderr)
        self.assertNotIn('rclpy', result.stderr)

    def test_full_preparation_dry_run_without_position_gate(self):
        result = self.run_cli('--joint-command-mode', 'position_pd')
        self.assertEqual(result.returncode, 0)
        self.assertIn('not applied; error will be recorded only', result.stdout)
        self.assertIn('arm_hold_stand (3.1 s)', result.stdout)

    def test_fixstand_only_blocks_later_services_before_client_creation(self):
        node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'call')
        scope = {'args': SimpleNamespace(fixstand_only=True)}
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(SCRIPT), 'exec'), scope)
        for service in ('arm_hold_stand', 'arm_hold'):
            with self.assertRaisesRegex(RuntimeError, 'forbidden'): scope['call'](service)

    def test_watch_checks_last_sample_for_speed(self):
        node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'watch')
        readings = iter(({'policy_commands': 0, 'peak_dq': 0}, {'policy_commands': 3, 'peak_dq': 2.1}))
        times = iter((0.0, 0.0, 2.0))
        scope = {'sample': lambda: next(readings), 'check_policy_source': lambda value: value, 'status': lambda mode: {'write_count': '0'},
                 'time': SimpleNamespace(monotonic=lambda: next(times)),
                 'check_speed': lambda value: (_ for _ in ()).throw(RuntimeError('speed exceeded')) if value['peak_dq'] > 2 else None}
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(SCRIPT), 'exec'), scope)
        with self.assertRaisesRegex(RuntimeError, 'speed exceeded'):
            scope['watch'](1, 'FixStand')

    def test_monitor_signal_handler_ends_lifetime_loop(self):
        monitor = Path(__file__).with_name('prepare_monitor.cpp').read_text()
        declarations = re.search(r'volatile std::sig_atomic_t stop_requested = 0;\nvoid request_stop\(int\) \{ stop_requested = 1; \}', monitor)
        self.assertIsNotNone(declarations)
        self.assertIn('while (!stop_requested)', monitor)
        self.assertIn('std::signal(SIGTERM, request_stop)', monitor)
        self.assertIn('states.CloseChannel();', monitor)
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / 'lifecycle.cpp'
            binary = Path(tmp) / 'lifecycle'
            src.write_text('#include <csignal>\n#include <cstdio>\n#include <unistd.h>\n' +
                           declarations.group() + '\nint main() { std::signal(SIGTERM, request_stop); '
                           'std::puts("ready"); std::fflush(stdout); while (!stop_requested) usleep(1000); return 0; }\n')
            subprocess.run(['g++', '-std=c++17', str(src), '-o', str(binary)], check=True)
            proc = subprocess.Popen([str(binary)], stdout=subprocess.PIPE, text=True)
            try:
                self.assertEqual(proc.stdout.readline().strip(), 'ready')
                proc.send_signal(signal.SIGTERM)
                self.assertEqual(proc.wait(timeout=3), 0)
            finally:
                if proc.poll() is None: proc.kill(); proc.wait()
                proc.stdout.close()

    def test_parent_death_guard_stops_child(self):
        helper_dir = str(SCRIPT.parent)
        parent_code = """import os, subprocess, sys, time
sys.path.insert(0, sys.argv[1])
from process_guard import parent_death_preexec
child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                         start_new_session=True, preexec_fn=parent_death_preexec())
print(child.pid, flush=True)
time.sleep(60)
"""
        parent = subprocess.Popen([sys.executable, '-c', parent_code, helper_dir],
                                  stdout=subprocess.PIPE, text=True)
        child_pid = None
        try:
            child_pid = int(parent.stdout.readline())
            os.kill(parent.pid, signal.SIGKILL)
            parent.wait(timeout=3)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                try:
                    state = Path(f'/proc/{child_pid}/stat').read_text().split(') ', 1)[1][0]
                except FileNotFoundError:
                    break
                if state == 'Z': break
                time.sleep(.01)
            else:
                self.fail('child still running after parent death')
        finally:
            if parent.poll() is None: parent.kill(); parent.wait()
            parent.stdout.close()
            if child_pid is not None:
                try: os.kill(child_pid, signal.SIGKILL)
                except ProcessLookupError: pass

    def test_parent_death_guard_failure_does_not_exec(self):
        code = """import subprocess, sys
sys.path.insert(0, sys.argv[1])
import process_guard
process_guard._prctl = lambda *args: -1
result = subprocess.run([sys.executable, '-c', 'open(sys.argv[1], \"w\").close()', sys.argv[2]],
                        start_new_session=True, preexec_fn=process_guard.parent_death_preexec())
raise SystemExit(0 if result.returncode == 127 else 1)
"""
        with tempfile.TemporaryDirectory() as tmp:
            marker = Path(tmp) / 'executed'
            result = subprocess.run([sys.executable, '-c', code, str(SCRIPT.parent), str(marker)], timeout=3)
            self.assertEqual(result.returncode, 0)
            self.assertFalse(marker.exists())

    def test_position_gate_and_invalid_input(self):
        node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'fixstand_error')
        scope = {'math': math}
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(SCRIPT), 'exec'), scope)
        check = scope['fixstand_error']
        self.assertAlmostEqual(check([.6], [.5], .11), .1)
        self.assertGreater(check([1.1], [.5], .11), .11)
        for q in ([float('nan')], []):
            with self.assertRaises(RuntimeError): check(q, [.5], .11)

if __name__ == '__main__':
    unittest.main()
