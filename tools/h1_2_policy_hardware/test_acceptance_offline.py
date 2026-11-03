#!/usr/bin/env python3
"""Offline checks; never imports ROS or starts the monitor."""
import ast
import math
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

    def test_requires_explicit_finite_positive_limit_before_ros_import(self):
        for args in [('--execute',), ('--execute', '--fixstand-max-error', 'nan'),
                     ('--execute', '--fixstand-max-error', '0'), ('--execute', '--fixstand-max-error', 'inf')]:
            result = self.run_cli(*args, '--confirm-debug-mode', '--confirm-exclusive-control')
            self.assertEqual(result.returncode, 2)
            self.assertIn('finite positive', result.stderr)

    def test_execute_requires_both_human_assertions_before_ros_import(self):
        for flags in ((), ('--confirm-debug-mode',), ('--confirm-exclusive-control',)):
            result = self.run_cli('--execute', '--fixstand-max-error', '.1', *flags)
            self.assertEqual(result.returncode, 2)
            self.assertIn('requires --confirm-debug-mode and --confirm-exclusive-control', result.stderr)
            self.assertNotIn('rclpy', result.stderr)

    def test_fixstand_only_rejects_policy_budget(self):
        self.assertEqual(self.run_cli('--fixstand-only', '--policy-steps', '1',
                                      '--confirm-supported-upright').returncode, 2)
        result = self.run_cli('--fixstand-only')
        self.assertEqual(result.returncode, 0)
        self.assertNotIn('arm_hold_stand (3.1 s)', result.stdout)
        self.assertIn('operator_confirmed_debug_mode:=false', result.stdout)
        self.assertIn('operator_confirmed_exclusive_control:=false', result.stdout)
        self.assertIn('neither is machine verified', result.stdout)

    def test_fixstand_only_blocks_later_services_before_client_creation(self):
        node = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'call')
        scope = {'args': SimpleNamespace(fixstand_only=True)}
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(SCRIPT), 'exec'), scope)
        for service in ('arm_hold_stand', 'arm_hold'):
            with self.assertRaisesRegex(RuntimeError, 'forbidden'): scope['call'](service)

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
