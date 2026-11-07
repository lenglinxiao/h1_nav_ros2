#!/usr/bin/env python3
"""Offline regression of watch timing and final freshness checks."""
import ast
import types
import unittest
from pathlib import Path


class WatchWindowTests(unittest.TestCase):
    def run_watch(self, received_rate=50, written_rate=50, final_age=0):
        tree = ast.parse(Path(__file__).with_name('run_acceptance.py').read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'watch')
        clock = types.SimpleNamespace(value=0.0)
        summary = {}
        calls = 0
        def status(expected):
            nonlocal calls
            calls += 1
            if calls == 1: clock.value += .081
            return dict(write_count=str(int(clock.value * written_rate)), authorized='true', publisher_present='true')
        def sample():
            age = final_age if clock.value >= .181 else 0
            return dict(commands=int(clock.value * received_rate), command_age=age, peak_dq=0, dq=[0])
        def spin_once(*args, **kwargs): clock.value += .01
        process = types.SimpleNamespace(poll=lambda: None)
        namespace = dict(time=types.SimpleNamespace(monotonic=lambda: clock.value), sample=sample, status=status,
                         process=process, monitor=process, pulse=None, stationary_gate=None, check_speed=lambda value: None,
                         rclpy=types.SimpleNamespace(spin_once=spin_once), node=None, summary=summary)
        exec(compile(ast.Module(body=[function], type_ignores=[]), '<watch>', 'exec'), namespace)
        try:
            namespace['watch'](.1, 'ArmHold')
        except RuntimeError as exc:
            return summary, str(exc)
        return summary, None

    def test_slow_initial_status_keeps_full_observation_window(self):
        summary, error = self.run_watch()
        self.assertIsNone(error)
        self.assertGreaterEqual(summary['windows'][0]['seconds'], .1)
        self.assertGreaterEqual(summary['windows'][0]['written'], 2)

    def test_low_rate_with_progress_is_diagnostic_only(self):
        summary, error = self.run_watch(received_rate=10, written_rate=10)
        self.assertIsNone(error)
        self.assertTrue(summary['windows'][0]['under_expected_rate'])
        self.assertEqual(summary['windows'][0]['minimum'], 2)

    def test_final_freshness_checks_remain_enforced(self):
        for kwargs, field in [({'received_rate': 0}, 'received'), ({'written_rate': 0}, 'written'), ({'final_age': .11}, 'command_age')]:
            with self.subTest(field=field):
                summary, error = self.run_watch(**kwargs)
                self.assertIn('Insufficient fresh command frames', error)
                failed = summary['failed_observation_window']
                self.assertAlmostEqual(failed['setup_elapsed'], .081)
                self.assertGreaterEqual(failed['actual_seconds'], .1)
                self.assertIn(field, failed)
                self.assertIn('minimum=2', error)

if __name__ == '__main__':
    unittest.main()
