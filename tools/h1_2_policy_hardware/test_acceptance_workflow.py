#!/usr/bin/env python3
"""Exercise the real acceptance workflow offline with isolated I/O fakes."""
import contextlib
import io
import json
import runpy
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

SCRIPT = Path(__file__).with_name('run_acceptance.py')

class FakeProcess:
    def __init__(self, name, case):
        self.name, self.case, self.returncode, self.pid = name, case, None, 12345
    def poll(self):
        if self.name == 'monitor' and self.case.monitor_exited:
            return 1
        return self.returncode

class Case:
    def __init__(self, failure=None):
        self.failure = failure
        self.services = []
        self.mode = 'Passive'
        self.authorized = False
        self.writes = 0
        self.clock = 0.0
        self.monitor_exited = False
        self.samples = 0
        self.failed = False
    def now(self):
        self.clock += .015
        return self.clock
    def sample(self):
        self.samples += 1
        if self.failure == 'schema' and self.samples == 1:
            raise RuntimeError('Incompatible monitor schema; rebuild prepare_monitor')
        if self.authorized:
            self.writes += 2
        q = [0, -.2, 0, .5, -.3, 0, 0, -.2, 0, .5, -.3, 0, 0, .28, 0, 0, .52, 0, 0, 0, .28, 0, 0, .52, 0, 0, 0]
        if self.failure == 'position' and self.mode == 'FixStand': q[23] = 1.2
        peak = 2.1 if self.failure == 'speed' and self.mode == 'FixStand' else 0
        if self.failure == 'monitor' and self.mode == 'FixStand': self.monitor_exited = True
        return dict(schema=2, states=100, commands=self.writes, bad_crc=0, age=0, command_age=0,
                    finite=True, machine=6, pr=0, native_damping=self.mode == 'Passive',
                    peak_dq=peak, q=q, dq=[0]*27)
    def call(self, name):
        self.services.append(name)
        if name == 'authorize_output':
            self.authorized = True
            if self.failure == 'authorization_timeout': raise RuntimeError('Service timeout: authorize_output')
        elif name == 'fix_stand':
            self.mode = 'FixStand'
            if self.failure == 'service_timeout': raise RuntimeError('Service timeout: fix_stand')
        elif name == 'passive':
            if self.failure == 'passive_failure': raise RuntimeError('Passive unavailable')
            self.mode = 'Passive'
        elif name == 'revoke_output':
            self.authorized = False
            self.mode = 'Fault'
        if name == 'status':
            if self.authorized: self.writes += 2
            msg = f'mode={self.mode} state=valid authorized={str(self.authorized).lower()} publisher_present={str(self.authorized).lower()} write_count={self.writes}'
            return types.SimpleNamespace(success=True, message=msg)
        return types.SimpleNamespace(success=True, message='accepted')

class WorkflowTests(unittest.TestCase):
    def run_case(self, failure=None):
        case = Case(failure)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = SCRIPT.read_text()
            source = source.replace("ws = Path('/home/unitree/h1_nav2_ros2_ws')", f'ws = Path({tmp!r})')
            source = source.replace("for entry in Path('/proc').iterdir():", 'for entry in ():')
            source = source.replace("monitor=spawn([str(ws/'tools/hardware-preflight/h1_prepare_monitor'),str(telemetry_path),str(record/'trace.csv')],'monitor')", "monitor=spawn([], 'monitor'); telemetry_path.touch()")
            # Only replace external I/O boundaries; status, watch, main workflow and finally remain real.
            marker = '\ntry:\n    monitor=spawn('
            inject = '''\nsample = _case.sample\ncall = _case.call\ndef spawn(command, name): return FakeProcess(name, _case)\n'''
            if marker not in source: self.fail('workflow injection point changed')
            source = source.replace(marker, inject + marker, 1)
            script = root / 'isolated_runner.py'
            script.write_text(source)
            ros = types.ModuleType('rclpy')
            ros.init = lambda **kw: None
            ros.ok = lambda: False
            ros.create_node = lambda *a, **kw: types.SimpleNamespace(destroy_node=lambda: None)
            ros.spin_once = lambda *a, **kw: None
            srv = types.ModuleType('std_srvs.srv'); srv.Trigger = object
            mods = {'rclpy': ros, 'std_srvs': types.ModuleType('std_srvs'), 'std_srvs.srv': srv}
            argv = [str(script), '--source-commit', 'test', '--execute', '--fixstand-only', '--fixstand-max-error', '.2',
                    '--confirm-debug-mode', '--confirm-exclusive-control']
            with patch.dict(sys.modules, mods), patch.object(sys, 'argv', argv), \
                 patch('time.monotonic', side_effect=case.now), patch('time.sleep', return_value=None), \
                 patch('os.killpg', return_value=None), contextlib.redirect_stdout(io.StringIO()):
                # Owned fake processes stop without touching the operating system.
                FakeProcess.wait = lambda self, timeout=None: setattr(self, 'returncode', 0)
                with self.assertRaises(SystemExit):
                    runpy.run_path(str(script), init_globals={'_case': case, 'FakeProcess': FakeProcess}, run_name='__main__')
            summaries = list(root.glob('third_party/h1_policy_runtime/integration-record/*/summary.json'))
            self.assertEqual(len(summaries), 1)
            return case, json.loads(summaries[0].read_text())

    def test_success_and_cleanup(self):
        c, result = self.run_case()
        self.assertEqual(result['result'], 'fixstand_only_completed')
        self.assertIn('fix_stand', c.services); self.assertIn('passive', c.services); self.assertIn('revoke_output', c.services)
        self.assertFalse({'arm_hold_stand', 'arm_hold'} & set(c.services))
        self.assertTrue(result['revocation_confirmed'])

    def test_faults_revoke(self):
        for failure in ('position', 'speed', 'authorization_timeout', 'passive_failure', 'monitor', 'service_timeout'):
            with self.subTest(failure=failure):
                c, result = self.run_case(failure)
                self.assertNotEqual(result['result'], 'fixstand_only_completed')
                self.assertIn('revoke_output', c.services)
                self.assertFalse({'arm_hold_stand', 'arm_hold'} & set(c.services))
                self.assertTrue('error' in result or 'exit_error' in result)

    def test_old_schema_rejected_before_authorization(self):
        c, result = self.run_case('schema')
        self.assertNotIn('authorize_output', c.services)
        self.assertEqual(result['result'], 'aborted')

if __name__ == '__main__': unittest.main()
