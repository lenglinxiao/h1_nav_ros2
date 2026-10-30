#!/usr/bin/env python3
"""Black-box contract for the offline headless runner."""
import csv
import json
import pathlib
import subprocess
import sys
import tempfile

runner = pathlib.Path(sys.argv[1])
with tempfile.TemporaryDirectory() as directory:
    output = pathlib.Path(directory)
    result = subprocess.run([str(runner), '--duration', '0.1', '--output', str(output)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    summary = json.loads((output / 'summary.json').read_text())
    assert summary['physics_dt'] > 0
    assert summary['control_period'] == 0.02
    assert summary['dds_enabled'] is False
    assert summary['base_fixed_after_takeover'] is False
    assert summary['preparation'] == 'ideal_fixed_base'
    assert summary['takeover_accepted'] is True
    assert summary['evaluation_status'] == 'completed_without_fall'
    assert summary['active_duration'] >= 0.1 - 1e-9
    assert summary['base_fixed_during_preparation'] is True
    assert summary['active_applied_upward_force_newtons'] == 0
    assert summary['takeover_time'] >= 6
    assert summary['model_sha256'] == '3090836d0ce454a35161982b60dde20e0fd87627150e06126aabd1e6bbfabae7'
    with (output / 'trace.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows and {'stage', 'time', 'height', 'roll', 'pitch', 'vx', 'vy', 'wz', 'saturated', 'fall'} <= set(rows[0])
    assert any(row['stage'] == 'ArmHold' and float(row['support_newtons']) == 0 and row['base_constrained'] == '0' for row in rows)
for option in ('--duration', '--vx', '--wz'):
    for value in ('nan', 'inf', '-inf'):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([str(runner), '--output', directory, option, value], capture_output=True, timeout=30)
            assert result.returncode != 0, (option, value)
