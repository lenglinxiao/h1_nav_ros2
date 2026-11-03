#!/usr/bin/env python3
"""Short real MuJoCo/DDS regression, isolated to domain 42 and loopback."""
import csv
import os
from pathlib import Path
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parent
BUILD = Path('/tmp/h1_2_policy_sim_build')
SDK = Path('/home/llx/unitree_sdk2')
LIB = SDK / 'thirdparty/lib/x86_64'
subprocess.run(['g++', '-std=c++17', '-O2', '-I'+str(SDK/'include'),
                '-I'+str(SDK/'thirdparty/include'), '-I'+str(SDK/'thirdparty/include/ddscxx'),
                str(ROOT/'test_position_sender.cpp'), str(SDK/'lib/x86_64/libunitree_sdk2.a'),
                str(LIB/'libddscxx.so'), str(LIB/'libddsc.so'), '-pthread',
                '-Wl,--disable-new-dtags,-rpath,'+str(LIB), '-o', str(BUILD/'position_sender')], check=True)

def run(kd):
    with tempfile.TemporaryDirectory(prefix='h1_position_dds_') as temp:
        folder = Path(temp); (folder/'stage').write_text('fix\n')
        p = subprocess.Popen([str(BUILD/'h1_2_dds_physics'), str(folder/'stage'), str(folder/'trace.csv'), 'position_pd'],
                             stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        try:
            time.sleep(.7)
            subprocess.run([str(BUILD/'position_sender'), str(kd)], check=True, timeout=8)
            time.sleep(.1)
        finally:
            p.terminate(); p.wait(timeout=5)
        rows = list(csv.DictReader((folder/'trace.csv').open()))
        assert rows and max(int(x['commands']) for x in rows) > 30
        # Five wrong-mode frames then five non-finite frames: >5 proves both phases were rejected.
        assert max(int(x['bad_command']) for x in rows) > 5
        q = [float(x['left_knee']) for x in rows if int(x['commands']) > 5]
        assert max(q) > .54, (kd, min(q), max(q))
        return max(q), q[-1], max(int(x['simulated_torque_clips']) for x in rows)

low, high = run(.5), run(10)
assert abs(low[0]-high[0]) > .005, (low, high)
print('position PD loopback: kd=.5', low, 'kd=10', high,
      '(peak q, final q, simulation clip count; not firmware evidence)')
