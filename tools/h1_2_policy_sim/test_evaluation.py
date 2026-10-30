#!/usr/bin/env python3
"""Black-box acceptance fixtures: a bad physical trace must never pass."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile

evaluator = Path(__file__).with_name('evaluate_dds.py')
fields = ['wall_time', 'sim_time', 'stage', 'height', 'roll', 'pitch', 'body_vx', 'body_vy', 'body_wz', 'commands', 'bad_crc', 'bad_command', 'command_age', 'states', 'base_constrained', 'support_newtons', 'contacts', 'fall', 'invalid']
def row(t):
    return dict(zip(fields, [t,t,'active',.95,0,0,0,0,0,int(t*50),0,0,.01,int(t*500),0,0,2,0,0]))
def run(rows):
    with tempfile.TemporaryDirectory() as tmp:
        trace = Path(tmp)/'trace.csv'; output = Path(tmp)/'summary.json'
        with trace.open('w') as f:
            w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(rows)
        p=subprocess.run([sys.executable,str(evaluator),str(trace),str(output),'30','0','0'])
        return p.returncode,json.loads(output.read_text())
assert run([row(0),row(30)])[0] == 0
for key,value in [('sim_time',20),('bad_crc',1),('body_vx','nan'),('command_age',.11),('commands',3000),('states',1500),('base_constrained',1),('support_newtons',1),('fall',1),('invalid',1)]:
    rows=[row(0),row(30)];rows[-1][key]=value
    assert run(rows)[0] != 0,key
