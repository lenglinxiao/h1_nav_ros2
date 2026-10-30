#!/usr/bin/env python3
"""Evaluate an observable DDS physics CSV trace; nonzero exit means failure."""
import csv
import json
import math
from pathlib import Path
import sys

def evaluate(trace, wanted, vx, wz):
    wanted, vx, wz = map(float, (wanted, vx, wz))
    result = {'evaluation_status': 'failed', 'errors': []}
    try:
        rows = [r for r in csv.DictReader(Path(trace).open()) if r['stage'] == 'active']
        if len(rows) < 2:
            raise ValueError('insufficient active samples')
        fields = ('wall_time','sim_time','height','roll','pitch','body_vx','body_vy','body_wz',
                  'commands','bad_crc','bad_command','command_age','states','base_constrained','support_newtons','contacts','fall','invalid')
        data = [{k: float(r[k]) for k in fields} for r in rows]
        if not all(math.isfinite(r[k]) for r in data for k in fields):
            raise ValueError('nonfinite active physics or communications sample')
        wall = data[-1]['wall_time'] - data[0]['wall_time']
        sim = data[-1]['sim_time'] - data[0]['sim_time']
        count = data[-1]['commands'] - data[0]['commands']
        state_count = data[-1]['states'] - data[0]['states']
        if wall <= 0 or sim <= 0:
            raise ValueError('nonpositive active duration')
        errors = result['errors']
        if sim + 1e-8 < wanted: errors.append('insufficient active physics duration')
        if any(b['sim_time'] < a['sim_time'] or b['wall_time'] < a['wall_time'] or b['commands'] < a['commands'] for a,b in zip(data,data[1:])): errors.append('nonmonotonic samples')
        if any(r['invalid'] != 0 for r in data): errors.append('invalid physics state')
        if any(r['fall'] != 0 for r in data): errors.append('fall')
        if any(r['base_constrained'] != 0 or r['support_newtons'] != 0 for r in data): errors.append('support active after takeover')
        if any(r['bad_crc'] != 0 or r['bad_command'] != 0 for r in data): errors.append('invalid DDS command or CRC')
        if any(not 0 <= r['command_age'] < .1 for r in data): errors.append('stale or missing command')
        if not 45 <= count/wall <= 55 or not 45 <= count/sim <= 55: errors.append('command rate outside 45–55 Hz')
        if not 450 <= state_count/wall <= 550 or not 450 <= state_count/sim <= 550: errors.append('state rate outside 450–550 Hz')
        result.update(active_wall_duration=wall, active_physics_duration=sim,
            lowcmd_wall_hz=count/wall, lowcmd_physics_hz=count/sim, lowstate_wall_hz=state_count/wall, lowstate_physics_hz=state_count/sim,
            bad_crc=int(data[-1]['bad_crc']), bad_command=int(data[-1]['bad_command']),
            max_command_age=max(r['command_age'] for r in data),
            active_min_height=min(r['height'] for r in data), active_max_height=max(r['height'] for r in data),
            active_height_within_095_plus_minus_003=all(.92 <= r['height'] <= .98 for r in data),
            active_max_abs_roll=max(abs(r['roll']) for r in data), active_max_abs_pitch=max(abs(r['pitch']) for r in data),
            active_mean_body_vx=sum(r['body_vx'] for r in data)/len(data),
            active_mean_body_vy=sum(r['body_vy'] for r in data)/len(data),
            active_mean_body_wz=sum(r['body_wz'] for r in data)/len(data),
            active_base_constrained=any(r['base_constrained'] != 0 for r in data),
            active_support_newtons=max(r['support_newtons'] for r in data), command_vx=vx, command_wz=wz)
        foot_fields = ('left_max_gap','right_max_gap','left_max_lift','right_max_lift','left_max_drift','right_max_drift')
        if all(k in rows[0] for k in foot_fields):
            foot = {k: float(rows[-1][k]) for k in foot_fields}
            if not all(math.isfinite(v) for v in foot.values()):
                errors.append('nonfinite foot metric')
            else:
                result.update(foot)
                result['foot_contact_sample_period'] = .002
                result['strict_no_step_assessed'] = False
        if not errors: result['evaluation_status'] = 'passed_physics_and_transport_checks'
    except (ValueError, TypeError, OSError, KeyError, ZeroDivisionError) as exc:
        result['errors'].append(str(exc))
    return result

if __name__ == '__main__':
    trace, output, wanted, vx, wz = sys.argv[1:]
    result = evaluate(trace, wanted, vx, wz)
    rendered = json.dumps(result, indent=2, allow_nan=False)
    Path(output).write_text(rendered + '\n')
    print(rendered)
    sys.exit(0 if result['evaluation_status'] == 'passed_physics_and_transport_checks' else 1)
