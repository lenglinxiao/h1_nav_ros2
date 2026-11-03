#!/usr/bin/env python3
"""Dry-run hardware acceptance by default; execution requires explicit opt-in."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import select
import sys
import subprocess
import time

def fixstand_error(q, target, limit):
    if len(q) != len(target) or not all(math.isfinite(x) for x in q):
        raise RuntimeError('Invalid FixStand positions')
    error = max(abs(a-b) for a, b in zip(q, target))
    return error


parser = argparse.ArgumentParser(description="H1 hardware acceptance; dry-run by default")
parser.add_argument('--source-commit', required=True, help='commit actually deployed on the robot (operator supplied)')
parser.add_argument('--fixstand-only', action='store_true', help='stop after FixStand and cleanup')
parser.add_argument('--fixstand-max-error', type=float, help='required maximum absolute joint position error in radians for execution')
parser.add_argument('--execute', action='store_true', help='enable physical preparation')
parser.add_argument('--confirm-debug-mode', action='store_true', help='operator confirms debug mode by the applicable procedure')
parser.add_argument('--confirm-exclusive-control', action='store_true', help='operator confirms exclusive control by the applicable procedure')
parser.add_argument('--policy-steps', type=int, choices=range(6), default=0, metavar='0..5')
parser.add_argument('--confirm-supported-upright', action='store_true', help='enable the onsite confirmation prompt after ArmHoldStand settles')
args = parser.parse_args()
if args.fixstand_only and args.policy_steps:
    parser.error('--fixstand-only requires --policy-steps=0')
if args.execute and not (args.confirm_debug_mode and args.confirm_exclusive_control):
    parser.error('--execute requires --confirm-debug-mode and --confirm-exclusive-control')
if args.execute and (args.fixstand_max_error is None or not math.isfinite(args.fixstand_max_error) or args.fixstand_max_error <= 0):
    parser.error('--execute requires a finite positive --fixstand-max-error')
if args.policy_steps and not args.confirm_supported_upright:
    parser.error('--policy-steps > 0 requires --confirm-supported-upright and an onsite confirmation after settling')
ws = Path('/home/unitree/h1_nav2_ros2_ws')
namespace = '/h1_prepare_' + str(os.getpid())
command=[str(ws/'install/h1_2_policy_bridge/lib/h1_2_policy_bridge/h1_2_policy_node'),
         '--ros-args','-r','__ns:='+namespace,
         '-p','profile:=hardware','-p','network_interface:=eth0','-p','dds_domain:=0',
         '-p','model_path:='+str(ws/'models/h1_2_armhold/policy.onnx'),
         '-p','enable_hardware_output:=true','-p','operator_confirmed_debug_mode:='+str(args.confirm_debug_mode).lower(),
         '-p','operator_confirmed_exclusive_control:='+str(args.confirm_exclusive_control).lower(),'-p','max_policy_steps:='+str(args.policy_steps)]
if not args.execute:
    import shlex
    print('DRY RUN: no process, ROS connection or physical command executed')
    print('Operator assertions: debug_mode=', args.confirm_debug_mode, 'exclusive_control=', args.confirm_exclusive_control, '; neither is machine verified')
    print('Monitor:', shlex.join([str(ws/'tools/hardware-preflight/h1_prepare_monitor'), '<telemetry.json>', '<trace.csv>']))
    print('Node:', shlex.join(command))
    print('Services: status, authorize_output, passive observation, fix_stand (3.1 s)' + ('' if args.fixstand_only else ', arm_hold_stand (3.1 s)'))
    print('Checks: fresh finite LowState and CRC; no existing LowCmd writer; advancing command counts, exclusivity, valid state and speed <= 2 rad/s; native damping; revoke and verify no further writes.')
    print('Policy:', args.policy_steps, 'frames; requires --confirm-supported-upright and onsite typed confirmation after 3 s in ArmHoldStand, sling ~60%, height ~0.95 m and straight legs, and both knee errors <0.07 rad; no retry.')
    print('Finally: passive, verify damping, revoke output, verify no writes, stop owned processes. Deployed source commit:', args.source_commit)
    raise SystemExit(0)
os.environ.update(ROS_DOMAIN_ID='43', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
import rclpy
from std_srvs.srv import Trigger
record = ws / 'third_party/h1_policy_runtime/integration-record' / ('acceptance-' + time.strftime('%Y%m%dT%H%M%S'))
record.mkdir(parents=True)
(record / 'COLCON_IGNORE').touch()
os.environ['ROS_LOG_DIR'] = str(record / 'ros-logs')
telemetry_path = record / 'telemetry.json'
summary = dict(result='not_started', policy_budget=args.policy_steps, policy_entered='unconfirmed',
               fixstand_only=args.fixstand_only, fixstand_max_error_limit=args.fixstand_max_error,
               operator_confirmed_debug_mode=args.confirm_debug_mode,
               operator_confirmed_exclusive_control=args.confirm_exclusive_control,
               source_commit=args.source_commit, source_commit_evidence='operator_supplied_unconfirmed',
               onsite='unconfirmed', events=[], endpoints={})
process = monitor = node = None
authorized = False
logs = []

for entry in Path('/proc').iterdir():
    if not entry.name.isdigit(): continue
    try:
        proc_args=(entry/'cmdline').read_bytes().split(b'\0')
        if proc_args and os.path.basename(proc_args[0].decode()) == 'h1_2_policy_node':
            raise RuntimeError('Existing policy node: ' + entry.name)
    except (OSError,UnicodeError): pass

def interrupt(signum, frame):
    raise RuntimeError('Preparation interrupted')
signal.signal(signal.SIGTERM, interrupt)
signal.signal(signal.SIGINT, interrupt)

def spawn(command, name):
    log=(record/(name+'.log')).open('w'); logs.append(log)
    return subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

def sample():
    value=json.loads(telemetry_path.read_text())
    if value.get('schema') != 2:
        raise RuntimeError('Incompatible monitor schema; rebuild prepare_monitor')
    if (time.time()-telemetry_path.stat().st_mtime > .15 or value['states'] < 2 or
        value['age'] > .1 or not value['finite'] or value['bad_crc'] != 0 or
        value['machine'] != 6 or value['pr'] != 0):
        raise RuntimeError('Telemetry invalid, stale, or bad CRC')
    if not all(math.isfinite(v) for v in value['q']+value['dq']):
        raise RuntimeError('Nonfinite joint telemetry')
    return value

def call(name):
    # Only the listed acceptance services are available; no release API is called.
    if args.fixstand_only and name in ('arm_hold_stand', 'arm_hold'):
        raise RuntimeError('Service forbidden in fixstand-only mode')
    if name not in ('status','authorize_output','passive','fix_stand','arm_hold_stand','arm_hold','revoke_output'):
        raise RuntimeError('Unapproved service')
    if process.poll() is not None: raise RuntimeError('Policy process exited')
    client=node.create_client(Trigger,namespace+'/h1_2_policy_node/'+name)
    try:
        if not client.wait_for_service(timeout_sec=4 if not authorized else .8):
            raise RuntimeError('Service unavailable: '+name)
        request_monotonic=time.monotonic()
        future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=2 if name == 'authorize_output' else .8)
        if not future.done() or future.exception(): raise RuntimeError('Service timeout: '+name)
        res=future.result()
        summary['events'].append(dict(time=time.time(),request_monotonic=request_monotonic,response_monotonic=time.monotonic(),service=name,evidence='sent_and_service_response',success=res.success,message=res.message))
        return res
    finally: node.destroy_client(client)

def status(expected=None):
    res=call('status')
    if not res.success: raise RuntimeError('Status failed')
    fields=dict(v.split('=',1) for v in res.message.split() if '=' in v)
    if fields.get('state') != 'valid': raise RuntimeError('Controller state invalid')
    if args.fixstand_only and fields.get('mode') not in (('Passive', 'FixStand', 'Fault') if expected == 'Fault' else ('Passive', 'FixStand')):
        raise RuntimeError('Unapproved fixstand-only state')
    if fields.get('mode') == 'ArmHold' and not args.policy_steps: raise RuntimeError('Unapproved policy state')
    if expected is not None and fields.get('mode') != expected:
        raise RuntimeError('Unexpected mode: '+res.message)
    return fields

def watch(seconds, expected, monitor_motion=True):
    started=time.monotonic()
    deadline=started+seconds
    baseline=sample()['commands']
    initial_fields=status(expected)
    initial_writes=int(initial_fields['write_count'])
    last_status=0
    fields=None
    while time.monotonic()<deadline:
        if process.poll() is not None or monitor.poll() is not None:
            raise RuntimeError('A controlled test process exited')
        value=sample()
        if time.monotonic()-started>.1 and (value['commands']<=baseline or value['command_age']>.1):
            raise RuntimeError('LowCmd stream not progressing or stale')
        if monitor_motion and value['peak_dq']>2.0:
            raise RuntimeError('Preparation joint speed exceeded 2 rad/s')
        if time.monotonic()-last_status>.1:
            fields=status(expected)
            if fields.get('authorized')!='true' or fields.get('publisher_present')!='true':
                raise RuntimeError('Output authorization lost')
            if value['commands'] > int(fields['write_count'])+12:
                raise RuntimeError('Possible second LowCmd writer')
            last_status=time.monotonic()
        rclpy.spin_once(node,timeout_sec=.01)
    final=sample()
    final_fields=status(expected)
    minimum=max(2,int(seconds*25))
    received=final['commands']-baseline
    written=int(final_fields['write_count'])-initial_writes
    if received<minimum or written<minimum or final['command_age']>.1:
        raise RuntimeError('Insufficient fresh command frames in observation window')
    summary.setdefault('windows',[]).append(dict(evidence='observedDDS_and_node_status',mode=expected,seconds=time.monotonic()-started,
                                               received=received,written=written))
    return final

fix=[0,-.2,0,.5,-.3,0,0,-.2,0,.5,-.3,0,0,.28,0,0,.52,0,0,0,.28,0,0,.52,0,0,0]
stand=[.027,-.45,0,.9,-.45,0,-.027,-.45,0,.9,-.475,0,0,
       .270,.001,0,.483,-.030,-.003,-.002,-.239,-.192,.941,-.633,-.076,.004,.026]
try:
    monitor=spawn([str(ws/'tools/hardware-preflight/h1_prepare_monitor'),str(telemetry_path),str(record/'trace.csv')],'monitor')
    deadline=time.monotonic()+5
    while not telemetry_path.exists():
        if monitor.poll() is not None or time.monotonic()>deadline: raise RuntimeError('Monitor unavailable')
        time.sleep(.02)
    time.sleep(.6)
    initial=sample()
    if initial['commands'] != 0: raise RuntimeError('Existing LowCmd traffic; not taking control')
    if initial['peak_dq'] > .5: raise RuntimeError('Robot not stationary before preparation')
    summary['initial']=initial
    process=spawn(command,'node')
    rclpy.init(args=[]); node=rclpy.create_node('preparation_runner',namespace=namespace)
    time.sleep(.6)
    fields=status('Passive')
    if fields['authorized']!='false' or fields['publisher_present']!='false' or fields['write_count']!='0':
        raise RuntimeError('Unexpected startup output')
    if sample()['commands']!=0: raise RuntimeError('Another writer started before authorization')
    # Authorization may succeed even if its response is lost; always attempt cleanup.
    authorized=True
    res=call('authorize_output')
    if not res.success: raise RuntimeError('Authorization rejected: '+res.message)
    value=watch(.25,'Passive')
    if not value['native_damping']: raise RuntimeError('Passive command is not native damping')
    print('Passive output verified; starting approved FixStand preparation',flush=True)
    if not call('fix_stand').success: raise RuntimeError('FixStand rejected')
    value=watch(3.1,'FixStand')
    summary['endpoints']['FixStand']=value
    error=fixstand_error(value['q'], fix, args.fixstand_max_error)
    summary['fixstand_max_error']=error
    summary['fixstand_worst_joint_sdk']=max(range(len(fix)), key=lambda i: abs(value['q'][i]-fix[i]))
    if error > args.fixstand_max_error:
        raise RuntimeError(f'FixStand position error {error:.6f} exceeds configured limit {args.fixstand_max_error:.6f}')
    if args.fixstand_only:
        summary['result']='fixstand_only_completed'
    else:
        print('FixStand finished; starting approved ArmHoldStand preparation',flush=True)
        if not call('arm_hold_stand').success: raise RuntimeError('ArmHoldStand rejected')
        value=watch(3.1,'ArmHoldStand')
        summary['endpoints']['ArmHoldStand']=value
        summary['armholdstand_max_error']=max(abs(a-b) for a,b in zip(value['q'],stand))
        summary['knees_within_policy_position_gate']=all(abs(value['q'][i]-.9)<.07 for i in (3,9))
        summary['result']='preparation_executed_no_policy'
        if args.policy_steps:
            if not summary['knees_within_policy_position_gate']:
                raise RuntimeError('Knee position gate failed; no policy')
            print('Onsite: verify sling ~60% support, height ~0.95 m and legs straight. Type CONFIRM to continue:', flush=True)
            confirmation_deadline=time.monotonic()+10
            answer=None
            while time.monotonic()<confirmation_deadline:
                watch(.2,'ArmHoldStand')
                if select.select([sys.stdin],[],[],0)[0]:
                    answer=sys.stdin.readline().strip()
                    break
            if answer != 'CONFIRM':
                raise RuntimeError('Onsite confirmation withheld or timed out')
            summary['onsite'] = 'operator_confirmed_sling_60_percent_height_0.95m_straight_legs'
            # Recheck telemetry and posture after the operator confirmation, before requesting policy.
            value=watch(.25,'ArmHoldStand')
            if not all(abs(value['q'][i]-.9)<.07 for i in (3,9)):
                raise RuntimeError('Knee position gate failed after onsite confirmation')
            if not call('arm_hold').success: raise RuntimeError('ArmHold rejected')
            summary['policy_entered']='service_accepted_unconfirmed_by_dds'
            # A brief policy state can disappear before the next status poll.
            deadline=time.monotonic()+.6
            while True:
                value=sample()
                if process.poll() is not None or monitor.poll() is not None or value['peak_dq']>2.0 or value['command_age']>.1:
                    raise RuntimeError('Fault during policy budget')
                fields=status()
                if fields.get('authorized')!='true' or fields.get('publisher_present')!='true' or value['commands']>int(fields['write_count'])+12:
                    raise RuntimeError('Output authorization or exclusivity lost during policy budget')
                if fields.get('mode')=='Passive': break
                if fields.get('mode')!='ArmHold' or time.monotonic()>deadline:
                    raise RuntimeError('Policy did not return to Passive')
                rclpy.spin_once(node,timeout_sec=.01)
            value=watch(.3,'Passive')
            if 'policy budget exhausted; entering Passive damping' not in (record/'node.log').read_text(errors='replace'):
                raise RuntimeError('Policy budget exhaustion not confirmed in node log')
            summary['policy_budget_evidence']='node log reports budget exhausted; Passive and command stream observed; physical execution unconfirmed'
            summary['policy_after_budget_observed_dds']=value
            summary['result']='policy_budget_completed_unconfirmed_physical_behavior'
except Exception as exc:
    summary.update(result='aborted',error=str(exc))
    print('PREPARATION ABORT:',exc,'; returning to damping if available. Use physical emergency stop for abnormal motion.',flush=True)
finally:
    # A second terminal interrupt must not interrupt the already-running
    # damping/revocation/owned-process cleanup sequence.
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    if authorized and process is not None and process.poll() is None and node is not None:
        try:
            res=call('passive')
            if not res.success: raise RuntimeError('Passive rejected: '+res.message)
            final=watch(.3,'Passive',monitor_motion=False)
            if not final['native_damping']: raise RuntimeError('Native damping not observed at exit')
            summary['final_damping']=final
        except Exception as exc:
            summary.update(result='needs_operator_check',exit_error=str(exc))
            print('Unable to verify damping; onsite operator must use the physical emergency stop if needed:',exc,flush=True)
        try:
            res=call('revoke_output')
            if not res.success: raise RuntimeError('Revoke request rejected: '+res.message)
            fields=status('Fault')
            if fields.get('authorized')!='false' or fields.get('publisher_present')!='false':
                raise RuntimeError('Output revocation not confirmed')
            count=int(fields['write_count'])
            time.sleep(.1)
            if int(status('Fault')['write_count'])!=count:
                raise RuntimeError('Writes continued after revocation')
            summary['revocation_confirmed']=True
        except Exception as exc:
            summary.update(result='needs_operator_check',revoke_error=str(exc))
    elif authorized:
        summary.update(result='needs_operator_check', cleanup_status='unknown_process_exited_or_unavailable')
        print('Cleanup could not be verified; onsite operator check required.', flush=True)
    if node is not None: node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    for p in (process,monitor):
        if p is None: continue
        for sig,timeout in ((signal.SIGINT,3),(signal.SIGTERM,2),(signal.SIGKILL,2)):
            if p.poll() is not None: break
            try: os.killpg(p.pid,sig); p.wait(timeout=timeout)
            except ProcessLookupError: pass
            except subprocess.TimeoutExpired: continue
    for log in logs: log.close()
    summary['node_returncode']=process.returncode if process else None
    summary['monitor_returncode']=monitor.returncode if monitor else None
    summary['record']=str(record)
    (record/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(json.dumps(summary,indent=2,allow_nan=False),flush=True)
raise SystemExit(0 if summary['result'] in ('fixstand_only_completed','preparation_executed_no_policy','policy_budget_completed_unconfirmed_physical_behavior') else 1)
