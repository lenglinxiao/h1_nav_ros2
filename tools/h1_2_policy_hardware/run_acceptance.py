#!/usr/bin/env python3
"""Dry-run hardware acceptance by default; execution requires explicit opt-in."""
import argparse
import json
import math
import os
import re
from pathlib import Path
import signal
import select
import sys
import subprocess
import time

from process_guard import parent_death_preexec
from manual_pulse import ManualPulse
from repeat_input import RepeatInput
from speed_guard import speed_violation
from train_guard_profile import build_profile

class TrainedGuardStop(RuntimeError):
    def __init__(self, fields):
        self.fields = fields
        super().__init__("Training guard entered Passive")


class PreparationRejected(RuntimeError):
    pass

class PolicySourcePending(RuntimeError):
    pass


def fixstand_error(q, target, limit):
    if len(q) != len(target) or not all(math.isfinite(x) for x in q):
        raise RuntimeError('Invalid FixStand positions')
    error = max(abs(a-b) for a, b in zip(q, target))
    return error


parser = argparse.ArgumentParser(description="H1 hardware acceptance; dry-run by default")
parser.add_argument('--source-commit', required=True, help='commit actually deployed on the robot (operator supplied)')
parser.add_argument('--fixstand-only', action='store_true', help='stop after FixStand and cleanup')
parser.add_argument('--joint-command-mode', choices=('torque', 'position_pd'), default='torque')
parser.add_argument('--fixstand-max-error', type=float, help='optional maximum absolute FixStand joint error in radians; required for fixstand-only')
parser.add_argument('--protection-profile', choices=('legacy', 'train_20261106'), default='legacy')
parser.add_argument('--train-urdf', type=Path)
parser.add_argument('--continuous-walk-trial', action='store_true', help='isolated one-shot manual velocity trial')
parser.add_argument('--manual-pulse-seconds', type=float, default=.5, help='one-shot pulse duration, 0.5 to 3.0 seconds')
parser.add_argument('--manual-motion', choices=('forward', 'turn_left'), default='forward')
parser.add_argument('--allow-repeat-walk', action='store_true', help='allow explicit WALK commands while holding ArmHold')
parser.add_argument('--arm-hold-knee-tolerance', type=float, default=.07, help='knee position tolerance in radians (0.07 to 0.10 for continuous trial)')
parser.add_argument('--walk-leg-speed-limit', type=float, default=None, help='leg cumulative speed limit after manual pulse starts, 2 to 3 rad/s')
parser.add_argument('--execute', action='store_true', help='enable physical preparation')
parser.add_argument('--brief-summary', action='store_true', help='print a concise final summary; retain full summary.json')
parser.add_argument('--confirm-debug-mode', action='store_true', help='operator confirms debug mode by the applicable procedure')
parser.add_argument('--confirm-exclusive-control', action='store_true', help='operator confirms exclusive control by the applicable procedure')
parser.add_argument('--policy-steps', type=int, choices=range(156), default=0, metavar='0..155')
parser.add_argument('--max-policy-seconds', type=float, default=0.0,
                    help='monotonic deadline from ArmHold acceptance; 0 disables the time bound')
parser.add_argument('--onsite-confirm-timeout', type=float, default=10.0,
                    help='seconds allowed for onsite posture confirmation, at most 60')
parser.add_argument('--onsite-confirmed-before-start', action='store_true', help='onsite conditions confirmed before this continuous trial starts')
parser.add_argument('--confirm-supported-upright', action='store_true', help='enable the onsite confirmation prompt after ArmHoldStand settles')
args = parser.parse_args()
if not math.isfinite(args.manual_pulse_seconds) or not .5 <= args.manual_pulse_seconds <= 3.0:
    parser.error('--manual-pulse-seconds must be between 0.5 and 3.0')
if args.manual_pulse_seconds != .5 and not args.continuous_walk_trial:
    parser.error('--manual-pulse-seconds requires --continuous-walk-trial')
if args.manual_motion != 'forward' and not args.continuous_walk_trial:
    parser.error('--manual-motion requires --continuous-walk-trial')
if args.allow_repeat_walk and (not args.continuous_walk_trial or not args.execute or not sys.stdin.isatty()):
    parser.error('--allow-repeat-walk requires an interactive executed continuous trial')
if args.protection_profile == 'train_20261106':
    if not args.continuous_walk_trial or args.joint_command_mode != 'position_pd' or args.train_urdf is None:
        parser.error('train_20261106 requires --continuous-walk-trial, position_pd and --train-urdf')
    if args.walk_leg_speed_limit is not None:
        parser.error('--walk-leg-speed-limit is incompatible with train_20261106')
    try:
        train_profile = build_profile(args.train_urdf)
    except Exception as exc:
        parser.error('invalid training URDF: ' + str(exc))
else:
    if args.train_urdf is not None:
        parser.error('--train-urdf requires train_20261106')
    train_profile = None
if not math.isfinite(args.arm_hold_knee_tolerance) or not .07 <= args.arm_hold_knee_tolerance <= .10 or (args.arm_hold_knee_tolerance > .07 and not args.continuous_walk_trial):
    parser.error('--arm-hold-knee-tolerance must be 0.07 to 0.10; above 0.07 requires --continuous-walk-trial')
if args.walk_leg_speed_limit is not None:
    if not args.continuous_walk_trial:
        parser.error('--walk-leg-speed-limit requires --continuous-walk-trial')
    if not math.isfinite(args.walk_leg_speed_limit) or not 2.0 <= args.walk_leg_speed_limit <= 3.0:
        parser.error('--walk-leg-speed-limit must be finite and between 2 and 3 rad/s')
else:
    args.walk_leg_speed_limit = 2.0
if args.continuous_walk_trial and (args.joint_command_mode != 'position_pd' or args.policy_steps != 0 or args.max_policy_seconds != 0 or not args.confirm_supported_upright or args.fixstand_only):
    parser.error('--continuous-walk-trial requires position_pd, zero policy limits, --confirm-supported-upright and no --fixstand-only')
if args.onsite_confirmed_before_start and not (args.continuous_walk_trial and args.confirm_supported_upright and args.confirm_debug_mode and args.confirm_exclusive_control and args.execute and sys.stdin.isatty()):
    parser.error('--onsite-confirmed-before-start requires an executed interactive continuous trial with all confirmations')
if args.continuous_walk_trial and args.execute and not sys.stdin.isatty():
    parser.error('--continuous-walk-trial requires an interactive terminal')
if not math.isfinite(args.onsite_confirm_timeout) or not 0 < args.onsite_confirm_timeout <= 60:
    parser.error('--onsite-confirm-timeout must be finite and between 0 and 60')
if args.execute and args.joint_command_mode == 'position_pd' and not sys.stdin.isatty():
    parser.error('position_pd execution requires an interactive terminal for supported release')
if args.fixstand_only and args.policy_steps:
    parser.error('--fixstand-only requires --policy-steps=0')
if args.execute and not (args.confirm_debug_mode and args.confirm_exclusive_control):
    parser.error('--execute requires --confirm-debug-mode and --confirm-exclusive-control')
if args.fixstand_max_error is not None and (not math.isfinite(args.fixstand_max_error) or args.fixstand_max_error <= 0):
    parser.error('--fixstand-max-error must be finite and positive')
if args.fixstand_only and args.fixstand_max_error is None:
    parser.error('--fixstand-only requires a finite positive --fixstand-max-error')
if args.policy_steps and not args.confirm_supported_upright:
    parser.error('--policy-steps > 0 requires --confirm-supported-upright and an onsite confirmation after settling')
if not math.isfinite(args.max_policy_seconds) or not 0 <= args.max_policy_seconds <= 3.1:
    parser.error('--max-policy-seconds must be finite and between 0 and 3.1')
if args.max_policy_seconds and (not args.policy_steps or args.joint_command_mode != 'position_pd'):
    parser.error('--max-policy-seconds requires a nonzero policy budget and position_pd')
if args.policy_steps > 5 and not args.max_policy_seconds:
    parser.error('--policy-steps above 5 requires --max-policy-seconds')
ws = Path('/home/unitree/h1_nav2_ros2_ws')
namespace = '/h1_prepare_' + str(os.getpid())
command=[str(ws/'install/h1_2_policy_bridge/lib/h1_2_policy_bridge/h1_2_policy_node'),
         '--ros-args','-r','__ns:='+namespace,
         '-p','profile:=hardware','-p','network_interface:=eth0','-p','dds_domain:=0',
         '-p','joint_command_mode:='+args.joint_command_mode,
         '-p','model_path:='+str(ws/'models/h1_2_armhold/policy.onnx'),
         '-p','enable_hardware_output:=true','-p','operator_confirmed_debug_mode:='+str(args.confirm_debug_mode).lower(),
         '-p','operator_confirmed_exclusive_control:='+str(args.confirm_exclusive_control).lower(),'-p','max_policy_steps:='+str(args.policy_steps),
         '-p','max_policy_seconds:='+str(args.max_policy_seconds),
         '-p','arm_hold_knee_tolerance:='+str(args.arm_hold_knee_tolerance)]
if train_profile:
    command += ['-p', 'protection_profile:=train_20261106']
    for key in ('speed_limits_sdk', 'position_lower_sdk', 'position_upper_sdk', 'default_positions_sdk', 'joint_names_sdk'):
        command += ['-p', 'train_guard_' + key + ':=' + json.dumps(train_profile[key], separators=(',', ':'))]
    command += ['-p', 'train_guard_profile_hash:=' + train_profile['profile_hash']]
if args.continuous_walk_trial:
    command += ['-p', 'enable_navigation_commands:=true', '-p', 'continuous_policy_navigation:=true',
                '-p', 'cmd_vel_topic:='+namespace+'/private_cmd_vel',
                '-r', '/h1_2/policy_gate_alive:='+namespace+'/private_manual_gate_alive']
if not args.execute:
    import shlex
    print('DRY RUN: no process, ROS connection or physical command executed')
    print('Operator assertions: debug_mode=', args.confirm_debug_mode, 'exclusive_control=', args.confirm_exclusive_control, '; neither is machine verified')
    print('Joint command mode:', args.joint_command_mode, '; software torque clipping:', 'enabled' if args.joint_command_mode == 'torque' else 'not applied to native PD')
    print('FixStand position gate:', f'applied, limit={args.fixstand_max_error} rad' if args.fixstand_max_error is not None else 'not applied; error will be recorded only')
    print('Monitor:', shlex.join([str(ws/'tools/hardware-preflight/h1_prepare_monitor'), '<telemetry.json>', '<trace.csv>']))
    print('Node:', shlex.join(command))
    print('Services: status, authorize_output, passive observation, fix_stand (3.1 s)' + ('' if args.fixstand_only else ', arm_hold_stand (3.1 s)'))
    if train_profile:
        print('Training guard profile:', json.dumps(train_profile, sort_keys=True))
    if train_profile:
        print('Checks: fresh finite LowState and CRC, source-identified policy command progression; 50 Hz node training guard, cumulative joint peaks recorded only; Passive native damping and operator handoff on guard trip.')
    else:
        print('Checks: fresh finite LowState and CRC; source-identified policy command progression, valid state, cumulative speed <= 2 rad/s before pulse and upper body throughout; leg limit after pulse:', args.walk_leg_speed_limit, 'rad/s; native damping; revoke and verify no further writes.')
    print('Policy:', args.policy_steps, 'frames; monotonic deadline:', args.max_policy_seconds, 'seconds (0 disables); requires --confirm-supported-upright and onsite typed confirmation after 3 s in ArmHoldStand, sling ~60%, height ~0.95 m and straight legs, and both knee errors below configured tolerance; no retry. Tolerance:', args.arm_hold_knee_tolerance, 'rad')
    if args.continuous_walk_trial:
        print('Manual trial: motion', args.manual_motion, 'command', ManualPulse(args.manual_pulse_seconds, args.manual_motion).command, 'for', args.manual_pulse_seconds, 's, then zero in ArmHold until onsite REVOKE; no automatic release. Deployed source commit:', args.source_commit)
    else:
        print('Finally: position PD holds the last command until onsite RELEASE confirms external support for robot and both arms; then exit, verify damping and revoke. Deployed source commit:', args.source_commit)
    print('After revocation: observe telemetry for 0.5 s without motor commands; incomplete observation makes the result non-success.' if train_profile else 'After revocation: observe telemetry for 0.5 s without motor commands; any observed speed excess or incomplete observation makes the result non-success.')
    raise SystemExit(0)
os.environ.update(ROS_DOMAIN_ID='43', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
import rclpy
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
record = ws / 'third_party/h1_policy_runtime/integration-record' / ('acceptance-' + time.strftime('%Y%m%dT%H%M%S'))
record.mkdir(parents=True)
(record / 'COLCON_IGNORE').touch()
os.environ['ROS_LOG_DIR'] = str(record / 'ros-logs')
telemetry_path = record / 'telemetry.json'
writer_guid_path = record / 'policy_writer_guid'
summary = dict(result='not_started', policy_budget=args.policy_steps, max_policy_seconds=args.max_policy_seconds, policy_entered='unconfirmed',
               joint_command_mode=args.joint_command_mode, walk_leg_speed_limit=args.walk_leg_speed_limit,
               upper_body_speed_limit=2.0, preparation_speed_limit=2.0,
               position_gate_applied=args.fixstand_max_error is not None,
               fixstand_only=args.fixstand_only, fixstand_max_error_limit=args.fixstand_max_error,
               operator_confirmed_debug_mode=args.confirm_debug_mode,
               operator_confirmed_exclusive_control=args.confirm_exclusive_control,
               source_commit=args.source_commit, source_commit_evidence='operator_supplied_unconfirmed',
               onsite='unconfirmed', events=[], endpoints={}, observation_incomplete=False,
               max_observed_cumulative_peak_dq=0.0, first_observed_over_limit=None)
summary['protection_profile'] = args.protection_profile
summary['manual_pulse_seconds'] = args.manual_pulse_seconds
summary['manual_motion'] = args.manual_motion
if train_profile:
    summary['train_guard_profile'] = train_profile
    summary['train_guard_urdf_path'] = str(args.train_urdf)
process = monitor = node = None
trained_guard_handoff = False
authorized = False
blocked_wait_interrupted = False
logs = []
observation_phase = 'preflight'
pulse = ManualPulse(args.manual_pulse_seconds, args.manual_motion) if args.continuous_walk_trial else None
repeat_reader = None
repeat_busy = False
repeat_revoke = False
if pulse:
    summary['manual_command'] = pulse.command

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
signal.signal(signal.SIGHUP, interrupt)

def spawn(command, name):
    log=(record/(name+'.log')).open('w'); logs.append(log)
    # Both children start before rclpy.init(); do not use preexec after threads start.
    env = os.environ.copy()
    if name == 'node':
        env['H1_POLICY_WRITER_GUID_FILE'] = str(writer_guid_path)
    return subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                            preexec_fn=parent_death_preexec(), env=env)

def read_sample():
    value=json.loads(telemetry_path.read_text())
    if (time.time()-telemetry_path.stat().st_mtime > .15 or value['states'] < 2 or
        value['age'] > .1 or not value['state_finite'] or value['state_bad_crc'] != 0 or
        value['machine'] != 6 or value['pr'] != 0):
        raise RuntimeError('Telemetry invalid, stale, or bad CRC')
    if not all(math.isfinite(v) for v in value['q']+value['dq']):
        raise RuntimeError('Nonfinite joint telemetry')
    return value

def sample():
    try:
        if monitor is None or monitor.poll() is not None:
            raise RuntimeError('Monitor unavailable during observation')
        value = read_sample()
        if value.get('schema') != 3 or value.get('lifetime') != 'until_stopped':
            raise RuntimeError('Incompatible monitor schema or lifetime; rebuild prepare_monitor')
        peak = value['peak_dq']
        if train_profile:
            peaks = value.get('peak_dq_by_joint')
            if not isinstance(peaks, list) or len(peaks) != 27 or not all(isinstance(x, (int, float)) and math.isfinite(x) and x >= 0 for x in peaks):
                raise RuntimeError('Invalid 27-joint speed peak telemetry')
            violation = None
        else:
            violation = speed_violation(value, args.walk_leg_speed_limit,
                                        pulse is not None and pulse.started is not None)
        observed_at = time.monotonic()
        summary['max_observed_cumulative_peak_dq'] = max(summary['max_observed_cumulative_peak_dq'], peak)
        if 'peak_dq_by_joint' in value:
            summary['max_observed_cumulative_peak_dq_by_joint'] = value['peak_dq_by_joint']
        summary['last_observation'] = dict(phase=observation_phase, monotonic=observed_at,
                                           cumulative_peak_dq=peak)
        if violation is not None and summary['first_observed_over_limit'] is None:
            summary['first_observed_over_limit'] = dict(phase=observation_phase, monotonic=observed_at,
                                                        **violation)
        value['_speed_violation'] = violation
        return value
    except Exception:
        if authorized:
            summary['observation_incomplete'] = True
        raise

def check_policy_source(value):
    try:
        guid = writer_guid_path.read_text().strip()
    except OSError as exc:
        raise PolicySourcePending('Policy writer identity unavailable') from exc
    if not re.fullmatch(r'[0-9a-f]{32}', guid):
        raise RuntimeError('Invalid policy writer identity')
    if not value.get('policy_writer_guid'):
        raise PolicySourcePending('Policy writer not yet bound')
    if value['policy_writer_guid'] != guid:
        raise RuntimeError('Policy writer identity mismatched')
    if value.get('policy_commands') == 0 and value.get('policy_bad_crc') == 0:
        raise PolicySourcePending('Policy writer has no received frames')
    if value.get('policy_bad_crc') != 0 or value.get('policy_finite') is not True:
        raise RuntimeError('Policy LowCmd invalid or unavailable')
    return value

def check_speed(value):
    violation = value['_speed_violation']
    if violation is not None:
        raise RuntimeError('Joint speed exceeded limit: ' + json.dumps(violation))

def call(name):
    # Only the listed acceptance services are available; no release API is called.
    if args.fixstand_only and name in ('arm_hold_stand', 'arm_hold'):
        raise RuntimeError('Service forbidden in fixstand-only mode')
    if name not in ('status','authorize_output','passive','fix_stand','arm_hold_stand','arm_hold','release_supported','revoke_output'):
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
    if train_profile:
        if fields.get('protection_profile') != 'train_20261106' or fields.get('train_guard_profile_hash') != train_profile['profile_hash'] or fields.get('trained_guard_tripped') not in ('true', 'false'):
            raise RuntimeError('Training guard profile status missing or mismatch')
        if trained_guard_handoff and fields.get('mode') != 'Fault' and (fields['trained_guard_tripped'] != 'true' or fields.get('mode') != 'Passive' or fields.get('authorized') != 'true' or fields.get('publisher_present') != 'true'):
            raise RuntimeError('Training guard Passive handoff state lost')
        if fields['trained_guard_tripped'] == 'true' and fields.get('mode') == 'Passive' and fields.get('authorized') == 'true' and fields.get('publisher_present') == 'true' and not trained_guard_handoff:
            if pulse: pulse.cancel('training guard triggered')
            raise TrainedGuardStop(fields)
    if fields.get('joint_command_mode') != args.joint_command_mode:
        raise RuntimeError('Node joint command mode missing or does not match requested mode')
    if args.max_policy_seconds:
        try:
            reported_seconds = float(fields.get('max_policy_seconds', 'nan'))
        except ValueError:
            raise RuntimeError('Node policy time bound is invalid') from None
        if not math.isfinite(reported_seconds) or abs(reported_seconds - args.max_policy_seconds) > 1e-6:
            raise RuntimeError('Node policy time bound missing or does not match request')
    try:
        reported_tolerance = float(fields.get('arm_hold_knee_tolerance', 'nan'))
    except ValueError:
        raise RuntimeError('Node knee tolerance invalid') from None
    if not math.isfinite(reported_tolerance) or abs(reported_tolerance - args.arm_hold_knee_tolerance) > 1e-6:
        raise RuntimeError('Node knee tolerance missing or does not match request')
    expected_clamp = 'true' if args.joint_command_mode == 'torque' else 'false'
    if fields.get('software_torque_clamp') != expected_clamp:
        raise RuntimeError('Node torque clamp status missing or does not match requested mode')
    if args.joint_command_mode == 'position_pd' and (fields.get('position_pd_sample_torque_projection') != 'true' or fields.get('output_transition') not in ('none', 'entry', 'exit', 'hold') or fields.get('controlled_stop_supported') != 'false' or fields.get('exit_protocol') != 'hold_supported_release_v3' or fields.get('supported_release_required') != 'true' or fields.get('release_blocked') not in ('true', 'false')):
        raise RuntimeError('Node position PD projection or controlled transition status missing')
    if args.joint_command_mode == 'position_pd' and fields.get('mode') == 'Hold' and fields.get('output_transition') != 'hold':
        raise RuntimeError('Hold transition status missing')
    if args.fixstand_only and fields.get('mode') not in (('Passive', 'FixStand', 'Hold', 'Stopping', 'Fault') if expected == 'Fault' else ('Passive', 'FixStand', 'Hold', 'Stopping')):
        raise RuntimeError('Unapproved fixstand-only state')
    if fields.get('mode') == 'ArmHold' and not (args.policy_steps or args.continuous_walk_trial): raise RuntimeError('Unapproved policy state')
    if expected is not None and fields.get('mode') != expected:
        raise RuntimeError('Unexpected mode: '+res.message)
    return fields

def watch(seconds, expected, monitor_motion=True):
    global repeat_revoke
    def refresh_ready(fields):
        if fields.get('authorized') != 'true' or fields.get('publisher_present') != 'true':
            raise RuntimeError('Output authorization lost')
        if pulse and expected == 'ArmHold':
            pulse.authorize(time.monotonic(), fields.get('navigation_ready') == 'true')

    setup_started=time.monotonic()
    baseline=check_policy_source(sample())['policy_commands']
    last_status=time.monotonic()
    initial_fields=status(expected)
    refresh_ready(initial_fields)
    initial_writes=int(initial_fields['write_count'])
    started=time.monotonic()
    setup_elapsed=started-setup_started
    deadline=started+seconds
    fields=None
    while time.monotonic()<deadline:
        if repeat_reader and repeat_busy and repeat_reader.poll(False) == 'REVOKE':
            repeat_revoke = True
            pulse.cancel('operator requested REVOKE')
        if process.poll() is not None or monitor.poll() is not None:
            raise RuntimeError('A controlled test process exited')
        value=check_policy_source(sample())
        if pulse and pulse.fault: raise RuntimeError('Manual timer failed: '+pulse.fault)
        if time.monotonic()-started>.1 and (value['policy_commands']<=baseline or value['policy_command_age']>.1):
            raise RuntimeError('LowCmd stream not progressing or stale')
        if monitor_motion: check_speed(value)
        status_interval = .02 if pulse and expected == 'ArmHold' else .1
        if time.monotonic()-last_status>status_interval:
            last_status=time.monotonic()
            fields=status(expected)
            value=sample()
            check_speed(value)
            check_policy_source(value)
            refresh_ready(fields)
            if pulse and expected == 'ArmHold':
                if pulse.started is not None:
                    try:
                        policy_cmd = tuple(float(x) for x in fields.get('policy_cmd', '').split(','))
                    except ValueError:
                        policy_cmd = ()
                    if len(policy_cmd) == 3 and all(math.isfinite(x) for x in policy_cmd):
                        if time.monotonic() < pulse.started + pulse.duration and all(abs(a-b) < 1e-5 for a,b in zip(policy_cmd, pulse.command)):
                            summary['manual_policy_nonzero_observed'] = True
                        if time.monotonic() >= pulse.started + pulse.duration and all(abs(x) < 1e-5 for x in policy_cmd):
                            summary['manual_policy_zero_in_armhold_observed'] = True
        rclpy.spin_once(node,timeout_sec=.01)
    final=check_policy_source(sample())
    if monitor_motion: check_speed(final)
    final_fields=status(expected)
    refresh_ready(final_fields)
    minimum=max(2,int(seconds*25))
    received=final['policy_commands']-baseline
    written=int(final_fields['write_count'])-initial_writes
    if received <= 0 or written <= 0 or final['policy_command_age'] > .1:
        actual=time.monotonic()-started
        summary['failed_observation_window']=dict(mode=expected, requested_seconds=seconds, actual_seconds=actual, setup_elapsed=setup_elapsed, received=received, written=written, command_age=final['policy_command_age'], minimum=minimum)
        raise RuntimeError(f"Insufficient fresh command frames in observation window: received={received} written={written} minimum={minimum} command_age={final['policy_command_age']:.3f}s")
    summary.setdefault('windows',[]).append(dict(evidence='observedDDS_and_node_status',mode=expected,seconds=time.monotonic()-started,
                                               received=received,written=written,minimum=minimum,under_expected_rate=received < minimum or written < minimum))
    return final

def wait_for_mode(target, timeout, allowed):
    deadline = time.monotonic() + timeout
    while True:
        value = sample()
        if process.poll() is not None or monitor.poll() is not None:
            raise RuntimeError('A controlled test process exited while stopping')
        check_speed(value)
        if check_policy_source(value)['policy_command_age'] > .1:
            raise RuntimeError('LowCmd stream stale while stopping')
        fields = status()
        if fields.get('authorized') != 'true' or fields.get('publisher_present') != 'true':
            raise RuntimeError('Output authorization lost while stopping')
        if args.joint_command_mode == 'position_pd' and fields.get('mode') == 'Stopping' and fields.get('output_transition') != 'exit':
            raise RuntimeError('Controlled exit status missing while stopping')
        if fields.get('mode') == target:
            if args.joint_command_mode == 'position_pd' and fields.get('output_transition') != ('hold' if target == 'Hold' else 'none'):
                raise RuntimeError('Unexpected output transition in ' + target)
            return value
        if fields.get('mode') not in allowed:
            raise RuntimeError('Unexpected mode while stopping: ' + fields.get('mode', 'missing'))
        if time.monotonic() > deadline:
            raise RuntimeError('Timed out waiting for ' + target)
        rclpy.spin_once(node, timeout_sec=.01)

def wait_for_supported_release():
    print('Hold active. Body sling is not arm support. Support both arms, bearing the right forearm/elbow weight; contact or steadying alone is insufficient. Then type RELEASE:', flush=True)
    while True:
        watch(.2, 'Hold')
        if select.select([sys.stdin], [], [], 0)[0]:
            answer = sys.stdin.readline()
            if answer == '':
                raise RuntimeError('Terminal EOF while awaiting supported release')
            if answer.rstrip('\n') == 'RELEASE':
                summary['supported_release'] = 'operator_asserted_external_support_robot_and_both_arms'
                res = call('release_supported')
                if not res.success:
                    fields = status('Hold')
                    if fields.get('release_blocked') == 'true':
                        wait_for_blocked_release_revoke()
                        return 'blocked'
                    raise RuntimeError('Supported release rejected: ' + res.message)
                return 'requested'
            print('Type RELEASE after support is in place.', flush=True)

def wait_for_blocked_release_revoke():
    """Keep monitoring in Hold until the operator confirms safe handoff and requests revoke."""
    global blocked_wait_interrupted
    summary['result'] = 'needs_operator_check'
    summary['release_blocked'] = True
    summary['release_blocked_action'] = 'held_for_operator_physical_safety_handoff'
    print('Release is blocked; output remains authorized in Hold. Hold preserves the last output; kp may already be zero, so position support is not guaranteed. After physically securing the robot, type REVOKE to revoke output:', flush=True)
    try:
        while True:
            watch(.2, 'Hold')
            fields = status('Hold')
            if fields.get('release_blocked') != 'true':
                raise RuntimeError('Release block latch was lost while awaiting operator handoff')
            if select.select([sys.stdin], [], [], 0)[0]:
                answer = sys.stdin.readline()
                if answer == '':
                    raise RuntimeError('Terminal EOF while awaiting physical safety handoff')
                if answer.rstrip('\n') == 'REVOKE':
                    summary['release_blocked_action'] = 'operator_confirmed_physical_safety_handoff_and_requested_revoke'
                    summary['release_blocked_operator_confirmation'] = 'REVOKE'
                    return
                print('After physically securing the robot, type REVOKE to revoke output.', flush=True)
    except BaseException:
        blocked_wait_interrupted = True
        raise

def wait_for_release_exit():
    """Wait up to 12 s for v3 release; blocked release stays monitored in Hold."""
    deadline = time.monotonic() + 12.0
    while True:
        value = sample()
        if process.poll() is not None or monitor.poll() is not None:
            raise RuntimeError('A controlled test process exited while stopping')
        check_speed(value)
        if check_policy_source(value)['policy_command_age'] > .1:
            raise RuntimeError('LowCmd stream stale while stopping')
        fields = status()
        if fields.get('authorized') != 'true' or fields.get('publisher_present') != 'true':
            raise RuntimeError('Output authorization lost while stopping')
        if fields.get('release_blocked') == 'true':
            if fields.get('mode') != 'Hold' or fields.get('output_transition') != 'hold':
                raise RuntimeError('Blocked release did not remain in Hold')
            wait_for_blocked_release_revoke()
            return 'blocked'
        if fields.get('mode') == 'Passive':
            if fields.get('output_transition') != 'none':
                raise RuntimeError('Unexpected output transition after release')
            return 'released'
        if fields.get('mode') not in ('Hold', 'Stopping'):
            raise RuntimeError('Unexpected mode while waiting for release: ' + fields.get('mode', 'missing'))
        if fields.get('mode') == 'Stopping' and fields.get('output_transition') != 'exit':
            raise RuntimeError('Controlled exit status missing while stopping')
        if time.monotonic() > deadline:
            raise RuntimeError('Timed out waiting for Passive')
        rclpy.spin_once(node, timeout_sec=.01)


def wait_for_entry_complete(mode):
    if args.joint_command_mode != 'position_pd':
        return
    deadline = time.monotonic() + .5
    while True:
        fields = status(mode)
        if fields['output_transition'] == 'none':
            return
        if fields['output_transition'] != 'entry' or time.monotonic() > deadline:
            raise RuntimeError('Entry transition did not complete')
        watch(.1, mode)

def manual_handoff(mode, reason):
    global pulse, repeat_reader, repeat_busy, repeat_revoke
    initial_pulse = pulse
    if reason != 'pulse window elapsed; zero command':
        pulse.cancel(reason)
    summary['manual_cancel_reason'] = pulse.cancel_reason
    summary['manual_pulse_state'] = ('nonzero_published_zero_requested' if summary.get('manual_nonzero_published') else 'nonzero_not_published')
    if pulse.cancel_reason:
        print('Manual velocity canceled:', pulse.cancel_reason, '; pulse:', summary['manual_pulse_state'], '; holding', mode, 'until onsite REVOKE.', flush=True)
    else:
        print('Pulse window elapsed; zero command continues in', mode, 'until onsite REVOKE.', flush=True)
    if args.allow_repeat_walk and mode == 'ArmHold' and not pulse.cancel_reason:
        repeat_reader = RepeatInput(sys.stdin.fileno())
        try:
            if repeat_reader.poll(False) == 'REVOKE': repeat_revoke = True
            repeat_reader.discard_partial()
            while not repeat_revoke:
                print('ArmHold idle: type WALK for one 3 s forward pulse, or REVOKE after securing support.', flush=True)
                answer = None
                while answer is None:
                    watch(.2, mode)
                    answer = repeat_reader.poll(True)
                if answer == 'REVOKE':
                    repeat_revoke = True
                    break
                old_nonzero = summary.get('manual_policy_nonzero_observed')
                old_zero = summary.get('manual_policy_zero_in_armhold_observed')
                item = dict(requested=time.monotonic(), command=[.12, 0., 0.], duration=3.)
                summary.setdefault('repeat_walks', []).append(item)
                summary['manual_policy_nonzero_observed'] = False
                summary['manual_policy_zero_in_armhold_observed'] = False
                try:
                    fields = status('ArmHold')
                    check_speed(sample()); check_policy_source(sample())
                    if repeat_reader.poll(False) == 'REVOKE':
                        repeat_revoke = True
                        item['cancel_reason'] = 'operator requested REVOKE'
                        break
                    repeat_reader.discard_partial()
                    pulse = ManualPulse(3., 'forward')
                    pulse.authorize(time.monotonic(), fields.get('navigation_ready') == 'true' and fields.get('authorized') == 'true' and fields.get('publisher_present') == 'true')
                    if not pulse.ready:
                        item['cancel_reason'] = 'readiness unavailable'
                        continue
                    pulse.start(time.monotonic()); item['start'] = pulse.started
                    repeat_busy = True
                    watch(pulse.duration + .1, 'ArmHold')
                    if not summary['manual_policy_zero_in_armhold_observed']: watch(.4, 'ArmHold')
                except Exception as exc:
                    item['error'] = str(exc)
                    raise
                finally:
                    repeat_busy = False
                    item.update(end=time.monotonic(), nonzero_observed=summary.get('manual_policy_nonzero_observed', False), zero_observed=summary.get('manual_policy_zero_in_armhold_observed', False))
                    item.setdefault('cancel_reason', pulse.cancel_reason)
                    summary['manual_policy_nonzero_observed'] = old_nonzero
                    summary['manual_policy_zero_in_armhold_observed'] = old_zero
                if repeat_reader.poll(False) == 'REVOKE': repeat_revoke = True
                repeat_reader.discard_partial()
                if pulse.cancel_reason: break
        finally:
            repeat_reader.close(); repeat_reader = None
        if not repeat_revoke and pulse.cancel_reason:
            print('Repeat canceled; waiting for REVOKE.', flush=True)
        pulse = initial_pulse
    if not repeat_revoke:
        while True:
            watch(.2, mode)
            if select.select([sys.stdin], [], [], 0)[0]:
                answer = sys.stdin.readline()
                if answer == '': raise RuntimeError('Terminal EOF awaiting handoff')
                if answer.strip() == 'REVOKE': break
                print('After physically securing robot and both arms, type REVOKE.', flush=True)
    summary['manual_handoff'] = 'operator_confirmed_REVOKE'

fix=[0,-.2,0,.5,-.3,0,0,-.2,0,.5,-.3,0,0,.28,0,0,.52,0,0,0,.28,0,0,.52,0,0,0]
stand=[.027,-.45,0,.9,-.45,0,-.027,-.45,0,.9,-.475,0,0,
       .270,.001,0,.483,-.030,-.003,-.002,-.239,-.192,.941,-.633,-.076,.004,.026]
if train_profile is not None:
    stand = train_profile['default_positions_sdk']
try:
    monitor=spawn([str(ws/'tools/hardware-preflight/h1_prepare_monitor'),str(telemetry_path),str(record/'trace.csv'),str(writer_guid_path)],'monitor')
    deadline=time.monotonic()+5
    while not telemetry_path.exists():
        if monitor.poll() is not None or time.monotonic()>deadline: raise RuntimeError('Monitor unavailable')
        time.sleep(.02)
    time.sleep(.6)
    initial=sample()
    if initial['peak_dq'] > .5: raise RuntimeError('Robot not stationary before preparation')
    summary['initial']=initial
    process=spawn(command,'node')
    rclpy.init(args=[]); node=rclpy.create_node('preparation_runner',namespace=namespace)
    if pulse:
        gate_pub=node.create_publisher(Bool, namespace+'/private_manual_gate_alive', 10)
        cmd_pub=node.create_publisher(Twist, namespace+'/private_cmd_vel', 10)
        def manual_tick():
            try:
                value=sample()
                check_speed(value)
                velocity=pulse.output(time.monotonic(), True)
                if velocity is not None:
                    msg=Bool(); msg.data=True; gate_pub.publish(msg)
                    cmd=Twist(); cmd.linear.x, cmd.linear.y, cmd.angular.z = velocity; cmd_pub.publish(cmd)
                    if any(velocity): summary['manual_nonzero_published'] = True
            except Exception as exc:
                pulse.fault=str(exc)
        node.create_timer(.02, manual_tick)
    time.sleep(.6)
    fields=status('Passive')
    if fields['authorized']!='false' or fields['publisher_present']!='false' or fields['write_count']!='0':
        raise RuntimeError('Unexpected startup output')
    sample()
    # Authorization may succeed even if its response is lost; always attempt cleanup.
    authorized=True
    res=call('authorize_output')
    if not res.success: raise RuntimeError('Authorization rejected: '+res.message)
    # Writer creation happens during authorization; wait for the monitor to bind
    # the exact writer GUID before using any LowCmd as policy evidence.
    deadline = time.monotonic() + 2.0
    while True:
        try:
            check_policy_source(sample())
            break
        except PolicySourcePending:
            if time.monotonic() >= deadline:
                raise
            time.sleep(.02)
    observation_phase='initial_passive'
    value=watch(.25,'Passive')
    if not value['policy_native_damping']: raise RuntimeError('Passive command is not native damping')
    print('Passive output verified; starting approved FixStand preparation',flush=True)
    if not call('fix_stand').success: raise RuntimeError('FixStand rejected')
    observation_phase='fix_stand'
    value=watch(3.1,'FixStand')
    wait_for_entry_complete('FixStand')
    value=sample()
    summary['endpoints']['FixStand']=value
    error=fixstand_error(value['q'], fix, args.fixstand_max_error)
    summary['fixstand_max_error']=error
    summary['fixstand_worst_joint_sdk']=max(range(len(fix)), key=lambda i: abs(value['q'][i]-fix[i]))
    if args.fixstand_max_error is not None and error > args.fixstand_max_error:
        raise PreparationRejected(f'FixStand position error {error:.6f} exceeds configured limit {args.fixstand_max_error:.6f}')
    if args.fixstand_only:
        summary['result']='fixstand_only_completed'
    else:
        print('FixStand finished; starting approved ArmHoldStand preparation',flush=True)
        if not call('arm_hold_stand').success: raise RuntimeError('ArmHoldStand rejected')
        observation_phase='arm_hold_stand'
        value=watch(3.1,'ArmHoldStand')
        wait_for_entry_complete('ArmHoldStand')
        value=sample()
        summary['endpoints']['ArmHoldStand']=value
        summary['armholdstand_max_error']=max(abs(a-b) for a,b in zip(value['q'],stand))
        summary['knee_gate'] = dict(left_q=value['q'][3], right_q=value['q'][9], target=.9, tolerance=args.arm_hold_knee_tolerance)
        print('Knee gate:', summary['knee_gate'], flush=True)
        summary['knees_within_policy_position_gate']=all(abs(value['q'][i]-.9)<args.arm_hold_knee_tolerance for i in (3,9))
        summary['result']='preparation_executed_no_policy'
        if args.policy_steps or args.continuous_walk_trial:
            if not args.continuous_walk_trial and not summary['knees_within_policy_position_gate']:
                raise PreparationRejected('Knee position gate failed; no policy')
            if args.onsite_confirmed_before_start:
                summary['onsite'] = 'operator_confirmed_before_start'
            else:
                print('Onsite: verify sling ~60% support, height ~0.95 m and legs straight. Type CONFIRM to continue:', flush=True)
                confirmation_deadline=time.monotonic()+args.onsite_confirm_timeout
                answer=None
                while time.monotonic()<confirmation_deadline:
                    watch(.2,'ArmHoldStand')
                    if select.select([sys.stdin],[],[],0)[0]:
                        answer=sys.stdin.readline().strip()
                        break
                if answer != 'CONFIRM':
                    raise PreparationRejected('Onsite confirmation withheld or timed out')
                summary['onsite'] = 'operator_confirmed_sling_60_percent_height_0.95m_straight_legs'
            # Recheck telemetry and posture after the operator confirmation, before requesting policy.
            value=watch(.25,'ArmHoldStand')
            summary['knee_gate_after_confirmation'] = dict(left_q=value['q'][3], right_q=value['q'][9], target=.9, tolerance=args.arm_hold_knee_tolerance)
            print('Knee gate after confirmation:', summary['knee_gate_after_confirmation'], flush=True)
            if not args.continuous_walk_trial and not all(abs(value['q'][i]-.9)<args.arm_hold_knee_tolerance for i in (3,9)):
                raise PreparationRejected('Knee position gate failed after onsite confirmation')
            arm_response = call('arm_hold')
            if not arm_response.success:
                if not args.continuous_walk_trial:
                    raise RuntimeError('ArmHold rejected: ' + arm_response.message)
                fields = status('ArmHoldStand')
                if fields.get('authorized') != 'true' or fields.get('publisher_present') != 'true':
                    raise RuntimeError('ArmHold rejected with authorization or publisher lost')
                watch(.2, 'ArmHoldStand')
                manual_handoff('ArmHoldStand', 'ArmHold rejected: ' + arm_response.message)
                summary['result'] = 'manual_trial_canceled_no_policy'
            else:
                summary['policy_entered']='service_accepted_unconfirmed_by_dds'
                observation_phase='arm_hold'
                if args.continuous_walk_trial:
                    # Allow the entry blend to finish while issuing only zero commands.
                    watch(3.1, 'ArmHold')
                    wait_for_entry_complete('ArmHold')
                    ready_deadline=time.monotonic()+2.0
                    while not pulse.ready and time.monotonic() <= ready_deadline:
                        watch(.2, 'ArmHold')
                    summary['manual_policy_nonzero_observed'] = False
                    summary['manual_policy_zero_in_armhold_observed'] = False
                    if not pulse.ready:
                        manual_handoff('ArmHold', 'navigation readiness timeout before pulse')
                        summary['result'] = 'manual_trial_canceled_no_pulse'
                    else:
                        check_speed(sample())
                        fields = status('ArmHold')
                        pulse.authorize(time.monotonic(), fields.get('navigation_ready') == 'true')
                        if not pulse.ready:
                            manual_handoff('ArmHold', 'readiness lost before pulse')
                            summary['result'] = 'manual_trial_canceled_no_pulse'
                        else:
                            pulse.start(time.monotonic())
                            watch(pulse.duration + .1, 'ArmHold')
                            if not summary['manual_policy_zero_in_armhold_observed']:
                                watch(.4, 'ArmHold')
                            print('Bridge policy_cmd observed: nonzero=', summary['manual_policy_nonzero_observed'], 'zero in ArmHold=', summary['manual_policy_zero_in_armhold_observed'], '; physical motion unconfirmed', flush=True)
                            manual_handoff('ArmHold', pulse.cancel_reason or 'pulse window elapsed; zero command')
                            summary['result'] = ('manual_trial_completed_physical_behavior_unconfirmed'
                                                 if not pulse.cancel_reason
                                                 else 'manual_trial_canceled_partial_or_unconfirmed')
                            if not pulse.cancel_reason and not (summary['manual_policy_nonzero_observed'] and summary['manual_policy_zero_in_armhold_observed']):
                                summary['result'] = 'manual_trial_policy_command_unconfirmed'
                else:
                    target = 'Hold' if args.joint_command_mode == 'position_pd' else 'Passive'
                    wait_for_mode(target, 4.5, ('ArmHold',))
                    value=watch(.3,target)
                    budget_log = ('policy budget exhausted; holding last output; external support required'
                                  if args.joint_command_mode == 'position_pd' else
                                  'policy budget exhausted; exiting control; motor behavior not verified')
                    node_log = (record/'node.log').read_text(errors='replace')
                    time_log = 'policy time budget exhausted; holding last output; external support required'
                    if budget_log not in node_log and not (args.max_policy_seconds and time_log in node_log):
                        raise RuntimeError('Policy budget or time cutoff not confirmed in node log')
                    summary['policy_budget_evidence']='budget or time cutoff observed; this does not prove completed entry blend or physical execution'
                    summary['policy_after_budget_observed_dds']=value
                    summary['result']='policy_budget_completed_unconfirmed_physical_behavior'
except TrainedGuardStop as exc:
    trained_guard_handoff = True
    summary.update(result='needs_operator_check', trained_guard_stop=exc.fields,
                   trained_guard_handoff='awaiting_operator_REVOKE',
                   trained_guard_command_evidence='Passive DDS output; not proof of firmware remote-control path',
                   manual_zero='zero requested; policy_cmd zero not independently confirmed')
    print('Training guard entered Passive. Secure the robot and type REVOKE for physical handoff.', flush=True)
    try:
        while True:
            value = watch(.2, 'Passive')
            if not value.get('policy_native_damping'):
                raise RuntimeError('Native damping not observed during training guard handoff')
            summary['trained_guard_native_damping_observed'] = True
            if select.select([sys.stdin], [], [], 0)[0]:
                answer = sys.stdin.readline()
                if answer == '': raise RuntimeError('Terminal EOF awaiting guard handoff')
                if answer.strip() == 'REVOKE':
                    summary['trained_guard_handoff'] = 'operator_confirmed_REVOKE'
                    break
    except Exception as handoff_exc:
        summary.update(result='aborted', error=str(handoff_exc), trained_guard_handoff='fault_during_handoff')
except Exception as exc:
    summary.update(result='aborted',error=str(exc))
    summary['normal_cleanup_allowed'] = isinstance(exc, PreparationRejected)
    print('PREPARATION ABORT:',exc,'; cleanup will attempt revocation if needed. Use physical emergency stop for abnormal motion.',flush=True)
finally:
    # A second terminal interrupt must not interrupt the already-running
    # damping/revocation/owned-process cleanup sequence.
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    summary['preparation_result'] = summary['result']
    observation_phase = 'cleanup_passive'
    if authorized and process is not None and process.poll() is None and node is not None:
        if not args.continuous_walk_trial and (summary['preparation_result'] != 'aborted' or summary.get('normal_cleanup_allowed', False)):
            try:
                signal.signal(signal.SIGTERM, interrupt)
                signal.signal(signal.SIGINT, interrupt)
                signal.signal(signal.SIGHUP, interrupt)
                current = status()['mode']
                if current not in ('Passive', 'Hold'):
                    res=call('passive')
                    if not res.success: raise RuntimeError('Passive rejected: '+res.message)
                    target = 'Hold' if args.joint_command_mode == 'position_pd' else 'Passive'
                    wait_for_mode(target, 4.5, ('FixStand', 'ArmHoldStand', 'ArmHold'))
                    current = target
                if current == 'Hold':
                    if args.joint_command_mode != 'position_pd': raise RuntimeError('Unexpected Hold')
                    observation_phase = 'cleanup_hold'
                    release_request = wait_for_supported_release()
                    if release_request != 'blocked':
                        observation_phase = 'cleanup_exit'
                        release_result = wait_for_release_exit()
                        if release_result == 'blocked':
                            current = 'Hold'
                        elif release_result == 'released':
                            current = 'Passive'
                if current != 'Hold':
                    final=watch(.3,'Passive')
                    summary['final_damping']=final
                    if not final['policy_native_damping']: raise RuntimeError('Native damping not observed at exit')
            except Exception as exc:
                summary.update(result='needs_operator_check',exit_error=str(exc))
                print('Unable to verify damping; onsite operator must use the physical emergency stop if needed:',exc,flush=True)
        elif summary['preparation_result'] == 'aborted':
            summary['result'] = 'needs_operator_check'
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        try:
            if summary.get('release_blocked') and summary.get('release_blocked_operator_confirmation') != 'REVOKE' and not blocked_wait_interrupted:
                raise RuntimeError('Blocked release is still awaiting explicit operator REVOKE')
            res=call('revoke_output')
            if not res.success: raise RuntimeError('Revoke request rejected: '+res.message)
            fields=status('Fault')
            if fields.get('authorized')!='false' or fields.get('publisher_present')!='false':
                raise RuntimeError('Output revocation not confirmed')
            count=int(fields['write_count'])
            observation_phase = 'revoke_observation_unverified'
            observation_start = time.monotonic()
            try:
                while time.monotonic() - observation_start < .5:
                    sample()
                    time.sleep(.01)
            except Exception as exc:
                summary['observation_incomplete'] = True
                summary['post_revoke_observation_error'] = str(exc)
            finally:
                summary['post_revoke_observation_window'] = dict(start_monotonic=observation_start,
                                                                 end_monotonic=time.monotonic())
            if int(status('Fault')['write_count'])!=count:
                raise RuntimeError('Writes continued after revocation')
            summary['revocation_confirmed']=True
        except Exception as exc:
            summary.update(result='needs_operator_check',revoke_error=str(exc))
    elif authorized:
        summary.update(result='needs_operator_check', cleanup_status='unknown_process_exited_or_unavailable')
        print('Cleanup could not be verified; onsite operator check required.', flush=True)
    if authorized:
        observation_phase = 'post_revoke' if summary.get('revocation_confirmed') else 'cleanup_unverified'
        try:
            final_observation_start = time.monotonic()
            sample()
            summary['final_observation_window'] = dict(start_monotonic=final_observation_start,
                                                        end_monotonic=time.monotonic())
        except Exception as exc:
            summary['observation_incomplete'] = True
            summary['final_observation_error'] = str(exc)
    if summary['first_observed_over_limit'] is not None or summary['observation_incomplete']:
        summary['result'] = 'needs_operator_check'
        print('Observation exceeded the speed limit or was incomplete; onsite operator check required.', flush=True)
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
    brief_keys = ('result', 'preparation_result', 'policy_budget', 'max_policy_seconds', 'release_blocked', 'revocation_confirmed', 'node_returncode', 'monitor_returncode', 'record')
    console_summary = {key: summary[key] for key in brief_keys if key in summary} if args.brief_summary else summary
    print(json.dumps(console_summary,indent=2,allow_nan=False),flush=True)
raise SystemExit(0 if summary['result'] in ('fixstand_only_completed','preparation_executed_no_policy','policy_budget_completed_unconfirmed_physical_behavior','manual_trial_completed_physical_behavior_unconfirmed') else 1)
