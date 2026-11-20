import sys
import time

import pytest

from manual_test_web import TrialController, validate_trial


def valid_payload(**changes):
    payload = {
        'motion': 'forward', 'speed': .12, 'duration': 1.0,
        'confirmations': {'debug_mode': True, 'exclusive_control': True, 'supported_upright': True},
    }
    payload.update(changes)
    return payload


@pytest.mark.parametrize(('motion', 'speed', 'unit'), [
    ('forward', .12, 'm/s'), ('backward', .1, 'm/s'),
    ('turn_left', .1, 'rad/s'), ('turn_right', .05, 'rad/s'),
])
def test_trial_limits_by_direction(motion, speed, unit):
    selected, actual_speed, duration = validate_trial(valid_payload(motion=motion, speed=speed))
    assert (selected, actual_speed, duration) == (motion, speed, 1.0)
    assert unit == ('rad/s' if selected.startswith('turn_') else 'm/s')


@pytest.mark.parametrize('payload', [
    valid_payload(speed=.1201), valid_payload(motion='turn_left', speed=.1001),
    valid_payload(motion='turn_right', speed=float('nan')),
    valid_payload(duration=3.1), valid_payload(duration=.49),
    valid_payload(confirmations={'debug_mode': True}),
    valid_payload(motion='diagonal'),
])
def test_invalid_or_unconfirmed_trial_rejected(payload):
    with pytest.raises(ValueError):
        validate_trial(payload)


def test_controller_requires_runner_prompts_before_confirm_and_revoke(tmp_path):
    runner = tmp_path / 'fake_runner.py'
    runner.write_text(
        "import sys\n"
        "print('Manual trial will hold ArmHold until onsite REVOKE.', flush=True)\n"
        "print('Type CONFIRM to continue:', flush=True)\n"
        "assert sys.stdin.readline().strip() == 'CONFIRM'\n"
        "print('MANUAL_WEB_EVENT {\"state\": \"awaiting_revoke\"}', flush=True)\n"
        "assert sys.stdin.readline().strip() == 'REVOKE'\n"
    )
    controller = TrialController(tmp_path, runner)
    controller.start(valid_payload())
    deadline = time.monotonic() + 3
    while controller.status()['state'] != 'awaiting_confirmation' and time.monotonic() < deadline:
        time.sleep(.02)
    assert controller.status()['state'] == 'awaiting_confirmation'
    with pytest.raises(ValueError):
        controller.revoke()
    controller.confirm()
    while controller.status()['state'] != 'awaiting_revoke' and time.monotonic() < deadline:
        time.sleep(.02)
    assert controller.status()['state'] == 'awaiting_revoke'
    controller.revoke()
    while controller.status()['active'] and time.monotonic() < deadline:
        time.sleep(.02)
    assert controller.status()['state'] == 'complete'
    assert controller.status()['exit_code'] == 0


def test_web_binds_loopback_and_exposes_controls():
    from manual_test_web import HTML, main

    assert "default='127.0.0.1'" in __import__('inspect').getsource(main)
    for expected in ('前进', '后退', '左转', '右转', 'rad/s', 'm/s', '/api/confirm', '/api/revoke'):
        assert expected in HTML


def test_repeat_trials_keep_same_runner_and_reject_busy_or_failed_requests(tmp_path):
    runner = tmp_path / 'fake_repeat.py'
    runner.write_text(
        "import sys,json\n"
        "print('MANUAL_WEB_EVENT '+json.dumps({'state':'ready_next'}),flush=True)\n"
        "for expected in [('turn_right',.05,1.5),('forward',.08,.5)]:\n"
        " line=sys.stdin.readline().strip(); assert line.startswith('TRIAL ')\n"
        " d=json.loads(line[6:]); assert (d['motion'],d['speed'],d['duration'])==expected\n"
        " print('MANUAL_WEB_EVENT '+json.dumps({'state':'running'}),flush=True)\n"
        " print('MANUAL_WEB_EVENT '+json.dumps({'state':'ready_next'}),flush=True)\n"
        "assert sys.stdin.readline().strip()=='REVOKE'\n"
    )
    c=TrialController(tmp_path,runner)
    c.start(valid_payload())
    original_process=c.process
    def wait_state(state):
        deadline=time.monotonic()+3
        while c.status()['state']!=state and time.monotonic()<deadline:
            time.sleep(.02)
        assert c.status()['state']==state, c.status()
    wait_state('ready_next')
    with pytest.raises(ValueError):
        c.next_trial(valid_payload(speed=.2))
    for payload in (valid_payload(motion='turn_right',speed=.05,duration=1.5),
                    valid_payload(speed=.08,duration=.5)):
        c.next_trial(payload)
        assert c.process is original_process
        with pytest.raises(ValueError):
            c.next_trial(payload)
        wait_state('ready_next')
    c.revoke()
    wait_state('complete')
    with pytest.raises(ValueError):
        c.next_trial(valid_payload())


def test_output_parser_uses_current_complete_events_not_historical_log():
    c=TrialController()
    c.state='starting'
    c._output_line('Type CONFIRM to continue:')
    assert c.state=='awaiting_confirmation'
    c.state='confirmed'
    c._output_line('MANUAL_WEB_EVENT {"state":"ready_next"}')
    assert c.state=='ready_next'
    c._output_line('MANUAL_WEB_EVENT {"state":"running"}')
    assert c.state=='running'
    c._output_line('unrelated log')
    assert c.state=='running'
    c._output_line('Training guard entered Passive. Secure the robot')
    assert c.state=='awaiting_revoke'
