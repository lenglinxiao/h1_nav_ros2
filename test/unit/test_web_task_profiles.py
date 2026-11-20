"""Exercise task persistence and launch guards without importing ROS or starting processes."""
import ast
import copy
import json
import os
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

SOURCE = Path(__file__).resolve().parents[2] / 'src/h1_2_nav_tools/h1_2_nav_tools/h1_2_nav_web_gui.py'


@pytest.fixture
def module(tmp_path):
    tree = ast.parse(SOURCE.read_text())
    names = {'normalize_memory_route', 'normalize_task_profiles', 'task_execution_snapshot',
             'normalize_ssh_target', 'normalize_voice_workspace', 'WebGuiApp'}
    subset = ast.Module(body=[node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names], type_ignores=[])
    namespace = dict(Path=Path, json=json, os=os, threading=threading, time=time, HEARTBEAT_TIMEOUT_SEC=2.0, CONFIG_PATH=tmp_path/'config.json', CONFIG_WORKFLOW_VERSION=2)
    exec(compile(subset, str(SOURCE), 'exec'), namespace)
    return SimpleNamespace(**namespace)


@pytest.fixture
def config():
    return {'run_mode':'navigation', 'task_profiles':[
        {'id':'one', 'name':'迎宾', 'steps':[{'point':'B','gesture':'wave_hand','turn':'ccw'}, {'point':'A','gesture':'','turn':'cw'}]},
        {'id':'two', 'name':'巡检', 'steps':[{'point':'C','gesture':'shake_hand','turn':''}]},
    ], 'selected_task_id':'one', 'memory_route':'stale', 'gesture_route':'stale'}


def test_legacy_migration_retains_order_and_actions(module):
    config = {'memory_route':'START,B,A,B', 'gesture_route':'B:wave_hand,A:reach_hand+retract_hand'}
    selected = module.normalize_task_profiles(config)
    assert selected['name'] == '默认任务'
    assert config['memory_route'] == 'B,A'
    assert selected['steps'] == [{'point':'B','gesture':'wave_hand','turn':'ccw'}, {'point':'A','gesture':'reach_hand+retract_hand','turn':'cw'}]
    assert module.normalize_task_profiles(config) == selected


def test_selection_derives_route_without_mutating_other_task(module, config):
    original = copy.deepcopy(config['task_profiles'][0])
    config['selected_task_id'] = 'two'
    module.normalize_task_profiles(config)
    assert config['memory_route'] == 'C'
    assert config['gesture_route'] == 'C:shake_hand'
    assert config['task_profiles'][0] == original
    config['selected_task_id'] = 'one'
    module.normalize_task_profiles(config)
    assert config['memory_route'] == 'B,A'


@pytest.mark.parametrize('change', [
    lambda c: c.update(selected_task_id='missing'),
    lambda c: c.update(task_profiles=[]),
    lambda c: c['task_profiles'][1].update(id='one'),
    lambda c: c['task_profiles'][1].update(name='迎宾'),
    lambda c: c['task_profiles'][0].update(name=' '),
    lambda c: c['task_profiles'][0]['steps'].append({'point':'B'}),
    lambda c: c['task_profiles'][0]['steps'].append({'point':'START'}),
    lambda c: c['task_profiles'][0]['steps'].append({'point':'A,B'}),
    lambda c: c['task_profiles'][0]['steps'][0].update(turn='left'),
    lambda c: c['task_profiles'][0]['steps'][0].update(turn=None),
])
def test_invalid_profiles_rejected(module, config, change):
    change(config)
    with pytest.raises(ValueError):
        module.normalize_task_profiles(config)


def app_without_ros(module, workspace_root=None):
    app = module.WebGuiApp.__new__(module.WebGuiApp)
    app.workspace_root = workspace_root or Path.cwd()
    app.task_lock = threading.RLock()
    app.heartbeat_lock = threading.Lock()
    app.last_heartbeat = time.monotonic()
    app.heartbeat_timeout_sec = 2.0
    app.heartbeat_expired = False
    app.pc4_profile = False
    app.loaded_task = None
    app.processes = Mock()
    app.processes.status.return_value = {'navigation':{'running':True}}
    app.ros = Mock()
    app.ros.policy_route_status.return_value = {'success': True, 'state': 'idle'}
    app.logs = Mock()
    return app


def policy_confirmations():
    return {
        'confirm_debug_mode': True,
        'confirm_exclusive_control': True,
        'confirm_lowcmd_handoff': True,
        'confirm_robot_supported': True,
    }


def policy_status(mode='Passive', authorized=False, publisher_present=False):
    return {
        'available': True, 'profile': 'hardware', 'joint_command_mode': 'position_pd',
        'output_enabled': True, 'debug_confirmed': True, 'exclusive_confirmed': True,
        'navigation_enabled': True, 'continuous_navigation': True, 'state_valid': True,
        'output_transition': 'none', 'mode': mode, 'authorized': authorized,
        'publisher_present': publisher_present,
    }


def test_policy_web_requires_confirmations_live_policy_and_expected_state(module):
    app = app_without_ros(module)
    app.processes.status.return_value = {'navigation': {'running': True, 'motion_backend': 'policy'}, 'policy_bridge': {'running': True}}
    app.ros.policy_control_status.return_value = policy_status()
    app.ros.policy_control_command.return_value = {'success': True, 'message': 'ok'}

    missing_confirmation = app.policy_control_step({'step': 'authorize_output'})
    assert missing_confirmation['success'] is False
    app.ros.policy_control_command.assert_not_called()

    payload = policy_confirmations() | {'step': 'authorize_output'}
    app.processes.status.return_value = {'navigation': {'running': True, 'motion_backend': 'unitree'}, 'policy_bridge': {'running': False}}
    assert app.policy_control_step(payload)['success'] is False
    app.ros.policy_control_command.assert_not_called()

    app.processes.status.return_value = {'navigation': {'running': True, 'motion_backend': 'policy'}, 'policy_bridge': {'running': True}}
    app.ros.policy_control_status.return_value = policy_status(mode='FixStand', authorized=True, publisher_present=True)
    assert app.policy_control_step(payload)['success'] is False
    app.ros.policy_control_command.assert_not_called()


def test_policy_web_allows_only_ordered_manual_state_transitions(module):
    app = app_without_ros(module)
    app.processes.status.return_value = {'navigation': {'running': True, 'motion_backend': 'policy'}, 'policy_bridge': {'running': True}}
    app.ros.policy_control_command.return_value = {'success': True, 'message': 'ok'}
    confirmations = policy_confirmations()
    states = [
        policy_status(),
        policy_status(mode='Passive', authorized=True, publisher_present=True),
        policy_status(mode='FixStand', authorized=True, publisher_present=True),
        policy_status(mode='ArmHoldStand', authorized=True, publisher_present=True),
    ]
    steps = ['authorize_output', 'fix_stand', 'arm_hold_stand', 'arm_hold']
    for step, status in zip(steps, states):
        app.ros.policy_control_status.return_value = status
        payload = confirmations | {'step': step, 'confirm_upright': step == 'arm_hold'}
        assert app.policy_control_step(payload)['success'] is True
    assert [call.args[0] for call in app.ros.policy_control_command.call_args_list] == steps


def test_policy_web_requires_onsite_upright_confirmation_for_arm_hold(module):
    app = app_without_ros(module)
    app.processes.status.return_value = {
        'navigation': {'running': True, 'motion_backend': 'policy'},
        'policy_bridge': {'running': True},
    }
    app.ros.policy_control_status.return_value = policy_status(
        mode='ArmHoldStand', authorized=True, publisher_present=True
    )
    payload = policy_confirmations() | {'step': 'arm_hold', 'confirm_upright': False}
    assert app.policy_control_step(payload)['success'] is False
    app.ros.policy_control_command.assert_not_called()


def test_policy_navigation_start_requires_arm_hold_and_never_autostarts_route(module):
    app = app_without_ros(module)
    app.processes.status.return_value = {
        'navigation': {'running': False}, 'policy_bridge': {'running': True}
    }
    app.validate_navigation = Mock(return_value=(True, ''))
    app.navigation_args = Mock(return_value=['mode=online', 'motion_backend=policy'])
    app.processes.start_navigation.return_value = {'success': True}
    app.ros.policy_control_status.return_value = {
        'available': True, 'control_ready': False,
        'message': 'ONNX 运动控制尚未进入 ArmHold'
    }
    payload = {'motion_backend': 'policy', 'run_mode': 'test', 'auto_start_route': True}

    blocked = app.start_navigation_task(payload)
    assert blocked['success'] is False
    app.processes.start_navigation.assert_not_called()

    app.ros.policy_control_status.return_value = {'available': True, 'control_ready': True}
    started = app.start_navigation_task(payload)
    assert started['success'] is True
    app.processes.start_navigation.assert_called_once()
    app.navigation_args.assert_called_once()
    assert app.navigation_args.call_args.args[0]['auto_start_route'] is False


def test_policy_control_can_be_prepared_before_navigation(module):
    app = app_without_ros(module)
    app.processes.status.return_value = {
        'navigation': {'running': False}, 'policy_bridge': {'running': True}
    }
    app.ros.policy_control_status.return_value = policy_status()
    app.ros.policy_control_command.return_value = {'success': True, 'message': 'ok'}
    result = app.policy_control_step(policy_confirmations() | {'step': 'authorize_output'})
    assert result['success'] is True


def test_policy_web_blocks_controls_after_heartbeat_timeout(module):
    app = app_without_ros(module)
    app.heartbeat_expired = True
    result = app.policy_control_step(policy_confirmations() | {'step': 'authorize_output'})
    assert result['success'] is False
    app.ros.policy_control_command.assert_not_called()


def test_watchdog_does_not_latch_timeout_before_first_browser_heartbeat(module):
    app = app_without_ros(module)
    app.last_heartbeat = None

    class StopAfterOneCheck:
        calls = 0

        def wait(self, _interval):
            self.calls += 1
            return self.calls > 1

    app.watchdog_stop = StopAfterOneCheck()
    app._watch_heartbeat()

    assert app.heartbeat_expired is False
    app.processes.stop_navigation.assert_not_called()
    app.ros.route_stop.assert_not_called()


def test_launch_uses_selected_task_and_only_marks_success_loaded(module, config):
    app = app_without_ros(module)
    app.validate_navigation = Mock(return_value=(True, ''))
    app.save_config = Mock()
    app.processes.start_navigation.return_value = {'success':True}
    app.start_navigation_task(config)
    args = app.processes.start_navigation.call_args.args[0]
    assert 'memory_route=B,A' in args
    assert 'gesture_route=B:wave_hand' in args
    assert 'turn_route=B:ccw,A:cw' in args
    assert app.loaded_task['id'] == 'one'
    config['selected_task_id'] = 'two'
    app.processes.start_navigation.return_value = {'success':False}
    app.start_navigation_task(config)
    assert app.loaded_task['id'] == 'one'


def test_pc4_launch_keeps_configured_gesture_route(module, config):
    app = app_without_ros(module)
    app.pc4_profile = True
    app.validate_navigation = Mock(return_value=(True, ''))
    app.save_config = Mock()
    app.processes.start_navigation.return_value = {'success': True}

    result = app.start_navigation_task(config)

    assert result['success'] is True
    args = app.processes.start_navigation.call_args.args[0]
    assert 'gesture_route=B:wave_hand' in args


def test_cruise_rejects_changed_task_order_action_and_stopped_navigation(module, config, tmp_path):
    app = app_without_ros(module, tmp_path)
    points_file = module.WebGuiApp._task_points_file(tmp_path, {'id': 'one', 'name': '迎宾'})
    points_file.parent.mkdir(parents=True)
    points_file.write_text(json.dumps({'frame_id': 'map', 'points': {'A': {}, 'B': {}}}))
    app.loaded_task = module.task_execution_snapshot(config)
    for change in [lambda c: c.update(selected_task_id='two'),
                   lambda c: c['task_profiles'][0]['steps'].reverse(),
                   lambda c: c['task_profiles'][0]['steps'][0].update(gesture='shake_hand'),
                   lambda c: c['task_profiles'][0]['steps'][0].update(turn='cw')]:
        changed = copy.deepcopy(config)
        change(changed)
        assert app.start_selected_task(changed)['success'] is False
    app.ros.route_start.assert_not_called()
    app.ros.route_start.return_value = {'success':True}
    assert app.start_selected_task(config)['success'] is True
    app.processes.status.return_value = {'navigation':{'running':False}}
    assert app.start_selected_task(config)['success'] is False
    assert app.ros.route_start.call_count == 1


def test_web_can_attach_a_matching_policy_route_to_an_externally_started_nav_stack(module, config, tmp_path):
    app = app_without_ros(module, tmp_path)
    app.loaded_task = None
    app.processes.status.return_value = {'navigation': {
        'running': True, 'external': True, 'run_mode': 'online', 'motion_backend': 'policy'
    }}
    config.update(run_mode='online', motion_backend='policy',
                  confirm_debug_mode=True, confirm_exclusive_control=True)
    config['task_profiles'][0]['steps'] = [
        {'point': 'B', 'gesture': '', 'turn': ''},
        {'point': 'A', 'gesture': '', 'turn': ''},
    ]
    points_file = module.WebGuiApp._task_points_file(tmp_path, {'id': 'one', 'name': '迎宾'})
    points_file.parent.mkdir(parents=True)
    points_file.write_text(json.dumps({'frame_id': 'camera_init', 'points': {'A': {}, 'B': {}}}))
    app.ros.policy_control_status.return_value = {'ready': True, 'message': 'ready'}
    app.ros.policy_localization_status.return_value = {'ready': True, 'message': 'ready'}
    app.ros.policy_route_start.return_value = {'success': True}

    result = app.start_selected_task(config)

    assert result['success'] is True
    assert app.loaded_task['id'] == 'one'
    app.ros.policy_route_start.assert_called_once()


def test_web_refuses_to_attach_route_to_a_different_external_backend(module, config):
    app = app_without_ros(module)
    app.loaded_task = None
    app.processes.status.return_value = {'navigation': {
        'running': True, 'external': True, 'run_mode': 'online', 'motion_backend': 'unitree'
    }}
    config.update(run_mode='online', motion_backend='policy',
                  confirm_debug_mode=True, confirm_exclusive_control=True)

    result = app.start_selected_task(config)

    assert result['success'] is False
    assert '后端不一致' in result['message']
    app.ros.route_start.assert_not_called()


def test_save_reload_keeps_all_profiles_and_selected_task(module, config, tmp_path):
    app = app_without_ros(module)
    app.workspace_root = tmp_path
    app.ui_only = True
    config['selected_task_id'] = 'two'
    app.save_config(config)
    loaded = app.default_config()
    assert loaded['selected_task_id'] == 'two'
    assert loaded['task_profiles'] == config['task_profiles']
    assert loaded['memory_route'] == 'C'
    assert loaded['gesture_route'] == 'C:shake_hand'


def test_saved_heartbeat_port_cannot_override_active_websocket_port(module, config, tmp_path):
    app = app_without_ros(module)
    app.workspace_root = tmp_path
    app.ui_only = False
    app.heartbeat_ws_port = 8776
    config['heartbeat_ws_port'] = 8766
    app.save_config(config)

    assert app.default_config()['heartbeat_ws_port'] == 8776


def test_navigation_rejects_missing_task_points_before_start(module, config, tmp_path):
    app = app_without_ros(module)
    existing = tmp_path/'map'
    existing.write_text('placeholder')
    config.update(map=str(existing), pcd=str(existing))
    app.memory_points = Mock(return_value=[{'name':'A'}])
    result = app.start_navigation_task(config)
    assert result['success'] is False
    assert 'B' in result['message']
    app.processes.start_navigation.assert_not_called()


def test_old_profiles_inherit_previous_shell_turn_default(module):
    config = {'task_profiles':[{'id':'old', 'name':'旧任务', 'steps':[{'point':'B'}, {'point':'A'}]}]}
    module.normalize_task_profiles(config)
    assert config['turn_route'] == 'B:ccw,A:cw'
    assert [step['turn'] for step in config['task_profiles'][0]['steps']] == ['ccw', 'cw']


def test_legacy_explicit_turns_override_shell_default(module):
    config = {'memory_route':'B,A', 'turn_route':'B:cw,A:ccw'}
    module.normalize_task_profiles(config)
    assert config['turn_route'] == 'B:cw,A:ccw'


def test_explicit_no_turn_survives_save_reload_and_emits_empty_argument(module, config, tmp_path):
    for task in config['task_profiles']:
        for step in task['steps']:
            step['turn'] = ''
    config['turn_route'] = 'B:ccw,A:cw'
    app = app_without_ros(module)
    app.workspace_root = tmp_path
    app.ui_only = True
    app.save_config(config)
    loaded = app.default_config()
    assert loaded['turn_route'] == ''
    assert 'turn_route=' in app.navigation_args(loaded)
    assert all(step['turn'] == '' for task in loaded['task_profiles'] for step in task['steps'])


def test_task_turns_are_independent_and_persisted(module, config, tmp_path):
    config['task_profiles'][1]['steps'] = [{'point':'A', 'gesture':'', 'turn':'ccw'}]
    app = app_without_ros(module)
    app.workspace_root = tmp_path
    app.ui_only = True
    config['selected_task_id'] = 'two'
    app.save_config(config)
    loaded = app.default_config()
    assert loaded['turn_route'] == 'A:ccw'
    loaded['selected_task_id'] = 'one'
    assert 'turn_route=B:ccw,A:cw' in app.navigation_args(loaded)


@pytest.mark.parametrize(
    ("mode", "profile", "expected"),
    [
        ("online", "nav2_h1_2.yaml", "nav2_h1_2_camera_init.yaml"),
        ("online", "nav2_h1_2_rpp.yaml", "nav2_h1_2_camera_init.yaml"),
        ("online", "nav2_h1_2_camera_init.yaml", "nav2_h1_2_camera_init.yaml"),
        ("navigation", "nav2_h1_2.yaml", "nav2_h1_2_rpp.yaml"),
        ("navigation", "nav2_h1_2_camera_init.yaml", "nav2_h1_2_rpp.yaml"),
        ("navigation", "nav2_h1_2_rpp.yaml", "nav2_h1_2_rpp.yaml"),
    ],
)
def test_navigation_args_selects_bundled_profile_for_mode(module, config, tmp_path, mode, profile, expected):
    app = app_without_ros(module)
    config["run_mode"] = mode
    config["nav2_params_file"] = str(tmp_path / profile)

    params_arg = next(arg for arg in app.navigation_args(config) if arg.startswith("params_file="))

    assert Path(params_arg.removeprefix("params_file=")) == tmp_path / expected


@pytest.mark.parametrize("mode", ["online", "navigation"])
def test_navigation_args_preserves_custom_profile(module, config, tmp_path, mode):
    app = app_without_ros(module)
    custom_profile = tmp_path / "operator_nav2.yaml"
    config.update(run_mode=mode, nav2_params_file=str(custom_profile))

    params_arg = next(arg for arg in app.navigation_args(config) if arg.startswith("params_file="))

    assert Path(params_arg.removeprefix("params_file=")) == custom_profile


def test_motion_backend_defaults_to_unitree_and_forwards_selection(module, config, tmp_path):
    app = app_without_ros(module)
    app.workspace_root = tmp_path
    app.ui_only = False

    assert app.default_config()["motion_backend"] == "unitree"
    config["motion_backend"] = "policy"
    config["auto_start_route"] = True
    config["enable_remote_velocity"] = True
    args = app.navigation_args(config)
    assert "motion_backend=policy" in args
    assert "enable_remote_velocity=true" in args
    assert "auto_start_route=false" in args

    config["motion_backend"] = "unitree"
    assert "motion_backend=unitree" in app.navigation_args(config)


def test_policy_backend_is_available_for_online_navigation(module, config):
    app = app_without_ros(module)
    config.update(run_mode="online", motion_backend="policy",
                  confirm_debug_mode=True, confirm_exclusive_control=True)

    ok, message = app.validate_navigation(config)

    assert ok, message
    assert "motion_backend=policy" in app.navigation_args(config)
