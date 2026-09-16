"""Exercise task persistence and launch guards without importing ROS or starting processes."""
import ast
import copy
import json
import os
import threading
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
    namespace = dict(Path=Path, json=json, os=os, threading=threading, CONFIG_PATH=tmp_path/'config.json', CONFIG_WORKFLOW_VERSION=2)
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


def app_without_ros(module):
    app = module.WebGuiApp.__new__(module.WebGuiApp)
    app.task_lock = threading.RLock()
    app.loaded_task = None
    app.processes = Mock()
    app.processes.status.return_value = {'navigation':{'running':True}}
    app.ros = Mock()
    return app


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


def test_cruise_rejects_changed_task_order_action_and_stopped_navigation(module, config):
    app = app_without_ros(module)
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
