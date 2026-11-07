import importlib.util
import os
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.utilities import normalize_to_list_of_substitutions

ROOT = os.path.dirname(os.path.dirname(__file__))
PATH = os.path.join(ROOT, 'launch', 'h1_2_nav2_navigation.launch.py')
WORKSPACE = os.path.abspath(os.path.join(ROOT, '..', '..'))


def package_share(name):
    if name == 'h1_2_description':
        return os.path.join(WORKSPACE, 'src', name)
    return os.path.join('/tmp', name)


def description():
    spec = importlib.util.spec_from_file_location('navigation_launch', PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch.object(module, 'get_package_share_directory', side_effect=package_share):
        return module.generate_launch_description()


def selected(backend=None, **overrides):
    desc = description()
    ctx = LaunchContext()
    if backend is not None:
        ctx.launch_configurations['motion_backend'] = backend
    ctx.launch_configurations.update(overrides)
    for entity in desc.entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name not in ctx.launch_configurations:
            if entity.name == 'map':
                ctx.launch_configurations['map'] = '/tmp/map.yaml'
            elif entity.default_value is not None:
                ctx.launch_configurations[entity.name] = ''.join(
                    x.perform(ctx) for x in normalize_to_list_of_substitutions(entity.default_value))
    for entity in desc.entities:
        if isinstance(entity, OpaqueFunction):
            entity.execute(ctx)
    result = []
    for entity in desc.entities:
        if isinstance(entity, IncludeLaunchDescription) and (entity.condition is None or entity.condition.evaluate(ctx)):
            with patch.object(FindPackageShare, 'perform', return_value=os.path.join(WORKSPACE, 'src', 'h1_2_policy_bridge')):
                try:
                    entity.launch_description_source.get_launch_description(ctx)
                except FileNotFoundError:
                    pass
            result.append(entity)
    return ctx, result


class NavigationBackendTest(unittest.TestCase):
    def test_default_selects_unitree(self):
        ctx, includes = selected()
        self.assertEqual(ctx.launch_configurations['motion_backend'], 'unitree')
        paths = [str(x.launch_description_source.location) for x in includes]
        self.assertTrue(any('h1_2_loco_bridge.launch.py' in p for p in paths))
        self.assertFalse(any('policy_bridge.launch.py' in p for p in paths))

    def test_policy_excludes_unitree_and_rejects_unsafe_configuration(self):
        _, includes = selected('policy')
        paths = [str(x.launch_description_source.location) for x in includes]
        self.assertFalse(any('h1_2_loco_bridge.launch.py' in p for p in paths))
        self.assertTrue(any('policy_bridge.launch.py' in p for p in paths))
        for args in ({'policy_network_interface': 'eth0'}, {'policy_dds_domain': '0'}, {'gesture_route': 'A'}):
            with self.subTest(args=args), self.assertRaises(RuntimeError):
                selected('policy', **args)

    def test_hardware_policy_accepts_non_loopback_and_legal_domain(self):
        selected('policy', policy_profile='hardware', policy_network_interface='eth0', policy_dds_domain='7')
        for args in (
            {'policy_profile': 'unknown'},
            {'policy_profile': 'hardware', 'policy_network_interface': 'lo'},
            {'policy_profile': 'hardware', 'policy_network_interface': 'eth0', 'policy_dds_domain': '233'},
            {'localization_backend': 'unknown'},
        ):
            with self.subTest(args=args), self.assertRaises(RuntimeError):
                selected('policy', **args)

    def test_policy_include_loads_and_forwards_public_arguments(self):
        ctx, includes = selected(
            'policy', policy_model_path='/tmp/custom.onnx', policy_profile='hardware',
            policy_network_interface='eth0', policy_dds_domain='7',
            policy_joint_command_mode='position_pd', policy_enable_hardware_output='true',
            policy_operator_confirmed_debug_mode='true',
            policy_operator_confirmed_exclusive_control='true', policy_max_policy_steps='123',
            policy_continuous_policy_navigation='true', policy_enable_navigation_commands='true')
        include = next(x for x in includes if x.launch_description_source.location.endswith('policy_bridge.launch.py'))
        child = include.launch_description_source.get_launch_description(ctx)
        child_args = {x.name for x in child.entities if isinstance(x, DeclareLaunchArgument)}
        forwarded = {
            ''.join(part.perform(ctx) for part in normalize_to_list_of_substitutions(name)): ''.join(
                part.perform(ctx) for part in normalize_to_list_of_substitutions(value))
            for name, value in include.launch_arguments
        }
        self.assertEqual(forwarded, {
            'model_path': '/tmp/custom.onnx', 'cmd_vel_topic': '/cmd_vel_final',
            'network_interface': 'eth0', 'dds_domain': '7', 'profile': 'hardware',
            'joint_command_mode': 'position_pd', 'enable_hardware_output': 'true',
            'operator_confirmed_debug_mode': 'true', 'operator_confirmed_exclusive_control': 'true',
            'max_policy_steps': '123', 'continuous_policy_navigation': 'true',
            'enable_navigation_commands': 'true',
        })
        self.assertTrue(set(forwarded).issubset(child_args))

    def test_policy_navigation_options_default_off(self):
        ctx, includes = selected('policy')
        include = next(x for x in includes if x.launch_description_source.location.endswith('policy_bridge.launch.py'))
        forwarded = {
            ''.join(part.perform(ctx) for part in normalize_to_list_of_substitutions(name)):
            ''.join(part.perform(ctx) for part in normalize_to_list_of_substitutions(value))
            for name, value in include.launch_arguments
        }
        self.assertEqual(forwarded['continuous_policy_navigation'], 'false')
        self.assertEqual(forwarded['enable_navigation_commands'], 'false')

    def test_joint_mode_defaults_follow_profile(self):
        self.assertEqual(selected('policy')[0].launch_configurations['policy_joint_command_mode'], 'torque')
        self.assertEqual(selected('policy', policy_profile='hardware', policy_network_interface='eth0',
                                  policy_dds_domain='0')[0].launch_configurations[
            'policy_joint_command_mode'], 'position_pd')
        self.assertEqual(selected('policy', policy_profile='hardware', policy_network_interface='eth0',
                                  policy_dds_domain='0',
                                  policy_joint_command_mode='torque')[0].launch_configurations[
            'policy_joint_command_mode'], 'torque')

    def test_policy_nav2_remaps_and_composition_forwarding(self):
        desc = description()
        for use_composition in ('false',):
            with self.subTest(use_composition=use_composition):
                ctx = LaunchContext()
                for entity in desc.entities:
                    if isinstance(entity, DeclareLaunchArgument) and entity.default_value is not None:
                        value = '/tmp/map.yaml' if entity.name == 'map' else ''.join(
                            part.perform(ctx) for part in normalize_to_list_of_substitutions(entity.default_value))
                        ctx.launch_configurations[entity.name] = value
                ctx.launch_configurations.update(motion_backend='policy', use_composition=use_composition)
                group = next(e for e in desc.entities if isinstance(e, GroupAction))
                actions = group._GroupAction__actions
                remaps = [e for e in actions if isinstance(e, SetRemap)]
                actual = {(
                    ''.join(part.perform(ctx) for part in e._SetRemap__src),
                    ''.join(part.perform(ctx) for part in e._SetRemap__dst),
                ) for e in remaps}
                suffixes = ('', '/_action/send_goal', '/_action/get_result', '/_action/cancel_goal',
                            '/_action/feedback', '/_action/status')
                expected = {('/' + name + suffix, '/h1_2/nav2/' + name + suffix)
                            for name in ('navigate_to_pose', 'navigate_through_poses')
                            for suffix in suffixes}
                self.assertEqual(actual, expected)
                nav_include = next(e for e in actions if isinstance(e, IncludeLaunchDescription))
                forwarded = {
                    ''.join(part.perform(ctx) for part in normalize_to_list_of_substitutions(name)):
                    ''.join(part.perform(ctx) for part in normalize_to_list_of_substitutions(value))
                    for name, value in nav_include.launch_arguments
                }
                self.assertEqual(forwarded['use_composition'], use_composition)
                self.assertTrue(nav_include.condition.evaluate(ctx))

    def test_use_composition_is_rejected_without_container(self):
        with self.assertRaisesRegex(RuntimeError, 'does not create a Nav2 container'):
            selected('policy', use_composition='true')

    def test_invalid_backend_fails(self):
        with self.assertRaises(RuntimeError):
            selected('unitre')


if __name__ == '__main__':
    unittest.main()
