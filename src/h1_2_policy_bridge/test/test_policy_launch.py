"""Evaluate launch parameters without executing the ROS node or opening DDS."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities.evaluate_parameters import evaluate_parameters


class LaunchParametersTest(unittest.TestCase):
    def evaluate(self, overrides):
        path = Path(__file__).resolve().parents[1] / 'launch' / 'policy_bridge.launch.py'
        spec = importlib.util.spec_from_file_location('policy_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with patch.object(module, 'get_package_share_directory', side_effect=lambda name: str(
                Path(__file__).resolve().parents[2] / name)):
            description = module.generate_launch_description()
        context = LaunchContext()
        context.launch_configurations.update(overrides)
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
        node = next(e for e in description.entities if isinstance(e, Node))
        evaluated = evaluate_parameters(context, node._Node__parameters)
        return next(item for item in evaluated if isinstance(item, dict))

    def test_sim_defaults(self):
        params = self.evaluate({})
        self.assertEqual(params['profile'], 'sim')
        self.assertEqual(params['joint_command_mode'], 'torque')
        self.assertEqual(params['network_interface'], 'lo')
        self.assertEqual(params['dds_domain'], 42)
        self.assertTrue(params['urdf_path'].endswith('/h1_2_handless.urdf'))
        self.assertEqual(params['max_policy_steps'], 0)
        self.assertIsInstance(params['max_policy_steps'], int)
        self.assertIs(params['continuous_policy_navigation'], False)
        self.assertIs(params['enable_navigation_commands'], False)
        self.assertIs(params['enable_remote_velocity'], False)
        for name in ('enable_hardware_output', 'operator_confirmed_debug_mode',
                     'operator_confirmed_exclusive_control'):
            self.assertIs(params[name], False)

    def test_hardware_overrides_are_typed(self):
        params = self.evaluate({'profile': 'hardware', 'network_interface': 'eth0',
                                'dds_domain': '0', 'max_policy_steps': '123',
                                'enable_hardware_output': 'true',
                                'operator_confirmed_debug_mode': 'true',
                                'operator_confirmed_exclusive_control': 'false',
                                'continuous_policy_navigation': 'true',
                                'enable_navigation_commands': 'true',
                                'enable_remote_velocity': 'true'})
        self.assertEqual(params['profile'], 'hardware')
        self.assertEqual(params['network_interface'], 'eth0')
        self.assertEqual(params['dds_domain'], 0)
        self.assertEqual(params['max_policy_steps'], 123)
        self.assertIsInstance(params['max_policy_steps'], int)
        self.assertIs(params['enable_hardware_output'], True)
        self.assertIs(params['operator_confirmed_debug_mode'], True)
        self.assertIs(params['operator_confirmed_exclusive_control'], False)
        self.assertIs(params['continuous_policy_navigation'], True)
        self.assertIs(params['enable_navigation_commands'], True)
        self.assertIs(params['enable_remote_velocity'], True)

    def test_sim_position_pd_option(self):
        self.assertEqual(self.evaluate({'joint_command_mode': 'position_pd'})['joint_command_mode'], 'position_pd')

    def test_direct_navigation_uses_guarded_profile_and_bounded_speed(self):
        params = self.evaluate({'profile': 'hardware', 'protection_profile': 'train_20261106',
                                'max_linear_speed': '0.20', 'max_angular_speed': '0.1'})
        self.assertEqual(params['protection_profile'], 'train_20261106')
        self.assertEqual(params['max_linear_speed'], 0.20)
        self.assertEqual(params['max_angular_speed'], 0.1)
        import yaml
        config = Path(__file__).resolve().parents[1] / 'config' / 'train_guard_20261106.yaml'
        guard = yaml.safe_load(config.read_text())['/**']['ros__parameters']
        self.assertEqual(len(guard['train_guard_joint_names_sdk']), 27)
        self.assertEqual(guard['train_guard_profile_hash'],
                         'f47bae87fe47d4743dea5b748acc79ff3d4f6c69df339fc34e662d92b8436284')
        self.assertTrue(params['urdf_path'].endswith('/h1_2_handless.urdf'))

    def test_joint_mode_defaults_follow_profile(self):
        self.assertEqual(self.evaluate({'profile': 'sim'})['joint_command_mode'], 'torque')
        self.assertEqual(self.evaluate({'profile': 'hardware'})['joint_command_mode'], 'position_pd')
        self.assertEqual(self.evaluate({'profile': 'hardware', 'joint_command_mode': 'torque'})['joint_command_mode'], 'torque')


if __name__ == '__main__':
    unittest.main()
