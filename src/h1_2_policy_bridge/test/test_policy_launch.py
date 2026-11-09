"""Evaluate launch parameters without executing the ROS node or opening DDS."""
import importlib.util
from pathlib import Path
import unittest

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
        description = module.generate_launch_description()
        context = LaunchContext()
        context.launch_configurations.update(overrides)
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
        node = next(e for e in description.entities if isinstance(e, Node))
        return evaluate_parameters(context, node._Node__parameters)[0]

    def test_sim_defaults(self):
        params = self.evaluate({})
        self.assertEqual(params['profile'], 'sim')
        self.assertEqual(params['joint_command_mode'], 'torque')
        self.assertEqual(params['network_interface'], 'lo')
        self.assertEqual(params['dds_domain'], 42)
        self.assertEqual(params['input_identity_mode'], 'legacy')
        self.assertEqual(params['owner_boot_id'], '')
        self.assertEqual(params['owner_pid'], 0)
        self.assertEqual(params['urdf_path'], '')
        self.assertEqual(params['max_policy_steps'], 0)
        self.assertIsInstance(params['max_policy_steps'], int)
        self.assertIs(params['continuous_policy_navigation'], False)
        self.assertIs(params['enable_navigation_commands'], False)
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
                                'enable_navigation_commands': 'true'})
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

    def test_sim_position_pd_option(self):
        self.assertEqual(self.evaluate({'joint_command_mode': 'position_pd'})['joint_command_mode'], 'position_pd')

    def test_claimed_input_identity_mode_is_forwarded(self):
        params = self.evaluate({'input_identity_mode': 'claimed'})
        self.assertEqual(params['input_identity_mode'], 'claimed')

    def test_runtime_identity_owner_and_urdf_metadata_are_forwarded(self):
        params = self.evaluate({'owner_boot_id': 'owner-boot', 'owner_pid': '123',
                                'urdf_path': '/opt/h1/robot.urdf'})
        self.assertEqual(params['owner_boot_id'], 'owner-boot')
        self.assertEqual(params['owner_pid'], 123)
        self.assertEqual(params['urdf_path'], '/opt/h1/robot.urdf')

    def test_joint_mode_defaults_follow_profile(self):
        self.assertEqual(self.evaluate({'profile': 'sim'})['joint_command_mode'], 'torque')
        self.assertEqual(self.evaluate({'profile': 'hardware'})['joint_command_mode'], 'position_pd')
        self.assertEqual(self.evaluate({'profile': 'hardware', 'joint_command_mode': 'torque'})['joint_command_mode'], 'torque')


if __name__ == '__main__':
    unittest.main()
