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
        self.assertEqual(params['max_policy_steps'], 0)
        self.assertIsInstance(params['max_policy_steps'], int)
        for name in ('enable_hardware_output', 'operator_confirmed_debug_mode',
                     'operator_confirmed_exclusive_control'):
            self.assertIs(params[name], False)

    def test_hardware_overrides_are_typed(self):
        params = self.evaluate({'profile': 'hardware', 'network_interface': 'eth0',
                                'dds_domain': '0', 'max_policy_steps': '123',
                                'enable_hardware_output': 'true',
                                'operator_confirmed_debug_mode': 'true',
                                'operator_confirmed_exclusive_control': 'false'})
        self.assertEqual(params['profile'], 'hardware')
        self.assertEqual(params['network_interface'], 'eth0')
        self.assertEqual(params['dds_domain'], 0)
        self.assertEqual(params['max_policy_steps'], 123)
        self.assertIsInstance(params['max_policy_steps'], int)
        self.assertIs(params['enable_hardware_output'], True)
        self.assertIs(params['operator_confirmed_debug_mode'], True)
        self.assertIs(params['operator_confirmed_exclusive_control'], False)

    def test_sim_position_pd_option(self):
        self.assertEqual(self.evaluate({'joint_command_mode': 'position_pd'})['joint_command_mode'], 'position_pd')


if __name__ == '__main__':
    unittest.main()
