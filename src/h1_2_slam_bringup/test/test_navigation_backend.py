import importlib.util
import os
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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
    for entity in desc.entities:
        if isinstance(entity, DeclareLaunchArgument):
            if entity.name == 'map':
                ctx.launch_configurations['map'] = '/tmp/map.yaml'
            elif entity.default_value is not None:
                ctx.launch_configurations[entity.name] = ''.join(x.perform(ctx) for x in entity.default_value)
    if backend is not None:
        ctx.launch_configurations['motion_backend'] = backend
    ctx.launch_configurations.update(overrides)
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

    def test_policy_include_loads_and_forwards_public_arguments(self):
        ctx, includes = selected('policy', policy_model_path='/tmp/custom.onnx')
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
            'network_interface': 'lo', 'dds_domain': '42',
        })
        self.assertTrue(set(forwarded).issubset(child_args))

    def test_invalid_backend_fails(self):
        with self.assertRaises(RuntimeError):
            selected('unitre')


if __name__ == '__main__':
    unittest.main()
