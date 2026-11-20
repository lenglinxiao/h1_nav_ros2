import importlib.util
import os
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.utilities import normalize_to_list_of_substitutions

ROOT = os.path.dirname(os.path.dirname(__file__))
PATH = os.path.join(ROOT, 'launch', 'h1_2_nav2_navigation.launch.py')
WORKSPACE = os.path.abspath(os.path.join(ROOT, '..', '..'))


def package_share(name):
    if name == 'h1_2_description':
        return os.path.join(WORKSPACE, 'src', name)
    return os.path.join('/tmp', name)


def description(path=PATH):
    spec = importlib.util.spec_from_file_location('navigation_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch.object(module, 'get_package_share_directory', side_effect=package_share):
        return module.generate_launch_description()


def selected(backend=None, path=PATH, **overrides):
    desc = description(path)
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
        if isinstance(entity, (IncludeLaunchDescription, Node)) and (entity.condition is None or entity.condition.evaluate(ctx)):
            if isinstance(entity, Node):
                result.append(entity)
                continue
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
        paths = [str(x.launch_description_source.location) for x in includes
                 if isinstance(x, IncludeLaunchDescription)]
        self.assertTrue(any('h1_2_loco_bridge.launch.py' in p for p in paths))

    def test_policy_uses_standard_nav2_and_smoother_without_unitree_bridge(self):
        _, actions = selected('policy')
        paths = [str(x.launch_description_source.location) for x in actions if isinstance(x, IncludeLaunchDescription)]
        executables = [x.node_executable for x in actions if isinstance(x, Node)]
        self.assertFalse(any('h1_2_loco_bridge.launch.py' in p for p in paths))
        self.assertIn('velocity_smoother_ema', executables)
        self.assertTrue(any(p.endswith('navigation_launch.py') for p in paths))
        self.assertFalse(any('policy_nav_session_manager' in x for x in executables))
        for args in ({'gesture_route': 'A'}, {'localization_backend': 'unknown'}):
            with self.subTest(args=args), self.assertRaises(RuntimeError):
                selected('policy', **args)

    def test_use_composition_is_rejected_without_container(self):
        with self.assertRaisesRegex(RuntimeError, 'does not create a Nav2 container'):
            selected('policy', use_composition='true')

    def test_invalid_backend_fails(self):
        with self.assertRaises(RuntimeError):
            selected('unitre')

    def test_camera_init_policy_keeps_mapping_and_replaces_only_motion_output(self):
        camera_path = os.path.join(ROOT, 'launch', 'h1_2_nav2_camera_init_navigation.launch.py')
        _, actions = selected('policy', path=camera_path)
        nodes = [x for x in actions if isinstance(x, Node)]
        executables = [x.node_executable for x in nodes]
        paths = [str(x.launch_description_source.location) for x in actions
                 if isinstance(x, IncludeLaunchDescription)]

        self.assertIn('pointcloud_to_occupancy_grid', executables)
        self.assertNotIn('policy_nav_session_manager', executables)
        self.assertIn('velocity_smoother_ema', executables)
        self.assertFalse(any('h1_2_loco_bridge.launch.py' in p for p in paths))
        self.assertTrue(any(p.endswith('navigation_launch.py') for p in paths))
        self.assertTrue(any('fastlio2_mid360_mapping.launch.py' in p for p in paths))


if __name__ == '__main__':
    unittest.main()
