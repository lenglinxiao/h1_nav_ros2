import json
from pathlib import Path
import sys
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent.parent / 'tools' / 'h1_2_policy_hardware'))
from train_guard_profile import build_profile


class TrainGuardConfigTest(unittest.TestCase):
    def test_launch_profile_matches_reviewed_urdf(self):
        config = yaml.safe_load((ROOT / 'config' / 'train_guard_20261106.yaml').read_text())
        params = config['/**']['ros__parameters']
        profile = build_profile(ROOT.parent / 'h1_2_description' / 'h1_2_handless.urdf')
        self.assertEqual(params['train_guard_joint_names_sdk'], profile['joint_names_sdk'])
        self.assertEqual(params['train_guard_speed_limits_sdk'], profile['speed_limits_sdk'])
        self.assertEqual(params['train_guard_position_lower_sdk'], profile['position_lower_sdk'])
        self.assertEqual(params['train_guard_position_upper_sdk'], profile['position_upper_sdk'])
        self.assertEqual(params['train_guard_default_positions_sdk'], profile['default_positions_sdk'])
        self.assertEqual(params['train_guard_profile_hash'], profile['profile_hash'])


if __name__ == '__main__':
    unittest.main()
