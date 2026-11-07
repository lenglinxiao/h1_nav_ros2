import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from train_guard_profile import build_profile

URDF = Path(__file__).resolve().parents[2] / 'src/h1_2_description/h1_2_handless.urdf'

class ProfileTest(unittest.TestCase):
    def test_mapping_and_intersection(self):
        p = build_profile(URDF)
        self.assertEqual(p['joint_names_sdk'][1:3], ['left_hip_pitch_joint', 'left_hip_roll_joint'])
        self.assertEqual(p['speed_limits_sdk'][1:3], [4.5, 2.5])
        self.assertEqual(p['joint_names_sdk'][17:19], ['left_wrist_roll_joint', 'left_wrist_pitch_joint'])
        self.assertEqual(p['position_lower_sdk'][1:3], [-1.0, -0.43])
        self.assertEqual(p['position_upper_sdk'][1:3], [0.0, 0.45])
        self.assertEqual(len(p['profile_hash']), 64)

    def test_missing_duplicate_and_nonfinite_limits_rejected(self):
        for kind in ('missing', 'duplicate', 'nonfinite', 'empty', 'default_outside'):
            tree = ET.parse(URDF); root = tree.getroot()
            j = next(j for j in root.findall('joint') if j.get('name') == 'left_hip_pitch_joint')
            if kind == 'missing': root.remove(j)
            elif kind == 'duplicate': root.append(ET.fromstring(ET.tostring(j)))
            elif kind == 'nonfinite': j.find('limit').set('lower', 'nan')
            elif kind == 'empty': j.find('limit').set('lower', '0.1')
            else: j.find('limit').set('lower', '-0.4')
            with tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp) / 'robot.urdf'; tree.write(path)
                with self.subTest(kind=kind), self.assertRaises(ValueError): build_profile(path)
