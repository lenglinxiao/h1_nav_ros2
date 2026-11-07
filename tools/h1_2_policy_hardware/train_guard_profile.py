"""Construct the reviewed training guard in explicit SDK motor order."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

LEG = ['hip_yaw', 'hip_pitch', 'hip_roll', 'knee', 'ankle_pitch', 'ankle_roll']
ARM = ['shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw']
JOINT_NAMES_SDK = ([f'left_{n}_joint' for n in LEG] + [f'right_{n}_joint' for n in LEG] +
                   ['torso_joint'] + [f'left_{n}_joint' for n in ARM] + [f'right_{n}_joint' for n in ARM])
DEFAULTS_SDK = [.027,-.45,0.,.9,-.45,0.,-.027,-.45,0.,.9,-.475,0.,0.,.28,0.,0.,.52,0.,0.,0.,-.20,-.23,.90,-.55,0.,0.,0.]
SPEEDS_SDK = [4.,4.5,2.5,9.,7.5,6.,4.,4.5,2.5,9.5,8.,5.5,1.,1.5,1.,1.,2.,.5,1.,1.,1.,1.,1.,1.5,.5,1.,1.]


def build_profile(urdf_path):
    raw = Path(urdf_path).read_bytes()
    root = ET.fromstring(raw)
    joints = {}
    for joint in root.findall('joint'):
        name = joint.get('name')
        if name in JOINT_NAMES_SDK:
            if name in joints:
                raise ValueError(f'duplicate joint {name}')
            joints[name] = joint
    if set(joints) != set(JOINT_NAMES_SDK):
        raise ValueError('URDF missing required SDK joints')
    leg = [(-.55,.55),(-1.,0.),(-.45,.45),(.3,1.7),(-1.,0.),(-.45,.45)]
    soft = leg * 2 + [(-.3,.4)] + [(x-.35,x+.35) for x in DEFAULTS_SDK[13:]]
    lower, upper = [], []
    for name, default, (lo, hi) in zip(JOINT_NAMES_SDK, DEFAULTS_SDK, soft):
        limit = joints[name].find('limit')
        try:
            a, b = float(limit.get('lower')), float(limit.get('upper'))
        except (AttributeError, TypeError, ValueError):
            raise ValueError(f'invalid limits for {name}') from None
        if not math.isfinite(a) or not math.isfinite(b) or a > b:
            raise ValueError(f'invalid limits for {name}')
        a, b = max(a, lo), min(b, hi)
        if a > b or not a <= default <= b:
            raise ValueError(f'empty intersection or default outside bounds: {name}')
        lower.append(float(a)); upper.append(float(b))
    result = dict(joint_names_sdk=JOINT_NAMES_SDK, speed_limits_sdk=SPEEDS_SDK,
                  position_lower_sdk=lower, position_upper_sdk=upper,
                  default_positions_sdk=DEFAULTS_SDK, urdf_sha256=hashlib.sha256(raw).hexdigest())
    result['profile_hash'] = hashlib.sha256(json.dumps(result, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    return result
