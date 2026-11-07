import unittest
from speed_guard import speed_violation

class SpeedGuardTest(unittest.TestCase):
    def sample(self, leg=0.0, upper=0.0):
        peaks = [0.0] * 27
        peaks[0], peaks[12] = leg, upper
        return {'peak_dq': max(peaks), 'peak_dq_by_joint': peaks}

    def test_group_limits_and_phase(self):
        self.assertEqual(speed_violation(self.sample(2.2), 3.0, False)['sdk_joint_index'], 0)
        self.assertIsNone(speed_violation(self.sample(2.2), 3.0, True))
        self.assertEqual(speed_violation(self.sample(3.1), 3.0, True)['group'], 'legs')
        self.assertEqual(speed_violation(self.sample(2.2, 2.1), 3.0, True)['sdk_joint_index'], 12)
        self.assertEqual(speed_violation(self.sample(2.2), 2.0, True)['limit'], 2.0)

    def test_scalar_array_mismatch_rejected(self):
        for limit in (2.0, 3.0):
            with self.subTest(limit=limit), self.assertRaisesRegex(RuntimeError, 'disagree'):
                speed_violation({'peak_dq': 2.1, 'peak_dq_by_joint': [0.0] * 27}, limit, True)

    def test_old_monitor_and_malformed_peaks(self):
        self.assertIsNone(speed_violation({'peak_dq': 1.9}, 2.0, False))
        self.assertIsNotNone(speed_violation({'peak_dq': 2.1}, 2.0, False))
        for peaks in (None, [], [0.0] * 26, [0.0] * 26 + [float('nan')], [0.0] * 26 + [-1]):
            with self.subTest(peaks=peaks), self.assertRaises(RuntimeError):
                speed_violation({'peak_dq': 0.0, 'peak_dq_by_joint': peaks}, 3.0, True)

if __name__ == '__main__':
    unittest.main()
