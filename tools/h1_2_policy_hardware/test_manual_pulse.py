import unittest
from manual_pulse import ManualPulse, ManualSettle

class ManualPulseTests(unittest.TestCase):
    def test_left_turn_expired_lease_cannot_replay(self):
        p = ManualPulse(3., 'turn_left'); p.authorize(0., True); p.start(.01)
        self.assertEqual(p.output(.02, True), (0., 0., .1))
        self.assertIsNone(p.output(.16, True))
        p.authorize(.17, True)
        self.assertEqual(p.output(.18, True), (0., 0., 0.))
        with self.assertRaises(RuntimeError): p.start(.19)

    def test_long_window_finishes_zero_and_is_single_use(self):
        p = ManualPulse(1.5)
        p.authorize(0., True); p.start(.01)
        for i in range(1, 16):
            t = i / 10
            p.authorize(t, True)
            self.assertEqual(p.output(t, True), (.12, 0., 0.))
        p.authorize(1.51, True)
        self.assertEqual(p.output(1.51, True), (0., 0., 0.))
        with self.assertRaises(RuntimeError): p.start(1.52)

    def test_long_window_ready_loss_and_expired_lease_cancel(self):
        for lose_ready in (True, False):
            p = ManualPulse(1.5); p.authorize(0., True); p.start(.01)
            for i in range(1, 9): p.authorize(i / 10, True)
            if lose_ready:
                p.authorize(.85, False)
            else:
                p.authorize(1., True)
            p.authorize(1.01, True)
            self.assertEqual(p.output(1.02, True), (0., 0., 0.))
            self.assertEqual(p.cancel_reason, 'readiness lost' if lose_ready else 'ready lease expired')

    def test_duration_bounds(self):
        for value in (.49, 3.01, float('nan'), float('inf')):
            with self.assertRaises(ValueError): ManualPulse(value)

    def test_three_second_window_and_late_interruption(self):
        for failure in (None, 'ready', 'lease'):
            p = ManualPulse(3.); p.authorize(0., True); p.start(.01)
            for i in range(1, 21): p.authorize(i / 10, True)
            if failure == 'ready': p.authorize(2.05, False)
            if failure == 'lease': p.authorize(2.2, True)
            if failure:
                p.authorize(2.21, True)
                self.assertEqual(p.output(2.22, True), (0., 0., 0.))
                self.assertEqual(p.cancel_reason, 'readiness lost' if failure == 'ready' else 'ready lease expired')
            else:
                for i in range(21, 31): p.authorize(i / 10, True)
                self.assertEqual(p.output(3., True), (.12, 0., 0.))
                p.authorize(3.01, True)
                self.assertEqual(p.output(3.01, True), (0., 0., 0.))

    def test_single_window_ready_and_lease(self):
        p = ManualPulse()
        self.assertIsNone(p.output(1., True))
        p.authorize(1., False)
        self.assertEqual(p.output(1.01, True), (0., 0., 0.))
        with self.assertRaises(RuntimeError): p.start(1.01)
        p.authorize(2., True)
        p.start(2.01)
        self.assertEqual(p.output(2.02, True), (.12, 0., 0.))
        self.assertIsNone(p.output(2.16, True))
        self.assertIsNone(p.output(2.02, False))
        p.authorize(2.49, True)
        self.assertEqual(p.output(2.5, True), (0., 0., 0.))
        self.assertEqual(p.cancel_reason, 'ready lease expired')
        self.assertEqual(p.output(2.51, True), (0., 0., 0.))
        with self.assertRaises(RuntimeError): p.start(2.52)
        p.fault = 'monitor lost'
        self.assertIsNone(p.output(2.52, True))

    def test_canceled_pulse_cannot_start(self):
        p = ManualPulse()
        p.cancel('canceled before start')
        p.authorize(1., True)
        with self.assertRaises(RuntimeError): p.start(1.01)

    def test_lease_gap_spanning_end_still_cancels(self):
        p = ManualPulse()
        p.authorize(1., True)
        p.start(1.01)
        p.authorize(1.6, True)
        self.assertEqual(p.cancel_reason, 'ready lease expired')
        self.assertEqual(p.output(1.61, True), (0., 0., 0.))

    def test_ready_loss_latches_cancel_without_replay(self):
        p = ManualPulse()
        p.authorize(1., True)
        p.start(1.01)
        self.assertEqual(p.output(1.02, True), (.12, 0., 0.))
        p.authorize(1.03, False)
        self.assertEqual(p.output(1.04, True), (0., 0., 0.))
        p.authorize(1.05, True)
        self.assertEqual(p.output(1.06, True), (0., 0., 0.))
        self.assertEqual(p.cancel_reason, 'readiness lost')

class ManualSettleTests(unittest.TestCase):
    def test_transient_restarts_three_second_window(self):
        gate = ManualSettle(0.)
        self.assertFalse(gate.observe(0., .1))
        self.assertFalse(gate.observe(2.9, .5))
        self.assertFalse(gate.observe(3., .1))
        self.assertFalse(gate.observe(5.99, .1))
        self.assertTrue(gate.observe(6., .1))

    def test_motion_after_completion_revokes_it(self):
        gate = ManualSettle(0.)
        gate.observe(0., .1)
        self.assertTrue(gate.observe(3., .1))
        self.assertFalse(gate.observe(3.1, .5))
        self.assertFalse(gate.observe(3.2, .1))
        self.assertFalse(gate.observe(6.19, .1))
        self.assertTrue(gate.observe(6.2, .1))

    def test_deadline_rejects_unsettled(self):
        gate = ManualSettle(0.)
        gate.observe(9., .6)
        with self.assertRaisesRegex(RuntimeError, 'Timed out'): gate.observe(10., .1)

    def test_original_speed_limit_still_aborts(self):
        gate = ManualSettle(0.)
        with self.assertRaisesRegex(RuntimeError, 'speed exceeded'): gate.observe(1., 2.01)
