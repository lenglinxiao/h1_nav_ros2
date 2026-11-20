import os
import pty
import unittest
from repeat_input import RepeatInput


class RepeatInputTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.reader = RepeatInput(self.slave)

    def tearDown(self):
        self.reader.close()
        os.close(self.master); os.close(self.slave)

    def test_busy_walk_discarded_partial_cannot_replay(self):
        os.write(self.master, b'WALK\nWA')
        self.assertIsNone(self.reader.poll(False))
        self.reader.discard_partial()
        os.write(self.master, b'LK\n')
        self.assertIsNone(self.reader.poll(True))
        os.write(self.master, b'WALK\n')
        self.assertEqual(self.reader.poll(True), 'WALK')

    def test_revoke_wins_over_multiple_walks(self):
        os.write(self.master, b'WALK\nWALK\nREVOKE\n')
        self.assertEqual(self.reader.poll(True), 'REVOKE')
        self.assertIsNone(self.reader.poll(True))

    def test_busy_revoke_is_preserved(self):
        os.write(self.master, b'WALK\nREVOKE\n')
        self.assertEqual(self.reader.poll(False), 'REVOKE')

    def test_structured_trials_opt_in_and_discard_busy_input(self):
        self.reader.allow_trials = True
        request = 'TRIAL {"motion":"turn_left","speed":0.05,"duration":1}\n'
        os.write(self.master, request.encode())
        self.assertIsNone(self.reader.poll(False))
        os.write(self.master, (request + request).encode())
        self.assertEqual(self.reader.poll(True), request.strip())
        self.assertIsNone(self.reader.poll(True))
        os.write(self.master, (request + 'REVOKE\n').encode())
        self.assertEqual(self.reader.poll(True), 'REVOKE')
