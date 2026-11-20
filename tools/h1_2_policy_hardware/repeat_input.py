"""Opt-in terminal input for repeated pulses; no background reader."""
import os
import select
import termios
import tty


class RepeatInput:
    def __init__(self, fd, allow_trials=False):
        self.fd = fd
        self.allow_trials = allow_trials
        self.original = termios.tcgetattr(fd)
        self.partial = b''
        self.commands = []
        tty.setcbreak(fd, termios.TCSANOW)

    def close(self):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.original)

    def poll(self, accept_walk):
        while select.select([self.fd], [], [], 0)[0]:
            data = os.read(self.fd, 4096)
            if not data:
                raise EOFError('Terminal EOF awaiting handoff')
            self.partial += data
            while b'\n' in self.partial:
                line, self.partial = self.partial.split(b'\n', 1)
                command = line.strip()
                if command == b'REVOKE':
                    self.commands.append('REVOKE')
                elif command == b'WALK' and accept_walk:
                    self.commands.append('WALK')
                elif self.allow_trials and command.startswith(b'TRIAL ') and accept_walk:
                    self.commands.append(command.decode('utf-8'))
        if 'REVOKE' in self.commands:
            self.commands.clear()
            return 'REVOKE'
        if self.commands:
            command = self.commands[0]
            self.commands.clear()  # never queue several WALK commands
            return command
        return None

    def discard_partial(self):
        self.partial = b''
