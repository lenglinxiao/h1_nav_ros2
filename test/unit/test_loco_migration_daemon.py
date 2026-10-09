from pathlib import Path
import shutil
import signal
import socket
import subprocess
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[2]
DAEMON = ROOT / "src" / "h1_2_loco_bridge" / "src" / "h1_2_loco_udp_daemon.cpp"


class LocoDaemonBehaviorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("g++")
        if not compiler:
            raise unittest.SkipTest("g++ is required to compile the daemon against mock SDK headers")
        cls.tmp = tempfile.TemporaryDirectory(prefix="loco-daemon-test-")
        temp = Path(cls.tmp.name)
        include = temp / "include" / "unitree" / "robot"
        (include / "channel").mkdir(parents=True)
        (include / "h1" / "loco").mkdir(parents=True)
        (include / "channel" / "channel_factory.hpp").write_text(
            """#pragma once
#include <string>
namespace unitree::robot {
class ChannelFactory {
 public:
  static ChannelFactory* Instance() { static ChannelFactory instance; return &instance; }
  void Init(int, const std::string&) {}
};
}
"""
        )
        (include / "h1" / "loco" / "h1_loco_client.hpp").write_text(
            """#pragma once
#include <iostream>
namespace unitree::robot::h1 {
class LocoClient {
 public:
  void Init() {}
  int SwitchMoveMode(bool) { return 0; }
  int Start() { std::cout << "MOCK Start" << std::endl; return 0; }
  int SetVelocity(float, float, float, float) { return 0; }
  int StopMove() { std::cout << \"MOCK StopMove\" << std::endl; return 0; }
  int WaveHand() { return 0; }
  int ShakeHand(int) { return 0; }
  int LowStand() { return 0; }
  int HighStand() { return 0; }
  int SetTaskId(int) { return 0; }
};
}
"""
        )
        cls.binary = temp / "h1_2_loco_udp_daemon_mock"
        built = subprocess.run(
            [compiler, "-std=c++20", "-Wall", "-Wextra", "-I", str(temp / "include"), str(DAEMON), "-o", str(cls.binary)],
            capture_output=True,
            text=True,
            timeout=30,
        )
        if built.returncode:
            raise AssertionError(f"mock daemon compile failed:\n{built.stdout}\n{built.stderr}")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def start_daemon(self, timeout):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        proc = subprocess.Popen(
            [str(self.binary), "--udp-port", str(port), "--command-timeout", str(timeout)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        deadline = time.monotonic() + 3
        lines = []
        while time.monotonic() < deadline:
            line = proc.stdout.readline()
            if line:
                lines.append(line)
                if "listening on 127.0.0.1:" in line:
                    return proc, port, lines
            elif proc.poll() is not None:
                break
        proc.kill()
        raise AssertionError(f"mock daemon did not start: {''.join(lines)}")

    def finish(self, proc):
        try:
            out, _ = proc.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate()
            self.fail("daemon did not exit after shutdown")
        return out

    def test_invalid_velocity_does_not_refresh_watchdog(self):
        timeout = 0.35
        proc, port, initial = self.start_daemon(timeout)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            deadline = time.monotonic() + 0.9
            while time.monotonic() < deadline:
                sock.sendto(b"vel nan 0 0 0.1", ("127.0.0.1", port))
                time.sleep(0.04)
        finally:
            sock.close()
            proc.terminate()
            output = self.finish(proc)
        output = "".join(initial) + output
        self.assertIn("Control command timeout; StopMove ret=", output)
        self.assertIn("MOCK StopMove", output)

    def test_sigterm_and_sigint_each_call_stopmove(self):
        for sig in (signal.SIGTERM, signal.SIGINT):
            with self.subTest(signal=sig.name):
                proc, _, _ = self.start_daemon(5.0)
                proc.send_signal(sig)
                output = self.finish(proc)
                self.assertIn("MOCK StopMove", output)

    def test_start_then_stop_command_calls_sdk_stop_and_replies(self):
        proc, port, _ = self.start_daemon(5.0)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2)
        try:
            sock.sendto(b"start", ("127.0.0.1", port))
            self.assertTrue(sock.recv(256).decode().startswith("ok 0 start"))
            sock.sendto(b"stop", ("127.0.0.1", port))
            self.assertTrue(sock.recv(256).decode().startswith("ok 0 stop"))
        finally:
            sock.close()
            proc.terminate()
        output = self.finish(proc)
        self.assertLess(output.index("MOCK Start"), output.index("MOCK StopMove"))


if __name__ == "__main__":
    unittest.main()
