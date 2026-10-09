from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
BRIDGE = ROOT / "src" / "h1_2_loco_bridge"


class LocoMigrationContractTest(unittest.TestCase):
    def test_sdk_root_is_configurable_without_machine_user_path(self):
        cmake = (BRIDGE / "CMakeLists.txt").read_text()
        launch = (BRIDGE / "launch" / "h1_2_loco_bridge.launch.py").read_text()
        self.assertIn("UNITREE_SDK2_ROOT", cmake)
        self.assertIn("$ENV{UNITREE_SDK2_ROOT}", cmake)
        self.assertIn("UNITREE_SDK2_ROOT", launch)
        self.assertNotIn("/home/llx", cmake + launch)

    def test_daemon_stops_on_signals_and_control_timeout(self):
        daemon = (BRIDGE / "src" / "h1_2_loco_udp_daemon.cpp").read_text()
        self.assertIn("std::signal(SIGINT, handle_signal)", daemon)
        self.assertIn("std::signal(SIGTERM, handle_signal)", daemon)
        self.assertIn("--command-timeout", daemon)
        self.assertIn('client.StopMove();\n  close(sock);', daemon)
        self.assertIn('"Control command timeout; StopMove ret="', daemon)

    def test_sdk_interface_is_separate_and_loopback_udp_is_fixed(self):
        launch = (BRIDGE / "launch" / "h1_2_loco_bridge.launch.py").read_text()
        velocity = (BRIDGE / "src" / "cmd_vel_to_h1_2_loco.cpp").read_text()
        gesture = (BRIDGE / "src" / "gesture_to_h1_2_loco.cpp").read_text()
        daemon = (BRIDGE / "src" / "h1_2_loco_udp_daemon.cpp").read_text()
        self.assertIn('"network_interface", default_value=os.environ.get("H1_2_NETWORK_INTERFACE", "")', launch)
        self.assertIn('LaunchConfiguration("network_interface")', launch)
        self.assertIn("addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK)", daemon)
        self.assertEqual(velocity.count('udp_host_ != "127.0.0.1"'), 1)
        self.assertEqual(gesture.count('udp_host_ != "127.0.0.1"'), 1)


if __name__ == "__main__":
    unittest.main()
