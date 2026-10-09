from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]


class TestPc4DeploymentAssets(unittest.TestCase):
    def read(self, relative_path):
        return (ROOT / relative_path).read_text(encoding="utf-8")

    def test_pc4_environment_keeps_sdk_and_dds_interfaces_separate(self):
        environment = self.read("config/pc4.env.example")
        self.assertIn("H1_2_RUNTIME_PROFILE=pc4", environment)
        self.assertIn("H1_2_NETWORK_INTERFACE=eth0", environment)
        self.assertIn("H1_2_DDS_CONFIG=", environment)
        self.assertIn("UNITREE_ROS_SETUP=", environment)
        self.assertIn("UNITREE_SDK2_ROOT=", environment)
        self.assertIn("H1_2_WEB_HEARTBEAT_TIMEOUT_SEC=2", environment)

    def test_deploy_archive_excludes_build_artifacts_without_dropping_vcs_metadata(self):
        script = self.read("scripts/pc4_deploy.sh")
        self.assertIn("--exclude='build'", script)
        self.assertIn("--exclude='install'", script)
        self.assertIn("--exclude='log'", script)
        self.assertIn("scp -o BatchMode=yes", script)
        self.assertNotIn("--exclude='**/build_*'", script)
        self.assertTrue((ROOT / "src/FAST_LIO_ROS2/include/IKFoM_toolkit/mtk/build_manifold.hpp").is_file())
        self.assertIn("tar --skip-old-files", script)
        self.assertIn("--exclude='maps'", script)
        self.assertNotIn("--exclude-vcs", script)

    def test_build_is_limited_to_ros_source_and_two_workers(self):
        script = self.read("scripts/pc4_build.sh")
        self.assertIn("--base-paths src", script)
        self.assertIn("--parallel-workers 2", script)
        self.assertIn("UNITREE_SDK2_ROOT", script)
        self.assertIn("H1_2_SKIP_ROSDEP", script)

    def test_web_unit_uses_fixed_port_and_user_service_target(self):
        service = self.read("config/systemd/h1-2-web.service.in")
        start_web = self.read("scripts/pc4_start_web.sh")
        self.assertIn("--host 0.0.0.0 --port 8765 --port-tries 1", start_web)
        self.assertIn("ExecStart=/bin/bash @WORKSPACE@/scripts/pc4_start_web.sh", service)
        self.assertIn("TimeoutStopSec=20", service)
        self.assertIn("WantedBy=default.target", service)
        self.assertNotIn("run_ai_slam_nav.sh", service)

    def test_stop_script_targets_owned_launch_session_only(self):
        script = self.read("stop_ai_slam_nav.sh")
        self.assertIn("b\"vel 0.0 0.0 0.0 0.5\"", script)
        self.assertIn("b\"stop\"", script)
        self.assertIn("ps -o sid=,pgid=,args=", script)
        self.assertIn("read -r launch_sid launch_pgid launch_args", script)
        self.assertIn("kill -INT", script)
        self.assertIn("kill -TERM", script)
        self.assertIn("kill -KILL", script)
        self.assertNotIn("pkill", script)

    def test_rviz_uses_configured_domain_and_nav_yaml_has_no_developer_path(self):
        launch = self.read("src/h1_2_slam_bringup/launch/h1_2_nav2_rviz.launch.py")
        params = self.read("src/h1_2_nav_tools/config/nav2_h1_2_camera_init.yaml")
        self.assertIn('os.environ.get("ROS_DOMAIN_ID", "0")', launch)
        self.assertNotIn("/home/llx/", params)


if __name__ == "__main__":
    unittest.main()
