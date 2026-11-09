import signal
import subprocess
import sys
import os
import time
import hashlib
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys
import types

from policy_owner import (
    CMD_TOPIC, HEARTBEAT_TOPIC, CLAIM_SERVICE, COORDINATOR_CLAIM_SERVICE,
    ChildSpec, ProcessSupervisor, bind_owner_tokens,
    build_runtime_manifest, canonical_profile_sha256,
    claim_and_start_coordinator, configure_runtime_policy, load_child_specs, run as run_owner,
    runtime_profile_from_urdf,
    runtime_identity_allows_shutdown, verify_runtime_identity,
)


VALID_ENV = {
    "H1_POLICY_OWNER_BRIDGE_COMMAND": (
        "/opt/policy-node --ros-args -p enable_hardware_output:=false "
        "-p input_identity_mode:=claimed -p owner_boot_id:={owner_boot_id} "
        "-p owner_pid:={owner_pid}"
    ),
    "H1_POLICY_OWNER_COORDINATOR_COMMAND": "ros2 run pkg coordinator --ros-args -p owner_boot_id:={owner_boot_id}",
    "H1_POLICY_OWNER_MONITOR_COMMAND": "monitor --until-stopped",
}


class Endpoint:
    def __init__(self, gid):
        self.endpoint_gid = gid


def sample_profile():
    from train_guard_profile import JOINT_NAMES_SDK

    profile = {
        "joint_names": list(JOINT_NAMES_SDK),
        "arrays": {
            "speed_limits_sdk": [1] * 27,
            "position_lower_sdk": [-1] * 27,
            "position_upper_sdk": [1] * 27,
            "default_positions_sdk": [0] * 27,
        },
        "source_sha256": hashlib.sha256(b"profile source").hexdigest(),
        "urdf_sha256": hashlib.sha256(b"urdf").hexdigest(),
        "profile_sha256": "0" * 64,
    }
    profile["profile_sha256"] = canonical_profile_sha256(profile)
    return profile


def make_manifest(temp_dir):
    paths = []
    for name, contents in (("model.onnx", b"model"), ("robot.urdf", b"urdf"),
                           ("h1_2_policy_node", b"executable")):
        path = Path(temp_dir) / name
        path.write_bytes(contents)
        if name == "h1_2_policy_node":
            path.chmod(0o700)
        paths.append(str(path))
    return build_runtime_manifest(
        paths, sample_profile(), owner_boot_id="0123456789abcdef0123456789abcdef", owner_pid=100,
        policy_pid=101, env={"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "0"},
    )


def make_status(manifest):
    files = manifest["files"]
    return SimpleNamespace(
        owner_boot_id=manifest["owner_boot_id"], owner_pid=manifest["owner_pid"],
        policy_pid=manifest["policy_pid"], node_fqn=manifest["node_fqn"],
        ros_domain_id=manifest["ros_domain_id"],
        ros_localhost_only=manifest["ros_localhost_only"],
        model_path=files["model"]["path"], model_sha256=files["model"]["sha256"],
        urdf_path=files["urdf"]["path"], urdf_sha256=files["urdf"]["sha256"],
        executable_path=files["executable"]["path"],
        executable_sha256=files["executable"]["sha256"],
        profile_sha256=manifest["profile"]["profile_sha256"],
        authorized=False, writer_present=False, lowcmd_writer_gid=[0] * 16,
        valid=True, reason="",
    )


def write_test_urdf(path):
    from train_guard_profile import JOINT_NAMES_SDK

    path.write_text(
        '<robot name="test">' + "".join(
            f'<joint name="{name}" type="revolute"><limit lower="-3" upper="3"/></joint>'
            for name in JOINT_NAMES_SDK
        ) + "</robot>",
        encoding="utf-8",
    )


def make_runtime_env(temp_dir):
    root = Path(temp_dir)
    model = root / "policy.onnx"
    model.write_bytes(b"model")
    executable = root / "h1_2_policy_node"
    executable.write_bytes(b"executable")
    executable.chmod(0o700)
    urdf = root / "robot.urdf"
    write_test_urdf(urdf)
    return {
        **VALID_ENV,
        "ROS_DOMAIN_ID": "0",
        "ROS_LOCALHOST_ONLY": "0",
        "H1_POLICY_OWNER_URDF_PATH": str(urdf),
        "H1_POLICY_OWNER_BRIDGE_COMMAND": (
            f"{executable} --ros-args -p model_path:={model} -p profile:=hardware "
            "-p joint_command_mode:=position_pd "
            "-p enable_hardware_output:=false -p input_identity_mode:=claimed "
            "-p owner_boot_id:={owner_boot_id} -p owner_pid:={owner_pid}"
        ),
    }


def invoke_fake_claim_startup(manifest, response, env, *, identity_available=True, timeout=0.01,
                              calls=None):
    calls = [] if calls is None else calls
    cmd_gid = list(range(1, 25))
    heartbeat_gid = list(range(25, 49))

    class Request:
        pass

    class ClaimService:
        pass

    class IdentityService:
        pass

    ClaimService.Request = Request
    IdentityService.Request = Request

    class Future:
        def __init__(self, result):
            self._result = result

        def done(self):
            return True

        def result(self):
            return self._result

    class Client:
        def __init__(self, service, name):
            self.service, self.name = service, name

        def service_is_ready(self):
            return identity_available or self.service is not IdentityService

        def call_async(self, request):
            if self.service is IdentityService:
                calls.append("identity")
                return Future(response)
            calls.append(self.name)
            return Future(SimpleNamespace(success=True, reason="claimed"))

    class FakeNode:
        def __init__(self, _name):
            pass

        def create_client(self, service, name):
            return Client(service, name)

        def get_publishers_info_by_topic(self, topic):
            if topic == CMD_TOPIC:
                return [Endpoint(cmd_gid)]
            if topic == HEARTBEAT_TOPIC:
                return [Endpoint(heartbeat_gid)]
            return []

        def destroy_node(self):
            pass

    fake_interfaces_srv = types.ModuleType("h1_2_policy_interfaces.srv")
    fake_interfaces_srv.ClaimPolicyInput = ClaimService
    fake_interfaces_srv.PolicyRuntimeIdentity = IdentityService
    with patch.dict(sys.modules, {"h1_2_policy_interfaces.srv": fake_interfaces_srv}), \
         patch("rclpy.init"), patch("rclpy.ok", return_value=True), \
         patch("rclpy.spin_once"), patch("rclpy.shutdown"), \
         patch("rclpy.node.Node", FakeNode):
        claim_and_start_coordinator(
            str(manifest["owner_boot_id"]), manifest, env, timeout_seconds=timeout
        )
    return calls


class PolicyOwnerTests(unittest.TestCase):
    def test_owner_claims_bridge_then_coordinator_before_heartbeat(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            env = make_runtime_env(temp_dir)
            _, profile, paths = configure_runtime_policy(load_child_specs(env), env)
            manifest = build_runtime_manifest(paths, profile, owner_boot_id="a" * 32,
                                              owner_pid=1234, policy_pid=5678, env=env)
            calls = invoke_fake_claim_startup(manifest, make_status(manifest), env)
        self.assertEqual(["identity", CLAIM_SERVICE, COORDINATOR_CLAIM_SERVICE], calls)

    def test_runtime_profile_hash_matches_bridge_profile_and_urdf_bytes(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            env = make_runtime_env(temp_dir)
            specs, profile, paths = configure_runtime_policy(load_child_specs(env), env)
            from train_guard_profile import build_profile

            expected_profile = build_profile(paths[1])
            self.assertEqual(expected_profile["profile_hash"], profile["profile_sha256"])
            self.assertEqual(hashlib.sha256(Path(paths[1]).read_bytes()).hexdigest(),
                             profile["urdf_sha256"])
            command = next(spec.command for spec in specs if spec.name == "policy")
            self.assertIn("protection_profile:=train_20261106", command)
            self.assertIn("train_guard_profile_hash:=" + expected_profile["profile_hash"], command)

    def test_identity_mismatch_blocks_both_input_claims(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            env = make_runtime_env(temp_dir)
            _, profile, paths = configure_runtime_policy(load_child_specs(env), env)
            manifest = build_runtime_manifest(paths, profile, owner_boot_id="b" * 32,
                                              owner_pid=1234, policy_pid=5678, env=env)
            status = make_status(manifest)
            status.policy_pid += 1
            calls = []
            with self.assertRaisesRegex(RuntimeError, "IDENTITY_MISMATCH"):
                invoke_fake_claim_startup(manifest, status, env, calls=calls)
            self.assertEqual(["identity"], calls)

    def test_runtime_hash_profile_and_domain_mismatches_block_claims(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            env = make_runtime_env(temp_dir)
            _, profile, paths = configure_runtime_policy(load_child_specs(env), env)
            manifest = build_runtime_manifest(paths, profile, owner_boot_id="d" * 32,
                                              owner_pid=1234, policy_pid=5678, env=env)
            cases = (
                ("model_sha256", "0" * 64, "HASH_MISMATCH", env),
                ("profile_sha256", "0" * 64, "PROFILE_MISMATCH", env),
                ("ros_domain_id", 42, "DOMAIN_MISMATCH", env),
                ("owner_boot_id", "e" * 32, "IDENTITY_MISMATCH", env),
                ("env_domain", None, "DOMAIN_MISMATCH", {**env, "ROS_DOMAIN_ID": "42"}),
            )
            for field, value, reason, runtime_env in cases:
                with self.subTest(field=field, reason=reason):
                    status = make_status(manifest)
                    if field == "model_sha256":
                        status.model_sha256 = value
                    elif field != "env_domain":
                        setattr(status, field, value)
                    calls = []
                    with self.assertRaisesRegex(RuntimeError, reason):
                        invoke_fake_claim_startup(manifest, status, runtime_env, calls=calls)
                    self.assertEqual(["identity"], calls)

    def test_active_runtime_and_unavailable_identity_block_claims(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            env = make_runtime_env(temp_dir)
            _, profile, paths = configure_runtime_policy(load_child_specs(env), env)
            manifest = build_runtime_manifest(paths, profile, owner_boot_id="c" * 32,
                                              owner_pid=1234, policy_pid=5678, env=env)
            status = make_status(manifest)
            status.authorized = True
            status.writer_present = True
            status.lowcmd_writer_gid = [1] + [0] * 15
            calls = []
            with self.assertRaisesRegex(RuntimeError, "unauthorized and writer-free"):
                invoke_fake_claim_startup(manifest, status, env, calls=calls)
            self.assertEqual(["identity"], calls)
            calls = []
            with self.assertRaises(TimeoutError):
                invoke_fake_claim_startup(
                    manifest, make_status(manifest), env, identity_available=False,
                    timeout=0.001, calls=calls
                )
            self.assertEqual([], calls)

    def test_owner_shutdown_requires_unauthed_runtime_without_writer(self):
        status = SimpleNamespace(valid=True, authorized=False, writer_present=False,
                                 lowcmd_writer_gid=[0] * 16)
        self.assertTrue(runtime_identity_allows_shutdown(status))
        for changes in (
            {"authorized": True},
            {"writer_present": True},
            {"lowcmd_writer_gid": [1] + [0] * 15},
            {"lowcmd_writer_gid": [0] * 24},
            {"valid": False},
        ):
            candidate = SimpleNamespace(**{**vars(status), **changes})
            with self.subTest(changes=changes):
                self.assertFalse(runtime_identity_allows_shutdown(candidate))
        self.assertFalse(runtime_identity_allows_shutdown(None))

    def test_systemd_owner_stop_cannot_force_kill_an_active_policy(self):
        service = (Path(__file__).parent / "systemd" / "h1-policy-owner.service").read_text()
        self.assertIn("KillMode=control-group", service)
        self.assertIn("TimeoutStopSec=infinity", service)

    def test_owner_waits_for_inactive_output_before_stopping_children(self):
        states = iter((False, True))
        holder = {}

        class FakeSupervisor:
            def __init__(self, _specs):
                self.stop_requested = False
                self.stopped = False
                holder["self"] = self

            def start(self):
                pass

            def pid_for(self, name):
                self.asserted_pid_name = name
                return 98765

            def request_stop(self, *_args):
                self.stop_requested = True

            def poll_failure(self):
                return None

            def stop_children(self):
                self.stopped = True
                self.stopped_after_safe_status = holder["last_safe"]

        def startup(*_args, **kwargs):
            kwargs["should_stop"]()
            holder["self"].request_stop()

        def shutdown_safe(**_kwargs):
            value = next(states)
            holder["last_safe"] = value
            return value

        with tempfile.TemporaryDirectory() as temp_dir, patch("policy_owner.signal.signal"):
            env = make_runtime_env(temp_dir)
            state_dir = Path(temp_dir) / "state"
            env["H1_POLICY_OWNER_STATE_DIR"] = str(state_dir)
            result = run_owner(
                env,
                poll_interval=0.001,
                supervisor_factory=FakeSupervisor,
                startup=startup,
                shutdown_safe=shutdown_safe,
            )
            manifest_count = len(list(state_dir.glob("*/runtime-manifest.json")))

        self.assertEqual(0, result)
        self.assertTrue(holder["self"].stopped)
        self.assertTrue(holder["self"].stopped_after_safe_status)
        self.assertEqual(1, manifest_count)

    def test_systemd_owner_stop_cannot_force_kill_an_active_policy(self):
        service = (Path(__file__).parent / "systemd" / "h1-policy-owner.service").read_text()
        self.assertIn("KillMode=control-group", service)
        self.assertIn("TimeoutStopSec=infinity", service)

    def test_valid_startup_commands_are_accepted(self):
        self.assertEqual(("monitor", "policy", "coordinator"),
                         tuple(spec.name for spec in load_child_specs(VALID_ENV)))

    def test_policy_command_requires_direct_pid_and_binds_owner_parameters(self):
        with self.assertRaisesRegex(ValueError, "exec the policy node directly"):
            load_child_specs({
                **VALID_ENV,
                "H1_POLICY_OWNER_BRIDGE_COMMAND": (
                    "ros2 launch pkg launch.py -p enable_hardware_output:=false "
                    "-p input_identity_mode:=claimed -p owner_boot_id:={owner_boot_id} "
                    "-p owner_pid:={owner_pid}"
                ),
            })
        specs = bind_owner_tokens(
            load_child_specs(VALID_ENV), Path("/tmp/run"), "0123456789abcdef0123456789abcdef", 321
        )
        policy = next(spec.command for spec in specs if spec.name == "policy")
        self.assertIn("owner_boot_id:=0123456789abcdef0123456789abcdef", policy)
        self.assertIn("owner_pid:=321", policy)

    def test_requires_explicit_output_authorization_policy(self):
        with self.assertRaisesRegex(ValueError, "explicit"):
            load_child_specs({**VALID_ENV, "H1_POLICY_OWNER_BRIDGE_COMMAND": "ros2 launch pkg launch.py input_identity_mode:=claimed"})

    def test_manual_output_authorization_capability_can_be_enabled_without_owner_authorizing(self):
        env = {
            **VALID_ENV,
            "H1_POLICY_OWNER_BRIDGE_COMMAND": (
                "/opt/policy-node --ros-args -p enable_hardware_output:=true "
                "-p input_identity_mode:=claimed -p owner_boot_id:={owner_boot_id} "
                "-p owner_pid:={owner_pid}"
            ),
        }
        self.assertEqual("policy", next(spec.name for spec in load_child_specs(env) if spec.name == "policy"))

    def test_rejects_output_authorization_and_navigation_enablement(self):
        for flag in ("continuous_policy_navigation:=true", "enable_navigation_commands:=true"):
            with self.subTest(flag=flag), self.assertRaises(ValueError):
                load_child_specs({
                    **VALID_ENV,
                    "H1_POLICY_OWNER_BRIDGE_COMMAND": (
                        "/opt/policy-node --ros-args -p enable_hardware_output:=false "
                        "-p input_identity_mode:=claimed -p owner_boot_id:={owner_boot_id} "
                        "-p owner_pid:={owner_pid} -p " + flag
                    ),
                })

    def test_runtime_manifest_hashes_files_profile_and_binds_distinct_pids(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest = make_manifest(temp_dir)
            self.assertEqual(100, manifest["owner_pid"])
            self.assertEqual(101, manifest["policy_pid"])
            self.assertNotEqual(manifest["owner_pid"], manifest["policy_pid"])
            self.assertEqual(hashlib.sha256(b"model").hexdigest(),
                             manifest["files"]["model"]["sha256"])
            self.assertEqual(canonical_profile_sha256(sample_profile()),
                             manifest["profile"]["profile_sha256"])

    def test_runtime_manifest_rejects_wrong_profile_lengths_and_hash(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            paths = []
            for name in ("model", "urdf", "executable"):
                path = Path(temp_dir) / name
                path.write_bytes(name.encode())
                paths.append(str(path))
            write_test_urdf(Path(paths[1]))
            invalid_profile = sample_profile()
            invalid_profile["arrays"]["speed_limits_sdk"].pop()
            with self.assertRaisesRegex(ValueError, "exactly 27"):
                build_runtime_manifest(paths, invalid_profile)

            invalid_profile = sample_profile()
            invalid_profile["profile_sha256"] = "1" * 64
            with self.assertRaisesRegex(ValueError, "does not match canonical"):
                build_runtime_manifest(paths, invalid_profile)

    def test_runtime_identity_accepts_unauthed_and_bound_writer_states(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest = make_manifest(temp_dir)
            status = make_status(manifest)
            env = {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "0"}
            self.assertEqual("OK", verify_runtime_identity(manifest, status, env))
            status.authorized = True
            status.writer_present = True
            status.lowcmd_writer_gid = [1] + [0] * 15
            self.assertEqual("OK", verify_runtime_identity(manifest, status, env))

    def test_runtime_identity_failure_codes_distinguish_unreachable_and_domain(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest = make_manifest(temp_dir)
            status = make_status(manifest)
            self.assertEqual("UNREACHABLE", verify_runtime_identity(manifest, None, {
                "ROS_DOMAIN_ID": "43", "ROS_LOCALHOST_ONLY": "0"
            }))
            self.assertEqual("DOMAIN_MISMATCH", verify_runtime_identity(manifest, status, {
                "ROS_DOMAIN_ID": "43", "ROS_LOCALHOST_ONLY": "0"
            }))
            self.assertEqual("LOCALHOST_MISMATCH", verify_runtime_identity(manifest, status, {
                "ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "1"
            }))

    def test_runtime_identity_reports_identity_profile_hash_and_writer_conflicts(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest = make_manifest(temp_dir)
            env = {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "0"}
            status = make_status(manifest)
            status.owner_pid += 1
            self.assertEqual("IDENTITY_MISMATCH", verify_runtime_identity(manifest, status, env))

            status = make_status(manifest)
            status.profile_sha256 = "0" * 64
            self.assertEqual("PROFILE_MISMATCH", verify_runtime_identity(manifest, status, env))

            status = make_status(manifest)
            status.model_sha256 = "0" * 64
            self.assertEqual("HASH_MISMATCH", verify_runtime_identity(manifest, status, env))

            status = make_status(manifest)
            status.writer_present = True
            self.assertEqual("WRITER_CONFLICT", verify_runtime_identity(manifest, status, env))

    def test_discovery_requires_one_complete_gid(self):
        from policy_owner import unique_writer_gid

        gid = list(range(24))
        self.assertEqual(gid, unique_writer_gid([Endpoint(gid)], "/topic"))
        with self.assertRaisesRegex(RuntimeError, "exactly one publisher"):
            unique_writer_gid([], "/topic")
        with self.assertRaisesRegex(RuntimeError, "exactly one publisher"):
            unique_writer_gid([Endpoint(gid), Endpoint(gid)], "/topic")
        with self.assertRaisesRegex(RuntimeError, "exposed 0 GID bytes"):
            unique_writer_gid([Endpoint([])], "/topic")
        with self.assertRaisesRegex(RuntimeError, "requires 24"):
            unique_writer_gid([Endpoint([1, 2, 3])], "/topic")

    def test_shutdown_stops_children_without_killing_owner_process(self):
        command = (sys.executable, "-c", "import time; time.sleep(60)")
        supervisor = ProcessSupervisor((ChildSpec("fake", command),))
        supervisor.start()
        child = supervisor.children[0][1]
        owner_pid = os.getpid()
        supervisor.request_stop(signal.SIGTERM)
        supervisor.stop_children()
        self.assertIsNotNone(child.poll())
        self.assertEqual(0, subprocess.run(["kill", "-0", str(owner_pid)], check=False,
                                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode)

    def test_shutdown_bounds_term_wait_for_child_that_ignores_term(self):
        command = (sys.executable, "-c",
                   "import signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); "
                   "print('ready', flush=True); time.sleep(60)")
        popen = lambda cmd, **kwargs: subprocess.Popen(
            cmd, stdout=subprocess.PIPE, text=True, **kwargs
        )
        supervisor = ProcessSupervisor((ChildSpec("ignores-term", command),), popen=popen)
        supervisor.start()
        child = supervisor.children[0][1]
        self.addCleanup(child.stdout.close)
        self.assertEqual("ready\n", child.stdout.readline())
        started = time.monotonic()
        supervisor.stop_children(grace_seconds=0.1)
        self.assertLess(time.monotonic() - started, 2.0)
        self.assertIsNotNone(child.poll())


if __name__ == "__main__":
    unittest.main()
