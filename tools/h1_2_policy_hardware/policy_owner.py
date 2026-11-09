#!/usr/bin/env python3
"""Supervise the persistent policy stack without authorizing hardware output."""

import argparse
import hashlib
import json
import math
import os
import re
import shlex
import signal
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping, Optional, Sequence

from train_guard_profile import JOINT_NAMES_SDK


COMMAND_ENV = {
    "monitor": "H1_POLICY_OWNER_MONITOR_COMMAND",
    "policy": "H1_POLICY_OWNER_BRIDGE_COMMAND",
    "coordinator": "H1_POLICY_OWNER_COORDINATOR_COMMAND",
}
CMD_TOPIC = "/h1_2/policy/cmd_vel"
HEARTBEAT_TOPIC = "/h1_2/policy_gate_alive"
CLAIM_SERVICE = "/h1_2/policy/claim_input"
COORDINATOR_CLAIM_SERVICE = "/h1_2/policy/confirm_input_claim"
HUMBLE_RMW_GID_STORAGE_SIZE = 24


@dataclass(frozen=True)
class ChildSpec:
    name: str
    command: tuple[str, ...]


def _parse_command(name: str, env_name: str, env: Mapping[str, str]) -> ChildSpec:
    raw = env.get(env_name, "").strip()
    if not raw:
        raise ValueError(f"{env_name} must be configured")
    command = tuple(shlex.split(raw))
    if not command:
        raise ValueError(f"{env_name} must contain a command")
    return ChildSpec(name, command)


def load_child_specs(env: Mapping[str, str]) -> tuple[ChildSpec, ...]:
    specs = tuple(_parse_command(name, variable, env) for name, variable in COMMAND_ENV.items())
    policy = next(spec.command for spec in specs if spec.name == "policy")
    output_flags = [arg.split(":=", 1)[1].lower() for arg in policy
                    if arg.startswith("enable_hardware_output:=")]
    if len(output_flags) != 1 or output_flags[0] not in ("false", "true"):
        raise ValueError(
            "policy bridge command must contain exactly one explicit "
            "enable_hardware_output:=true|false setting"
        )
    if Path(policy[0]).name == "ros2":
        raise ValueError("policy command must exec the policy node directly so its PID is verifiable")
    if not Path(policy[0]).is_absolute():
        raise ValueError("policy node executable path must be absolute")
    for flag in ("continuous_policy_navigation", "enable_navigation_commands"):
        values = [arg.split(":=", 1)[1].lower() for arg in policy if arg.startswith(flag + ":=")]
        if any(value != "false" for value in values):
            raise ValueError("policy navigation must remain disabled until explicitly commissioned")
    identity_modes = [arg for arg in policy if arg.startswith("input_identity_mode:=")]
    if identity_modes != ["input_identity_mode:=claimed"]:
        raise ValueError("policy bridge must start in claimed input identity mode")
    for name in ("owner_boot_id", "owner_pid"):
        values = [arg for arg in policy if arg.startswith(name + ":=")]
        expected = f"{name}:={{{name}}}"
        if values != [expected]:
            raise ValueError(f"policy command must receive {expected}")
    coordinator = next(spec.command for spec in specs if spec.name == "coordinator")
    owner_ids = [arg for arg in coordinator if arg.startswith("owner_boot_id:=")]
    if owner_ids != ["owner_boot_id:={owner_boot_id}"]:
        raise ValueError("coordinator command must receive owner_boot_id:={owner_boot_id}")
    return specs


def unique_writer_gid(endpoint_infos, topic: str) -> list[int]:
    if len(endpoint_infos) != 1:
        raise RuntimeError(f"expected exactly one publisher for {topic}; found {len(endpoint_infos)}")
    gid = bytes(endpoint_infos[0].endpoint_gid)
    if len(gid) != HUMBLE_RMW_GID_STORAGE_SIZE:
        raise RuntimeError(
            f"publisher for {topic} exposed {len(gid)} GID bytes; "
            f"ROS Humble requires {HUMBLE_RMW_GID_STORAGE_SIZE}"
        )
    return list(gid)


PROFILE_ARRAY_KEYS = (
    "speed_limits_sdk",
    "position_lower_sdk",
    "position_upper_sdk",
    "default_positions_sdk",
)
SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")
FAILURE_CODES = {
    "UNREACHABLE", "DOMAIN_MISMATCH", "LOCALHOST_MISMATCH", "IDENTITY_MISMATCH",
    "PROFILE_MISMATCH", "HASH_MISMATCH", "WRITER_CONFLICT",
}


def _valid_sha256(value: object) -> bool:
    return isinstance(value, str) and SHA256_RE.fullmatch(value) is not None


def _canonical_profile(profile: Mapping[str, object]) -> dict:
    if set(profile) != {"joint_names", "arrays", "source_sha256", "profile_sha256"}:
        raise ValueError("profile must contain exactly joint_names, arrays, source_sha256 and profile_sha256")
    joint_names = profile["joint_names"]
    arrays = profile["arrays"]
    if not isinstance(joint_names, list) or len(joint_names) != 27:
        raise ValueError("profile joint_names must contain exactly 27 names")
    if (any(not isinstance(name, str) for name in joint_names) or
            joint_names != JOINT_NAMES_SDK or len(set(joint_names)) != 27):
        raise ValueError("profile joint_names do not match the reviewed SDK motor order")
    if not isinstance(arrays, Mapping) or set(arrays) != set(PROFILE_ARRAY_KEYS):
        raise ValueError("profile arrays must contain the four reviewed 27-value arrays")
    normalized_arrays = {}
    for key in PROFILE_ARRAY_KEYS:
        values = arrays[key]
        if not isinstance(values, list) or len(values) != 27:
            raise ValueError(f"profile array {key} must contain exactly 27 values")
        normalized = []
        for value in values:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"profile array {key} contains a non-finite or non-numeric value")
            try:
                number = float(value)
            except (OverflowError, ValueError):
                raise ValueError(f"profile array {key} contains a non-finite or non-numeric value") from None
            if not math.isfinite(number):
                raise ValueError(f"profile array {key} contains a non-finite or non-numeric value")
            normalized.append(number)
        normalized_arrays[key] = normalized
    if not _valid_sha256(profile["source_sha256"]):
        raise ValueError("profile source_sha256 must be a 64-character SHA256")
    if not _valid_sha256(profile["profile_sha256"]):
        raise ValueError("profile profile_sha256 must be a 64-character SHA256")
    return {
        "joint_names": list(joint_names),
        "arrays": normalized_arrays,
        "source_sha256": profile["source_sha256"].lower(),
    }


def canonical_profile_sha256(profile: Mapping[str, object]) -> str:
    canonical = _canonical_profile(profile)
    payload = {"joint_names": canonical["joint_names"], "arrays": canonical["arrays"]}
    data = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise ValueError("ROS_LOCALHOST_ONLY must be a boolean environment value")


def build_runtime_manifest(
    deployment_paths: Sequence[str],
    profile: Mapping[str, object],
    *,
    owner_boot_id: Optional[str] = None,
    owner_pid: Optional[int] = None,
    policy_pid: Optional[int] = None,
    node_fqn: str = "/h1_2_policy_node",
    env: Optional[Mapping[str, str]] = None,
) -> dict:
    """Build a manifest for model, URDF and direct policy executable in that order.

    `policy_pid` is supplied only after the direct executable is spawned. A missing
    PID keeps the manifest useful for offline review but cannot pass identity checks.
    """
    if len(deployment_paths) != 3:
        raise ValueError("deployment_paths must be [model, urdf, executable]")
    canonical = _canonical_profile(profile)
    computed_profile_hash = canonical_profile_sha256(profile)
    if computed_profile_hash != profile["profile_sha256"].lower():
        raise ValueError("profile_sha256 does not match canonical joint arrays")

    file_names = ("model", "urdf", "executable")
    files = {}
    for key, raw_path in zip(file_names, deployment_paths):
        path = Path(raw_path).expanduser().resolve(strict=True)
        if not path.is_file() or path.stat().st_size == 0 or not os.access(path, os.R_OK):
            raise ValueError(f"deployment {key} must be a readable, nonempty regular file: {path}")
        if key == "executable" and not os.access(path, os.X_OK):
            raise ValueError(f"policy executable is not executable: {path}")
        files[key] = {"path": str(path), "sha256": _sha256_file(path)}

    runtime_env = os.environ if env is None else env
    try:
        ros_domain_id = int(runtime_env.get("ROS_DOMAIN_ID", "0"))
        ros_localhost_only = _parse_bool(runtime_env.get("ROS_LOCALHOST_ONLY", "0"))
    except ValueError as exc:
        raise ValueError(f"invalid ROS environment: {exc}") from None
    if ros_domain_id < 0 or ros_domain_id > 232:
        raise ValueError("ROS_DOMAIN_ID must be between 0 and 232")
    if not node_fqn.startswith("/") or not node_fqn:
        raise ValueError("node_fqn must be an absolute ROS name")
    current_owner_pid = os.getpid() if owner_pid is None else owner_pid
    if (isinstance(current_owner_pid, bool) or not isinstance(current_owner_pid, int) or
            current_owner_pid <= 0 or
            (policy_pid is not None and
             (isinstance(policy_pid, bool) or not isinstance(policy_pid, int) or policy_pid <= 0))):
        raise ValueError("owner_pid and policy_pid must be positive process IDs")
    if policy_pid is not None and policy_pid == current_owner_pid:
        raise ValueError("policy_pid must identify a child process distinct from owner_pid")
    boot_id = owner_boot_id or uuid.uuid4().hex
    if re.fullmatch(r"[0-9a-fA-F]{32}", boot_id) is None:
        raise ValueError("owner_boot_id must be a uuid4 hex string")
    return {
        "schema_version": 1,
        "owner_boot_id": boot_id,
        "owner_pid": current_owner_pid,
        "policy_pid": policy_pid,
        "node_fqn": node_fqn,
        "ros_domain_id": ros_domain_id,
        "ros_localhost_only": ros_localhost_only,
        "files": files,
        "profile": {
            **canonical,
            "profile_sha256": computed_profile_hash,
        },
    }


def _response_failure_code(response) -> Optional[str]:
    reason = getattr(response, "reason", "")
    normalized = str(reason).strip().upper()
    for code in FAILURE_CODES:
        if normalized == code or normalized.startswith(code + ":") or (" " + code + " ") in (" " + normalized + " "):
            return code
    return None


def _runtime_value(response, name):
    value = getattr(response, name, None)
    if value is None:
        raise ValueError(f"runtime identity missing {name}")
    return value


def verify_runtime_identity(manifest: Mapping[str, object], response, env: Mapping[str, str]) -> str:
    """Compare runtime status to its manifest and only report evidenced failures."""
    if response is None or isinstance(response, BaseException):
        return "UNREACHABLE"
    reason_code = _response_failure_code(response)
    if reason_code == "UNREACHABLE":
        return "UNREACHABLE"
    try:
        expected_domain = int(manifest["ros_domain_id"])
        expected_localhost = manifest["ros_localhost_only"]
        if not isinstance(expected_localhost, bool):
            return "IDENTITY_MISMATCH"
        env_domain = int(env.get("ROS_DOMAIN_ID", str(expected_domain)))
        env_localhost = _parse_bool(env.get("ROS_LOCALHOST_ONLY", str(int(expected_localhost))))
    except (KeyError, TypeError, ValueError):
        return "IDENTITY_MISMATCH"
    if env_domain != expected_domain:
        return "DOMAIN_MISMATCH"
    if env_localhost != expected_localhost:
        return "LOCALHOST_MISMATCH"

    try:
        status_domain = int(_runtime_value(response, "ros_domain_id"))
        status_localhost = _runtime_value(response, "ros_localhost_only")
        if not isinstance(status_localhost, bool):
            return "IDENTITY_MISMATCH"
    except (TypeError, ValueError):
        return "IDENTITY_MISMATCH"
    if status_domain != env_domain:
        return "DOMAIN_MISMATCH"
    if status_localhost != env_localhost:
        return "LOCALHOST_MISMATCH"

    if reason_code:
        return reason_code
    try:
        identity_pairs = (
            ("owner_boot_id", "owner_boot_id"),
            ("owner_pid", "owner_pid"),
            ("policy_pid", "policy_pid"),
            ("node_fqn", "node_fqn"),
        )
        for manifest_key, response_key in identity_pairs:
            expected = manifest.get(manifest_key)
            actual = _runtime_value(response, response_key)
            if expected is None or actual != expected:
                return "IDENTITY_MISMATCH"
        if int(manifest["owner_pid"]) == int(manifest["policy_pid"]):
            return "IDENTITY_MISMATCH"
    except (KeyError, TypeError, ValueError, AttributeError):
        return "IDENTITY_MISMATCH"

    try:
        files = manifest["files"]
        for key, path_field, hash_field in (
            ("model", "model_path", "model_sha256"),
            ("urdf", "urdf_path", "urdf_sha256"),
            ("executable", "executable_path", "executable_sha256"),
        ):
            expected_file = files[key]
            if (_runtime_value(response, path_field) != expected_file["path"] or
                    _runtime_value(response, hash_field).lower() != expected_file["sha256"]):
                return "HASH_MISMATCH"
    except (KeyError, TypeError, ValueError, AttributeError):
        return "IDENTITY_MISMATCH"

    try:
        profile = manifest["profile"]
        expected_profile_hash = canonical_profile_sha256(profile)
        if expected_profile_hash != profile["profile_sha256"].lower():
            return "PROFILE_MISMATCH"
        if _runtime_value(response, "profile_sha256").lower() != expected_profile_hash:
            return "PROFILE_MISMATCH"
    except (KeyError, TypeError, ValueError, AttributeError):
        return "PROFILE_MISMATCH"

    try:
        authorized = bool(_runtime_value(response, "authorized"))
        writer_present = bool(_runtime_value(response, "writer_present"))
        writer_gid = bytes(_runtime_value(response, "lowcmd_writer_gid"))
    except (TypeError, ValueError):
        return "WRITER_CONFLICT"
    if len(writer_gid) != 16:
        return "WRITER_CONFLICT"
    if authorized:
        if not writer_present or not any(writer_gid):
            return "WRITER_CONFLICT"
    elif writer_present or any(writer_gid):
        return "WRITER_CONFLICT"
    try:
        if not bool(_runtime_value(response, "valid")):
            return "IDENTITY_MISMATCH"
    except ValueError:
        return "IDENTITY_MISMATCH"
    return "OK"


def bind_owner_tokens(specs: Sequence[ChildSpec], run_dir: Path, owner_boot_id: str,
                      owner_pid: int) -> tuple[ChildSpec, ...]:
    replacements = {
        "{run_dir}": str(run_dir),
        "{owner_boot_id}": owner_boot_id,
        "{owner_pid}": str(owner_pid),
    }
    bound = []
    for spec in specs:
        command = []
        for arg in spec.command:
            for token, value in replacements.items():
                arg = arg.replace(token, value)
            command.append(arg)
        bound.append(ChildSpec(spec.name, tuple(command)))
    return tuple(bound)


def _spin_until(node, future, deadline: float, description: str,
                should_stop: Callable[[], bool] = lambda: False):
    import rclpy

    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        if should_stop():
            raise InterruptedError(f"owner stop requested while waiting for {description}")
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError(f"timed out waiting for {description}")
    result = future.result()
    if result is None:
        raise RuntimeError(f"{description} returned no response")
    return result


def claim_and_start_coordinator(owner_boot_id: str, timeout_seconds: float = 15.0,
                                should_stop: Callable[[], bool] = lambda: False) -> None:
    """Claim the coordinator's discovered final writers before enabling heartbeat."""
    import rclpy
    from h1_2_policy_interfaces.srv import ClaimPolicyInput
    from rclpy.node import Node

    rclpy.init(args=None)
    node = Node("h1_2_policy_owner_claim")
    claim_client = node.create_client(ClaimPolicyInput, CLAIM_SERVICE)
    coordinator_claim_client = node.create_client(ClaimPolicyInput, COORDINATOR_CLAIM_SERVICE)
    deadline = time.monotonic() + timeout_seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if should_stop():
                raise InterruptedError("owner stop requested before publisher claim")
            rclpy.spin_once(node, timeout_sec=0.05)
            if (claim_client.service_is_ready() and coordinator_claim_client.service_is_ready()):
                try:
                    cmd_gid = unique_writer_gid(node.get_publishers_info_by_topic(CMD_TOPIC), CMD_TOPIC)
                    heartbeat_gid = unique_writer_gid(
                        node.get_publishers_info_by_topic(HEARTBEAT_TOPIC), HEARTBEAT_TOPIC
                    )
                    break
                except RuntimeError:
                    continue
        else:
            raise TimeoutError("coordinator publishers or input claim services were unavailable")

        request = ClaimPolicyInput.Request()
        request.owner_boot_id = owner_boot_id
        request.cmd_writer_gid = cmd_gid
        request.heartbeat_writer_gid = heartbeat_gid
        if cmd_gid == heartbeat_gid:
            raise RuntimeError("command and heartbeat must have distinct publisher GIDs")
        claim = _spin_until(node, claim_client.call_async(request), deadline, "policy input claim",
                            should_stop)
        if not claim.success:
            raise RuntimeError(f"policy input claim rejected: {claim.reason}")
        coordinator_claim = _spin_until(
            node, coordinator_claim_client.call_async(request), deadline,
            "coordinator input claim", should_stop
        )
        if not coordinator_claim.success:
            raise RuntimeError(f"coordinator rejected input claim: {coordinator_claim.reason}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def runtime_identity_allows_shutdown(response) -> bool:
    """A policy process may be stopped only after output is observably inactive."""
    try:
        return (
            bool(response.valid)
            and not bool(response.authorized)
            and not bool(response.writer_present)
            and len(response.lowcmd_writer_gid) == 16
            and not any(response.lowcmd_writer_gid)
        )
    except (AttributeError, TypeError):
        return False


def policy_runtime_allows_shutdown(timeout_seconds: float = 2.0) -> bool:
    """Fail closed when the policy runtime cannot prove it has no active writer."""
    import rclpy
    from h1_2_policy_interfaces.srv import PolicyRuntimeIdentity
    from rclpy.node import Node

    rclpy.init(args=None)
    node = Node("h1_2_policy_owner_shutdown_check")
    client = node.create_client(PolicyRuntimeIdentity, "/h1_2/policy/runtime_identity")
    deadline = time.monotonic() + timeout_seconds
    try:
        while rclpy.ok() and not client.service_is_ready() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not client.service_is_ready():
            return False
        response = _spin_until(node, client.call_async(PolicyRuntimeIdentity.Request()), deadline,
                               "policy runtime shutdown status")
        return runtime_identity_allows_shutdown(response)
    except Exception:
        return False
    finally:
        node.destroy_node()
        rclpy.shutdown()


class ProcessSupervisor:
    """Run children in the owner's systemd process group; never restart them."""

    def __init__(self, specs: Sequence[ChildSpec], popen: Callable = subprocess.Popen):
        self.specs = tuple(specs)
        self.popen = popen
        self.children: list[tuple[ChildSpec, subprocess.Popen]] = []
        self.stop_requested = False

    def start(self) -> None:
        try:
            for spec in self.specs:
                process = self.popen(spec.command, start_new_session=False)
                self.children.append((spec, process))
                print(f"started {spec.name}: pid={process.pid}", flush=True)
        except Exception:
            self.stop_children()
            raise

    def request_stop(self, signum=None, frame=None) -> None:
        self.stop_requested = True

    def poll_failure(self) -> Optional[tuple[str, int]]:
        for spec, process in self.children:
            return_code = process.poll()
            if return_code is not None:
                return spec.name, return_code
        return None

    def stop_children(self, grace_seconds: float = 5.0) -> None:
        live = [(spec, process) for spec, process in self.children if process.poll() is None]
        for _, process in live:
            try:
                process.send_signal(signal.SIGTERM)
            except ProcessLookupError:
                pass

        deadline = time.monotonic() + grace_seconds
        for _, process in live:
            remaining = max(0.0, deadline - time.monotonic())
            try:
                process.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                pass

        for _, process in live:
            if process.poll() is None:
                try:
                    process.send_signal(signal.SIGKILL)
                except ProcessLookupError:
                    pass
        for _, process in live:
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                pass


def run(env: Mapping[str, str], poll_interval: float = 0.2,
        supervisor_factory: Callable = ProcessSupervisor,
        startup: Callable = claim_and_start_coordinator,
        shutdown_safe: Callable = policy_runtime_allows_shutdown) -> int:
    specs = load_child_specs(env)
    owner_boot_id = uuid.uuid4().hex
    state_root = Path(env.get("H1_POLICY_OWNER_STATE_DIR", "~/.local/state/h1-policy-owner")).expanduser()
    run_dir = state_root / owner_boot_id
    run_dir.mkdir(mode=0o700, parents=True, exist_ok=False)
    specs = bind_owner_tokens(specs, run_dir, owner_boot_id, os.getpid())
    supervisor = supervisor_factory(specs)
    signal.signal(signal.SIGTERM, supervisor.request_stop)
    signal.signal(signal.SIGINT, supervisor.request_stop)
    print(f"policy owner boot id: {owner_boot_id}", flush=True)
    supervisor.start()
    startup_failure = None
    try:
        try:
            startup(
                owner_boot_id,
                timeout_seconds=float(env.get("H1_POLICY_OWNER_STARTUP_TIMEOUT_S", "15")),
                should_stop=lambda: supervisor.stop_requested,
            )
        except Exception as exc:
            startup_failure = exc
            print(f"owner startup failed; waiting for verified inactive output: {exc}",
                  file=sys.stderr, flush=True)
        reported_child_failure = None
        while True:
            failure = supervisor.poll_failure()
            if failure is not None and failure != reported_child_failure:
                reported_child_failure = failure
                name, return_code = failure
                print(f"child exited; no automatic restart: {name} rc={return_code}; "
                      "waiting for verified inactive output", file=sys.stderr, flush=True)
            if supervisor.stop_requested or startup_failure is not None or failure is not None:
                try:
                    safe_to_stop = shutdown_safe()
                except Exception as exc:
                    safe_to_stop = False
                    print(f"runtime shutdown check failed; keeping owner alive: {exc}",
                          file=sys.stderr, flush=True)
                if safe_to_stop:
                    break
                time.sleep(poll_interval)
                continue
            time.sleep(poll_interval)
        return 1 if startup_failure is not None or reported_child_failure is not None else 0
    finally:
        supervisor.stop_children()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="validate configured commands without starting them")
    args = parser.parse_args(argv)
    try:
        specs = load_child_specs(os.environ)
    except ValueError as exc:
        parser.error(str(exc))
    if args.check:
        for spec in specs:
            print(f"{spec.name}: {shlex.join(spec.command)}")
        return 0
    return run(os.environ)


if __name__ == "__main__":
    raise SystemExit(main())
