import argparse
import errno
import fcntl
import json
import os
import signal
import signal
import shlex
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import rclpy
from cv_bridge import CvBridge
try:
    from h1_2_icp_localization.msg import LocalizationStatus
    from h1_2_nav_msgs.srv import ExecuteGesture, RouteStatus, StartRoute, StopRoute, SaveMemoryPoint
    ROS_CUSTOM_MESSAGES_AVAILABLE = True
except ModuleNotFoundError:
    LocalizationStatus = ExecuteGesture = RouteStatus = StartRoute = StopRoute = SaveMemoryPoint = None
    ROS_CUSTOM_MESSAGES_AVAILABLE = False
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

from h1_2_nav_tools.camera_stream import (
    CameraFrameStore,
    encode_depth_jpeg,
    encode_rgb_jpeg,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_ros import Buffer, TransformListener


CONFIG_PATH = Path.home() / ".ros" / "h1_2_nav_web_gui.json"
CONFIG_WORKFLOW_VERSION = 2
LOCALIZATION_REQUIRED_STABLE_SAMPLES = 5
LOCALIZATION_MAX_SCORE = 0.15
LOCALIZATION_STATUS_TIMEOUT_SEC = 10.0
LOCALIZATION_TF_TARGET_FRAME = "map"
LOCALIZATION_TF_SOURCE_FRAME = "camera_init"
GESTURE_OPTIONS = ["", "wave_hand", "shake_hand", "grab_pull"]
HEARTBEAT_TIMEOUT_SEC = max(0.5, float(os.environ.get("H1_2_WEB_HEARTBEAT_TIMEOUT_SEC", "2.0")))
def configure_ros_environment():
    os.environ["ROS_LOCALHOST_ONLY"] = "0"
    os.environ.setdefault("ROS_DOMAIN_ID", "0")
    rmw_implementation = os.environ.get("H1_2_RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp").strip()
    if rmw_implementation not in ("rmw_cyclonedds_cpp", "rmw_fastrtps_cpp"):
        raise RuntimeError(f"Unsupported RMW implementation: {rmw_implementation}")
    os.environ["RMW_IMPLEMENTATION"] = rmw_implementation
    os.environ.pop("ROS_HOSTNAME", None)
    os.environ.pop("ROS_IP", None)
    if rmw_implementation == "rmw_cyclonedds_cpp":
        workspace_root = find_workspace_root()
        configured_dds = os.environ.get("H1_2_DDS_CONFIG", "").strip()
        dds_config = Path(configured_dds).expanduser() if configured_dds else workspace_root / "config" / "cyclonedds_auto.xml"
        if not dds_config.is_file():
            raise RuntimeError(f"Cyclone DDS config does not exist: {dds_config}")
        os.environ["CYCLONEDDS_URI"] = f"file://{dds_config}"
    else:
        os.environ.pop("CYCLONEDDS_URI", None)
    os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)


def find_workspace_root():
    candidates = []
    env_root = os.environ.get("H1_2_NAV_WS")
    if env_root:
        candidates.append(Path(env_root))
    cwd = Path.cwd()
    candidates.extend([Path(__file__).resolve().parents[3], cwd, *cwd.parents])
    for candidate in candidates:
        if (candidate / "run_ai_slam_nav.sh").exists():
            return candidate
    return cwd


def point_sort_key(name):
    if name == "START":
        return (0, name)
    return (1, name)


def normalize_memory_route(value):
    if isinstance(value, str):
        names = value.split(",")
    else:
        names = value or []
    return ",".join(
        name
        for name in (str(item).strip() for item in names)
        if name and name != "START"
    )


def task_points_file(workspace_root, profile):
    """Return a readable per-task point file path."""
    if profile.get("id") == "default":
        return workspace_root / "maps" / "memory_points.json"
    name = str(profile.get("name") or profile.get("id") or "task").strip()
    safe_name = "".join("_" if char in '/\\:*?\"<>|' else char for char in name).strip(" .")
    return workspace_root / "maps" / "tasks" / f"{safe_name or 'task'}.json"


def normalize_task_profiles(config):
    """Migrate the single route and derive launch arguments from the selected task."""
    # Older GUI versions omitted this argument and inherited the shell default.
    legacy_turns = {}
    for entry in str(config.get("turn_route", "B:ccw,A:cw")).split(","):
        if ":" in entry:
            point, direction = (part.strip() for part in entry.split(":", 1))
            if point and direction.lower() in ("cw", "ccw"):
                legacy_turns[point] = direction.lower()
    profiles = config.get("task_profiles")
    if profiles is None:
        names = list(dict.fromkeys(normalize_memory_route(config.get("memory_route", "")).split(",")))
        gestures = {}
        for entry in str(config.get("gesture_route", "")).split(","):
            if ":" in entry:
                name, gesture = entry.split(":", 1)
                gestures[name.strip()] = gesture.strip()
        profiles = [{"id": "default", "name": "默认任务", "steps": [
            {"point": name, "gesture": gestures.get(name, "")} for name in names if name
        ]}]
    if not isinstance(profiles, list) or not 1 <= len(profiles) <= 100:
        raise ValueError("请保留 1 到 100 个任务。")
    cleaned, ids, names = [], set(), set()
    for profile in profiles:
        if not isinstance(profile, dict):
            raise ValueError("任务格式无效。")
        task_id = profile.get("id", "")
        name = profile.get("name", "")
        if not isinstance(task_id, str) or not task_id or len(task_id) > 80 or task_id in ids:
            raise ValueError("任务标识无效或重复。")
        if not isinstance(name, str) or not name.strip() or len(name.strip()) > 80 or name.strip() in names:
            raise ValueError("任务名称不能为空、重复或超过 80 个字符。")
        steps = profile.get("steps", [])
        if not isinstance(steps, list):
            raise ValueError("任务点位格式无效。")
        points, cleaned_steps = set(), []
        for step in steps:
            if not isinstance(step, dict):
                raise ValueError("任务点位格式无效。")
            point, gesture = step.get("point", ""), step.get("gesture", "")
            if not isinstance(point, str) or not point.strip() or any(c in point for c in ",:\n\r"):
                raise ValueError("任务中的点位名称无效。")
            point = point.strip()
            if point == "START" or point in points:
                raise ValueError("START 不能作为巡航点，同一任务不能重复选择同一个点位。")
            if not isinstance(gesture, str) or any(c in gesture for c in ",:\n\r"):
                raise ValueError("任务中的动作格式无效。")
            turn = step.get("turn", legacy_turns.get(point, ""))
            if not isinstance(turn, str) or turn not in ("", "cw", "ccw"):
                raise ValueError("到点转向只能为不转向、cw（顺时针）或 ccw（逆时针）。")
            points.add(point)
            cleaned_steps.append({"point": point, "gesture": gesture.strip(), "turn": turn})
        ids.add(task_id)
        names.add(name.strip())
        cleaned_profile = {"id": task_id, "name": name.strip(), "steps": cleaned_steps}
        if "memory_points_file" in profile:
            cleaned_profile["memory_points_file"] = str(profile.get("memory_points_file", "")).strip()
        cleaned.append(cleaned_profile)
    selected_id = config.get("selected_task_id", cleaned[0]["id"])
    selected = next((task for task in cleaned if task["id"] == selected_id), None)
    if selected is None:
        raise ValueError("所选任务不存在，请重新选择。")
    config["task_profiles"] = cleaned
    config["selected_task_id"] = selected["id"]
    config["memory_route"] = ",".join(step["point"] for step in selected["steps"])
    config["gesture_route"] = ",".join(f'{step["point"]}:{step["gesture"]}' for step in selected["steps"] if step["gesture"])
    config["turn_route"] = ",".join(f'{step["point"]}:{step["turn"]}' for step in selected["steps"] if step["turn"])
    return selected


def task_execution_snapshot(config):
    selected = normalize_task_profiles(config)
    return {"id": selected["id"], "name": selected["name"],
            "memory_route": config["memory_route"], "gesture_route": config["gesture_route"], "turn_route": config["turn_route"]}


def bind_http_server(host, port, handler, port_tries):
    for offset in range(port_tries):
        candidate = port + offset
        try:
            return ThreadingHTTPServer((host, candidate), handler), candidate
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE or offset == port_tries - 1:
                raise
            print(f"Port {candidate} is already in use, trying {candidate + 1}...", flush=True)
    raise OSError(errno.EADDRINUSE, "no free HTTP port found")


def normalize_ssh_target(value):
    text = str(value or "").strip()
    if text.startswith("ssh "):
        parts = shlex.split(text)
        if len(parts) >= 2 and parts[0] == "ssh":
            text = parts[-1]
    return text


def normalize_voice_workspace(value):
    text = str(value or "").strip() or "~/ros2_ws"
    if text in ("~/h1_nav2_ros2_ws", "/home/unitree/h1_nav2_ros2_ws"):
        return "~/ros2_ws"
    return text


def quote_remote_path(path):
    text = str(path or "").strip() or "~/ros2_ws"
    if text == "~":
        return "$HOME"
    if text.startswith("~/"):
        rest = text[2:]
        return "$HOME" + ("/" + shlex.quote(rest) if rest else "")
    return shlex.quote(text)


def ancestor_pids(pid):
    ancestors = set()
    current = pid
    while current > 1:
        try:
            stat = Path(f"/proc/{current}/stat").read_text(encoding="utf-8")
            parent = int(stat.rsplit(") ", 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            break
        if parent <= 1 or parent in ancestors:
            break
        ancestors.add(parent)
        current = parent
    return ancestors


def find_existing_web_gui_pids():
    current_pid = os.getpid()
    protected_pids = {current_pid, *ancestor_pids(current_pid)}
    pids = []
    proc_root = Path("/proc")
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid in protected_pids:
            continue
        try:
            raw_cmdline = (entry / "cmdline").read_bytes()
        except OSError:
            continue
        if not raw_cmdline:
            continue
        cmdline = raw_cmdline.replace(b"\0", b" ").decode("utf-8", errors="replace")
        if "h1_2_nav_web_gui" in cmdline:
            pids.append(pid)
    return sorted(set(pids))


def stop_existing_web_guis(timeout=2.0):
    pids = find_existing_web_gui_pids()
    if not pids:
        return []

    print(f"Stopping existing h1_2_nav_web_gui processes: {', '.join(map(str, pids))}", flush=True)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except PermissionError as exc:
            print(f"Could not terminate old Web GUI pid {pid}: {exc}", flush=True)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = [pid for pid in pids if Path(f"/proc/{pid}").exists()]
        if not remaining:
            return pids
        time.sleep(0.05)

    for pid in pids:
        if not Path(f"/proc/{pid}").exists():
            continue
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except PermissionError as exc:
            print(f"Could not kill old Web GUI pid {pid}: {exc}", flush=True)
    return pids


class LogStore:
    def __init__(self, limit=2000):
        self.limit = limit
        self.seq = 0
        self.entries = []
        self.lock = threading.Lock()

    def add(self, source, message):
        if source == "http":
            return
        with self.lock:
            self.seq += 1
            entry = {
                "seq": self.seq,
                "time": time.strftime("%H:%M:%S"),
                "source": source,
                "message": message,
            }
            self.entries.append(entry)
            if len(self.entries) > self.limit:
                self.entries = self.entries[-self.limit :]

    def since(self, seq):
        with self.lock:
            return [entry for entry in self.entries if entry["seq"] > seq]


class RosServiceBridge:
    def __init__(self, logs, config):
        self.logs = logs
        if not rclpy.ok():
            configure_ros_environment()
            rclpy.init(args=None)
        self.node = Node(os.environ.get("H1_2_WEB_NODE_NAME", "h1_2_nav_web_gui"))
        self.cv_bridge = CvBridge()
        self.camera_frames = CameraFrameStore()
        self.camera_subscriptions = []
        self.camera_topics = {}
        self.camera_preview_enabled = False
        self.localization_lock = threading.Lock()
        self.localization_stable_samples = 0
        self.latest_localization_status = None
        self.latest_localization_time = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.start_client = self.node.create_client(StartRoute, "start_memory_route")
        self.stop_client = self.node.create_client(StopRoute, "stop_memory_route")
        self.status_client = self.node.create_client(RouteStatus, "memory_route_status")
        self.teaching_start_client = self.node.create_client(StartRoute, "start_teaching_route")
        self.teaching_stop_client = self.node.create_client(StopRoute, "stop_teaching_route")
        self.teaching_status_client = self.node.create_client(RouteStatus, "teaching_route_status")
        self.teaching_record_start_client = self.node.create_client(Trigger, "start_teaching_path_recording")
        self.teaching_record_stop_save_client = self.node.create_client(Trigger, "stop_and_save_teaching_path")
        self.gesture_client = self.node.create_client(ExecuteGesture, "/h1_2_loco/execute_gesture")
        self.save_point_client = self.node.create_client(SaveMemoryPoint, "save_memory_point")
        self.policy_service_lock = threading.Lock()
        self.localization_sub = self.node.create_subscription(
            LocalizationStatus,
            "/icp_localization/status",
            self._on_localization_status,
            10,
        )
        self.configure_camera(config)

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def route_start(self, online_camera_init=False):
        localization = self.localization_status(online_camera_init=online_camera_init)
        if not localization["ready"]:
            return {"success": False, "message": localization["message"]}
        return self._call_simple("start_memory_route", self.start_client, StartRoute.Request())

    def save_memory_point(self, name, overwrite=False):
        request = SaveMemoryPoint.Request()
        request.name = str(name).strip()
        request.overwrite = bool(overwrite)
        return self._call_simple("save_memory_point", self.save_point_client, request, include_pose=True)

    def route_stop(self):
        return self._call_simple("stop_memory_route", self.stop_client, StopRoute.Request())

    def route_status(self):
        return self._route_status("memory_route_status", self.status_client)

    def policy_route_status(self):
        return self._call_policy_service("route_status")

    def policy_route_start(self):
        return self._call_policy_service("route_start")

    def policy_route_stop(self):
        return self._call_policy_service("route_stop")

    def policy_localization_status(self, online_camera_init=False):
        return self._call_policy_service("localization", {"online_camera_init": bool(online_camera_init)})

    def policy_save_memory_point(self, name, overwrite=False):
        return self._call_policy_service(
            "save_memory_point", {"name": str(name).strip(), "overwrite": bool(overwrite)}
        )

    def policy_control_status(self):
        result = self._call_policy_service("policy_status")
        if not result.get("success"):
            return {"available": False, "ready": False, "message": result.get("message", "ONNX Policy bridge 状态服务不可用")}
        fields = {}
        for token in result.get("message", "").split():
            if "=" in token:
                key, value = token.split("=", 1)
                fields[key] = value
        control_ready = (
            fields.get("authorized") == "true"
            and fields.get("mode") == "ArmHold"
            and fields.get("publisher_present") == "true"
            and fields.get("state") == "valid"
            and fields.get("navigation_enabled") == "true"
        )
        ready = control_ready and fields.get("navigation_ready") == "true"
        if ready:
            message = "ONNX 已授权，ArmHold 与导航输入就绪"
        elif control_ready:
            message = "ONNX 运动控制已进入 ArmHold；等待导航栈输入"
        elif fields.get("authorized") != "true":
            message = "ONNX 输出尚未授权"
        elif fields.get("mode") != "ArmHold":
            message = f"ONNX 当前模式为 {fields.get('mode', '未知')}，需要 ArmHold"
        elif fields.get("state") != "valid":
            message = "Policy LowState 无效"
        elif fields.get("navigation_ready") != "true":
            message = "ONNX 导航输入尚未就绪"
        else:
            message = "ONNX 硬件输出未就绪"
        return {
            "available": True,
            "control_ready": control_ready,
            "ready": ready,
            "mode": fields.get("mode"),
            "profile": fields.get("profile"),
            "authorized": fields.get("authorized") == "true",
            "output_enabled": None if "output_enabled" not in fields else fields["output_enabled"] == "true",
            "debug_confirmed": None if "operator_confirmed_debug_mode" not in fields else fields["operator_confirmed_debug_mode"] == "true",
            "exclusive_confirmed": None if "operator_confirmed_exclusive_control" not in fields else fields["operator_confirmed_exclusive_control"] == "true",
            "publisher_present": fields.get("publisher_present") == "true",
            "state_valid": fields.get("state") == "valid",
            "navigation_enabled": fields.get("navigation_enabled") == "true",
            "continuous_navigation": fields.get("continuous_navigation") == "true",
            "remote_velocity_enabled": fields.get("remote_velocity_enabled") == "true",
            "remote_source": fields.get("remote_source", "zero"),
            "remote_active": fields.get("remote_active") == "true",
            "navigation_ready": fields.get("navigation_ready") == "true",
            "output_transition": fields.get("output_transition"),
            "joint_command_mode": fields.get("joint_command_mode"),
            "message": message,
        }

    def policy_control_command(self, step):
        if step not in ("authorize_output", "fix_stand", "arm_hold_stand", "arm_hold"):
            return {"success": False, "message": "未知的 Policy 准备步骤。"}
        return self._call_policy_service(step)

    def _call_policy_service(self, operation, payload=None):
        env = os.environ.copy()
        env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        env["ROS_DOMAIN_ID"] = env.get("ROS_DOMAIN_ID", "0")
        env["ROS_LOCALHOST_ONLY"] = "0"
        env.pop("CYCLONEDDS_URI", None)
        env.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
        command = [sys.executable, "-m", "h1_2_nav_tools.policy_web_client"]
        request = {"operation": operation}
        request.update(payload or {})
        with self.policy_service_lock:
            try:
                completed = subprocess.run(
                    command, env=env, input=json.dumps(request), capture_output=True,
                    text=True, timeout=12.0, check=False
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                return {"success": False, "message": f"Fast DDS Policy 服务调用失败：{exc}"}
        try:
            response = json.loads(completed.stdout.strip().splitlines()[-1])
        except (IndexError, json.JSONDecodeError):
            detail = completed.stderr.strip() or completed.stdout.strip() or f"exit={completed.returncode}"
            return {"success": False, "message": f"Fast DDS Policy 服务无有效响应：{detail}"}
        return response

    def teaching_route_start(self):
        localization = self.localization_status()
        if not localization["ready"]:
            return {"success": False, "message": localization["message"]}
        return self._call_simple(
            "start_teaching_route",
            self.teaching_start_client,
            StartRoute.Request(),
        )

    def teaching_route_stop(self):
        return self._call_simple(
            "stop_teaching_route",
            self.teaching_stop_client,
            StopRoute.Request(),
        )

    def teaching_route_status(self):
        return self._route_status("teaching_route_status", self.teaching_status_client)

    def teaching_route_service_available(self, timeout_sec=0.2):
        return self.teaching_start_client.wait_for_service(timeout_sec=timeout_sec)

    def teaching_record_service_available(self, timeout_sec=0.2):
        return self.teaching_record_start_client.wait_for_service(timeout_sec=timeout_sec)

    def teaching_record_start(self):
        localization = self.localization_status()
        if not localization["ready"]:
            return {"success": False, "message": localization["message"]}
        return self._call_simple(
            "start_teaching_path_recording",
            self.teaching_record_start_client,
            Trigger.Request(),
        )

    def teaching_record_stop_save(self):
        return self._call_simple(
            "stop_and_save_teaching_path",
            self.teaching_record_stop_save_client,
            Trigger.Request(),
        )

    def _route_status(self, label, client):
        if not client.wait_for_service(timeout_sec=0.05):
            return {
                "success": False,
                "state": "unavailable",
                "current_goal": "",
                "current_index": 0,
                "message": f"{label} is not available",
            }
        ok, result = self._wait_future(client.call_async(RouteStatus.Request()))
        if not ok:
            return {
                "success": False,
                "state": "error",
                "current_goal": "",
                "current_index": 0,
                "message": str(result),
            }
        return {
            "success": bool(result.success),
            "state": result.state,
            "current_goal": result.current_goal,
            "current_index": int(result.current_index),
            "message": result.message,
        }

    def execute_gesture(self, name):
        if name not in ("wave_hand", "shake_hand", "reach_hand", "retract_hand"):
            return {"success": False, "message": f"unsupported gesture: {name}", "sdk_ret": -1}
        if not self.gesture_client.wait_for_service(timeout_sec=0.2):
            return {"success": False, "message": "/h1_2_loco/execute_gesture is not available", "sdk_ret": -1}
        request = ExecuteGesture.Request()
        request.name = name
        ok, result = self._wait_future(self.gesture_client.call_async(request))
        if not ok:
            return {"success": False, "message": str(result), "sdk_ret": -1}
        return {"success": bool(result.success), "message": result.message, "sdk_ret": int(result.sdk_ret)}

    def configure_camera(self, config):
        for subscription in self.camera_subscriptions:
            self.node.destroy_subscription(subscription)
        self.camera_subscriptions = []
        self.camera_preview_enabled = bool(config.get("camera_preview_enabled", False))
        self.camera_topics = {
            "rgb": str(config.get("camera_rgb_topic", "")).strip(),
            "depth": str(config.get("camera_depth_topic", "")).strip(),
            "info": str(config.get("camera_info_topic", "")).strip(),
            "annotated": str(config.get("vision_annotated_topic", "")).strip(),
        }
        if not self.camera_preview_enabled:
            self.logs.add("camera", "camera preview disabled; no image subscriptions created")
            return
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        if self.camera_topics["rgb"]:
            self.camera_subscriptions.append(
                self.node.create_subscription(
                    Image,
                    self.camera_topics["rgb"],
                    self._on_rgb_image,
                    qos,
                )
            )
        if self.camera_topics["depth"]:
            self.camera_subscriptions.append(
                self.node.create_subscription(
                    Image,
                    self.camera_topics["depth"],
                    self._on_depth_image,
                    qos,
                )
            )
        if self.camera_topics["info"]:
            self.camera_subscriptions.append(
                self.node.create_subscription(
                    CameraInfo,
                    self.camera_topics["info"],
                    self._on_camera_info,
                    qos,
                )
            )
        if self.camera_topics["annotated"]:
            self.camera_subscriptions.append(
                self.node.create_subscription(
                    Image,
                    self.camera_topics["annotated"],
                    self._on_annotated_image,
                    qos,
                )
            )
        self.logs.add(
            "camera",
            "subscribed rgb={rgb} depth={depth} info={info}".format(**self.camera_topics),
        )

    def camera_status(self):
        if not self.camera_preview_enabled:
            return {
                "state": "disabled",
                "message": "相机预览已关闭",
                "rgb_topic": self.camera_topics.get("rgb", ""),
                "depth_topic": self.camera_topics.get("depth", ""),
                "info_topic": self.camera_topics.get("info", ""),
                "rgb": self.camera_frames.stream_status("rgb"),
                "depth": self.camera_frames.stream_status("depth"),
                "camera_info": {"received": False, "width": 0, "height": 0, "last_frame": None},
            }
        return self.camera_frames.status(
            self.camera_topics.get("rgb", ""),
            self.camera_topics.get("depth", ""),
            self.camera_topics.get("info", ""),
        )

    def vision_status(self):
        stream = self.camera_frames.stream_status("annotated")
        if stream["online"]:
            state = "online"
            message = "YOLO 检测画面在线"
        elif stream["received"]:
            state = "stale"
            message = "YOLO 检测画面已超时"
        else:
            state = "waiting"
            message = "等待 YOLO 检测话题"
        return {
            "state": state,
            "message": message,
            "annotated_topic": self.camera_topics.get("annotated", ""),
            "annotated": stream,
        }

    def _on_localization_status(self, message):
        is_tracking_result = message.message not in (
            "manual initial pose accepted",
            "manual relocalize pose accepted",
        )
        stable = (
            message.initialized
            and message.converged
            and not message.degraded
            and message.score <= LOCALIZATION_MAX_SCORE
            and is_tracking_result
        )
        with self.localization_lock:
            self.latest_localization_status = message
            self.latest_localization_time = time.monotonic()
            self.localization_stable_samples = (
                self.localization_stable_samples + 1 if stable else 0
            )

    def localization_status(self, online_camera_init=False):
        if online_camera_init:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "camera_init", "body", rclpy.time.Time(), timeout=Duration(seconds=0.1)
                )
                stamp = rclpy.time.Time.from_msg(transform.header.stamp)
                age = max(0.0, (self.node.get_clock().now() - stamp).nanoseconds / 1e9)
                ready = stamp.nanoseconds > 0 and age <= 0.5
                return {
                    "ready": ready,
                    "frame_id": "camera_init",
                    "age_sec": round(age, 3),
                    "stable_samples": 0,
                    "required_stable_samples": 0,
                    "score": None,
                    "message": (
                        f"在线位姿可用（camera_init→body，{age:.2f}s）"
                        if ready else f"在线位姿已过期：camera_init→body，{age:.2f}s"
                    ),
                }
            except Exception as exc:
                return {
                    "ready": False,
                    "frame_id": "camera_init",
                    "age_sec": None,
                    "stable_samples": 0,
                    "required_stable_samples": 0,
                    "score": None,
                    "message": f"等待在线位姿 camera_init→body：{exc}",
                }
        with self.localization_lock:
            message = self.latest_localization_status
            stable_samples = self.localization_stable_samples
            updated_at = self.latest_localization_time
        if message is None or updated_at is None:
            return {
                "ready": False,
                "stable_samples": 0,
                "required_stable_samples": LOCALIZATION_REQUIRED_STABLE_SAMPLES,
                "score": None,
                "message": "定位未稳定：等待 /icp_localization/status",
            }
        age = max(0.0, time.monotonic() - updated_at)
        status_ready = (
            age <= LOCALIZATION_STATUS_TIMEOUT_SEC
            and stable_samples >= LOCALIZATION_REQUIRED_STABLE_SAMPLES
        )
        tf_ready, tf_message = self.localization_tf_available()
        ready = status_ready and tf_ready
        if status_ready and tf_ready:
            status_message = (
                f"定位稳定，可开始巡航（score={message.score:.3f}，"
                f"{stable_samples}/{LOCALIZATION_REQUIRED_STABLE_SAMPLES}）"
            )
        elif status_ready:
            status_message = f"定位状态正常，但 TF 未发布：{tf_message}"
        elif age > LOCALIZATION_STATUS_TIMEOUT_SEC:
            status_message = f"定位未稳定：状态已超时 {age:.1f}s"
        else:
            status_message = (
                f"定位未稳定：score={message.score:.3f}，"
                f"{stable_samples}/{LOCALIZATION_REQUIRED_STABLE_SAMPLES}，"
                f"{message.message}"
            )
        return {
            "ready": ready,
            "stable_samples": stable_samples,
            "required_stable_samples": LOCALIZATION_REQUIRED_STABLE_SAMPLES,
            "score": float(message.score),
            "initialized": bool(message.initialized),
            "converged": bool(message.converged),
            "degraded": bool(message.degraded),
            "age_sec": round(age, 3),
            "message": status_message,
        }

    def localization_tf_available(self):
        try:
            self.tf_buffer.lookup_transform(
                LOCALIZATION_TF_TARGET_FRAME,
                LOCALIZATION_TF_SOURCE_FRAME,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            return True, f"{LOCALIZATION_TF_TARGET_FRAME}->{LOCALIZATION_TF_SOURCE_FRAME} available"
        except Exception as exc:
            return False, str(exc)

    def _on_rgb_image(self, message):
        try:
            image = self.cv_bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            jpeg = encode_rgb_jpeg(image, message.encoding)
            self.camera_frames.update_rgb(jpeg, message.width, message.height, message.encoding)
        except Exception as exc:
            self.camera_frames.set_error(f"RGB 转换失败: {exc}")

    def _on_depth_image(self, message):
        try:
            image = self.cv_bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            jpeg = encode_depth_jpeg(image, message.encoding)
            self.camera_frames.update_depth(jpeg, message.width, message.height, message.encoding)
        except Exception as exc:
            self.camera_frames.set_error(f"深度图转换失败: {exc}")

    def _on_annotated_image(self, message):
        try:
            image = self.cv_bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            jpeg = encode_rgb_jpeg(image, message.encoding)
            self.camera_frames.update_annotated(
                jpeg,
                message.width,
                message.height,
                message.encoding,
            )
        except Exception as exc:
            self.logs.add("vision", f"检测图转换失败: {exc}")

    def _on_camera_info(self, message):
        self.camera_frames.update_camera_info(message.width, message.height)

    def _call_simple(self, label, client, request, include_pose=False):
        wait_timeout = 3.0 if label == "save_memory_point" else 0.2
        if not client.wait_for_service(timeout_sec=wait_timeout):
            if label == "save_memory_point":
                return {"success": False, "message": "存点服务尚未就绪。请先启动当前模式，等待在线导航启动完成后再记录点位。"}
            return {"success": False, "message": f"{label} is not available"}
        ok, result = self._wait_future(client.call_async(request))
        if not ok:
            return {"success": False, "message": str(result)}
        response = {"success": bool(result.success), "message": result.message}
        if include_pose and response["success"]:
            pose = result.pose
            response["pose"] = {
                "frame_id": pose.header.frame_id,
                "position": {"x": pose.pose.position.x, "y": pose.pose.position.y, "z": pose.pose.position.z},
                "orientation": {"x": pose.pose.orientation.x, "y": pose.pose.orientation.y,
                                 "z": pose.pose.orientation.z, "w": pose.pose.orientation.w},
            }
        return response

    def _wait_future(self, future, timeout=5.0):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            return False, "service call timed out"
        try:
            return True, future.result()
        except Exception as exc:
            return False, exc


class UiOnlyRosBridge:
    def __init__(self, logs, config):
        self.logs = logs
        self.camera_frames = CameraFrameStore()
        self.camera_topics = {}
        self.configure_camera(config)

    @staticmethod
    def _blocked():
        return {"success": False, "message": "UI-only 模式：未连接 ROS，操作未执行"}

    def shutdown(self):
        return

    def configure_camera(self, config):
        self.camera_topics = {
            "rgb": str(config.get("camera_rgb_topic", "")).strip(),
            "depth": str(config.get("camera_depth_topic", "")).strip(),
            "info": str(config.get("camera_info_topic", "")).strip(),
            "annotated": str(config.get("vision_annotated_topic", "")).strip(),
        }

    def route_start(self):
        return self._blocked()

    def save_memory_point(self, _name, _overwrite=False):
        return self._blocked()

    def route_stop(self):
        return self._blocked()

    def teaching_route_start(self):
        return self._blocked()

    def teaching_route_stop(self):
        return self._blocked()

    def teaching_record_start(self):
        return self._blocked()

    def teaching_record_stop_save(self):
        return self._blocked()

    def execute_gesture(self, _name):
        return self._blocked()

    def teaching_route_service_available(self, timeout_sec=0.2):
        return False

    def teaching_record_service_available(self, timeout_sec=0.2):
        return False

    @staticmethod
    def _route_status(label):
        return {
            "success": False,
            "state": "ui-only",
            "current_goal": "",
            "current_index": 0,
            "message": f"{label}：UI-only 模拟状态",
        }

    def route_status(self):
        return self._route_status("记忆点巡航")

    def teaching_route_status(self):
        return self._route_status("示教巡航")

    @staticmethod
    def localization_status():
        return {
            "ready": False,
            "stable_samples": 0,
            "required_stable_samples": LOCALIZATION_REQUIRED_STABLE_SAMPLES,
            "score": None,
            "message": "UI-only 模式：未连接 ROS",
        }

    def camera_status(self):
        return {
            "state": "ui-only",
            "message": "UI-only 模式：无相机数据",
            "rgb_topic": self.camera_topics.get("rgb", ""),
            "depth_topic": self.camera_topics.get("depth", ""),
            "info_topic": self.camera_topics.get("info", ""),
            "rgb": self.camera_frames.stream_status("rgb"),
            "depth": self.camera_frames.stream_status("depth"),
            "camera_info": {"received": False, "width": 0, "height": 0, "last_frame": None},
        }

    def vision_status(self):
        return {
            "state": "ui-only",
            "message": "UI-only 模式：无识别数据",
            "annotated_topic": self.camera_topics.get("annotated", ""),
            "annotated": self.camera_frames.stream_status("annotated"),
        }


class ProcessManager:
    def __init__(self, workspace_root, logs):
        self.workspace_root = workspace_root
        self.logs = logs
        self.nav_process = None
        self.policy_start_process = None
        self.motion_backend = "unitree"
        self.run_mode = "online"
        self.policy_debug_confirmed = False
        self.policy_exclusive_confirmed = False
        self.rviz_process = None
        self.voice_process = None
        self.voice_bridge_process = None
        self.vision_process = None
        self.teaching_route_process = None
        self.teaching_record_process = None
        self.lock = threading.Lock()
        self._closed = False

    def status(self):
        navigation = self._process_state(self.nav_process)
        external = self._external_navigation()
        if not navigation["running"] and external:
            navigation.update({"running": True, "external": True,
                               "motion_backend": external["motion_backend"],
                               "run_mode": external["run_mode"]})
        elif navigation["running"]:
            navigation.update({"external": False, "motion_backend": self.motion_backend,
                               "run_mode": self.run_mode})
        return {
            "navigation": navigation,
            "policy_bridge": self._policy_bridge_state(),
            "rviz": self._process_state(self.rviz_process),
            "voice": self._process_state(self.voice_process),
            "voice_bridge": self._process_state(self.voice_bridge_process),
            "vision": self._process_state(self.vision_process),
            "teaching_route": self._process_state(self.teaching_route_process),
            "teaching_record": self._process_state(self.teaching_record_process),
        }

    @staticmethod
    def _policy_bridge_state():
        pid_file = Path("/tmp/h1_2_policy_bridge.pid")
        try:
            pid = int(pid_file.read_text(encoding="ascii").strip())
            if pid <= 1 or os.getsid(pid) != pid or os.getpgid(pid) != pid:
                return {"running": False, "external": False}
            command = Path(f"/proc/{pid}/cmdline").read_bytes().replace(bytes((0,)), b" ").decode(errors="replace")
            if "policy_bridge.launch.py" not in command:
                return {"running": False, "external": False}
            return {"running": True, "external": True, "pid": pid}
        except (OSError, ValueError, ProcessLookupError):
            return {"running": False, "external": False}

    def start_policy_bridge(self, payload):
        with self.lock:
            if self._is_running(self.policy_start_process):
                return {"success": False, "message": "ONNX 控制桥正在启动，请等待状态更新。"}
            if self._policy_bridge_state()["running"]:
                return {"success": True, "message": "ONNX 控制桥已运行；可继续检查状态并完成准备。"}
            navigation = self.status()["navigation"]
            if navigation["running"]:
                return {"success": False, "message": "当前导航栈已运行；请先确认后端，不能在其上另起 Policy bridge。"}
            script = self.workspace_root / "run_ai_slam_nav.sh"
            if not script.is_file():
                return {"success": False, "message": f"missing script: {script}"}
            args = [str(script), "mode=policy_bridge_only", "motion_backend=policy"]
            args.extend((
                f"confirm_debug_mode={str(payload.get('confirm_debug_mode') is True).lower()}",
                f"confirm_exclusive_control={str(payload.get('confirm_exclusive_control') is True).lower()}",
                f"enable_remote_velocity={str(payload.get('enable_remote_velocity') is True).lower()}",
            ))
            self.motion_backend = "policy"
            self.run_mode = str(payload.get("run_mode", "online"))
            self.policy_debug_confirmed = payload.get("confirm_debug_mode") is True
            self.policy_exclusive_confirmed = payload.get("confirm_exclusive_control") is True
            self.policy_start_process = self._start("policy-bridge-start", args)
            return {"success": True, "message": "ONNX 控制桥启动请求已接受；导航栈与路线尚未启动。"}

    def _external_navigation(self):
        try:
            pid_file = Path("/tmp/h1_2_ai_slam_nav.pid")
            backend_file = Path("/tmp/h1_2_ai_slam_nav.backend")
            pid = int(pid_file.read_text().strip())
            backend_pid, backend = backend_file.read_text().split()[:2]
            if int(backend_pid) != pid or backend not in ("unitree", "policy"):
                return None
            if os.getsid(pid) != pid or os.getpgid(pid) != pid:
                return None
            command = Path(f"/proc/{pid}/cmdline").read_bytes().replace(bytes((0,)), b" ").decode(errors="replace")
            if "ros2 launch h1_2_slam_bringup" not in command:
                return None
            if "h1_2_nav2_camera_init_navigation.launch.py" in command:
                run_mode = "online"
            elif "h1_2_nav2_navigation.launch.py" in command:
                run_mode = "navigation"
            else:
                return None
            return {"motion_backend": backend, "run_mode": run_mode}
        except (OSError, ValueError, ProcessLookupError):
            return None

    def start_navigation(self, args):
        with self.lock:
            if self._external_navigation():
                return {"success": False, "message": "导航已由其他终端启动，请先接管或停止当前实例。"}
            script = self.workspace_root / "run_ai_slam_nav.sh"
            if not script.is_file():
                return {"success": False, "message": f"missing script: {script}"}
            if args and args[0] in ("mode=save_map", "mode=save_pcd_map"):
                label = "map-save" if args[0] == "mode=save_map" else "pcd-save"
                self._start(label, [str(script), *args])
                return {"success": True, "message": f"{label} process started"}
            if self._is_running(self.nav_process):
                return {"success": False, "message": "navigation is already running"}
            self.motion_backend = next(
                (arg.split("=", 1)[1] for arg in args if arg.startswith("motion_backend=")),
                "unitree",
            )
            self.policy_debug_confirmed = "confirm_debug_mode=true" in args
            self.policy_exclusive_confirmed = "confirm_exclusive_control=true" in args
            self.run_mode = next(
                (arg.split("=", 1)[1] for arg in args if arg.startswith("mode=")),
                "online",
            )
            if self.motion_backend == "policy" and not self._policy_bridge_state()["running"]:
                return {"success": False, "message": "ONNX 控制桥未运行；请先完成 ONNX ArmHold 准备。"}
            self.nav_process = self._start("nav", [str(script), *args])
            return {"success": True, "message": "navigation process started"}

    def stop_all(self, payload=None):
        try:
            with self.lock:
                result = self._stop_navigation_stack()
        except OSError as exc:
            self.logs.add("web", f"failed to run navigation stop script: {exc}")
            return {"success": False, "message": f"无法执行导航停止脚本：{exc}"}
        if not result["success"]:
            return result
        self.stop_voice_control(payload or {})
        self.stop_teaching_record_runner()
        self.stop_teaching_route_runner()
        return result

    def _stop_navigation_stack(self):
        script = self.workspace_root / "stop_ai_slam_nav.sh"
        if not script.is_file():
            return {"success": False, "message": f"missing script: {script}"}

        navigation = self.status()["navigation"]
        backend = navigation.get("motion_backend", self.motion_backend)
        stop_env = {"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"} if backend == "policy" else None
        stop_process = self._start("stop", [str(script)], env=stop_env)
        try:
            stop_process.wait(timeout=25.0)
        except subprocess.TimeoutExpired:
            return {"success": False, "message": "导航停止确认超时；请查看日志，确认 SLAM/Nav2 状态。"}
        if stop_process.returncode != 0:
            return {"success": False, "message": "停止脚本未确认导航进程全部退出；请查看停止日志。"}

        if self._is_running(self.nav_process):
            try:
                self.nav_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._terminate_process(self.nav_process)
                try:
                    self.nav_process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    return {"success": False, "message": "导航启动器进程仍未退出。"}
        if self._external_navigation() is not None:
            return {"success": False, "message": "导航 PID 标记仍有效；停止未完成。"}

        message = (
            "导航已停止；Policy bridge 保持 ArmHold，直到现场支撑并单独撤权。"
            if backend == "policy"
            else "SLAM/Nav2 已停止。"
        )
        return {"success": True, "message": message}

    def stop_navigation(self, reason="requested"):
        with self.lock:
            self.logs.add("web", f"stopping navigation: {reason}")
            try:
                return self._stop_navigation_stack()
            except OSError as exc:
                self.logs.add("web", f"failed to run navigation stop script: {exc}")
                return {"success": False, "message": f"无法执行导航停止脚本：{exc}"}

    def start_voice_control(self, payload=None):
        with self.lock:
            if self._is_running(self.voice_process) or self._is_running(self.voice_bridge_process):
                return {"success": False, "message": "voice control is already running"}
            payload = payload or {}
            self.voice_process = self._start("voice-reader", self._voice_start_command(payload))
            self.voice_bridge_process = self._start(
                "voice-bridge",
                ["ros2", "launch", "h1_2_nav_tools", "h1_2_voice_action_bridge.launch.py"],
            )
            return {"success": True, "message": "voice control process started"}

    def stop_voice_control(self, payload=None):
        with self.lock:
            stopped = False
            if self._is_running(self.voice_bridge_process):
                self.logs.add("voice-bridge", "terminating voice action bridge")
                self.voice_bridge_process.terminate()
                stopped = True
            if self._is_running(self.voice_process):
                self.logs.add("voice-reader", "terminating voice reader")
                self.voice_process.terminate()
                stopped = True
            if stopped:
                return {"success": True, "message": "voice control terminate requested"}
            command = self._voice_stop_command(payload or {})
            if command:
                self._start("voice-reader-stop", command)
                return {"success": True, "message": "remote voice control stop requested"}
            return {"success": False, "message": "voice control is not running"}

    def _voice_start_command(self, payload):
        remote_host = normalize_ssh_target(payload.get("voice_remote_host", ""))
        if not remote_host:
            return ["ros2", "launch", "h1_2_voice_control", "h1_2_voice_control.launch.py"]
        workspace = normalize_voice_workspace(payload.get("voice_workspace", "~/ros2_ws"))
        ros_setup = str(payload.get("voice_ros_setup", "/opt/ros/humble/setup.bash")).strip()
        return ["ssh", remote_host, self._remote_voice_shell_command(workspace, ros_setup)]

    def _voice_stop_command(self, payload):
        remote_host = normalize_ssh_target(payload.get("voice_remote_host", ""))
        if not remote_host:
            return None
        return ["ssh", remote_host, "pkill -f 'h1_2_voice_control.launch.py|h1_2_voice_control' || true"]

    def start_vision(self, payload=None):
        with self.lock:
            if self._is_running(self.vision_process):
                return {"success": False, "message": "YOLO vision is already running"}
            payload = payload or {}
            self.vision_process = self._start("vision", self._vision_start_command(payload))
            return {"success": True, "message": "YOLO vision start requested"}

    def stop_vision(self, payload=None):
        with self.lock:
            stopped = False
            if self._is_running(self.vision_process):
                self.logs.add("vision", "terminating YOLO launch process")
                self.vision_process.terminate()
                stopped = True
            command = self._vision_stop_command(payload or {})
            if command:
                self._start("vision-stop", command)
                return {"success": True, "message": "remote YOLO vision stop requested"}
            if stopped:
                return {"success": True, "message": "YOLO vision terminate requested"}
            return {"success": False, "message": "YOLO vision is not running"}

    def start_teaching_route_runner(self, payload=None):
        with self.lock:
            if self._is_running(self.teaching_route_process):
                return {"success": False, "message": "teaching route runner is already running"}
            payload = payload or {}
            path_file = Path(str(payload.get("teaching_path_file", ""))).expanduser()
            events_file = Path(str(payload.get("teaching_events_file", ""))).expanduser()
            if not path_file.is_file():
                return {"success": False, "message": f"teaching path file does not exist: {path_file}"}
            if not events_file.is_file():
                return {"success": False, "message": f"teaching events file does not exist: {events_file}"}
            self.teaching_route_process = self._start(
                "teaching-route",
                [
                    "ros2",
                    "launch",
                    "h1_2_nav_tools",
                    "teaching_route_runner.launch.py",
                    f"path_file:={path_file}",
                    f"events_file:={events_file}",
                    f"memory_points_file:={self.workspace_root / 'maps' / 'memory_points.json'}",
                    "control_mode:=direct",
                    f"loop:={str(bool(payload.get('teaching_loop', True))).lower()}",
                ],
            )
            return {"success": True, "message": "teaching route runner process started"}

    def start_teaching_record_runner(self, payload=None):
        with self.lock:
            if self._is_running(self.teaching_record_process):
                return {"success": False, "message": "teaching path recorder is already running"}
            path_value = str((payload or {}).get("teaching_path_file", "")).strip()
            if not path_value:
                return {"success": False, "message": "teaching path file is empty"}
            path_file = Path(path_value).expanduser()
            self.teaching_record_process = self._start(
                "teaching-record",
                [
                    "ros2",
                    "launch",
                    "h1_2_nav_tools",
                    "record_teaching_path.launch.py",
                    f"file_path:={path_file}",
                    "path_topic:=/teaching_route_path",
                    "interactive_console:=false",
                ],
            )
            return {"success": True, "message": "teaching path recorder process started"}

    def stop_teaching_record_runner(self):
        with self.lock:
            if not self._is_running(self.teaching_record_process):
                return {"success": False, "message": "teaching path recorder is not running"}
            self.logs.add("teaching-record", "terminating teaching path recorder")
            self.teaching_record_process.terminate()
            return {"success": True, "message": "teaching path recorder terminate requested"}

    def stop_teaching_route_runner(self):
        with self.lock:
            if not self._is_running(self.teaching_route_process):
                return {"success": False, "message": "teaching route runner is not running"}
            self.logs.add("teaching-route", "terminating teaching route runner")
            self.teaching_route_process.terminate()
            return {"success": True, "message": "teaching route runner terminate requested"}

    def _vision_start_command(self, payload):
        remote_host = normalize_ssh_target(payload.get("vision_remote_host", ""))
        if not remote_host:
            return ["ros2", "launch", "h1_2_vision", "yolo_detector.launch.py"]
        workspace = normalize_voice_workspace(payload.get("vision_workspace", "~/ros2_ws"))
        ros_setup = str(payload.get("vision_ros_setup", "/opt/ros/humble/setup.bash")).strip()
        commands = [f"cd {quote_remote_path(workspace)}"]
        if ros_setup:
            commands.append(f"source {quote_remote_path(ros_setup)}")
        commands.extend(
            [
                "source install/setup.bash",
                "ros2 launch h1_2_vision yolo_detector.launch.py",
            ]
        )
        return ["ssh", remote_host, "bash -lc " + shlex.quote(" && ".join(commands))]

    def _vision_stop_command(self, payload):
        remote_host = normalize_ssh_target(payload.get("vision_remote_host", ""))
        if not remote_host:
            return None
        return [
            "ssh",
            remote_host,
            "pkill -f 'yolo_detector.launch.py|h1_2_vision/yolo_detector|h1_2_vision.*yolo_detector' || true",
        ]

    @staticmethod
    def _remote_voice_shell_command(workspace, ros_setup):
        commands = [f"cd {quote_remote_path(workspace)}"]
        if ros_setup:
            commands.append(f"source {quote_remote_path(ros_setup)}")
        commands.extend(
            [
                "source install/setup.bash",
                "ros2 launch h1_2_voice_control h1_2_voice_control.launch.py",
            ]
        )
        return "bash -lc " + shlex.quote(" && ".join(commands))

    def open_rviz(self, mode="online", rviz_mode="auto"):
        with self.lock:
            if self._is_running(self.rviz_process):
                return {"success": False, "message": "RViz is already running"}
            rviz_mode = self._resolve_rviz_mode(mode, rviz_mode)
            self.rviz_process = self._start(
                "rviz",
                ["ros2", "launch", "h1_2_slam_bringup", "h1_2_nav2_rviz.launch.py", f"mode:={rviz_mode}"],
            )
            return {"success": True, "message": "RViz process started"}

    def close_rviz(self):
        with self.lock:
            if not self._is_running(self.rviz_process):
                return {"success": False, "message": "RViz is not running"}
            self.logs.add("rviz", "terminating RViz")
            self.rviz_process.terminate()
            return {"success": True, "message": "RViz terminate requested"}

    def restart_rviz(self, mode="online", rviz_mode="auto"):
        self.close_rviz()
        time.sleep(0.5)
        return self.open_rviz(mode, rviz_mode)

    @staticmethod
    def _resolve_rviz_mode(mode, rviz_mode):
        if rviz_mode in ("online", "offline"):
            return rviz_mode
        return "offline" if mode == "navigation" else "online"

    def shutdown(self):
        with self.lock:
            if self._closed:
                return
            self._closed = True
            if self.status()["navigation"]["running"]:
                try:
                    result = self._stop_navigation_stack()
                except OSError as exc:
                    result = {"success": False, "message": f"无法执行导航停止脚本：{exc}"}
                if not result["success"]:
                    self.logs.add("web", result["message"])
            for label, process in [
                ("rviz", self.rviz_process),
                ("voice-reader", self.voice_process),
                ("voice-bridge", self.voice_bridge_process),
                ("vision", self.vision_process),
                ("teaching-route", self.teaching_route_process),
                ("teaching-record", self.teaching_record_process),
            ]:
                if self._is_running(process):
                    self.logs.add(label, "terminating on server shutdown")
                    self._terminate_process(process)
    @staticmethod
    def _terminate_process(process, interrupt_timeout=5.0, terminate_timeout=2.0):
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            try:
                process.terminate()
            except ProcessLookupError:
                return
        try:
            process.wait(timeout=interrupt_timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                process.terminate()
            try:
                process.wait(timeout=terminate_timeout)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    process.kill()

    def _start(self, label, command, env=None):
        self.logs.add(label, "$ " + " ".join(command))
        process = subprocess.Popen(
            command,
            cwd=str(self.workspace_root),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            start_new_session=True,
            env={**os.environ, **env} if env else None,
        )
        threading.Thread(target=self._read_stream, args=(label, "stdout", process.stdout), daemon=True).start()
        threading.Thread(target=self._read_stream, args=(label, "stderr", process.stderr), daemon=True).start()
        threading.Thread(target=self._wait_process, args=(label, process), daemon=True).start()
        return process

    def _read_stream(self, label, stream_name, stream):
        if stream is None:
            return
        for line in stream:
            self.logs.add(label, f"{stream_name}: {line.rstrip()}")

    def _wait_process(self, label, process):
        code = process.wait()
        self.logs.add(label, f"exited code={code}")

    @staticmethod
    def _is_running(process):
        return process is not None and process.poll() is None

    @staticmethod
    def _process_state(process):
        if process is None:
            return {"running": False, "returncode": None}
        return {"running": process.poll() is None, "returncode": process.poll()}


class WebGuiApp:
    @staticmethod
    def _task_points_file(workspace_root, profile):
        if profile.get("id") == "default":
            return workspace_root / "maps" / "memory_points.json"
        name = str(profile.get("name") or profile.get("id") or "task").strip()
        safe_name = "".join("_" if char in '/\\:*?\"<>|' else char for char in name).strip(" .")
        return workspace_root / "maps" / "tasks" / f"{safe_name or 'task'}.json"

    def __init__(self, workspace_root, ui_only=False):
        self.workspace_root = workspace_root
        self.ui_only = bool(ui_only)
        self.runtime_profile = os.environ.get("H1_2_RUNTIME_PROFILE", "local").strip().lower()
        self.pc4_profile = self.runtime_profile == "pc4"
        self.heartbeat_timeout_sec = HEARTBEAT_TIMEOUT_SEC
        self.heartbeat_expired = False
        self.heartbeat_lock = threading.Lock()
        self.last_heartbeat = None
        self.watchdog_stop = threading.Event()
        self.loaded_task = None
        self.task_lock = threading.RLock()
        self.points_file = workspace_root / "maps" / "memory_points.json"
        self.logs = LogStore()
        self.processes = ProcessManager(workspace_root, self.logs)
        bridge_type = UiOnlyRosBridge if self.ui_only else RosServiceBridge
        self.ros = bridge_type(self.logs, self.default_config())
        self.watchdog_thread = threading.Thread(target=self._watch_heartbeat, daemon=True)
        self.watchdog_thread.start()
        self.logs.add("web", f"workspace={workspace_root}")
        if self.ui_only:
            self.logs.add("web", "UI-only mode enabled; ROS and process controls are disabled")

    def shutdown(self):
        self.watchdog_stop.set()
        if not self.ui_only:
            self.processes.shutdown()
        self.ros.shutdown()

    def record_heartbeat(self):
        with self.heartbeat_lock:
            self.last_heartbeat = time.monotonic()

    def _heartbeat_age(self):
        with self.heartbeat_lock:
            last_heartbeat = self.last_heartbeat
        if last_heartbeat is None:
            return float("inf")
        return max(0.0, time.monotonic() - last_heartbeat)

    def _watch_heartbeat(self):
        timeout_sec = getattr(self, "heartbeat_timeout_sec", HEARTBEAT_TIMEOUT_SEC)
        interval = min(0.25, timeout_sec / 4.0)
        while not self.watchdog_stop.wait(interval):
            if self.heartbeat_expired:
                continue
            if not self.processes.status()["navigation"]["running"]:
                continue
            with self.heartbeat_lock:
                if self.last_heartbeat is None:
                    continue
            age = self._heartbeat_age()
            if age <= timeout_sec:
                continue
            self.logs.add("web", f"browser heartbeat timed out after {age:.2f}s; stopping navigation")
            self.heartbeat_expired = True
            # The stop script sends zero velocity independently of ROS service availability.
            self.processes.stop_navigation("browser heartbeat timeout")
            self.ros.route_stop()
            self.ros.teaching_route_stop()

    def default_config(self):
        config = {
            "heartbeat_ws_port": getattr(self, "heartbeat_ws_port", 0),
            "map": str(self.workspace_root / "maps" / "h1_2_lab.yaml"),
            "pcd": str(self.workspace_root / "maps" / "h1_2_lab.pcd"),
            "run_mode": "online",
            "motion_backend": "unitree",
            "confirm_debug_mode": False,
            "confirm_exclusive_control": False,
            "enable_remote_velocity": True,
            "rviz_mode": "auto",
            "rviz_embed_url": "http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=remote",
            "rviz_embed_enabled": True,
            "map_prefix": "maps/h1_2_lab",
            "pcd_prefix": "maps/h1_2_lab",
            "localization": "global_gicp",
            "initial_pose": "START",
            "start_gesture": "",
            "start_turn": "",
            "auto_start_route": False,
            "return_to_start": False,
            "memory_route": "A,B,C",
            "gesture_route": "",
            "nav2_params_file": str(
                self.workspace_root / "src" / "h1_2_nav_tools" / "config" / "nav2_h1_2_rpp.yaml"
            ),
            "teaching_path_file": str(
                self.workspace_root / "src" / "h1_2_nav_tools" / "path" / "teaching_paths.txt"
            ),
            "teaching_events_file": str(
                self.workspace_root
                / "src"
                / "h1_2_nav_tools"
                / "config"
                / "teaching_route_events.example.json"
            ),
            "teaching_loop": True,
            "workflow_version": CONFIG_WORKFLOW_VERSION,
            "voice_remote_host": os.environ.get("H1_2_VOICE_REMOTE", ""),
            "voice_workspace": os.environ.get("H1_2_VOICE_WORKSPACE", "~/ros2_ws"),
            "voice_ros_setup": os.environ.get("H1_2_VOICE_ROS_SETUP", "/opt/ros/humble/setup.bash"),
            "camera_rgb_topic": "/camera/camera/color/image_raw",
            "camera_depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "camera_preview_enabled": False,
            "vision_annotated_topic": "/h1_2/vision/annotated_image",
            "vision_remote_host": os.environ.get(
                "H1_2_VISION_REMOTE",
                os.environ.get("H1_2_VOICE_REMOTE", ""),
            ),
            "vision_workspace": os.environ.get(
                "H1_2_VISION_WORKSPACE",
                os.environ.get("H1_2_VOICE_WORKSPACE", "~/ros2_ws"),
            ),
            "vision_ros_setup": os.environ.get(
                "H1_2_VISION_ROS_SETUP",
                os.environ.get("H1_2_VOICE_ROS_SETUP", "/opt/ros/humble/setup.bash"),
            ),
        }
        try:
            saved = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            if saved.get("workflow_version", 1) < CONFIG_WORKFLOW_VERSION:
                saved["auto_start_route"] = False
                saved["memory_route"] = normalize_memory_route(
                    saved.get("memory_route", "A,B,C")
                )
            config.update(saved)
        except (OSError, json.JSONDecodeError):
            pass
        config["heartbeat_ws_port"] = getattr(self, "heartbeat_ws_port", 0)
        config["memory_route"] = normalize_memory_route(
            config.get("memory_route", "A,B,C")
        )
        if not str(config.get("nav2_params_file", "")).strip():
            config["nav2_params_file"] = str(
                self.workspace_root / "src" / "h1_2_nav_tools" / "config" / "nav2_h1_2_rpp.yaml"
            )
        config["camera_preview_enabled"] = False
        config["workflow_version"] = CONFIG_WORKFLOW_VERSION
        config["enable_remote_velocity"] = bool(config.get("enable_remote_velocity", True))
        config["rviz_embed_url"] = str(config.get("rviz_embed_url", "")).strip()
        config["rviz_embed_enabled"] = bool(config.get("rviz_embed_enabled", True))
        config["camera_preview_enabled"] = False
        config["voice_remote_host"] = normalize_ssh_target(config.get("voice_remote_host", ""))
        config["voice_workspace"] = normalize_voice_workspace(config.get("voice_workspace", "~/ros2_ws"))
        config["vision_remote_host"] = normalize_ssh_target(config.get("vision_remote_host", ""))
        config["vision_workspace"] = normalize_voice_workspace(
            config.get("vision_workspace", "~/ros2_ws")
        )
        config["ui_only"] = self.ui_only
        runtime_profile = getattr(self, "runtime_profile", "local")
        config["runtime_profile"] = runtime_profile
        config["pc4_profile"] = getattr(self, "pc4_profile", runtime_profile == "pc4")
        normalize_task_profiles(config)
        return config

    def save_config(self, config):
        normalize_task_profiles(config)
        if getattr(self, "pc4_profile", False):
            config["camera_preview_enabled"] = False
        for profile in config["task_profiles"]:
            new_task = "memory_points_file" in profile and not str(profile["memory_points_file"]).strip()
            previous_file = Path(str(profile.get("memory_points_file", ""))).expanduser() if str(profile.get("memory_points_file", "")).strip() else None
            points_file = self._task_points_file(self.workspace_root, profile)
            if not points_file.is_absolute():
                points_file = self.workspace_root / points_file
            profile["memory_points_file"] = str(points_file)
            if not points_file.exists():
                points_file.parent.mkdir(parents=True, exist_ok=True)
                legacy_points = getattr(self, "points_file", self.workspace_root / "maps" / "memory_points.json")
                if previous_file and previous_file.exists():
                    points_file.write_text(previous_file.read_text(encoding="utf-8"), encoding="utf-8")
                elif not new_task and legacy_points.exists():
                    points_file.write_text(legacy_points.read_text(encoding="utf-8"), encoding="utf-8")
                else:
                    points_file.write_text(json.dumps({"frame_id": "map", "points": {}}, ensure_ascii=False, indent=2), encoding="utf-8")
        config["memory_route"] = normalize_memory_route(
            config.get("memory_route", "A,B,C")
        )
        config["nav2_params_file"] = str(config.get("nav2_params_file", "")).strip()
        config["teaching_path_file"] = str(config.get("teaching_path_file", "")).strip()
        config["teaching_events_file"] = str(config.get("teaching_events_file", "")).strip()
        config["teaching_loop"] = bool(config.get("teaching_loop", True))
        config["rviz_embed_url"] = str(config.get("rviz_embed_url", "")).strip()
        config["rviz_embed_enabled"] = bool(config.get("rviz_embed_enabled", True))
        config["workflow_version"] = CONFIG_WORKFLOW_VERSION
        config["enable_remote_velocity"] = bool(config.get("enable_remote_velocity", True))
        if "voice_remote_host" in config:
            config["voice_remote_host"] = normalize_ssh_target(config["voice_remote_host"])
        if "voice_workspace" in config:
            config["voice_workspace"] = normalize_voice_workspace(config["voice_workspace"])
        if "vision_remote_host" in config:
            config["vision_remote_host"] = normalize_ssh_target(config["vision_remote_host"])
        if "vision_workspace" in config:
            config["vision_workspace"] = normalize_voice_workspace(config["vision_workspace"])
        if "teaching_path_file" in config:
            config["teaching_path_file"] = str(config["teaching_path_file"]).strip()
        if "teaching_events_file" in config:
            config["teaching_events_file"] = str(config["teaching_events_file"]).strip()
        if "teaching_loop" in config:
            config["teaching_loop"] = bool(config["teaching_loop"])
        if "nav2_params_file" in config:
            config["nav2_params_file"] = str(config["nav2_params_file"]).strip()
        CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with self.task_lock:
            temporary = CONFIG_PATH.with_suffix(".json.tmp")
            temporary.write_text(json.dumps(config, indent=2, sort_keys=True), encoding="utf-8")
            temporary.replace(CONFIG_PATH)

    def start_navigation_task(self, payload):
        with self.task_lock:
            timeout_sec = getattr(self, "heartbeat_timeout_sec", 2.0)
            if self._heartbeat_age() > timeout_sec:
                return {"success": False, "message": "浏览器心跳未连接或已超时，导航未启动。"}
            ok, message = self.validate_navigation(payload)
            if not ok:
                return {"success": False, "message": message}
            if payload.get("motion_backend", "unitree") == "policy":
                if not self.processes.status()["policy_bridge"].get("running"):
                    return {"success": False, "message": "请先启动 ONNX 控制并完成 ArmHold 准备。"}
                policy = self.ros.policy_control_status()
                if not policy.get("available") or not policy.get("control_ready"):
                    return {"success": False, "message": policy.get("message", "ONNX 尚未进入 ArmHold。")}
                if self.processes.status()["navigation"].get("running"):
                    route = self.ros.policy_route_status()
                    if not route.get("success") or route.get("state") != "idle":
                        return {"success": False, "message": "Policy 路线状态未知或非 idle；未启动导航栈。"}
                payload = dict(payload)
                payload["auto_start_route"] = False
            result = self.processes.start_navigation(self.navigation_args(payload))
            if result.get("success") and payload.get("run_mode") in ("online", "navigation"):
                self.heartbeat_expired = False
                self.loaded_task = task_execution_snapshot(payload)
                if payload.get("run_mode") not in ("save_map", "save_pcd_map"):
                    self.save_config(payload)
            return result

    def start_policy_control(self, payload):
        with self.task_lock:
            if self._heartbeat_age() > self.heartbeat_timeout_sec:
                return {"success": False, "message": "浏览器心跳未连接或已超时；ONNX 控制桥未启动。"}
            if not all(payload.get(key) is True for key in (
                "confirm_debug_mode", "confirm_exclusive_control",
                "confirm_lowcmd_handoff", "confirm_robot_supported",
            )):
                return {"success": False, "message": "请先完成页面上的现场准入确认。"}
            if payload.get("motion_backend") != "policy":
                return {"success": False, "message": "ONNX 控制桥启动请求后端不匹配。"}
            result = self.processes.start_policy_bridge(payload)
            if result.get("success"):
                self.heartbeat_expired = False
            return result

    def start_selected_task(self, payload):
        with self.task_lock:
            selected = task_execution_snapshot(payload)
            process_status = self.processes.status()
            navigation = process_status["navigation"]
            if self.loaded_task is None and navigation.get("external"):
                if (payload.get("run_mode") != navigation.get("run_mode") or
                        payload.get("motion_backend", "unitree") != navigation.get("motion_backend")):
                    return {"success": False, "message": "网页所选模式与当前终端启动的导航后端不一致。"}
                if navigation.get("motion_backend") == "policy":
                    if not (payload.get("confirm_debug_mode") is True and payload.get("confirm_exclusive_control") is True):
                        return {"success": False, "message": "请重新确认 Debug 模式和独占控制。"}
                    if selected["gesture_route"] or selected["turn_route"]:
                        return {"success": False, "message": "Policy 导航当前不支持路线手势或到点转身，请先清空路线动作。"}
                self.loaded_task = selected
            if self.loaded_task is None or not navigation["running"]:
                return {"success": False, "message": "请先启动导航，载入当前任务。"}
            keys = ("id", "memory_route", "gesture_route", "turn_route")
            if any(selected[key] != self.loaded_task[key] for key in keys):
                return {"success": False, "message": "当前任务或编排与已加载的任务不同。请停止全部后重新启动导航，载入所选任务。"}
            if navigation.get("motion_backend") == "policy":
                policy = self.ros.policy_control_status()
                if not policy["ready"]:
                    return {"success": False, "message": policy["message"]}
                route_status = self.ros.policy_route_status()
                if not route_status.get("success") or route_status.get("state") != "idle":
                    return {"success": False, "message": "Policy 路线服务不可用或当前路线未处于 idle。"}
            online_camera_init = navigation.get("run_mode") == "online"
            expected_frame = "camera_init" if online_camera_init else "map"
            points_file = self._task_points_file(self.workspace_root, selected)
            try:
                point_data = json.loads(points_file.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                return {"success": False, "message": f"无法读取任务点位：{exc}"}
            point_frame = str(point_data.get("frame_id", "map"))
            if point_frame != expected_frame:
                return {
                    "success": False,
                    "message": (
                        f"当前导航使用 {expected_frame} 坐标系，任务点位文件是 {point_frame}。"
                        "请在当前导航模式下重新记录点位，或选择对应坐标系的任务。"
                    ),
                }
            if navigation.get("motion_backend") == "policy":
                localization = self.ros.policy_localization_status(online_camera_init=online_camera_init)
                if not localization.get("ready"):
                    return {"success": False, "message": localization.get("message", "Policy 定位未就绪。")}
                return self.ros.policy_route_start()
            return self.ros.route_start(online_camera_init=online_camera_init)

    def policy_control_step(self, payload):
        with self.task_lock:
            return self._policy_control_step_locked(payload)

    def cancel_policy_startup(self):
        with self.task_lock:
            if getattr(self, "heartbeat_expired", False) or self._heartbeat_age() > self.heartbeat_timeout_sec:
                return {"success": False, "message": "网页心跳未就绪；未停止任何进程。"}
            processes = self.processes.status()
            navigation = processes["navigation"]
            bridge = processes["policy_bridge"]
            if not bridge.get("running"):
                return {"success": False, "message": "当前没有运行中的 ONNX 控制桥。"}
            if navigation.get("running") and navigation.get("motion_backend") != "policy":
                return {"success": False, "message": "当前运行的是其他导航后端；未停止任何进程。"}
            policy = self.ros.policy_control_status()
            if not policy.get("available"):
                return {"success": False, "message": "Policy 状态未知；先重试状态查询，未停止任何进程。"}
            if navigation.get("running"):
                route = self.ros.policy_route_status()
                if not route.get("success") or route.get("state") != "idle":
                    return {"success": False, "message": "路线状态未知或非 idle；未停止任何进程。"}
            if (policy.get("mode") != "Passive" or policy.get("authorized") is not False
                    or policy.get("publisher_present") is not False
                    or policy.get("output_transition") != "none" or not policy.get("state_valid")):
                return {"success": False, "message": "Policy 不是已验证的未授权 Passive 状态；未停止任何进程。"}

            nav_pid, nav_error = (None, None)
            if navigation.get("running"):
                nav_pid, nav_error = self._verified_launch_group(
                    Path("/tmp/h1_2_ai_slam_nav.pid"), "h1_2_ai_slam_nav.backend", "ros2 launch h1_2_slam_bringup"
                )
            bridge_pid, bridge_error = self._verified_launch_group(
                Path("/tmp/h1_2_policy_bridge.pid"), None, "policy_bridge.launch.py"
            )
            if nav_error or bridge_error:
                return {"success": False, "message": nav_error or bridge_error}

            if nav_pid is not None and not self._interrupt_launch_group(nav_pid, timeout_sec=8.0):
                return {"success": False, "message": "Nav2 启动进程未在 8 秒内退出；Policy 未停止，请现场检查。"}
            if navigation.get("running") and self.processes._is_running(self.processes.nav_process):
                self.processes.nav_process.kill()
                try:
                    self.processes.nav_process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    return {"success": False, "message": "Nav2 已停止，但网页启动包装进程未退出；Policy 未停止。"}
            if not self._interrupt_launch_group(bridge_pid, timeout_sec=5.0):
                return {"success": False, "message": "Nav2 已停止，但 Policy bridge 未在 5 秒内退出；请现场检查。"}
            if nav_pid is not None:
                self._remove_matching_marker(Path("/tmp/h1_2_ai_slam_nav.pid"), str(nav_pid))
                self._remove_matching_backend_marker(nav_pid)
            self._remove_matching_marker(Path("/tmp/h1_2_policy_bridge.pid"), str(bridge_pid))
            self.logs.add("web", "cancelled un-authorized Policy startup while bridge Passive and route idle")
            message = "未授权的 ONNX 控制桥和导航栈已停止。" if navigation.get("running") else "未授权的 ONNX 控制桥已停止；导航栈未启动。"
            return {"success": True, "message": message}

    @staticmethod
    def _verified_launch_group(pid_path, backend_name, command_fragment):
        try:
            pid_text = pid_path.read_text(encoding="ascii").strip().split()[0]
            pid = int(pid_text)
            if pid <= 1 or (backend_name and pid_path.with_name(backend_name).read_text().split()[1] != "policy"):
                return None, "启动标记与 Policy 后端不匹配；未停止任何进程。"
            result = subprocess.run(
                ["ps", "-o", "sid=", "-o", "pgid=", "-o", "args=", "-p", str(pid)],
                check=False, capture_output=True, text=True, timeout=1.0,
            )
            fields = result.stdout.strip().split(maxsplit=2)
            if result.returncode != 0 or len(fields) != 3:
                return None, "找不到启动标记对应的进程；未停止任何进程。"
            sid, pgid, command = fields
            if sid != str(pid) or pgid != str(pid) or command_fragment not in command:
                return None, "启动进程组身份无法验证；未停止任何进程。"
            return pid, None
        except (OSError, ValueError, IndexError, subprocess.TimeoutExpired):
            return None, "缺少有效的启动进程标记；未停止任何进程。"

    @staticmethod
    def _interrupt_launch_group(pid, timeout_sec):
        try:
            os.killpg(pid, signal.SIGINT)
        except ProcessLookupError:
            return True
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                os.killpg(pid, 0)
            except ProcessLookupError:
                return True
            time.sleep(0.1)
        return False

    @staticmethod
    def _remove_matching_marker(path, expected):
        try:
            if path.read_text(encoding="ascii").strip().split()[0] == expected:
                path.unlink()
        except (OSError, IndexError):
            pass

    @staticmethod
    def _remove_matching_backend_marker(pid):
        path = Path("/tmp/h1_2_ai_slam_nav.backend")
        try:
            if int(path.read_text(encoding="ascii").split()[0]) == pid:
                path.unlink()
        except (OSError, ValueError, IndexError):
            pass

    def _policy_control_step_locked(self, payload):
        timeout_sec = getattr(self, "heartbeat_timeout_sec", HEARTBEAT_TIMEOUT_SEC)
        if getattr(self, "heartbeat_expired", False):
            return {"success": False, "message": "网页心跳曾超时；请显式重启导航实例后再做 Policy 准备。"}
        if self._heartbeat_age() > timeout_sec:
            return {"success": False, "message": "浏览器心跳未连接或已超时；Policy 控制请求未发送。"}
        if not all(payload.get(key) is True for key in (
            "confirm_debug_mode", "confirm_exclusive_control",
            "confirm_lowcmd_handoff", "confirm_robot_supported",
        )):
            return {"success": False, "message": "请先完成页面上的现场准入确认。"}

        navigation = self.processes.status()["navigation"]
        if not self.processes.status()["policy_bridge"].get("running"):
            return {"success": False, "message": "当前没有运行中的 ONNX Policy 控制桥。"}
        if navigation.get("running") and navigation.get("motion_backend") != "policy":
            return {"success": False, "message": "当前导航后端不是 ONNX Policy。"}
        if navigation.get("running"):
            route_status = self.ros.policy_route_status()
            if not route_status.get("success") or route_status.get("state") != "idle":
                return {"success": False, "message": "路线服务不可用或路线正在运行；先停止路线并确认 idle。"}

        step = str(payload.get("step", ""))
        expected = {
            "authorize_output": ("Passive", False, False),
            "fix_stand": ("Passive", True, True),
            "arm_hold_stand": ("FixStand", True, True),
            "arm_hold": ("ArmHoldStand", True, True),
        }
        if step not in expected:
            return {"success": False, "message": "未知的 Policy 准备步骤。"}
        status = self.ros.policy_control_status()
        if not status.get("available"):
            return {"success": False, "message": status.get("message", "Policy 状态不可用。")}
        if (status.get("profile") != "hardware" or status.get("joint_command_mode") != "position_pd"
                or status.get("output_enabled") is False or status.get("debug_confirmed") is False
                or status.get("exclusive_confirmed") is False
                or not status.get("navigation_enabled") or not status.get("continuous_navigation")):
            return {"success": False, "message": "Policy bridge 配置不满足连续导航准备条件。"}
        mode, authorized, publisher_present = expected[step]
        if status.get("mode") != mode or status.get("authorized") is not authorized:
            return {"success": False, "message": f"当前为 {status.get('mode')} / authorized={status.get('authorized')}，不能执行此步骤。"}
        if status.get("publisher_present") is not publisher_present:
            return {"success": False, "message": "Policy LowCmd publisher 状态与当前步骤不匹配。"}
        if not status.get("state_valid") or status.get("output_transition") != "none":
            return {"success": False, "message": "LowState 无效或输出转场尚未完成，请刷新状态后再操作。"}
        if step == "arm_hold" and payload.get("confirm_upright") is not True:
            return {"success": False, "message": "进入 ArmHold 前需现场确认站立姿态符合本次测试方案。"}

        result = self.ros.policy_control_command(step)
        self.logs.add("policy", f"web step={step} success={result.get('success', False)}")
        return result

    def reconnect_camera(self, config):
        if self.pc4_profile:
            return {"success": False, "message": "PC4 第一阶段未启用相机预览"}
        config["camera_preview_enabled"] = True
        self.save_config(config)
        self.ros.configure_camera(config)
        return {"success": True, "message": "相机话题订阅已重新连接"}

    def start_camera_preview(self, config):
        if self.pc4_profile:
            return {"success": False, "message": "PC4 第一阶段未启用相机预览"}
        config["camera_preview_enabled"] = True
        self.save_config(config)
        self.ros.configure_camera(config)
        return {"success": True, "message": "相机预览已开启"}

    def stop_camera_preview(self, config):
        config["camera_preview_enabled"] = False
        self.save_config(config)
        self.ros.configure_camera(config)
        return {"success": True, "message": "相机预览已关闭"}

    @staticmethod
    def _wait_for(predicate, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.1)
        return False

    def start_teaching_recording(self, payload):
        self.save_config(payload)
        if not self.ros.teaching_record_service_available(timeout_sec=0.2):
            process_start = self.processes.start_teaching_record_runner(payload)
            if not process_start.get("success") and "already running" not in process_start.get("message", ""):
                return process_start
            if not self._wait_for(lambda: self.ros.teaching_record_service_available(timeout_sec=0.2)):
                return {
                    "success": False,
                    "message": (
                        "teaching path recorder started, but start_teaching_path_recording "
                        "service did not become available"
                    ),
                }
        return self.ros.teaching_record_start()

    def start_teaching_route(self, payload):
        self.save_config(payload)
        if not self.ros.teaching_route_service_available(timeout_sec=0.2):
            process_start = self.processes.start_teaching_route_runner(payload)
            if not process_start.get("success") and "already running" not in process_start.get("message", ""):
                return process_start
            if not self._wait_for(lambda: self.ros.teaching_route_service_available(timeout_sec=0.2)):
                return {
                    "success": False,
                    "message": (
                        "teaching route runner started, but start_teaching_route service "
                        "did not become available"
                    ),
                }
        return self.ros.teaching_route_start()

    def vision_status(self):
        status = self.ros.vision_status()
        status["process"] = self.processes.status()["vision"]
        return status

    def memory_points(self):
        return self.memory_points_for_task(None)

    def memory_points_for_task(self, task_id, task_name=""):
        workspace_root = getattr(self, "workspace_root", Path("."))
        points_file = getattr(self, "points_file", workspace_root / "maps" / "memory_points.json")
        if task_id:
            config = self.default_config()
            profile = next((item for item in config["task_profiles"] if item["id"] == task_id), None)
            resolved_profile = profile or {"id": task_id, "name": task_name}
            points_file = self._task_points_file(self.workspace_root, resolved_profile)
            configured = Path(str(resolved_profile.get("memory_points_file", ""))).expanduser() if str(resolved_profile.get("memory_points_file", "")).strip() else None
            if not points_file.exists() and configured and configured.exists():
                points_file = configured
        try:
            data = json.loads(points_file.read_text(encoding="utf-8"))
            points = data.get("points", {})
            if not isinstance(points, dict):
                raise ValueError("points object is invalid")
        except FileNotFoundError:
            points = {}
        except Exception as exc:
            self.logs.add("web", f"failed to load memory points from {points_file}: {exc}")
            points = {}
        rows = []
        for name in sorted(points.keys(), key=point_sort_key):
            position = points[name].get("position", {})
            orientation = points[name].get("orientation", {})
            rows.append(
                {
                    "name": name,
                    "position": {
                        "x": float(position.get("x", 0.0)),
                        "y": float(position.get("y", 0.0)),
                        "z": float(position.get("z", 0.0)),
                    },
                    "orientation": {
                        "x": float(orientation.get("x", 0.0)),
                        "y": float(orientation.get("y", 0.0)),
                        "z": float(orientation.get("z", 0.0)),
                        "w": float(orientation.get("w", 1.0)),
                    },
                }
            )
        return rows

    def save_task_point(self, task_id, task_name, name, overwrite=False):
        name = str(name or "").strip()
        if not name or any(c in name for c in ",:\n\r"):
            return {"success": False, "message": "点位名称无效。"}
        navigation = self.processes.status()["navigation"]
        if navigation.get("running") and navigation.get("motion_backend") == "policy":
            result = self.ros.policy_save_memory_point(name, overwrite)
        else:
            result = self.ros.save_memory_point(name, overwrite)
        if not result.get("success"):
            return result
        config = self.default_config()
        profile = next((item for item in config["task_profiles"] if item["id"] == task_id), None)
        if not profile:
            profile = {"id": task_id, "name": task_name}
        target = self._task_points_file(self.workspace_root, profile)
        try:
            configured = Path(str(profile.get("memory_points_file", ""))).expanduser() if str(profile.get("memory_points_file", "")).strip() else None
            source = target if target.exists() else configured if configured and configured.exists() else None
            data = json.loads(source.read_text(encoding="utf-8")) if source else {"frame_id": "map", "points": {}}
            pose = result.get("pose")
            if not pose:
                return {"success": False, "message": "ROS 未返回当前位姿。"}
            existing_frame = str(data.get("frame_id", "map"))
            pose_frame = str(pose.get("frame_id", "map"))
            if data.get("points") and existing_frame != pose_frame:
                return {
                    "success": False,
                    "message": (
                        f"任务已有点位使用 {existing_frame} 坐标系，当前位姿是 {pose_frame}。"
                        "请新建任务后在当前导航模式重新记录点位。"
                    ),
                }
            points = data.setdefault("points", {})
            record = points.get(name, {})
            record.update({"position": pose["position"], "orientation": pose["orientation"]})
            points[name] = record
            data["frame_id"] = pose.get("frame_id", "map")
            data["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        except (OSError, ValueError) as exc:
            return {"success": False, "message": f"点位已保存到 ROS，但写入任务文件失败：{exc}"}
        return {"success": True, "message": f"已记录点位：{name}"}

    def delete_task_points(self, task_id, task_name, configured_file=""):
        if str(task_id) == "default":
            return {"success": False, "message": "默认任务不能删除。"}
        target = self._task_points_file(self.workspace_root, {"id": str(task_id), "name": str(task_name or "")})
        configured = Path(str(configured_file)).expanduser() if configured_file else None
        tasks_root = (self.workspace_root / "maps" / "tasks").resolve()
        if configured:
            try:
                if configured.resolve().parent == tasks_root:
                    target = configured
            except OSError:
                pass
        try:
            target.unlink(missing_ok=True)
        except OSError as exc:
            return {"success": False, "message": f"删除任务点位文件失败：{exc}"}
        return {"success": True, "message": "任务点位文件已删除。"}

    def validate_navigation(self, payload):
        normalize_task_profiles(payload)
        mode = str(payload.get("run_mode", "online"))
        motion_backend = str(payload.get("motion_backend", "unitree"))
        map_path = Path(str(payload.get("map", ""))).expanduser()
        pcd_path = Path(str(payload.get("pcd", ""))).expanduser()
        memory_route = normalize_memory_route(payload.get("memory_route", ""))
        if mode not in ("online", "navigation", "save_map", "save_pcd_map"):
            return False, f"Unsupported mode: {mode}"
        if motion_backend not in ("unitree", "policy"):
            return False, "motion_backend must be unitree or policy."
        if motion_backend == "policy" and mode not in ("online", "navigation"):
            return False, "Policy backend is available only for online or offline navigation."
        if motion_backend == "policy" and not (
            payload.get("confirm_debug_mode") is True and payload.get("confirm_exclusive_control") is True
        ):
            return False, "Policy startup requires debug-mode and exclusive-control confirmations."
        if mode == "navigation" and not map_path.is_file():
            return False, f"Map file does not exist: {map_path}"
        if mode == "navigation" and not pcd_path.is_file():
            return False, f"PCD file does not exist: {pcd_path}"
        params_file = str(payload.get("nav2_params_file", "")).strip()
        if params_file and not Path(params_file).expanduser().is_file():
            return False, f"Nav2 params file does not exist: {params_file}"
        if mode == "navigation" and not memory_route:
            return False, "Select at least one route point."
        if mode == "navigation":
            selected = next(item for item in payload["task_profiles"] if item["id"] == payload["selected_task_id"])
            if not hasattr(self, "points_file"):
                available_points = {point["name"] for point in self.memory_points()}
            else:
                available_points = {point["name"] for point in self.memory_points_for_task(selected["id"])}
            missing = [name for name in memory_route.split(",") if name not in available_points]
            if missing:
                return False, "当前任务包含不存在的点位：" + ", ".join(missing)
        if mode == "save_map" and not str(payload.get("map_prefix", "")).strip():
            return False, "Save map mode requires map_prefix."
        if mode == "save_pcd_map" and not str(payload.get("pcd_prefix", "")).strip():
            return False, "Save PCD map mode requires pcd_prefix."
        return True, ""

    def navigation_args(self, payload):
        normalize_task_profiles(payload)
        mode = str(payload.get("run_mode", "online"))
        args = [f"mode={mode}"]
        if mode in ("online", "navigation"):
            backend = str(payload.get("motion_backend", "unitree"))
            if backend not in ("unitree", "policy"):
                raise ValueError("motion_backend must be unitree or policy")
            args.append(f"motion_backend={backend}")
            if backend == "policy":
                args.extend((
                    "confirm_debug_mode=" + str(payload.get("confirm_debug_mode") is True).lower(),
                    "confirm_exclusive_control=" + str(payload.get("confirm_exclusive_control") is True).lower(),
                    "enable_remote_velocity=" + str(payload.get("enable_remote_velocity") is True).lower(),
                ))
        selected = next(item for item in payload["task_profiles"] if item["id"] == payload["selected_task_id"])
        turn_route = "" if payload.get("motion_backend") == "policy" else payload.get("turn_route", "")
        gesture_route = "" if payload.get("motion_backend") == "policy" else payload.get("gesture_route", "")
        task_args = [
            f"memory_route={normalize_memory_route(payload.get('memory_route', ''))}",
            f"gesture_route={gesture_route}",
            f"turn_route={turn_route}",
            f"memory_points_file={self._task_points_file(getattr(self, 'workspace_root', Path('.')), selected)}",
            f"auto_start_route={str(bool(payload.get('auto_start_route', False)) and payload.get('motion_backend') != 'policy').lower()}",
            f"return_to_start={str(bool(payload.get('return_to_start', False))).lower()}",
        ]
        def mode_params_path(value):
            path = Path(value).expanduser()
            # Online mapping publishes camera_init, while offline navigation
            # relies on localization to publish map. Only switch bundled
            # profiles so an operator-provided Nav2 file remains unchanged.
            if mode == "online" and path.name in ("nav2_h1_2.yaml", "nav2_h1_2_rpp.yaml"):
                return path.with_name("nav2_h1_2_camera_init.yaml")
            if mode == "navigation" and path.name in ("nav2_h1_2.yaml", "nav2_h1_2_camera_init.yaml"):
                return path.with_name("nav2_h1_2_rpp.yaml")
            return path
        if mode == "online":
            args.append(f"pcd_prefix={payload.get('pcd_prefix', 'maps/h1_2_lab')}")
            args.extend(task_args)
            params_file = str(payload.get("nav2_params_file", "")).strip()
            if params_file:
                args.append(f"params_file={mode_params_path(params_file)}")
            return args
        if mode == "save_map":
            args.append(f"map_prefix={payload.get('map_prefix', 'maps/h1_2_lab')}")
            return args
        if mode == "save_pcd_map":
            args.append(f"pcd_prefix={payload.get('pcd_prefix', 'maps/h1_2_lab')}")
            return args
        args.extend(
            [
            f"map={Path(str(payload.get('map', ''))).expanduser()}",
            f"localization={payload.get('localization', 'global_gicp')}",
            f"pcd={Path(str(payload.get('pcd', ''))).expanduser()}",
            f"initial_pose={payload.get('initial_pose') or 'START'}",
            ]
        )
        args.extend(task_args)
        params_file = str(payload.get("nav2_params_file", "")).strip()
        if params_file:
            args.append(f"params_file={mode_params_path(params_file)}")
        return args

class RequestHandler(BaseHTTPRequestHandler):
    server_version = "H12NavWebGui/0.1"

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            self._send_text(INDEX_HTML, "text/html; charset=utf-8")
        elif parsed.path == "/api/config":
            self._send_json(self.app.default_config())
        elif parsed.path in ("/api/info", "/api/status/info"):
            self._send_json({"http_port": self.app.http_port,
                             "heartbeat_ws_port": self.app.heartbeat_ws_port})
        elif parsed.path == "/api/points":
            task_id = parse_qs(parsed.query).get("task_id", [None])[0]
            task_name = parse_qs(parsed.query).get("task_name", [""])[0]
            self._send_json({"points": self.app.memory_points_for_task(task_id, task_name), "gestures": GESTURE_OPTIONS})
        elif parsed.path == "/api/status":
            processes = self.app.processes.status()
            navigation = processes["navigation"]
            online_camera_init = navigation.get("running") and navigation.get("run_mode") == "online"
            policy_backend = (
                navigation.get("running") and navigation.get("motion_backend") == "policy"
            ) or processes["policy_bridge"].get("running")
            policy_control = (
                self.app.ros.policy_control_status()
                if policy_backend
                else {"available": False, "ready": False, "message": "未选择 ONNX Policy"}
            )
            route = self.app.ros.policy_route_status() if policy_backend else self.app.ros.route_status()
            localization = (
                self.app.ros.policy_localization_status(online_camera_init=navigation.get("run_mode") == "online")
                if policy_backend and navigation.get("running")
                else self.app.ros.localization_status(online_camera_init=online_camera_init)
            )
            teaching_route = self.app.ros.teaching_route_status()
            self._send_json(
                {
                    "route": route,
                    "loaded_task": self.app.loaded_task if navigation["running"] else None,
                    "teaching_route": teaching_route,
                    "localization": localization,
                    "policy_control": policy_control,
                    "processes": processes,
                    "camera": self.app.ros.camera_status(),
                    "vision": self.app.ros.vision_status(),
                    "runtime_profile": self.app.runtime_profile,
                    "pc4_profile": self.app.pc4_profile,
                    "heartbeat_ws_port": self.app.heartbeat_ws_port,
                }
            )
        elif parsed.path == "/api/camera/status":
            self._send_json(self.app.ros.camera_status())
        elif parsed.path == "/api/camera/rgb.mjpg":
            self._send_mjpeg("rgb")
        elif parsed.path == "/api/camera/depth.mjpg":
            self._send_mjpeg("depth")
        elif parsed.path == "/api/vision/status":
            self._send_json(self.app.vision_status())
        elif parsed.path == "/api/vision/annotated.mjpg":
            self._send_mjpeg("annotated")
        elif parsed.path == "/api/logs":
            query = parse_qs(parsed.query)
            after = int(query.get("after", ["0"])[0] or 0)
            self._send_json({"entries": self.app.logs.since(after)})
        elif parsed.path == "/assets/h1_2_status.png":
            self._send_file(self.app.workspace_root / "src" / "h1_2_nav_tools" / "assets" / "h1_2_status.png", "image/png")
        else:
            self.send_error(404)

    def do_POST(self):
        parsed = urlparse(self.path)
        payload = self._read_json()
        if not isinstance(payload, dict):
            self._send_json({"success": False, "message": "请求配置必须是对象。"})
            return
        if parsed.path == "/api/heartbeat":
            # Retained for older clients, but HTTP traffic cannot keep the
            # navigation watchdog alive. Only a WebSocket Pong does that.
            self._send_json({"success": False, "message": "浏览器心跳已迁移至 WebSocket，请刷新页面。"})
            return
        if parsed.path in ("/api/config", "/api/navigation/start", "/api/route/start") or "task_profiles" in payload:
            try:
                normalize_task_profiles(payload)
            except ValueError as exc:
                self._send_json({"success": False, "message": str(exc)})
                return
        if self.app.ui_only and parsed.path != "/api/config":
            self.app.logs.add("ui-only", f"blocked control request: {parsed.path}")
            self._send_json(
                {
                    "success": False,
                    "message": "UI-only 模式：未连接 ROS，控制操作未执行",
                }
            )
            return
        if self.app.pc4_profile and parsed.path in (
            "/api/voice/start", "/api/voice/stop", "/api/vision/start", "/api/vision/stop",
            "/api/camera/reconnect", "/api/camera/start"
        ):
            self._send_json({"success": False, "message": "PC4 第一阶段未启用相机、语音或视觉功能。"})
            return
        if getattr(self.app, "heartbeat_expired", False) and parsed.path in (
            "/api/route/start", "/api/teaching_route/start", "/api/gesture"
        ):
            self._send_json({"success": False, "message": "浏览器心跳曾超时，请显式重新启动导航后再控制机器人。"})
            return
        if parsed.path == "/api/config":
            self.app.save_config(payload)
            self._send_json({"success": True, "message": "config saved"})
        elif parsed.path == "/api/task/delete":
            self._send_json(
                self.app.delete_task_points(
                    payload.get("task_id", ""),
                    payload.get("task_name", ""),
                    payload.get("memory_points_file", ""),
                )
            )
        elif parsed.path == "/api/points/save":
            self._send_json(self.app.save_task_point(payload.get("task_id", ""), payload.get("task_name", ""), payload.get("name", ""), payload.get("overwrite", False)))
        elif parsed.path == "/api/navigation/start":
            self._send_json(self.app.start_navigation_task(payload))
        elif parsed.path == "/api/policy/start":
            self._send_json(self.app.start_policy_control(payload))
        elif parsed.path == "/api/policy/control":
            self._send_json(self.app.policy_control_step(payload))
        elif parsed.path == "/api/policy/cancel_startup":
            self._send_json(self.app.cancel_policy_startup())
        elif parsed.path == "/api/navigation/stop_all":
            navigation = self.app.processes.status()["navigation"]
            result = self.app.processes.stop_all(payload)
            if result.get("success"):
                if navigation.get("motion_backend") == "policy":
                    self.app.ros.policy_route_stop()
                else:
                    self.app.ros.route_stop()
                self.app.ros.teaching_route_stop()
            self._send_json(result)
        elif parsed.path == "/api/voice/start":
            self.app.save_config(payload)
            self._send_json(self.app.processes.start_voice_control(payload))
        elif parsed.path == "/api/voice/stop":
            self._send_json(self.app.processes.stop_voice_control(payload))
        elif parsed.path == "/api/camera/reconnect":
            self._send_json(self.app.reconnect_camera(payload))
        elif parsed.path == "/api/camera/start":
            self._send_json(self.app.start_camera_preview(payload))
        elif parsed.path == "/api/camera/stop":
            self._send_json(self.app.stop_camera_preview(payload))
        elif parsed.path == "/api/vision/start":
            self.app.save_config(payload)
            self.app.ros.configure_camera(payload)
            self._send_json(self.app.processes.start_vision(payload))
        elif parsed.path == "/api/vision/stop":
            self._send_json(self.app.processes.stop_vision(payload))
        elif parsed.path == "/api/rviz/open":
            if self.app.pc4_profile:
                self._send_json({"success": False, "message": "请在 AI PC 本地启动 RViz。"})
                return
            self._send_json(
                self.app.processes.open_rviz(
                    str(payload.get("run_mode", "online")),
                    str(payload.get("rviz_mode", "auto")),
                )
            )
        elif parsed.path == "/api/rviz/close":
            if self.app.pc4_profile:
                self._send_json({"success": False, "message": "请在 AI PC 本地管理 RViz。"})
                return
            self._send_json(self.app.processes.close_rviz())
        elif parsed.path == "/api/rviz/restart":
            if self.app.pc4_profile:
                self._send_json({"success": False, "message": "请在 AI PC 本地启动 RViz。"})
                return
            self._send_json(
                self.app.processes.restart_rviz(
                    str(payload.get("run_mode", "online")),
                    str(payload.get("rviz_mode", "auto")),
                )
            )
        elif parsed.path == "/api/route/start":
            self._send_json(self.app.start_selected_task(payload))
        elif parsed.path == "/api/route/stop":
            navigation = self.app.processes.status()["navigation"]
            result = (self.app.ros.policy_route_stop() if navigation.get("motion_backend") == "policy"
                      else self.app.ros.route_stop())
            self._send_json(result)
        elif parsed.path == "/api/teaching_route/process/start":
            self.app.save_config(payload)
            self._send_json(self.app.processes.start_teaching_route_runner(payload))
        elif parsed.path == "/api/teaching_record/process/start":
            self.app.save_config(payload)
            self._send_json(self.app.processes.start_teaching_record_runner(payload))
        elif parsed.path == "/api/teaching_record/process/stop":
            record_stop = self.app.ros.teaching_record_stop_save()
            process_stop = self.app.processes.stop_teaching_record_runner()
            message = f"{record_stop.get('message', '')}; {process_stop.get('message', '')}".strip("; ")
            self._send_json(
                {
                    "success": bool(record_stop.get("success") or process_stop.get("success")),
                    "message": message,
                }
            )
        elif parsed.path == "/api/teaching_record/start":
            self._send_json(self.app.start_teaching_recording(payload))
        elif parsed.path == "/api/teaching_record/stop_save":
            self._send_json(self.app.ros.teaching_record_stop_save())
        elif parsed.path == "/api/teaching_route/process/stop":
            route_stop = self.app.ros.teaching_route_stop()
            process_stop = self.app.processes.stop_teaching_route_runner()
            message = f"{route_stop.get('message', '')}; {process_stop.get('message', '')}".strip("; ")
            self._send_json(
                {
                    "success": bool(route_stop.get("success") or process_stop.get("success")),
                    "message": message,
                }
            )
        elif parsed.path == "/api/teaching_route/start":
            self._send_json(self.app.start_teaching_route(payload))
        elif parsed.path == "/api/teaching_route/stop":
            self._send_json(self.app.ros.teaching_route_stop())
        elif parsed.path == "/api/gesture":
            self._send_json(self.app.ros.execute_gesture(str(payload.get("name", ""))))
        else:
            self.send_error(404)

    @property
    def app(self):
        return self.server.app

    def log_message(self, fmt, *args):
        return

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length <= 0:
            return {}
        raw = self.rfile.read(length).decode("utf-8")
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {}

    def _send_json(self, data):
        body = json.dumps(data).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, text, content_type):
        body = text.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path, content_type):
        try:
            body = Path(path).read_bytes()
        except OSError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "public, max-age=3600")
        self.end_headers()
        self.wfile.write(body)

    def _send_mjpeg(self, stream):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        sequence = 0
        try:
            while True:
                frame = self.app.ros.camera_frames.wait_for_frame(stream, sequence, timeout=1.0)
                if frame is None:
                    continue
                sequence = frame["sequence"]
                jpeg = frame["jpeg"]
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return

INDEX_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Jarvis机器人控制平台</title>
  <style>
    :root {
      --bg: #f4f6f8;
      --panel: #ffffff;
      --panel-2: #f8fafc;
      --panel-3: #eef2f6;
      --text: #17202a;
      --muted: #687386;
      --line: #d8e0ea;
      --accent: #2463eb;
      --accent-2: #0f766e;
      --danger: #b42318;
      --warn: #a15c00;
      --mono: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      --sans: Inter, "Noto Sans SC", "Microsoft YaHei", Arial, sans-serif;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--text);
      font-family: var(--sans);
      font-size: 14px;
      letter-spacing: 0;
    }
    button, input, select { font: inherit; }
    .app {
      height: 100vh;
      display: grid;
      grid-template-rows: 108px minmax(220px, 1fr) minmax(140px, 26vh);
    }
    header {
      min-height: 108px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 0 16px;
      border-bottom: 1px solid var(--line);
      background: #ffffff;
      position: sticky;
      top: 0;
      z-index: 3;
    }
    .brand { display: flex; align-items: center; gap: 14px; min-width: 0; }
    .logo {
      width: 112px;
      min-height: 92px;
      display: grid;
      justify-items: center;
      align-content: center;
      padding: 8px 0 6px;
      color: var(--text);
      line-height: 1;
    }
    .logo-main {
      font-size: 44px;
      font-weight: 950;
      letter-spacing: 0;
      font-family: "Noto Sans SC", "Microsoft YaHei", Arial, sans-serif;
    }
    .logo-rule {
      width: 88px;
      height: 2px;
      margin: 8px 0 7px;
      background: var(--text);
    }
    .logo-sub {
      font-size: 14px;
      font-weight: 850;
      white-space: nowrap;
      color: var(--muted);
    }
    .actions { display: flex; gap: 8px; flex-wrap: wrap; justify-content: flex-end; }
    .btn {
      min-height: 32px;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 0 12px;
      background: #ffffff;
      color: var(--text);
      font-weight: 750;
      cursor: pointer;
    }
    .btn:hover { background: #f1f5f9; border-color: #aeb9c8; }
    .btn.primary { background: #eff6ff; border-color: #bfdbfe; color: #1d4ed8; }
    .btn.primary:hover { background: #dbeafe; border-color: #93c5fd; }
    .btn.soft { background: #e9f7f5; border-color: #bdd7d4; color: #0f766e; }
    .btn.danger { background: #fff1f0; border-color: #f3b8b2; color: #b42318; }
    .btn:disabled { background: #f8fafc; border-color: #e2e8f0; color: #64748b; cursor: not-allowed; }
    .btn:focus-visible { outline: 2px solid #93c5fd; outline-offset: 2px; }
    .ui-only-badge {
      display: none;
      padding: 7px 10px;
      border: 1px solid #f1d49a;
      border-radius: 999px;
      background: #fffbeb;
      color: #92400e;
      font-size: 12px;
      font-weight: 850;
    }
    body.ui-only .ui-only-badge { display: none; }
    main {
      display: grid;
      grid-template-columns: 290px minmax(480px, 1fr) 320px;
      gap: 10px;
      padding: 10px;
      min-height: 0;
      overflow: hidden;
    }
    .panel {
      min-width: 0;
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
    }
    .panel-head {
      min-height: 38px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      padding: 8px 12px;
      border-bottom: 1px solid var(--line);
      background: var(--panel-2);
      font-weight: 850;
    }
    .body { padding: 12px; }
    .stack { display: grid; gap: 10px; align-content: start; min-height: 0; }
    label { display: block; margin-bottom: 6px; color: var(--muted); font-size: 12px; font-weight: 800; }
    input, select {
      width: 100%;
      height: 32px;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #ffffff;
      padding: 0 10px;
      color: var(--text);
      outline: none;
    }
    input:focus, select:focus { border-color: #22d3ee; box-shadow: 0 0 0 3px rgba(34, 211, 238, .14); }
    .check {
      display: flex; gap: 8px; align-items: center;
      color: var(--text); font-weight: 650; font-size: 13px;
    }
    .check input { width: 15px; height: 15px; }
    table { width: 100%; border-collapse: collapse; table-layout: fixed; }
    th, td { height: 38px; border-bottom: 1px solid var(--line); padding: 7px 8px; text-align: left; }
    th { color: var(--muted); background: #f3f6fa; font-size: 12px; font-weight: 850; }
    th:first-child, td:first-child { width: 44px; text-align: center; }
    th:nth-child(2), td:nth-child(2) { width: 72px; }
    th:nth-child(3), td:nth-child(3) { width: 118px; }
    tr:last-child td { border-bottom: 0; }
    .point {
      display: inline-flex; min-width: 32px; height: 24px;
      align-items: center; justify-content: center;
      background: #eff6ff; color: #1d4ed8;
      border: 1px solid #bfdbfe;
      border-radius: 6px; font-weight: 900;
    }
    .pos { color: var(--muted); font-family: var(--mono); font-size: 12px; }
    .two { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .status-row { display: grid; grid-template-columns: 88px 1fr; gap: 10px; padding: 8px 0; border-bottom: 1px solid var(--line); }
    .status-row:last-child { border-bottom: 0; }
    .key { color: var(--muted); font-size: 12px; font-weight: 850; }
    .pill {
      display: inline-flex; align-items: center; width: fit-content;
      min-height: 23px; padding: 3px 9px; border-radius: 999px;
      background: #f1f5f9; color: #475569; font-weight: 850; font-size: 12px;
    }
    .pill.ok { background: #ecfdf5; color: #166534; }
    .pill.warn { background: #fffbeb; color: #92400e; }
    .localization-indicator {
      width: 100%;
      min-height: 52px;
      border: 1px solid var(--line);
      border-radius: 7px;
      padding: 10px 14px;
      font-size: 15px;
      font-weight: 900;
      text-align: left;
      cursor: default;
      transition: background 160ms ease, border-color 160ms ease, color 160ms ease;
    }
    .localization-indicator.waiting {
      background: #f1f5f9;
      border-color: #d8e0ea;
      color: #64748b;
    }
    .localization-indicator.checking {
      background: #fffbeb;
      border-color: #f1d49a;
      color: #92400e;
    }
    .localization-indicator.ready {
      background: #ecfdf5;
      border-color: #86cda5;
      color: #166534;
      box-shadow: 0 0 0 3px rgba(34, 197, 94, 0.15);
    }
    .robot-overview {
      display: grid;
      grid-template-columns: 118px minmax(0, 1fr);
      gap: 12px;
      align-items: center;
      padding-bottom: 12px;
      border-bottom: 1px solid var(--line);
      margin-bottom: 10px;
    }
    .robot-figure {
      height: 184px;
      display: grid;
      place-items: center;
      overflow: hidden;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #f8fafc;
    }
    .robot-figure img {
      width: 100%;
      height: 100%;
      object-fit: cover;
      object-position: center;
      display: block;
    }
    .robot-summary {
      display: grid;
      gap: 10px;
      min-width: 0;
    }
    .status-metric {
      display: grid;
      gap: 5px;
      width: 100%;
      min-height: 72px;
      padding: 12px 14px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #ffffff;
      text-align: left;
      cursor: default;
      align-content: center;
      transition: background 160ms ease, border-color 160ms ease, box-shadow 160ms ease;
    }
    .status-metric .metric-label {
      color: var(--muted);
      font-size: 12px;
      font-weight: 850;
    }
    .status-metric .metric-value {
      color: var(--text);
      font-size: 18px;
      font-weight: 950;
      line-height: 1.25;
      word-break: break-word;
    }
    .status-metric.ready {
      background: #ecfdf5;
      border-color: #86cda5;
      box-shadow: none;
    }
    .status-metric.ready .metric-value { color: #166534; }
    .status-metric.waiting {
      background: #fffbeb;
      border-color: #f1d49a;
    }
    .status-metric.waiting .metric-value { color: #92400e; }
    .status-metric.motion {
      background: #f8fafc;
      border-color: #d8e0ea;
    }
    .status-metric.motion.active {
      background: #eff6ff;
      border-color: #bfdbfe;
      box-shadow: none;
    }
    .status-metric.motion.active .metric-value { color: #1d4ed8; }
    .runtime-panel {
      padding-top: 12px;
      border-top: 1px solid var(--line);
      display: grid;
      gap: 12px;
    }
    .runtime-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      color: var(--text);
      font-weight: 900;
    }
    .runtime-grid {
      display: grid;
      grid-template-columns: 92px minmax(0, 1fr);
      gap: 12px;
      align-items: center;
    }
    .runtime-ring {
      --percent: 0%;
      width: 80px;
      height: 80px;
      display: grid;
      place-items: center;
      border-radius: 50%;
      background: conic-gradient(#86cda5 var(--percent), #e2e8f0 0);
      box-shadow: inset 0 0 0 1px rgba(255,255,255,.06);
      position: relative;
    }
    .runtime-ring::after {
      content: "";
      position: absolute;
      inset: 8px;
      border-radius: 50%;
      background: #ffffff;
      border: 1px solid var(--line);
    }
    .runtime-ring span {
      position: relative;
      z-index: 1;
      color: var(--text);
      font-size: 17px;
      font-weight: 950;
    }
    .runtime-list {
      display: grid;
      gap: 7px;
    }
    .runtime-item {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 8px;
      align-items: center;
      min-height: 28px;
      color: var(--muted);
      font-size: 12px;
      font-weight: 800;
    }
    .runtime-dot {
      min-width: 52px;
      min-height: 22px;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      padding: 2px 8px;
      border-radius: 999px;
      background: #f1f5f9;
      color: #475569;
      font-size: 12px;
      font-weight: 900;
    }
    .runtime-dot.ok { background: #ecfdf5; color: #166534; }
    .runtime-dot.warn { background: #fffbeb; color: #92400e; }
    .runtime-hidden { display: none; }
    .left-rail, .center-stage {
      height: 100%;
      max-height: 100%;
      min-height: 0;
      overflow: hidden;
    }
    .right-rail {
      height: 100%;
      max-height: 100%;
      min-height: 0;
      overflow-x: hidden;
      overflow-y: scroll;
      overscroll-behavior-y: contain;
      scrollbar-gutter: stable;
      scrollbar-color: #8492a3 #e9edf2;
      scrollbar-width: auto;
    }
    .left-rail { display: block; }
    .left-rail > .panel { display: flex; flex-direction: column; height: 100%; min-height: 0; }
    .left-rail > .panel > .body { flex: 1 1 auto; min-height: 0; overflow-x: hidden; overflow-y: scroll; }
    .center-stage > .panel { height: 100%; min-height: 0; }
    .center-stage { display: block; }
    .right-rail { display: block; }
    .right-rail > .panel + .panel { margin-top: 10px; }
    .route-panel { display: flex; flex-direction: column; overflow: hidden; }
    .route-panel .body { flex: 1 1 auto; min-height: 0; overflow-x: hidden; overflow-y: scroll; }
    .right-rail::-webkit-scrollbar,
    .left-rail > .panel > .body::-webkit-scrollbar, .route-panel .body::-webkit-scrollbar,
    footer pre::-webkit-scrollbar, .policy-wizard-modal::-webkit-scrollbar {
      width: 14px;
      height: 14px;
    }
    .right-rail::-webkit-scrollbar-track,
    .left-rail > .panel > .body::-webkit-scrollbar-track, .route-panel .body::-webkit-scrollbar-track,
    footer pre::-webkit-scrollbar-track, .policy-wizard-modal::-webkit-scrollbar-track {
      background: #e9edf2;
    }
    .right-rail::-webkit-scrollbar-thumb,
    .left-rail > .panel > .body::-webkit-scrollbar-thumb, .route-panel .body::-webkit-scrollbar-thumb,
    footer pre::-webkit-scrollbar-thumb, .policy-wizard-modal::-webkit-scrollbar-thumb {
      min-height: 36px;
      border: 3px solid #e9edf2;
      border-radius: 8px;
      background: #8492a3;
    }
    .right-rail::-webkit-scrollbar-thumb:hover, .left-rail > .panel > .body::-webkit-scrollbar-thumb:hover,
    .route-panel .body::-webkit-scrollbar-thumb:hover,
    footer pre::-webkit-scrollbar-thumb:hover, .policy-wizard-modal::-webkit-scrollbar-thumb:hover {
      background: #596a7d;
    }
    .center-controls .body { padding: 10px; }
    .control-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
    }
    .control-grid .btn {
      width: 100%;
      min-height: 36px;
      padding: 7px 10px;
      background: #eff6ff;
      border-color: #bfdbfe;
      color: #1d4ed8;
      font-weight: 800;
    }
    .policy-step-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 7px; }
    .policy-step-grid .btn { width: 100%; min-height: 36px; padding: 6px; }
    .policy-checks { display: grid; gap: 7px; }
    .policy-checks label { display: flex; align-items: flex-start; gap: 7px; line-height: 1.35; }
    .policy-checks input { margin-top: 2px; flex: 0 0 auto; }
    .policy-note { margin: 0; color: var(--muted); font-size: 12px; line-height: 1.4; }
    .policy-control-panel .body { display: grid; gap: 10px; }
    .policy-summary { display: grid; gap: 0; border: 1px solid var(--line); border-radius: 6px; overflow: hidden; }
    .policy-summary-row { display: grid; grid-template-columns: minmax(0, 1fr) auto; align-items: center; gap: 10px; min-height: 38px; padding: 7px 10px; border-bottom: 1px solid var(--line); }
    .policy-summary-row:last-child { border-bottom: 0; }
    .policy-summary-label { min-width: 0; font-size: 13px; font-weight: 650; }
    .policy-summary-state { color: var(--muted); font-size: 12px; text-align: right; }
    .policy-summary-row.active { background: #f0f7ff; }
    .policy-summary-row.done .policy-summary-state { color: #16834a; }
    .policy-control-panel .policy-start-button { width: 100%; min-height: 40px; }
    .policy-wizard-section-title { margin: 2px 0 -2px; font-size: 13px; font-weight: 750; }
    .control-grid .btn:hover { background: #dbeafe; border-color: #93c5fd; }
    .control-grid .btn:disabled { background: #f3f7fd; border-color: #d7e3f4; color: #7890b4; }
    .setup-grid { display: grid; grid-template-columns: 1fr; gap: 10px; }
    .setup-grid > div { min-width: 0; }
    .left-rail .two { grid-template-columns: 1fr; }
    .advanced-config { max-height: none; }
    .route-panel { min-height: 0; }
    .route-intro, #task_count { display: none; }
    .task-toolbar { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
    .task-toolbar .route-intro { margin: 0; margin-right: auto; }
    .task-name { display: flex; gap: 8px; }
    .task-name input { min-width: 0; }
    .task-name .btn { flex-shrink: 0; white-space: nowrap; }
    #task_feedback:empty { display: none; }
    #task_feedback { color: #166534; font-size: 13px; overflow-wrap: anywhere; }
    #task_feedback.error { color: #b42318; }
    .modal-backdrop { position: fixed; inset: 0; z-index: 20; display: none; place-items: center; overflow-x: hidden; overflow-y: auto; overscroll-behavior: contain; padding: 12px; background: rgba(15, 23, 42, .28); }
    .modal-backdrop.open { display: grid; }
    .modal { width: min(420px, 100%); background: #fff; border: 1px solid var(--line); border-radius: 8px; box-shadow: 0 18px 50px rgba(15,23,42,.18); padding: 20px; }
    .policy-wizard-modal { width: calc(100vw - 24px); max-width: 520px; min-width: 0; max-height: calc(100dvh - 24px); overflow-x: hidden; overflow-y: auto; padding: 18px; }
    .policy-wizard-head { display: flex; align-items: flex-start; justify-content: space-between; gap: 12px; }
    .policy-wizard-head h3 { margin-bottom: 5px; }
    .policy-wizard-subtitle, .policy-wizard-detail { margin: 0; color: var(--muted); font-size: 13px; line-height: 1.45; }
    .policy-wizard-status { padding: 10px 12px; border: 1px solid var(--line); border-left: 4px solid #d97706; background: #fffaf0; border-radius: 4px; line-height: 1.45; overflow-wrap: anywhere; }
    .policy-wizard-status.ready { border-left-color: #16834a; background: #f0fdf4; }
    .policy-wizard-preflight { display: grid; gap: 10px; min-width: 0; padding: 12px; background: #f8fafc; border: 1px solid var(--line); border-radius: 6px; }
    .policy-wizard-preflight strong { margin-bottom: 1px; }
    .policy-wizard-preflight label, #confirm_upright_label { display: grid; grid-template-columns: 18px minmax(0, 1fr); align-items: start; gap: 10px; min-width: 0; margin: 0; color: var(--text); font-size: 13px; font-weight: 600; line-height: 1.45; }
    .policy-wizard-preflight label span, #confirm_upright_label span { min-width: 0; overflow-wrap: anywhere; }
    .policy-wizard-preflight input[type="checkbox"], #confirm_upright_label input[type="checkbox"] { width: 18px; min-width: 18px; height: 18px; margin: 1px 0 0; padding: 0; flex: none; }
    #confirm_upright_label { padding: 10px 12px; border: 1px solid var(--line); border-radius: 6px; background: #f8fafc; }
    .policy-wizard-steps { display: grid; gap: 6px; }
    .policy-wizard-step { display: grid; grid-template-columns: 28px minmax(0, 1fr) auto; align-items: center; gap: 9px; min-height: 42px; padding: 6px 8px; border-bottom: 1px solid var(--line); }
    .policy-wizard-step[aria-current="step"] { background: #f0f7ff; }
    .policy-wizard-step.done .policy-step-state { color: #16834a; }
    .policy-step-number { color: var(--muted); font-variant-numeric: tabular-nums; }
    .policy-step-title { font-weight: 650; }
    .policy-step-state { color: var(--muted); font-size: 12px; text-align: right; }
    .policy-wizard-modal .modal-actions { position: sticky; bottom: -20px; padding: 12px 0 0; background: #fff; }
    .policy-wizard-recovery { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 12px; padding-top: 10px; border-top: 1px solid var(--line); }
    .policy-wizard-recovery .btn { min-height: 34px; }
    .modal h3 { margin: 0 0 16px; font-size: 18px; }
    .modal-actions { display: flex; justify-content: flex-end; gap: 8px; margin-top: 18px; }
    .route-table { overflow-x: auto; }
    .route-table table { min-width: 820px; }
    .route-table th:nth-child(2), .route-table td:nth-child(2) { width: 72px; overflow-wrap: anywhere; }
    .route-table th:nth-child(3), .route-table td:nth-child(3) { width: 105px; }
    .route-table th:nth-child(4), .route-table td:nth-child(4) { width: 125px; }
    .route-table th:nth-child(5), .route-table td:nth-child(5) { width: 90px; }
    .route-table th:nth-child(6), .route-table td:nth-child(6) { width: 330px; }
    .pose-values { display: grid; gap: 3px; white-space: nowrap; line-height: 1.35; }
    .route-table td input { width: 16px; height: 16px; }
    .route-table .btn { padding: 3px 8px; min-height: 28px; margin-right: 3px; }
    .route-table .missing { color: #b42318; }
    .route-table tr.selected { background: #f8fbff; }
    #loaded_task_hint { padding: 9px; background: #f8fafc; border-radius: 6px; margin: 0; }
    footer {
      display: grid;
      grid-template-columns: minmax(0, 1fr);
      gap: 10px;
      padding: 0 10px 10px;
      min-height: 0;
    }
    footer > .panel { min-height: 0; display: flex; flex-direction: column; }
    pre {
      margin: 0;
      flex: 1;
      min-height: 0;
      max-height: none;
      overflow: auto;
      padding: 12px;
      font-family: var(--mono);
      font-size: 12px;
      line-height: 1.55;
      white-space: pre-wrap;
      word-break: break-word;
    }
    #logs { background: #f8fafc; color: #334155; }
    #logs { user-select: text; }
    @media (max-width: 1220px) {
      .app { height: auto; min-height: 100vh; grid-template-rows: auto auto minmax(140px, 24vh); }
      main, footer, .setup-grid { grid-template-columns: 1fr; }
      header { height: auto; padding: 12px 14px; align-items: flex-start; flex-direction: column; }
      .actions { justify-content: flex-start; }
      .left-rail, .center-stage { height: auto; max-height: 70vh; min-height: 0; overflow: hidden; }
      .right-rail { height: auto; max-height: 70vh; min-height: 0; overflow-x: hidden; overflow-y: scroll; }
      .left-rail > .panel { height: 70vh; max-height: 70vh; min-height: 0; }
      .left-rail > .panel > .body { min-height: 0; overflow-y: scroll; }
      .center-stage > .panel { height: auto; min-height: 0; }
      .center-stage { display: block; }
      .route-panel .body { max-height: 58vh; overflow-y: scroll; }
      .right-rail { display: block; }
      .advanced-config { max-height: none; }
      .policy-wizard-modal { width: calc(100vw - 16px); max-width: none; max-height: calc(100dvh - 16px); padding: 14px; }
      .policy-wizard-head { gap: 8px; }
      .policy-wizard-preflight { padding: 10px; }
      .policy-wizard-step { grid-template-columns: 22px minmax(0, 1fr); }
      .policy-step-state { grid-column: 2; text-align: left; }
    }
  </style>
</head>
<body>
  <div class="app">
    <header>
      <!-- UI ONLY · 未连接 ROS 状态不在界面展示。 -->
      <div class="brand">
        <div class="logo" aria-label="引望 工具装备团队">
          <div class="logo-main">引望</div>
          <div class="logo-rule"></div>
          <div class="logo-sub">工具装备团队</div>
        </div>
      </div>
      <div class="actions">
        <button class="btn primary" onclick="startNavigation()">启动当前模式</button>
        <button id="start_policy_mode_button" class="btn danger" onclick="startPolicyWizard()">启动 ONNX 模式</button>
        <button class="btn danger" onclick="stopAll()">停止全部</button>
        <button id="native_rviz_button" class="btn soft" onclick="openNativeRviz()">打开原生 RViz</button>
      </div>
    </header>

    <main>
      <aside class="left-rail stack">
        <section class="panel">
          <div class="panel-head">设置</div>
          <div class="body stack advanced-config">
            <div class="setup-grid">
              <div><label>Run Mode</label><select id="run_mode" onchange="onModeChanged()"><option value="online">在线建图导航</option><option value="navigation">离线地图导航</option></select></div>
              <div><label>运动后端</label><select id="motion_backend"><option value="unitree">Unitree</option><option value="policy">ONNX Policy</option></select><small id="motion_backend_hint" class="policy-note" hidden>选择仅修改配置；启动与准备请用顶部“启动 ONNX 模式”。</small></div>
              <label class="check"><input id="enable_remote_velocity" type="checkbox"> ONNX 与遥控器速度共存（摇杆优先，回中后恢复导航）</label>
              <div><label>RViz Mode</label><select id="rviz_mode"><option value="auto">跟随 Run Mode</option><option value="online">在线 RViz</option><option value="offline">离线 RViz</option></select></div>
              <div><label>Map YAML</label><input id="map"></div>
              <div><label>PCD Map</label><input id="pcd"></div>
              <div><label>Localization</label><select id="localization"><option>global_gicp</option><option>icp</option></select></div>
              <div><label>Initial Pose</label><input id="initial_pose" value="START"></div>
              <div><label>Nav2 Params</label><input id="nav2_params_file" placeholder="/home/llx/h1_nav2_ros2_ws/src/h1_2_nav_tools/config/nav2_h1_2_rpp.yaml"></div>
              <div class="runtime-hidden"><label>Teaching Path</label><input id="teaching_path_file"></div>
              <div class="runtime-hidden"><label>Teaching Events</label><input id="teaching_events_file"></div>
              <div><label>Voice SSH</label><input id="voice_remote_host" placeholder="unitree@192.168.x.x"></div>
              <div><label>Voice Workspace</label><input id="voice_workspace" value="~/ros2_ws"></div>
              <div><label>Voice ROS setup</label><input id="voice_ros_setup" value="/opt/ros/humble/setup.bash"></div>
            </div>
            <div class="two">
              <label class="check"><input id="auto_start_route" type="checkbox"> auto_start_route</label>
              <label class="check"><input id="return_to_start" type="checkbox"> return_to_start</label>
            </div>
            <label class="check runtime-hidden"><input id="teaching_loop" type="checkbox"> teaching_loop</label>
            <button class="btn" onclick="saveConfig()">保存配置</button>
          </div>
        </section>
      </aside>
      <section class="center-stage">
        <section class="panel route-panel">
          <div class="panel-head"><span>任务与动作编排</span><span id="save_state" class="pill">已保存</span></div>
          <!-- 任务 · 点位与动作编排 -->
          <div class="body stack">
            <div><label for="task_select">当前任务</label><select id="task_select" onchange="switchTask(this.value)"></select></div>
            <div class="task-toolbar">
              <button class="btn primary" onclick="openNewTaskModal()">新建任务</button>
              <button class="btn" onclick="createTask(true)">复制任务</button>
              <button id="delete_task" class="btn" onclick="deleteTask()">删除任务</button>
              <button id="save_tasks" class="btn primary" onclick="saveConfig()">保存任务与设置</button>
            </div>
            <div class="task-name"><input id="task_name" maxlength="80" aria-label="任务名称"><button class="btn" onclick="renameTask()">重命名</button></div>
            <div id="task_feedback" role="status" aria-live="polite"></div>
            <div id="loaded_task_hint"></div>
            <div class="task-toolbar">
              <span id="task_count" class="route-intro"></span>
              <input id="point_name" placeholder="点位名称" maxlength="40" style="max-width:150px">
              <button class="btn primary" onclick="saveCurrentPoint()">记录当前位置</button>
              <button class="btn" onclick="selectAll(true)">全选点位</button>
              <button class="btn" onclick="selectAll(false)">清空</button>
              <button class="btn" onclick="loadPoints().catch(showError)">刷新点位</button>
            </div>
            <div class="route-table"><table>
              <thead><tr><th>启用</th><th>点位</th><th>动作</th><th>到点转向</th><th>顺序</th><th>位置与姿态</th></tr></thead>
              <tbody id="routeBody"></tbody>
            </table></div>
          </div>
        </section>
      </section>
      <aside class="right-rail stack">
        <section class="panel center-controls">
          <div class="panel-head">主控制</div>
          <div class="body control-grid">
            <button id="start_route_button" class="btn primary" onclick="startRoute()" disabled>开始巡航</button>
            <button id="start_teaching_route_button" class="runtime-hidden" disabled></button>
            <button id="voice_start_button" class="btn primary" onclick="postAction('/api/voice/start', true)">启动语音控制</button>
            <button id="voice_stop_button" class="btn primary" onclick="postAction('/api/voice/stop', true)">停止语音控制</button>
            <button class="btn soft" onclick="saveSnapshot('save_map')">保存 2D 地图</button>
            <button class="btn soft" onclick="saveSnapshot('save_pcd_map')">保存 3D 点云</button>
            <button class="btn primary" onclick="gesture('wave_hand')">测试挥手</button>
            <button id="shake_hand_button" class="btn primary" onclick="toggleShakeHand()">出手 / 收手</button>
            <button id="native_rviz_open_button" class="btn primary" onclick="openNativeRviz()">打开原生 RViz</button>
            <button id="native_rviz_close_button" class="btn primary" onclick="closeNativeRviz()">关闭 RViz</button>
            <button id="native_rviz_restart_button" class="btn primary" onclick="postAction('/api/rviz/restart', true)">重启 RViz</button>
            <button class="btn primary" onclick="refreshStatus()">刷新状态</button>
          </div>
        </section>
        <section class="panel policy-control-panel" id="policy_control_panel">
          <div class="panel-head"><span>ONNX Policy 控制</span><span id="policy_mode_badge" class="pill">未启动</span></div>
          <div class="body stack">
            <div id="policy_control_status" class="policy-wizard-status" role="status">未启动。选择 ONNX 只是选择后端，不会启动机器人控制。</div>
            <div class="policy-summary" aria-label="ONNX 控制流程状态">
              <div id="policy_overview_stack" class="policy-summary-row"><span class="policy-summary-label">1. ONNX 控制桥</span><span id="policy_overview_stack_state" class="policy-summary-state">未启动</span></div>
              <div id="policy_overview_control" class="policy-summary-row"><span class="policy-summary-label">2. ONNX 运动控制</span><span id="policy_overview_control_state" class="policy-summary-state">未就绪</span></div>
              <div id="policy_overview_route" class="policy-summary-row"><span class="policy-summary-label">3. 导航栈</span><span id="policy_overview_route_state" class="policy-summary-state">未启动</span></div>
              <div id="policy_overview_cruise" class="policy-summary-row"><span class="policy-summary-label">4. 巡航任务</span><span id="policy_overview_cruise_state" class="policy-summary-state">未开始</span></div>
            </div>
            <p class="policy-note">ONNX 就绪不会自动开始巡航；路线需在任务区单独启动。</p>
          </div>
        </section>
        <section class="panel">
          <div class="panel-head">机器人状态</div>
          <div class="body">
            <div class="robot-overview">
              <div class="robot-figure">
                <img id="robot_status_image" src="/assets/h1_2_status.png" alt="H1-2 Robot">
              </div>
              <div class="robot-summary">
                <div class="runtime-hidden">
                  <button id="cruise_ready_card" class="status-metric waiting" type="button" disabled>
                    <div id="cruise_ready_text" class="metric-value">等待定位</div>
                  </button>
                </div>
                <button id="motion_state_card" class="status-metric motion" type="button" disabled>
                  <div class="metric-label">当前运动状态</div>
                  <div id="motion_state" class="metric-value">待机</div>
                </button>
              </div>
            </div>
            <div class="runtime-panel">
              <div class="runtime-head">
                <span>系统运行状态</span>
                <span id="battery_state" class="pill">未接入</span>
              </div>
              <div class="runtime-grid">
                <div id="battery_ring" class="runtime-ring" style="--percent: 0%">
                  <span id="battery_percent">0%</span>
                </div>
                <div class="runtime-list">
                  <div class="runtime-item"><span>电压</span><span id="battery_voltage" class="runtime-dot">--</span></div>
                  <div class="runtime-item"><span>温度</span><span id="battery_temperature" class="runtime-dot">--</span></div>
                  <div class="runtime-item"><span>充放电</span><span id="battery_charge_state" class="runtime-dot">--</span></div>
                  <div class="runtime-item"><span>循环次数</span><span id="battery_cycles" class="runtime-dot">--</span></div>
                </div>
              </div>
            </div>
            <div class="runtime-hidden">
              <span id="state" class="pill">-</span>
              <span id="goal">-</span>
              <span id="teaching_state" class="pill">-</span>
              <span id="teaching_goal">-</span>
              <span id="localization_state">定位未稳定</span>
              <span id="processes">-</span>
              <span id="message">-</span>
              <span id="teaching_message">-</span>
            </div>
          </div>
        </section>
      </aside>
    </main>

    <div id="policy_wizard_modal" class="modal-backdrop" role="dialog" aria-modal="true" aria-labelledby="policy_wizard_title" onclick="closePolicyWizard(event)">
      <div class="modal policy-wizard-modal" onclick="event.stopPropagation()">
        <div class="policy-wizard-head">
          <div>
            <h3 id="policy_wizard_title">进入 ONNX 控制模式</h3>
        <p class="policy-wizard-subtitle">按当前状态完成准备；每一步都会等待桥接状态确认。</p>
          </div>
          <span id="policy_wizard_phase" class="pill">待启动</span>
        </div>
        <div id="policy_wizard_status" class="policy-wizard-status" role="status" aria-live="polite">先启动 ONNX 控制桥并完成 ArmHold 准备，之后再单独启动导航栈。</div>
        <p id="policy_wizard_detail" class="policy-wizard-detail"></p>
        <div class="policy-wizard-preflight">
          <strong>启动前现场确认</strong>
          <label><input id="confirm_debug_mode" type="checkbox"><span>机器人当前处于 Debug 模式</span></label>
          <label><input id="confirm_exclusive_control" type="checkbox"><span>已确认没有其他 LowCmd 控制进程</span></label>
          <label><input id="confirm_policy_writer" type="checkbox"><span>已现场核实其他 LowCmd writer 已停止，控制权已交给 Policy</span></label>
          <label><input id="confirm_policy_support" type="checkbox"><span>已按本次测试方案做好机器人支撑，并确认急停可用</span></label>
        </div>
        <div class="policy-wizard-section-title">准备进度</div>
        <div class="policy-wizard-steps" aria-label="ONNX 控制阶段">
          <div id="policy_step_start" class="policy-wizard-step"><span class="policy-step-number">1</span><span class="policy-step-title">启动 ONNX 控制桥</span><span id="policy_step_start_state" class="policy-step-state">等待启动</span></div>
          <div id="policy_step_authorize" class="policy-wizard-step"><span class="policy-step-number">2</span><span class="policy-step-title">授权 LowCmd 输出</span><span id="policy_step_authorize_state" class="policy-step-state">等待启动</span></div>
          <div id="policy_step_fix" class="policy-wizard-step"><span class="policy-step-number">3</span><span class="policy-step-title">FixStand</span><span id="policy_step_fix_state" class="policy-step-state">等待授权</span></div>
          <div id="policy_step_hold_stand" class="policy-wizard-step"><span class="policy-step-number">4</span><span class="policy-step-title">ArmHoldStand</span><span id="policy_step_hold_stand_state" class="policy-step-state">等待 FixStand</span></div>
          <div id="policy_step_arm_hold" class="policy-wizard-step"><span class="policy-step-number">5</span><span class="policy-step-title">进入 ArmHold 控制状态</span><span id="policy_step_arm_hold_state" class="policy-step-state">等待站立准备</span></div>
          <div id="policy_step_navigation" class="policy-wizard-step"><span class="policy-step-number">6</span><span class="policy-step-title">启动导航栈</span><span id="policy_step_navigation_state" class="policy-step-state">等待 ArmHold</span></div>
        </div>
        <label id="confirm_upright_label"><input id="confirm_policy_upright" type="checkbox"><span>我已现场确认站立姿态符合本次测试方案</span></label>
        <p class="policy-note">授权会开始向 LowCmd 发布。网页不能自动识别其他 DDS writer；每次姿态转换前都要现场观察。ArmHold 后再启动导航栈，最后单独点击“开始巡航”。</p>
        <div class="policy-wizard-recovery">
          <button id="policy_retry_status" class="btn" onclick="retryPolicyStatus()">重试状态</button>
          <button id="policy_cancel_startup" class="btn danger" onclick="cancelPolicyStartup()" disabled>取消未授权启动</button>
        </div>
        <div class="modal-actions">
          <button class="btn" onclick="closePolicyWizard()">稍后</button>
          <button id="policy_wizard_next" class="btn danger" onclick="policyWizardNext()">启动 ONNX 模式</button>
        </div>
      </div>
    </div>

    <div id="new_task_modal" class="modal-backdrop" role="dialog" aria-modal="true" aria-labelledby="new_task_title" onclick="closeNewTaskModal(event)">
      <div class="modal" onclick="event.stopPropagation()">
        <h3 id="new_task_title">新建任务</h3>
        <label for="new_task_name">任务名称</label>
        <input id="new_task_name" maxlength="80" placeholder="请输入任务名称">
        <p id="new_task_error" class="error" role="alert"></p>
        <div class="modal-actions">
          <button class="btn" onclick="closeNewTaskModal()">取消</button>
          <button class="btn primary" onclick="confirmNewTask()">创建任务</button>
        </div>
      </div>
    </div>

    <footer>
      <section class="panel">
        <div class="panel-head">Logs</div>
        <pre id="logs"></pre>
      </section>
    </footer>
  </div>

  <script>
    let points = [];
    let taskProfiles = [];
    let selectedTaskId = "";
    let baseConfig = {};
    let latestStatus = {};
    let connected = false;
    let statusRefreshInFlight = false;
    let revision = 0;
    let savedRevision = 0;
    let saving = false;
    let starting = false;
    let lastLogSeq = 0;
    let logsAutoFollow = true;
    let logsSelecting = false;
    let logsPollInFlight = false;
    const MAX_LOG_TEXT_LENGTH = 200000;
    let startGesture = "";
    let startTurn = "";
    let shakeHandExtended = false;

    const $ = (id) => document.getElementById(id);

    function payload() {
      const steps = currentTask()?.steps || [];
      const selected = steps.map(step => step.point);
      const gestures = steps.filter(step => step.gesture).map(step => `${step.point}:${step.gesture}`);
      return {
        ...baseConfig,
        task_profiles: structuredClone(taskProfiles),
        selected_task_id: selectedTaskId,
        run_mode: $("run_mode").value,
        motion_backend: $("motion_backend").value,
        confirm_debug_mode: $("confirm_debug_mode").checked,
        confirm_exclusive_control: $("confirm_exclusive_control").checked,
        enable_remote_velocity: $("enable_remote_velocity").checked,
        rviz_mode: $("rviz_mode").value,
        map: $("map").value.trim(),
        pcd: $("pcd").value.trim(),
        map_prefix: baseConfig.map_prefix || "maps/h1_2_lab",
        pcd_prefix: baseConfig.pcd_prefix || "maps/h1_2_lab",
        localization: $("localization").value,
        initial_pose: $("initial_pose").value.trim() || "START",
        start_gesture: startGesture,
        start_turn: startTurn,
        nav2_params_file: $("nav2_params_file").value.trim(),
        auto_start_route: $("auto_start_route").checked,
        return_to_start: $("return_to_start").checked,
        teaching_path_file: $("teaching_path_file").value.trim(),
        teaching_events_file: $("teaching_events_file").value.trim(),
        teaching_loop: $("teaching_loop").checked,
        memory_route: selected.join(","),
        gesture_route: gestures.join(","),
        turn_route: steps.filter(step => step.turn).map(step => `${step.point}:${step.turn}`).join(","),
        voice_remote_host: $("voice_remote_host").value.trim(),
        voice_workspace: $("voice_workspace").value.trim() || "~/ros2_ws",
        voice_ros_setup: $("voice_ros_setup").value.trim() || "/opt/ros/humble/setup.bash",
        camera_preview_enabled: false
      };
    }

    async function api(path, options = {}) {
      const response = await fetch(path, options);
      if (!response.ok) throw new Error(`请求失败 (${response.status})`);
      return response.json();
    }

    async function loadConfig() {
      const config = await api("/api/config");
      baseConfig = config;
      await connectHeartbeat(config.heartbeat_ws_port);
      taskProfiles = structuredClone(config.task_profiles);
      selectedTaskId = config.selected_task_id;
      revision = savedRevision = 0;
      document.body.classList.toggle("ui-only", config.ui_only === true);
      const pc4Profile = config.pc4_profile === true;
      $("native_rviz_button").hidden = false;
      $("native_rviz_open_button").hidden = false;
      $("native_rviz_button").textContent = pc4Profile ? "打开本机 RViz" : "打开原生 RViz";
      $("native_rviz_open_button").textContent = pc4Profile ? "打开本机 RViz" : "打开原生 RViz";
      $("native_rviz_close_button").hidden = pc4Profile;
      $("native_rviz_restart_button").hidden = pc4Profile;
      $("voice_start_button").hidden = pc4Profile;
      $("voice_stop_button").hidden = pc4Profile;
      $("rviz_mode").disabled = false;
      $("run_mode").value = config.run_mode || "online";
      $("motion_backend").value = config.motion_backend || "unitree";
      $("enable_remote_velocity").checked = config.enable_remote_velocity !== false;
      $("confirm_debug_mode").checked = false;
      $("confirm_exclusive_control").checked = false;
      $("confirm_policy_writer").checked = false;
      $("confirm_policy_support").checked = false;
      $("confirm_policy_upright").checked = false;
      $("rviz_mode").value = config.rviz_mode || "auto";
      $("map").value = config.map || "";
      $("pcd").value = config.pcd || "";
      $("localization").value = config.localization || "global_gicp";
      $("initial_pose").value = config.initial_pose || "START";
      startGesture = config.start_gesture || "";
      startTurn = config.start_turn || "";
      $("nav2_params_file").value = config.nav2_params_file || "";
      $("voice_remote_host").value = config.voice_remote_host || "";
      $("voice_workspace").value = config.voice_workspace || "~/ros2_ws";
      $("voice_ros_setup").value = config.voice_ros_setup || "/opt/ros/humble/setup.bash";
      $("teaching_path_file").value = config.teaching_path_file || "";
      $("teaching_events_file").value = config.teaching_events_file || "";
      $("teaching_loop").checked = config.teaching_loop !== false;
      $("auto_start_route").checked = config.auto_start_route === true;
      $("return_to_start").checked = !!config.return_to_start;
      renderTasks();
      onModeChanged();
    }

    let heartbeatSocket;
    let heartbeatReconnectTimer;
    let heartbeatReady = false;
    function connectHeartbeat(port, waitForReady = true) {
      return new Promise((resolve, reject) => {
        const socket = new WebSocket(`${location.protocol === "https:" ? "wss:" : "ws:"}//${location.hostname}:${port}`);
        heartbeatSocket = socket;
        let settled = false;
        const startupTimer = waitForReady ? setTimeout(() => {
          if (!settled) { settled = true; socket.close(); reject(new Error("浏览器心跳未就绪，页面保持未启动。")); }
        }, 4000) : null;
        socket.addEventListener("message", event => {
          // The server emits ready only after this connection's first Ping/Pong.
          try { if (JSON.parse(event.data).type === "ready" && !settled) {
            settled = true; heartbeatReady = true; if (startupTimer) clearTimeout(startupTimer); resolve();
          } } catch (_) {}
        });
        socket.addEventListener("error", () => {
          if (!settled) { settled = true; if (startupTimer) clearTimeout(startupTimer); reject(new Error("无法连接浏览器 WebSocket 心跳服务。")); }
        });
        socket.addEventListener("close", () => {
          heartbeatReady = false;
          if (!settled) { settled = true; if (startupTimer) clearTimeout(startupTimer); reject(new Error("浏览器 WebSocket 心跳服务已关闭。")); }
          clearTimeout(heartbeatReconnectTimer);
          heartbeatReconnectTimer = setTimeout(() => connectHeartbeat(port, false).catch(() => {}), 500);
        });
      });
    }
    function ensureHeartbeatReady() {
      if (!heartbeatReady || !heartbeatSocket || heartbeatSocket.readyState !== WebSocket.OPEN) {
        throw new Error("浏览器心跳未连接，导航操作未发送。请等待心跳重连后重试。");
      }
    }

    function onModeChanged() {
      const mode = $("run_mode").value;
      document.body.dataset.mode = mode;
      const backend = $("motion_backend");
      backend.disabled = mode !== "online" && mode !== "navigation";
      if (backend.disabled) backend.value = "unitree";
    }

    function currentTask() { return taskProfiles.find(task => task.id === selectedTaskId); }
    function showError(error) { showResult({success: false, message: error.message || String(error)}); }
    async function preserveScroll(action) {
      const scrollables = [document.documentElement, document.body, ...document.querySelectorAll(".left-rail, .center-stage, .right-rail, .advanced-config, #logs")];
      const positions = scrollables.map(element => ({element, left: element.scrollLeft, top: element.scrollTop}));
      const x = window.scrollX, y = window.scrollY;
      try { return await action(); }
      finally {
        const restore = () => {
          window.scrollTo(x, y);
          for (const position of positions) {
            if (position.element.isConnected) {
              position.element.scrollLeft = position.left;
              position.element.scrollTop = position.top;
            }
          }
        };
        restore();
        requestAnimationFrame(restore);
      }
    }
    function showResult(result) {
      $("task_feedback").textContent = result.message || (result.success ? "操作成功" : "操作失败");
      $("task_feedback").className = result.success ? "" : "error";
    }
    function markDirty() { revision++; updateSaveState(); updateTaskReadiness(); }
    function updateSaveState() {
      $("save_state").textContent = saving ? "保存中…" : revision === savedRevision ? "已保存" : "有未保存更改";
      $("save_tasks").disabled = saving;
    }
    function renderTasks() {
      $("task_select").replaceChildren(...taskProfiles.map(task => new Option(task.name, task.id)));
      $("task_select").value = selectedTaskId;
      $("task_name").value = currentTask().name;
      $("delete_task").disabled = taskProfiles.length <= 1;
      updateSaveState();
      renderRoute();
    }
    function switchTask(id) {
      if (!taskProfiles.some(task => task.id === id)) return;
      selectedTaskId = id;
      points = [];
      markDirty();
      renderTasks();
      loadPoints().catch(showError);
    }
    function uniqueTaskName(base) {
      let name = base, i = 2;
      while (taskProfiles.some(task => task.name === name)) name = `${base} ${i++}`;
      return name;
    }
    function openNewTaskModal() {
      if (taskProfiles.length >= 100) { showError(new Error("最多可保存 100 个任务。")); return; }
      $("new_task_error").textContent = "";
      $("new_task_name").value = uniqueTaskName("新任务");
      $("new_task_modal").classList.add("open");
      requestAnimationFrame(() => { $("new_task_name").focus(); $("new_task_name").select(); });
    }
    function closeNewTaskModal(event) {
      if (event && event.target !== $("new_task_modal")) return;
      $("new_task_modal").classList.remove("open");
    }
    function confirmNewTask() {
      const name = $("new_task_name").value.trim();
      if (!name || name.length > 80) { showNewTaskError("任务名不能为空或超过 80 个字符。"); return; }
      const existing = taskProfiles.find(task => task.name === name);
      if (existing) {
        if (!confirm(`任务“${name}”已存在，是否覆盖？`)) return;
      }
      if (createTask(false, name, existing ? existing.id : "")) closeNewTaskModal();
    }
    function showNewTaskError(message) {
      $("new_task_error").textContent = message;
      showError(new Error(message));
    }
    function newTaskId() {
      // getRandomValues also works on the robot's plain HTTP LAN page.
      const bytes = crypto.getRandomValues(new Uint8Array(16));
      bytes[6] = (bytes[6] & 0x0f) | 0x40;
      bytes[8] = (bytes[8] & 0x3f) | 0x80;
      const hex = Array.from(bytes, value => value.toString(16).padStart(2, "0")).join("");
      return `${hex.slice(0, 8)}-${hex.slice(8, 12)}-${hex.slice(12, 16)}-${hex.slice(16, 20)}-${hex.slice(20)}`;
    }
    function createTask(copy = false, taskName = "", replaceTaskId = "") {
      if (taskProfiles.length >= 100 && !replaceTaskId) { showNewTaskError("最多可保存 100 个任务。"); return false; }
      let task;
      try {
        task = {id: newTaskId(), name: taskName || uniqueTaskName(copy ? `${currentTask().name.slice(0, 65)} 副本` : "新任务"),
          steps: copy ? structuredClone(currentTask().steps) : [], memory_points_file: ""};
      } catch (error) {
        showNewTaskError(`创建任务失败：${error.message || error}`);
        return false;
      }
      if (replaceTaskId) taskProfiles = taskProfiles.filter(item => item.id !== replaceTaskId);
      taskProfiles.push(task);
      selectedTaskId = task.id;
      points = [];
      markDirty(); renderTasks();
      loadPoints().catch(showError);
      $("task_name").focus(); $("task_name").select();
      showResult({success: true, message: "任务已创建，请保存更改。"});
      return true;
    }
    function renameTask() {
      const name = $("task_name").value.trim();
      if (!name || name.length > 80 || taskProfiles.some(task => task.id !== selectedTaskId && task.name === name)) {
        showError(new Error("任务名不能为空、重复或超过 80 个字符。")); return;
      }
      currentTask().name = name;
      markDirty(); renderTasks(); showResult({success:true, message:"已重命名，请保存更改。"});
    }
    $("new_task_name").addEventListener("keydown", event => {
      if (event.key === "Enter") { event.preventDefault(); confirmNewTask(); }
      if (event.key === "Escape") closeNewTaskModal();
    });
    async function deleteTask() {
      if (taskProfiles.length <= 1 || !confirm(`删除任务“${currentTask().name}”？其点位和动作编排也会删除。`)) return;
      const deleted = await api("/api/task/delete", {
        method: "POST",
        body: JSON.stringify({task_id: selectedTaskId, task_name: currentTask().name, memory_points_file: currentTask().memory_points_file || ""}),
      });
      if (!deleted.success) { showResult(deleted); return; }
      taskProfiles = taskProfiles.filter(task => task.id !== selectedTaskId);
      selectedTaskId = taskProfiles[0].id;
      points = [];
      markDirty(); renderTasks();
      loadPoints().catch(showError);
      await saveConfig();
      showResult({success:true, message:"任务及其点位文件已删除。"});
    }
    let pointsRequestSeq = 0;
    async function loadPoints() {
      const taskId = selectedTaskId;
      const requestSeq = ++pointsRequestSeq;
      points = [];
      renderRoute();
      let data;
      try {
        data = await api(`/api/points?task_id=${encodeURIComponent(taskId)}&task_name=${encodeURIComponent(currentTask()?.name || "")}`);
      } catch (error) {
        if (requestSeq === pointsRequestSeq && taskId === selectedTaskId) throw error;
        return;
      }
      if (requestSeq !== pointsRequestSeq || taskId !== selectedTaskId) return;
      points = data.points || [];
      renderRoute();
    }
    async function saveCurrentPoint() {
      const name = $("point_name").value.trim();
      if (!name) { showError(new Error("请输入点位名称。")); return; }
      try {
        let overwrite = points.some(point => point.name === name);
        if (overwrite && !confirm(`点位“${name}”已存在，是否用机器人当前位置覆盖？`)) return;
        let result = await api("/api/points/save", {method:"POST", body:JSON.stringify({task_id:selectedTaskId, task_name:currentTask()?.name || "", name, overwrite})});
        if (!result.success && /already exists|已存在/i.test(result.message || "") && confirm(`点位“${name}”已存在，是否覆盖？`)) {
          result = await api("/api/points/save", {method:"POST", body:JSON.stringify({task_id:selectedTaskId, task_name:currentTask()?.name || "", name, overwrite:true})});
        }
        showResult(result);
        if (result.success) { $("point_name").value = ""; await loadPoints(); }
      } catch (error) { showError(error); }
    }
    function renderRoute() {
      const task = currentTask();
      if (!task) return;
      const library = new Map(points.map(point => [point.name, point]));
      const selected = new Set(task.steps.map(step => step.point));
      const ordered = [...task.steps.map(step => library.get(step.point) || {name: step.point}), ...points.filter(point => !selected.has(point.name))];
      $("routeBody").replaceChildren();
      for (const point of ordered) {
        const index = task.steps.findIndex(step => step.point === point.name);
        const step = task.steps[index];
        const tr = document.createElement("tr");
        tr.className = step ? "selected" : "";
        const cells = Array.from({length:6}, () => tr.appendChild(document.createElement("td")));
        const check = document.createElement("input"); check.type = "checkbox";
        check.checked = !!step; check.disabled = point.name === "START";
        check.setAttribute("aria-label", `启用 ${point.name}`);
        check.onchange = () => setUse(point.name, check.checked); cells[0].append(check);
        const name = document.createElement("span"); name.className = "point"; name.textContent = point.name;
        cells[1].append(name);
        const action = document.createElement("select"); action.setAttribute("aria-label", `${point.name} 到达后动作`);
        const options = [["", "无动作"], ["wave_hand", "挥手"], ["shake_hand", "出手"]];
        if (step?.gesture && !options.some(([value]) => value === step.gesture)) options.push([step.gesture, step.gesture]);
        action.replaceChildren(...options.map(([value,label]) => new Option(label, value)));
        action.value = point.name === "START" ? startGesture : (step?.gesture || "");
        action.disabled = false;
        action.title = point.name === "START" ? "配置初始位姿动作" : "选择动作后会自动启用该点位";
        action.onchange = () => setGesture(point.name, action.value); cells[2].append(action);
        const turn = document.createElement("select"); turn.setAttribute("aria-label", `${point.name} 到点转向`);
        turn.replaceChildren(...[["", "不转向"], ["cw", "顺时针"], ["ccw", "逆时针"]].map(([value,label]) => new Option(label,value)));
        turn.value = point.name === "START" ? startTurn : (step?.turn || "");
        turn.disabled = false;
        turn.title = point.name === "START" ? "配置初始位姿转向" : "选择转向后会自动启用该点位";
        turn.onchange = () => setTurn(point.name, turn.value); cells[3].append(turn);
        if (step) {
          for (const [offset, label] of [[-1,"上移"],[1,"下移"]]) {
            const button = document.createElement("button"); button.className = "btn"; button.textContent = label;
            button.setAttribute("aria-label", `${label} ${point.name}`);
            button.disabled = index + offset < 0 || index + offset >= task.steps.length;
            button.onclick = () => movePoint(point.name, offset); cells[4].append(button);
          }
        }
        cells[5].className = "pos";
        if (point.position) {
          const pose = document.createElement("div"); pose.className = "pose-values";
          const position = document.createElement("span");
          position.textContent = `X ${Number(point.position.x || 0).toFixed(3)}   Y ${Number(point.position.y || 0).toFixed(3)}   Z ${Number(point.position.z || 0).toFixed(3)}`;
          const orientation = point.orientation || {};
          const quaternion = document.createElement("span");
          quaternion.textContent = `qx ${Number(orientation.x || 0).toFixed(4)}   qy ${Number(orientation.y || 0).toFixed(4)}   qz ${Number(orientation.z || 0).toFixed(4)}   qw ${Number(orientation.w ?? 1).toFixed(4)}`;
          pose.append(position, quaternion); cells[5].append(pose);
        } else {
          cells[5].textContent = "点位不存在，请取消或刷新";
        }
        if (!point.position) cells[5].classList.add("missing");
        $("routeBody").append(tr);
      }
      $("task_count").textContent = `已选 ${task.steps.length} 个点位`;
      updateTaskReadiness();
    }
    function setUse(name, use) {
      if (name === "START") return;
      const task = currentTask();
      if (use && !task.steps.some(step => step.point === name)) task.steps.push({point:name, gesture:"", turn:""});
      if (!use) task.steps = task.steps.filter(step => step.point !== name);
      markDirty(); renderRoute();
    }
    function setGesture(name, gesture) {
      if (name === "START") {
        startGesture = gesture;
        markDirty();
        return;
      }
      const task = currentTask();
      let step = task.steps.find(step => step.point === name);
      if (!step && gesture) {
        step = {point: name, gesture: "", turn: ""};
        task.steps.push(step);
      }
      if (!step) return;
      step.gesture = gesture;
      markDirty();
      renderRoute();
    }
    function setTurn(name, turn) {
      if (name === "START") {
        startTurn = turn;
        markDirty();
        return;
      }
      const task = currentTask();
      let step = task.steps.find(step => step.point === name);
      if (!step && turn) {
        step = {point: name, gesture: "", turn: ""};
        task.steps.push(step);
      }
      if (!step) return;
      step.turn = turn;
      markDirty();
      renderRoute();
    }
    function movePoint(name, offset) {
      const steps = currentTask().steps, index = steps.findIndex(step => step.point === name);
      if (index < 0 || index + offset < 0 || index + offset >= steps.length) return;
      [steps[index], steps[index + offset]] = [steps[index + offset], steps[index]];
      markDirty(); renderRoute();
    }
    function selectAll(enabled) {
      const task = currentTask();
      if (!enabled) task.steps = [];
      else for (const point of points) {
        if (point.name !== "START" && !task.steps.some(step => step.point === point.name)) task.steps.push({point:point.name, gesture:"", turn:""});
      }
      markDirty(); renderRoute();
    }
    function taskMatchesLoaded() {
      const loaded = latestStatus.loaded_task, task = currentTask();
      const navigation = latestStatus.processes?.navigation;
      if (!task || !navigation?.running) return false;
      const config = payload();
      if (!loaded && navigation.external) {
        return config.run_mode === navigation.run_mode &&
          config.motion_backend === navigation.motion_backend &&
          (navigation.motion_backend !== "policy" ||
            (config.confirm_debug_mode && config.confirm_exclusive_control));
      }
      if (!loaded) return false;
      return loaded.id === task.id && loaded.memory_route === config.memory_route && loaded.gesture_route === config.gesture_route && loaded.turn_route === config.turn_route;
    }
    function canStartTask() {
      const navigation = latestStatus.processes?.navigation || {};
      const policyReady = navigation.motion_backend !== "policy" || latestStatus.policy_control?.ready === true;
      return connected && !baseConfig.ui_only && latestStatus.localization?.ready && policyReady && taskMatchesLoaded();
    }
    function updateTaskReadiness() {
      const loaded = latestStatus.loaded_task;
      $("start_route_button").disabled = !canStartTask();
      const navigation = latestStatus.processes?.navigation || {};
      const hint = !connected ? "等待连接状态…" : baseConfig.ui_only ? "界面预览模式 · 未连接 ROS" :
        navigation.motion_backend === "policy" && !latestStatus.policy_control?.ready ? (latestStatus.policy_control?.message || "等待 ONNX Policy 就绪") :
        !loaded && navigation.external && taskMatchesLoaded() ? "已连接当前终端导航，可将所选任务提交到此实例。" :
        !loaded || !navigation.running ? "尚未加载任务。选择导航模式，点击“启动当前模式”加载当前任务。" :
        taskMatchesLoaded() ? `已加载：${loaded.name} · ${latestStatus.localization?.ready ? "定位就绪，可开始巡航" : "等待定位稳定"}` :
        `已加载：${loaded.name}；当前编排尚未加载。请停止全部，再启动当前模式以加载所选任务。`;
      $("loaded_task_hint").textContent = hint;
      $("start_route_button").title = hint;
      if (latestStatus.localization?.ready || !connected) {
        $("cruise_ready_card").className = "status-metric " + (canStartTask() ? "ready" : "waiting");
        $("cruise_ready_card").title = hint;
        $("cruise_ready_text").textContent = !connected ? "● 连接未就绪" : canStartTask() ? "● 可开始巡航" : "● 任务未加载";
      }
    }
    async function saveConfig() {
      if (saving) return;
      saving = true; const version = revision; const config = payload(); updateSaveState();
      try {
        const result = await api("/api/config", {method:"POST", body:JSON.stringify(config)});
        if (result.success) { savedRevision = version; baseConfig = config; }
        showResult({...result, message: result.success ? "任务与设置已保存。" : result.message});
      } catch (error) { showError(error); }
      finally { saving = false; updateSaveState(); }
    }
    async function startNavigation() {
      if ($("motion_backend").value === "policy") { startPolicyWizard(); return; }
      if (starting) return;
      starting = true; const version = revision; const config = payload();
      try { await preserveScroll(async () => {
        ensureHeartbeatReady();
        const result = await api("/api/navigation/start", {method:"POST", body:JSON.stringify(config)});
        if (result.success) { savedRevision = version; baseConfig = config; updateSaveState(); }
        showResult(result); appendLog("web", result.message); await refreshStatus();
      }); } catch (error) { showError(error); }
      finally { starting = false; }
    }

    function startPolicyWizard() {
      const navigation = latestStatus.processes?.navigation || {};
      const policyAlreadyRunning = latestStatus.processes?.policy_bridge?.running ||
        (navigation.running && navigation.motion_backend === "policy");
      $("motion_backend").value = "policy";
      if (!policyAlreadyRunning) {
        for (const id of ["confirm_debug_mode", "confirm_exclusive_control", "confirm_policy_writer", "confirm_policy_support", "confirm_policy_upright"]) $(id).checked = false;
        markDirty();
      }
      $("policy_wizard_modal").classList.add("open");
      updatePolicyControls();
      refreshStatus();
    }

    function closePolicyWizard(event) {
      if (event && event.target !== $("policy_wizard_modal")) return;
      $("policy_wizard_modal").classList.remove("open");
    }

    async function launchPolicyNavigation() {
      if (starting) return;
      starting = true;
      const config = payload();
      config.motion_backend = "policy";
      config.auto_start_route = false;
      config.confirm_lowcmd_handoff = $("confirm_policy_writer").checked;
      config.confirm_robot_supported = $("confirm_policy_support").checked;
      $("auto_start_route").checked = false;
      const version = revision;
      updatePolicyControls();
      try {
        ensureHeartbeatReady();
        const result = await api("/api/policy/start", {method:"POST", body:JSON.stringify(config)});
        if (result.success) { savedRevision = version; baseConfig = config; updateSaveState(); }
        appendLog("web", result.message || "ONNX 控制桥启动请求已发送");
        $("policy_wizard_status").textContent = result.success
          ? "ONNX 控制桥启动请求已接受；正在等待状态。导航栈和巡航路线均未启动。"
          : (result.message || "ONNX 控制桥启动失败。");
        await refreshStatus();
      } catch (error) {
        showError(error);
        $("policy_wizard_status").textContent = error.message || "启动请求失败。";
      } finally {
        starting = false;
        updatePolicyControls();
      }
    }

    async function policyWizardNext() {
      const bridge = latestStatus.processes?.policy_bridge || {};
      if (!bridge.running) {
        const confirmed = $("confirm_debug_mode").checked && $("confirm_exclusive_control").checked &&
          $("confirm_policy_writer").checked && $("confirm_policy_support").checked;
        if (!confirmed) {
          $("policy_wizard_status").textContent = "请先逐项确认 Debug、独占控制、LowCmd writer 交接、支撑和急停状态。";
          return;
        }
        await launchPolicyNavigation();
        return;
      }
      const policy = latestStatus.policy_control || {};
      if (!policy.available) {
        $("policy_wizard_status").textContent = policy.message || "正在等待 Policy bridge 状态服务。";
        await refreshStatus();
        return;
      }
      if (policy.mode === "Passive" && !policy.authorized) return policyControlStep("authorize_output");
      if (policy.mode === "Passive" && policy.authorized) return policyControlStep("fix_stand");
      if (policy.mode === "FixStand") return policyControlStep("arm_hold_stand");
      if (policy.mode === "ArmHoldStand") return policyControlStep("arm_hold");
      if (policy.control_ready && !latestStatus.processes?.navigation?.running) {
        return startPolicyNavigationStack();
      }
    }

    async function startPolicyNavigationStack() {
      if (starting) return;
      const policy = latestStatus.policy_control || {};
      if (policy.control_ready !== true) {
        $("policy_wizard_status").textContent = "请先完成 ONNX ArmHold 控制准备。";
        return;
      }
      starting = true;
      const config = payload();
      config.motion_backend = "policy";
      config.auto_start_route = false;
      $("auto_start_route").checked = false;
      updatePolicyControls();
      try {
        ensureHeartbeatReady();
        const result = await api("/api/navigation/start", {method:"POST", body:JSON.stringify(config)});
        appendLog("nav", result.message || "导航栈启动请求已发送");
        $("policy_wizard_status").textContent = result.success
          ? "导航栈启动请求已接受。等待 Nav2 与定位就绪后，再单独开始巡航。"
          : (result.message || "导航栈启动失败。");
        await refreshStatus();
      } catch (error) {
        showError(error);
        $("policy_wizard_status").textContent = error.message || "导航栈启动请求失败。";
      } finally {
        starting = false;
        updatePolicyControls();
      }
    }

    async function saveSnapshot(mode) {
      if (starting) return;
      starting = true;
      const config = payload();
      config.run_mode = mode;
      try { await preserveScroll(async () => {
        ensureHeartbeatReady();
        const result = await api("/api/navigation/start", {method:"POST", body:JSON.stringify(config)});
        showResult(result);
        appendLog("web", result.message || (mode === "save_map" ? "2D 地图保存请求已发送" : "3D 点云保存请求已发送"));
        await refreshStatus();
      }); } catch (error) { showError(error); }
      finally { starting = false; }
    }

    async function postAction(path, includePayload = false) {
      await preserveScroll(async () => {
        const result = await api(path, { method: "POST", body: JSON.stringify(includePayload ? payload() : {}) });
        appendLog("web", result.message || JSON.stringify(result));
        await refreshStatus();
      });
    }

    async function stopAll() {
      await preserveScroll(async () => {
        const result = await api("/api/navigation/stop_all", { method: "POST", body: JSON.stringify({}) });
        appendLog("web", result.message || JSON.stringify(result));
        await refreshStatus();
        if (result.success) await closeNativeRviz();
      });
    }

    async function closeNativeRviz() {
      if (baseConfig.pc4_profile === true) {
        if (/Windows/i.test(navigator.userAgent)) {
          showError(new Error("本机 RViz 启动器目前配置在 Linux 上位机，请在该上位机打开此页面。"));
          return;
        }
        appendLog("rviz", "正在关闭由网页启动的本机 RViz。");
        window.location.href = "h1-rviz://close";
        return;
      }
      await postAction("/api/rviz/close");
    }

    async function openNativeRviz() {
      if (baseConfig.pc4_profile === true) {
        if (/Windows/i.test(navigator.userAgent)) {
          showError(new Error("本机 RViz 启动器目前配置在 Linux 上位机，请在该上位机打开此页面。"));
          return;
        }
        const selectedMode = $("rviz_mode").value;
        const mode = selectedMode === "offline" ||
          (selectedMode === "auto" && $("run_mode").value === "navigation") ? "offline" : "online";
        const backend = $("motion_backend").value;
        appendLog("rviz", "正在请求打开本机 RViz；首次使用请允许浏览器打开 H1 Local RViz 应用。");
        window.location.href = `h1-rviz://open?mode=${mode}&backend=${backend}`;
        return;
      }
      await postAction("/api/rviz/open", true);
    }

    async function startRoute() {
      if (!canStartTask()) { showError(new Error($("loaded_task_hint").textContent)); return; }
      await preserveScroll(async () => {
        const result = await api("/api/route/start", { method: "POST", body: JSON.stringify(payload()) });
        showResult(result);
        appendLog("route", result.message || JSON.stringify(result));
        await refreshStatus();
      });
    }

    async function startTeachingRoute() {
      ensureHeartbeatReady();
      const result = await api("/api/teaching_route/start", { method: "POST", body: JSON.stringify(payload()) });
      appendLog("teaching-route", result.message || JSON.stringify(result));
      refreshStatus();
    }

    async function gesture(name) {
      const result = await api("/api/gesture", { method: "POST", body: JSON.stringify({ name }) });
      appendLog("gesture", formatGestureResult(name, result));
    }

    function formatGestureResult(name, result) {
      const sdkRet = Object.prototype.hasOwnProperty.call(result, "sdk_ret") ? ` sdk_ret=${result.sdk_ret}` : "";
      return `${name}: ${result.success ? "ok" : "failed"} ${result.message || ""}${sdkRet}`;
    }

    async function toggleShakeHand() {
      const name = shakeHandExtended ? "retract_hand" : "reach_hand";
      const button = $("shake_hand_button");
      button.disabled = true;
      try {
        const result = await api("/api/gesture", {method:"POST", body:JSON.stringify({name})});
        appendLog("gesture", formatGestureResult(name, result));
        if (result.success) {
          shakeHandExtended = !shakeHandExtended;
          button.textContent = shakeHandExtended ? "收手" : "出手";
        }
        await refreshStatus();
      } catch (error) { showError(error); }
      finally { button.disabled = false; }
    }

    function formatBatteryMetric(value, unit) {
      if (value == null || value === "") return "--";
      const numeric = Number(value);
      return Number.isFinite(numeric) ? `${numeric.toFixed(1)}${unit}` : String(value);
    }

    async function refreshStatus() {
      if (statusRefreshInFlight) return;
      statusRefreshInFlight = true;
      let data;
      try { data = await api("/api/status"); connected = true; }
      catch (error) { connected = false; updateTaskReadiness(); showError(error); statusRefreshInFlight = false; return; }
      latestStatus = data;
      updatePolicyControls();
      const route = data.route || {};
      const teachingRoute = data.teaching_route || {};
      $("state").textContent = route.state || "-";
      $("state").className = "pill " + (route.success ? "ok" : "warn");
      $("goal").textContent = `${route.current_goal || "-"} / index ${route.current_index || 0}`;
      $("message").textContent = route.message || "-";
      $("teaching_state").textContent = teachingRoute.state || "-";
      $("teaching_state").className = "pill " + (teachingRoute.success ? "ok" : "warn");
      $("teaching_goal").textContent = `${teachingRoute.current_goal || "-"} / segment ${teachingRoute.current_index || 0}`;
      $("teaching_message").textContent = teachingRoute.message || "-";
      const localization = data.localization || {};
      $("localization_state").textContent = localization.message || "定位未稳定";
      const indicatorState = localization.ready
        ? "ready"
        : localization.score == null
          ? "waiting"
          : "checking";
      const cruiseReadyCard = $("cruise_ready_card");
      cruiseReadyCard.className = "status-metric " + (localization.ready ? "ready" : "waiting");
      cruiseReadyCard.title = localization.message || "定位未稳定";
      $("cruise_ready_text").textContent = canStartTask()
        ? "● 可开始巡航"
        : data.processes?.navigation?.motion_backend === "policy" && !data.policy_control?.ready
          ? "● ONNX 未就绪"
        : indicatorState === "checking"
          ? "● 定位确认中"
          : "● 不可开始巡航（任务或定位未就绪）";
      const startButton = $("start_route_button");
      startButton.disabled = !localization.ready || !canStartTask();
      startButton.title = localization.message || "定位未稳定";
      const startTeachingButton = $("start_teaching_route_button");
      startTeachingButton.disabled = !localization.ready;
      startTeachingButton.title = localization.message || "定位未稳定";
      const processes = data.processes || {};
      const motionState = processes.teaching_route?.running
        ? `示教巡航 ${teachingRoute.state || "running"}`
        : processes.navigation?.running
          ? `导航 ${route.state || "running"}`
          : route.state && route.state !== "-"
            ? route.state
            : "待机";
      $("motion_state_card").className =
        "status-metric motion " + ((processes.teaching_route?.running || processes.navigation?.running) ? "active" : "");
      $("motion_state").textContent = motionState;
      const battery = data.battery || {};
      const batteryPercent = Number.isFinite(Number(battery.percent))
        ? Math.max(0, Math.min(100, Math.round(Number(battery.percent))))
        : 0;
      $("battery_ring").style.setProperty("--percent", `${batteryPercent}%`);
      $("battery_percent").textContent = `${batteryPercent}%`;
      $("battery_state").textContent = battery.state || (data.battery ? "在线" : "未接入");
      $("battery_state").className = "pill " + (data.battery ? "ok" : "warn");
      $("battery_voltage").textContent = formatBatteryMetric(battery.voltage, "V");
      $("battery_temperature").textContent = formatBatteryMetric(battery.temperature, "°C");
      $("battery_charge_state").textContent = battery.charge_state || "--";
      $("battery_charge_state").className = "runtime-dot " + (battery.charge_state ? "ok" : "");
      $("battery_cycles").textContent = battery.cycles == null ? "--" : String(battery.cycles);
      $("processes").textContent = `nav=${processes.navigation?.running ? "running" : "idle"}, rviz=${processes.rviz?.running ? "running" : "idle"}, teaching_record=${processes.teaching_record?.running ? "running" : "idle"}, teaching=${processes.teaching_route?.running ? "running" : "idle"}, voice_reader=${processes.voice?.running ? "running" : "idle"}, voice_bridge=${processes.voice_bridge?.running ? "running" : "idle"}, vision=${processes.vision?.running ? "running" : "idle"}`;
      updateTaskReadiness();
      statusRefreshInFlight = false;
    }

    function updatePolicyControls() {
      const navigation = latestStatus.processes?.navigation || {};
      const bridge = latestStatus.processes?.policy_bridge || {};
      const policy = latestStatus.policy_control || {};
      const isPolicy = bridge.running || (navigation.running && navigation.motion_backend === "policy");
      const controlReady = policy.control_ready === true;
      const statusText = !isPolicy ? (navigation.running
        ? "当前运行的不是 ONNX Policy 导航。打开 ONNX 向导可查看当前模式并重新启动。"
        : "尚未启动 ONNX。选择后端不会启动控制；请点“启动 ONNX 模式”并按向导逐步操作。") : !policy.available
        ? (policy.message || "Policy bridge 状态不可用。")
        : policy.output_enabled === false ? "Policy bridge 启动时未启用硬件输出。"
        : policy.debug_confirmed === false || policy.exclusive_confirmed === false ? "Policy bridge 启动确认参数不完整。"
        : `${policy.mode || "未知"} · ${policy.authorized ? "输出已授权" : "尚未授权输出"} · LowCmd ${policy.publisher_present ? "已发布" : "未发布"} · ${policy.state_valid ? "机器人状态有效" : "机器人状态无效"}` +
          (policy.remote_velocity_enabled ? ` · 遥控器共存：${policy.remote_active ? "遥控器输入" : "导航/回中"}` : "");
      $("policy_mode_badge").textContent = !isPolicy ? "未启动" : !policy.available ? "等待桥接" : policy.mode || "未知";
      $("policy_mode_badge").className = "pill " + (controlReady ? "ok" : "warn");
      $("policy_control_status").textContent = statusText;
      $("motion_backend_hint").hidden = $("motion_backend").value !== "policy";
      const routeState = latestStatus.route?.state || "idle";
      const routeRunning = !["idle", "unavailable", "error", "ui-only", ""].includes(routeState);
      const canCancelStartup = isPolicy && policy.available &&
        policy.mode === "Passive" && policy.authorized === false && policy.publisher_present === false &&
        policy.output_transition === "none" && policy.state_valid === true &&
        (!navigation.running || (latestStatus.route?.success === true && latestStatus.route?.state === "idle"));
      const overviewRows = {
        stack: [bridge.running ? "已启动" : navigation.running ? "其他后端运行中" : "未启动", bridge.running, !bridge.running && !navigation.running],
        control: [controlReady ? "ArmHold 就绪" : isPolicy ? (policy.mode || "等待桥接") : "未就绪", controlReady, isPolicy && !controlReady],
        route: [navigation.running && navigation.motion_backend === "policy" ? "已启动" : "未启动", navigation.running && navigation.motion_backend === "policy", controlReady && !navigation.running],
        cruise: [routeRunning ? "执行中" : "未开始", routeRunning, navigation.running && controlReady && !routeRunning],
      };
      for (const [key, [label, done, active]] of Object.entries(overviewRows)) {
        const row = $("policy_overview_" + key), state = $("policy_overview_" + key + "_state");
        state.textContent = label;
        row.classList.toggle("done", done);
        row.classList.toggle("active", active);
      }
      $("start_policy_mode_button").textContent = isPolicy ? "ONNX 控制状态" : "启动 ONNX 模式";
      $("policy_cancel_startup").disabled = starting || !canCancelStartup;
      const confirmed = $("confirm_debug_mode").checked && $("confirm_exclusive_control").checked &&
        $("confirm_policy_writer").checked && $("confirm_policy_support").checked;
      const canStep = isPolicy && policy.available && policy.profile === "hardware" &&
        policy.joint_command_mode === "position_pd" && policy.output_enabled !== false && policy.debug_confirmed !== false &&
        policy.exclusive_confirmed !== false && policy.navigation_enabled && policy.continuous_navigation &&
        policy.state_valid && policy.output_transition === "none" && confirmed;
      const activeStep = !isPolicy ? "start" : !policy.available ? "waiting" :
        !policy.authorized && policy.mode === "Passive" ? "authorize" :
        policy.mode === "Passive" ? "fix" :
        policy.mode === "FixStand" ? "hold_stand" :
        policy.mode === "ArmHoldStand" ? "arm_hold" :
        policy.mode === "ArmHold" ? (navigation.running ? "navigation" : "complete") : "waiting";
      const phaseNames = {start:"待启动", waiting:"等待状态", authorize:"待授权", fix:"FixStand", hold_stand:"ArmHoldStand", arm_hold:"进入 ArmHold", complete:"控制已就绪", navigation:"导航已启动"};
      const phase = $("policy_wizard_phase");
      phase.textContent = phaseNames[activeStep];
      phase.className = "pill " + (activeStep === "complete" ? "ok" : "warn");
      const status = $("policy_wizard_status");
      status.classList.toggle("ready", activeStep === "complete" || activeStep === "navigation");
      const detail = $("policy_wizard_detail");
      const navigationMode = navigation.run_mode === "navigation" ? "离线地图导航" : "在线建图导航";
      const statusDetails = {
        start: `当前设置：${navigationMode}。启动 ONNX 只会启动控制桥；完成 ArmHold 后再启动导航栈。`,
        waiting: "正在等待可验证的 Policy bridge 状态。状态服务不可用、LowState 无效或输出转场中时，不会执行下一步。",
        authorize: "下一步会开放 Policy 对 LowCmd 的发布。再次确认现场控制权已交接、支撑有效且急停可用。",
        fix: "输出已授权且 LowCmd publisher 已出现。下一步会请求 FixStand，请现场持续观察机器人。",
        hold_stand: "FixStand 已完成。下一步会请求 ArmHoldStand；若姿态或支撑不符合预期，请停止并按现场流程处理。",
        arm_hold: "ArmHoldStand 已完成。先在现场确认机器人姿态符合测试方案，再勾选下方确认并进入 ArmHold。",
        complete: "ONNX 已进入 ArmHold 控制状态。下一步单独启动导航栈；导航和定位就绪后再点击“开始巡航”。",
        navigation: "导航栈已启动。等待定位就绪后，在主页面单独点击“开始巡航”。"
      };
      status.textContent = isPolicy
        ? !policy.available ? (policy.message || statusDetails.waiting)
          : policy.mode === "ArmHold" && controlReady ? statusDetails[activeStep]
            : !policy.state_valid ? "机器人状态无效，暂停准备流程并检查 LowState。"
              : policy.output_transition !== "none" ? "Policy 输出状态正在转换，请等待状态稳定后再继续。"
                : statusDetails[activeStep]
        : navigation.running ? "检测到其他导航后端正在运行。请先停止当前导航，再启动 ONNX 模式。"
          : statusDetails.start;
      detail.textContent = activeStep === "start" && !isPolicy
        ? "后端选项本身不启动任何进程；确认下方现场条件后，点击“启动 ONNX 模式”。"
        : statusText;
      const preflightIds = ["confirm_debug_mode", "confirm_exclusive_control", "confirm_policy_writer", "confirm_policy_support"];
      for (const id of preflightIds) $(id).disabled = false;
      const setStep = (id, stateText, done, active) => {
        const row = $(id), state = $(id + "_state");
        row.classList.toggle("done", done);
        if (active) row.setAttribute("aria-current", "step"); else row.removeAttribute("aria-current");
        state.textContent = stateText;
      };
      const mode = policy.mode;
      const authorized = policy.authorized === true;
      setStep("policy_step_start", bridge.running ? "已启动" : navigation.running ? "当前后端冲突" : "等待启动", bridge.running, activeStep === "start");
      setStep("policy_step_authorize", authorized ? "已授权" : isPolicy && policy.available ? "待确认" : "等待启动", authorized, activeStep === "authorize");
      setStep("policy_step_fix", ["FixStand", "ArmHoldStand", "ArmHold"].includes(mode) ? "已完成" : "待执行", ["FixStand", "ArmHoldStand", "ArmHold"].includes(mode), activeStep === "fix");
      setStep("policy_step_hold_stand", ["ArmHoldStand", "ArmHold"].includes(mode) ? "已完成" : "待执行", ["ArmHoldStand", "ArmHold"].includes(mode), activeStep === "hold_stand");
      setStep("policy_step_arm_hold", mode === "ArmHold" && controlReady ? "已就绪" : "待确认", mode === "ArmHold" && controlReady, activeStep === "arm_hold" || activeStep === "complete" || activeStep === "navigation");
      setStep("policy_step_navigation", navigation.running && navigation.motion_backend === "policy" ? "已启动" : "等待 ArmHold", navigation.running && navigation.motion_backend === "policy", activeStep === "complete" || activeStep === "navigation");
      $("confirm_upright_label").hidden = activeStep !== "arm_hold";
      const next = $("policy_wizard_next");
      const labels = {
        start: "启动 ONNX 模式", waiting: "等待 Policy 状态", authorize: "确认并授权 LowCmd 输出",
        fix: "确认并执行 FixStand", hold_stand: "确认并执行 ArmHoldStand",
        arm_hold: "确认站姿并进入 ArmHold", complete: "启动导航栈", navigation: "导航栈已启动"
      };
      next.textContent = starting ? "请求处理中…" : labels[activeStep];
      next.className = "btn " + (["start", "authorize"].includes(activeStep) ? "danger" : "primary");
      next.disabled = starting || activeStep === "waiting" || activeStep === "navigation" ||
        (activeStep === "start" && (!confirmed || navigation.running)) ||
        (activeStep === "complete" && !controlReady) ||
        (activeStep !== "start" && !canStep) ||
        (activeStep === "arm_hold" && !$("confirm_policy_upright").checked) ||
        (activeStep === "authorize" && (policy.authorized || policy.publisher_present)) ||
        (activeStep === "fix" && !policy.publisher_present) ||
        (activeStep === "hold_stand" && !policy.publisher_present);
    }

    async function policyControlStep(step) {
      if (starting) return;
      starting = true;
      updatePolicyControls();
      try {
        const request = {
          step,
          confirm_debug_mode: $("confirm_debug_mode").checked,
          confirm_exclusive_control: $("confirm_exclusive_control").checked,
          confirm_lowcmd_handoff: $("confirm_policy_writer").checked,
          confirm_robot_supported: $("confirm_policy_support").checked,
          confirm_upright: $("confirm_policy_upright").checked
        };
        const result = await api("/api/policy/control", {method:"POST", body:JSON.stringify(request)});
        showResult(result);
        appendLog("policy", result.message || step);
        await refreshStatus();
      } catch (error) { showError(error); }
      finally { starting = false; updatePolicyControls(); }
    }

    async function retryPolicyStatus() {
      if (starting) return;
      $("policy_retry_status").disabled = true;
      $("policy_wizard_status").textContent = "正在重新查询 Policy 与路线服务…";
      try { await refreshStatus(); }
      finally { $("policy_retry_status").disabled = false; updatePolicyControls(); }
    }

    async function cancelPolicyStartup() {
      if (starting || $("policy_cancel_startup").disabled) return;
      if (!window.confirm("确认取消这次尚未授权的 ONNX 启动？仅当状态为 Passive、未授权、无 LowCmd publisher 且路线 idle 时可执行。")) return;
      starting = true;
      updatePolicyControls();
      try {
        ensureHeartbeatReady();
        const result = await api("/api/policy/cancel_startup", {method:"POST", body:JSON.stringify({})});
        showResult(result);
        $("policy_wizard_status").textContent = result.message || "取消请求已处理。";
        appendLog("policy", result.message || "取消未授权启动");
        await refreshStatus();
      } catch (error) { showError(error); }
      finally { starting = false; updatePolicyControls(); }
    }

    function appendLog(source, message) {
      const logs = $("logs");
      const shouldFollow = shouldAutoFollowLogs();
      appendLogText(logs, `[${new Date().toLocaleTimeString()}] [${source}] ${message}\n`);
      followLogsIfAllowed(shouldFollow);
    }

    function appendLogText(logs, text) {
      let content = logs.textContent + text;
      if (content.length > MAX_LOG_TEXT_LENGTH) {
        const overflow = content.length - MAX_LOG_TEXT_LENGTH;
        const nextLine = content.indexOf("\n", overflow);
        content = nextLine >= 0 ? content.slice(nextLine + 1) : content.slice(-MAX_LOG_TEXT_LENGTH);
      }
      logs.textContent = content;
    }

    function logsNearBottom() {
      const logs = $("logs");
      return logs.scrollHeight - logs.scrollTop - logs.clientHeight < 24;
    }

    function logsHasSelection() {
      const selection = window.getSelection();
      return !!selection && !selection.isCollapsed && $("logs").contains(selection.anchorNode);
    }

    function shouldAutoFollowLogs() {
      return logsAutoFollow && !logsSelecting && !logsHasSelection() && logsNearBottom();
    }

    function followLogsIfAllowed(shouldFollow) {
      if (!shouldFollow) return;
      const logs = $("logs");
      logs.scrollTop = logs.scrollHeight;
    }

    async function pollLogs() {
      if (logsPollInFlight) return;
      logsPollInFlight = true;
      try {
        const shouldFollow = shouldAutoFollowLogs();
        const data = await api(`/api/logs?after=${lastLogSeq}`);
        const lines = [];
        for (const entry of data.entries || []) {
          if (entry.seq <= lastLogSeq) continue;
          lastLogSeq = entry.seq;
          if (entry.source === "http") continue;
          lines.push(`[${entry.time}] [${entry.source}] ${entry.message}\n`);
        }
        if (lines.length) appendLogText($("logs"), lines.join(""));
        followLogsIfAllowed(shouldFollow);
      } finally {
        logsPollInFlight = false;
      }
    }

    function setupLogScrollLock() {
      const logs = $("logs");
      logs.addEventListener("scroll", () => {
        logsAutoFollow = logsNearBottom();
      });
      logs.addEventListener("mousedown", () => {
        logsSelecting = true;
        logsAutoFollow = false;
      });
      document.addEventListener("mouseup", () => {
        logsSelecting = false;
      });
      document.addEventListener("selectionchange", () => {
        if (logsHasSelection()) logsAutoFollow = false;
      });
    }

    (async function boot() {
      await loadConfig();
      await loadPoints();
      await refreshStatus();
      await pollLogs();
      setupLogScrollLock();
      setInterval(refreshStatus, 1200);
      setInterval(() => pollLogs().catch(() => {}), 700);
      document.querySelectorAll(".left-rail input, .left-rail select").forEach(el => el.addEventListener("change", markDirty));
      ["confirm_debug_mode", "confirm_exclusive_control", "confirm_policy_writer", "confirm_policy_support", "confirm_policy_upright"]
        .forEach(id => $(id).addEventListener("change", updatePolicyControls));
    })().catch(showError);
    window.addEventListener("beforeunload", event => {
      if (revision !== savedRevision) { event.preventDefault(); event.returnValue = ""; }
    });
    window.addEventListener("unhandledrejection", event => { showError(event.reason); event.preventDefault(); });
  </script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(description="H1-2 offline navigation local web GUI")
    default_host = "0.0.0.0" if os.environ.get("H1_2_RUNTIME_PROFILE", "").strip().lower() == "pc4" else "127.0.0.1"
    parser.add_argument("--host", default=default_host)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--port-tries", type=int, default=20, help="number of sequential ports to try")
    parser.add_argument("--no-single-instance", action="store_true", help="do not stop older Web GUI instances")
    parser.add_argument(
        "--ui-only",
        action="store_true",
        help="serve the UI without initializing ROS or allowing process controls",
    )
    args = parser.parse_args()
    if args.port_tries < 1:
        parser.error("--port-tries must be >= 1")
    if not args.ui_only and not ROS_CUSTOM_MESSAGES_AVAILABLE:
        parser.error("缺少 ROS 自定义消息包，请先 source 工作区 install；仅预览界面可使用 --ui-only。")

    try:
        from h1_2_nav_tools.heartbeat_transport import HeartbeatWebSocketServer
    except ImportError as exc:
        print(f"WebSocket heartbeat dependency unavailable; refusing to start Web GUI: {exc}", file=sys.stderr, flush=True)
        return 1

    workspace_root = find_workspace_root()
    instance_lock = None
    if not args.no_single_instance:
        lock_path = Path.home() / ".ros" / "h1_2_nav_web_gui.lock"
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        instance_lock = lock_path.open("w")
        try:
            fcntl.flock(instance_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("another h1_2_nav_web_gui instance is already running")
        instance_lock.write(str(os.getpid()))
        instance_lock.flush()
    server, actual_port = bind_http_server(args.host, args.port, RequestHandler, args.port_tries)
    app = WebGuiApp(workspace_root, ui_only=args.ui_only)
    app.http_port = actual_port
    app.heartbeat_ws_port = actual_port + 1
    server.app = app
    heartbeat_server = HeartbeatWebSocketServer(
        args.host, app.heartbeat_ws_port, actual_port, app,
        logger=lambda message: app.logs.add("web", message),
    )

    stopping = threading.Event()

    def stop(_signum=None, _frame=None):
        if stopping.is_set():
            return
        stopping.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    url = f"http://{args.host}:{actual_port}"
    print(f"H1-2 web GUI: {url}", flush=True)
    print(f"H1-2 browser heartbeat WebSocket: {args.host}:{app.heartbeat_ws_port}", flush=True)
    print("Press Ctrl-C to stop the web GUI server.", flush=True)
    try:
        heartbeat_server.start()
        server.serve_forever()
    finally:
        heartbeat_server.shutdown()
        app.shutdown()
        server.server_close()
        if instance_lock is not None:
            fcntl.flock(instance_lock.fileno(), fcntl.LOCK_UN)
            instance_lock.close()


if __name__ == "__main__":
    sys.exit(main())
