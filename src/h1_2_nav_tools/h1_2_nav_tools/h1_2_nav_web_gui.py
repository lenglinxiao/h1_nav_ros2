import argparse
import errno
import fcntl
import json
import os
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
GESTURE_OPTIONS = ["", "wave_hand", "shake_hand"]
HEARTBEAT_TIMEOUT_SEC = max(0.5, float(os.environ.get("H1_2_WEB_HEARTBEAT_TIMEOUT_SEC", "2.0")))
# Let the independent stop script deliver StopMove before this Web process
# interrupts its own navigation process group.
STOP_COMMAND_GRACE_SEC = 0.5


def configure_ros_environment():
    os.environ["ROS_LOCALHOST_ONLY"] = "0"
    os.environ.setdefault("ROS_DOMAIN_ID", "0")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ.pop("ROS_HOSTNAME", None)
    os.environ.pop("ROS_IP", None)
    workspace_root = find_workspace_root()
    configured_dds = os.environ.get("H1_2_DDS_CONFIG", "").strip()
    dds_config = Path(configured_dds).expanduser() if configured_dds else workspace_root / "config" / "cyclonedds_auto.xml"
    if not dds_config.is_file():
        raise RuntimeError(f"Cyclone DDS config does not exist: {dds_config}")
    os.environ["CYCLONEDDS_URI"] = f"file://{dds_config}"
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
        self.node = Node("h1_2_nav_web_gui")
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

    def route_start(self):
        localization = self.localization_status()
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

    def localization_status(self):
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
        self.rviz_process = None
        self.voice_process = None
        self.voice_bridge_process = None
        self.vision_process = None
        self.teaching_route_process = None
        self.teaching_record_process = None
        self.lock = threading.Lock()
        self._closed = False

    def status(self):
        return {
            "navigation": self._process_state(self.nav_process),
            "rviz": self._process_state(self.rviz_process),
            "voice": self._process_state(self.voice_process),
            "voice_bridge": self._process_state(self.voice_bridge_process),
            "vision": self._process_state(self.vision_process),
            "teaching_route": self._process_state(self.teaching_route_process),
            "teaching_record": self._process_state(self.teaching_record_process),
        }

    def start_navigation(self, args):
        with self.lock:
            script = self.workspace_root / "run_ai_slam_nav.sh"
            if not script.is_file():
                return {"success": False, "message": f"missing script: {script}"}
            if args and args[0] in ("mode=save_map", "mode=save_pcd_map"):
                label = "map-save" if args[0] == "mode=save_map" else "pcd-save"
                self._start(label, [str(script), *args])
                return {"success": True, "message": f"{label} process started"}
            if self._is_running(self.nav_process):
                return {"success": False, "message": "navigation is already running"}
            self.nav_process = self._start("nav", [str(script), *args])
            return {"success": True, "message": "navigation process started"}

    def stop_all(self, payload=None):
        script = self.workspace_root / "stop_ai_slam_nav.sh"
        if not script.is_file():
            return {"success": False, "message": f"missing script: {script}"}
        self._start("stop", [str(script)])
        time.sleep(STOP_COMMAND_GRACE_SEC)
        if self._is_running(self.nav_process):
            self._terminate_process(self.nav_process)
        self.stop_voice_control(payload or {})
        self.stop_teaching_record_runner()
        self.stop_teaching_route_runner()
        return {"success": True, "message": "stop script started"}

    def stop_navigation(self, reason="requested"):
        with self.lock:
            self.logs.add("web", f"stopping navigation: {reason}")
            script = self.workspace_root / "stop_ai_slam_nav.sh"
            stopped = False
            navigation_running = self._is_running(self.nav_process)
            if navigation_running and script.is_file():
                try:
                    self._start("stop", [str(script)])
                    time.sleep(STOP_COMMAND_GRACE_SEC)
                    stopped = True
                except OSError as exc:
                    self.logs.add("web", f"failed to run navigation stop script: {exc}")
            if navigation_running:
                self._terminate_process(self.nav_process)
                stopped = True
            return {"success": stopped, "message": "navigation stop requested" if stopped else "navigation is not running"}

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
            nav_running = self._is_running(self.nav_process)
            if nav_running:
                script = self.workspace_root / "stop_ai_slam_nav.sh"
                if script.is_file():
                    try:
                        self._start("stop", [str(script)])
                        time.sleep(STOP_COMMAND_GRACE_SEC)
                    except OSError as exc:
                        self.logs.add("web", f"failed to run navigation stop script: {exc}")
            if nav_running:
                self.logs.add("navigation", "terminating on server shutdown")
                self._terminate_process(self.nav_process)
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

    def _start(self, label, command):
        self.logs.add(label, "$ " + " ".join(command))
        process = subprocess.Popen(
            command,
            cwd=str(self.workspace_root),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            start_new_session=True,
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
            "map": str(self.workspace_root / "maps" / "h1_2_lab.yaml"),
            "pcd": str(self.workspace_root / "maps" / "h1_2_lab.pcd"),
            "run_mode": "online",
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
        config["memory_route"] = normalize_memory_route(
            config.get("memory_route", "A,B,C")
        )
        if not str(config.get("nav2_params_file", "")).strip():
            config["nav2_params_file"] = str(
                self.workspace_root / "src" / "h1_2_nav_tools" / "config" / "nav2_h1_2_rpp.yaml"
            )
        config["camera_preview_enabled"] = False
        config["workflow_version"] = CONFIG_WORKFLOW_VERSION
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
            result = self.processes.start_navigation(self.navigation_args(payload))
            if result.get("success") and payload.get("run_mode") in ("online", "navigation"):
                self.heartbeat_expired = False
                self.loaded_task = task_execution_snapshot(payload) if payload.get("run_mode") == "navigation" else None
                if payload.get("run_mode") not in ("save_map", "save_pcd_map"):
                    self.save_config(payload)
            return result

    def start_selected_task(self, payload):
        with self.task_lock:
            selected = task_execution_snapshot(payload)
            if self.loaded_task is None or not self.processes.status()["navigation"]["running"]:
                return {"success": False, "message": "请先启动导航，载入当前任务。"}
            keys = ("id", "memory_route", "gesture_route", "turn_route")
            if any(selected[key] != self.loaded_task[key] for key in keys):
                return {"success": False, "message": "当前任务或编排与已加载的任务不同。请停止全部后重新启动导航，载入所选任务。"}
            return self.ros.route_start()

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
            points = data.setdefault("points", {})
            pose = result.get("pose")
            if not pose:
                return {"success": False, "message": "ROS 未返回当前位姿。"}
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
        map_path = Path(str(payload.get("map", ""))).expanduser()
        pcd_path = Path(str(payload.get("pcd", ""))).expanduser()
        memory_route = normalize_memory_route(payload.get("memory_route", ""))
        if mode not in ("online", "navigation", "save_map", "save_pcd_map"):
            return False, f"Unsupported mode: {mode}"
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
        selected = next(item for item in payload["task_profiles"] if item["id"] == payload["selected_task_id"])
        task_args = [
            f"memory_route={normalize_memory_route(payload.get('memory_route', ''))}",
            f"gesture_route={payload.get('gesture_route', '')}",
            f"turn_route={payload.get('turn_route', '')}",
            f"memory_points_file={self._task_points_file(getattr(self, 'workspace_root', Path('.')), selected)}",
            f"auto_start_route={str(bool(payload.get('auto_start_route', False))).lower()}",
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
        elif parsed.path == "/api/points":
            task_id = parse_qs(parsed.query).get("task_id", [None])[0]
            task_name = parse_qs(parsed.query).get("task_name", [""])[0]
            self._send_json({"points": self.app.memory_points_for_task(task_id, task_name), "gestures": GESTURE_OPTIONS})
        elif parsed.path == "/api/status":
            route = self.app.ros.route_status()
            teaching_route = self.app.ros.teaching_route_status()
            self._send_json(
                {
                    "route": route,
                    "loaded_task": self.app.loaded_task if self.app.processes.status()["navigation"]["running"] else None,
                    "teaching_route": teaching_route,
                    "localization": self.app.ros.localization_status(),
                    "processes": self.app.processes.status(),
                    "camera": self.app.ros.camera_status(),
                    "vision": self.app.ros.vision_status(),
                    "runtime_profile": self.app.runtime_profile,
                    "pc4_profile": self.app.pc4_profile,
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
            self.app.record_heartbeat()
            self._send_json({"success": True})
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
        elif parsed.path == "/api/navigation/stop_all":
            result = self.app.processes.stop_all(payload)
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
            self._send_json(self.app.ros.route_stop())
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
      grid-template-rows: 108px minmax(0, 1fr) 290px;
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
    .left-rail, .center-stage, .right-rail { min-height: 0; overflow: auto; }
    .left-rail > .panel, .center-stage > .panel { min-height: 100%; }
    .center-stage { display: block; }
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
    .control-grid .btn:hover { background: #dbeafe; border-color: #93c5fd; }
    .control-grid .btn:disabled { background: #f3f7fd; border-color: #d7e3f4; color: #7890b4; }
    .setup-grid { display: grid; grid-template-columns: 1fr; gap: 10px; }
    .setup-grid > div { min-width: 0; }
    .left-rail .two { grid-template-columns: 1fr; }
    .advanced-config { max-height: none; overflow: visible; }
    .route-panel { min-height: 0; overflow: auto; }
    .route-panel .body { min-height: 0; overflow: auto; }
    .route-intro, #task_count { display: none; }
    .task-toolbar { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
    .task-toolbar .route-intro { margin: 0; margin-right: auto; }
    .task-name { display: flex; gap: 8px; }
    .task-name input { min-width: 0; }
    .task-name .btn { flex-shrink: 0; white-space: nowrap; }
    #task_feedback:empty { display: none; }
    #task_feedback { color: #166534; font-size: 13px; overflow-wrap: anywhere; }
    #task_feedback.error { color: #b42318; }
    .modal-backdrop { position: fixed; inset: 0; z-index: 20; display: none; place-items: center; padding: 20px; background: rgba(15, 23, 42, .28); }
    .modal-backdrop.open { display: grid; }
    .modal { width: min(420px, 100%); background: #fff; border: 1px solid var(--line); border-radius: 8px; box-shadow: 0 18px 50px rgba(15,23,42,.18); padding: 20px; }
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
      .app { height: auto; min-height: 100vh; grid-template-rows: auto auto 260px; }
      main, footer, .setup-grid { grid-template-columns: 1fr; }
      header { height: auto; padding: 12px 14px; align-items: flex-start; flex-direction: column; }
      .actions { justify-content: flex-start; }
      .left-rail, .center-stage, .right-rail { overflow: visible; }
      .advanced-config { max-height: none; }
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
        <button class="btn danger" onclick="postAction('/api/navigation/stop_all')">停止全部</button>
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
            <button id="native_rviz_close_button" class="btn primary" onclick="postAction('/api/rviz/close')">关闭 RViz</button>
            <button id="native_rviz_restart_button" class="btn primary" onclick="postAction('/api/rviz/restart', true)">重启 RViz</button>
            <button class="btn primary" onclick="refreshStatus()">刷新状态</button>
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
    let revision = 0;
    let savedRevision = 0;
    let saving = false;
    let starting = false;
    let lastLogSeq = 0;
    let logsAutoFollow = true;
    let logsSelecting = false;
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

    function onModeChanged() {
      const mode = $("run_mode").value;
      document.body.dataset.mode = mode;
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
      if (!loaded || !task || !latestStatus.processes?.navigation?.running) return false;
      const config = payload();
      return loaded.id === task.id && loaded.memory_route === config.memory_route && loaded.gesture_route === config.gesture_route && loaded.turn_route === config.turn_route;
    }
    function canStartTask() {
      return connected && !baseConfig.ui_only && latestStatus.localization?.ready && taskMatchesLoaded();
    }
    function updateTaskReadiness() {
      const loaded = latestStatus.loaded_task;
      $("start_route_button").disabled = !canStartTask();
      const hint = !connected ? "等待连接状态…" : baseConfig.ui_only ? "界面预览模式 · 未连接 ROS" :
        !loaded || !latestStatus.processes?.navigation?.running ? "尚未加载任务。选择离线地图导航，点击“启动当前模式”加载当前任务。" :
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
      if (starting) return;
      starting = true; const version = revision; const config = payload();
      try { await preserveScroll(async () => {
        await api("/api/heartbeat", {method:"POST", body:"{}"});
        const result = await api("/api/navigation/start", {method:"POST", body:JSON.stringify(config)});
        if (result.success) { savedRevision = version; baseConfig = config; updateSaveState(); }
        showResult(result); appendLog("web", result.message); await refreshStatus();
      }); } catch (error) { showError(error); }
      finally { starting = false; }
    }

    async function saveSnapshot(mode) {
      if (starting) return;
      starting = true;
      const config = payload();
      config.run_mode = mode;
      try { await preserveScroll(async () => {
        await api("/api/heartbeat", {method:"POST", body:"{}"});
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

    async function openNativeRviz() {
      if (baseConfig.pc4_profile === true) {
        if (/Windows/i.test(navigator.userAgent)) {
          showError(new Error("本机 RViz 启动器目前配置在 Linux 上位机，请在该上位机打开此页面。"));
          return;
        }
        const selectedMode = $("rviz_mode").value;
        const mode = selectedMode === "offline" ||
          (selectedMode === "auto" && $("run_mode").value === "navigation") ? "offline" : "online";
        appendLog("rviz", "正在请求打开本机 RViz；首次使用请允许浏览器打开 H1 Local RViz 应用。");
        window.location.href = `h1-rviz://open?mode=${mode}`;
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
      let data;
      try { data = await api("/api/status"); connected = true; }
      catch (error) { connected = false; updateTaskReadiness(); showError(error); return; }
      latestStatus = data;
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
      $("cruise_ready_text").textContent = localization.ready
        ? "● 可开始巡航"
        : indicatorState === "checking"
          ? "● 定位确认中"
          : "● 不可开始巡航";
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
    }

    function appendLog(source, message) {
      const logs = $("logs");
      const shouldFollow = shouldAutoFollowLogs();
      logs.textContent += `[${new Date().toLocaleTimeString()}] [${source}] ${message}\n`;
      followLogsIfAllowed(shouldFollow);
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
      const shouldFollow = shouldAutoFollowLogs();
      const data = await api(`/api/logs?after=${lastLogSeq}`);
      for (const entry of data.entries || []) {
        lastLogSeq = Math.max(lastLogSeq, entry.seq);
        if (entry.source === "http") continue;
        $("logs").textContent += `[${entry.time}] [${entry.source}] ${entry.message}\n`;
      }
      followLogsIfAllowed(shouldFollow);
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
      setInterval(() => api("/api/heartbeat", {method:"POST", body:"{}"}).catch(() => {}), 500);
      setInterval(() => pollLogs().catch(() => {}), 700);
      document.querySelectorAll(".left-rail input, .left-rail select").forEach(el => el.addEventListener("change", markDirty));
    })().catch(showError);
    window.addEventListener("beforeunload", event => {
      if (navigator.sendBeacon) navigator.sendBeacon("/api/heartbeat", new Blob(["{}"], {type:"application/json"}));
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
    server.app = app

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
    print("Press Ctrl-C to stop the web GUI server.", flush=True)
    try:
        server.serve_forever()
    finally:
        app.shutdown()
        server.server_close()
        if instance_lock is not None:
            fcntl.flock(instance_lock.fileno(), fcntl.LOCK_UN)
            instance_lock.close()


if __name__ == "__main__":
    sys.exit(main())
