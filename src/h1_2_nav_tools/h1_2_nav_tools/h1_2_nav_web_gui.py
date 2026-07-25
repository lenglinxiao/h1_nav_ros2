import argparse
import errno
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
from h1_2_icp_localization.msg import LocalizationStatus
from h1_2_nav_msgs.srv import ExecuteGesture, RouteStatus, StartRoute, StopRoute
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


def configure_ros_environment():
    os.environ["ROS_LOCALHOST_ONLY"] = "0"
    os.environ.setdefault("ROS_DOMAIN_ID", "0")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ.pop("ROS_HOSTNAME", None)
    os.environ.pop("ROS_IP", None)
    dds_config = find_workspace_root() / "config" / "cyclonedds_enp45s0.xml"
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
    candidates.extend([cwd, *cwd.parents])
    candidates.extend([Path.home() / "h1_nav2_ros2_ws", Path("/home/llx/h1_nav2_ros2_ws")])
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
        if name not in ("wave_hand", "shake_hand"):
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

    def _call_simple(self, label, client, request):
        if not client.wait_for_service(timeout_sec=0.2):
            return {"success": False, "message": f"{label} is not available"}
        ok, result = self._wait_future(client.call_async(request))
        if not ok:
            return {"success": False, "message": str(result)}
        return {"success": bool(result.success), "message": result.message}

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
            if self._is_running(self.nav_process):
                return {"success": False, "message": "navigation is already running"}
            script = self.workspace_root / "run_ai_slam_nav.sh"
            if not script.is_file():
                return {"success": False, "message": f"missing script: {script}"}
            self.nav_process = self._start("nav", [str(script), *args])
            return {"success": True, "message": "navigation process started"}

    def stop_all(self, payload=None):
        script = self.workspace_root / "stop_ai_slam_nav.sh"
        if not script.is_file():
            return {"success": False, "message": f"missing script: {script}"}
        self.stop_voice_control(payload or {})
        self.stop_teaching_record_runner()
        self.stop_teaching_route_runner()
        self._start("stop", [str(script)])
        return {"success": True, "message": "stop script started"}

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
                    process.terminate()

    def _start(self, label, command):
        self.logs.add(label, "$ " + " ".join(command))
        process = subprocess.Popen(
            command,
            cwd=str(self.workspace_root),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
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
    def __init__(self, workspace_root, ui_only=False):
        self.workspace_root = workspace_root
        self.ui_only = bool(ui_only)
        self.points_file = workspace_root / "maps" / "memory_points.json"
        self.logs = LogStore()
        self.processes = ProcessManager(workspace_root, self.logs)
        bridge_type = UiOnlyRosBridge if self.ui_only else RosServiceBridge
        self.ros = bridge_type(self.logs, self.default_config())
        self.logs.add("web", f"workspace={workspace_root}")
        if self.ui_only:
            self.logs.add("web", "UI-only mode enabled; ROS and process controls are disabled")

    def shutdown(self):
        self.processes.shutdown()
        self.ros.shutdown()

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
        config["voice_remote_host"] = normalize_ssh_target(config.get("voice_remote_host", ""))
        config["voice_workspace"] = normalize_voice_workspace(config.get("voice_workspace", "~/ros2_ws"))
        config["vision_remote_host"] = normalize_ssh_target(config.get("vision_remote_host", ""))
        config["vision_workspace"] = normalize_voice_workspace(
            config.get("vision_workspace", "~/ros2_ws")
        )
        config["ui_only"] = self.ui_only
        return config

    def save_config(self, config):
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
        CONFIG_PATH.write_text(json.dumps(config, indent=2, sort_keys=True), encoding="utf-8")

    def reconnect_camera(self, config):
        config["camera_preview_enabled"] = True
        self.save_config(config)
        self.ros.configure_camera(config)
        return {"success": True, "message": "相机话题订阅已重新连接"}

    def start_camera_preview(self, config):
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
        try:
            data = json.loads(self.points_file.read_text(encoding="utf-8"))
            points = data.get("points", {})
            if not isinstance(points, dict):
                raise ValueError("points object is invalid")
        except Exception as exc:
            self.logs.add("web", f"failed to load memory points: {exc}")
            points = {}
        rows = []
        for name in sorted(points.keys(), key=point_sort_key):
            position = points[name].get("position", {})
            rows.append(
                {
                    "name": name,
                    "position": {
                        "x": float(position.get("x", 0.0)),
                        "y": float(position.get("y", 0.0)),
                        "z": float(position.get("z", 0.0)),
                    },
                }
            )
        return rows

    def validate_navigation(self, payload):
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
        if mode == "save_map" and not str(payload.get("map_prefix", "")).strip():
            return False, "Save map mode requires map_prefix."
        if mode == "save_pcd_map" and not str(payload.get("pcd_prefix", "")).strip():
            return False, "Save PCD map mode requires pcd_prefix."
        return True, ""

    def navigation_args(self, payload):
        mode = str(payload.get("run_mode", "online"))
        args = [f"mode={mode}"]
        if mode == "online":
            args.append(f"pcd_prefix={payload.get('pcd_prefix', 'maps/h1_2_lab')}")
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
            f"memory_route={normalize_memory_route(payload.get('memory_route', ''))}",
            f"gesture_route={payload.get('gesture_route', '')}",
            f"auto_start_route={str(bool(payload.get('auto_start_route', False))).lower()}",
            f"return_to_start={str(bool(payload.get('return_to_start', False))).lower()}",
            ]
        )
        params_file = str(payload.get("nav2_params_file", "")).strip()
        if params_file:
            params_path = Path(params_file).expanduser()
            args.append(f"params_file={params_path}")
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
            self._send_json({"points": self.app.memory_points(), "gestures": GESTURE_OPTIONS})
        elif parsed.path == "/api/status":
            route = self.app.ros.route_status()
            teaching_route = self.app.ros.teaching_route_status()
            self._send_json(
                {
                    "route": route,
                    "teaching_route": teaching_route,
                    "localization": self.app.ros.localization_status(),
                    "processes": self.app.processes.status(),
                    "camera": self.app.ros.camera_status(),
                    "vision": self.app.ros.vision_status(),
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
        if self.app.ui_only and parsed.path != "/api/config":
            self.app.logs.add("ui-only", f"blocked control request: {parsed.path}")
            self._send_json(
                {
                    "success": False,
                    "message": "UI-only 模式：未连接 ROS，控制操作未执行",
                }
            )
            return
        if parsed.path == "/api/config":
            self.app.save_config(payload)
            self._send_json({"success": True, "message": "config saved"})
        elif parsed.path == "/api/navigation/start":
            ok, message = self.app.validate_navigation(payload)
            if not ok:
                self._send_json({"success": False, "message": message})
                return
            self.app.save_config(payload)
            self._send_json(self.app.processes.start_navigation(self.app.navigation_args(payload)))
        elif parsed.path == "/api/navigation/stop_all":
            self.app.ros.teaching_route_stop()
            self._send_json(self.app.processes.stop_all(payload))
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
            self._send_json(
                self.app.processes.open_rviz(
                    str(payload.get("run_mode", "online")),
                    str(payload.get("rviz_mode", "auto")),
                )
            )
        elif parsed.path == "/api/rviz/close":
            self._send_json(self.app.processes.close_rviz())
        elif parsed.path == "/api/rviz/restart":
            self._send_json(
                self.app.processes.restart_rviz(
                    str(payload.get("run_mode", "online")),
                    str(payload.get("rviz_mode", "auto")),
                )
            )
        elif parsed.path == "/api/route/start":
            self._send_json(self.app.ros.route_start())
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
  <title>H1-2 Offline Navigation</title>
  <style>
    :root {
      --bg: #070b0f;
      --panel: #0d151a;
      --panel-2: #101d24;
      --panel-3: #13252e;
      --text: #e5f3f6;
      --muted: #82a3ad;
      --line: #1f343c;
      --accent: #22d3ee;
      --accent-2: #34d399;
      --danger: #f87171;
      --warn: #fbbf24;
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
      min-height: 100vh;
      display: grid;
      grid-template-rows: 108px minmax(0, 1fr) 190px;
    }
    header {
      min-height: 108px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 0 16px;
      border-bottom: 1px solid var(--line);
      background: #091016;
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
      background: #132229;
      color: var(--text);
      font-weight: 750;
      cursor: pointer;
    }
    .btn:hover { background: #18313a; border-color: #2c5663; }
    .btn.primary { background: #0891b2; border-color: #22d3ee; color: #ecfeff; }
    .btn.soft { background: #0f2c2a; border-color: #176a62; color: #86efac; }
    .btn.danger { background: #3b1217; border-color: #7f1d1d; color: #fecaca; }
    .btn:disabled { opacity: .55; cursor: not-allowed; }
    .ui-only-badge {
      display: none;
      padding: 7px 10px;
      border: 1px solid #a16207;
      border-radius: 999px;
      background: #422006;
      color: #fde68a;
      font-size: 12px;
      font-weight: 850;
    }
    body.ui-only .ui-only-badge { display: inline-flex; }
    main {
      display: grid;
      grid-template-columns: 300px minmax(540px, 1fr) 340px;
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
      background: #081116;
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
    th { color: var(--muted); background: #0a1217; font-size: 12px; font-weight: 850; }
    th:first-child, td:first-child { width: 44px; text-align: center; }
    th:nth-child(2), td:nth-child(2) { width: 72px; }
    th:nth-child(3), td:nth-child(3) { width: 118px; }
    tr:last-child td { border-bottom: 0; }
    .point {
      display: inline-flex; min-width: 32px; height: 24px;
      align-items: center; justify-content: center;
      background: #073344; color: #67e8f9;
      border: 1px solid #155e75;
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
      background: #15252d; color: #bad6dc; font-weight: 850; font-size: 12px;
    }
    .pill.ok { background: #063f32; color: #86efac; }
    .pill.warn { background: #3d2a08; color: #fcd34d; }
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
      background: #111c23;
      border-color: #314751;
      color: #9bb3bb;
    }
    .localization-indicator.checking {
      background: #312308;
      border-color: #b45309;
      color: #fbbf24;
    }
    .localization-indicator.ready {
      background: #073b2d;
      border-color: #22c55e;
      color: #bbf7d0;
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
      background: #081116;
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
      background: #091218;
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
      background: #063f32;
      border-color: #22c55e;
      box-shadow: 0 0 0 3px rgba(34, 197, 94, 0.14);
    }
    .status-metric.ready .metric-value { color: #bbf7d0; }
    .status-metric.waiting {
      background: #312308;
      border-color: #b45309;
    }
    .status-metric.waiting .metric-value { color: #fbbf24; }
    .status-metric.motion {
      background: #0b171d;
      border-color: #274955;
    }
    .status-metric.motion.active {
      background: #073344;
      border-color: #0891b2;
      box-shadow: 0 0 0 3px rgba(8, 145, 178, 0.12);
    }
    .status-metric.motion.active .metric-value { color: #67e8f9; }
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
      background: conic-gradient(#22c55e var(--percent), #20323a 0);
      box-shadow: inset 0 0 0 1px rgba(255,255,255,.06);
      position: relative;
    }
    .runtime-ring::after {
      content: "";
      position: absolute;
      inset: 8px;
      border-radius: 50%;
      background: #091218;
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
      background: #15252d;
      color: #bad6dc;
      font-size: 12px;
      font-weight: 900;
    }
    .runtime-dot.ok { background: #063f32; color: #86efac; }
    .runtime-dot.warn { background: #3d2a08; color: #fcd34d; }
    .runtime-hidden { display: none; }
    .left-rail, .center-stage, .right-rail { min-height: 0; overflow: auto; }
    .center-stage { display: grid; grid-template-rows: minmax(300px, 46vh) auto minmax(0, 1fr); gap: 10px; overflow: hidden; }
    .center-controls .body { padding: 10px; }
    .control-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(128px, 1fr));
      gap: 8px;
    }
    .control-grid .btn {
      width: 100%;
      min-height: 32px;
      padding: 6px 10px;
    }
    .setup-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .advanced-config { max-height: 230px; overflow: auto; }
    .route-panel { min-height: 0; overflow: auto; }
    .camera-section { margin: 0; }
    .camera-config {
      display: grid;
      grid-template-columns: 1fr;
      gap: 10px;
      align-items: end;
      padding: 12px;
      border-bottom: 1px solid var(--line);
    }
    .camera-grid {
      display: grid;
      grid-template-columns: 1fr;
      gap: 10px;
      padding: 12px;
    }
    .vision-config {
      display: grid;
      grid-template-columns: 1fr;
      gap: 10px;
      align-items: end;
      padding: 12px;
      border-bottom: 1px solid var(--line);
      background: #0a1217;
    }
    .camera-view {
      position: relative;
      min-height: 138px;
      display: grid;
      place-items: center;
      overflow: hidden;
      border: 1px solid var(--line);
      border-radius: 7px;
      background: #020608;
    }
    .camera-view img { width: 100%; height: auto; display: none; }
    .camera-view.online img { display: block; }
    .camera-placeholder {
      position: absolute;
      color: #cbd5e1;
      font-weight: 750;
    }
    .camera-view.online .camera-placeholder { display: none; }
    .camera-caption {
      display: flex;
      justify-content: space-between;
      gap: 10px;
      padding: 7px 9px;
      color: var(--muted);
      background: #0a1217;
      font-size: 12px;
    }
    .rviz-stage {
      min-height: 0;
      display: grid;
      grid-template-rows: auto minmax(0, 1fr);
      background: #020608;
    }
    .rviz-toolbar { display: none; }
    .rviz-url-row { display: grid; grid-template-columns: 140px minmax(0, 1fr); gap: 8px; align-items: end; }
    .rviz-actions { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; justify-content: flex-end; }
    .rviz-frame-wrap {
      position: relative;
      min-height: 0;
      overflow: hidden;
      background: #020608;
    }
    #rviz_embed_frame {
      width: 100%;
      height: 100%;
      min-height: 100%;
      border: 0;
      display: none;
      background: #020608;
    }
    .rviz-frame-wrap.online #rviz_embed_frame { display: block; }
    .rviz-placeholder {
      position: absolute;
      inset: 0;
      display: grid;
      place-items: center;
      padding: 24px;
      color: var(--muted);
      text-align: center;
      background:
        linear-gradient(transparent 31px, rgba(34, 211, 238, .08) 32px),
        linear-gradient(90deg, transparent 31px, rgba(34, 211, 238, .08) 32px),
        #020608;
      background-size: 32px 32px;
    }
    .rviz-frame-wrap.online .rviz-placeholder { display: none; }
    .rviz-placeholder strong {
      display: block;
      color: var(--text);
      font-size: 20px;
      margin-bottom: 8px;
    }
    footer {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 420px;
      gap: 10px;
      padding: 0 10px 10px;
      min-height: 0;
    }
    pre {
      margin: 0;
      min-height: 130px;
      max-height: 148px;
      overflow: auto;
      padding: 12px;
      font-family: var(--mono);
      font-size: 12px;
      line-height: 1.55;
      white-space: pre-wrap;
      word-break: break-word;
    }
    #logs { background: #020608; color: #d3eef5; }
    #logs { user-select: text; }
    #command { background: #081116; color: #bad6dc; }
    @media (max-width: 1220px) {
      .app { grid-template-rows: auto auto auto; }
      main, footer, .setup-grid, .rviz-toolbar, .rviz-url-row { grid-template-columns: 1fr; }
      header { height: auto; padding: 12px 14px; align-items: flex-start; flex-direction: column; }
      .actions { justify-content: flex-start; }
      .left-rail, .center-stage, .right-rail { overflow: visible; }
    }
  </style>
</head>
<body>
  <div class="app">
    <header>
      <div class="brand">
        <div class="logo" aria-label="引望 工具装备团队">
          <div class="logo-main">引望</div>
          <div class="logo-rule"></div>
          <div class="logo-sub">工具装备团队</div>
        </div>
      </div>
      <div class="actions">
        <span class="ui-only-badge">UI ONLY · 未连接 ROS</span>
        <button class="btn primary" onclick="startNavigation()">启动当前模式</button>
        <button class="btn danger" onclick="postAction('/api/navigation/stop_all')">停止全部</button>
        <button id="native_rviz_button" class="btn soft" onclick="openNativeRviz()">打开原生 RViz</button>
      </div>
    </header>

    <main>
      <aside class="left-rail stack">
        <section class="panel">
          <div class="panel-head">机器人状态</div>
          <div class="body">
            <div class="robot-overview">
              <div class="robot-figure">
                <img id="robot_status_image" src="/assets/h1_2_status.png" alt="H1-2 Robot">
              </div>
              <div class="robot-summary">
                <button id="cruise_ready_card" class="status-metric waiting" type="button" disabled>
                  <div id="cruise_ready_text" class="metric-value">等待定位</div>
                </button>
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

        <section class="panel camera-section">
          <div class="panel-head">
            <span>Camera Preview</span>
            <span id="camera_state" class="pill warn">等待相机话题</span>
          </div>
          <div class="camera-grid">
            <div>
              <div id="camera_rgb_view" class="camera-view">
                <img id="camera_rgb" alt="RealSense RGB">
                <div class="camera-placeholder">等待 RGB 相机话题</div>
              </div>
              <div class="camera-caption"><strong>RGB</strong><span id="camera_rgb_meta">-</span></div>
            </div>
            <div>
              <div id="camera_depth_view" class="camera-view">
                <img id="camera_depth" alt="RealSense aligned depth">
                <div class="camera-placeholder">等待对齐深度话题</div>
              </div>
              <div class="camera-caption"><strong>Aligned Depth</strong><span id="camera_depth_meta">-</span></div>
            </div>
            <div>
              <div id="vision_annotated_view" class="camera-view">
                <img id="vision_annotated" alt="YOLO detection">
                <div class="camera-placeholder">等待 YOLO 检测话题</div>
              </div>
              <div class="camera-caption">
                <strong>YOLO Detection</strong>
                <span><span id="vision_state">waiting</span> · <span id="vision_annotated_meta">-</span></span>
              </div>
            </div>
          </div>
        </section>
      </aside>

      <section class="center-stage">
        <section class="panel rviz-stage">
          <div class="panel-head">
            <span>RViz 监控大屏</span>
            <span id="rviz_embed_state" class="pill warn">等待 RViz Web 地址</span>
          </div>
          <div id="rviz_frame_wrap" class="rviz-frame-wrap">
            <iframe id="rviz_embed_frame" title="RViz Web View" referrerpolicy="no-referrer"></iframe>
            <div class="rviz-placeholder">
              <div>
                <strong>RViz Web 画面未加载</strong>
                <span>启动 noVNC/xpra 后填写地址，或直接打开原生 RViz 调试。</span>
              </div>
            </div>
          </div>
        </section>

        <section class="panel center-controls">
          <div class="panel-head">主控制</div>
          <div class="body control-grid">
            <button id="start_route_button" class="btn primary" onclick="startRoute()" disabled>开始巡航</button>
            <button class="btn" onclick="postAction('/api/route/stop')">停止巡航</button>
            <button id="start_teaching_record_button" class="btn primary" onclick="postAction('/api/teaching_record/start', true)" disabled>启动示教录制</button>
            <button class="btn" onclick="postAction('/api/teaching_record/process/stop')">结束示教录制</button>
            <button id="start_teaching_route_button" class="btn primary" onclick="startTeachingRoute()" disabled>启动示教巡航</button>
            <button class="btn" onclick="postAction('/api/teaching_route/process/stop')">结束示教巡航</button>
            <button class="btn soft" onclick="postAction('/api/voice/start', true)">启动语音控制</button>
            <button class="btn" onclick="postAction('/api/voice/stop', true)">停止语音控制</button>
            <button class="btn" onclick="gesture('wave_hand')">测试挥手</button>
            <button class="btn" onclick="gesture('shake_hand')">测试出手</button>
            <button class="btn soft" onclick="loadRvizEmbed()">加载中间 RViz</button>
            <button class="btn" onclick="reloadRvizEmbed()">刷新嵌入画面</button>
            <button class="btn" onclick="openRvizEmbedWindow()">新窗口打开</button>
            <button class="btn soft" onclick="openNativeRviz()">打开原生 RViz</button>
            <button class="btn" onclick="postAction('/api/rviz/close')">关闭 RViz</button>
            <button class="btn" onclick="postAction('/api/rviz/restart', true)">重启 RViz</button>
            <button class="btn" onclick="refreshStatus()">刷新状态</button>
          </div>
        </section>

        <section class="panel">
          <div class="panel-head">Setup</div>
          <div class="body stack advanced-config">
            <div class="setup-grid">
              <div><label>Run Mode</label><select id="run_mode" onchange="onModeChanged()"><option value="online">在线建图导航</option><option value="navigation">离线地图导航</option><option value="save_map">保存 2D 地图</option><option value="save_pcd_map">保存 3D PCD</option></select></div>
              <div><label>RViz Mode</label><select id="rviz_mode"><option value="auto">跟随 Run Mode</option><option value="online">在线 RViz</option><option value="offline">离线 RViz</option></select></div>
              <label class="check"><input id="rviz_embed_enabled" type="checkbox"> 启用中间 RViz 嵌入</label>
              <div><label>RViz Web URL</label><input id="rviz_embed_url" placeholder="http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=remote"></div>
              <div><label>Map YAML</label><input id="map"></div>
              <div><label>PCD Map</label><input id="pcd"></div>
              <div><label>Map Prefix</label><input id="map_prefix" value="maps/h1_2_lab"></div>
              <div><label>PCD Prefix</label><input id="pcd_prefix" value="maps/h1_2_lab"></div>
              <div><label>Localization</label><select id="localization"><option>global_gicp</option><option>icp</option></select></div>
              <div><label>Initial Pose</label><input id="initial_pose" value="START"></div>
              <div><label>Nav2 Params</label><input id="nav2_params_file" placeholder="/home/llx/h1_nav2_ros2_ws/src/h1_2_nav_tools/config/nav2_h1_2_rpp.yaml"></div>
              <div><label>Teaching Path</label><input id="teaching_path_file"></div>
              <div><label>Teaching Events</label><input id="teaching_events_file"></div>
              <div><label>Voice SSH</label><input id="voice_remote_host" placeholder="unitree@192.168.x.x"></div>
              <div><label>Voice Workspace</label><input id="voice_workspace" value="~/ros2_ws"></div>
              <div><label>Voice ROS setup</label><input id="voice_ros_setup" value="/opt/ros/humble/setup.bash"></div>
            </div>
            <div class="two">
              <label class="check"><input id="auto_start_route" type="checkbox"> auto_start_route</label>
              <label class="check"><input id="return_to_start" type="checkbox"> return_to_start</label>
            </div>
            <label class="check"><input id="teaching_loop" type="checkbox"> teaching_loop</label>
            <button class="btn" onclick="saveConfig()">保存配置</button>
          </div>
        </section>
      </section>

      <aside class="right-rail stack">
        <section class="panel route-panel">
          <div class="panel-head">
            <span>Route</span>
            <span>
              <button class="btn" onclick="selectAll(true)">全选</button>
              <button class="btn" onclick="selectAll(false)">清空</button>
              <button class="btn" onclick="loadPoints()">刷新点位</button>
            </span>
          </div>
          <div class="body">
            <table>
              <thead><tr><th>启用</th><th>点位</th><th>动作</th><th>位置</th></tr></thead>
              <tbody id="routeBody"></tbody>
            </table>
          </div>
        </section>

        <section class="panel">
          <div class="panel-head">Camera / Vision Control</div>
          <div class="body stack">
            <div><label>RGB Topic</label><input id="camera_rgb_topic"></div>
            <div><label>Aligned Depth Topic</label><input id="camera_depth_topic"></div>
            <div><label>CameraInfo Topic</label><input id="camera_info_topic"></div>
            <div class="two">
              <button class="btn soft" onclick="startCameraPreview()">开启相机预览</button>
              <button class="btn" onclick="stopCameraPreview()">关闭相机预览</button>
            </div>
            <button class="btn" onclick="reconnectCamera()">重新连接</button>
            <div><label>YOLO Annotated Topic</label><input id="vision_annotated_topic"></div>
            <div><label>Vision SSH</label><input id="vision_remote_host" placeholder="unitree@192.168.x.x"></div>
            <div><label>Vision Workspace</label><input id="vision_workspace" value="~/ros2_ws"></div>
            <div><label>Vision ROS setup</label><input id="vision_ros_setup" value="/opt/ros/humble/setup.bash"></div>
            <div class="two">
              <button class="btn soft" onclick="postAction('/api/vision/start', true)">启动识别</button>
              <button class="btn" onclick="postAction('/api/vision/stop', true)">停止识别</button>
            </div>
          </div>
        </section>
      </aside>
    </main>

    <footer>
      <section class="panel">
        <div class="panel-head">Logs</div>
        <pre id="logs"></pre>
      </section>
      <section class="panel">
        <div class="panel-head">Generated Command</div>
        <pre id="command"></pre>
      </section>
    </footer>
  </div>

  <script>
    let points = [];
    let routeState = {};
    let lastLogSeq = 0;
    let logsAutoFollow = true;
    let logsSelecting = false;
    let cameraPreviewEnabled = false;
    let rvizEmbedLoaded = false;

    const $ = (id) => document.getElementById(id);

    function payload() {
      const selected = points.filter((p) => p.name !== "START" && routeState[p.name]?.use).map((p) => p.name);
      const gestures = points
        .filter((p) => routeState[p.name]?.use && routeState[p.name]?.gesture)
        .map((p) => `${p.name}:${routeState[p.name].gesture}`);
      return {
        run_mode: $("run_mode").value,
        rviz_mode: $("rviz_mode").value,
        rviz_embed_url: $("rviz_embed_url").value.trim(),
        rviz_embed_enabled: $("rviz_embed_enabled").checked,
        map: $("map").value.trim(),
        pcd: $("pcd").value.trim(),
        map_prefix: $("map_prefix").value.trim(),
        pcd_prefix: $("pcd_prefix").value.trim(),
        localization: $("localization").value,
        initial_pose: $("initial_pose").value.trim() || "START",
        nav2_params_file: $("nav2_params_file").value.trim(),
        auto_start_route: $("auto_start_route").checked,
        return_to_start: $("return_to_start").checked,
        teaching_path_file: $("teaching_path_file").value.trim(),
        teaching_events_file: $("teaching_events_file").value.trim(),
        teaching_loop: $("teaching_loop").checked,
        memory_route: selected.join(","),
        gesture_route: gestures.join(","),
        voice_remote_host: $("voice_remote_host").value.trim(),
        voice_workspace: $("voice_workspace").value.trim() || "~/ros2_ws",
        voice_ros_setup: $("voice_ros_setup").value.trim() || "/opt/ros/humble/setup.bash",
        camera_rgb_topic: $("camera_rgb_topic").value.trim(),
        camera_depth_topic: $("camera_depth_topic").value.trim(),
        camera_info_topic: $("camera_info_topic").value.trim(),
        camera_preview_enabled: cameraPreviewEnabled,
        vision_annotated_topic: $("vision_annotated_topic").value.trim(),
        vision_remote_host: $("vision_remote_host").value.trim(),
        vision_workspace: $("vision_workspace").value.trim() || "~/ros2_ws",
        vision_ros_setup: $("vision_ros_setup").value.trim() || "/opt/ros/humble/setup.bash"
      };
    }

    function updateCommand() {
      const p = payload();
      const command = [`./run_ai_slam_nav.sh mode=${p.run_mode}`];
      if (p.run_mode === "online") {
        command.push(`pcd_prefix=${p.pcd_prefix}`);
      } else if (p.run_mode === "save_map") {
        command.push(`map_prefix=${p.map_prefix}`);
      } else if (p.run_mode === "save_pcd_map") {
        command.push(`pcd_prefix=${p.pcd_prefix}`);
      } else {
        command.push(
          `map=${p.map}`,
          `localization=${p.localization}`,
          `pcd=${p.pcd}`,
          ...(p.nav2_params_file ? [`params_file=${p.nav2_params_file}`] : []),
          `initial_pose=${p.initial_pose}`,
          `memory_route=${p.memory_route}`,
          `gesture_route=${p.gesture_route}`,
          `auto_start_route=${p.auto_start_route}`,
          `return_to_start=${p.return_to_start}`
        );
      }
      command.push(
        "",
        "ros2 launch h1_2_nav_tools teaching_route_runner.launch.py",
        `  path_file:=${p.teaching_path_file}`,
        `  events_file:=${p.teaching_events_file}`,
        `  loop:=${p.teaching_loop}`
      );
      $("command").textContent = command.join(" \\\n  ");
    }
    async function api(path, options = {}) {
      const response = await fetch(path, options);
      return response.json();
    }

    async function loadConfig() {
      const config = await api("/api/config");
      document.body.classList.toggle("ui-only", config.ui_only === true);
      $("run_mode").value = config.run_mode || "online";
      $("rviz_mode").value = config.rviz_mode || "auto";
      $("rviz_embed_url").value = config.rviz_embed_url || "http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=remote";
      $("rviz_embed_enabled").checked = config.rviz_embed_enabled !== false;
      $("map").value = config.map || "";
      $("pcd").value = config.pcd || "";
      $("map_prefix").value = config.map_prefix || "maps/h1_2_lab";
      $("pcd_prefix").value = config.pcd_prefix || "maps/h1_2_lab";
      $("localization").value = config.localization || "global_gicp";
      $("initial_pose").value = config.initial_pose || "START";
      $("nav2_params_file").value = config.nav2_params_file || "";
      $("voice_remote_host").value = config.voice_remote_host || "";
      $("voice_workspace").value = config.voice_workspace || "~/ros2_ws";
      $("voice_ros_setup").value = config.voice_ros_setup || "/opt/ros/humble/setup.bash";
      $("teaching_path_file").value = config.teaching_path_file || "";
      $("teaching_events_file").value = config.teaching_events_file || "";
      $("teaching_loop").checked = config.teaching_loop !== false;
      $("camera_rgb_topic").value = config.camera_rgb_topic || "/camera/camera/color/image_raw";
      $("camera_depth_topic").value = config.camera_depth_topic || "/camera/camera/aligned_depth_to_color/image_raw";
      $("camera_info_topic").value = config.camera_info_topic || "/camera/camera/color/camera_info";
      cameraPreviewEnabled = false;
      $("vision_annotated_topic").value = config.vision_annotated_topic || "/h1_2/vision/annotated_image";
      $("vision_remote_host").value = config.vision_remote_host || config.voice_remote_host || "";
      $("vision_workspace").value = config.vision_workspace || config.voice_workspace || "~/ros2_ws";
      $("vision_ros_setup").value = config.vision_ros_setup || config.voice_ros_setup || "/opt/ros/humble/setup.bash";
      $("auto_start_route").checked = config.auto_start_route === true;
      $("return_to_start").checked = !!config.return_to_start;
      routeState.__memory_route = config.memory_route || "A,B,C";
      routeState.__gesture_route = config.gesture_route || "";
      onModeChanged();
      updateRvizEmbedState();
      updateCommand();
    }

    function onModeChanged() {
      const mode = $("run_mode").value;
      document.body.dataset.mode = mode;
      updateCommand();
    }

    async function loadPoints() {
      const data = await api("/api/points");
      points = data.points || [];
      const route = new Set((routeState.__memory_route || "A,B,C").split(",").filter((name) => name && name !== "START"));
      const gestures = Object.fromEntries((routeState.__gesture_route || "").split(",").filter((x) => x.includes(":")).map((x) => x.split(":")));
      for (const point of points) {
        if (!routeState[point.name]) {
          routeState[point.name] = { use: point.name !== "START" && route.has(point.name), gesture: gestures[point.name] || "" };
        }
      }
      renderRoute();
    }

    function renderRoute() {
      $("routeBody").innerHTML = "";
      for (const point of points) {
        const state = routeState[point.name] || { use: false, gesture: "" };
        const tr = document.createElement("tr");
        const pos = point.position || {x: 0, y: 0, z: 0};
        tr.innerHTML = `
          <td><input type="checkbox" ${state.use ? "checked" : ""} ${point.name === "START" ? "disabled" : ""} onchange="setUse('${point.name}', this.checked)"></td>
          <td><span class="point">${point.name}</span></td>
          <td>
            <select onchange="setGesture('${point.name}', this.value)">
              <option value="" ${state.gesture === "" ? "selected" : ""}>无动作</option>
              <option value="wave_hand" ${state.gesture === "wave_hand" ? "selected" : ""}>wave_hand</option>
              <option value="shake_hand" ${state.gesture === "shake_hand" ? "selected" : ""}>shake_hand</option>
            </select>
          </td>
          <td class="pos">x=${pos.x.toFixed(3)}, y=${pos.y.toFixed(3)}, z=${pos.z.toFixed(3)}</td>
        `;
        $("routeBody").appendChild(tr);
      }
      updateCommand();
    }

    function setUse(name, use) {
      if (name === "START") return;
      routeState[name] = routeState[name] || { use: false, gesture: "" };
      routeState[name].use = use;
      updateCommand();
    }

    function setGesture(name, gesture) {
      routeState[name] = routeState[name] || { use: false, gesture: "" };
      routeState[name].gesture = gesture;
      updateCommand();
    }

    function selectAll(enabled) {
      for (const point of points) {
        routeState[point.name] = routeState[point.name] || { use: false, gesture: "" };
        routeState[point.name].use = enabled && point.name !== "START";
      }
      renderRoute();
    }

    async function saveConfig() {
      const result = await api("/api/config", { method: "POST", body: JSON.stringify(payload()) });
      appendLog("web", result.message || "config saved");
    }

    async function startNavigation() {
      const result = await api("/api/navigation/start", { method: "POST", body: JSON.stringify(payload()) });
      appendLog("web", result.message);
      refreshStatus();
    }

    async function postAction(path, includePayload = false) {
      const result = await api(path, { method: "POST", body: JSON.stringify(includePayload ? payload() : {}) });
      appendLog("web", result.message || JSON.stringify(result));
      refreshStatus();
    }

    async function openNativeRviz() {
      await postAction("/api/rviz/open", true);
    }

    function updateRvizEmbedState(message = null, state = null) {
      const enabled = $("rviz_embed_enabled").checked;
      const url = $("rviz_embed_url").value.trim();
      const badge = $("rviz_embed_state");
      const wrap = $("rviz_frame_wrap");
      wrap.classList.toggle("online", enabled && rvizEmbedLoaded && !!url);
      if (!enabled) {
        badge.textContent = "嵌入已关闭";
        badge.className = "pill warn";
        return;
      }
      if (!url) {
        badge.textContent = "等待 RViz Web 地址";
        badge.className = "pill warn";
        return;
      }
      badge.textContent = message || (rvizEmbedLoaded ? "RViz Web 已加载" : "可加载 RViz Web");
      badge.className = "pill " + (state || (rvizEmbedLoaded ? "ok" : ""));
    }

    function loadRvizEmbed() {
      const url = $("rviz_embed_url").value.trim();
      if (!$("rviz_embed_enabled").checked || !url) {
        rvizEmbedLoaded = false;
        $("rviz_embed_frame").removeAttribute("src");
        updateRvizEmbedState();
        return;
      }
      $("rviz_embed_frame").src = url;
      rvizEmbedLoaded = true;
      updateRvizEmbedState("RViz Web 已加载", "ok");
    }

    function reloadRvizEmbed() {
      const url = $("rviz_embed_url").value.trim();
      if (!url) {
        updateRvizEmbedState();
        return;
      }
      $("rviz_embed_frame").src = `${url}${url.includes("?") ? "&" : "?"}t=${Date.now()}`;
      rvizEmbedLoaded = true;
      updateRvizEmbedState("RViz Web 已刷新", "ok");
    }

    function openRvizEmbedWindow() {
      const url = $("rviz_embed_url").value.trim();
      if (!url) {
        updateRvizEmbedState();
        return;
      }
      window.open(url, "_blank", "noopener,noreferrer");
    }

    async function startRoute() {
      const result = await api("/api/route/start", { method: "POST", body: "{}" });
      appendLog("route", result.message || JSON.stringify(result));
      refreshStatus();
    }

    async function startTeachingRoute() {
      const result = await api("/api/teaching_route/start", { method: "POST", body: JSON.stringify(payload()) });
      appendLog("teaching-route", result.message || JSON.stringify(result));
      refreshStatus();
    }

    async function gesture(name) {
      const result = await api("/api/gesture", { method: "POST", body: JSON.stringify({ name }) });
      appendLog("gesture", `${name}: ${result.success ? "ok" : "failed"} ${result.message || ""}`);
    }

    function restartCameraImages() {
      const stamp = Date.now();
      $("camera_rgb").src = `/api/camera/rgb.mjpg?t=${stamp}`;
      $("camera_depth").src = `/api/camera/depth.mjpg?t=${stamp}`;
      $("vision_annotated").src = `/api/vision/annotated.mjpg?t=${stamp}`;
    }

    function stopCameraImages() {
      $("camera_rgb").removeAttribute("src");
      $("camera_depth").removeAttribute("src");
      $("vision_annotated").removeAttribute("src");
    }

    async function startCameraPreview() {
      const request = payload();
      request.camera_preview_enabled = true;
      const result = await api("/api/camera/start", { method: "POST", body: JSON.stringify(request) });
      cameraPreviewEnabled = true;
      appendLog("camera", result.message || JSON.stringify(result));
      restartCameraImages();
      await refreshCameraStatus();
    }

    async function stopCameraPreview() {
      const request = payload();
      request.camera_preview_enabled = false;
      const result = await api("/api/camera/stop", { method: "POST", body: JSON.stringify(request) });
      cameraPreviewEnabled = false;
      appendLog("camera", result.message || JSON.stringify(result));
      stopCameraImages();
      await refreshCameraStatus();
    }

    async function reconnectCamera() {
      const request = payload();
      request.camera_preview_enabled = true;
      const result = await api("/api/camera/reconnect", { method: "POST", body: JSON.stringify(request) });
      cameraPreviewEnabled = true;
      appendLog("camera", result.message || JSON.stringify(result));
      restartCameraImages();
      await refreshCameraStatus();
    }

    function renderCameraStream(name, stream) {
      const view = $(`camera_${name}_view`);
      const meta = $(`camera_${name}_meta`);
      view.classList.toggle("online", !!stream.online);
      const age = stream.age_sec == null ? "-" : `${stream.age_sec.toFixed(1)}s`;
      meta.textContent = `${stream.resolution || "-"} · ${stream.encoding || "-"} · age ${age}`;
    }

    async function refreshCameraStatus(cameraData = null) {
      const camera = cameraData || await api("/api/camera/status");
      const state = $("camera_state");
      state.textContent = camera.message || "等待相机话题";
      state.className = "pill " + (camera.state === "online" ? "ok" : "warn");
      renderCameraStream("rgb", camera.rgb || {});
      renderCameraStream("depth", camera.depth || {});
    }

    async function refreshVisionStatus(visionData = null) {
      const vision = visionData || await api("/api/vision/status");
      const stream = vision.annotated || {};
      $("vision_state").textContent = vision.state || "waiting";
      $("vision_annotated_view").classList.toggle("online", !!stream.online);
      const age = stream.age_sec == null ? "-" : `${stream.age_sec.toFixed(1)}s`;
      const fps = stream.fps == null ? "0.0" : stream.fps.toFixed(1);
      $("vision_annotated_meta").textContent =
        `${stream.resolution || "-"} · ${fps} FPS · age ${age}`;
    }

    function formatBatteryMetric(value, unit) {
      if (value == null || value === "") return "--";
      const numeric = Number(value);
      return Number.isFinite(numeric) ? `${numeric.toFixed(1)}${unit}` : String(value);
    }

    async function refreshStatus() {
      const data = await api("/api/status");
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
      startButton.disabled = !localization.ready;
      startButton.title = localization.message || "定位未稳定";
      const startTeachingButton = $("start_teaching_route_button");
      startTeachingButton.disabled = !localization.ready;
      startTeachingButton.title = localization.message || "定位未稳定";
      const startTeachingRecordButton = $("start_teaching_record_button");
      startTeachingRecordButton.disabled = !localization.ready;
      startTeachingRecordButton.title = localization.message || "定位未稳定";
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
      await refreshCameraStatus(data.camera || null);
      await refreshVisionStatus(data.vision || null);
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

    for (const id of ["run_mode", "rviz_mode", "rviz_embed_url", "rviz_embed_enabled", "map", "pcd", "map_prefix", "pcd_prefix", "localization", "initial_pose", "nav2_params_file", "voice_remote_host", "voice_workspace", "voice_ros_setup", "teaching_path_file", "teaching_events_file", "camera_rgb_topic", "camera_depth_topic", "camera_info_topic", "vision_annotated_topic", "vision_remote_host", "vision_workspace", "vision_ros_setup", "auto_start_route", "return_to_start", "teaching_loop"]) {
      document.addEventListener("input", (event) => { if (event.target.id === id) updateCommand(); });
      document.addEventListener("change", (event) => { if (event.target.id === id) updateCommand(); });
    }
    document.addEventListener("input", (event) => {
      if (event.target.id === "rviz_embed_url") updateRvizEmbedState();
    });
    document.addEventListener("change", (event) => {
      if (event.target.id === "rviz_embed_url" || event.target.id === "rviz_embed_enabled") updateRvizEmbedState();
    });

    (async function boot() {
      await loadConfig();
      stopCameraImages();
      await loadPoints();
      await refreshStatus();
      await pollLogs();
      setupLogScrollLock();
      setInterval(refreshStatus, 1200);
      setInterval(pollLogs, 700);
    })();
  </script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(description="H1-2 offline navigation local web GUI")
    parser.add_argument("--host", default="127.0.0.1")
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

    if not args.no_single_instance:
        stop_existing_web_guis()

    workspace_root = find_workspace_root()
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


if __name__ == "__main__":
    sys.exit(main())
