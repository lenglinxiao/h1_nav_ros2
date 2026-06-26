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

from h1_2_nav_tools.camera_stream import (
    CameraFrameStore,
    encode_depth_jpeg,
    encode_rgb_jpeg,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


CONFIG_PATH = Path.home() / ".ros" / "h1_2_nav_web_gui.json"
CONFIG_WORKFLOW_VERSION = 2
LOCALIZATION_REQUIRED_STABLE_SAMPLES = 5
LOCALIZATION_MAX_SCORE = 0.15
LOCALIZATION_STATUS_TIMEOUT_SEC = 10.0
GESTURE_OPTIONS = ["", "wave_hand", "shake_hand"]


def configure_ros_environment():
    os.environ["ROS_LOCALHOST_ONLY"] = "0"
    os.environ.setdefault("ROS_DOMAIN_ID", "0")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ.pop("ROS_HOSTNAME", None)
    os.environ.pop("ROS_IP", None)
    os.environ.pop("CYCLONEDDS_URI", None)
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
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.start_client = self.node.create_client(StartRoute, "start_memory_route")
        self.stop_client = self.node.create_client(StopRoute, "stop_memory_route")
        self.status_client = self.node.create_client(RouteStatus, "memory_route_status")
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
        if not self.status_client.wait_for_service(timeout_sec=0.05):
            return {
                "success": False,
                "state": "unavailable",
                "current_goal": "",
                "current_index": 0,
                "message": "memory_route_status is not available",
            }
        ok, result = self._wait_future(self.status_client.call_async(RouteStatus.Request()))
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
        ready = (
            age <= LOCALIZATION_STATUS_TIMEOUT_SEC
            and stable_samples >= LOCALIZATION_REQUIRED_STABLE_SAMPLES
        )
        if ready:
            status_message = (
                f"定位稳定，可开始巡航（score={message.score:.3f}，"
                f"{stable_samples}/{LOCALIZATION_REQUIRED_STABLE_SAMPLES}）"
            )
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


class ProcessManager:
    def __init__(self, workspace_root, logs):
        self.workspace_root = workspace_root
        self.logs = logs
        self.nav_process = None
        self.rviz_process = None
        self.voice_process = None
        self.voice_bridge_process = None
        self.vision_process = None
        self.lock = threading.Lock()

    def status(self):
        return {
            "navigation": self._process_state(self.nav_process),
            "rviz": self._process_state(self.rviz_process),
            "voice": self._process_state(self.voice_process),
            "voice_bridge": self._process_state(self.voice_bridge_process),
            "vision": self._process_state(self.vision_process),
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
    def __init__(self, workspace_root):
        self.workspace_root = workspace_root
        self.points_file = workspace_root / "maps" / "memory_points.json"
        self.logs = LogStore()
        self.processes = ProcessManager(workspace_root, self.logs)
        self.ros = RosServiceBridge(self.logs, self.default_config())
        self.logs.add("web", f"workspace={workspace_root}")

    def shutdown(self):
        self.processes.shutdown()
        self.ros.shutdown()

    def default_config(self):
        config = {
            "map": str(self.workspace_root / "maps" / "h1_2_lab.yaml"),
            "pcd": str(self.workspace_root / "maps" / "h1_2_lab.pcd"),
            "run_mode": "online",
            "rviz_mode": "auto",
            "map_prefix": "maps/h1_2_lab",
            "pcd_prefix": "maps/h1_2_lab",
            "localization": "global_gicp",
            "initial_pose": "START",
            "auto_start_route": False,
            "return_to_start": False,
            "memory_route": "A,B,C",
            "gesture_route": "",
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
        config["camera_preview_enabled"] = False
        config["workflow_version"] = CONFIG_WORKFLOW_VERSION
        config["voice_remote_host"] = normalize_ssh_target(config.get("voice_remote_host", ""))
        config["voice_workspace"] = normalize_voice_workspace(config.get("voice_workspace", "~/ros2_ws"))
        config["vision_remote_host"] = normalize_ssh_target(config.get("vision_remote_host", ""))
        config["vision_workspace"] = normalize_voice_workspace(
            config.get("vision_workspace", "~/ros2_ws")
        )
        return config

    def save_config(self, config):
        config["memory_route"] = normalize_memory_route(
            config.get("memory_route", "A,B,C")
        )
        config["workflow_version"] = CONFIG_WORKFLOW_VERSION
        if "voice_remote_host" in config:
            config["voice_remote_host"] = normalize_ssh_target(config["voice_remote_host"])
        if "voice_workspace" in config:
            config["voice_workspace"] = normalize_voice_workspace(config["voice_workspace"])
        if "vision_remote_host" in config:
            config["vision_remote_host"] = normalize_ssh_target(config["vision_remote_host"])
        if "vision_workspace" in config:
            config["vision_workspace"] = normalize_voice_workspace(config["vision_workspace"])
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
            self._send_json(
                {
                    "route": route,
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
        else:
            self.send_error(404)

    def do_POST(self):
        parsed = urlparse(self.path)
        payload = self._read_json()
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
      --bg: #f7f7f4;
      --panel: #ffffff;
      --panel-2: #fbfbf9;
      --text: #1f2328;
      --muted: #6b7280;
      --line: #e3e3df;
      --accent: #2563eb;
      --accent-2: #0f766e;
      --danger: #b42318;
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
      grid-template-rows: auto 1fr auto;
    }
    header {
      min-height: 96px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 18px;
      padding: 0 28px;
      border-bottom: 1px solid var(--line);
      background: #fff;
      backdrop-filter: blur(14px);
      position: sticky;
      top: 0;
      z-index: 2;
    }
    .brand { display: flex; align-items: center; gap: 12px; min-width: 0; }
    .logo {
      width: 112px;
      display: grid;
      justify-items: center;
      padding: 8px 0 6px;
      color: #111827;
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
      background: #111827;
    }
    .logo-sub {
      font-size: 14px;
      font-weight: 850;
      white-space: nowrap;
      color: #374151;
    }
    .actions { display: flex; gap: 8px; flex-wrap: wrap; justify-content: flex-end; }
    .btn {
      height: 34px;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 0 12px;
      background: #fff;
      color: var(--text);
      font-weight: 750;
      cursor: pointer;
    }
    .btn:hover { background: #f1f5f9; }
    .btn.primary { background: var(--accent); border-color: var(--accent); color: #fff; }
    .btn.soft { background: #ecfdf5; border-color: #bbf7d0; color: var(--accent-2); }
    .btn.danger { background: #fff1f0; border-color: #fecaca; color: var(--danger); }
    main {
      display: grid;
      grid-template-columns: 280px minmax(520px, 1fr) 340px;
      gap: 14px;
      padding: 14px;
      min-height: 0;
    }
    .panel {
      min-width: 0;
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 12px;
      overflow: hidden;
    }
    .panel-head {
      height: 44px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 0 14px;
      border-bottom: 1px solid var(--line);
      background: var(--panel-2);
      font-weight: 850;
    }
    .body { padding: 14px; }
    .stack { display: grid; gap: 11px; align-content: start; }
    label { display: block; margin-bottom: 6px; color: var(--muted); font-size: 12px; font-weight: 800; }
    input, select {
      width: 100%;
      height: 34px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fff;
      padding: 0 10px;
      color: var(--text);
      outline: none;
    }
    input:focus, select:focus { border-color: #93c5fd; box-shadow: 0 0 0 3px rgba(37, 99, 235, .12); }
    .check {
      display: flex; gap: 8px; align-items: center;
      color: var(--text); font-weight: 650; font-size: 13px;
    }
    .check input { width: 15px; height: 15px; }
    table { width: 100%; border-collapse: collapse; table-layout: fixed; }
    th, td { height: 44px; border-bottom: 1px solid #ededeb; padding: 8px 10px; text-align: left; }
    th { color: var(--muted); background: #fafafa; font-size: 12px; font-weight: 850; }
    th:first-child, td:first-child { width: 56px; text-align: center; }
    th:nth-child(2), td:nth-child(2) { width: 90px; }
    th:nth-child(3), td:nth-child(3) { width: 150px; }
    tr:last-child td { border-bottom: 0; }
    .point {
      display: inline-flex; min-width: 36px; height: 26px;
      align-items: center; justify-content: center;
      background: #eef2ff; color: #3730a3;
      border-radius: 7px; font-weight: 900;
    }
    .pos { color: var(--muted); font-family: var(--mono); font-size: 12px; }
    .two { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .status-row { display: grid; grid-template-columns: 90px 1fr; gap: 10px; padding: 9px 0; border-bottom: 1px solid #ededeb; }
    .status-row:last-child { border-bottom: 0; }
    .key { color: var(--muted); font-size: 12px; font-weight: 850; }
    .pill {
      display: inline-flex; align-items: center; width: fit-content;
      min-height: 24px; padding: 3px 9px; border-radius: 999px;
      background: #eef2f7; color: #475569; font-weight: 850; font-size: 12px;
    }
    .pill.ok { background: #ecfdf5; color: #047857; }
    .pill.warn { background: #fff7ed; color: #c2410c; }
    .localization-indicator {
      width: 100%;
      min-height: 56px;
      border: 1px solid #cbd5e1;
      border-radius: 10px;
      padding: 10px 14px;
      font-size: 15px;
      font-weight: 900;
      text-align: left;
      cursor: default;
      transition: background 160ms ease, border-color 160ms ease, color 160ms ease;
    }
    .localization-indicator.waiting {
      background: #f1f5f9;
      border-color: #cbd5e1;
      color: #64748b;
    }
    .localization-indicator.checking {
      background: #fff7ed;
      border-color: #fdba74;
      color: #c2410c;
    }
    .localization-indicator.ready {
      background: #dcfce7;
      border-color: #4ade80;
      color: #166534;
      box-shadow: 0 0 0 3px rgba(74, 222, 128, 0.18);
    }
    .camera-section { margin: 0 14px 14px; }
    .camera-config {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr auto;
      gap: 10px;
      align-items: end;
      padding: 14px;
      border-bottom: 1px solid var(--line);
    }
    .camera-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 14px;
      padding: 14px;
    }
    .vision-config {
      display: grid;
      grid-template-columns: 1.3fr 1fr 1fr 1fr auto auto;
      gap: 10px;
      align-items: end;
      padding: 14px;
      border-bottom: 1px solid var(--line);
      background: #fbfbf9;
    }
    .camera-view {
      position: relative;
      min-height: 260px;
      display: grid;
      place-items: center;
      overflow: hidden;
      border: 1px solid var(--line);
      border-radius: 10px;
      background: #111827;
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
      padding: 8px 10px;
      color: var(--muted);
      background: #fff;
      font-size: 12px;
    }
    footer {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 420px;
      gap: 14px;
      padding: 0 14px 14px;
    }
    pre {
      margin: 0;
      min-height: 168px;
      max-height: 240px;
      overflow: auto;
      padding: 13px 14px;
      font-family: var(--mono);
      font-size: 12px;
      line-height: 1.55;
      white-space: pre-wrap;
      word-break: break-word;
    }
    #logs { background: #0b1020; color: #e0ecff; }
    #logs { user-select: text; }
    #command { background: #fafafa; color: #334155; }
    @media (max-width: 1120px) {
      main, footer, .camera-grid, .camera-config, .vision-config { grid-template-columns: 1fr; }
      header { height: auto; padding: 12px 14px; align-items: flex-start; flex-direction: column; }
      .actions { justify-content: flex-start; }
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
        <button class="btn primary" onclick="startNavigation()">启动当前模式</button>
        <button class="btn danger" onclick="postAction('/api/navigation/stop_all')">停止全部</button>
        <button class="btn soft" onclick="postAction('/api/rviz/open', true)">打开 RViz</button>
      </div>
    </header>

    <main>
      <section class="panel">
        <div class="panel-head">Setup</div>
        <div class="body stack">
          <div><label>Run Mode</label><select id="run_mode" onchange="onModeChanged()"><option value="online">在线建图导航</option><option value="navigation">离线地图导航</option><option value="save_map">保存 2D 地图</option><option value="save_pcd_map">保存 3D PCD</option></select></div>
          <div><label>RViz Mode</label><select id="rviz_mode"><option value="auto">跟随 Run Mode</option><option value="online">在线 RViz</option><option value="offline">离线 RViz</option></select></div>
          <div><label>Map YAML</label><input id="map"></div>
          <div><label>PCD Map</label><input id="pcd"></div>
          <div><label>Map Prefix</label><input id="map_prefix" value="maps/h1_2_lab"></div>
          <div><label>PCD Prefix</label><input id="pcd_prefix" value="maps/h1_2_lab"></div>
          <div><label>Localization</label><select id="localization"><option>global_gicp</option><option>icp</option></select></div>
          <div><label>Initial Pose</label><input id="initial_pose" value="START"></div>
          <div><label>Voice SSH</label><input id="voice_remote_host" placeholder="unitree@192.168.x.x"></div>
          <div><label>Voice Workspace</label><input id="voice_workspace" value="~/ros2_ws"></div>
          <div><label>Voice ROS setup</label><input id="voice_ros_setup" value="/opt/ros/humble/setup.bash"></div>
          <label class="check"><input id="auto_start_route" type="checkbox"> auto_start_route</label>
          <label class="check"><input id="return_to_start" type="checkbox"> return_to_start</label>
          <button class="btn" onclick="saveConfig()">保存配置</button>
        </div>
      </section>

      <section class="panel">
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

      <aside class="stack">
        <section class="panel">
          <div class="panel-head">Route Control</div>
          <div class="body stack">
            <button id="localization_indicator" class="localization-indicator waiting" type="button" disabled>
              ● 等待定位
            </button>
            <div class="two">
            <button id="start_route_button" class="btn primary" onclick="startRoute()" disabled>开始巡航</button>
            <button class="btn" onclick="postAction('/api/route/stop')">停止巡航</button>
            </div>
            <div class="two">
              <button class="btn soft" onclick="postAction('/api/voice/start', true)">启动语音控制</button>
              <button class="btn" onclick="postAction('/api/voice/stop', true)">停止语音控制</button>
            </div>
            <button class="btn" onclick="refreshStatus()">刷新状态</button>
            <div class="two">
              <button class="btn" onclick="gesture('wave_hand')">测试挥手</button>
              <button class="btn" onclick="gesture('shake_hand')">测试出手</button>
            </div>
            <div class="two">
              <button class="btn" onclick="postAction('/api/rviz/close')">关闭 RViz</button>
              <button class="btn" onclick="postAction('/api/rviz/restart', true)">重启 RViz</button>
            </div>
          </div>
        </section>
        <section class="panel">
          <div class="panel-head">Runtime Status</div>
          <div class="body">
            <div class="status-row"><div class="key">state</div><div><span id="state" class="pill">-</span></div></div>
            <div class="status-row"><div class="key">goal</div><div id="goal">-</div></div>
            <div class="status-row"><div class="key">message</div><div id="message">-</div></div>
            <div class="status-row"><div class="key">localization</div><div id="localization_state">定位未稳定</div></div>
            <div class="status-row"><div class="key">process</div><div id="processes">-</div></div>
          </div>
        </section>
      </aside>
    </main>

    <section class="panel camera-section">
      <div class="panel-head">
        <span>Camera Preview</span>
        <span id="camera_state" class="pill warn">等待相机话题</span>
      </div>
      <div class="camera-config">
        <div><label>RGB Topic</label><input id="camera_rgb_topic"></div>
        <div><label>Aligned Depth Topic</label><input id="camera_depth_topic"></div>
        <div><label>CameraInfo Topic</label><input id="camera_info_topic"></div>
        <button class="btn soft" onclick="startCameraPreview()">开启相机预览</button>
        <button class="btn" onclick="stopCameraPreview()">关闭相机预览</button>
        <button class="btn" onclick="reconnectCamera()">重新连接</button>
      </div>
      <div class="vision-config">
        <div><label>YOLO Annotated Topic</label><input id="vision_annotated_topic"></div>
        <div><label>Vision SSH</label><input id="vision_remote_host" placeholder="unitree@192.168.x.x"></div>
        <div><label>Vision Workspace</label><input id="vision_workspace" value="~/ros2_ws"></div>
        <div><label>Vision ROS setup</label><input id="vision_ros_setup" value="/opt/ros/humble/setup.bash"></div>
        <button class="btn soft" onclick="postAction('/api/vision/start', true)">启动识别</button>
        <button class="btn" onclick="postAction('/api/vision/stop', true)">停止识别</button>
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

    const $ = (id) => document.getElementById(id);

    function payload() {
      const selected = points.filter((p) => p.name !== "START" && routeState[p.name]?.use).map((p) => p.name);
      const gestures = points
        .filter((p) => routeState[p.name]?.use && routeState[p.name]?.gesture)
        .map((p) => `${p.name}:${routeState[p.name].gesture}`);
      return {
        run_mode: $("run_mode").value,
        rviz_mode: $("rviz_mode").value,
        map: $("map").value.trim(),
        pcd: $("pcd").value.trim(),
        map_prefix: $("map_prefix").value.trim(),
        pcd_prefix: $("pcd_prefix").value.trim(),
        localization: $("localization").value,
        initial_pose: $("initial_pose").value.trim() || "START",
        auto_start_route: $("auto_start_route").checked,
        return_to_start: $("return_to_start").checked,
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
          `initial_pose=${p.initial_pose}`,
          `memory_route=${p.memory_route}`,
          `gesture_route=${p.gesture_route}`,
          `auto_start_route=${p.auto_start_route}`,
          `return_to_start=${p.return_to_start}`
        );
      }
      $("command").textContent = command.join(" \\\n  ");
    }
    async function api(path, options = {}) {
      const response = await fetch(path, options);
      return response.json();
    }

    async function loadConfig() {
      const config = await api("/api/config");
      $("run_mode").value = config.run_mode || "online";
      $("rviz_mode").value = config.rviz_mode || "auto";
      $("map").value = config.map || "";
      $("pcd").value = config.pcd || "";
      $("map_prefix").value = config.map_prefix || "maps/h1_2_lab";
      $("pcd_prefix").value = config.pcd_prefix || "maps/h1_2_lab";
      $("localization").value = config.localization || "global_gicp";
      $("initial_pose").value = config.initial_pose || "START";
      $("voice_remote_host").value = config.voice_remote_host || "";
      $("voice_workspace").value = config.voice_workspace || "~/ros2_ws";
      $("voice_ros_setup").value = config.voice_ros_setup || "/opt/ros/humble/setup.bash";
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

    async function startRoute() {
      const result = await api("/api/route/start", { method: "POST", body: "{}" });
      appendLog("route", result.message || JSON.stringify(result));
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

    async function refreshStatus() {
      const data = await api("/api/status");
      const route = data.route || {};
      $("state").textContent = route.state || "-";
      $("state").className = "pill " + (route.success ? "ok" : "warn");
      $("goal").textContent = `${route.current_goal || "-"} / index ${route.current_index || 0}`;
      $("message").textContent = route.message || "-";
      const localization = data.localization || {};
      $("localization_state").textContent = localization.message || "定位未稳定";
      const localizationIndicator = $("localization_indicator");
      const indicatorState = localization.ready
        ? "ready"
        : localization.score == null
          ? "waiting"
          : "checking";
      localizationIndicator.className = `localization-indicator ${indicatorState}`;
      localizationIndicator.textContent = localization.ready
        ? "● 可开始巡航"
        : indicatorState === "checking"
          ? "● 定位稳定性确认中"
          : "● 等待定位";
      localizationIndicator.title = localization.message || "定位未稳定";
      const startButton = $("start_route_button");
      startButton.disabled = !localization.ready;
      startButton.title = localization.message || "定位未稳定";
      const processes = data.processes || {};
      $("processes").textContent = `nav=${processes.navigation?.running ? "running" : "idle"}, rviz=${processes.rviz?.running ? "running" : "idle"}, voice_reader=${processes.voice?.running ? "running" : "idle"}, voice_bridge=${processes.voice_bridge?.running ? "running" : "idle"}, vision=${processes.vision?.running ? "running" : "idle"}`;
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

    for (const id of ["run_mode", "rviz_mode", "map", "pcd", "map_prefix", "pcd_prefix", "localization", "initial_pose", "voice_remote_host", "voice_workspace", "voice_ros_setup", "camera_rgb_topic", "camera_depth_topic", "camera_info_topic", "vision_annotated_topic", "vision_remote_host", "vision_workspace", "vision_ros_setup", "auto_start_route", "return_to_start"]) {
      document.addEventListener("input", (event) => { if (event.target.id === id) updateCommand(); });
      document.addEventListener("change", (event) => { if (event.target.id === id) updateCommand(); });
    }

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
    args = parser.parse_args()
    if args.port_tries < 1:
        parser.error("--port-tries must be >= 1")

    if not args.no_single_instance:
        stop_existing_web_guis()

    workspace_root = find_workspace_root()
    server, actual_port = bind_http_server(args.host, args.port, RequestHandler, args.port_tries)
    app = WebGuiApp(workspace_root)
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
