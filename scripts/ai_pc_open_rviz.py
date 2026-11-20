#!/usr/bin/env python3
"""Handle a fixed local RViz URI; never execute commands supplied by a URL."""
import fcntl
import ctypes
import ctypes.util
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from urllib.parse import parse_qs, urlsplit


def parse_mode(uri):
    return parse_options(uri)[0]


def parse_options(uri):
    parsed = urlsplit(uri)
    if parsed.scheme != "h1-rviz" or parsed.netloc != "open" or parsed.path or parsed.fragment:
        raise ValueError("Unsupported RViz link")
    values = parse_qs(parsed.query, keep_blank_values=True, strict_parsing=True)
    if set(values) - {"mode", "backend"}:
        raise ValueError("Unsupported RViz link")
    if any(len(items) != 1 for items in values.values()):
        raise ValueError("Unsupported RViz link")
    mode = values.get("mode", [""])[0]
    backend = values.get("backend", ["unitree"])[0]
    if mode not in ("online", "offline") or backend not in ("unitree", "policy"):
        raise ValueError("Unsupported RViz link")
    return mode, backend


def parse_action(uri):
    parsed = urlsplit(uri)
    if parsed.scheme != "h1-rviz" or parsed.path or parsed.fragment:
        raise ValueError("Unsupported RViz link")
    if parsed.netloc == "close":
        if parsed.query:
            raise ValueError("Unsupported RViz link")
        return "close", None, None
    if parsed.netloc != "open":
        raise ValueError("Unsupported RViz link")
    mode, backend = parse_options(uri)
    return "open", mode, backend


def is_managed_rviz_command(argv):
    parts = [os.fsdecode(part) for part in argv]
    if len(parts) >= 4 and Path(parts[0]).name == "ros2":
        command = parts[1:4]
    elif len(parts) >= 5 and Path(parts[1]).name == "ros2":
        command = parts[2:5]
    else:
        return False
    return command == ["launch", "h1_2_slam_bringup", "h1_2_nav2_rviz.launch.py"]


def select_unique_managed_rviz_pid(candidates):
    matches = {
        pid
        for pid, sid, pgid, argv in candidates
        if pid > 1 and sid == pid and pgid == pid and is_managed_rviz_command(argv)
    }
    return next(iter(matches)) if len(matches) == 1 else None


def _is_managed_rviz_process(pid):
    if pid <= 1 or os.getsid(pid) != pid or os.getpgid(pid) != pid:
        return False
    argv = Path(f"/proc/{pid}/cmdline").read_bytes().split(bytes((0,)))
    return is_managed_rviz_command([part for part in argv if part])


def discover_managed_rviz_pid():
    candidates = []
    try:
        process_entries = Path("/proc").iterdir()
        for entry in process_entries:
            if not entry.name.isdecimal():
                continue
            pid = int(entry.name)
            try:
                argv = entry.joinpath("cmdline").read_bytes().split(bytes((0,)))
                sid = os.getsid(pid)
                pgid = os.getpgid(pid)
            except (OSError, ProcessLookupError):
                continue
            candidates.append((pid, sid, pgid, [part for part in argv if part]))
    except OSError:
        return None
    return select_unique_managed_rviz_pid(candidates)


def _rviz_group_exists(pid):
    try:
        os.killpg(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def close_managed_rviz(pid_path, timeout_sec=8.0):
    pid_path = Path(pid_path)
    try:
        pid = int(pid_path.read_text(encoding="ascii").strip())
    except (OSError, ValueError):
        pid = None
    try:
        if pid is None or not _is_managed_rviz_process(pid):
            pid = discover_managed_rviz_pid()
        if pid is None or not _is_managed_rviz_process(pid):
            return False
    except (OSError, ProcessLookupError):
        return False

    try:
        os.killpg(pid, signal.SIGINT)
    except ProcessLookupError:
        return False

    deadline = time.monotonic() + timeout_sec
    while _rviz_group_exists(pid) and time.monotonic() < deadline:
        time.sleep(0.1)
    if _rviz_group_exists(pid):
        try:
            os.killpg(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + 2.0
        while _rviz_group_exists(pid) and time.monotonic() < deadline:
            time.sleep(0.1)
    if _rviz_group_exists(pid):
        return False

    try:
        if pid_path.read_text(encoding="ascii").strip() == str(pid):
            pid_path.unlink()
    except OSError:
        pass
    return True


def raise_rviz_window():
    """Ask the X11 window manager to activate an existing RViz window."""
    try:
        listing = subprocess.check_output(
            ["xprop", "-root", "_NET_CLIENT_LIST"], text=True, timeout=3,
            stderr=subprocess.DEVNULL,
        )
        window = None
        for candidate in re.findall(r"0x[0-9a-fA-F]+", listing):
            properties = subprocess.check_output(
                ["xprop", "-id", candidate, "WM_CLASS"], text=True, timeout=3,
                stderr=subprocess.DEVNULL,
            )
            if re.search(r'WM_CLASS\(STRING\) = "rviz2", "rviz2"', properties):
                window = int(candidate, 16)
                break
        if window is None:
            return False

        class ClientMessage(ctypes.Structure):
            _fields_ = [
                ("type", ctypes.c_int), ("serial", ctypes.c_ulong),
                ("send_event", ctypes.c_int), ("display", ctypes.c_void_p),
                ("window", ctypes.c_ulong), ("message_type", ctypes.c_ulong),
                ("format", ctypes.c_int), ("data", ctypes.c_long * 5),
            ]

        class Event(ctypes.Union):
            _fields_ = [("client", ClientMessage), ("padding", ctypes.c_long * 24)]

        library = ctypes.util.find_library("X11")
        if not library:
            return False
        x11 = ctypes.CDLL(library)
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        x11.XDefaultRootWindow.restype = ctypes.c_ulong
        x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        x11.XInternAtom.restype = ctypes.c_ulong
        x11.XMapRaised.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        x11.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_long, ctypes.POINTER(Event)]
        x11.XFlush.argtypes = [ctypes.c_void_p]
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        display = x11.XOpenDisplay(None)
        if not display:
            return False
        try:
            event = Event()
            event.client.type = 33  # ClientMessage
            event.client.display = display
            event.client.window = window
            event.client.message_type = x11.XInternAtom(display, b"_NET_ACTIVE_WINDOW", 0)
            event.client.format = 32
            event.client.data[0] = 2  # Explicit user activation, as a desktop launcher.
            x11.XMapRaised(display, window)
            sent = x11.XSendEvent(display, x11.XDefaultRootWindow(display), 0,
                                  (1 << 19) | (1 << 20), ctypes.byref(event))
            x11.XFlush(display)
            return bool(sent)
        finally:
            x11.XCloseDisplay(display)
    except (OSError, subprocess.SubprocessError):
        return False


def main():
    if len(sys.argv) != 2:
        return 2
    try:
        action, mode, backend = parse_action(sys.argv[1])
    except ValueError:
        return 2
    ros_dir = Path.home() / ".ros"
    ros_dir.mkdir(parents=True, exist_ok=True)
    pid_path = ros_dir / "h1-rviz-launcher.pid"
    if action == "close":
        closed = close_managed_rviz(pid_path)
        with (ros_dir / "h1-rviz-launcher.log").open("a") as log:
            log.write(f"Managed RViz close requested; closed={closed}\n")
        return 0 if closed else 1

    with (ros_dir / "h1-rviz-launcher.lock").open("a") as lock:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            activated = raise_rviz_window()
            with (ros_dir / "h1-rviz-launcher.log").open("a") as log:
                log.write(f"RViz already running; activation requested={activated}\n")
            return 0
        script = Path(__file__).resolve().with_name("ai_pc_start_rviz.sh")
        with (ros_dir / "h1-rviz-launcher.log").open("a") as log:
            process = subprocess.Popen(
                ["/bin/bash", str(script), mode, backend], stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True,
            )
            pid_path.write_text(f"{process.pid}\n", encoding="ascii")
            try:
                return process.wait()
            finally:
                try:
                    if pid_path.read_text(encoding="ascii").strip() == str(process.pid):
                        pid_path.unlink()
                except OSError:
                    pass


if __name__ == "__main__":
    sys.exit(main())
