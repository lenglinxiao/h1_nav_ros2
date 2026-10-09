#!/usr/bin/env python3
"""Handle a fixed local RViz URI; never execute commands supplied by a URL."""
import fcntl
import ctypes
import ctypes.util
from pathlib import Path
import re
import subprocess
import sys


def parse_mode(uri):
    allowed = {
        "h1-rviz://open?mode=online": "online",
        "h1-rviz://open?mode=offline": "offline",
    }
    if uri not in allowed:
        raise ValueError("Unsupported RViz link")
    return allowed[uri]


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
        mode = parse_mode(sys.argv[1])
    except ValueError:
        return 2
    ros_dir = Path.home() / ".ros"
    ros_dir.mkdir(parents=True, exist_ok=True)
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
            return subprocess.run(
                ["/bin/bash", str(script), mode], stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True,
            ).returncode


if __name__ == "__main__":
    sys.exit(main())
