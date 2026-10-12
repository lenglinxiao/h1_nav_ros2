#!/usr/bin/env python3
"""Read-only sampler for correlating PC4 web heartbeats with TCP and load."""

import argparse
import datetime
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from urllib.error import URLError
from urllib.parse import urlencode
from urllib.request import urlopen


def find_web_pid():
    """Find the current user's web GUI process once, without reading env data."""
    proc_root = Path("/proc")
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        try:
            if entry.stat().st_uid != os.geteuid():
                continue
            comm = (entry / "comm").read_text().strip()
            if comm != "h1_2_nav_web_gu":
                continue
            return pid
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
            continue
    return None


def read_process_cpu(pid):
    if pid is None:
        return None
    try:
        raw = Path(f"/proc/{pid}/stat").read_text()
        # comm may contain spaces or parentheses; fields after its final ')' are stable.
        fields = raw[raw.rfind(")") + 2 :].split()
        return {"utime_ticks": int(fields[11]), "stime_ticks": int(fields[12])}
    except (FileNotFoundError, PermissionError, IndexError, ValueError, OSError):
        return None


def read_cpu_counters():
    try:
        line = Path("/proc/stat").read_text().splitlines()[0]
        values = line.split()
        return [int(value) for value in values[1:]] if values[0] == "cpu" else None
    except (OSError, ValueError, IndexError):
        return None


def read_iface_counters(interface):
    try:
        for line in Path("/proc/net/dev").read_text().splitlines()[2:]:
            name, raw_counts = line.split(":", 1)
            if name.strip() == interface:
                values = [int(value) for value in raw_counts.split()]
                return {
                    "rx_bytes": values[0], "rx_packets": values[1],
                    "rx_errs": values[2], "rx_drop": values[3],
                    "tx_bytes": values[8], "tx_packets": values[9],
                    "tx_errs": values[10], "tx_drop": values[11],
                }
    except (OSError, ValueError, IndexError):
        pass
    return None


def read_ss_raw(http_port):
    try:
        result = subprocess.run(
            ["ss", "-tin", "sport", "=", f":{http_port + 1}"],
            capture_output=True, text=True, timeout=0.5, check=False,
        )
        return result.stdout.strip() or result.stderr.strip() or f"ss exit={result.returncode}"
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError) as exc:
        return f"ss unavailable: {type(exc).__name__}"


def is_relevant_log(entry):
    source = str(entry.get("source", "")).lower()
    message = str(entry.get("message", ""))
    if source == "web":
        return bool(re.search(r"heartbeat|navigation|stopping|stop", message, re.I))
    if source in {"nav", "navigation", "stop"}:
        # Do not persist the command line, which may contain user task details.
        if message.startswith("$ "):
            return True
        return bool(re.search(r"exited code=|stderr:", message, re.I))
    return False


def scrub_log(entry):
    clean = {key: entry.get(key) for key in ("seq", "time", "source", "message")}
    message = str(clean.get("message", ""))
    if message.startswith("$ "):
        source = str(clean.get("source", "")).lower()
        clean["message"] = "stop process launched" if source == "stop" else "navigation process launched"
    return clean


def poll_logs(port, after):
    query = urlencode({"after": after})
    url = f"http://127.0.0.1:{port}/api/logs?{query}"
    try:
        with urlopen(url, timeout=0.5) as response:
            payload = json.loads(response.read().decode("utf-8"))
        entries = payload.get("entries", [])
        selected = []
        for entry in entries:
            try:
                after = max(after, int(entry.get("seq", after)))
            except (TypeError, ValueError):
                continue
            if is_relevant_log(entry):
                selected.append(scrub_log(entry))
        return after, selected, None
    except (URLError, TimeoutError, OSError, ValueError, json.JSONDecodeError) as exc:
        return after, [], type(exc).__name__


def latest_log_seq(port):
    """Set a capture baseline without persisting entries from before startup."""
    query = urlencode({"after": 0})
    try:
        with urlopen(f"http://127.0.0.1:{port}/api/logs?{query}", timeout=0.5) as response:
            payload = json.loads(response.read().decode("utf-8"))
        return max((int(entry.get("seq", 0)) for entry in payload.get("entries", [])), default=0)
    except (URLError, TimeoutError, OSError, ValueError, json.JSONDecodeError):
        return 0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=900, help="capture duration in seconds (default: 900)")
    parser.add_argument("--output", help="new JSONL output path; default: /tmp/pc4_web_disconnect_<timestamp>.jsonl")
    parser.add_argument("--http-port", type=int, default=8765)
    parser.add_argument("--interface", default="wlxf8ce21a12bfe")
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be finite and positive")
    if not 1 <= args.http_port <= 65535:
        parser.error("--http-port must be in 1..65535")
    if not re.fullmatch(r"[A-Za-z0-9_.:-]{1,64}", args.interface):
        parser.error("invalid interface name")
    if args.output is None:
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"/tmp/pc4_web_disconnect_{stamp}.jsonl"
    return args


def main(argv=None):
    args = parse_args(argv)
    output = Path(args.output)
    output.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    os.fchmod(fd, 0o600)
    web_pid = find_web_pid()
    last_seq = latest_log_seq(args.http_port)
    started = time.monotonic()
    deadline = started + args.duration
    next_sample = started
    samples = 0
    try:
        with os.fdopen(fd, "w", encoding="utf-8", buffering=1) as stream:
            while time.monotonic() < deadline:
                now_mono = time.monotonic()
                record = {
                    "local_walltime": datetime.datetime.now().astimezone().isoformat(timespec="milliseconds"),
                    "monotonic": now_mono,
                    "loadavg": None,
                    "cpu_counters": read_cpu_counters(),
                    "web_pid": web_pid,
                    "web_cpu_ticks": read_process_cpu(web_pid),
                    "interface": args.interface,
                    "interface_counters": read_iface_counters(args.interface),
                    "ss_raw": read_ss_raw(args.http_port),
                    "web_logs": [],
                    "web_log_error": None,
                }
                try:
                    record["loadavg"] = [float(value) for value in Path("/proc/loadavg").read_text().split()[:3]]
                except (OSError, ValueError):
                    pass
                if web_pid is None or record["web_cpu_ticks"] is None:
                    replacement_pid = find_web_pid()
                    if replacement_pid != web_pid:
                        web_pid = replacement_pid
                        last_seq = 0
                        record["web_pid"] = web_pid
                        record["web_cpu_ticks"] = read_process_cpu(web_pid)
                last_seq, entries, error = poll_logs(args.http_port, last_seq)
                record["web_logs"] = entries
                record["web_log_error"] = error
                stream.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
                samples += 1
                next_sample += 1.0
                delay = next_sample - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_sample = time.monotonic()
    except KeyboardInterrupt:
        return 130
    finally:
        os.chmod(output, 0o600)
    print(f"captured {samples} samples to {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
