"""Loopback-only checks for browser Ping/Pong heartbeat transport."""

import asyncio
import json
import socket
import sys
import threading
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/h1_2_nav_tools"))

try:
    import websockets
    from websockets.legacy.client import WebSocketClientProtocol
except ImportError:
    websockets = None
    WebSocketClientProtocol = object
from h1_2_nav_tools.heartbeat_transport import HeartbeatWebSocketServer, _origin_matches


class FakeApp:
    def __init__(self):
        self.count = 0
        self.lock = threading.Lock()

    def record_heartbeat(self):
        with self.lock:
            self.count += 1


def free_port():
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        return listener.getsockname()[1]


def test_origin_requires_matching_host_and_http_port():
    assert _origin_matches("http://127.0.0.1:8765", "127.0.0.1:8766", 8765)
    assert not _origin_matches(None, "127.0.0.1:8766", 8765)
    assert not _origin_matches("http://evil.example:8765", "127.0.0.1:8766", 8765)
    assert not _origin_matches("http://127.0.0.1:8764", "127.0.0.1:8766", 8765)


@pytest.mark.skipif(websockets is None, reason="websockets package unavailable")
def test_loopback_client_receives_ready_only_after_server_ping_pong():
    http_port = free_port()
    ws_port = http_port + 1
    app = FakeApp()
    server = HeartbeatWebSocketServer("127.0.0.1", ws_port, http_port, app, logger=lambda _: None)
    server.start()
    try:
        async def receive_ready():
            async with websockets.connect(
                f"ws://127.0.0.1:{ws_port}",
                origin=f"http://127.0.0.1:{http_port}",
            ) as client:
                return json.loads(await asyncio.wait_for(client.recv(), timeout=2.0))
        assert asyncio.run(receive_ready()) == {"type": "ready"}
        # A reconnecting browser must receive its own ready message too.
        assert asyncio.run(receive_ready()) == {"type": "ready"}
        assert app.count >= 2
    finally:
        server.shutdown()


@pytest.mark.skipif(websockets is None, reason="websockets package unavailable")
def test_loopback_rejects_missing_and_cross_host_origins():
    http_port = free_port()
    ws_port = http_port + 1
    server = HeartbeatWebSocketServer("127.0.0.1", ws_port, http_port, FakeApp(), logger=lambda _: None)
    server.start()
    try:
        async def assert_rejected(origin):
            with pytest.raises(websockets.exceptions.InvalidStatusCode):
                async with websockets.connect(f"ws://127.0.0.1:{ws_port}", origin=origin):
                    pass
        for origin in (None, f"http://example.invalid:{http_port}"):
            asyncio.run(assert_rejected(origin))
    finally:
        server.shutdown()


@pytest.mark.skipif(websockets is None, reason="websockets package unavailable")
def test_idle_browser_receives_repeated_ping_pong_updates_past_watchdog_timeout():
    http_port = free_port()
    ws_port = http_port + 1
    app = FakeApp()
    server = HeartbeatWebSocketServer("127.0.0.1", ws_port, http_port, app, logger=lambda _: None)
    server.start()
    try:
        async def stay_idle():
            async with websockets.connect(
                f"ws://127.0.0.1:{ws_port}", origin=f"http://127.0.0.1:{http_port}",
            ) as client:
                assert json.loads(await client.recv()) == {"type": "ready"}
                await asyncio.sleep(2.2)
        asyncio.run(stay_idle())
        assert app.count >= 4
    finally:
        server.shutdown()


@pytest.mark.skipif(websockets is None, reason="websockets package unavailable")
def test_delayed_pongs_do_not_delay_fixed_ping_cadence(monkeypatch):
    from websockets.legacy.server import WebSocketServerProtocol

    http_port = free_port()
    ws_port = http_port + 1
    app = FakeApp()
    ping_times = []
    original_ping = WebSocketServerProtocol.ping

    async def record_ping(protocol, data=None):
        ping_times.append(asyncio.get_running_loop().time())
        return await original_ping(protocol, data)

    class DelayedPongProtocol(WebSocketClientProtocol):
        async def pong(self, data=b""):
            await asyncio.sleep(0.7)
            await super().pong(data)

    monkeypatch.setattr(WebSocketServerProtocol, "ping", record_ping)
    server = HeartbeatWebSocketServer("127.0.0.1", ws_port, http_port, app, logger=lambda _: None)
    server.start()
    try:
        async def stay_connected():
            async with websockets.connect(
                f"ws://127.0.0.1:{ws_port}", origin=f"http://127.0.0.1:{http_port}",
                create_protocol=DelayedPongProtocol, ping_interval=None,
            ) as client:
                assert json.loads(await client.recv()) == {"type": "ready"}
                await asyncio.sleep(1.7)

        asyncio.run(stay_connected())
        intervals = [later - earlier for earlier, later in zip(ping_times, ping_times[1:])]
        assert len(intervals) >= 3
        assert max(intervals) <= 0.65
        assert app.count >= 2
    finally:
        server.shutdown()


@pytest.mark.skipif(websockets is None, reason="websockets package unavailable")
def test_client_that_does_not_pong_is_closed_without_refreshing_heartbeat():
    class NoPongProtocol(WebSocketClientProtocol):
        async def pong(self, data=b""):
            return

    http_port = free_port()
    ws_port = http_port + 1
    app = FakeApp()
    logs = []
    disconnected = threading.Event()

    def record_log(message):
        logs.append(message)
        if "WebSocket heartbeat disconnected:" in message:
            disconnected.set()

    server = HeartbeatWebSocketServer("127.0.0.1", ws_port, http_port, app, logger=record_log)
    server.start()
    try:
        async def ignore_pings():
            async with websockets.connect(
                f"ws://127.0.0.1:{ws_port}", origin=f"http://127.0.0.1:{http_port}",
                create_protocol=NoPongProtocol, ping_interval=None,
            ) as client:
                await asyncio.wait_for(client.wait_closed(), timeout=3.0)
                return client.close_code
        assert asyncio.run(ignore_pings()) is not None
        assert app.count == 0
        assert disconnected.wait(1.0)
        assert any("WebSocket heartbeat connected: peer=" in message for message in logs)
        disconnect_log = next(message for message in logs
                              if "WebSocket heartbeat disconnected:" in message)
        assert "error=TimeoutError" in disconnect_log
        assert "elapsed=" in disconnect_log
        assert "last_pong_age=never" in disconnect_log
        assert "last_pong_rtt=n/a" in disconnect_log

        async def no_sender_task_remains():
            return [task.get_name() for task in asyncio.all_tasks()
                    if task.get_name() == "heartbeat-ping-sender"]

        for _ in range(20):
            names = asyncio.run_coroutine_threadsafe(no_sender_task_remains(), server.loop).result()
            if not names:
                break
            threading.Event().wait(0.01)
        assert names == []
    finally:
        server.shutdown()


@pytest.mark.skipif(websockets is None, reason="websockets package unavailable")
def test_websocket_port_conflict_fails_startup_closed():
    http_port = free_port()
    ws_port = http_port + 1
    with socket.socket() as occupied:
        occupied.bind(("127.0.0.1", ws_port))
        occupied.listen()
        server = HeartbeatWebSocketServer("127.0.0.1", ws_port, http_port, FakeApp(), logger=lambda _: None)
        with pytest.raises(RuntimeError, match="WebSocket heartbeat server failed"):
            server.start()
        server.shutdown()


def test_handler_logs_ping_timeout_with_elapsed_and_last_pong_age(monkeypatch):
    import h1_2_nav_tools.heartbeat_transport as transport

    class NeverPongSocket:
        remote_address = ("127.0.0.1", 12345)

        async def ping(self):
            return asyncio.get_running_loop().create_future()

    logs = []
    app = FakeApp()
    server = HeartbeatWebSocketServer("127.0.0.1", 8766, 8765, app, logger=logs.append)
    monkeypatch.setattr(transport, "PING_TIMEOUT_SEC", 0.01)
    asyncio.run(server._handler(NeverPongSocket()))

    disconnect_log = next(message for message in logs
                          if "WebSocket heartbeat disconnected:" in message)
    assert "peer=('127.0.0.1', 12345)" in logs[0]
    assert "error=TimeoutError" in disconnect_log
    assert "elapsed=" in disconnect_log
    assert "last_pong_age=never" in disconnect_log
    assert "last_pong_rtt=n/a" in disconnect_log
    assert app.count == 0


def test_handler_propagates_ping_send_error_without_waiting_for_timeout():
    class BrokenSocket:
        remote_address = ("127.0.0.1", 12345)

        async def ping(self):
            raise RuntimeError("ping send failed")

    logs = []
    server = HeartbeatWebSocketServer("127.0.0.1", 8766, 8765, FakeApp(), logger=logs.append)

    async def run_handler():
        before = asyncio.get_running_loop().time()
        await server._handler(BrokenSocket())
        return asyncio.get_running_loop().time() - before

    elapsed = asyncio.run(run_handler())
    disconnect_log = next(message for message in logs
                          if "WebSocket heartbeat disconnected:" in message)
    assert "error=RuntimeError" in disconnect_log
    assert "detail='ping send failed'" in disconnect_log
    assert elapsed < 0.5
