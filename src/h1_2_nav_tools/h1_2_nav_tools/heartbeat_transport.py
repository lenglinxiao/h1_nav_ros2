"""Browser heartbeat transport based on the standard websockets server library."""

import asyncio
import json
import threading
import time
from urllib.parse import urlsplit


PING_INTERVAL_SEC = 0.5
PING_TIMEOUT_SEC = 2.0
SLOW_PONG_RTT_SEC = 0.5
SLOW_PONG_LOG_INTERVAL_SEC = 5.0


def _header(headers, name):
    try:
        return headers.get(name)
    except AttributeError:
        return None


def _origin_matches(origin, host_header, http_port):
    """Allow only same-host browser origins served by the paired HTTP port."""
    if not origin or not host_header:
        return False
    try:
        parsed = urlsplit(origin)
        if parsed.scheme not in ("http", "https") or not parsed.hostname:
            return False
        origin_port = parsed.port or (443 if parsed.scheme == "https" else 80)
        if origin_port != http_port:
            return False
        host = host_header.rsplit(":", 1)
        if len(host) != 2 or int(host[1]) != http_port + 1:
            return False
        return host[0].strip("[]").lower() == parsed.hostname.lower()
    except (ValueError, TypeError):
        return False


class HeartbeatWebSocketServer:
    """Own an asyncio websocket server on a dedicated, cleanly stopped thread."""

    def __init__(self, host, port, http_port, app, logger=print):
        self.host = host
        self.port = port
        self.http_port = http_port
        self.app = app
        self.logger = logger
        self.loop = None
        self.server = None
        self.thread = None
        self.listener_ready = threading.Event()
        self.start_error = None
        self.stopping = threading.Event()

    async def _process_request(self, *args):
        # websockets <= 13: (path, request_headers); >= 14: (connection, request).
        request = args[-1]
        headers = getattr(request, "headers", request)
        origin = _header(headers, "Origin")
        host = _header(headers, "Host")
        if _origin_matches(origin, host, self.http_port):
            return None
        reason = b"WebSocket Origin must match the web GUI host and port."
        if len(args) == 2 and not isinstance(args[0], str):
            connection = args[0]
            return connection.respond(403, reason.decode("ascii"))
        from http import HTTPStatus
        return HTTPStatus.FORBIDDEN, [("Content-Type", "text/plain")], reason

    async def _handler(self, websocket, *_legacy_path):
        first_pong = True
        connected_at = time.monotonic()
        last_successful_pong = None
        last_rtt = None
        last_slow_rtt_log = float("-inf")
        peer = getattr(websocket, "remote_address", None)
        self.logger(f"WebSocket heartbeat connected: peer={peer!r}")
        pong_queue = asyncio.Queue()
        pending_pongs = set()

        async def send_pings():
            next_ping_at = time.monotonic()
            while not self.stopping.is_set():
                delay = next_ping_at - time.monotonic()
                if delay > 0:
                    await asyncio.sleep(delay)
                if len(pending_pongs) >= 4:
                    next_ping_at += PING_INTERVAL_SEC
                    continue
                sent_at = time.monotonic()
                try:
                    pong = await websocket.ping()
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    pong_queue.put_nowait(("sender_error", exc))
                    return
                pending_pongs.add(pong)
                def on_pong_done(done, timestamp=sent_at):
                    if done.cancelled():
                        pong_queue.put_nowait(("failed_pong", done, timestamp, None))
                        return
                    try:
                        done.result()
                    except Exception:
                        pong_queue.put_nowait(("failed_pong", done, timestamp, None))
                        return
                    pong_at = time.monotonic()
                    self.app.record_heartbeat()
                    pong_queue.put_nowait(("pong", done, timestamp, pong_at))

                pong.add_done_callback(on_pong_done)
                next_ping_at = max(next_ping_at + PING_INTERVAL_SEC,
                                   sent_at + PING_INTERVAL_SEC)

        try:
            timeout_sec = getattr(self.app, "heartbeat_timeout_sec", PING_TIMEOUT_SEC)
            heartbeat_reference = connected_at
            sender = asyncio.create_task(send_pings(), name="heartbeat-ping-sender")
            while not self.stopping.is_set():
                try:
                    event = pong_queue.get_nowait()
                except asyncio.QueueEmpty:
                    pong_age = time.monotonic() - heartbeat_reference
                    if pong_age >= timeout_sec:
                        raise TimeoutError("no Pong within heartbeat timeout")
                    try:
                        event = await asyncio.wait_for(
                            pong_queue.get(), timeout=timeout_sec - pong_age
                        )
                    except asyncio.TimeoutError:
                        raise TimeoutError("no Pong within heartbeat timeout")
                if event[0] == "sender_error":
                    raise event[1]
                if event[0] == "failed_pong":
                    pending_pongs.discard(event[1])
                    continue
                _, pong, ping_sent_at, pong_at = event
                pending_pongs.discard(pong)
                last_rtt = pong_at - ping_sent_at
                last_successful_pong = pong_at
                heartbeat_reference = pong_at
                if (last_rtt > SLOW_PONG_RTT_SEC and
                        pong_at - last_slow_rtt_log >= SLOW_PONG_LOG_INTERVAL_SEC):
                    self.logger(f"WebSocket heartbeat slow Pong: peer={peer!r} "
                                f"rtt={last_rtt:.3f}s")
                    last_slow_rtt_log = pong_at
                if first_pong:
                    await websocket.send(json.dumps({"type": "ready"}))
                    first_pong = False
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            now = time.monotonic()
            elapsed = now - connected_at
            pong_age = (now - last_successful_pong
                        if last_successful_pong is not None else None)
            pong_age_text = f"{pong_age:.3f}s" if pong_age is not None else "never"
            rtt_text = f"{last_rtt:.3f}s" if last_rtt is not None else "n/a"
            self.logger(
                f"WebSocket heartbeat disconnected: peer={peer!r} "
                f"error={type(exc).__name__} detail={str(exc)!r} "
                f"elapsed={elapsed:.3f}s last_pong_age={pong_age_text} "
                f"last_pong_rtt={rtt_text}"
            )
        finally:
            sender_task = locals().get("sender")
            if sender_task is not None:
                sender_task.cancel()
                await asyncio.gather(sender_task, return_exceptions=True)
            for pong in pending_pongs:
                if not pong.done():
                    pong.cancel()
            if pending_pongs:
                await asyncio.gather(*pending_pongs, return_exceptions=True)

    async def _serve(self):
        try:
            try:
                from websockets.legacy.server import serve
            except ImportError:
                from websockets.server import serve
            self.server = await serve(
                self._handler,
                self.host,
                self.port,
                process_request=self._process_request,
                ping_interval=None,
                ping_timeout=PING_TIMEOUT_SEC,
                max_size=1024,
                max_queue=1,
            )
            self.listener_ready.set()
            await self.server.wait_closed()
        except Exception as exc:
            self.start_error = exc
            self.listener_ready.set()
            self.logger(f"WebSocket heartbeat server failed: {exc}")

    def _thread_main(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._serve())
        finally:
            pending = asyncio.all_tasks(self.loop)
            for task in pending:
                task.cancel()
            if pending:
                self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            self.loop.close()

    def start(self, timeout=5.0):
        self.thread = threading.Thread(target=self._thread_main, name="web-heartbeat-ws", daemon=True)
        self.thread.start()
        if not self.listener_ready.wait(timeout):
            raise RuntimeError("WebSocket heartbeat server startup timed out")
        if self.start_error:
            raise RuntimeError(f"WebSocket heartbeat server failed: {self.start_error}")

    def shutdown(self):
        self.stopping.set()
        if self.loop and self.server:
            async def close_server():
                self.server.close()
                await self.server.wait_closed()
            future = asyncio.run_coroutine_threadsafe(close_server(), self.loop)
            try:
                future.result(timeout=3.0)
            except Exception as exc:
                self.logger(f"WebSocket heartbeat shutdown issue: {exc}")
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=3.0)
