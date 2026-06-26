import threading

from h1_2_nav_msgs.srv import RouteStatus
from rclpy.callback_groups import ReentrantCallbackGroup


INACTIVE_ROUTE_STATES = {"", "idle", "blocked"}


class RouteActivityGuard:
    def __init__(self, node, status_service_name, peer_label, timeout_sec=0.5):
        self.node = node
        self.status_service_name = str(status_service_name or "").strip()
        self.peer_label = peer_label
        self.timeout_sec = max(0.05, float(timeout_sec))
        self.client = None
        if self.status_service_name:
            self.client = node.create_client(
                RouteStatus,
                self.status_service_name,
                callback_group=ReentrantCallbackGroup(),
            )

    def check_peer_inactive(self):
        if self.client is None:
            return True, ""
        if not self.client.wait_for_service(timeout_sec=self.timeout_sec):
            return True, f"{self.peer_label} status service is not available; continuing."

        future = self.client.call_async(RouteStatus.Request())
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self.timeout_sec):
            return False, f"{self.peer_label} status check timed out; refusing to start route."

        try:
            result = future.result()
        except Exception as exc:
            return False, f"{self.peer_label} status check failed: {exc}"

        state = str(result.state or "").strip()
        if state in INACTIVE_ROUTE_STATES:
            return True, f"{self.peer_label} is inactive: {state or 'unknown'}"
        return False, f"{self.peer_label} is active: state={state}, current_goal={result.current_goal}"
