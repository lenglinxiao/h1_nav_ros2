"""One-shot Fast DDS ROS client for the navigation Web GUI."""

import json
import sys
import time

import rclpy
from h1_2_icp_localization.msg import LocalizationStatus
from h1_2_nav_msgs.srv import RouteStatus, SaveMemoryPoint, StartRoute, StopRoute
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


TRIGGER_SERVICES = {
    "authorize_output": "/h1_2_policy_node/authorize_output",
    "fix_stand": "/h1_2_policy_node/fix_stand",
    "arm_hold_stand": "/h1_2_policy_node/arm_hold_stand",
    "arm_hold": "/h1_2_policy_node/arm_hold",
    "policy_status": "/h1_2_policy_node/status",
}
ROUTE_SERVICES = {
    "route_status": (RouteStatus, "memory_route_status", RouteStatus.Request),
    "route_start": (StartRoute, "start_memory_route", StartRoute.Request),
    "route_stop": (StopRoute, "stop_memory_route", StopRoute.Request),
}
SERVICE_DISCOVERY_TIMEOUT_SEC = 5.0
SERVICE_RESPONSE_TIMEOUT_SEC = 4.0


def _call(node, executor, service_type, name, request):
    client = node.create_client(service_type, name)
    if not client.wait_for_service(timeout_sec=SERVICE_DISCOVERY_TIMEOUT_SEC):
        return {"success": False, "message": f"{name} is not available"}
    future = client.call_async(request)
    deadline = time.monotonic() + SERVICE_RESPONSE_TIMEOUT_SEC
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    if not future.done():
        return {"success": False, "message": f"{name} timed out"}
    response = future.result()
    result = {"success": bool(response.success), "message": response.message}
    if hasattr(response, "state") and hasattr(response, "current_goal"):
        result.update({"state": response.state, "current_goal": response.current_goal,
                       "current_index": int(response.current_index)})
    if hasattr(response, "pose") and response.success:
        pose = response.pose
        result["pose"] = {
            "frame_id": pose.header.frame_id,
            "position": {"x": pose.pose.position.x, "y": pose.pose.position.y, "z": pose.pose.position.z},
            "orientation": {"x": pose.pose.orientation.x, "y": pose.pose.orientation.y,
                            "z": pose.pose.orientation.z, "w": pose.pose.orientation.w},
        }
    return result


def _policy_localization(node, executor, online_camera_init):
    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=False)
    samples = {"count": 0, "score": None, "message": ""}
    subscription = None
    if not online_camera_init:
        def on_status(message):
            stable = (message.initialized and message.converged and not message.degraded
                      and message.score <= 0.15 and message.message not in (
                          "manual initial pose accepted", "manual relocalize pose accepted"))
            samples["count"] = samples["count"] + 1 if stable else 0
            samples["score"] = float(message.score)
            samples["message"] = message.message
            samples["updated"] = time.monotonic()
        subscription = node.create_subscription(LocalizationStatus, "/icp_localization/status", on_status, 10)

    target, source = ("camera_init", "body") if online_camera_init else ("map", "camera_init")
    deadline = time.monotonic() + (0.8 if online_camera_init else 2.0)
    transform = None
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        try:
            transform = buffer.lookup_transform(target, source, Time())
            stamp = Time.from_msg(transform.header.stamp)
            age = max(0.0, (node.get_clock().now() - stamp).nanoseconds / 1e9)
            if stamp.nanoseconds > 0 and age <= 0.5:
                break
        except Exception:
            transform = None
    if online_camera_init:
        ready = transform is not None and stamp.nanoseconds > 0 and age <= 0.5
        return {"ready": ready, "frame_id": "camera_init", "age_sec": round(age, 3) if transform else None,
                "stable_samples": 0, "required_stable_samples": 0, "score": None,
                "message": (f"在线位姿可用（camera_init→body，{age:.2f}s）" if ready
                            else "等待新鲜在线位姿 camera_init→body")}

    status_age = time.monotonic() - samples.get("updated", 0.0)
    tf_ready = transform is not None
    status_ready = samples["count"] >= 5 and status_age <= 10.0
    ready = tf_ready and status_ready
    return {"ready": ready, "stable_samples": samples["count"], "required_stable_samples": 5,
            "score": samples["score"], "age_sec": round(status_age, 3),
            "message": (f"定位稳定，可开始巡航（score={samples['score']:.3f}，{samples['count']}/5）"
                        if ready else f"等待定位稳定或 map→camera_init TF（{samples['count']}/5）")}


def dispatch(node, executor, request):
    operation = request.get("operation", "")
    if operation in TRIGGER_SERVICES:
        return _call(node, executor, Trigger, TRIGGER_SERVICES[operation], Trigger.Request())
    if operation in ROUTE_SERVICES:
        service_type, service_name, request_type = ROUTE_SERVICES[operation]
        result = _call(node, executor, service_type, service_name, request_type())
        if operation == "route_status" and "state" not in result:
            result["state"] = "unavailable"
        return result
    if operation == "save_memory_point":
        message = SaveMemoryPoint.Request()
        message.name = str(request.get("name", "")).strip()
        message.overwrite = bool(request.get("overwrite", False))
        return _call(node, executor, SaveMemoryPoint, "save_memory_point", message)
    if operation == "localization":
        return _policy_localization(node, executor, bool(request.get("online_camera_init", False)))
    return {"success": False, "message": "unknown operation"}


def main():
    try:
        request = json.load(sys.stdin)
        rclpy.init(args=[])
        node = Node("h1_2_policy_web_client")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            print(json.dumps(dispatch(node, executor, request), ensure_ascii=False))
        finally:
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown()
    except Exception as exc:
        print(json.dumps({"success": False, "message": str(exc)}, ensure_ascii=False))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
