import argparse
import sys

import rclpy
from h1_2_nav_msgs.srv import RouteStatus, StartRoute, StopRoute
from rclpy.node import Node


class ServiceCli(Node):
    def __init__(self):
        super().__init__("route_runner_cli")

    def call(self, service_type, service_name, request, timeout_sec):
        client = self.create_client(service_type, service_name)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service is not available: {service_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise TimeoutError(f"Service call timed out: {service_name}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"Service call failed: {service_name}")
        return result


def build_parser():
    parser = argparse.ArgumentParser(description="Control the memory-point route runner.")
    parser.add_argument("--timeout", type=float, default=3.0, help="Service wait/call timeout in seconds.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    start = subparsers.add_parser("start", help="Start looping the configured memory route.")
    start.add_argument("--service", default="start_memory_route")

    stop = subparsers.add_parser("stop", help="Stop the active memory route.")
    stop.add_argument("--service", default="stop_memory_route")

    status = subparsers.add_parser("status", help="Show route-runner status.")
    status.add_argument("--service", default="memory_route_status")
    return parser


def run(args, node):
    if args.command == "start":
        response = node.call(StartRoute, args.service, StartRoute.Request(), args.timeout)
        print(response.message)
        return 0 if response.success else 1

    if args.command == "stop":
        response = node.call(StopRoute, args.service, StopRoute.Request(), args.timeout)
        print(response.message)
        return 0 if response.success else 1

    if args.command == "status":
        response = node.call(RouteStatus, args.service, RouteStatus.Request(), args.timeout)
        print(f"state={response.state}")
        print(f"current_goal={response.current_goal}")
        print(f"current_index={response.current_index}")
        print(response.message)
        return 0 if response.success else 1

    raise RuntimeError(f"Unsupported command: {args.command}")


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    rclpy.init(args=None)
    node = ServiceCli()
    try:
        return run(args, node)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
