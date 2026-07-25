import argparse
import sys

import rclpy
from h1_2_nav_msgs.srv import DeleteMemoryPoint, ListMemoryPoints, SaveMemoryPoint
from rclpy.node import Node


class ServiceCli(Node):
    def __init__(self):
        super().__init__("memory_point_cli")

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
    parser = argparse.ArgumentParser(description="Manage named navigation memory points.")
    parser.add_argument("--timeout", type=float, default=3.0, help="Service wait/call timeout in seconds.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    save = subparsers.add_parser("save", help="Save current robot pose as a memory point.")
    save.add_argument("name")
    save.add_argument("--overwrite", action="store_true")
    save.add_argument("--service", default="save_memory_point")

    delete = subparsers.add_parser("delete", help="Delete a memory point.")
    delete.add_argument("name")
    delete.add_argument("--service", default="delete_memory_point")

    list_cmd = subparsers.add_parser("list", help="List memory points.")
    list_cmd.add_argument("--service", default="list_memory_points")
    return parser


def run(args, node):
    if args.command == "save":
        request = SaveMemoryPoint.Request()
        request.name = args.name
        request.overwrite = args.overwrite
        response = node.call(SaveMemoryPoint, args.service, request, args.timeout)
        print(response.message)
        if response.success:
            p = response.pose.pose.position
            print(
                f"{args.name}: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, "
                f"frame={response.pose.header.frame_id}"
            )
        return 0 if response.success else 1

    if args.command == "delete":
        request = DeleteMemoryPoint.Request()
        request.name = args.name
        response = node.call(DeleteMemoryPoint, args.service, request, args.timeout)
        print(response.message)
        return 0 if response.success else 1

    if args.command == "list":
        response = node.call(ListMemoryPoints, args.service, ListMemoryPoints.Request(), args.timeout)
        print(response.message)
        for name, pose in zip(response.names, response.poses):
            p = pose.pose.position
            print(f"{name}: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, frame={pose.header.frame_id}")
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
