import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_prefix = get_package_prefix("h1_2_loco_bridge")
    daemon_path = os.path.join(package_prefix, "lib", "h1_2_loco_bridge", "h1_2_loco_udp_daemon")
    unitree_lib = "/home/llx/unitree_slam/unitree_robotics/lib/x86_64"
    daemon_env = {
        "LD_LIBRARY_PATH": unitree_lib + ":" + os.environ.get("LD_LIBRARY_PATH", ""),
        "ROS_LOCALHOST_ONLY": "0",
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument("udp_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("udp_port", default_value="39001"),
            DeclareLaunchArgument("enabled", default_value="false"),
            DeclareLaunchArgument("start_ros_shim", default_value="true"),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument("continuous_move", default_value="true"),
            DeclareLaunchArgument("network_interface", default_value=""),
            DeclareLaunchArgument("domain_id", default_value="0"),
            DeclareLaunchArgument("max_x", default_value="0.10"),
            DeclareLaunchArgument("max_y", default_value="0.0"),
            DeclareLaunchArgument("max_yaw", default_value="0.30"),
            DeclareLaunchArgument("timeout", default_value="0.4"),
            DeclareLaunchArgument("publish_rate", default_value="10.0"),
            DeclareLaunchArgument("command_duration", default_value="0.5"),
            ExecuteProcess(
                cmd=[
                    "/usr/bin/env",
                    "-u", "ROS_LOCALHOST_ONLY",
                    "-u", "CYCLONEDDS_URI",
                    daemon_path,
                    "--network-interface", LaunchConfiguration("network_interface"),
                    "--domain-id", LaunchConfiguration("domain_id"),
                    "--udp-port", LaunchConfiguration("udp_port"),
                    "--auto-start-value", LaunchConfiguration("auto_start"),
                    "--max-x", LaunchConfiguration("max_x"),
                    "--max-y", LaunchConfiguration("max_y"),
                    "--max-yaw", LaunchConfiguration("max_yaw"),
                ],
                output="screen",
                additional_env=daemon_env,
                condition=IfCondition(LaunchConfiguration("enabled")),
            ),
            Node(
                package="h1_2_loco_bridge",
                executable="cmd_vel_to_h1_2_loco",
                name="cmd_vel_to_h1_2_loco",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_ros_shim")),
                parameters=[
                    {
                        "cmd_topic": LaunchConfiguration("cmd_topic"),
                        "udp_host": LaunchConfiguration("udp_host"),
                        "udp_port": ParameterValue(LaunchConfiguration("udp_port"), value_type=int),
                        "enabled": ParameterValue(LaunchConfiguration("enabled"), value_type=bool),
                        "auto_start": ParameterValue(LaunchConfiguration("auto_start"), value_type=bool),
                        "continuous_move": ParameterValue(LaunchConfiguration("continuous_move"), value_type=bool),
                        "network_interface": LaunchConfiguration("network_interface"),
                        "domain_id": ParameterValue(LaunchConfiguration("domain_id"), value_type=int),
                        "max_x": ParameterValue(LaunchConfiguration("max_x"), value_type=float),
                        "max_y": ParameterValue(LaunchConfiguration("max_y"), value_type=float),
                        "max_yaw": ParameterValue(LaunchConfiguration("max_yaw"), value_type=float),
                        "timeout": ParameterValue(LaunchConfiguration("timeout"), value_type=float),
                        "publish_rate": ParameterValue(LaunchConfiguration("publish_rate"), value_type=float),
                        "command_duration": ParameterValue(LaunchConfiguration("command_duration"), value_type=float),
                    }
                ],
            ),
        ]
    )
