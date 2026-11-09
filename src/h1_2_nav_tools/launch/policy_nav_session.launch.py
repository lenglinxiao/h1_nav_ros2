"""Launch an isolated Nav2 action and velocity session for policy control."""

import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    backend_namespace = LaunchConfiguration("backend_namespace")
    cmd_topic = LaunchConfiguration("cmd_topic")
    raw_cmd_topic = LaunchConfiguration("raw_cmd_topic")
    priority_cmd_topic = LaunchConfiguration("priority_cmd_topic")
    nav2_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"
    ])

    action_names = ("navigate_to_pose", "navigate_through_poses")
    action_endpoints = ("send_goal", "get_result", "cancel_goal", "feedback", "status")
    backend_remaps = []
    for name in action_names:
        backend_remaps.append(SetRemap(src=f"/{name}", dst=[backend_namespace, "/", name]))
        for endpoint in action_endpoints:
            backend_remaps.append(SetRemap(
                src=f"/{name}/_action/{endpoint}",
                dst=[backend_namespace, f"/{name}/_action/{endpoint}"],
            ))

    def validate_session(context):
        backend = backend_namespace.perform(context)
        topic = cmd_topic.perform(context)
        raw_topic = raw_cmd_topic.perform(context)
        priority_topic = priority_cmd_topic.perform(context)
        session = re.fullmatch(
            r"(/h1_2/policy/session/o_[0-9a-f]{32}/c_[0-9a-f]{32}/e_[0-9]+)",
            topic.removesuffix("/cmd_vel"),
        )
        if session is None or topic != session.group(1) + "/cmd_vel":
            raise RuntimeError("cmd_topic must be a complete coordinator-issued policy session topic")
        prefix = session.group(1)
        if backend != prefix + "/backend":
            raise RuntimeError("backend_namespace must match the command topic session token")
        if raw_topic != prefix + "/raw_cmd_vel" or priority_topic != prefix + "/priority_cmd_vel":
            raise RuntimeError("velocity topics must use the exact command session namespace")
        if LaunchConfiguration("use_composition").perform(context).lower() == "true":
            raise RuntimeError("use_composition=true is unsupported because session remaps need separate processes")
        return []

    return LaunchDescription([
        DeclareLaunchArgument("backend_namespace"),
        DeclareLaunchArgument("cmd_topic"),
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_composition", default_value="false"),
        DeclareLaunchArgument(
            "raw_cmd_topic",
            default_value=PythonExpression(["'", cmd_topic, "'.rsplit('/', 1)[0] + '/raw_cmd_vel'"]),
        ),
        DeclareLaunchArgument(
            "priority_cmd_topic",
            default_value=PythonExpression(["'", cmd_topic, "'.rsplit('/', 1)[0] + '/priority_cmd_vel'"]),
        ),
        OpaqueFunction(function=validate_session),
        GroupAction(actions=[
            *backend_remaps,
            SetRemap(src="/cmd_vel", dst=LaunchConfiguration("raw_cmd_topic")),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": LaunchConfiguration("params_file"),
                    "autostart": LaunchConfiguration("autostart"),
                    "use_composition": LaunchConfiguration("use_composition"),
                    "map_subscribe_transient_local": "true",
                }.items(),
            ),
            Node(
                package="h1_2_nav_tools",
                executable="velocity_smoother_ema",
                name="velocity_smoother_ema",
                output="screen",
                parameters=[{
                    "raw_cmd_topic": raw_cmd_topic,
                    "priority_cmd_topic": priority_cmd_topic,
                    "cmd_topic": cmd_topic,
                }],
            ),
        ]),
    ])
