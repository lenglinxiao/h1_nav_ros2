from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_path_file = PathJoinSubstitution(
        [FindPackageShare("h1_2_nav_tools"), "path", "teaching_paths.txt"]
    )
    default_events_file = PathJoinSubstitution(
        [FindPackageShare("h1_2_nav_tools"), "config", "teaching_route_events.example.json"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("path_file", default_value=default_path_file),
            DeclareLaunchArgument("events_file", default_value=default_events_file),
            DeclareLaunchArgument("memory_points_file", default_value="maps/memory_points.json"),
            DeclareLaunchArgument("follow_path_action", default_value="follow_path"),
            DeclareLaunchArgument("controller_id", default_value="FollowPath"),
            DeclareLaunchArgument("control_mode", default_value="direct"),
            DeclareLaunchArgument("cmd_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("control_frequency", default_value="20.0"),
            DeclareLaunchArgument("tracking_linear_speed", default_value="0.08"),
            DeclareLaunchArgument("tracking_lookahead_dist", default_value="0.25"),
            DeclareLaunchArgument("tracking_goal_tolerance", default_value="0.18"),
            DeclareLaunchArgument("max_tracking_error", default_value="0.6"),
            DeclareLaunchArgument("heading_gain", default_value="1.4"),
            DeclareLaunchArgument("lateral_gain", default_value="0.8"),
            DeclareLaunchArgument("max_tracking_angular_speed", default_value="0.35"),
            DeclareLaunchArgument("loop", default_value="true"),
            DeclareLaunchArgument("gesture_service_name", default_value="/h1_2_loco/execute_gesture"),
            DeclareLaunchArgument("voice_feedback_topic", default_value="/h1_2/voice_feedback"),
            DeclareLaunchArgument("stop_cmd_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("robot_frame", default_value="body"),
            DeclareLaunchArgument("enable_start_pose_guard", default_value="true"),
            DeclareLaunchArgument("start_xy_tolerance", default_value="0.5"),
            DeclareLaunchArgument("start_yaw_tolerance", default_value="1.0"),
            DeclareLaunchArgument("start_tf_timeout_sec", default_value="0.2"),
            DeclareLaunchArgument("visualization_path_topic", default_value="/teaching_route_path"),
            DeclareLaunchArgument("start_service", default_value="start_teaching_route"),
            DeclareLaunchArgument("stop_service", default_value="stop_teaching_route"),
            DeclareLaunchArgument("status_service", default_value="teaching_route_status"),
            DeclareLaunchArgument("peer_status_service", default_value="memory_route_status"),
            Node(
                package="h1_2_nav_tools",
                executable="teaching_route_runner",
                name="teaching_route_runner",
                output="screen",
                parameters=[
                    {
                        "path_file": LaunchConfiguration("path_file"),
                        "events_file": LaunchConfiguration("events_file"),
                        "memory_points_file": LaunchConfiguration("memory_points_file"),
                        "follow_path_action": LaunchConfiguration("follow_path_action"),
                        "controller_id": LaunchConfiguration("controller_id"),
                        "control_mode": LaunchConfiguration("control_mode"),
                        "cmd_topic": LaunchConfiguration("cmd_topic"),
                        "control_frequency": LaunchConfiguration("control_frequency"),
                        "tracking_linear_speed": LaunchConfiguration("tracking_linear_speed"),
                        "tracking_lookahead_dist": LaunchConfiguration("tracking_lookahead_dist"),
                        "tracking_goal_tolerance": LaunchConfiguration("tracking_goal_tolerance"),
                        "max_tracking_error": LaunchConfiguration("max_tracking_error"),
                        "heading_gain": LaunchConfiguration("heading_gain"),
                        "lateral_gain": LaunchConfiguration("lateral_gain"),
                        "max_tracking_angular_speed": LaunchConfiguration("max_tracking_angular_speed"),
                        "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
                        "gesture_service_name": LaunchConfiguration("gesture_service_name"),
                        "voice_feedback_topic": LaunchConfiguration("voice_feedback_topic"),
                        "stop_cmd_topic": LaunchConfiguration("stop_cmd_topic"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "robot_frame": LaunchConfiguration("robot_frame"),
                        "enable_start_pose_guard": ParameterValue(
                            LaunchConfiguration("enable_start_pose_guard"), value_type=bool
                        ),
                        "start_xy_tolerance": LaunchConfiguration("start_xy_tolerance"),
                        "start_yaw_tolerance": LaunchConfiguration("start_yaw_tolerance"),
                        "start_tf_timeout_sec": LaunchConfiguration("start_tf_timeout_sec"),
                        "visualization_path_topic": LaunchConfiguration("visualization_path_topic"),
                        "start_service": LaunchConfiguration("start_service"),
                        "stop_service": LaunchConfiguration("stop_service"),
                        "status_service": LaunchConfiguration("status_service"),
                        "peer_status_service": LaunchConfiguration("peer_status_service"),
                    }
                ],
            ),
        ]
    )
