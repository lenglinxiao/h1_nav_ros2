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
            DeclareLaunchArgument("loop", default_value="true"),
            DeclareLaunchArgument("gesture_service_name", default_value="/h1_2_loco/execute_gesture"),
            DeclareLaunchArgument("voice_feedback_topic", default_value="/h1_2/voice_feedback"),
            DeclareLaunchArgument("stop_cmd_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("frame_id", default_value="map"),
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
                        "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
                        "gesture_service_name": LaunchConfiguration("gesture_service_name"),
                        "voice_feedback_topic": LaunchConfiguration("voice_feedback_topic"),
                        "stop_cmd_topic": LaunchConfiguration("stop_cmd_topic"),
                        "frame_id": LaunchConfiguration("frame_id"),
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
