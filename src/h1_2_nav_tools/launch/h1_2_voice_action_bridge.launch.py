from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("voice_command_topic", default_value="/h1_2/voice_command"),
            DeclareLaunchArgument("voice_feedback_topic", default_value="/h1_2/voice_feedback"),
            DeclareLaunchArgument("cmd_topic", default_value="/cmd_vel_voice"),
            DeclareLaunchArgument("gesture_service_name", default_value="/h1_2_loco/execute_gesture"),
            DeclareLaunchArgument("memory_points_service_name", default_value="list_memory_points"),
            DeclareLaunchArgument("navigation_action_name", default_value="navigate_to_pose"),
            DeclareLaunchArgument("forward_speed", default_value="0.12"),
            DeclareLaunchArgument("turn_yaw_speed", default_value="0.30"),
            DeclareLaunchArgument("publish_rate", default_value="10.0"),
            DeclareLaunchArgument("continuous_move", default_value="true"),
            Node(
                package="h1_2_nav_tools",
                executable="h1_2_voice_action_bridge",
                name="h1_2_voice_action_bridge",
                output="screen",
                parameters=[
                    {
                        "voice_command_topic": LaunchConfiguration("voice_command_topic"),
                        "voice_feedback_topic": LaunchConfiguration("voice_feedback_topic"),
                        "cmd_topic": LaunchConfiguration("cmd_topic"),
                        "gesture_service_name": LaunchConfiguration("gesture_service_name"),
                        "memory_points_service_name": LaunchConfiguration(
                            "memory_points_service_name"
                        ),
                        "navigation_action_name": LaunchConfiguration(
                            "navigation_action_name"
                        ),
                        "forward_speed": ParameterValue(
                            LaunchConfiguration("forward_speed"), value_type=float
                        ),
                        "turn_yaw_speed": ParameterValue(
                            LaunchConfiguration("turn_yaw_speed"), value_type=float
                        ),
                        "publish_rate": ParameterValue(
                            LaunchConfiguration("publish_rate"), value_type=float
                        ),
                        "continuous_move": ParameterValue(
                            LaunchConfiguration("continuous_move"), value_type=bool
                        ),
                        "code_stop": [1, 2],
                        "code_forward": [4],
                        "code_squat": [5],
                        "code_stand": [6],
                        "code_right": [7],
                        "code_nav_a": [19],
                        "code_nav_b": [20],
                        "code_nav_c": [21],
                        "code_nav_start": [33],
                        "code_reach_hand": [45],
                        "code_retract_hand": [46],
                        "code_wave": [52],
                    }
                ],
            ),
        ]
    )
