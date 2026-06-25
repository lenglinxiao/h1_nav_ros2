from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/myspeech"),
            DeclareLaunchArgument("voice_command_topic", default_value="/h1_2/voice_command"),
            DeclareLaunchArgument("voice_feedback_topic", default_value="/h1_2/voice_feedback"),
            DeclareLaunchArgument("poll_rate", default_value="10.0"),
            Node(
                package="h1_2_voice_control",
                executable="h1_2_voice_control",
                name="h1_2_voice_control",
                output="screen",
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("serial_port"),
                        "voice_command_topic": LaunchConfiguration("voice_command_topic"),
                        "voice_feedback_topic": LaunchConfiguration("voice_feedback_topic"),
                        "poll_rate": ParameterValue(LaunchConfiguration("poll_rate"), value_type=float),
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
