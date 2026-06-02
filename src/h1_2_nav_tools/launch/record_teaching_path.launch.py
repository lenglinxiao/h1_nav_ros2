from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_path_file = PathJoinSubstitution(
        [FindPackageShare("h1_2_nav_tools"), "path", "teaching_paths.txt"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("file_path", default_value=default_path_file),
            DeclareLaunchArgument("odom_topic", default_value="/slam_odom"),
            DeclareLaunchArgument("distance_threshold", default_value="0.3"),
            Node(
                package="h1_2_nav_tools",
                executable="record_teaching_path",
                name="record_teaching_path",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "file_path": LaunchConfiguration("file_path"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "distance_threshold": LaunchConfiguration("distance_threshold"),
                        "frame_id": "map",
                    }
                ],
            )
        ]
    )
