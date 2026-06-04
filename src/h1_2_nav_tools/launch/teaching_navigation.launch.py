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
            DeclareLaunchArgument("path_file", default_value=default_path_file),
            DeclareLaunchArgument("odom_topic", default_value="/slam_odom"),
            DeclareLaunchArgument("raw_cmd_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("cmd_topic", default_value="/cmd_vel_smooth"),
            Node(
                package="h1_2_nav_tools",
                executable="play_teaching_path",
                name="play_teaching_path",
                output="screen",
                parameters=[
                    {
                        "path_file": LaunchConfiguration("path_file"),
                        "path_topic": "/teach_path",
                        "frame_id": "map",
                    }
                ],
            ),
            Node(
                package="h1_2_nav_tools",
                executable="path_to_waypoint",
                name="path_to_waypoint",
                output="screen",
                parameters=[
                    {
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "path_topic": "/teach_path",
                        "waypoint_topic": "/way_point",
                        "target_frame": "map",
                    }
                ],
            ),
            Node(
                package="h1_2_nav_tools",
                executable="velocity_smoother_ema",
                name="velocity_smoother_ema",
                output="screen",
                parameters=[
                    {
                        "raw_cmd_topic": LaunchConfiguration("raw_cmd_topic"),
                        "cmd_topic": LaunchConfiguration("cmd_topic"),
                    }
                ],
            ),
            Node(
                package="h1_2_nav_tools",
                executable="costmap_clear_on_initialpose",
                name="costmap_clear_on_initialpose",
                output="screen",
            ),
        ]
    )
