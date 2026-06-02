import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("g1_slam_bringup")
    nav_tools_share = get_package_share_directory("g1_nav_tools")
    fastlio2_launch = os.path.join(bringup_share, "launch", "fastlio2_mid360_mapping.launch.py")
    default_path_file = os.path.join(nav_tools_share, "path", "teaching_paths.txt")

    return LaunchDescription(
        [
            DeclareLaunchArgument("path_file", default_value=default_path_file),
            DeclareLaunchArgument("odom_topic", default_value="/slam_odom"),
            DeclareLaunchArgument("target_frame", default_value="camera_init"),
            DeclareLaunchArgument("start_livox_driver", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fastlio2_launch),
                launch_arguments={
                    "start_livox_driver": LaunchConfiguration("start_livox_driver"),
                }.items(),
            ),
            Node(
                package="g1_nav_tools",
                executable="play_teaching_path",
                name="play_teaching_path",
                output="screen",
                parameters=[
                    {
                        "path_file": LaunchConfiguration("path_file"),
                        "frame_id": LaunchConfiguration("target_frame"),
                    }
                ],
            ),
            Node(
                package="g1_nav_tools",
                executable="path_to_waypoint",
                name="path_to_waypoint",
                output="screen",
                parameters=[
                    {
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "target_frame": LaunchConfiguration("target_frame"),
                    }
                ],
            ),
            Node(
                package="g1_nav_tools",
                executable="velocity_smoother_ema",
                name="velocity_smoother_ema",
                output="screen",
            ),
        ]
    )
