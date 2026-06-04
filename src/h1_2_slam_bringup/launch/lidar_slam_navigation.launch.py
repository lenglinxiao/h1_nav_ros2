import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("h1_2_slam_bringup")
    nav_tools_share = get_package_share_directory("h1_2_nav_tools")
    livox_launch = os.path.join(bringup_share, "launch", "livox_mid360.launch.py")
    p2l_params = os.path.join(bringup_share, "config", "pointcloud_to_laserscan.yaml")
    slam_params = os.path.join(bringup_share, "config", "slam_toolbox_localization.yaml")
    default_path_file = os.path.join(nav_tools_share, "path", "teaching_paths.txt")

    return LaunchDescription(
        [
            DeclareLaunchArgument("cloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("path_file", default_value=default_path_file),
            DeclareLaunchArgument("odom_topic", default_value="/slam_odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("livox_frame", default_value="livox_frame"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(livox_launch),
                launch_arguments={"frame_id": LaunchConfiguration("livox_frame")}.items(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_livox",
                arguments=["0", "0", "1.70", "0", "0", "0", LaunchConfiguration("base_frame"), LaunchConfiguration("livox_frame")],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_slam_odom",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "slam_odom"],
                output="screen",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[p2l_params],
                remappings=[
                    ("cloud_in", LaunchConfiguration("cloud_topic")),
                    ("scan", LaunchConfiguration("scan_topic")),
                ],
            ),
            Node(
                package="slam_toolbox",
                executable="localization_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params],
            ),
            Node(
                package="h1_2_nav_tools",
                executable="play_teaching_path",
                name="play_teaching_path",
                output="screen",
                parameters=[{"path_file": LaunchConfiguration("path_file")}],
            ),
            Node(
                package="h1_2_nav_tools",
                executable="path_to_waypoint",
                name="path_to_waypoint",
                output="screen",
                parameters=[{"odom_topic": LaunchConfiguration("odom_topic")}],
            ),
        ]
    )
