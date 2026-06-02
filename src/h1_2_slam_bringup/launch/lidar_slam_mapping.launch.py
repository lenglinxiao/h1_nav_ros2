import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("h1_2_slam_bringup")
    livox_launch = os.path.join(bringup_share, "launch", "livox_mid360.launch.py")
    p2l_params = os.path.join(bringup_share, "config", "pointcloud_to_laserscan.yaml")
    slam_params = os.path.join(bringup_share, "config", "slam_toolbox_mapping.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("cloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("livox_frame", default_value="livox_frame"),
            DeclareLaunchArgument("livox_x", default_value="0.0"),
            DeclareLaunchArgument("livox_y", default_value="0.0"),
            DeclareLaunchArgument("livox_z", default_value="1.70"),
            DeclareLaunchArgument("livox_roll", default_value="0.0"),
            DeclareLaunchArgument("livox_pitch", default_value="0.0"),
            DeclareLaunchArgument("livox_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(livox_launch),
                launch_arguments={"frame_id": LaunchConfiguration("livox_frame")}.items(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_livox",
                arguments=[
                    LaunchConfiguration("livox_x"),
                    LaunchConfiguration("livox_y"),
                    LaunchConfiguration("livox_z"),
                    LaunchConfiguration("livox_yaw"),
                    LaunchConfiguration("livox_pitch"),
                    LaunchConfiguration("livox_roll"),
                    LaunchConfiguration("base_frame"),
                    LaunchConfiguration("livox_frame"),
                ],
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
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params],
            ),
        ]
    )
