import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("h1_2_slam_bringup")
    fastlio_launch = os.path.join(bringup_share, "launch", "fastlio2_mid360_mapping.launch.py")
    p2l_params = os.path.join(bringup_share, "config", "pointcloud_to_laserscan.yaml")
    slam_params = os.path.join(bringup_share, "config", "slam_toolbox_mapping.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_livox_driver", default_value="false"),
            DeclareLaunchArgument("fastlio_body_cloud_topic", default_value="/cloud_registered_body"),
            DeclareLaunchArgument("nav_cloud_topic", default_value="/cloud_nav_body"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("input_odom_topic", default_value="/slam_odom"),
            DeclareLaunchArgument("output_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("livox_frame", default_value="livox_frame"),
            DeclareLaunchArgument("livox_x", default_value="0.0"),
            DeclareLaunchArgument("livox_y", default_value="0.0"),
            DeclareLaunchArgument("livox_z", default_value="1.70"),
            DeclareLaunchArgument("livox_roll", default_value="0.0"),
            DeclareLaunchArgument("livox_pitch", default_value="0.0"),
            DeclareLaunchArgument("livox_yaw", default_value="0.0"),
            DeclareLaunchArgument("odom_yaw_offset", default_value="3.141592653589793"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fastlio_launch),
                launch_arguments={
                    "start_livox_driver": LaunchConfiguration("start_livox_driver"),
                    "livox_frame": LaunchConfiguration("livox_frame"),
                }.items(),
            ),
            Node(
                package="h1_2_nav_tools",
                executable="fastlio_odom_adapter",
                name="fastlio_odom_adapter",
                output="screen",
                parameters=[
                    {
                        "input_odom_topic": LaunchConfiguration("input_odom_topic"),
                        "output_odom_topic": LaunchConfiguration("output_odom_topic"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "yaw_offset": ParameterValue(LaunchConfiguration("odom_yaw_offset"), value_type=float),
                        "publish_tf": True,
                        "planarize": True,
                    }
                ],
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
                package="h1_2_nav_tools",
                executable="pointcloud_frame_adapter",
                name="pointcloud_frame_adapter",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("fastlio_body_cloud_topic"),
                        "output_topic": LaunchConfiguration("nav_cloud_topic"),
                        "target_frame": LaunchConfiguration("base_frame"),
                    }
                ],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[p2l_params],
                remappings=[
                    ("cloud_in", LaunchConfiguration("nav_cloud_topic")),
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
