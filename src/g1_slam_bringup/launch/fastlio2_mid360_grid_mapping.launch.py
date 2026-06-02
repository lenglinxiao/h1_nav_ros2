import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory("g1_slam_bringup")
    livox_launch = os.path.join(bringup_share, "launch", "livox_mid360.launch.py")
    default_fastlio_config = PathJoinSubstitution([FindPackageShare("fast_lio2"), "config", "mid360.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("fastlio_config", default_value=default_fastlio_config),
            DeclareLaunchArgument("livox_frame", default_value="livox_frame"),
            DeclareLaunchArgument("start_livox_driver", default_value="false"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("odom_topic", default_value="/upper_fastlio2/Odometry"),
            DeclareLaunchArgument("grid_cloud_topic", default_value="/upper_fastlio2/cloud_registered"),
            DeclareLaunchArgument("grid_frame_id", default_value="camera_init"),
            DeclareLaunchArgument("resolution", default_value="0.05"),
            DeclareLaunchArgument("width_m", default_value="40.0"),
            DeclareLaunchArgument("height_m", default_value="40.0"),
            DeclareLaunchArgument("origin_x", default_value="-20.0"),
            DeclareLaunchArgument("origin_y", default_value="-20.0"),
            DeclareLaunchArgument("map_z", default_value="-1.20"),
            DeclareLaunchArgument("min_z", default_value="-0.90"),
            DeclareLaunchArgument("max_z", default_value="1.80"),
            DeclareLaunchArgument("publish_hz", default_value="2.0"),
            DeclareLaunchArgument("inflation_radius", default_value="0.25"),
            DeclareLaunchArgument("unknown_value", default_value="-1"),
            DeclareLaunchArgument("raytrace_free", default_value="true"),
            DeclareLaunchArgument("sensor_origin_x", default_value="0.0"),
            DeclareLaunchArgument("sensor_origin_y", default_value="0.0"),
            DeclareLaunchArgument("sensor_origin_from_odom", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(livox_launch),
                launch_arguments={
                    "frame_id": LaunchConfiguration("livox_frame"),
                    "xfer_format": "1",
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_livox_driver")),
            ),
            Node(
                package="fast_lio2",
                executable="fastlio2_mapping",
                name="fastlio2_mapping",
                output="screen",
                parameters=[LaunchConfiguration("fastlio_config")],
                remappings=[
                    ("/Odometry", "/upper_fastlio2/Odometry"),
                    ("/cloud_registered", "/upper_fastlio2/cloud_registered"),
                    ("/cloud_registered_body", "/upper_fastlio2/cloud_registered_body"),
                    ("/cloud_effected", "/upper_fastlio2/cloud_effected"),
                    ("/Laser_map", "/upper_fastlio2/Laser_map"),
                    ("/path", "/upper_fastlio2/path"),
                    ("/map_save", "/upper_fastlio2/map_save"),
                ],
            ),
            Node(
                package="g1_nav_tools",
                executable="pointcloud_to_occupancy_grid",
                name="pointcloud_to_occupancy_grid",
                output="screen",
                parameters=[
                    {
                        "cloud_topic": LaunchConfiguration("grid_cloud_topic"),
                        "map_topic": LaunchConfiguration("map_topic"),
                        "frame_id": LaunchConfiguration("grid_frame_id"),
                        "resolution": LaunchConfiguration("resolution"),
                        "width_m": LaunchConfiguration("width_m"),
                        "height_m": LaunchConfiguration("height_m"),
                        "origin_x": LaunchConfiguration("origin_x"),
                        "origin_y": LaunchConfiguration("origin_y"),
                        "map_z": LaunchConfiguration("map_z"),
                        "min_z": LaunchConfiguration("min_z"),
                        "max_z": LaunchConfiguration("max_z"),
                        "publish_hz": LaunchConfiguration("publish_hz"),
                        "inflation_radius": LaunchConfiguration("inflation_radius"),
                        "cloud_qos_reliable": "false",
                        "unknown_value": LaunchConfiguration("unknown_value"),
                        "raytrace_free": LaunchConfiguration("raytrace_free"),
                        "sensor_origin_x": LaunchConfiguration("sensor_origin_x"),
                        "sensor_origin_y": LaunchConfiguration("sensor_origin_y"),
                        "sensor_origin_from_odom": LaunchConfiguration("sensor_origin_from_odom"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                    }
                ],
            ),
        ]
    )
