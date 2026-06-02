from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cloud_topic", default_value="/upper_fastlio2/Laser_map"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("frame_id", default_value="camera_init"),
            DeclareLaunchArgument("resolution", default_value="0.05"),
            DeclareLaunchArgument("width_m", default_value="40.0"),
            DeclareLaunchArgument("height_m", default_value="40.0"),
            DeclareLaunchArgument("origin_x", default_value="-20.0"),
            DeclareLaunchArgument("origin_y", default_value="-20.0"),
            DeclareLaunchArgument("map_z", default_value="-1.20"),
            DeclareLaunchArgument("min_z", default_value="-0.90"),
            DeclareLaunchArgument("max_z", default_value="1.80"),
            DeclareLaunchArgument("publish_hz", default_value="2.0"),
            DeclareLaunchArgument("point_stride", default_value="1"),
            DeclareLaunchArgument("inflation_radius", default_value="0.25"),
            DeclareLaunchArgument("cloud_qos_reliable", default_value="false"),
            DeclareLaunchArgument("unknown_value", default_value="-1"),
            DeclareLaunchArgument("raytrace_free", default_value="true"),
            DeclareLaunchArgument("sensor_origin_x", default_value="0.0"),
            DeclareLaunchArgument("sensor_origin_y", default_value="0.0"),
            Node(
                package="h1_2_nav_tools",
                executable="pointcloud_to_occupancy_grid",
                name="pointcloud_to_occupancy_grid",
                output="screen",
                parameters=[
                    {
                        "cloud_topic": LaunchConfiguration("cloud_topic"),
                        "map_topic": LaunchConfiguration("map_topic"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "resolution": LaunchConfiguration("resolution"),
                        "width_m": LaunchConfiguration("width_m"),
                        "height_m": LaunchConfiguration("height_m"),
                        "origin_x": LaunchConfiguration("origin_x"),
                        "origin_y": LaunchConfiguration("origin_y"),
                        "map_z": LaunchConfiguration("map_z"),
                        "min_z": LaunchConfiguration("min_z"),
                        "max_z": LaunchConfiguration("max_z"),
                        "publish_hz": LaunchConfiguration("publish_hz"),
                        "point_stride": LaunchConfiguration("point_stride"),
                        "inflation_radius": LaunchConfiguration("inflation_radius"),
                        "cloud_qos_reliable": LaunchConfiguration("cloud_qos_reliable"),
                        "unknown_value": LaunchConfiguration("unknown_value"),
                        "raytrace_free": LaunchConfiguration("raytrace_free"),
                        "sensor_origin_x": LaunchConfiguration("sensor_origin_x"),
                        "sensor_origin_y": LaunchConfiguration("sensor_origin_y"),
                    }
                ],
            ),
        ]
    )
