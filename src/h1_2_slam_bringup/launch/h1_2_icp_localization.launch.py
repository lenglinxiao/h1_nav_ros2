from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("pcd_map", default_value=""),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="camera_init"),
            DeclareLaunchArgument("base_frame", default_value="body"),
            DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered_body"),
            DeclareLaunchArgument("odom_topic", default_value="/slam_odom"),
            DeclareLaunchArgument("status_topic", default_value="/icp_localization/status"),
            DeclareLaunchArgument("refine_resolution", default_value="0.2"),
            DeclareLaunchArgument("rough_resolution", default_value="0.5"),
            DeclareLaunchArgument("refine_iter", default_value="5"),
            DeclareLaunchArgument("rough_iter", default_value="10"),
            DeclareLaunchArgument("score_threshold", default_value="0.15"),
            DeclareLaunchArgument("xy_offset", default_value="1.0"),
            DeclareLaunchArgument("xy_steps", default_value="3"),
            DeclareLaunchArgument("yaw_offset", default_value="1"),
            DeclareLaunchArgument("yaw_resolution", default_value="0.5"),
            DeclareLaunchArgument("localize_rate_hz", default_value="1.0"),
            Node(
                package="h1_2_icp_localization",
                executable="icp_localization_node",
                name="icp_localization_node",
                output="screen",
                parameters=[
                    {
                        "pcd_path": LaunchConfiguration("pcd_map"),
                        "map_frame": LaunchConfiguration("map_frame"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "cloud_topic": LaunchConfiguration("cloud_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "status_topic": "/icp_localization/status",
                        "publish_tf": True,
                        "refine_resolution": LaunchConfiguration("refine_resolution"),
                        "rough_resolution": LaunchConfiguration("rough_resolution"),
                        "refine_iter": LaunchConfiguration("refine_iter"),
                        "rough_iter": LaunchConfiguration("rough_iter"),
                        "score_threshold": LaunchConfiguration("score_threshold"),
                        "xy_offset": LaunchConfiguration("xy_offset"),
                        "xy_steps": LaunchConfiguration("xy_steps"),
                        "yaw_offset": LaunchConfiguration("yaw_offset"),
                        "yaw_resolution": LaunchConfiguration("yaw_resolution"),
                        "localize_rate_hz": LaunchConfiguration("localize_rate_hz"),
                    }
                ],
            ),
        ]
    )
