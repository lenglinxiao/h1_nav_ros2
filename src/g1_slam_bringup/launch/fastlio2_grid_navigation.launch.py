from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("g1_slam_bringup")
    nav_tools_share = get_package_share_directory("g1_nav_tools")
    loco_share = get_package_share_directory("g1_loco_bridge")
    navigation_launch = bringup_share + "/launch/fastlio2_mid360_mapping.launch.py"
    grid_navigation_launch = nav_tools_share + "/launch/simple_grid_navigation.launch.py"
    loco_bridge_launch = loco_share + "/launch/g1_loco_bridge.launch.py"

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_livox_driver", default_value="false"),
            DeclareLaunchArgument("enable_sport_bridge", default_value="true"),
            DeclareLaunchArgument("auto_start_loco", default_value="true"),
            DeclareLaunchArgument("network_interface", default_value="enp45s0"),
            DeclareLaunchArgument("domain_id", default_value="0"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("odom_topic", default_value="/slam_odom"),
            DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
            DeclareLaunchArgument("clicked_point_topic", default_value="/clicked_point"),
            DeclareLaunchArgument("max_linear_speed", default_value="0.20"),
            DeclareLaunchArgument("max_angular_speed", default_value="0.5"),
            DeclareLaunchArgument("goal_tolerance", default_value="0.30"),
            DeclareLaunchArgument("lookahead_distance", default_value="0.70"),
            DeclareLaunchArgument("odom_yaw_offset", default_value="3.141592653589793"),
            DeclareLaunchArgument("grid_min_z", default_value="0.30"),
            DeclareLaunchArgument("grid_max_z", default_value="2.00"),
            DeclareLaunchArgument("grid_filter_floating_points", default_value="true"),
            DeclareLaunchArgument("grid_support_min_z", default_value="0.05"),
            DeclareLaunchArgument("grid_support_max_z", default_value="0.30"),
            DeclareLaunchArgument("grid_occupancy_hit_increment", default_value="2"),
            DeclareLaunchArgument("grid_occupancy_miss_decrement", default_value="1"),
            DeclareLaunchArgument("grid_occupancy_threshold", default_value="6"),
            DeclareLaunchArgument("grid_occupancy_max_score", default_value="20"),
            DeclareLaunchArgument("inflation_radius", default_value="0.30"),
            DeclareLaunchArgument("allow_unknown", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    "start_livox_driver": LaunchConfiguration("start_livox_driver"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(grid_navigation_launch),
                launch_arguments={
                    "map_topic": LaunchConfiguration("map_topic"),
                    "odom_topic": LaunchConfiguration("odom_topic"),
                    "goal_topic": LaunchConfiguration("goal_topic"),
                    "clicked_point_topic": LaunchConfiguration("clicked_point_topic"),
                    "global_frame": "camera_init",
                    "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                    "max_angular_speed": LaunchConfiguration("max_angular_speed"),
                    "goal_tolerance": LaunchConfiguration("goal_tolerance"),
                    "lookahead_distance": LaunchConfiguration("lookahead_distance"),
                    "odom_yaw_offset": LaunchConfiguration("odom_yaw_offset"),
                    "min_z": LaunchConfiguration("grid_min_z"),
                    "max_z": LaunchConfiguration("grid_max_z"),
                    "filter_floating_points": LaunchConfiguration("grid_filter_floating_points"),
                    "support_min_z": LaunchConfiguration("grid_support_min_z"),
                    "support_max_z": LaunchConfiguration("grid_support_max_z"),
                    "occupancy_hit_increment": LaunchConfiguration("grid_occupancy_hit_increment"),
                    "occupancy_miss_decrement": LaunchConfiguration("grid_occupancy_miss_decrement"),
                    "occupancy_threshold": LaunchConfiguration("grid_occupancy_threshold"),
                    "occupancy_max_score": LaunchConfiguration("grid_occupancy_max_score"),
                    "allow_unknown": LaunchConfiguration("allow_unknown"),
                    "udp_bridge_enabled": LaunchConfiguration("enable_sport_bridge"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(loco_bridge_launch),
                launch_arguments={
                    "cmd_topic": "/cmd_vel_nav",
                    "enabled": LaunchConfiguration("enable_sport_bridge"),
                    "start_ros_shim": "false",
                    "auto_start": LaunchConfiguration("auto_start_loco"),
                    "network_interface": LaunchConfiguration("network_interface"),
                    "domain_id": LaunchConfiguration("domain_id"),
                    "max_x": LaunchConfiguration("max_linear_speed"),
                    "max_y": "0.0",
                    "max_yaw": LaunchConfiguration("max_angular_speed"),
                }.items(),
            ),
        ]
    )
