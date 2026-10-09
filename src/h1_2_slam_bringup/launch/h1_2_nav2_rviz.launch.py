import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    nav_tools_share = get_package_share_directory("h1_2_nav_tools")
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    rviz_config = LaunchConfiguration("rviz_config").perform(context).strip()

    if not rviz_config:
        config_name = "h1_2_nav2_offline.rviz" if mode == "offline" else "h1_2_nav2_camera_init.rviz"
        rviz_config = os.path.join(nav_tools_share, "rviz", config_name)

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            UnsetEnvironmentVariable("ROS_HOSTNAME"),
            UnsetEnvironmentVariable("ROS_IP"),
            UnsetEnvironmentVariable("ROS_MASTER_URI"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),
            SetEnvironmentVariable("ROS_DOMAIN_ID", os.environ.get("ROS_DOMAIN_ID", "0")),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")),
            DeclareLaunchArgument("mode", default_value="online"),
            DeclareLaunchArgument("rviz_config", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
