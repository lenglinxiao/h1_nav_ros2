import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_tools_share = get_package_share_directory("h1_2_nav_tools")
    default_rviz_config = os.path.join(
        nav_tools_share, "rviz", "h1_2_nav2_camera_init.rviz"
    )

    return LaunchDescription(
        [
            UnsetEnvironmentVariable("ROS_HOSTNAME"),
            UnsetEnvironmentVariable("ROS_IP"),
            UnsetEnvironmentVariable("ROS_MASTER_URI"),
            UnsetEnvironmentVariable("CYCLONEDDS_URI"),
            UnsetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),
            SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        ]
    )
