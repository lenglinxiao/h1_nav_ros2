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
    bringup_share = get_package_share_directory("h1_2_slam_bringup")
    livox_launch = os.path.join(bringup_share, "launch", "livox_mid360.launch.py")
    default_fastlio2_config = PathJoinSubstitution([FindPackageShare("fast_lio2"), "config", "mid360.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("fastlio_config", default_value=default_fastlio2_config),
            DeclareLaunchArgument("livox_frame", default_value="livox_frame"),
            DeclareLaunchArgument("start_livox_driver", default_value="false"),
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
                remappings=[("/Odometry", "/slam_odom")],
            ),
        ]
    )
