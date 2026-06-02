from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("livox_ros_driver2"), "config", "MID360_config.json"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("frame_id", default_value="livox_frame"),
            DeclareLaunchArgument("user_config_path", default_value=default_config),
            DeclareLaunchArgument("publish_freq", default_value="10.0"),
            DeclareLaunchArgument("xfer_format", default_value="1"),
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                output="screen",
                parameters=[
                    {
                        "xfer_format": LaunchConfiguration("xfer_format"),
                        "multi_topic": 0,
                        "data_src": 0,
                        "publish_freq": LaunchConfiguration("publish_freq"),
                        "output_data_type": 0,
                        "frame_id": LaunchConfiguration("frame_id"),
                        "lvx_file_path": "/tmp/livox_test.lvx",
                        "user_config_path": LaunchConfiguration("user_config_path"),
                        "cmdline_input_bd_code": "livox0000000001",
                    }
                ],
            ),
        ]
    )
