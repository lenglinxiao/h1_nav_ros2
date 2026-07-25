from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


RECORD_TOPICS = [
    "/livox/lidar",
    "/livox/imu",
    "/slam_odom",
    "/path",
    "/icp_localization/status",
    "/tf",
    "/tf_static",
    "/initialpose",
]


def generate_launch_description():
    recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            LaunchConfiguration("output"),
            "--storage",
            "sqlite3",
            "--compression-mode",
            "file",
            "--compression-format",
            "zstd",
            "--max-bag-size",
            LaunchConfiguration("max_bag_size"),
            *RECORD_TOPICS,
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output",
                default_value="/tmp/h1_2_slam_bag",
                description="New rosbag directory. It must not already exist.",
            ),
            DeclareLaunchArgument("max_bag_size", default_value="2147483648"),
            recorder,
        ]
    )
