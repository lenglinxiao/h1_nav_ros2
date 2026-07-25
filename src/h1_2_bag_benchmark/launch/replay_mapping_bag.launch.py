import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("h1_2_slam_bringup")
    fastlio_launch = os.path.join(
        bringup_share,
        "launch",
        "fastlio2_mid360_mapping.launch.py",
    )
    output_dir = LaunchConfiguration("output_dir")

    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_launch),
        launch_arguments={
            "start_livox_driver": "false",
            "use_sim_time": "true",
            "use_ros_time_stamps": "false",
            "save_pcd": "true",
            "map_file_path": PathJoinSubstitution([output_dir, "replayed_map.pcd"]),
        }.items(),
    )
    evaluator = Node(
        package="h1_2_bag_benchmark",
        executable="slam_bag_evaluator",
        name="mapping_bag_evaluator",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("h1_2_bag_benchmark"),
                "config",
                "benchmark_defaults.yaml",
            ),
            {
                "use_sim_time": True,
                "mode": "mapping",
                "output_dir": output_dir,
                "reference_csv": LaunchConfiguration("reference_csv"),
                "max_endpoint_error_m": ParameterValue(
                    LaunchConfiguration("max_endpoint_error_m"), value_type=float
                ),
                "max_pose_step_m": ParameterValue(
                    LaunchConfiguration("max_pose_step_m"), value_type=float
                ),
                "max_position_regression_p95_m": ParameterValue(
                    LaunchConfiguration("max_position_regression_p95_m"), value_type=float
                ),
            },
        ],
    )
    player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("bag"),
            "--clock",
            LaunchConfiguration("clock_hz"),
            "--rate",
            LaunchConfiguration("rate"),
            "--disable-keyboard-controls",
            "--topics",
            "/livox/lidar",
            "/livox/imu",
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", description="Input bag directory"),
            DeclareLaunchArgument("output_dir", default_value="/tmp/h1_2_mapping_benchmark"),
            DeclareLaunchArgument("reference_csv", default_value=""),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("clock_hz", default_value="100"),
            DeclareLaunchArgument("startup_delay_sec", default_value="3.0"),
            DeclareLaunchArgument("drain_delay_sec", default_value="5.0"),
            DeclareLaunchArgument("max_endpoint_error_m", default_value="-1.0"),
            DeclareLaunchArgument("max_pose_step_m", default_value="-1.0"),
            DeclareLaunchArgument("max_position_regression_p95_m", default_value="-1.0"),
            fastlio,
            evaluator,
            TimerAction(
                period=LaunchConfiguration("startup_delay_sec"),
                actions=[player],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=player,
                    on_exit=[
                        TimerAction(
                            period=LaunchConfiguration("drain_delay_sec"),
                            actions=[EmitEvent(event=Shutdown(reason="bag playback completed"))],
                        )
                    ],
                )
            ),
        ]
    )
