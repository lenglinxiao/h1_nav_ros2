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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("h1_2_slam_bringup")
    benchmark_share = get_package_share_directory("h1_2_bag_benchmark")
    fastlio_launch = os.path.join(bringup_share, "launch", "fastlio2_mid360_mapping.launch.py")
    icp_launch = os.path.join(bringup_share, "launch", "h1_2_icp_localization.launch.py")
    global_launch = os.path.join(bringup_share, "launch", "h1_2_global_relocalization.launch.py")
    backend = LaunchConfiguration("backend")

    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fastlio_launch),
        launch_arguments={
            "start_livox_driver": "false",
            "use_sim_time": "true",
            "use_ros_time_stamps": "false",
            "save_pcd": "false",
        }.items(),
    )
    icp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(icp_launch),
        condition=IfCondition(PythonExpression(["'", backend, "' == 'icp'"])),
        launch_arguments={
            "pcd_map": LaunchConfiguration("pcd_map"),
            "use_sim_time": "true",
        }.items(),
    )
    global_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(global_launch),
        condition=IfCondition(PythonExpression(["'", backend, "' == 'global_gicp'"])),
        launch_arguments={
            "pcd_map": LaunchConfiguration("pcd_map"),
            "use_sim_time": "true",
        }.items(),
    )
    evaluator = Node(
        package="h1_2_bag_benchmark",
        executable="slam_bag_evaluator",
        name="localization_bag_evaluator",
        output="screen",
        parameters=[
            os.path.join(benchmark_share, "config", "benchmark_defaults.yaml"),
            {
                "use_sim_time": True,
                "mode": "localization",
                "output_dir": LaunchConfiguration("output_dir"),
                "reference_csv": LaunchConfiguration("reference_csv"),
                "max_pose_step_m": ParameterValue(
                    LaunchConfiguration("max_pose_step_m"), value_type=float
                ),
                "max_position_regression_p95_m": ParameterValue(
                    LaunchConfiguration("max_position_regression_p95_m"), value_type=float
                ),
                "max_degraded_ratio": ParameterValue(
                    LaunchConfiguration("max_degraded_ratio"), value_type=float
                ),
                "min_converged_ratio": ParameterValue(
                    LaunchConfiguration("min_converged_ratio"), value_type=float
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
            "/initialpose",
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", description="Input bag directory"),
            DeclareLaunchArgument("pcd_map", description="Reference PCD map"),
            DeclareLaunchArgument("backend", default_value="global_gicp"),
            DeclareLaunchArgument("output_dir", default_value="/tmp/h1_2_localization_benchmark"),
            DeclareLaunchArgument("reference_csv", default_value=""),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("clock_hz", default_value="100"),
            DeclareLaunchArgument("startup_delay_sec", default_value="3.0"),
            DeclareLaunchArgument("drain_delay_sec", default_value="5.0"),
            DeclareLaunchArgument("max_pose_step_m", default_value="-1.0"),
            DeclareLaunchArgument("max_position_regression_p95_m", default_value="-1.0"),
            DeclareLaunchArgument("max_degraded_ratio", default_value="-1.0"),
            DeclareLaunchArgument("min_converged_ratio", default_value="-1.0"),
            fastlio,
            icp,
            global_localization,
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
