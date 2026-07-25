from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_global_initialization", default_value="true"),
            DeclareLaunchArgument("use_global_recovery", default_value="true"),
            DeclareLaunchArgument("verify_global_with_gicp", default_value="true"),
            DeclareLaunchArgument("use_kiss_matcher", default_value="true"),
            DeclareLaunchArgument("use_small_gicp", default_value="true"),
            DeclareLaunchArgument("global_voxel_resolution", default_value="0.35"),
            DeclareLaunchArgument("tracking_voxel_resolution", default_value="0.20"),
            DeclareLaunchArgument("normal_radius", default_value="0.8"),
            DeclareLaunchArgument("feature_radius", default_value="1.5"),
            DeclareLaunchArgument("gicp_max_correspondence_distance", default_value="1.0"),
            DeclareLaunchArgument("gicp_score_threshold", default_value="0.35"),
            DeclareLaunchArgument("global_score_threshold", default_value="0.5"),
            DeclareLaunchArgument("gicp_max_consecutive_failures", default_value="3"),
            DeclareLaunchArgument("min_input_points", default_value="100"),
            DeclareLaunchArgument("max_accumulated_points", default_value="120000"),
            DeclareLaunchArgument("recovery_min_points", default_value="1000"),
            DeclareLaunchArgument("recovery_cooldown_sec", default_value="2.0"),
            DeclareLaunchArgument("registration_rate_hz", default_value="2.0"),
            DeclareLaunchArgument("tf_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("transform_tolerance_sec", default_value="0.1"),
            DeclareLaunchArgument("num_threads", default_value="4"),
            Node(
                package="h1_2_icp_localization",
                executable="global_relocalization_node",
                name="global_relocalization_node",
                output="screen",
                parameters=[
                    {
                        "pcd_path": LaunchConfiguration("pcd_map"),
                        "map_frame": LaunchConfiguration("map_frame"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "cloud_topic": LaunchConfiguration("cloud_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "status_topic": LaunchConfiguration("status_topic"),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "publish_tf": True,
                        "use_global_initialization": ParameterValue(
                            LaunchConfiguration("use_global_initialization"), value_type=bool
                        ),
                        "use_global_recovery": ParameterValue(
                            LaunchConfiguration("use_global_recovery"), value_type=bool
                        ),
                        "verify_global_with_gicp": ParameterValue(
                            LaunchConfiguration("verify_global_with_gicp"), value_type=bool
                        ),
                        "use_kiss_matcher": ParameterValue(
                            LaunchConfiguration("use_kiss_matcher"), value_type=bool
                        ),
                        "use_small_gicp": ParameterValue(
                            LaunchConfiguration("use_small_gicp"), value_type=bool
                        ),
                        "global_voxel_resolution": ParameterValue(
                            LaunchConfiguration("global_voxel_resolution"), value_type=float
                        ),
                        "tracking_voxel_resolution": ParameterValue(
                            LaunchConfiguration("tracking_voxel_resolution"), value_type=float
                        ),
                        "normal_radius": ParameterValue(LaunchConfiguration("normal_radius"), value_type=float),
                        "feature_radius": ParameterValue(LaunchConfiguration("feature_radius"), value_type=float),
                        "gicp_max_correspondence_distance": ParameterValue(
                            LaunchConfiguration("gicp_max_correspondence_distance"), value_type=float
                        ),
                        "gicp_score_threshold": ParameterValue(
                            LaunchConfiguration("gicp_score_threshold"), value_type=float
                        ),
                        "global_score_threshold": ParameterValue(
                            LaunchConfiguration("global_score_threshold"), value_type=float
                        ),
                        "gicp_max_consecutive_failures": ParameterValue(
                            LaunchConfiguration("gicp_max_consecutive_failures"), value_type=int
                        ),
                        "min_input_points": ParameterValue(
                            LaunchConfiguration("min_input_points"), value_type=int
                        ),
                        "max_accumulated_points": ParameterValue(
                            LaunchConfiguration("max_accumulated_points"), value_type=int
                        ),
                        "recovery_min_points": ParameterValue(
                            LaunchConfiguration("recovery_min_points"), value_type=int
                        ),
                        "recovery_cooldown_sec": ParameterValue(
                            LaunchConfiguration("recovery_cooldown_sec"), value_type=float
                        ),
                        "registration_rate_hz": ParameterValue(
                            LaunchConfiguration("registration_rate_hz"), value_type=float
                        ),
                        "tf_rate_hz": ParameterValue(LaunchConfiguration("tf_rate_hz"), value_type=float),
                        "transform_tolerance_sec": ParameterValue(
                            LaunchConfiguration("transform_tolerance_sec"), value_type=float
                        ),
                        "num_threads": ParameterValue(LaunchConfiguration("num_threads"), value_type=int),
                    }
                ],
            ),
        ]
    )
