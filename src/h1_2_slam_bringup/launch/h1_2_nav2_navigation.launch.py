import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("h1_2_slam_bringup")
    description_share = get_package_share_directory("h1_2_description")
    nav_tools_share = get_package_share_directory("h1_2_nav_tools")
    loco_share = get_package_share_directory("h1_2_loco_bridge")
    nav2_share = get_package_share_directory("nav2_bringup")

    fastlio_launch = os.path.join(bringup_share, "launch", "fastlio2_mid360_mapping.launch.py")
    loco_bridge_launch = os.path.join(loco_share, "launch", "h1_2_loco_bridge.launch.py")
    nav2_navigation_launch = os.path.join(nav2_share, "launch", "navigation_launch.py")
    icp_localization_launch = os.path.join(bringup_share, "launch", "h1_2_icp_localization.launch.py")
    global_relocalization_launch = os.path.join(bringup_share, "launch", "h1_2_global_relocalization.launch.py")
    p2l_params = os.path.join(bringup_share, "config", "pointcloud_to_laserscan_body.yaml")
    default_nav2_params = os.path.join(nav_tools_share, "config", "nav2_h1_2.yaml")
    robot_urdf = os.path.join(description_share, "h1_2_handless.urdf")
    with open(robot_urdf, "r", encoding="utf-8") as stream:
        robot_description = stream.read().replace(
            'filename="meshes/',
            'filename="package://h1_2_description/meshes/',
        )
    robot_root = ET.fromstring(robot_description)
    for link in robot_root.findall("link"):
        link_name = link.attrib.get("name", "link")
        for index, visual in enumerate(link.findall("visual")):
            material = visual.find("material")
            if material is not None and "name" not in material.attrib:
                material.attrib["name"] = f"{link_name}_visual_material_{index}"
    for joint in robot_root.findall("joint"):
        joint.attrib["type"] = "fixed"
    robot_description = ET.tostring(robot_root, encoding="unicode")
    localization_backend = LaunchConfiguration("localization_backend")
    icp_backend_condition = IfCondition(PythonExpression(["'", localization_backend, "' == 'icp'"]))
    global_gicp_backend_condition = IfCondition(
        PythonExpression(["'", localization_backend, "' == 'global_gicp'"])
    )
    pcd_backend_condition = IfCondition(
        PythonExpression(
            ["'", localization_backend, "' == 'icp' or '", localization_backend, "' == 'global_gicp'"]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("localization_backend", default_value="global_gicp"),
            DeclareLaunchArgument("pcd_map", default_value=""),
            DeclareLaunchArgument("params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_livox_driver", default_value="false"),
            DeclareLaunchArgument("map_file_path", default_value="./test.pcd"),
            DeclareLaunchArgument("fastlio_body_cloud_topic", default_value="/cloud_registered_body"),
            DeclareLaunchArgument("nav_cloud_topic", default_value="/cloud_nav_body"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("input_odom_topic", default_value="/slam_odom"),
            DeclareLaunchArgument("output_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("odom_frame", default_value="camera_init"),
            DeclareLaunchArgument("base_frame", default_value="body"),
            DeclareLaunchArgument("livox_frame", default_value="livox_frame"),
            DeclareLaunchArgument("livox_x", default_value="0.0"),
            DeclareLaunchArgument("livox_y", default_value="0.0"),
            DeclareLaunchArgument("livox_z", default_value="1.70"),
            DeclareLaunchArgument("livox_roll", default_value="0.0"),
            DeclareLaunchArgument("livox_pitch", default_value="0.0"),
            DeclareLaunchArgument("livox_yaw", default_value="0.0"),
            DeclareLaunchArgument("odom_yaw_offset", default_value="3.141592653589793"),
            DeclareLaunchArgument("enable_sport_bridge", default_value="true"),
            DeclareLaunchArgument("auto_start_loco", default_value="true"),
            DeclareLaunchArgument("network_interface", default_value="enp45s0"),
            DeclareLaunchArgument("domain_id", default_value="0"),
            DeclareLaunchArgument("max_linear_speed", default_value="0.20"),
            DeclareLaunchArgument("max_angular_speed", default_value="0.5"),
            DeclareLaunchArgument("robot_pelvis_z", default_value="1.03"),
            DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
            DeclareLaunchArgument("memory_points_file", default_value="maps/memory_points.json"),
            DeclareLaunchArgument("memory_route", default_value="A,B,C"),
            DeclareLaunchArgument("arrival_wait_sec", default_value="2.0"),
            DeclareLaunchArgument("blocked_retry_wait_sec", default_value="5.0"),
            DeclareLaunchArgument("max_retries_per_goal", default_value="3"),
            DeclareLaunchArgument("auto_initial_pose", default_value="true"),
            DeclareLaunchArgument("initial_pose_name", default_value="START"),
            DeclareLaunchArgument("auto_start_route", default_value="false"),
            DeclareLaunchArgument("route_start_delay_sec", default_value="3.0"),
            DeclareLaunchArgument("initial_pose_publish_count", default_value="5"),
            DeclareLaunchArgument("initial_pose_publish_period_sec", default_value="0.5"),
            DeclareLaunchArgument("wait_for_icp_status", default_value="true"),
            DeclareLaunchArgument("required_stable_samples", default_value="1"),
            DeclareLaunchArgument("max_icp_score", default_value="0.15"),
            DeclareLaunchArgument("localization_timeout_sec", default_value="15.0"),
            DeclareLaunchArgument("return_to_start", default_value="false"),
            DeclareLaunchArgument("global_voxel_resolution", default_value="0.35"),
            DeclareLaunchArgument("tracking_voxel_resolution", default_value="0.20"),
            DeclareLaunchArgument("global_normal_radius", default_value="0.8"),
            DeclareLaunchArgument("global_feature_radius", default_value="1.5"),
            DeclareLaunchArgument("global_score_threshold", default_value="0.5"),
            DeclareLaunchArgument("gicp_score_threshold", default_value="0.35"),
            DeclareLaunchArgument("gicp_max_correspondence_distance", default_value="1.0"),
            DeclareLaunchArgument("gicp_max_consecutive_failures", default_value="3"),
            DeclareLaunchArgument("global_use_kiss_matcher", default_value="false"),
            DeclareLaunchArgument("global_use_small_gicp", default_value="true"),
            DeclareLaunchArgument("global_min_input_points", default_value="100"),
            DeclareLaunchArgument("global_max_accumulated_points", default_value="120000"),
            DeclareLaunchArgument("recovery_min_points", default_value="1000"),
            DeclareLaunchArgument("recovery_cooldown_sec", default_value="2.0"),
            DeclareLaunchArgument("global_registration_rate_hz", default_value="2.0"),
            DeclareLaunchArgument("global_tf_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("global_transform_tolerance_sec", default_value="0.5"),
            DeclareLaunchArgument("global_num_threads", default_value="4"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(robot_description, value_type=str),
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="body_to_pelvis",
                arguments=[
                    "0",
                    "0",
                    LaunchConfiguration("robot_pelvis_z"),
                    "0",
                    "0",
                    "0",
                    "body",
                    "pelvis",
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fastlio_launch),
                launch_arguments={
                    "start_livox_driver": LaunchConfiguration("start_livox_driver"),
                    "livox_frame": LaunchConfiguration("livox_frame"),
                    "map_file_path": LaunchConfiguration("map_file_path"),
                    "use_ros_time_stamps": PythonExpression(
                        ["'", localization_backend, "' == 'icp' or '", localization_backend, "' == 'global_gicp'"]
                    ),
                }.items(),
            ),
            Node(
                package="h1_2_nav_tools",
                executable="fastlio_odom_adapter",
                name="fastlio_odom_adapter",
                output="screen",
                parameters=[
                    {
                        "input_odom_topic": LaunchConfiguration("input_odom_topic"),
                        "output_odom_topic": LaunchConfiguration("output_odom_topic"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "yaw_offset": ParameterValue(LaunchConfiguration("odom_yaw_offset"), value_type=float),
                        "publish_tf": False,
                        "planarize": True,
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_livox",
                arguments=[
                    LaunchConfiguration("livox_x"),
                    LaunchConfiguration("livox_y"),
                    LaunchConfiguration("livox_z"),
                    LaunchConfiguration("livox_yaw"),
                    LaunchConfiguration("livox_pitch"),
                    LaunchConfiguration("livox_roll"),
                    LaunchConfiguration("base_frame"),
                    LaunchConfiguration("livox_frame"),
                ],
                output="screen",
            ),
            Node(
                package="h1_2_nav_tools",
                executable="pointcloud_frame_adapter",
                name="pointcloud_frame_adapter",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("fastlio_body_cloud_topic"),
                        "output_topic": LaunchConfiguration("nav_cloud_topic"),
                        "target_frame": LaunchConfiguration("base_frame"),
                    }
                ],
            ),
            Node(
                package="h1_2_nav_tools",
                executable="simple_pointcloud_to_laserscan",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[p2l_params],
                remappings=[
                    ("/cloud_nav_body", LaunchConfiguration("nav_cloud_topic")),
                    ("/scan", LaunchConfiguration("scan_topic")),
                ],
            ),
            Node(
                package="h1_2_nav_tools",
                executable="odometry_z_offset",
                name="odometry_z_offset",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_odom_topic"),
                        "output_topic": "/slam_odom_visual",
                        "z_offset": ParameterValue(LaunchConfiguration("robot_pelvis_z"), value_type=float),
                    }
                ],
            ),
            Node(
                package="h1_2_nav_tools",
                executable="initialpose_time_adapter",
                name="initialpose_time_adapter",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/rviz_initialpose",
                        "output_topic": "/initialpose",
                        "odom_topic": LaunchConfiguration("input_odom_topic"),
                        "global_frame": "map",
                    }
                ],
            ),
            Node(
                package="h1_2_nav_tools",
                executable="auto_initial_pose",
                name="auto_initial_pose",
                output="screen",
                condition=IfCondition(LaunchConfiguration("auto_initial_pose")),
                parameters=[
                    {
                        "points_file": LaunchConfiguration("memory_points_file"),
                        "start_point_name": LaunchConfiguration("initial_pose_name"),
                        "output_topic": "/initialpose",
                        "odom_topic": LaunchConfiguration("input_odom_topic"),
                        "wait_for_tf_target_frame": "map",
                        "wait_for_tf_source_frame": LaunchConfiguration("odom_frame"),
                        "auto_start_route": ParameterValue(LaunchConfiguration("auto_start_route"), value_type=bool),
                        "route_start_delay_sec": ParameterValue(LaunchConfiguration("route_start_delay_sec"), value_type=float),
                        "start_route_service": "start_memory_route",
                        "localization_backend": LaunchConfiguration("localization_backend"),
                        "icp_status_topic": "/icp_localization/status",
                        "icp_relocalize_service": "icp_relocalize",
                        "icp_pcd_path": LaunchConfiguration("pcd_map"),
                        "wait_for_icp_status": ParameterValue(LaunchConfiguration("wait_for_icp_status"), value_type=bool),
                        "max_icp_score": ParameterValue(LaunchConfiguration("max_icp_score"), value_type=float),
                        "required_stable_samples": ParameterValue(LaunchConfiguration("required_stable_samples"), value_type=int),
                        "localization_timeout_sec": ParameterValue(LaunchConfiguration("localization_timeout_sec"), value_type=float),
                        "max_publish_count": ParameterValue(LaunchConfiguration("initial_pose_publish_count"), value_type=int),
                        "publish_period_sec": ParameterValue(LaunchConfiguration("initial_pose_publish_period_sec"), value_type=float),
                    }
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                condition=pcd_backend_condition,
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "yaml_filename": LaunchConfiguration("map"),
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server",
                output="screen",
                condition=pcd_backend_condition,
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "autostart": LaunchConfiguration("autostart"),
                        "node_names": ["map_server"],
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_navigation_launch),
                condition=pcd_backend_condition,
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": LaunchConfiguration("params_file"),
                    "autostart": LaunchConfiguration("autostart"),
                    "map_subscribe_transient_local": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(icp_localization_launch),
                condition=icp_backend_condition,
                launch_arguments={
                    "pcd_map": LaunchConfiguration("pcd_map"),
                    "map_frame": "map",
                    "odom_frame": LaunchConfiguration("odom_frame"),
                    "base_frame": LaunchConfiguration("base_frame"),
                    "cloud_topic": LaunchConfiguration("fastlio_body_cloud_topic"),
                    "odom_topic": LaunchConfiguration("input_odom_topic"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(global_relocalization_launch),
                condition=global_gicp_backend_condition,
                launch_arguments={
                    "pcd_map": LaunchConfiguration("pcd_map"),
                    "map_frame": "map",
                    "odom_frame": LaunchConfiguration("odom_frame"),
                    "base_frame": LaunchConfiguration("base_frame"),
                    "cloud_topic": LaunchConfiguration("fastlio_body_cloud_topic"),
                    "odom_topic": LaunchConfiguration("input_odom_topic"),
                    "status_topic": "/icp_localization/status",
                    "global_voxel_resolution": LaunchConfiguration("global_voxel_resolution"),
                    "tracking_voxel_resolution": LaunchConfiguration("tracking_voxel_resolution"),
                    "normal_radius": LaunchConfiguration("global_normal_radius"),
                    "feature_radius": LaunchConfiguration("global_feature_radius"),
                    "global_score_threshold": LaunchConfiguration("global_score_threshold"),
                    "gicp_score_threshold": LaunchConfiguration("gicp_score_threshold"),
                    "gicp_max_correspondence_distance": LaunchConfiguration(
                        "gicp_max_correspondence_distance"
                    ),
                    "gicp_max_consecutive_failures": LaunchConfiguration(
                        "gicp_max_consecutive_failures"
                    ),
                    "use_kiss_matcher": LaunchConfiguration("global_use_kiss_matcher"),
                    "use_small_gicp": LaunchConfiguration("global_use_small_gicp"),
                    "min_input_points": LaunchConfiguration("global_min_input_points"),
                    "max_accumulated_points": LaunchConfiguration("global_max_accumulated_points"),
                    "recovery_min_points": LaunchConfiguration("recovery_min_points"),
                    "recovery_cooldown_sec": LaunchConfiguration("recovery_cooldown_sec"),
                    "registration_rate_hz": LaunchConfiguration("global_registration_rate_hz"),
                    "tf_rate_hz": LaunchConfiguration("global_tf_rate_hz"),
                    "transform_tolerance_sec": LaunchConfiguration("global_transform_tolerance_sec"),
                    "num_threads": LaunchConfiguration("global_num_threads"),
                }.items(),
            ),
            Node(
                package="h1_2_nav_tools",
                executable="goal_pose_to_nav2",
                name="goal_pose_to_nav2",
                output="screen",
                parameters=[
                    {
                        "goal_topic": LaunchConfiguration("goal_topic"),
                        "action_name": "navigate_to_pose",
                    }
                ],
            ),
            Node(
                package="h1_2_nav_tools",
                executable="memory_point_manager",
                name="memory_point_manager",
                output="screen",
                parameters=[
                    {
                        "points_file": LaunchConfiguration("memory_points_file"),
                        "odom_topic": LaunchConfiguration("input_odom_topic"),
                        "target_frame": "map",
                    }
                ],
            ),
            Node(
                package="h1_2_nav_tools",
                executable="memory_point_markers",
                name="memory_point_markers",
                output="screen",
                parameters=[
                    {
                        "points_file": LaunchConfiguration("memory_points_file"),
                        "marker_topic": "/memory_points_markers",
                    }
                ],
            ),
            Node(
                package="h1_2_nav_tools",
                executable="memory_route_runner",
                name="memory_route_runner",
                output="screen",
                parameters=[
                    {
                        "points_file": LaunchConfiguration("memory_points_file"),
                        "route_names": LaunchConfiguration("memory_route"),
                        "arrival_wait_sec": LaunchConfiguration("arrival_wait_sec"),
                        "blocked_retry_wait_sec": LaunchConfiguration("blocked_retry_wait_sec"),
                        "max_retries_per_goal": LaunchConfiguration("max_retries_per_goal"),
                        "action_name": "navigate_to_pose",
                        "return_to_start": ParameterValue(LaunchConfiguration("return_to_start"), value_type=bool),
                        "start_point_name": LaunchConfiguration("initial_pose_name"),
                        "required_tf_target_frame": "map",
                        "required_tf_source_frame": LaunchConfiguration("odom_frame"),
                    }
                ],
            ),
            Node(
                package="h1_2_nav_tools",
                executable="velocity_smoother_ema",
                name="velocity_smoother_ema",
                output="screen",
                parameters=[
                    {
                        "raw_cmd_topic": "/cmd_vel",
                        "cmd_topic": "/cmd_vel_nav",
                        "cmd_rate": 20.0,
                        "alpha_v": 0.45,
                        "alpha_w": 0.45,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(loco_bridge_launch),
                launch_arguments={
                    "cmd_topic": "/cmd_vel_nav",
                    "enabled": LaunchConfiguration("enable_sport_bridge"),
                    "start_ros_shim": "true",
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
