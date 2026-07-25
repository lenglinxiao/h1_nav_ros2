import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_share = get_package_share_directory("h1_2_slam_bringup")
    description_share = get_package_share_directory("h1_2_description")
    nav_tools_share = get_package_share_directory("h1_2_nav_tools")
    loco_share = get_package_share_directory("h1_2_loco_bridge")
    nav2_share = get_package_share_directory("nav2_bringup")

    fastlio_launch = os.path.join(bringup_share, "launch", "fastlio2_mid360_mapping.launch.py")
    loco_bridge_launch = os.path.join(loco_share, "launch", "h1_2_loco_bridge.launch.py")
    nav2_navigation_launch = os.path.join(nav2_share, "launch", "navigation_launch.py")
    default_nav2_params = os.path.join(nav_tools_share, "config", "nav2_h1_2_camera_init.yaml")
    default_nav2_bt = os.path.join(
        nav_tools_share,
        "behavior_trees",
        "h1_2_navigate_to_pose_no_spin_recovery.xml",
    )
    configured_nav2_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        param_rewrites={
            "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": default_nav2_bt,
            "bt_navigator.ros__parameters.default_bt_xml_filename": default_nav2_bt,
        },
        convert_types=True,
    )
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_livox_driver", default_value="false"),
            DeclareLaunchArgument("map_file_path", default_value="./test.pcd"),
            DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("odom_topic", default_value="/slam_odom"),
            DeclareLaunchArgument("global_frame", default_value="camera_init"),
            DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
            DeclareLaunchArgument("map_z", default_value="-1.20"),
            DeclareLaunchArgument("grid_width_m", default_value="80.0"),
            DeclareLaunchArgument("grid_height_m", default_value="80.0"),
            DeclareLaunchArgument("grid_origin_x", default_value="-40.0"),
            DeclareLaunchArgument("grid_origin_y", default_value="-40.0"),
            DeclareLaunchArgument("min_z", default_value="-0.35"),
            DeclareLaunchArgument("max_z", default_value="1.50"),
            DeclareLaunchArgument("filter_floating_points", default_value="false"),
            DeclareLaunchArgument("support_min_z", default_value="0.05"),
            DeclareLaunchArgument("support_max_z", default_value="0.30"),
            DeclareLaunchArgument("occupancy_hit_increment", default_value="2"),
            DeclareLaunchArgument("occupancy_miss_decrement", default_value="1"),
            DeclareLaunchArgument("occupancy_threshold", default_value="3"),
            DeclareLaunchArgument("occupancy_max_score", default_value="20"),
            DeclareLaunchArgument("occupied_decay_after_misses", default_value="30"),
            DeclareLaunchArgument("occupied_decay_decrement", default_value="1"),
            DeclareLaunchArgument("enable_sport_bridge", default_value="true"),
            DeclareLaunchArgument("auto_start_loco", default_value="true"),
            DeclareLaunchArgument("network_interface", default_value="enp45s0"),
            DeclareLaunchArgument("domain_id", default_value="0"),
            DeclareLaunchArgument("max_linear_speed", default_value="0.20"),
            DeclareLaunchArgument("max_angular_speed", default_value="0.5"),
            DeclareLaunchArgument("robot_pelvis_z", default_value="-0.22"),
            DeclareLaunchArgument("memory_points_file", default_value="maps/memory_points.json"),
            DeclareLaunchArgument("memory_route", default_value="A,B,C"),
            DeclareLaunchArgument("arrival_wait_sec", default_value="2.0"),
            DeclareLaunchArgument("blocked_retry_wait_sec", default_value="5.0"),
            DeclareLaunchArgument("max_retries_per_goal", default_value="3"),
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
                    "map_file_path": LaunchConfiguration("map_file_path"),
                }.items(),
            ),
            Node(
                package="h1_2_nav_tools",
                executable="pointcloud_to_occupancy_grid",
                name="pointcloud_to_occupancy_grid",
                output="screen",
                parameters=[
                    {
                        "cloud_topic": LaunchConfiguration("cloud_topic"),
                        "map_topic": LaunchConfiguration("map_topic"),
                        "frame_id": LaunchConfiguration("global_frame"),
                        "resolution": 0.05,
                        "width_m": ParameterValue(LaunchConfiguration("grid_width_m"), value_type=float),
                        "height_m": ParameterValue(LaunchConfiguration("grid_height_m"), value_type=float),
                        "origin_x": ParameterValue(LaunchConfiguration("grid_origin_x"), value_type=float),
                        "origin_y": ParameterValue(LaunchConfiguration("grid_origin_y"), value_type=float),
                        "map_z": ParameterValue(LaunchConfiguration("map_z"), value_type=float),
                        "min_z": ParameterValue(LaunchConfiguration("min_z"), value_type=float),
                        "max_z": ParameterValue(LaunchConfiguration("max_z"), value_type=float),
                        "filter_floating_points": ParameterValue(
                            LaunchConfiguration("filter_floating_points"), value_type=bool
                        ),
                        "support_min_z": ParameterValue(LaunchConfiguration("support_min_z"), value_type=float),
                        "support_max_z": ParameterValue(LaunchConfiguration("support_max_z"), value_type=float),
                        "occupancy_hit_increment": ParameterValue(
                            LaunchConfiguration("occupancy_hit_increment"), value_type=int
                        ),
                        "occupancy_miss_decrement": ParameterValue(
                            LaunchConfiguration("occupancy_miss_decrement"), value_type=int
                        ),
                        "occupancy_threshold": ParameterValue(
                            LaunchConfiguration("occupancy_threshold"), value_type=int
                        ),
                        "occupancy_max_score": ParameterValue(
                            LaunchConfiguration("occupancy_max_score"), value_type=int
                        ),
                        "occupied_decay_after_misses": ParameterValue(
                            LaunchConfiguration("occupied_decay_after_misses"), value_type=int
                        ),
                        "occupied_decay_decrement": ParameterValue(
                            LaunchConfiguration("occupied_decay_decrement"), value_type=int
                        ),
                        "publish_hz": 3.0,
                        "inflation_radius": 0.10,
                        "clear_robot_radius": 0.45,
                        "accumulate_map": True,
                        "unknown_value": -1,
                        "raytrace_free": True,
                        "raytrace_clears_occupied": False,
                        "sensor_origin_from_odom": True,
                        "odom_topic": LaunchConfiguration("odom_topic"),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_navigation_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": configured_nav2_params,
                    "autostart": LaunchConfiguration("autostart"),
                    "map_subscribe_transient_local": "true",
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
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "target_frame": "map",
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
                        "arrival_wait_sec": ParameterValue(
                            LaunchConfiguration("arrival_wait_sec"), value_type=float
                        ),
                        "blocked_retry_wait_sec": ParameterValue(
                            LaunchConfiguration("blocked_retry_wait_sec"), value_type=float
                        ),
                        "max_retries_per_goal": ParameterValue(
                            LaunchConfiguration("max_retries_per_goal"), value_type=int
                        ),
                        "action_name": "navigate_to_pose",
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
                        "priority_cmd_topic": "/cmd_vel_voice",
                        "cmd_topic": "/cmd_vel_final",
                        "cmd_rate": 20.0,
                        "alpha_v": 1.0,
                        "alpha_w": 1.0,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(loco_bridge_launch),
                launch_arguments={
                    "cmd_topic": "/cmd_vel_final",
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
