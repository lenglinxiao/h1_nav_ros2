from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    description_share = get_package_share_directory('h1_2_description')
    guard_params = os.path.join(
        get_package_share_directory('h1_2_policy_bridge'), 'config', 'train_guard_20261106.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='/home/llx/policy.onnx'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_final'),
        DeclareLaunchArgument('urdf_path', default_value=os.path.join(description_share, 'h1_2_handless.urdf')),
        DeclareLaunchArgument('profile', default_value='sim'),
        DeclareLaunchArgument('joint_command_mode', default_value=PythonExpression([
            "'position_pd' if '", LaunchConfiguration('profile'), "' == 'hardware' else 'torque'"
        ])),
        DeclareLaunchArgument('enable_hardware_output', default_value='false'),
        DeclareLaunchArgument('max_policy_steps', default_value='0'),
        DeclareLaunchArgument('continuous_policy_navigation', default_value='false'),
        DeclareLaunchArgument('enable_navigation_commands', default_value='false'),
        DeclareLaunchArgument('enable_remote_velocity', default_value='false'),
        DeclareLaunchArgument('protection_profile', default_value='legacy'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.20'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.10'),
        DeclareLaunchArgument('operator_confirmed_debug_mode', default_value='false'),
        DeclareLaunchArgument('operator_confirmed_exclusive_control', default_value='false'),
        DeclareLaunchArgument('network_interface', default_value='lo'),
        DeclareLaunchArgument('dds_domain', default_value='42'),
        Node(package='h1_2_policy_bridge', executable='h1_2_policy_node',
             output='screen', additional_env={'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}, parameters=[guard_params, {
                 'profile': LaunchConfiguration('profile'),
                 'protection_profile': LaunchConfiguration('protection_profile'),
                 'joint_command_mode': LaunchConfiguration('joint_command_mode'),
                 'max_policy_steps': ParameterValue(LaunchConfiguration('max_policy_steps'), value_type=int),
                 'continuous_policy_navigation': ParameterValue(LaunchConfiguration('continuous_policy_navigation'), value_type=bool),
                 'enable_navigation_commands': ParameterValue(LaunchConfiguration('enable_navigation_commands'), value_type=bool),
                 'enable_remote_velocity': ParameterValue(LaunchConfiguration('enable_remote_velocity'), value_type=bool),
                 'enable_hardware_output': ParameterValue(LaunchConfiguration('enable_hardware_output'), value_type=bool),
                 'operator_confirmed_debug_mode': ParameterValue(LaunchConfiguration('operator_confirmed_debug_mode'), value_type=bool),
                 'operator_confirmed_exclusive_control': ParameterValue(LaunchConfiguration('operator_confirmed_exclusive_control'), value_type=bool),
                 'model_path': LaunchConfiguration('model_path'),
                 'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                 'max_linear_speed': ParameterValue(LaunchConfiguration('max_linear_speed'), value_type=float),
                 'max_angular_speed': ParameterValue(LaunchConfiguration('max_angular_speed'), value_type=float),
                 'urdf_path': LaunchConfiguration('urdf_path'),
                 'network_interface': LaunchConfiguration('network_interface'),
                 'dds_domain': ParameterValue(LaunchConfiguration('dds_domain'), value_type=int),
             }]),
    ])
