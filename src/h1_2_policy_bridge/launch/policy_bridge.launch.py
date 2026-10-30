from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='/home/llx/policy.onnx'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_final'),
        DeclareLaunchArgument('network_interface', default_value='lo'),
        DeclareLaunchArgument('dds_domain', default_value='42'),
        Node(package='h1_2_policy_bridge', executable='h1_2_policy_node',
             output='screen', additional_env={'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}, parameters=[{
                 'model_path': LaunchConfiguration('model_path'),
                 'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                 'network_interface': LaunchConfiguration('network_interface'),
                 'dds_domain': ParameterValue(LaunchConfiguration('dds_domain'), value_type=int),
             }]),
    ])
