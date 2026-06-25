from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument(
                "annotated_topic",
                default_value="/h1_2/vision/annotated_image",
            ),
            DeclareLaunchArgument(
                "detections_topic",
                default_value="/h1_2/vision/detections",
            ),
            DeclareLaunchArgument("model_path", default_value="~/ros2_ws/models/yolov8n.pt"),
            DeclareLaunchArgument("confidence_threshold", default_value="0.40"),
            DeclareLaunchArgument("iou_threshold", default_value="0.45"),
            DeclareLaunchArgument("image_size", default_value="640"),
            DeclareLaunchArgument("max_inference_rate", default_value="10.0"),
            DeclareLaunchArgument("device", default_value="cpu"),
            Node(
                package="h1_2_vision",
                executable="yolo_detector",
                name="h1_2_yolo_detector",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "annotated_topic": LaunchConfiguration("annotated_topic"),
                        "detections_topic": LaunchConfiguration("detections_topic"),
                        "model_path": LaunchConfiguration("model_path"),
                        "confidence_threshold": ParameterValue(
                            LaunchConfiguration("confidence_threshold"),
                            value_type=float,
                        ),
                        "iou_threshold": ParameterValue(
                            LaunchConfiguration("iou_threshold"),
                            value_type=float,
                        ),
                        "image_size": ParameterValue(
                            LaunchConfiguration("image_size"),
                            value_type=int,
                        ),
                        "max_inference_rate": ParameterValue(
                            LaunchConfiguration("max_inference_rate"),
                            value_type=float,
                        ),
                        "device": LaunchConfiguration("device"),
                    }
                ],
            ),
        ]
    )
