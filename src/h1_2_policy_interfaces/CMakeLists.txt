cmake_minimum_required(VERSION 3.8)
project(h1_2_policy_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SessionToken.msg"
  "msg/PolicySourceState.msg"
  "srv/PrepareMotionSource.srv"
  "srv/ActivateMotionSource.srv"
  "srv/ClaimPolicyInput.srv"
  "srv/PolicyRuntimeIdentity.srv"
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
