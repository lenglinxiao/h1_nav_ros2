cmake_minimum_required(VERSION 3.5)
project(h1_2_nav_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DeleteMemoryPoint.srv"
  "srv/ExecuteGesture.srv"
  "srv/ListMemoryPoints.srv"
  "srv/RouteStatus.srv"
  "srv/SaveMemoryPoint.srv"
  "srv/StartRoute.srv"
  "srv/StopRoute.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
