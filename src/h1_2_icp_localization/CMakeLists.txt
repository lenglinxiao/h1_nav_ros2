cmake_minimum_required(VERSION 3.5)
project(h1_2_icp_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(H1_USE_KISS_MATCHER "Build optional KISS-Matcher backend" OFF)
option(H1_USE_SMALL_GICP "Build optional small_gicp backend" OFF)
get_filename_component(H1_WORKSPACE_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(H1_KISS_MATCHER_PREFIX
  "${H1_WORKSPACE_DIR}/install/kiss_matcher"
  CACHE PATH
  "KISS-Matcher install prefix"
)
if(EXISTS "${H1_KISS_MATCHER_PREFIX}")
  list(PREPEND CMAKE_PREFIX_PATH "${H1_KISS_MATCHER_PREFIX}")
endif()
set(H1_SMALL_GICP_PREFIX
  "${H1_WORKSPACE_DIR}/install/small_gicp"
  CACHE PATH
  "small_gicp install prefix"
)
if(EXISTS "${H1_SMALL_GICP_PREFIX}")
  list(PREPEND CMAKE_PREFIX_PATH "${H1_SMALL_GICP_PREFIX}")
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features filters io registration sample_consensus search)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robin QUIET)
find_package(TBB QUIET)
find_package(kiss_matcher QUIET)
find_package(small_gicp QUIET)

if(robin_FOUND AND TBB_FOUND AND kiss_matcher_FOUND)
  set(H1_HAVE_KISS_MATCHER TRUE)
  message(STATUS "KISS-Matcher found at: ${H1_KISS_MATCHER_PREFIX}")
else()
  set(H1_HAVE_KISS_MATCHER FALSE)
  if(H1_USE_KISS_MATCHER)
    message(FATAL_ERROR
      "H1_USE_KISS_MATCHER is ON, but robin/TBB/kiss_matcher were not all found. "
      "Set H1_KISS_MATCHER_PREFIX or CMAKE_PREFIX_PATH to the KISS-Matcher install prefix."
    )
  endif()
  message(STATUS "KISS-Matcher optional backend disabled; using PCL backend only.")
endif()

if(small_gicp_FOUND)
  set(H1_HAVE_SMALL_GICP TRUE)
  message(STATUS "small_gicp found at: ${H1_SMALL_GICP_PREFIX}")
else()
  set(H1_HAVE_SMALL_GICP FALSE)
  if(H1_USE_SMALL_GICP)
    message(FATAL_ERROR
      "H1_USE_SMALL_GICP is ON, but small_gicp was not found. "
      "Set H1_SMALL_GICP_PREFIX or CMAKE_PREFIX_PATH to the small_gicp install prefix."
    )
  endif()
  message(STATUS "small_gicp optional backend disabled; using PCL backend only.")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalizationStatus.msg"
  "srv/Relocalize.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_library(icp_localizer
  src/icp_localizer.cpp
)
ament_target_dependencies(icp_localizer
  pcl_conversions
)
target_link_libraries(icp_localizer
  ${PCL_LIBRARIES}
)

add_executable(icp_localization_node
  src/icp_localization_node.cpp
)
ament_target_dependencies(icp_localization_node
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(icp_localization_node
  icp_localizer
  ${PCL_LIBRARIES}
)

rosidl_target_interfaces(icp_localization_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(global_relocalization_node
  src/global_relocalization_node.cpp
)
ament_target_dependencies(global_relocalization_node
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(global_relocalization_node
  ${PCL_LIBRARIES}
)

if(H1_HAVE_KISS_MATCHER)
  target_compile_definitions(global_relocalization_node PRIVATE H1_HAVE_KISS_MATCHER)
  target_link_libraries(global_relocalization_node
    kiss_matcher::kiss_matcher_core
  )
endif()

if(H1_HAVE_SMALL_GICP)
  target_compile_definitions(global_relocalization_node PRIVATE H1_HAVE_SMALL_GICP)
  target_link_libraries(global_relocalization_node
    small_gicp::small_gicp
  )
endif()

rosidl_target_interfaces(global_relocalization_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS icp_localizer icp_localization_node global_relocalization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  ament_add_gtest(test_relocalization_state
    test/test_relocalization_state.cpp
  )
  target_include_directories(test_relocalization_state PUBLIC include)
  target_link_libraries(test_relocalization_state Eigen3::Eigen)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
