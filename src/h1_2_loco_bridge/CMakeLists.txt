cmake_minimum_required(VERSION 3.5)
project(h1_2_loco_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_sdk2 REQUIRED
  PATHS /home/llx/unitree_slam/unitree_robotics/lib/cmake
)

set(UNITREE_ROBOTICS_ROOT "/home/llx/unitree_slam/unitree_robotics")
set(UNITREE_ROBOTICS_LIB_DIR "${UNITREE_ROBOTICS_ROOT}/lib/x86_64")

add_executable(cmd_vel_to_h1_2_loco src/cmd_vel_to_h1_2_loco.cpp)
ament_target_dependencies(cmd_vel_to_h1_2_loco
  geometry_msgs
  rclcpp
)

add_executable(h1_2_loco_udp_daemon src/h1_2_loco_udp_daemon.cpp)
target_include_directories(h1_2_loco_udp_daemon PRIVATE
  ${UNITREE_ROBOTICS_ROOT}/include
)
target_link_directories(h1_2_loco_udp_daemon PRIVATE
  ${UNITREE_ROBOTICS_LIB_DIR}
)
target_link_libraries(h1_2_loco_udp_daemon
  unitree_sdk2
)
set_target_properties(h1_2_loco_udp_daemon PROPERTIES
  BUILD_RPATH "${UNITREE_ROBOTICS_LIB_DIR}"
  INSTALL_RPATH "${UNITREE_ROBOTICS_LIB_DIR}"
)

install(TARGETS cmd_vel_to_h1_2_loco h1_2_loco_udp_daemon
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
