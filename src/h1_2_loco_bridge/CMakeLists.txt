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
find_package(h1_2_nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)

# YAML-CPP for configuration parsing (optional, we use simple parser)
# find_package(yaml-cpp REQUIRED)

if(NOT UNITREE_SDK2_ROOT)
  if(DEFINED ENV{UNITREE_SDK2_ROOT})
    set(UNITREE_SDK2_ROOT "$ENV{UNITREE_SDK2_ROOT}")
  endif()
endif()
set(UNITREE_SDK2_ROOT "${UNITREE_SDK2_ROOT}" CACHE PATH "Path to Unitree SDK2")
if(NOT UNITREE_SDK2_ROOT)
  message(FATAL_ERROR "Set UNITREE_SDK2_ROOT to the Unitree SDK2 directory (CMake -D or environment).")
endif()
set(UNITREE_SDK2_LIB_DIR "${UNITREE_SDK2_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}")
set(UNITREE_SDK2_THIRDPARTY_LIB_DIR "${UNITREE_SDK2_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")
set(UNITREE_SDK2_THIRDPARTY_INCLUDE_DIR "${UNITREE_SDK2_ROOT}/thirdparty/include")

find_library(UNITREE_SDK2_LIB unitree_sdk2 PATHS "${UNITREE_SDK2_LIB_DIR}" NO_DEFAULT_PATH)
find_library(UNITREE_DDSCXX_LIB ddscxx PATHS "${UNITREE_SDK2_THIRDPARTY_LIB_DIR}" NO_DEFAULT_PATH)
find_library(UNITREE_DDSC_LIB ddsc PATHS "${UNITREE_SDK2_THIRDPARTY_LIB_DIR}" NO_DEFAULT_PATH)
if(NOT UNITREE_SDK2_LIB OR NOT UNITREE_DDSCXX_LIB OR NOT UNITREE_DDSC_LIB)
  message(FATAL_ERROR "unitree_sdk2 libraries not found under ${UNITREE_SDK2_ROOT}")
endif()

add_executable(cmd_vel_to_h1_2_loco src/cmd_vel_to_h1_2_loco.cpp)
ament_target_dependencies(cmd_vel_to_h1_2_loco
  geometry_msgs
  rclcpp
)

add_executable(h1_2_loco_udp_daemon src/h1_2_loco_udp_daemon.cpp)
target_include_directories(h1_2_loco_udp_daemon PRIVATE
  ${UNITREE_SDK2_ROOT}/include
  ${UNITREE_SDK2_THIRDPARTY_INCLUDE_DIR}
  ${UNITREE_SDK2_THIRDPARTY_INCLUDE_DIR}/ddscxx
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(h1_2_loco_udp_daemon
  ${UNITREE_SDK2_LIB}
  ${UNITREE_DDSCXX_LIB}
  ${UNITREE_DDSC_LIB}
  Threads::Threads
)
set_target_properties(h1_2_loco_udp_daemon PROPERTIES
  BUILD_RPATH "${UNITREE_SDK2_THIRDPARTY_LIB_DIR};${UNITREE_SDK2_LIB_DIR}"
  INSTALL_RPATH "${UNITREE_SDK2_THIRDPARTY_LIB_DIR};${UNITREE_SDK2_LIB_DIR}"
)

add_executable(grab_pull_demo src/grab_pull_demo.cpp)
target_include_directories(grab_pull_demo PRIVATE
  ${UNITREE_SDK2_ROOT}/include
  ${UNITREE_SDK2_THIRDPARTY_INCLUDE_DIR}
  ${UNITREE_SDK2_THIRDPARTY_INCLUDE_DIR}/ddscxx
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(grab_pull_demo
  ${UNITREE_SDK2_LIB}
  ${UNITREE_DDSCXX_LIB}
  ${UNITREE_DDSC_LIB}
  Threads::Threads
)
set_target_properties(grab_pull_demo PROPERTIES
  BUILD_RPATH "${UNITREE_SDK2_THIRDPARTY_LIB_DIR};${UNITREE_SDK2_LIB_DIR}"
  INSTALL_RPATH "${UNITREE_SDK2_THIRDPARTY_LIB_DIR};${UNITREE_SDK2_LIB_DIR}"
)

add_executable(gesture_to_h1_2_loco src/gesture_to_h1_2_loco.cpp)
ament_target_dependencies(gesture_to_h1_2_loco
  h1_2_nav_msgs
  rclcpp
)

install(TARGETS cmd_vel_to_h1_2_loco h1_2_loco_udp_daemon gesture_to_h1_2_loco grab_pull_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_arm_trajectory test/test_arm_trajectory.cpp)
  target_include_directories(test_arm_trajectory PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
endif()

ament_package()
