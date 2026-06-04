#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCK_FILE="/tmp/h1_2_ai_slam_nav.lock"
MODE="online"
MAP_FILE=""

for arg in "$@"; do
  case "${arg}" in
    mode=*)
      MODE="${arg#mode=}"
      ;;
    map=*)
      MAP_FILE="${arg#map=}"
      ;;
    *)
      echo "Unknown argument: ${arg}"
      echo "Usage:"
      echo "  $0"
      echo "  $0 mode=online"
      echo "  $0 mode=mapping"
      echo "  $0 mode=navigation map=/absolute/path/to/map.yaml"
      exit 2
      ;;
  esac
done

if [[ "${MODE}" == "mapping" ]]; then
  MODE="online"
fi

if [[ "${MODE}" != "online" && "${MODE}" != "navigation" ]]; then
  echo "Invalid mode: ${MODE}. Use mode=online or mode=navigation."
  exit 2
fi

if [[ "${MODE}" == "navigation" && -z "${MAP_FILE}" ]]; then
  echo "Navigation mode requires map=/absolute/path/to/map.yaml"
  exit 2
fi

if [[ "${MODE}" == "navigation" && ! -f "${MAP_FILE}" ]]; then
  echo "Map file does not exist: ${MAP_FILE}"
  exit 2
fi

exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "H1-2 ai slam nav is already running. Use ${WORKSPACE_DIR}/stop_ai_slam_nav.sh before starting again."
  exit 1
fi

source /opt/ros/foxy/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
source "$HOME/unitree_ros2/install/setup.bash"
source "${WORKSPACE_DIR}/install/setup.bash"

pkill -f "ros2 launch h1_2_slam_bringup fastlio2_grid_navigation.launch.py" || true
pkill -f "ros2 launch h1_2_slam_bringup h1_2_nav2_mapping.launch.py" || true
pkill -f "ros2 launch h1_2_slam_bringup h1_2_nav2_navigation.launch.py" || true
pkill -f "ros2 launch h1_2_slam_bringup h1_2_nav2_slam_navigation.launch.py" || true
pkill -f "ros2 launch h1_2_slam_bringup h1_2_nav2_camera_init_navigation.launch.py" || true
pkill -f "install/fast_lio2/lib/fast_lio2/fastlio2_mapping" || true
pkill -f "install/h1_2_nav_tools/lib/h1_2_nav_tools/fastlio_odom_adapter" || true
pkill -f "install/h1_2_nav_tools/lib/h1_2_nav_tools/pointcloud_frame_adapter" || true
pkill -f "install/h1_2_nav_tools/lib/h1_2_nav_tools/velocity_smoother_ema" || true
pkill -f "install/h1_2_nav_tools/lib/h1_2_nav_tools/pointcloud_to_occupancy_grid" || true
pkill -f "install/h1_2_nav_tools/lib/h1_2_nav_tools/simple_grid_navigator" || true
pkill -f "install/h1_2_nav_tools/lib/h1_2_nav_tools/slam_grid_navigator" || true
pkill -f "install/h1_2_loco_bridge/lib/h1_2_loco_bridge/cmd_vel_to_h1_2_loco" || true
pkill -f "install/h1_2_loco_bridge/lib/h1_2_loco_bridge/h1_2_loco_udp_daemon" || true
pkill -f "nav2_" || true
pkill -f "slam_toolbox" || true
pkill -f "pointcloud_to_laserscan" || true
sleep 1

export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_HOSTNAME
unset ROS_IP
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE

if [[ "${MODE}" == "online" ]]; then
  echo "Starting H1-2 camera_init map + Nav2 navigation..."
  exec ros2 launch h1_2_slam_bringup h1_2_nav2_camera_init_navigation.launch.py \
    start_livox_driver:=false \
    enable_sport_bridge:=true \
    network_interface:=enp45s0 \
    max_linear_speed:=0.12 \
    max_angular_speed:=0.50
fi

echo "Starting H1-2 Nav2 navigation with map: ${MAP_FILE}"
exec ros2 launch h1_2_slam_bringup h1_2_nav2_navigation.launch.py \
  map:="${MAP_FILE}" \
  start_livox_driver:=false \
  enable_sport_bridge:=true \
  network_interface:=enp45s0 \
  max_linear_speed:=0.12 \
  max_angular_speed:=0.50 \
  odom_yaw_offset:=3.141592653589793
