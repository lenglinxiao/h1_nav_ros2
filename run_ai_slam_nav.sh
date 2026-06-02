#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCK_FILE="/tmp/h1_2_ai_slam_nav.lock"
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
pkill -f "install/fast_lio2/lib/fast_lio2/fastlio2_mapping" || true
pkill -f "install/h1_2_nav_tools/lib/h1_2_nav_tools/simple_grid_navigator" || true
pkill -f "install/h1_2_nav_tools/lib/h1_2_nav_tools/slam_grid_navigator" || true
pkill -f "install/h1_2_loco_bridge/lib/h1_2_loco_bridge/h1_2_loco_udp_daemon" || true
sleep 1

export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_HOSTNAME
unset ROS_IP
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE

exec ros2 launch h1_2_slam_bringup fastlio2_grid_navigation.launch.py \
  start_livox_driver:=false \
  enable_sport_bridge:=true \
  network_interface:=enp45s0 \
  max_linear_speed:=0.12 \
  max_angular_speed:=0.50
