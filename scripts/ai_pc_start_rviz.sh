#!/usr/bin/env bash
set -eo pipefail
case "${1:-}" in online|offline) RVIZ_MODE="$1" ;; *) exit 2 ;; esac
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://${WORKSPACE_DIR}/config/cyclonedds_ai_pc.xml"
unset ROS_HOSTNAME ROS_IP ROS_MASTER_URI
exec ros2 launch h1_2_slam_bringup h1_2_nav2_rviz.launch.py "mode:=${RVIZ_MODE}"
