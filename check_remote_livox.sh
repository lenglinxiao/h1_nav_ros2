#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/foxy/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
source "$HOME/unitree_ros2/install/setup.bash"
source "$HOME/g1_nav2_ros2_ws/install/setup.bash"

export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS2CLI_DISABLE_DAEMON=1
unset ROS_HOSTNAME
unset ROS_IP
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo

if ! ros2 topic list | grep -qx /livox/lidar; then
  echo "ERROR: /livox/lidar is not visible from this host."
  echo "Check the robot-side ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY, RMW_IMPLEMENTATION, and DDS network interface."
  exit 2
fi

ros2 topic info -v /livox/lidar || true
echo
rm -f /tmp/check_remote_livox_imu.txt
timeout 5s ros2 topic echo --qos-reliability reliable /livox/imu >/tmp/check_remote_livox_imu.txt || true
if [ ! -s /tmp/check_remote_livox_imu.txt ]; then
  echo "ERROR: /livox/imu is visible but no samples were received."
  exit 3
fi
echo "OK: received /livox/imu samples from robot."
