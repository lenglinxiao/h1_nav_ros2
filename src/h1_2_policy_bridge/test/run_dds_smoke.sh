#!/usr/bin/env bash
# Run from the workspace root after a BUILD_TESTING=ON build.
set -eo pipefail
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_LOG_DIR=/tmp/h1_policy_ros_logs ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=43
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
mkdir -p /tmp/h1_policy_smoke
node_pid= fake_pid=
cleanup() {
  [[ -z "$fake_pid" ]] || kill "$fake_pid" 2>/dev/null || true
  [[ -z "$node_pid" ]] || kill "$node_pid" 2>/dev/null || true
  [[ -z "$fake_pid" ]] || wait "$fake_pid" 2>/dev/null || true
  [[ -z "$node_pid" ]] || wait "$node_pid" 2>/dev/null || true
}
trap cleanup EXIT
install/h1_2_policy_bridge/lib/h1_2_policy_bridge/h1_2_policy_node > /tmp/h1_policy_smoke/node.log 2>&1 &
node_pid=$!
build/h1_2_policy_bridge/h1_policy_dds_smoke "${1:-stale}" > /tmp/h1_policy_smoke/fake.log 2>&1 &
fake_pid=$!
sleep 3
timeout 10 ros2 service call /h1_2_policy_node/passive std_srvs/srv/Trigger '{}'
wait "$fake_pid"
fake_pid=
cat /tmp/h1_policy_smoke/fake.log
cat /tmp/h1_policy_smoke/node.log
