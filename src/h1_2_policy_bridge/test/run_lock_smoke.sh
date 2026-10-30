#!/usr/bin/env bash
# Run from the workspace root after building; requires loopback socket access.
set -eo pipefail
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_LOG_DIR=/tmp/h1_policy_ros_logs ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=43
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
mkdir -p /tmp/h1_policy_lock_smoke
node=install/h1_2_policy_bridge/lib/h1_2_policy_bridge/h1_2_policy_node
pid=
cleanup() {
  [[ -z "$pid" ]] || kill "$pid" 2>/dev/null || true
  [[ -z "$pid" ]] || wait "$pid" 2>/dev/null || true
}
trap cleanup EXIT
"$node" > /tmp/h1_policy_lock_smoke/first.log 2>&1 &
pid=$!
sleep 2
kill -0 "$pid"
if "$node" > /tmp/h1_policy_lock_smoke/second.log 2>&1; then
  echo 'second node unexpectedly started' >&2
  exit 1
fi
if ! rg -q 'policy bridge already running' /tmp/h1_policy_lock_smoke/second.log; then
  cat /tmp/h1_policy_lock_smoke/second.log >&2
  exit 1
fi
kill "$pid"
wait "$pid" || true
pid=
"$node" > /tmp/h1_policy_lock_smoke/restart.log 2>&1 &
pid=$!
sleep 2
kill -0 "$pid"
echo 'exclusive startup and restart after release: pass'
