#!/usr/bin/env bash
set -euo pipefail

LOCK_FILE="/tmp/h1_2_ai_slam_nav.lock"

printf 'vel 0.0 0.0 0.0 0.5' | nc -u -w1 127.0.0.1 39001 || true

patterns=(
  "h1_2_slam_bringup fastlio2_grid_navigation.launch.py"
  "install/fast_lio2/lib/fast_lio2/fastlio2_mapping"
  "install/h1_2_nav_tools/lib/h1_2_nav_tools/simple_grid_navigator"
  "install/h1_2_nav_tools/lib/h1_2_nav_tools/slam_grid_navigator"
  "install/h1_2_loco_bridge/lib/h1_2_loco_bridge/h1_2_loco_udp_daemon"
)

for pattern in "${patterns[@]}"; do
  pkill -TERM -f "${pattern}" || true
done

sleep 1

for pattern in "${patterns[@]}"; do
  pkill -KILL -f "${pattern}" || true
done

if command -v lsof >/dev/null 2>&1 && [[ -e "${LOCK_FILE}" ]]; then
  lsof -t "${LOCK_FILE}" 2>/dev/null | xargs -r kill -TERM || true
  sleep 0.2
  lsof -t "${LOCK_FILE}" 2>/dev/null | xargs -r kill -KILL || true
fi

printf 'Stopped H1-2 ai slam nav processes.\n'
