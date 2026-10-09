#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TARGET="${H1_2_PC4_TARGET:-unitree@192.168.8.6}"
REMOTE_WORKSPACE="${H1_2_PC4_WORKSPACE:-/home/unitree/h1_nav2_ros2_ws}"
ARCHIVE="$(mktemp --suffix=.tar.gz)"
REMOTE_ARCHIVE="/tmp/h1_nav2_ros2_ws-$(date +%s).tar.gz"
cleanup() { rm -f "${ARCHIVE}"; }
trap cleanup EXIT

tar -czf "${ARCHIVE}" \
  --exclude='build' --exclude='install' --exclude='log' \
  --exclude='**/build' --exclude='**/install' --exclude='**/log' \
  --exclude='third_party/small_gicp/build' --exclude='third_party/small_gicp/build_h1_offline' \
  --exclude='*/build/*' --exclude='*/build_*/*' --exclude='**/__pycache__/*' --exclude='**/.pytest_cache/*' \
  -C "${WORKSPACE_DIR}" src third_party config maps scripts run_ai_slam_nav.sh stop_ai_slam_nav.sh
ssh -o BatchMode=yes -o ConnectTimeout=5 "${TARGET}" "mkdir -p '${REMOTE_WORKSPACE}'"
scp -o BatchMode=yes "${ARCHIVE}" "${TARGET}:${REMOTE_ARCHIVE}"
ssh -o BatchMode=yes "${TARGET}" \
  "tar --exclude='maps' -xzf '${REMOTE_ARCHIVE}' -C '${REMOTE_WORKSPACE}' && tar --skip-old-files -xzf '${REMOTE_ARCHIVE}' -C '${REMOTE_WORKSPACE}' maps && rm -f '${REMOTE_ARCHIVE}' && chmod +x '${REMOTE_WORKSPACE}'/scripts/pc4_*.sh '${REMOTE_WORKSPACE}'/run_ai_slam_nav.sh '${REMOTE_WORKSPACE}'/stop_ai_slam_nav.sh"
echo "Copied source/config and missing map files to ${TARGET}:${REMOTE_WORKSPACE}; existing remote maps and tasks were preserved."
