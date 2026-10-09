#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${H1_2_PC4_ENV_FILE:-${HOME}/.config/h1-2/pc4.env}"

if [[ ! -f "${ENV_FILE}" ]]; then
  echo "PC4 environment file not found: ${ENV_FILE}" >&2
  echo "Copy ${WORKSPACE_DIR}/config/pc4.env.example and edit its paths." >&2
  return 2 2>/dev/null || exit 2
fi

set -a
# shellcheck disable=SC1090
source "${ENV_FILE}"
set +a

export H1_2_RUNTIME_PROFILE="${H1_2_RUNTIME_PROFILE:-pc4}"
export H1_2_WORKSPACE="${H1_2_WORKSPACE:-${WORKSPACE_DIR}}"
export UNITREE_ROS_SETUP="${UNITREE_ROS_SETUP:-${HOME}/unitree_ros2/install/setup.bash}"
export UNITREE_SDK2_ROOT="${UNITREE_SDK2_ROOT:-}"
export H1_2_NETWORK_INTERFACE="${H1_2_NETWORK_INTERFACE:-eth0}"
export H1_2_DDS_CONFIG="${H1_2_DDS_CONFIG:-${H1_2_WORKSPACE}/config/cyclonedds_auto.xml}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export H1_2_WEB_HEARTBEAT_TIMEOUT_SEC="${H1_2_WEB_HEARTBEAT_TIMEOUT_SEC:-2}"
export H1_2_NAV_WS="${H1_2_NAV_WS:-${H1_2_WORKSPACE}}"

had_nounset=0
[[ $- == *u* ]] && had_nounset=1
set +u
source /opt/ros/humble/setup.bash
if [[ -f "${UNITREE_ROS_SETUP}" ]]; then
  source "${UNITREE_ROS_SETUP}"
fi
if [[ -f "${H1_2_WORKSPACE}/install/setup.bash" ]]; then
  source "${H1_2_WORKSPACE}/install/setup.bash"
fi
(( had_nounset == 0 )) || set -u
