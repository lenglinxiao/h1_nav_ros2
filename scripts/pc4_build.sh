#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/pc4_env.sh"

if [[ -z "${UNITREE_SDK2_ROOT}" || ! -d "${UNITREE_SDK2_ROOT}" ]]; then
  echo "Set UNITREE_SDK2_ROOT to the installed Unitree SDK2 source/build root." >&2
  exit 2
fi
if [[ ! -f "${UNITREE_ROS_SETUP}" ]]; then
  echo "Unitree ROS setup not found: ${UNITREE_ROS_SETUP}" >&2
  exit 2
fi

cd "${WORKSPACE_DIR}"
if [[ "${H1_2_SKIP_ROSDEP:-0}" == "1" ]]; then
  echo "Skipping rosdep install because H1_2_SKIP_ROSDEP=1 was explicitly set."
else
  rosdep install --from-paths src --ignore-src -r -y
fi
colcon build --base-paths src --symlink-install --parallel-workers 2 --cmake-args "-DUNITREE_SDK2_ROOT=${UNITREE_SDK2_ROOT}"
