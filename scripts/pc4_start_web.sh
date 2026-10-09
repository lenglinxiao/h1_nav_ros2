#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/pc4_env.sh"
exec ros2 run h1_2_nav_tools h1_2_nav_web_gui --host 0.0.0.0 --port 8765 --port-tries 1 "$@"
