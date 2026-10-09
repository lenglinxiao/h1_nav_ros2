#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SERVICE_TEMPLATE="${WORKSPACE_DIR}/config/systemd/h1-2-web.service.in"
TARGET_USER="${USER}"
TARGET_HOME="$(getent passwd "${TARGET_USER}" | cut -d: -f6)"
ENV_FILE="${TARGET_HOME}/.config/h1-2/pc4.env"
UNIT_FILE="h1-2-web.service"

if [[ ! -f "${ENV_FILE}" ]]; then
  install -d -m 0750 "$(dirname "${ENV_FILE}")"
  install -m 0640 "${WORKSPACE_DIR}/config/pc4.env.example" "${ENV_FILE}"
  sed -i "s#^H1_2_WORKSPACE=.*#H1_2_WORKSPACE=${WORKSPACE_DIR}#; s#^H1_2_NAV_WS=.*#H1_2_NAV_WS=${WORKSPACE_DIR}#; s#^H1_2_DDS_CONFIG=.*#H1_2_DDS_CONFIG=${WORKSPACE_DIR}/config/cyclonedds_pc4.xml#" "${ENV_FILE}"
  echo "Created ${ENV_FILE}; review UNITREE_ROS_SETUP and UNITREE_SDK2_ROOT before starting." >&2
fi

USER_UNIT_DIR="${TARGET_HOME}/.config/systemd/user"
install -d -m 0755 "${USER_UNIT_DIR}"
sed -e "s#@ENV_FILE@#${ENV_FILE}#g" -e "s#@WORKSPACE@#${WORKSPACE_DIR}#g" "${SERVICE_TEMPLATE}" > "${USER_UNIT_DIR}/${UNIT_FILE}"
systemctl --user daemon-reload
systemctl --user enable --now "${UNIT_FILE}"

if [[ "$(loginctl show-user "${TARGET_USER}" -p Linger --value 2>/dev/null || true)" != "yes" ]]; then
  echo "User service is enabled for this session; reboot autostart requires loginctl enable-linger ${TARGET_USER} (admin action)." >&2
fi
