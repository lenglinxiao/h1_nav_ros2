#!/usr/bin/env bash
set -euo pipefail

LOCK_FILE="/tmp/h1_2_ai_slam_nav.lock"
PID_FILE="/tmp/h1_2_ai_slam_nav.pid"
BACKEND_FILE="/tmp/h1_2_ai_slam_nav.backend"
policy_session_held=false
unitree_stop_sent=false

send_safe_stop() {
  python3 -c 'import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.sendto(b"vel 0.0 0.0 0.0 0.5",("127.0.0.1",39001)); s.sendto(b"stop",("127.0.0.1",39001)); s.close()' || true
}

wait_for_session_exit() {
  local launch_pid="$1"
  local timeout_sec="$2"
  local deadline=$((SECONDS + timeout_sec))
  while kill -0 -- "-${launch_pid}" 2>/dev/null; do
    (( SECONDS >= deadline )) && return 1
    sleep 0.1
  done
}

nav_launch_candidates() {
  ps -eo pid=,sid=,pgid=,args= | awk '
    $1 == $2 && $1 == $3 &&
    /ros2 launch h1_2_slam_bringup h1_2_nav2_(camera_init_|slam_)?navigation\.launch\.py/ { print $1 }
  '
}

is_h1_navigation_launch() {
  local command="$1"
  [[ "${command}" == *"ros2 launch h1_2_slam_bringup h1_2_nav2_camera_init_navigation.launch.py"* ||
     "${command}" == *"ros2 launch h1_2_slam_bringup h1_2_nav2_navigation.launch.py"* ||
     "${command}" == *"ros2 launch h1_2_slam_bringup h1_2_nav2_slam_navigation.launch.py"* ]]
}

remaining_navigation_processes() {
  while read -r process_pid command; do
    [[ "${process_pid}" == "$$" || "${command}" == *"stop_ai_slam_nav.sh"* ]] && continue
    case "${command}" in
      *fastlio2_mapping*|*pointcloud_to_occupancy_grid*|*memory_route_runner*|*goal_pose_to_nav2*|\
      *nav2_controller/controller_server*|*nav2_planner/planner_server*|*nav2_behaviors/behavior_server*|\
      *nav2_bt_navigator/bt_navigator*|*nav2_smoother/smoother_server*|*nav2_waypoint_follower/waypoint_follower*|\
      *nav2_velocity_smoother/velocity_smoother*|*h1_2_nav2_camera_init_navigation.launch.py*|\
      *h1_2_nav2_navigation.launch.py*|*h1_2_nav2_slam_navigation.launch.py*)
        printf '%s %s\n' "${process_pid}" "${command}"
        ;;
    esac
  done < <(ps -eo pid=,args=)
}

launch_pid=""
launch_info=""
if [[ -f "${PID_FILE}" ]]; then
  launch_pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
  if [[ "${launch_pid}" =~ ^[0-9]+$ ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    launch_info="$(ps -o sid=,pgid=,args= -p "${launch_pid}" 2>/dev/null || true)"
  fi
fi
read -r launch_sid launch_pgid launch_args <<< "${launch_info}"
if [[ "${launch_sid}" != "${launch_pid}" || "${launch_pgid}" != "${launch_pid}" ]] ||
   ! is_h1_navigation_launch "${launch_args}"; then
  launch_pid=""
  launch_info=""
  candidates="$(nav_launch_candidates)"
  candidate_count="$(printf '%s\n' "${candidates}" | awk 'NF { count++ } END { print count + 0 }')"
  if [[ "${candidate_count}" -gt 1 ]]; then
    printf 'Multiple isolated H1-2 navigation launch groups found; refusing ambiguous stop.\n%s\n' "${candidates}" >&2
    exit 1
  elif [[ "${candidate_count}" -eq 1 ]]; then
    launch_pid="${candidates}"
    launch_info="$(ps -o sid=,pgid=,args= -p "${launch_pid}" 2>/dev/null || true)"
  fi
fi

if [[ -n "${launch_info}" ]]; then
  read -r launch_sid launch_pgid launch_args <<< "${launch_info}"
  if [[ "${launch_sid}" == "${launch_pid}" && "${launch_pgid}" == "${launch_pid}" ]] &&
     is_h1_navigation_launch "${launch_args}"; then
    backend_info="$(cat "${BACKEND_FILE}" 2>/dev/null || true)"
    read -r backend_pid motion_backend <<< "${backend_info}"
    if [[ "${backend_pid}" != "${launch_pid}" ]]; then
      motion_backend="unitree"
      [[ "${launch_args}" == *"motion_backend:=policy"* ]] && motion_backend="policy"
    fi
    if [[ "${motion_backend}" == "policy" ]]; then
      set +u
      source /opt/ros/humble/setup.bash
      source "${HOME}/unitree_ros2/install/setup.bash"
      source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install/setup.bash"
      set -u
      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
      export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
      unset CYCLONEDDS_URI
      unset FASTRTPS_DEFAULT_PROFILES_FILE ROS_HOSTNAME ROS_IP
      policy_status="$(timeout 4s ros2 service call /h1_2_policy_node/status std_srvs/srv/Trigger "{}" 2>/dev/null || true)"
      policy_already_held=false
      policy_output_inhibited=false
      if [[ "${policy_status}" == *"success=True"* && "${policy_status}" == *"mode=ArmHold"* &&
            "${policy_status}" == *"authorized=true"* && "${policy_status}" == *"publisher_present=true"* ]]; then
        policy_already_held=true
      elif [[ "${policy_status}" == *"success=True"* && "${policy_status}" == *"authorized=false"* &&
              "${policy_status}" == *"publisher_present=false"* ]]; then
        policy_output_inhibited=true
      else
        hold_response="$(timeout 4s ros2 service call /h1_2_policy_node/arm_hold std_srvs/srv/Trigger "{}" 2>&1 || true)"
        if [[ "${hold_response}" != *"success=True"* ]]; then
          printf 'Policy state is unknown or ArmHold was not accepted; leaving navigation running.\n%s\n' "${hold_response}" >&2
          exit 1
        fi
        policy_status="$(timeout 4s ros2 service call /h1_2_policy_node/status std_srvs/srv/Trigger "{}" 2>/dev/null || true)"
        if [[ "${policy_status}" != *"success=True"* || "${policy_status}" != *"mode=ArmHold"* ||
              "${policy_status}" != *"authorized=true"* || "${policy_status}" != *"publisher_present=true"* ]]; then
          printf 'Policy bridge did not confirm ArmHold; leaving navigation running.\n' >&2
          exit 1
        fi
      fi
      policy_session_held=true
      if [[ "${policy_output_inhibited}" == true ]]; then
        printf 'Policy output is already inhibited; stopping navigation without issuing control commands.\n'
      elif [[ "${policy_already_held}" == true ]]; then
        printf 'Policy ArmHold is confirmed; stopping navigation.\n'
      fi
    else
      send_safe_stop
      unitree_stop_sent=true
    fi
    # Only signal the dedicated session created by run_ai_slam_nav.sh.
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    if [[ "${motion_backend}" == "policy" ]]; then
      wait_for_session_exit "${launch_pid}" 8 || {
        printf 'Policy session did not exit after hold; leaving its process group running.\n' >&2
        exit 1
      }
    else
      wait_for_session_exit "${launch_pid}" 5 || {
        kill -TERM -- "-${launch_pid}" 2>/dev/null || true
        wait_for_session_exit "${launch_pid}" 2 || kill -KILL -- "-${launch_pid}" 2>/dev/null || true
      }
    fi
    if [[ "$(cat "${PID_FILE}" 2>/dev/null || true)" == "${launch_pid}" ]]; then
      rm -f "${PID_FILE}"
    fi
    if [[ "$(awk '{print $1}' "${BACKEND_FILE}" 2>/dev/null || true)" == "${launch_pid}" ]]; then
      rm -f "${BACKEND_FILE}"
    fi
  fi
fi

remaining="$(remaining_navigation_processes)"
if [[ -n "${remaining}" ]]; then
  printf 'Navigation-related processes remain; refusing to report a successful stop:\n%s\n' "${remaining}" >&2
  exit 1
fi

policy_bridge_running=false
if [[ -f /tmp/h1_2_policy_bridge.pid ]]; then
  policy_bridge_pid="$(cat /tmp/h1_2_policy_bridge.pid 2>/dev/null || true)"
  if [[ "${policy_bridge_pid}" =~ ^[0-9]+$ ]] && kill -0 "${policy_bridge_pid}" 2>/dev/null &&
      [[ "$(ps -o args= -p "${policy_bridge_pid}" 2>/dev/null || true)" == *"policy_bridge.launch.py"* ]]; then
    policy_bridge_running=true
  fi
fi

if [[ "${policy_session_held}" != true && "${unitree_stop_sent}" != true && "${policy_bridge_running}" != true ]]; then
  send_safe_stop
fi

printf 'Stopped H1-2 ai slam nav processes.\n'
