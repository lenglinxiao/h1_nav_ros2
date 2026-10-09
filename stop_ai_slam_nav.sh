#!/usr/bin/env bash
set -euo pipefail

LOCK_FILE="/tmp/h1_2_ai_slam_nav.lock"
PID_FILE="/tmp/h1_2_ai_slam_nav.pid"

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

send_safe_stop

if [[ -f "${PID_FILE}" ]]; then
  launch_pid="$(cat "${PID_FILE}")"
  launch_info=""
  if [[ "${launch_pid}" =~ ^[0-9]+$ ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    launch_info="$(ps -o sid=,pgid=,args= -p "${launch_pid}" 2>/dev/null || true)"
  fi
  read -r launch_sid launch_pgid launch_args <<< "${launch_info}"
  if [[ "${launch_sid}" == "${launch_pid}" && "${launch_pgid}" == "${launch_pid}" && "${launch_args}" == *"ros2 launch h1_2_slam_bringup"* ]]; then
    # Only signal the dedicated session created by run_ai_slam_nav.sh.
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    wait_for_session_exit "${launch_pid}" 5 || {
      kill -TERM -- "-${launch_pid}" 2>/dev/null || true
      wait_for_session_exit "${launch_pid}" 2 || kill -KILL -- "-${launch_pid}" 2>/dev/null || true
    }
    if [[ "$(cat "${PID_FILE}" 2>/dev/null || true)" == "${launch_pid}" ]]; then
      rm -f "${PID_FILE}"
    fi
  fi
fi

printf 'Stopped H1-2 ai slam nav processes.\n'
