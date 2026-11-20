#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_PROFILE="${H1_2_RUNTIME_PROFILE:-legacy}"
NETWORK_INTERFACE="${H1_2_NETWORK_INTERFACE:-}"
if [[ "${RUNTIME_PROFILE}" == "pc4" && -z "${NETWORK_INTERFACE}" ]]; then
  NETWORK_INTERFACE="eth0"
fi
UNITREE_ROS_SETUP="${UNITREE_ROS_SETUP:-${HOME}/unitree_ros2/install/setup.bash}"
CYCLONEDDS_CONFIG="${H1_2_DDS_CONFIG:-${WORKSPACE_DIR}/config/cyclonedds_auto.xml}"
LOCK_FILE="/tmp/h1_2_ai_slam_nav.lock"
PID_FILE="/tmp/h1_2_ai_slam_nav.pid"
BACKEND_FILE="/tmp/h1_2_ai_slam_nav.backend"
POLICY_BRIDGE_PID_FILE="/tmp/h1_2_policy_bridge.pid"
MODE="online"
STOP_POLICY_BRIDGE="false"
MAP_FILE=""
MAP_SAVE_PREFIX=""
PCD_SAVE_PREFIX="maps/h1_2_lab"
LOCALIZATION_BACKEND="global_gicp"
MOTION_BACKEND="unitree"
PCD_MAP_FILE=""
INITIAL_POSE_NAME="START"
MEMORY_ROUTE="A,B"
TURN_ROUTE="B:ccw,A:cw"
TURN_ROUTE_SET="false"
START_AT_FIRST_ROUTE_POINT="true"
GESTURE_ROUTE=""
GESTURE_HOLD_SEC="3.0"
AUTO_START_ROUTE="false"
RETURN_TO_START="false"
NAV2_PARAMS_FILE=""
MEMORY_POINTS_FILE="${WORKSPACE_DIR}/maps/memory_points.json"
POLICY_DEBUG_CONFIRMED="false"
POLICY_EXCLUSIVE_CONFIRMED="false"
ENABLE_REMOTE_VELOCITY="false"
POLICY_MODEL_PATH="${H1_2_POLICY_MODEL_PATH:-${WORKSPACE_DIR%%/releases/*}/models/h1_2_armhold/policy.onnx}"
LAUNCH_PID=""

source_ros_environment() {
  local had_nounset=0
  [[ $- == *u* ]] && had_nounset=1
  set +u
  source /opt/ros/humble/setup.bash
  source "${UNITREE_ROS_SETUP}"
  source "${WORKSPACE_DIR}/install/setup.bash"
  (( had_nounset == 0 )) || set -u
}

launch_session_is_owned() {
  local launch_info
  [[ -n "${LAUNCH_PID}" && -f "${PID_FILE}" ]] || return 1
  [[ "$(cat "${PID_FILE}" 2>/dev/null || true)" == "${LAUNCH_PID}" ]] || return 1
  kill -0 "${LAUNCH_PID}" 2>/dev/null || return 1
  local launch_sid launch_pgid launch_args
  launch_info="$(ps -o sid=,pgid=,args= -p "${LAUNCH_PID}" 2>/dev/null || true)"
  read -r launch_sid launch_pgid launch_args <<< "${launch_info}"
  [[ "${launch_sid}" == "${LAUNCH_PID}" && "${launch_pgid}" == "${LAUNCH_PID}" ]] &&
    [[ "${launch_args}" == *"ros2 launch h1_2_slam_bringup"* ]]
}

wait_for_session_exit() {
  local timeout_sec="$1"
  local deadline=$((SECONDS + timeout_sec))
  while kill -0 -- "-${LAUNCH_PID}" 2>/dev/null; do
    (( SECONDS >= deadline )) && return 1
    sleep 0.1
  done
}

cleanup_launch() {
  local status=$?
  if launch_session_is_owned; then
    if [[ "${MOTION_BACKEND}" == "policy" ]]; then
      policy_status="$(env RMW_IMPLEMENTATION=rmw_fastrtps_cpp timeout 4s ros2 service call /h1_2_policy_node/status std_srvs/srv/Trigger "{}" 2>/dev/null || true)"
      if [[ "${policy_status}" != *"mode=ArmHold"* ]] &&
          ! env RMW_IMPLEMENTATION=rmw_fastrtps_cpp timeout 4s ros2 service call /h1_2_policy_node/arm_hold std_srvs/srv/Trigger "{}" | rg -q 'success=True'; then
        echo "Policy bridge did not confirm ArmHold; leaving navigation running." >&2
        return "${status}"
      fi
    fi
    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
    if [[ "${MOTION_BACKEND}" == "policy" ]]; then
      wait_for_session_exit 8 || echo "Policy navigation stack did not exit after ArmHold; leaving it running." >&2
    else
      wait_for_session_exit 5 || {
        kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
        wait_for_session_exit 2 || kill -KILL -- "-${LAUNCH_PID}" 2>/dev/null || true
      }
    fi
  fi
  if [[ -n "${LAUNCH_PID}" && -f "${PID_FILE}" && "$(cat "${PID_FILE}" 2>/dev/null || true)" == "${LAUNCH_PID}" ]]; then
    rm -f "${PID_FILE}"
  fi
  if [[ -n "${LAUNCH_PID}" && -f "${BACKEND_FILE}" && "$(awk '{print $1}' "${BACKEND_FILE}" 2>/dev/null || true)" == "${LAUNCH_PID}" ]]; then
    rm -f "${BACKEND_FILE}"
  fi
  return "${status}"
}

print_usage() {
  cat <<'EOF'
Usage:
  ./run_ai_slam_nav.sh
  ./run_ai_slam_nav.sh mode=online
  ./run_ai_slam_nav.sh mode=mapping
  ./run_ai_slam_nav.sh mode=save_map map_prefix=maps/h1_2_lab
  ./run_ai_slam_nav.sh mode=save_pcd_map pcd_prefix=maps/h1_2_lab
  ./run_ai_slam_nav.sh mode=navigation map=/absolute/path/to/map.yaml localization=global_gicp pcd=/absolute/path/to/map.pcd
  ./run_ai_slam_nav.sh mode=navigation map=/absolute/path/to/map.yaml localization=icp pcd=/absolute/path/to/map.pcd
  ./run_ai_slam_nav.sh mode=navigation motion_backend=policy map=/absolute/path/to/map.yaml localization=icp pcd=/absolute/path/to/map.pcd
  ./run_ai_slam_nav.sh mode=policy_bridge_only motion_backend=policy confirm_debug_mode=true confirm_exclusive_control=true
  ./run_ai_slam_nav.sh stop_policy_bridge=true  # after onsite support and explicit revoke
  ./run_ai_slam_nav.sh mode=navigation map=/absolute/path/to/map.yaml pcd=/absolute/path/to/map.pcd initial_pose=START memory_route=A,B,C gesture_route=B:wave_hand,C:shake_hand auto_start_route=true

Modes:
  mode=online      Online camera_init mapping + Nav2 navigation. This is the default.
  mode=mapping     Compatibility alias for mode=online.
  mode=save_map    Save the currently published /map to map_prefix. Does not stop online mode.
  mode=save_pcd_map Save the current FAST-LIO2 3D PCD map to pcd_prefix. Does not stop online mode.
  mode=navigation  Offline fixed-map Nav2 navigation. Requires map=...
  mode=policy_bridge_only  Start only the ONNX Policy bridge; no Nav2 or route is started.
  pcd_prefix=...   Prefix for FAST-LIO2 PCD map save. Default: maps/h1_2_lab.
  localization=... Offline localization backend: global_gicp or icp. Default: global_gicp.
  motion_backend=... Startup motion controller: unitree or policy. Default: unitree.
  confirm_debug_mode=true confirm_exclusive_control=true
                    Required for Policy; records explicit operator acknowledgements.
  enable_remote_velocity=true
                    Optional Policy input arbitration: a non-neutral physical remote stick
                    is merged inside the single Policy bridge; no second LowCmd writer.
  pcd=...          3D PCD map for offline localization.
  initial_pose=... START memory point used for automatic PCD localization initialization. Default: START.
  memory_route=... Route names for memory route runner. Default: A,B.
  memory_points_file=... JSON file used to save and load points for this task.
  turn_route=...   Arrival turns, using cw/ccw, for example B:ccw,A:cw.
  gesture_route=... Optional per-point gestures; use + to sequence actions, for example A:reach_hand+retract_hand.
  gesture_hold_sec=... Hold time between sequenced gestures (seconds). Default: 3.0.
  auto_start_route=true|false Start memory route after localization TF is available. Default: false.
  return_to_start=true|false Navigate to initial_pose before looping memory_route. Default: false.
  params_file=...  Optional Nav2 params file. Leave empty to use launch defaults.

Offline map demo workflow:
  1. Start online mode and build the site map.
  2. Save the map:
     ./run_ai_slam_nav.sh mode=save_map map_prefix=maps/h1_2_lab
     ./run_ai_slam_nav.sh mode=save_pcd_map pcd_prefix=maps/h1_2_lab
  3. Start offline navigation:
     ./run_ai_slam_nav.sh mode=navigation map=/absolute/path/to/maps/h1_2_lab.yaml localization=global_gicp pcd=/absolute/path/to/maps/h1_2_lab.pcd
  4. If START exists in memory_points.json, the selected localization backend can be initialized automatically.
     Set auto_start_route=true to start the configured memory route after localization.
EOF
}

for arg in "$@"; do
  case "${arg}" in
    -h|--help|help)
      print_usage
      exit 0
      ;;
    mode=*)
      MODE="${arg#mode=}"
      ;;
    stop_policy_bridge=*)
      STOP_POLICY_BRIDGE="${arg#stop_policy_bridge=}"
      ;;
    map=*)
      MAP_FILE="${arg#map=}"
      ;;
    map_prefix=*)
      MAP_SAVE_PREFIX="${arg#map_prefix=}"
      ;;
    pcd_prefix=*)
      PCD_SAVE_PREFIX="${arg#pcd_prefix=}"
      ;;
    localization=*)
      LOCALIZATION_BACKEND="${arg#localization=}"
      ;;
    motion_backend=*)
      MOTION_BACKEND="${arg#motion_backend=}"
      ;;
    confirm_debug_mode=*)
      POLICY_DEBUG_CONFIRMED="${arg#confirm_debug_mode=}"
      ;;
    confirm_exclusive_control=*)
      POLICY_EXCLUSIVE_CONFIRMED="${arg#confirm_exclusive_control=}"
      ;;
    enable_remote_velocity=*)
      ENABLE_REMOTE_VELOCITY="${arg#enable_remote_velocity=}"
      ;;
    pcd=*)
      PCD_MAP_FILE="${arg#pcd=}"
      ;;
    initial_pose=*)
      INITIAL_POSE_NAME="${arg#initial_pose=}"
      ;;
    memory_route=*)
      MEMORY_ROUTE="${arg#memory_route=}"
      ;;
    memory_points_file=*)
      MEMORY_POINTS_FILE="${arg#memory_points_file=}"
      ;;
    gesture_route=*)
      GESTURE_ROUTE="${arg#gesture_route=}"
      ;;
    gesture_hold_sec=*)
      GESTURE_HOLD_SEC="${arg#gesture_hold_sec=}"
      ;;
    turn_route=*)
      TURN_ROUTE="${arg#turn_route=}"
      TURN_ROUTE_SET="true"
      ;;
    start_at_first_route_point=*)
      START_AT_FIRST_ROUTE_POINT="${arg#start_at_first_route_point=}"
      ;;
    auto_start_route=*)
      AUTO_START_ROUTE="${arg#auto_start_route=}"
      ;;
    return_to_start=*)
      RETURN_TO_START="${arg#return_to_start=}"
      ;;
    params_file=*)
      NAV2_PARAMS_FILE="${arg#params_file=}"
      ;;
    *)
      echo "Unknown argument: ${arg}"
      print_usage
      exit 2
      ;;
  esac
done

if [[ "${MODE}" == "mapping" ]]; then
  MODE="online"
fi

if [[ "${STOP_POLICY_BRIDGE}" == "true" ]]; then
  if [[ "${MODE}" != "online" ]]; then
    echo "Invalid bridge stop request." >&2
    exit 2
  fi
  source_ros_environment
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  unset CYCLONEDDS_URI
  bridge_pid="$(cat "${POLICY_BRIDGE_PID_FILE}" 2>/dev/null || true)"
  [[ "${bridge_pid}" =~ ^[0-9]+$ ]] || { echo "Policy bridge PID not found." >&2; exit 1; }
  bridge_info="$(ps -o sid=,pgid=,args= -p "${bridge_pid}" 2>/dev/null || true)"
  read -r bridge_sid bridge_pgid bridge_args <<< "${bridge_info}"
  [[ "${bridge_sid}" == "${bridge_pid}" && "${bridge_pgid}" == "${bridge_pid}" && "${bridge_args}" == *"ros2 launch h1_2_policy_bridge"* ]] || {
    echo "Policy bridge process identity mismatch; leaving it untouched." >&2; exit 1;
  }
  status="$(timeout 4s ros2 service call /h1_2_policy_node/status std_srvs/srv/Trigger "{}" 2>/dev/null || true)"
  if [[ "${status}" != *"authorized=false"* ]]; then
    echo "Policy bridge must be physically supported and explicitly revoked before stopping." >&2
    exit 1
  fi
  kill -INT -- "-${bridge_pid}"
  for _ in $(seq 1 50); do
    kill -0 "${bridge_pid}" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "${bridge_pid}" 2>/dev/null; then
    echo "Policy bridge has not exited; leaving it running." >&2
    exit 1
  fi
  rm -f "${POLICY_BRIDGE_PID_FILE}"
  echo "Policy bridge stopped after confirmed output revocation."
  exit 0
fi

if [[ "${MOTION_BACKEND}" != "unitree" && "${MOTION_BACKEND}" != "policy" ]]; then
  echo "Invalid motion backend: ${MOTION_BACKEND}. Use motion_backend=unitree or motion_backend=policy."
  exit 2
fi

if [[ "${ENABLE_REMOTE_VELOCITY}" != "true" && "${ENABLE_REMOTE_VELOCITY}" != "false" ]]; then
  echo "Invalid enable_remote_velocity: ${ENABLE_REMOTE_VELOCITY}. Use true or false."
  exit 2
fi

if [[ "${MOTION_BACKEND}" == "policy" && "${MODE}" != "navigation" && "${MODE}" != "online" && "${MODE}" != "policy_bridge_only" ]]; then
  echo "policy motion backend is only supported with mode=online, mode=navigation, or mode=policy_bridge_only."
  exit 2
fi

if [[ "${MOTION_BACKEND}" == "policy" ]]; then
  if [[ ! -r "${POLICY_MODEL_PATH}" ]]; then
    echo "Policy model is not readable: ${POLICY_MODEL_PATH}" >&2
    echo "Set H1_2_POLICY_MODEL_PATH to the deployed ONNX model." >&2
    exit 2
  fi
  if [[ "${POLICY_DEBUG_CONFIRMED:-false}" != "true" || "${POLICY_EXCLUSIVE_CONFIRMED:-false}" != "true" ]]; then
    echo "Policy backend requires confirm_debug_mode=true and confirm_exclusive_control=true." >&2
    exit 2
  fi
  if [[ "${TURN_ROUTE_SET}" == "true" && -n "${TURN_ROUTE}" ]]; then
    echo "turn_route is disabled for policy navigation until turning is commissioned."
    exit 2
  fi
  if [[ -n "${GESTURE_ROUTE}" ]]; then
    echo "gesture_route requires the Unitree motion backend."
    exit 2
  fi
  TURN_ROUTE=""
fi

if [[ "${MODE}" != "policy_bridge_only" && ! -f "${CYCLONEDDS_CONFIG}" ]]; then
  echo "Cyclone DDS config does not exist: ${CYCLONEDDS_CONFIG}"
  exit 2
fi

if [[ "${MODE}" != "online" && "${MODE}" != "navigation" && "${MODE}" != "save_map" && "${MODE}" != "save_pcd_map" && "${MODE}" != "policy_bridge_only" ]]; then
  echo "Invalid mode: ${MODE}. Use mode=online, mode=navigation, mode=policy_bridge_only, mode=save_map, or mode=save_pcd_map."
  exit 2
fi

if [[ "${MODE}" == "save_map" && -z "${MAP_SAVE_PREFIX}" ]]; then
  echo "Save-map mode requires map_prefix=maps/h1_2_lab"
  exit 2
fi

if [[ "${MODE}" == "save_pcd_map" && -z "${PCD_SAVE_PREFIX}" ]]; then
  echo "Save-PCD-map mode requires pcd_prefix=maps/h1_2_lab"
  exit 2
fi

if [[ "${MODE}" == "navigation" && -z "${MAP_FILE}" ]]; then
  echo "Navigation mode requires map=/absolute/path/to/map.yaml"
  exit 2
fi

if [[ "${MODE}" == "navigation" && ! -f "${MAP_FILE}" ]]; then
  echo "Map file does not exist: ${MAP_FILE}"
  exit 2
fi

if [[ "${MODE}" == "navigation" && "${LOCALIZATION_BACKEND}" != "global_gicp" && "${LOCALIZATION_BACKEND}" != "icp" ]]; then
  echo "Invalid localization backend: ${LOCALIZATION_BACKEND}. Use localization=global_gicp or localization=icp."
  exit 2
fi

if [[ "${MODE}" == "navigation" && -z "${PCD_MAP_FILE}" ]]; then
  echo "Offline PCD localization requires pcd=/absolute/path/to/map.pcd"
  exit 2
fi

if [[ "${MODE}" == "navigation" && -n "${PCD_MAP_FILE}" && ! -f "${PCD_MAP_FILE}" ]]; then
  echo "PCD map file does not exist: ${PCD_MAP_FILE}"
  exit 2
fi

if [[ "${MODE}" == "save_map" ]]; then
  source_ros_environment

  export ROS_LOCALHOST_ONLY=0
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset ROS_HOSTNAME
  unset ROS_IP
  export CYCLONEDDS_URI="file://${CYCLONEDDS_CONFIG}"
  unset FASTRTPS_DEFAULT_PROFILES_FILE

  MAP_SAVE_DIR="$(dirname "${MAP_SAVE_PREFIX}")"
  if [[ "${MAP_SAVE_DIR}" != "." ]]; then
    mkdir -p "${MAP_SAVE_DIR}"
  fi

  echo "Saving current /map to prefix: ${MAP_SAVE_PREFIX}"
  ros2 run nav2_map_server map_saver_cli -f "${MAP_SAVE_PREFIX}"

  MAP_YAML="${MAP_SAVE_PREFIX}.yaml"
  MAP_IMAGE="$(basename "${MAP_SAVE_PREFIX}.pgm")"
  if [[ -f "${MAP_YAML}" ]]; then
    python3 -c 'import sys, pathlib; p=pathlib.Path(sys.argv[1]); image=sys.argv[2]; lines=p.read_text().splitlines(); lines=["image: " + image if line.startswith("image:") else line for line in lines]; p.write_text("\n".join(lines) + "\n")' "${MAP_YAML}" "${MAP_IMAGE}"
    echo "Updated map image path in ${MAP_YAML} to ${MAP_IMAGE}"
  fi
  exit 0
fi

if [[ "${MODE}" == "save_pcd_map" ]]; then
  source_ros_environment

  export ROS_LOCALHOST_ONLY=0
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset ROS_HOSTNAME
  unset ROS_IP
  export CYCLONEDDS_URI="file://${CYCLONEDDS_CONFIG}"
  unset FASTRTPS_DEFAULT_PROFILES_FILE

  PCD_SAVE_DIR="$(dirname "${PCD_SAVE_PREFIX}")"
  if [[ "${PCD_SAVE_DIR}" != "." ]]; then
    mkdir -p "${PCD_SAVE_DIR}"
  fi

  echo "Saving current FAST-LIO2 3D map to prefix: ${PCD_SAVE_PREFIX}"
  echo "The online FAST-LIO2 node must have been started with map_file_path=${PCD_SAVE_PREFIX}.pcd."
  ros2 service call /map_save std_srvs/srv/Trigger "{}"
  exit 0
fi

if [[ "${MODE}" != "policy_bridge_only" ]]; then
  exec 9>"${LOCK_FILE}"
  if ! flock -n 9; then
    echo "H1-2 ai slam nav is already running. Use ${WORKSPACE_DIR}/stop_ai_slam_nav.sh before starting again."
    exit 1
  fi
fi

if [[ ! -f "${UNITREE_ROS_SETUP}" ]]; then
  echo "Unitree ROS setup not found: ${UNITREE_ROS_SETUP}" >&2
  echo "Set UNITREE_ROS_SETUP to the installed ROS workspace setup.bash." >&2
  exit 2
fi
source_ros_environment

export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_HOSTNAME
unset ROS_IP
export CYCLONEDDS_URI="file://${CYCLONEDDS_CONFIG}"
unset FASTRTPS_DEFAULT_PROFILES_FILE

if [[ "${MOTION_BACKEND}" == "policy" ]]; then
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  unset CYCLONEDDS_URI
fi

bridge_pid="$(cat "${POLICY_BRIDGE_PID_FILE}" 2>/dev/null || true)"
bridge_info=""
if [[ "${bridge_pid}" =~ ^[0-9]+$ ]] && kill -0 "${bridge_pid}" 2>/dev/null; then
  bridge_info="$(ps -o args= -p "${bridge_pid}" 2>/dev/null || true)"
fi
if [[ -n "${bridge_info}" && "${bridge_info}" != *"policy_bridge.launch.py"* ]]; then
  echo "Policy bridge PID file points at a different process; inspect ${POLICY_BRIDGE_PID_FILE}." >&2
  exit 1
fi
if [[ "${MOTION_BACKEND}" == "unitree" && -n "${bridge_info}" ]]; then
  echo "A Policy bridge is still running. Complete supported release/revoke and stop it before selecting Unitree." >&2
  exit 1
fi
if [[ "${MOTION_BACKEND}" == "policy" && -z "${bridge_info}" ]]; then
  rm -f "${POLICY_BRIDGE_PID_FILE}"
  BRIDGE_ARGS=(profile:=hardware model_path:="${POLICY_MODEL_PATH}" protection_profile:=train_20261106 joint_command_mode:=position_pd
    enable_hardware_output:=true operator_confirmed_debug_mode:="${POLICY_DEBUG_CONFIRMED}"
    operator_confirmed_exclusive_control:="${POLICY_EXCLUSIVE_CONFIRMED}"
    enable_navigation_commands:=true continuous_policy_navigation:=true max_policy_steps:=0
    enable_remote_velocity:="${ENABLE_REMOTE_VELOCITY}"
    cmd_vel_topic:=/cmd_vel_final max_linear_speed:=0.20 max_angular_speed:=0.10
    network_interface:="${NETWORK_INTERFACE:-eth0}" dds_domain:="${ROS_DOMAIN_ID}")
  setsid ros2 launch h1_2_policy_bridge policy_bridge.launch.py "${BRIDGE_ARGS[@]}" >/tmp/h1_2_policy_bridge.log 2>&1 &
  bridge_pid=$!
  echo "${bridge_pid}" > "${POLICY_BRIDGE_PID_FILE}"
  sleep 1
  if ! kill -0 "${bridge_pid}" 2>/dev/null; then
    echo "Policy bridge failed to start; see /tmp/h1_2_policy_bridge.log" >&2
    rm -f "${POLICY_BRIDGE_PID_FILE}"
    exit 1
  fi
  bridge_info="$(ps -o args= -p "${bridge_pid}" 2>/dev/null || true)"
fi

if [[ "${MODE}" == "policy_bridge_only" ]]; then
  if [[ -z "${bridge_info}" ]]; then
    echo "Policy bridge failed to start; see /tmp/h1_2_policy_bridge.log" >&2
    exit 1
  fi
  echo "ONNX Policy bridge started; navigation stack and route remain stopped."
  exit 0
fi

if [[ "${MODE}" == "online" ]]; then
  echo "Starting H1-2 camera_init map + Nav2 navigation..."
  ONLINE_LAUNCH_ARGS=(
    "motion_backend:=${MOTION_BACKEND}"
    "start_livox_driver:=false"
    map_file_path:="${PCD_SAVE_PREFIX}.pcd"
    "enable_sport_bridge:=true"
    "memory_points_file:=${MEMORY_POINTS_FILE}"
    "start_at_first_route_point:=$( [[ "${MOTION_BACKEND}" == "policy" ]] && echo false || echo "${START_AT_FIRST_ROUTE_POINT}" )"
    "max_linear_speed:=$( [[ "${MOTION_BACKEND}" == "policy" ]] && echo 0.20 || echo 0.12 )"
    "max_angular_speed:=$( [[ "${MOTION_BACKEND}" == "policy" ]] && echo 0.10 || echo 0.50 )"
    "confirm_debug_mode:=${POLICY_DEBUG_CONFIRMED}"
    "confirm_exclusive_control:=${POLICY_EXCLUSIVE_CONFIRMED}"
  )
  if [[ -n "${NETWORK_INTERFACE}" ]]; then
    ONLINE_LAUNCH_ARGS+=("network_interface:=${NETWORK_INTERFACE}")
  fi
  if [[ -n "${MEMORY_ROUTE}" ]]; then
    ONLINE_LAUNCH_ARGS+=("memory_route:=${MEMORY_ROUTE}")
  fi
  if [[ -n "${TURN_ROUTE}" ]]; then
    ONLINE_LAUNCH_ARGS+=("turn_route:=${TURN_ROUTE}")
  fi
  if [[ -n "${GESTURE_ROUTE}" ]]; then
    ONLINE_LAUNCH_ARGS+=("gesture_route:=${GESTURE_ROUTE}")
  fi
  if [[ -n "${NAV2_PARAMS_FILE}" ]]; then
    ONLINE_LAUNCH_ARGS+=("params_file:=${NAV2_PARAMS_FILE}")
  fi
  setsid ros2 launch h1_2_slam_bringup h1_2_nav2_camera_init_navigation.launch.py "${ONLINE_LAUNCH_ARGS[@]}" &
  LAUNCH_PID=$!
  echo "${LAUNCH_PID}" > "${PID_FILE}"
  printf '%s %s\n' "${LAUNCH_PID}" "${MOTION_BACKEND}" > "${BACKEND_FILE}"
  trap 'exit 130' INT
  trap 'exit 143' TERM
  trap cleanup_launch EXIT
  set +e
  wait "${LAUNCH_PID}"
  LAUNCH_STATUS=$?
  set -e
  rm -f "${PID_FILE}"
  LAUNCH_PID=""
  trap - EXIT INT TERM
  exit "${LAUNCH_STATUS}"
fi

echo "Starting H1-2 Nav2 navigation with map: ${MAP_FILE}"
NAV_LAUNCH_ARGS=(
  "map:=${MAP_FILE}"
  "localization_backend:=${LOCALIZATION_BACKEND}"
  "motion_backend:=${MOTION_BACKEND}"
  "pcd_map:=${PCD_MAP_FILE}"
  "start_livox_driver:=false"
  "map_file_path:=${PCD_SAVE_PREFIX}.pcd"
  "enable_sport_bridge:=true"
  "start_at_first_route_point:=$( [[ "${MOTION_BACKEND}" == "policy" ]] && echo false || echo "${START_AT_FIRST_ROUTE_POINT}" )"
  "max_linear_speed:=$( [[ "${MOTION_BACKEND}" == "policy" ]] && echo 0.20 || echo 0.12 )"
  "max_angular_speed:=$( [[ "${MOTION_BACKEND}" == "policy" ]] && echo 0.10 || echo 0.50 )"
  "confirm_debug_mode:=${POLICY_DEBUG_CONFIRMED}"
  "confirm_exclusive_control:=${POLICY_EXCLUSIVE_CONFIRMED}"
  "odom_yaw_offset:=3.141592653589793"
  "initial_pose_name:=${INITIAL_POSE_NAME}"
  "memory_points_file:=${MEMORY_POINTS_FILE}"
  "auto_start_route:=${AUTO_START_ROUTE}"
  "return_to_start:=${RETURN_TO_START}"
)

if [[ -n "${NETWORK_INTERFACE}" ]]; then
  NAV_LAUNCH_ARGS+=("network_interface:=${NETWORK_INTERFACE}")
fi

if [[ -n "${MEMORY_ROUTE}" ]]; then
  NAV_LAUNCH_ARGS+=("memory_route:=${MEMORY_ROUTE}")
fi
if [[ -n "${TURN_ROUTE}" ]]; then
  NAV_LAUNCH_ARGS+=("turn_route:=${TURN_ROUTE}")
fi

if [[ -n "${GESTURE_ROUTE}" ]]; then
  NAV_LAUNCH_ARGS+=("gesture_route:=${GESTURE_ROUTE}")
fi
NAV_LAUNCH_ARGS+=("gesture_hold_sec:=${GESTURE_HOLD_SEC}")

if [[ -n "${NAV2_PARAMS_FILE}" ]]; then
  NAV_LAUNCH_ARGS+=("params_file:=${NAV2_PARAMS_FILE}")
fi

setsid ros2 launch h1_2_slam_bringup h1_2_nav2_navigation.launch.py "${NAV_LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!
echo "${LAUNCH_PID}" > "${PID_FILE}"
printf '%s %s\n' "${LAUNCH_PID}" "${MOTION_BACKEND}" > "${BACKEND_FILE}"
trap 'exit 130' INT
trap 'exit 143' TERM
trap cleanup_launch EXIT
set +e
wait "${LAUNCH_PID}"
LAUNCH_STATUS=$?
set -e
rm -f "${PID_FILE}"
LAUNCH_PID=""
trap - EXIT INT TERM
exit "${LAUNCH_STATUS}"
