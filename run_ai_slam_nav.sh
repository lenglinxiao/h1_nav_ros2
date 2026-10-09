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
MODE="online"
MAP_FILE=""
MAP_SAVE_PREFIX=""
PCD_SAVE_PREFIX="maps/h1_2_lab"
LOCALIZATION_BACKEND="global_gicp"
PCD_MAP_FILE=""
INITIAL_POSE_NAME="START"
MEMORY_ROUTE="A,B"
TURN_ROUTE="B:ccw,A:cw"
START_AT_FIRST_ROUTE_POINT="true"
GESTURE_ROUTE=""
GESTURE_HOLD_SEC="3.0"
AUTO_START_ROUTE="false"
RETURN_TO_START="false"
NAV2_PARAMS_FILE=""
MEMORY_POINTS_FILE="${WORKSPACE_DIR}/maps/memory_points.json"
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
    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
    wait_for_session_exit 5 || {
      kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
      wait_for_session_exit 2 || kill -KILL -- "-${LAUNCH_PID}" 2>/dev/null || true
    }
  fi
  if [[ -n "${LAUNCH_PID}" && -f "${PID_FILE}" && "$(cat "${PID_FILE}" 2>/dev/null || true)" == "${LAUNCH_PID}" ]]; then
    rm -f "${PID_FILE}"
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
  ./run_ai_slam_nav.sh mode=navigation map=/absolute/path/to/map.yaml pcd=/absolute/path/to/map.pcd initial_pose=START memory_route=A,B,C gesture_route=B:wave_hand,C:shake_hand auto_start_route=true

Modes:
  mode=online      Online camera_init mapping + Nav2 navigation. This is the default.
  mode=mapping     Compatibility alias for mode=online.
  mode=save_map    Save the currently published /map to map_prefix. Does not stop online mode.
  mode=save_pcd_map Save the current FAST-LIO2 3D PCD map to pcd_prefix. Does not stop online mode.
  mode=navigation  Offline fixed-map Nav2 navigation. Requires map=...
  pcd_prefix=...   Prefix for FAST-LIO2 PCD map save. Default: maps/h1_2_lab.
  localization=... Offline localization backend: global_gicp or icp. Default: global_gicp.
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

if [[ ! -f "${CYCLONEDDS_CONFIG}" ]]; then
  echo "Cyclone DDS config does not exist: ${CYCLONEDDS_CONFIG}"
  exit 2
fi

if [[ "${MODE}" != "online" && "${MODE}" != "navigation" && "${MODE}" != "save_map" && "${MODE}" != "save_pcd_map" ]]; then
  echo "Invalid mode: ${MODE}. Use mode=online, mode=save_map, mode=save_pcd_map, or mode=navigation."
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

exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "H1-2 ai slam nav is already running. Use ${WORKSPACE_DIR}/stop_ai_slam_nav.sh before starting again."
  exit 1
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

if [[ "${MODE}" == "online" ]]; then
  echo "Starting H1-2 camera_init map + Nav2 navigation..."
  ONLINE_LAUNCH_ARGS=(
    "start_livox_driver:=false"
    map_file_path:="${PCD_SAVE_PREFIX}.pcd"
    "enable_sport_bridge:=true"
    "memory_points_file:=${MEMORY_POINTS_FILE}"
    "start_at_first_route_point:=${START_AT_FIRST_ROUTE_POINT}"
    "max_linear_speed:=0.12"
    "max_angular_speed:=0.50"
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
  "pcd_map:=${PCD_MAP_FILE}"
  "start_livox_driver:=false"
  "map_file_path:=${PCD_SAVE_PREFIX}.pcd"
  "enable_sport_bridge:=true"
  "start_at_first_route_point:=${START_AT_FIRST_ROUTE_POINT}"
  "max_linear_speed:=0.12"
  "max_angular_speed:=0.50"
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
