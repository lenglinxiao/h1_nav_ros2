#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCK_FILE="/tmp/h1_2_ai_slam_nav.lock"
MODE="online"
MAP_FILE=""
MAP_SAVE_PREFIX=""
PCD_SAVE_PREFIX="maps/h1_2_lab"
LOCALIZATION_BACKEND="global_gicp"
PCD_MAP_FILE=""
INITIAL_POSE_NAME="START"
MEMORY_ROUTE="A,B,C"
GESTURE_ROUTE=""
AUTO_START_ROUTE="false"
RETURN_TO_START="false"

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
  memory_route=... Route names for memory route runner. Default: A,B,C.
  gesture_route=... Optional per-point gestures, for example B:wave_hand,C:shake_hand.
  auto_start_route=true|false Start memory route after localization TF is available. Default: false.
  return_to_start=true|false Navigate to initial_pose before looping memory_route. Default: false.

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
    gesture_route=*)
      GESTURE_ROUTE="${arg#gesture_route=}"
      ;;
    auto_start_route=*)
      AUTO_START_ROUTE="${arg#auto_start_route=}"
      ;;
    return_to_start=*)
      RETURN_TO_START="${arg#return_to_start=}"
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
  source /opt/ros/foxy/setup.bash
  source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
  source "$HOME/unitree_ros2/install/setup.bash"
  source "${WORKSPACE_DIR}/install/setup.bash"

  export ROS_LOCALHOST_ONLY=0
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset ROS_HOSTNAME
  unset ROS_IP
  unset CYCLONEDDS_URI
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
  source /opt/ros/foxy/setup.bash
  source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
  source "$HOME/unitree_ros2/install/setup.bash"
  source "${WORKSPACE_DIR}/install/setup.bash"

  export ROS_LOCALHOST_ONLY=0
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset ROS_HOSTNAME
  unset ROS_IP
  unset CYCLONEDDS_URI
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

source /opt/ros/foxy/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
source "$HOME/unitree_ros2/install/setup.bash"
source "${WORKSPACE_DIR}/install/setup.bash"

pkill -f "ros2 launch h1_2_slam_bringup fastlio2_grid_navigation.launch.py" || true
pkill -f "ros2 launch h1_2_slam_bringup h1_2_nav2_mapping.launch.py" || true
pkill -f "ros2 launch h1_2_slam_bringup h1_2_nav2_navigation.launch.py" || true
pkill -f "ros2 launch h1_2_slam_bringup h1_2_nav2_slam_navigation.launch.py" || true
pkill -f "ros2 launch h1_2_slam_bringup h1_2_nav2_camera_init_navigation.launch.py" || true
pkill -f "install/fast_lio2/lib/fast_lio2/fastlio2_mapping" || true
pkill -f "install/h1_2_nav_tools/lib/h1_2_nav_tools/fastlio_odom_adapter" || true
pkill -f "install/h1_2_nav_tools/lib/h1_2_nav_tools/pointcloud_frame_adapter" || true
pkill -f "install/h1_2_nav_tools/lib/h1_2_nav_tools/velocity_smoother_ema" || true
pkill -f "install/h1_2_nav_tools/lib/h1_2_nav_tools/pointcloud_to_occupancy_grid" || true
pkill -f "install/h1_2_nav_tools/lib/h1_2_nav_tools/simple_grid_navigator" || true
pkill -f "install/h1_2_nav_tools/lib/h1_2_nav_tools/slam_grid_navigator" || true
pkill -f "install/h1_2_icp_localization/lib/h1_2_icp_localization/global_relocalization_node" || true
pkill -f "install/h1_2_loco_bridge/lib/h1_2_loco_bridge/cmd_vel_to_h1_2_loco" || true
pkill -f "install/h1_2_loco_bridge/lib/h1_2_loco_bridge/gesture_to_h1_2_loco" || true
pkill -f "install/h1_2_loco_bridge/lib/h1_2_loco_bridge/h1_2_loco_udp_daemon" || true
pkill -f "nav2_bt_navigator" || true
pkill -f "nav2_controller" || true
pkill -f "nav2_lifecycle_manager" || true
pkill -f "nav2_map_server" || true
pkill -f "nav2_planner" || true
pkill -f "nav2_recoveries" || true
pkill -f "nav2_waypoint_follower" || true
pkill -f "slam_toolbox" || true
pkill -f "pointcloud_to_laserscan" || true
sleep 1

export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_HOSTNAME
unset ROS_IP
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE

if [[ "${MODE}" == "online" ]]; then
  echo "Starting H1-2 camera_init map + Nav2 navigation..."
  exec ros2 launch h1_2_slam_bringup h1_2_nav2_camera_init_navigation.launch.py \
    start_livox_driver:=false \
    map_file_path:="${PCD_SAVE_PREFIX}.pcd" \
    enable_sport_bridge:=true \
    network_interface:=enp45s0 \
    max_linear_speed:=0.12 \
    max_angular_speed:=0.50
fi

echo "Starting H1-2 Nav2 navigation with map: ${MAP_FILE}"
NAV_LAUNCH_ARGS=(
  "map:=${MAP_FILE}"
  "localization_backend:=${LOCALIZATION_BACKEND}"
  "pcd_map:=${PCD_MAP_FILE}"
  "start_livox_driver:=false"
  "map_file_path:=${PCD_SAVE_PREFIX}.pcd"
  "enable_sport_bridge:=true"
  "network_interface:=enp45s0"
  "max_linear_speed:=0.12"
  "max_angular_speed:=0.50"
  "odom_yaw_offset:=3.141592653589793"
  "initial_pose_name:=${INITIAL_POSE_NAME}"
  "memory_route:=${MEMORY_ROUTE}"
  "auto_start_route:=${AUTO_START_ROUTE}"
  "return_to_start:=${RETURN_TO_START}"
)

if [[ -n "${GESTURE_ROUTE}" ]]; then
  NAV_LAUNCH_ARGS+=("gesture_route:=${GESTURE_ROUTE}")
fi

exec ros2 launch h1_2_slam_bringup h1_2_nav2_navigation.launch.py "${NAV_LAUNCH_ARGS[@]}"
