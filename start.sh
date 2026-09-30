#!/bin/bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Run logs live in the project so a reboot can't wipe them (it cleared /tmp).
# One folder per start/map run; run_logs/latest and the old
# /tmp/hospital_robot_logs path both point at the newest run.
RUN_LOGS="${ROOT}/run_logs"
start_stack() {
  stop_stack
  : > "${LOG_DIR}/base.log"
  : > "${LOG_DIR}/nav.log"
  : > "${LOG_DIR}/map.log"
  : > "${LOG_DIR}/voice.log"
  : > "${LOG_DIR}/rviz.log"

  ros2 launch hospital_bringup real_robot.launch.py \
    arduino_port:="${ARDUINO_PORT:-/dev/ttyUSB0}" \
    lidar_port:="${LIDAR_PORT:-/dev/ttyUSB1}" \
    start_lidar:=true start_base:=true \
    "${LOC_ARGS[@]}" use_sim_time:=false \
    > "${LOG_DIR}/base.log" 2>&1 &

  sleep 3
  ros2 launch hospital_bringup localization.launch.py "map:=${MAP}" "use_amcl:=${USE_AMCL}" \
    > "${LOG_DIR}/map.log" 2>&1 &

  # map_server often configures then stalls inactive (DDS change_state timeout),
  # so RViz Map display stays empty until we force activate.
  for _ in $(seq 1 12); do
    if timeout 2 ros2 lifecycle get /map_server 2>/dev/null | grep -q 'active'; then
      break
    fi
    timeout 2 ros2 lifecycle set /map_server activate >/dev/null 2>&1 || true
    sleep 0.5
  done

  sleep 2
  # CM on: velocity_smoother → cmd_vel_smoothed → collision_monitor → /cmd_vel.
  # Lidar stop zone 0.27–0.65 m ahead, half speed 0.65–1.10 m. If CM is not
  # active nothing reaches /cmd_vel, so the robot stays still (fail-safe).
  ros2 launch hospital_bringup navigation.launch.py \
    use_sim_time:=false enable_collision_monitor:=true \
    > "${LOG_DIR}/nav.log" 2>&1 &

  # Brief activate attempts only — never block bringup on DDS hangs.
  for _ in $(seq 1 8); do
    if timeout 2 ros2 lifecycle get /collision_monitor 2>/dev/null | grep -q 'active'; then
      break
    fi
    timeout 2 ros2 lifecycle set /collision_monitor activate >/dev/null 2>&1 || true
    sleep 0.5
  done

  sleep 1
  if [[ -f "${RVIZ}" ]]; then
    if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
      # Low priority: RViz is display only; voice (stop) and Nav2 come first.
      nice -n 10 rviz2 -d "${RVIZ}" > "${LOG_DIR}/rviz.log" 2>&1 &
      echo "RViz starting on DISPLAY=${DISPLAY:-wayland}"
    else
      echo "WARNING: No DISPLAY/WAYLAND — RViz skipped. Open a desktop terminal and rerun, or: DISPLAY=:0 ./start.sh"
    fi
  fi

  rm -f /tmp/hospital_hfp_mic.sock /tmp/hospital_hfp_mic.up
  if [[ -f "${ROOT}/scripts/hfp_mic.py" ]]; then
    python3 "${ROOT}/scripts/hfp_mic.py" > "${LOG_DIR}/hfp.log" 2>&1 &
  fi
  export PYTHONPATH="${HOME}/venv/lib/python3.12/site-packages:${HOME}/venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
  ros2 run hospital_delivery voice_delivery_node --ros-args -p "map_yaml:=${MAP}" \
    >> "${LOG_DIR}/voice.log" 2>&1 &
  # Rooms of this map in RViz (zones, names, the room being driven to).
  ros2 run hospital_delivery room_markers --ros-args -p "map_yaml:=${MAP}" \
    > "${LOG_DIR}/room_markers.log" 2>&1 &

  sleep 2
  grep -q 'Mega ready' "${LOG_DIR}/base.log" 2>/dev/null && echo "Mega: OK" || echo "WARNING: Mega not ready yet"
}

case "${1:-start}" in
  start|map) LOG_DIR="${RUN_LOGS}/$(date +%Y-%m-%d_%H-%M-%S)_${1:-start}" ;;
  *) LOG_DIR="${RUN_LOGS}/latest" ;;
esac
# Map: a scanned floor map from ./start.sh map wins once it exists, else the
# drawn 30 x 40 ft area. MAP=maps/x.yaml overrides (hospital_map = old lab).
if [[ -z "${MAP:-}" ]]; then
  # A scanned floor first; else the wall-free open floor (the drawn
  # 30x40 walls don't exist on the test floor: the robot detoured around
  # them). MAP=maps/area_30x40.yaml ./start.sh for the walled area.
  if [[ -f "${ROOT}/maps/floor_map.yaml" ]]; then
    MAP="${ROOT}/maps/floor_map.yaml"
  elif [[ -f "${ROOT}/maps/open_floor.yaml" ]]; then
    MAP="${ROOT}/maps/open_floor.yaml"
  else
    MAP="${ROOT}/maps/area_30x40.yaml"
  fi
fi
[[ "${MAP}" == /* ]] || MAP="${ROOT}/${MAP}"   # MAP=maps/x.yaml works from anywhere

# Drawn maps (no real walls to match) say "# localization: static" in their
# yaml: the robot starts at "# home: x y" facing +x and its pose is wheel
# odometry only. Scanned maps (./start.sh map) use AMCL: the lidar keeps the
# pose on the real walls.
LOC_ARGS=(static_map_odom:=false)
USE_AMCL=true
if grep -q '^# localization: static' "${MAP}" 2>/dev/null; then
  read -r HOME_X HOME_Y < <(awk '/^# home:/ {print $3, $4}' "${MAP}")
  LOC_ARGS=(static_map_odom:=true "map_odom_x:=${HOME_X}" "map_odom_y:=${HOME_Y}")
  USE_AMCL=false
fi
RVIZ="${ROOT}/src/hospital_delivery/rviz/hospital_nav.rviz"

source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
export FASTRTPS_BUILTIN_TRANSPORTS=UDPv4
# RViz needs a GUI display; Cursor/SSH shells often have DISPLAY unset.
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  if [[ -S /tmp/.X11-unix/X0 ]]; then
    export DISPLAY=:0
  elif [[ -S /tmp/.X11-unix/X1 ]]; then
    export DISPLAY=:1
  fi
fi
if [[ -n "${DISPLAY:-}" && -z "${XAUTHORITY:-}" && -f "${HOME}/.Xauthority" ]]; then
  export XAUTHORITY="${HOME}/.Xauthority"
fi

if [[ "${1:-start}" == start || "${1:-start}" == map ]]; then
  mkdir -p "${LOG_DIR}"
  ln -sfn "${LOG_DIR}" "${RUN_LOGS}/latest"
  if [[ -d /tmp/hospital_robot_logs && ! -L /tmp/hospital_robot_logs ]]; then
    mv /tmp/hospital_robot_logs "/tmp/hospital_robot_logs.old.$$"
  fi
  ln -sfn "${LOG_DIR}" /tmp/hospital_robot_logs
  # Keep the newest 30 runs.
  ls -1dt "${RUN_LOGS}"/20* 2>/dev/null | tail -n +31 | xargs -r rm -rf
fi

# Keep install share configs in sync with src (editable launch is often
# symlinked; yaml copies go stale and silently keep dead motion params).
SHARE_CFG="${ROOT}/install/hospital_bringup/share/hospital_bringup/config/nav2_params.yaml"
SRC_CFG="${ROOT}/src/hospital_bringup/config/nav2_params.yaml"
if [[ -f "${SRC_CFG}" && -f "${SHARE_CFG}" ]]; then
  cp -f "${SRC_CFG}" "${SHARE_CFG}"
fi
# Sync Python packages into build/ so `ros2 run` picks up the latest voice/base.
for pkg in hospital_delivery wheel_odometry; do
  src_py="${ROOT}/src/${pkg}/${pkg}"
  build_py="${ROOT}/build/${pkg}/${pkg}"
  if [[ -d "${src_py}" && -d "${build_py}" ]]; then
    cp -af "${src_py}/." "${build_py}/" 2>/dev/null || true
  fi
done

ARDUINO_PORT="${ARDUINO_PORT:-$(ls /dev/serial/by-id/usb-Arduino* 2>/dev/null | head -1)}"
LIDAR_PORT="${LIDAR_PORT:-$(ls /dev/serial/by-id/usb-Silicon_Labs* 2>/dev/null | head -1)}"

stop_stack() {
  pkill -f 'ros2 launch hospital_bringup' 2>/dev/null || true
  pkill -f 'hospital_delivery voice_delivery_node' 2>/dev/null || true
  pkill -f 'lib/hospital_delivery/voice_delivery_node' 2>/dev/null || true
  pkill -f 'scripts/hfp_mic.py' 2>/dev/null || true
  rm -f /tmp/hospital_hfp_mic.up
  pkill -f 'hospital_delivery room_markers' 2>/dev/null || true
  pkill -f 'lib/hospital_delivery/room_markers' 2>/dev/null || true
  pkill -f 'rviz2 -d' 2>/dev/null || true
  # Orphans from repeated starts make RViz "dance" (conflicting TF/odom).
  pkill -f 'wheel_odometry/base_controller' 2>/dev/null || true
  pkill -f 'robot_state_publisher' 2>/dev/null || true
  pkill -f 'static_transform_publisher' 2>/dev/null || true
  pkill -f 'rplidar_node' 2>/dev/null || true
  pkill -f 'nav2_controller/controller_server' 2>/dev/null || true
  pkill -f 'nav2_bt_navigator' 2>/dev/null || true
  pkill -f 'nav2_collision_monitor' 2>/dev/null || true
  pkill -f 'nav2_planner' 2>/dev/null || true
  pkill -f 'nav2_behaviors' 2>/dev/null || true
  pkill -f 'nav2_waypoint' 2>/dev/null || true
  pkill -f 'nav2_velocity_smoother' 2>/dev/null || true
  pkill -f 'nav2_map_server' 2>/dev/null || true
  pkill -f 'nav2_amcl' 2>/dev/null || true
  pkill -f 'slam_toolbox' 2>/dev/null || true
  # Catch-all: every Nav2 binary (route_server, smoother_server, … were
  # missed above and piled up for hours, overloading the Pi).
  pkill -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null || true
  pkill -f 'component_container' 2>/dev/null || true   # composed Nav2
  pkill -f 'lifecycle_manager' 2>/dev/null || true
  sleep 2
  # A node stuck in DDS ignores SIGTERM (a hung base_controller survived and
  # held the Mega port next to the new one). Force anything still alive.
  pkill -9 -f 'wheel_odometry/base_controller' 2>/dev/null || true
  pkill -9 -f 'lib/hospital_delivery/voice_delivery_node' 2>/dev/null || true
  pkill -9 -f 'rplidar_node' 2>/dev/null || true
  pkill -9 -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null || true
  pkill -9 -f 'component_container' 2>/dev/null || true
  pkill -9 -f 'ros2 launch hospital_bringup' 2>/dev/null || true
  pkill -9 -f 'slam_toolbox' 2>/dev/null || true
  # Force-killed nodes leave Fast DDS shared-memory files in /dev/shm; stale
  # ones can hang new nodes at startup. Removes only unused (zombie) files.
  ros2 daemon stop >/dev/null 2>&1 || true
  fastdds shm clean >/dev/null 2>&1 || true
}

save_map() {
  # $1 = name under maps/. Quiet unless it fails.
  timeout 25 ros2 run nav2_map_server map_saver_cli -f "${ROOT}/maps/$1" \
    --ros-args -p save_map_timeout:=10.0 >/dev/null 2>&1
}

case "${1:-start}" in
  stop)
    stop_stack
    echo "Stack stopped."
    ;;
  status)
    pgrep -af 'hospital_bringup|voice_delivery|rviz2|base_controller|controller_server' || echo "Stack not running."
    ;;
  start)
    start_stack
    # Direct rclpy check (the ros2 CLI gave false "not active" warnings).
    # A node sometimes hangs while joining DDS at startup (Nav2's lifecycle
    # manager then waits forever and nothing drives): restart once.
    sleep 5
    if ! timeout 60 python3 "${ROOT}/scripts/health_check.py"; then
      echo "Startup incomplete — restarting the stack once ..."
      start_stack
      sleep 5
      timeout 60 python3 "${ROOT}/scripts/health_check.py" \
        || echo "WARNING: still not ready. Run ./start.sh again; see ${LOG_DIR}/nav.log"
    fi
    echo "Using map: ${MAP}"

    echo "Hospital stack starting. Logs: ${LOG_DIR}"
    echo "Demo: both SW2 OFF, knobs CCW. If a driver shows RED, battery OFF 10s then ON."
    echo "Ready: say 'go to room one' (one to five), 'go home' or 'stop'."
    echo "Stop with: ./start.sh stop"
    ;;
  map)
    # One command: drive with the keyboard, map auto-saves every 30 s,
    # Ctrl+C saves the final map, then click rooms in RViz. Start with the
    # robot at its HOME spot: home becomes (0, 0) on the map.
    stop_stack
    : > "${LOG_DIR}/base.log"
    : > "${LOG_DIR}/slam.log"
    if [[ -f "${ROOT}/maps/floor_map.yaml" ]]; then
      mv -f "${ROOT}/maps/floor_map.yaml" "${ROOT}/maps/floor_map.bak.yaml"
      mv -f "${ROOT}/maps/floor_map.pgm" "${ROOT}/maps/floor_map.bak.pgm" 2>/dev/null || true
      sed -i 's/^image: floor_map.pgm/image: floor_map.bak.pgm/' "${ROOT}/maps/floor_map.bak.yaml"
      echo "Previous floor map kept as maps/floor_map.bak.*"
    fi
    ros2 launch hospital_bringup real_robot.launch.py \
      arduino_port:="${ARDUINO_PORT:-/dev/ttyUSB0}" \
      lidar_port:="${LIDAR_PORT:-/dev/ttyUSB1}" \
      start_lidar:=true start_base:=true \
      static_map_odom:=false use_sim_time:=false \
      > "${LOG_DIR}/base.log" 2>&1 &
    sleep 4
    # The launch file configures + activates slam_toolbox (a lifecycle node on
    # Jazzy); a bare `ros2 run` stays unconfigured and never builds a map.
    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false \
      slam_params_file:="${ROOT}/src/hospital_bringup/config/slam_params.yaml" \
      > "${LOG_DIR}/slam.log" 2>&1 &
    if [[ -f "${RVIZ}" && ( -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ) ]]; then
      # Low priority: RViz is display only; voice (stop) and Nav2 come first.
      nice -n 10 rviz2 -d "${RVIZ}" > "${LOG_DIR}/rviz.log" 2>&1 &
    fi
    echo -n "Waiting for lidar and map"
    for _ in $(seq 1 20); do
      timeout 3 ros2 topic echo /map --once --field info.width >/dev/null 2>&1 && break
      echo -n "."
    done
    echo
    timeout 60 python3 "${ROOT}/scripts/health_check.py" --mapping || true

    ( while sleep 30; do save_map floor_map && echo "[auto-saved maps/floor_map]"; done ) &
    AUTOSAVE_PID=$!

    echo
    echo "=== MAPPING — drive slowly along every wall (no obstacle stop now) ==="
    echo "    i = forward   , = back   j / l = curve left / right   k = stop"
    echo "    Map auto-saves every 30 s.  Press Ctrl+C when the map is complete."
    echo
    # A handler (not ignore) so Ctrl+C ends teleop but not this script.
    trap ':' INT
    ros2 run teleop_twist_keyboard teleop_twist_keyboard || true
    trap - INT
    kill "${AUTOSAVE_PID}" 2>/dev/null || true
    wait "${AUTOSAVE_PID}" 2>/dev/null || true

    echo "Saving final map ..."
    if save_map floor_map; then
      echo "Saved maps/floor_map.yaml — ./start.sh now uses it."
      echo
      echo "=== ROOMS — in RViz pick 'Publish Point', click each spot when asked ==="
      python3 "${ROOT}/scripts/pick_rooms.py" || echo "Room picking skipped; run later: python3 scripts/pick_rooms.py (with ./start.sh running)"
    else
      echo "WARNING: final save failed; last auto-save (if any) is in maps/floor_map.*"
    fi
    stop_stack
    echo "Done. Put the robot at home and run: ./start.sh"
    ;;
  calibrate)
    # Floor calibration of metres per wheel pulse (ROS is loaded above, so no
    # 'source' needed). The stack must be running: ./start.sh first.
    if ! pgrep -f 'wheel_odometry/base_controller' >/dev/null; then
      echo "Start the robot first: ./start.sh   (then ./start.sh calibrate in another terminal)"
      exit 1
    fi
    export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
    python3 "${ROOT}/scripts/floor_calibrate.py" --tape "${@:2}"
    ;;
  savemap)
    NAME="${2:-floor_map}"
    save_map "${NAME}" && echo "Saved maps/${NAME}.yaml/.pgm" || echo "Save failed — is ./start.sh map running?"
    ;;
  *)
    echo "Usage: $0 {start|stop|status|map|savemap [name]|calibrate}   (map = drive + auto-save + rooms; calibrate = measure distance per wheel pulse)"
    exit 1
    ;;
esac
