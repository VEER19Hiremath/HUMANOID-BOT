#!/bin/bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="/tmp/hospital_robot_logs"
MAP="${ROOT}/maps/hospital_map.yaml"
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

mkdir -p "${LOG_DIR}"

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
  pkill -f 'lifecycle_manager' 2>/dev/null || true
  sleep 2
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
      odom_source:=encoder open_loop_odom:=false idle_close_s:=0.0 \
      static_map_odom:=true use_sim_time:=false \
      > "${LOG_DIR}/base.log" 2>&1 &

    sleep 3
    ros2 launch hospital_bringup localization.launch.py "map:=${MAP}" \
      > "${LOG_DIR}/map.log" 2>&1 &

    # map_server often configures then stalls inactive (DDS change_state timeout),
    # so RViz Map display stays empty until we force activate.
    for _ in $(seq 1 30); do
      if ros2 lifecycle get /map_server 2>/dev/null | grep -q 'active'; then
        break
      fi
      timeout 5 ros2 lifecycle set /map_server activate >/dev/null 2>&1 || true
      sleep 1
    done

    sleep 2
    ros2 launch hospital_bringup navigation.launch.py \
      use_sim_time:=false enable_collision_monitor:=true \
      > "${LOG_DIR}/nav.log" 2>&1 &

    # Lifecycle manager can stall after bt_navigator; ensure collision_monitor
    # is active so cmd_vel_smoothed reaches /cmd_vel and the base moves.
    for _ in $(seq 1 40); do
      if ros2 lifecycle get /collision_monitor 2>/dev/null | grep -q 'active'; then
        break
      fi
      ros2 lifecycle set /collision_monitor activate >/dev/null 2>&1 || true
      sleep 1
    done

    sleep 1
    if [[ -f "${RVIZ}" ]]; then
      if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
        rviz2 -d "${RVIZ}" > "${LOG_DIR}/rviz.log" 2>&1 &
        echo "RViz starting on DISPLAY=${DISPLAY:-wayland}"
      else
        echo "WARNING: No DISPLAY/WAYLAND — RViz skipped. Open a desktop terminal and rerun, or: DISPLAY=:0 ./start.sh"
      fi
    fi

    rm -f /tmp/hospital_hfp_mic.sock
    if [[ -f "${ROOT}/scripts/hfp_mic.py" ]]; then
      python3 "${ROOT}/scripts/hfp_mic.py" > "${LOG_DIR}/hfp.log" 2>&1 &
    fi
    export PYTHONPATH="${HOME}/venv/lib/python3.12/site-packages:${HOME}/venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
    ros2 run hospital_delivery voice_delivery_node >> "${LOG_DIR}/voice.log" 2>&1 &

    echo "Hospital stack starting. Logs: ${LOG_DIR}"
    echo "Ready: say 'go to room one/two/three/four' or 'go home'."
    echo "Stop with: ./start.sh stop"
    ;;
  *)
    echo "Usage: $0 {start|stop|status}"
    exit 1
    ;;
esac
