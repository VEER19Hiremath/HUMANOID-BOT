#!/usr/bin/env bash
# One-command hospital robot bringup (real bot demo).
# Usage:
#   ./start.sh          # start everything
#   ./start.sh stop     # stop everything
#   ./start.sh status   # show what is running

set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$WORKSPACE/install/setup.bash"
LOG_DIR="/tmp/hospital_robot_logs"
MAP_YAML="$WORKSPACE/maps/hospital_map.yaml"
RVIZ_CFG="$WORKSPACE/src/hospital_delivery/rviz/hospital_nav.rviz"
PID_FILE="$LOG_DIR/pids"

# Demo defaults from Sep 24 lab bringup
ODOM_SOURCE="command"
IDLE_CLOSE_S="5.0"
STATIC_MAP_ODOM="true"

mkdir -p "$LOG_DIR"

detect_arduino() {
  local by_id
  by_id="$(ls -1 /dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_* 2>/dev/null | head -1 || true)"
  if [[ -n "$by_id" ]]; then
    echo "$by_id"
  elif [[ -e /dev/arduino ]]; then
    echo "/dev/arduino"
  elif [[ -e /dev/ttyACM0 ]]; then
    echo "/dev/ttyACM0"
  elif [[ -e /dev/ttyACM1 ]]; then
    echo "/dev/ttyACM1"
  else
    echo ""
  fi
}

# Mega often vanishes after EMI ("disabled by hub"). Wait and re-probe.
wait_for_arduino() {
  local seconds="${1:-90}"
  local port=""
  port="$(detect_arduino)"
  if [[ -n "$port" ]]; then
    echo "$port"
    return 0
  fi
  echo "Arduino not seen yet. Waiting up to ${seconds}s..." >&2
  echo "  Tip: motor battery OFF, unplug Mega USB, wait 3s, plug a DIFFERENT Pi port." >&2
  local i
  for i in $(seq 1 "$seconds"); do
    port="$(detect_arduino)"
    if [[ -n "$port" ]]; then
      echo "Arduino appeared after ${i}s: $port" >&2
      sleep 1
      echo "$port"
      return 0
    fi
    if (( i % 10 == 0 )); then
      echo "  still waiting... ${i}/${seconds}s" >&2
    fi
    sleep 1
  done
  echo ""
  return 1
}

detect_lidar() {
  if [[ -e /dev/lidar ]]; then
    echo "/dev/lidar"
  elif [[ -e /dev/ttyUSB0 ]]; then
    echo "/dev/ttyUSB0"
  else
    local by_id
    by_id="$(ls -1 /dev/serial/by-id/usb-Silicon_Labs_CP2102_* 2>/dev/null | head -1 || true)"
    echo "${by_id}"
  fi
}

detect_display() {
  if [[ -n "${DISPLAY:-}" ]]; then
    echo "$DISPLAY"
    return
  fi
  if [[ -S /tmp/.X11-unix/X1 ]]; then
    echo ":1"
  elif [[ -S /tmp/.X11-unix/X0 ]]; then
    echo ":0"
  else
    echo ""
  fi
}

source_ros() {
  if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ERROR: ROS 2 Jazzy not found at $ROS_SETUP" >&2
    exit 1
  fi
  if [[ ! -f "$WS_SETUP" ]]; then
    echo "ERROR: workspace not built: $WS_SETUP" >&2
    echo "Run: cd \"$WORKSPACE\" && source \"$ROS_SETUP\" && colcon build" >&2
    exit 1
  fi

  # Drop any other workspace overlay (e.g. ~/hospital_robot_ws) so this
  # Desktop tree is the only one that provides packages.
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
  unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX
  # Keep system PYTHONPATH clean of other ws site-packages
  if [[ -n "${PYTHONPATH:-}" ]]; then
    PYTHONPATH="$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v hospital_robot_ws | paste -sd: - || true)"
    export PYTHONPATH
  fi

  # Audio / desktop session (needed for mic + Google TTS playback)
  export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
  if [[ -z "${DBUS_SESSION_BUS_ADDRESS:-}" && -S "$XDG_RUNTIME_DIR/bus" ]]; then
    export DBUS_SESSION_BUS_ADDRESS="unix:path=$XDG_RUNTIME_DIR/bus"
  fi

  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  # shellcheck disable=SC1090
  source "$WS_SETUP"
  if [[ -f "$HOME/venv/bin/activate" ]]; then
    # shellcheck disable=SC1091
    source "$HOME/venv/bin/activate"
  fi
  set -u

  echo "  Using packages from: $(ros2 pkg prefix wheel_odometry 2>/dev/null || echo unknown)"
}

send_motor_stop() {
  local port="$1"
  [[ -z "$port" || ! -e "$port" ]] && return 0
  python3 - "$port" <<'PY' 2>/dev/null || true
import sys, serial, time
port = sys.argv[1]
try:
    s = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(0.2)
    for _ in range(8):
        s.write(b"VL:0.00 VR:0.00\n")
        time.sleep(0.05)
    s.close()
except Exception:
    pass
PY
}

stop_stack() {
  echo "Stopping hospital robot stack..."
  local arduino
  arduino="$(detect_arduino)"
  send_motor_stop "$arduino"

  # Prefer recorded PIDs, then fall back to pattern kill
  if [[ -f "$PID_FILE" ]]; then
    while read -r pid; do
      [[ -n "${pid:-}" ]] && kill "$pid" 2>/dev/null || true
    done < "$PID_FILE"
    rm -f "$PID_FILE"
  fi

  local patterns=(
    "hospital_bringup real_robot.launch"
    "hospital_bringup localization.launch"
    "hospital_bringup navigation.launch"
    "lib/wheel_odometry/base_controller"
    "nav2_map_server/map_server"
    "nav2_controller/controller_server"
    "nav2_planner/planner_server"
    "nav2_bt_navigator/bt_navigator"
    "nav2_smoother/smoother_server"
    "nav2_behaviors/behavior_server"
    "nav2_waypoint_follower/waypoint_follower"
    "nav2_velocity_smoother/velocity_smoother"
    "nav2_collision_monitor/collision_monitor"
    "nav2_lifecycle_manager/lifecycle_manager"
    "rplidar_ros/rplidar_node"
    "voice_delivery_node"
    "rviz2 -d .*hospital_nav.rviz"
    "robot_state_publisher"
    "static_transform_publisher .* map odom"
  )
  local pat
  for pat in "${patterns[@]}"; do
    pkill -f "$pat" 2>/dev/null || true
  done
  sleep 1
  echo "Stopped. Logs kept in $LOG_DIR"
}

show_status() {
  echo "Hospital robot status"
  echo "  Arduino: $(detect_arduino || echo missing)"
  echo "  Lidar:   $(detect_lidar || echo missing)"
  echo "  Display: $(detect_display || echo none)"
  echo "  Processes:"
  ps -eo pid,args | grep -E 'real_robot.launch|localization.launch|navigation.launch|base_controller|rplidar_node|map_server|controller_server|voice_delivery|rviz2' \
    | grep -v grep || echo "    (none)"
}

start_bg() {
  local name="$1"
  shift
  local log="$LOG_DIR/${name}.log"
  echo "  -> $name  (log: $log)"
  "$@" >"$log" 2>&1 &
  echo $! >>"$PID_FILE"
}

cmd="${1:-start}"

case "$cmd" in
  stop)
    stop_stack
    exit 0
    ;;
  status)
    show_status
    exit 0
    ;;
  start|"")
    ;;
  *)
    echo "Usage: $0 [start|stop|status]"
    exit 1
    ;;
esac

if [[ ! -f "$MAP_YAML" ]]; then
  echo "ERROR: map not found: $MAP_YAML" >&2
  exit 1
fi

ARDUINO_PORT="$(wait_for_arduino 90 || true)"
LIDAR_PORT="$(detect_lidar)"
export DISPLAY
DISPLAY="$(detect_display)"
export DISPLAY

echo "============================================"
echo " Hospital Robot — starting"
echo " Workspace: $WORKSPACE"
echo " Arduino:   ${ARDUINO_PORT:-NOT FOUND}"
echo " Lidar:     ${LIDAR_PORT:-NOT FOUND}"
echo " Odom:      $ODOM_SOURCE  idle_close=${IDLE_CLOSE_S}s"
echo " Map:       $MAP_YAML"
echo " Display:   ${DISPLAY:-none (RViz skipped)}"
echo "============================================"

if [[ -z "$ARDUINO_PORT" ]]; then
  echo "ERROR: Arduino Mega USB still missing after wait." >&2
  echo "Lidar can spin while Mega is gone — they are separate USB devices." >&2
  echo "Fix: battery OFF → reseat Mega USB on another port → ./start.sh" >&2
  exit 1
fi
if [[ -z "$LIDAR_PORT" ]]; then
  echo "ERROR: LiDAR not found. Plug lidar USB and retry." >&2
  exit 1
fi

stop_stack
source_ros
: >"$PID_FILE"
send_motor_stop "$ARDUINO_PORT"

start_bg base \
  ros2 launch hospital_bringup real_robot.launch.py \
    arduino_port:="$ARDUINO_PORT" \
    lidar_port:="$LIDAR_PORT" \
    odom_source:="$ODOM_SOURCE" \
    idle_close_s:="$IDLE_CLOSE_S" \
    static_map_odom:="$STATIC_MAP_ODOM" \
    use_sim_time:=false

sleep 3

start_bg map \
  ros2 launch hospital_bringup localization.launch.py \
    map:="$MAP_YAML"

sleep 2

start_bg nav \
  ros2 launch hospital_bringup navigation.launch.py \
    use_sim_time:=false

sleep 2

if [[ -n "$DISPLAY" && -f "$RVIZ_CFG" ]]; then
  # Xauthority for desktop sessions
  if [[ -z "${XAUTHORITY:-}" ]]; then
    if [[ -f /run/user/$(id -u)/gdm/Xauthority ]]; then
      export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
    elif [[ -f "$HOME/.Xauthority" ]]; then
      export XAUTHORITY="$HOME/.Xauthority"
    fi
  fi
  start_bg rviz \
    rviz2 -d "$RVIZ_CFG"
else
  echo "  !! RViz skipped (no DISPLAY or missing $RVIZ_CFG)"
fi

start_bg voice \
  ros2 run hospital_delivery voice_delivery_node

echo
echo "All started."
echo "  Stop with:  $WORKSPACE/start.sh stop"
echo "  Status:     $WORKSPACE/start.sh status"
echo "  Logs:       $LOG_DIR/"
echo
echo "Before driving: motor battery ON, SW2 OFF, knobs fully CCW."
echo "If only one wheel spins: battery off 10s, then on again."
