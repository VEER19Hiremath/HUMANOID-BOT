#!/usr/bin/env bash
# ==============================================================================
# 🏥 Hospital Delivery Robot — Simulation Startup
# ==============================================================================
# This script orchestrates the launch of:
#   1. Gazebo Hospital World + Robot Spawn
#   2. Nav2 Bringup (AMCL, Planner, Controller)
#   3. RViz2 Visualization
#   4. Initial Pose Publish for AMCL
#   5. Voice-Controlled Delivery Node (foreground)
# ==============================================================================

# Stylized startup header
echo -e "\033[1;36m"
echo "============================================================"
echo "      🏥 HOSPITAL DELIVERY ROBOT — SIMULATION STARTUP 🤖     "
echo "============================================================"
echo -e "\033[0m"

# Track background PIDs for graceful teardown
PIDS=()

# Teardown background services on exit/interrupt
cleanup() {
    echo -e "\n\033[1;33m🛑 [Sim Startup] Initiating complete simulation teardown...\033[0m"
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "Stopping background process PID: $pid..."
            kill -SIGINT "$pid" 2>/dev/null
            sleep 0.5
            kill -9 "$pid" 2>/dev/null
        fi
    done
    echo -e "\033[1;32m✨ Teardown complete. Environment cleaned. Goodbye! \033[0m"
    exit 0
}

# Trap terminal exits or Ctrl+C
trap cleanup SIGINT SIGTERM EXIT

# Ensure log directory exists
LOG_DIR="$HOME/hospital_robot_ws/log"
mkdir -p "$LOG_DIR"

# ----------------------------------------------------------------------------
# 0. Source ROS 2 and workspace
# ----------------------------------------------------------------------------
echo "⚙️  0. Sourcing ROS 2 Humble and Workspace Environment..."
source /opt/ros/humble/setup.bash
if [ -f "$HOME/hospital_robot_ws/install/setup.bash" ]; then
    source "$HOME/hospital_robot_ws/install/setup.bash"
else
    echo -e "\033[1;31m⚠️  Workspace setup.bash not found! Please build hospital_robot_ws first.\033[0m"
fi
export TURTLEBOT3_MODEL=burger

# ----------------------------------------------------------------------------
# 1. Gazebo Hospital World + Robot Spawn
# ----------------------------------------------------------------------------
echo -e "🚀 1. Launching Hospital World in Gazebo..."
echo -e "    👉 Logs redirected to: \033[1;34m$LOG_DIR/hospital_world.log\033[0m"
ros2 launch hospital_world hospital_world.launch.py > "$LOG_DIR/hospital_world.log" 2>&1 &
WORLD_PID=$!
PIDS+=($WORLD_PID)

echo "⏳ Waiting for Gazebo simulation clock to come online..."
for i in {1..30}; do
    if ros2 topic list 2>/dev/null | grep -q "/clock"; then
        echo -e "\033[1;32m✅ Gazebo simulation is ONLINE!\033[0m"
        break
    fi
    sleep 1
    if [ $i -eq 30 ]; then
        echo -e "\033[1;33m⚠️  Gazebo is taking longer than expected to start. Continuing...\033[0m"
    fi
done

# ----------------------------------------------------------------------------
# 2. Nav2 Bringup
# ----------------------------------------------------------------------------
echo -e "🚀 2. Launching Nav2 Bringup..."
echo -e "    👉 Logs redirected to: \033[1;34m$LOG_DIR/nav2.log\033[0m"
ros2 launch nav2_bringup bringup_launch.py \
    map:="$HOME/hospital_robot_ws/maps/hospital_map.yaml" \
    use_sim_time:=True > "$LOG_DIR/nav2.log" 2>&1 &
NAV2_PID=$!
PIDS+=($NAV2_PID)

echo "⏳ Waiting for Nav2 stack to initialize..."
for i in {1..35}; do
    if ros2 topic list 2>/dev/null | grep -q "/amcl_pose"; then
        echo -e "\033[1;32m✅ Nav2 stack is ONLINE!\033[0m"
        break
    fi
    sleep 1
    if [ $i -eq 35 ]; then
        echo -e "\033[1;33m⚠️  Nav2 startup is taking longer than expected. Continuing...\033[0m"
    fi
done

# ----------------------------------------------------------------------------
# 3. RViz2
# ----------------------------------------------------------------------------
echo -e "🚀 3. Launching RViz2 Visualization..."
echo -e "    👉 Logs redirected to: \033[1;34m$LOG_DIR/rviz2.log\033[0m"
rviz2 -d "$HOME/hospital_robot_ws/hospital_nav.rviz" > "$LOG_DIR/rviz2.log" 2>&1 &
RVIZ_PID=$!
PIDS+=($RVIZ_PID)

echo "⏳ Giving RViz2 a moment to render..."
for i in {1..10}; do
    if kill -0 "$RVIZ_PID" 2>/dev/null; then
        echo -e "\033[1;32m✅ RViz2 is running!\033[0m"
        break
    fi
    sleep 1
done

# ----------------------------------------------------------------------------
# 4. Initial Pose for AMCL
# ----------------------------------------------------------------------------
echo -e "🚀 4. Publishing Initial Pose for AMCL..."
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
'{
header: {frame_id: "map"},
pose: {
  pose: {
    position: {x: -2.61, y: 1.88, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}
}' > "$LOG_DIR/initialpose.log" 2>&1

echo -e "\033[1;32m✅ Initial pose published!\033[0m"

# ----------------------------------------------------------------------------
# 5. Voice-Controlled Delivery Node (foreground)
# ----------------------------------------------------------------------------
echo ""
echo -e "🚀 5. Starting Voice Delivery Node in foreground..."
echo -e "    👉 Terminal input/output is connected to the Voice Delivery Node."
echo -e "    👉 Speak commands like \033[1;33m'Go to Room 1'\033[0m, or press \033[1;31mCtrl+C\033[0m to shut down the entire simulation.\n"

cd "$HOME/hospital_robot_ws" || exit 1
source whisper_env/bin/activate
source /opt/ros/humble/setup.bash
source "$HOME/hospital_robot_ws/install/setup.bash"

python3 voice_delivery_node.py