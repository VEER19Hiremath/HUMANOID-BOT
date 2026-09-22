#!/bin/bash

echo "========================================="
echo " Hospital Delivery Robot Startup"
echo "========================================="

# ----------------------------
# Gazebo + Robot Spawn
# ----------------------------

gnome-terminal --title="Hospital World" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/hospital_robot_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

echo 'Starting Hospital World...'
ros2 launch hospital_world hospital_world.launch.py

exec bash
"

echo "Waiting for Gazebo to start..."
sleep 10

# ----------------------------
# Nav2 Bringup
# ----------------------------

gnome-terminal --title="Nav2 Bringup" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/hospital_robot_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

echo 'Starting Nav2 Bringup...'

ros2 launch nav2_bringup bringup_launch.py \
map:=/home/saikirtan/hospital_robot_ws/maps/hospital_map.yaml \
use_sim_time:=True

exec bash
"

echo "Waiting for Nav2..."
sleep 10

# ----------------------------
# RViz2
# ----------------------------

gnome-terminal --title="RViz2" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/hospital_robot_ws/install/setup.bash

echo 'Starting RViz2...'

rviz2 -d ~/hospital_robot_ws/hospital_nav.rviz

exec bash
"

echo "Waiting for RViz..."
sleep 4

# ----------------------------
# Initial Pose for AMCL
# ----------------------------

gnome-terminal --title="Initial Pose" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/hospital_robot_ws/install/setup.bash

echo 'Publishing Initial Pose...'

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
'{
header: {frame_id: "map"},
pose: {
  pose: {
    position: {x: -2.61, y: 1.88, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}
}'

exec bash
"

sleep 3

# ----------------------------
# Voice Control
# ----------------------------

gnome-terminal --title="Voice Delivery" -- bash -c "
cd ~/hospital_robot_ws

source whisper_env/bin/activate
source /opt/ros/humble/setup.bash
source ~/hospital_robot_ws/install/setup.bash

echo 'Starting Voice Delivery Node...'

python3 voice_delivery_node.py

exec bash
"

echo ""
echo "========================================="
echo " System Startup Complete"
echo "========================================="
echo ""
echo "1. Verify robot appears on the map."
echo "2. Check AMCL pose is correct."
echo "3. Speak: Go to Room 1"
echo ""
