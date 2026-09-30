#!/bin/bash
# One-time setup of the hospital delivery robot on a fresh Raspberry Pi /
# Ubuntu 24.04 machine. Safe to run again (each step skips what is done).
#
#   ./setup.sh            install everything and build the workspace
#   ./setup.sh --flash    also compile + upload arduino/wheelodom.ino to the Mega
#   ./setup.sh --test     also run the firmware and base-controller unit tests
#
# What it does:
#   1. ROS 2 Jazzy (if missing) + the ROS packages the robot uses
#   2. System packages: Python libs, audio (PipeWire, ALSA, BlueZ, SBC codec)
#   3. ~/venv with the speech packages (requirements.txt)
#   4. The Vosk speech model in ~/models
#   5. arduino-cli + the AVR core in .tools/ (for flashing the Mega)
#   6. Serial-port access for this user (dialout group)
#   7. colcon build of the workspace
# Docs: docs/HANDOVER.md

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FLASH=false
TEST=false
for arg in "$@"; do
  case "${arg}" in
    --flash) FLASH=true ;;
    --test) TEST=true ;;
    *) echo "Usage: $0 [--flash] [--test]"; exit 1 ;;
  esac
done

step() { echo; echo "=== $* ==="; }

if [[ "$(. /etc/os-release && echo "${VERSION_ID}")" != "24.04" ]]; then
  echo "WARNING: written for Ubuntu 24.04 (ROS 2 Jazzy). Continuing anyway."
fi

# --- 1. ROS 2 Jazzy ---------------------------------------------------------
step "1/7 ROS 2 Jazzy"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt-get update
  sudo apt-get install -y ros-jazzy-ros-base
fi
sudo apt-get install -y \
  ros-jazzy-navigation2 ros-jazzy-nav2-smac-planner ros-jazzy-nav2-collision-monitor \
  ros-jazzy-nav2-velocity-smoother ros-jazzy-nav2-map-server ros-jazzy-nav2-amcl \
  ros-jazzy-nav2-lifecycle-manager ros-jazzy-nav2-regulated-pure-pursuit-controller \
  ros-jazzy-rclcpp-components ros-jazzy-robot-state-publisher ros-jazzy-tf2-ros \
  ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard ros-jazzy-rviz2 \
  python3-colcon-common-extensions python3-rosdep

# --- 2. System packages -----------------------------------------------------
step "2/7 System packages"
sudo apt-get install -y \
  python3-serial python3-numpy python3-scipy python3-matplotlib python3-venv python3-pip \
  alsa-utils pipewire pipewire-pulse wireplumber libspa-0.2-bluetooth bluez libsbc1 \
  speech-dispatcher g++ unzip wget curl git

# --- 3. Python venv for speech -----------------------------------------------
step "3/7 Python packages (~/venv)"
if [[ ! -x "${HOME}/venv/bin/python3" ]]; then
  python3 -m venv --system-site-packages "${HOME}/venv"
fi
"${HOME}/venv/bin/pip" install --quiet -r "${ROOT}/requirements.txt"

# --- 4. Vosk speech model ----------------------------------------------------
step "4/7 Speech model"
MODEL="${HOME}/models/vosk-model-small-en-us-0.15"
if [[ ! -d "${MODEL}" ]]; then
  mkdir -p "${HOME}/models"
  wget -q --show-progress -O "${MODEL}.zip" \
    https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
  unzip -q "${MODEL}.zip" -d "${HOME}/models"
fi
echo "Model: ${MODEL}"

# --- 5. arduino-cli ----------------------------------------------------------
step "5/7 arduino-cli (Mega firmware tools)"
mkdir -p "${ROOT}/.tools"
if [[ ! -x "${ROOT}/.tools/arduino-cli" ]]; then
  ARCH="$(uname -m)"
  case "${ARCH}" in
    aarch64) PKG=Linux_ARM64 ;;
    x86_64) PKG=Linux_64bit ;;
    armv7l) PKG=Linux_ARMv7 ;;
    *) echo "Unknown CPU ${ARCH}: install arduino-cli by hand into .tools/"; exit 1 ;;
  esac
  wget -q -O "${ROOT}/.tools/arduino-cli.tgz" \
    "https://github.com/arduino/arduino-cli/releases/download/v1.1.1/arduino-cli_1.1.1_${PKG}.tar.gz"
  tar -xzf "${ROOT}/.tools/arduino-cli.tgz" -C "${ROOT}/.tools" arduino-cli
fi
"${ROOT}/.tools/arduino-cli" core update-index > /dev/null
"${ROOT}/.tools/arduino-cli" core install arduino:avr > /dev/null
echo "arduino-cli: $("${ROOT}/.tools/arduino-cli" version | head -1)"

# --- 6. Serial access ----------------------------------------------------------
step "6/7 Serial port access"
if ! id -nG "${USER}" | grep -qw dialout; then
  sudo usermod -aG dialout "${USER}"
  echo "Added ${USER} to 'dialout': log out and back in once for it to apply."
else
  echo "${USER} is in 'dialout'."
fi

# --- 7. Build ------------------------------------------------------------------
step "7/7 Build the workspace"
set +u   # ROS setup scripts use unset variables
source /opt/ros/jazzy/setup.bash
set -u
cd "${ROOT}"
colcon build --packages-select wheel_odometry hospital_delivery hospital_bringup \
  hospital_description hospital_world rplidar_ros

if ${TEST}; then
  step "Unit tests"
  "${ROOT}/arduino/test/run_tests.sh" | tail -1
  (cd "${ROOT}/src/wheel_odometry" && python3 -m pytest -q test/test_base_controller_math.py | tail -1)
fi

if ${FLASH}; then
  step "Flash the Mega"
  PORT="$(ls /dev/serial/by-id/usb-Arduino* 2>/dev/null | head -1)"
  if [[ -z "${PORT}" ]]; then
    echo "No Arduino found on USB: plug in the Mega and run ./setup.sh --flash again."
    exit 1
  fi
  BUILD_DIR="$(mktemp -d)/wheelodom"
  mkdir -p "${BUILD_DIR}"
  cp "${ROOT}/arduino/wheelodom.ino" "${BUILD_DIR}/"
  "${ROOT}/.tools/arduino-cli" compile --upload -p "${PORT}" --fqbn arduino:avr:mega "${BUILD_DIR}"
fi

echo
echo "Done. Next: pair a Bluetooth headset (or plug in a USB mic), put the robot"
echo "at home and run ./start.sh. Full guide: docs/HANDOVER.md"
