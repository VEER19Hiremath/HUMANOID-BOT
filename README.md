# HUMANOID-BOT — Hospital Delivery Robot

ROS 2 **Jazzy** workspace for a differential-drive hospital delivery robot: voice room commands, Nav2 navigation, RPLIDAR, and Arduino Mega + BLDC-5015 wheel drivers.

Repository: [VEER19Hiremath/HUMANOID-BOT](https://github.com/VEER19Hiremath/HUMANOID-BOT)

---

## Hardware

| Part | Notes |
|------|--------|
| Raspberry Pi (or Ubuntu PC) | ROS 2 Jazzy |
| Arduino Mega 2560 | Wheel PWM / ENBL / F/R / BRK |
| BLDC-5015 drivers (×2) | **SW2 OFF**, speed knobs fully **CCW** |
| RPLIDAR A2M8 | USB (`/dev/lidar` or `ttyUSB0`) |
| Mic + speaker | Voice in / Google TTS out |

**Arduino control pins (current firmware):**

| Signal | Left | Right |
|--------|------|-------|
| AVI (speed) | D9 | D10 |
| F/R | D30 | D31 |
| ENBL | D22 | D23 |
| BRK | D52 | D53 |
| Encoder A | D2 | D3 |
| Commons | Mega GND | Mega GND |

Motor battery powers the drivers (not USB). After a one-wheel fault: battery **OFF → wait ~10 s → ON**.

---

## Quick start

```bash
cd ~/Desktop/veeresh/hospital_robot_ws   # or your clone path
# build once
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# run
./start.sh          # base + lidar + map + Nav2 + RViz + voice
./start.sh status
./start.sh stop
```

**Before `./start.sh`:**
1. Mega USB plugged in (`/dev/ttyACM*` or `/dev/arduino`)
2. Lidar USB connected
3. Prefer **motor battery OFF** until the stack says ready (reduces USB EMI dropouts)
4. Then battery ON to drive

Logs: `/tmp/hospital_robot_logs/`

### Voice examples

After it says listening:

- `go to room one` / `room 2` / `room three`
- `go home` / `reception`

Speech uses **Google TTS** (needs network); falls back to `espeak` if offline.

---

## Packages (`src/`)

| Package | Role |
|---------|------|
| `hospital_bringup` | `real_robot.launch.py`, Nav2 params, localization |
| `hospital_delivery` | Voice delivery node, RViz config, rooms |
| `hospital_description` | URDF / meshes |
| `hospital_world` | Gazebo world (sim) |
| `wheel_odometry` | `base_controller` (Arduino + odom) |
| `rplidar_ros` / `sllidar_ros2` | Lidar drivers |

Demo defaults in `start.sh`:

- `odom_source:=command` (commanded speeds → odom; encoders optional)
- `idle_close_s:=5.0` (closes Arduino serial when parked — less EMI)
- Static `map` → `odom` at start pose `(2.2, 1.6)`
- Lidar obstacle layers disabled in `nav2_params.yaml` when the hospital map ≠ current room (re-enable after remapping)

---

## Arduino firmware

```bash
# Flash Mega (example)
arduino-cli compile -b arduino:avr:mega arduino/wheelodom
arduino-cli upload -b arduino:avr:mega -p /dev/ttyACM0 arduino/wheelodom
```

Sketch: `arduino/wheelodom.ino`

---

## Build notes

- Ignore `build/`, `install/`, `log/` (see `.gitignore`)
- Optional: `~/venv` for voice deps; system needs `gTTS`, `vosk`, `sounddevice`, `ffplay` or `mpg123`
- Udev helpers often symlink Mega → `/dev/arduino`, lidar → `/dev/lidar`

---

## Known issues (demo)

- **Mega USB disappears** under motor EMI (`disabled by hub (EMI?)`). Battery off, reseat Mega on another USB port, then `./start.sh` (script waits up to 90 s for the device).
- **One wheel on straight / both on turn** — often a driver latch or F/R / current brownout; power-cycle motor battery; firmware uses moderated straight PWM.
- Lidar can spin while Mega is missing — they are separate USB devices; start still requires the Arduino.

---

## License

Package licenses are declared in each package’s `package.xml` (mostly Apache-2.0 where specified). Third-party lidar SDK code retains upstream licenses.
