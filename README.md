# HUMANOID-BOT — Hospital Delivery Robot

ROS 2 **Jazzy** workspace for a differential-drive hospital delivery robot: voice room commands, Nav2 navigation, RPLIDAR, and Arduino Mega + BLDC-5015 wheel drivers.

Repository: [VEER19Hiremath/HUMANOID-BOT](https://github.com/VEER19Hiremath/HUMANOID-BOT)

---

## Hardware

| Part | Notes |
|------|--------|
| Raspberry Pi / Ubuntu PC | ROS 2 Jazzy |
| Arduino Mega 2560 | USB to Pi; control + encoder I/O |
| BLDC-5015 drivers (×2) | Left + right wheel; battery on **DC+/DC−** |
| Brushless hub motors (×2) | Phase U/V/W + Hall plug into each driver |
| Wheel encoders (×2) | Powered from Mega **5V**; channel A into D2/D3 |
| RPLIDAR A2M8 | USB (`/dev/lidar` or `ttyUSB0`) |
| Mic + speaker | Voice in / Google TTS out |

Driver switches (both sides):

| Setting | Position |
|---------|----------|
| **SW2** | **OFF** (speed from Mega AVI, not the knob) |
| **SW1** | Leave as-is (do not change for demo) |
| Round knob **RV** | Fully **counter-clockwise** (min) when using AVI |

Motor battery powers the drivers only (24–50 V on DC+/DC−). **Never** power drivers from Mega 5V. After a latched one-wheel fault: battery **OFF → wait ~10 s → ON**.

Firmware pin map: `arduino/wheelodom.ino`. Cable colors below match the **current robot harness** used in this project.

---

## Wiring (detailed)

Do all control wiring with the **motor battery disconnected**. Connect the battery last.

### 1. Battery → both BLDC-5015 drivers

| Battery | Left driver | Right driver |
|---------|-------------|--------------|
| + (24–50 V) | **DC+** | **DC+** |
| − | **DC−** | **DC−** |

Check polarity twice. Nothing else on DC+/DC−.

### 2. Motor → driver (each side)

| Motor cable | Driver terminal |
|-------------|-----------------|
| Three thick phase wires | **U**, **V**, **W** (keep the same order; do not swap for “direction”) |
| Small Hall sensor plug | Hall socket (**REF+**, **REF−**, **HU**, **HV**, **HW**) — push fully in |

Direction is controlled by the Mega **F/R** pin, not by swapping phases.

### 3. Driver control cable → Arduino Mega

Opto commons on the drivers share **+5 V / Vcc** on the driver side; Mega **black** wires are signal **GND** back to the Mega.

| Cable color | Driver signal | Left Mega pin | Right Mega pin | Role |
|-------------|---------------|---------------|----------------|------|
| **Purple** | **AVI** | **D9** | **D10** | Speed (PWM) |
| **Blue** | **F/R** | **D30** | **D31** | Direction |
| **Red** | **ENBL** | **D22** | **D23** | Run / stop |
| **Yellow** | **BRK** | **D52** | **D53** | Brake line |
| **Black** | Common / GND | **GND** | **GND** | Signal ground (both drivers) |

> If purple and yellow were ever swapped, AVI ends up on the brake pins and the wheels can run away when the sketch writes BRK HIGH. Always keep **purple → D9/D10** and **yellow → D52/D53** on this robot.

Same table by pin (quick check):

| Mega pin | Goes to |
|----------|---------|
| D9 | Left AVI (purple) |
| D10 | Right AVI (purple) |
| D30 | Left F/R (blue) |
| D31 | Right F/R (blue) |
| D22 | Left ENBL (red) |
| D23 | Right ENBL (red) |
| D52 | Left BRK (yellow) |
| D53 | Right BRK (yellow) |
| GND | Both driver commons (black) |

### 4. Wheel encoders → Arduino Mega

Encoders need **5V**. Share Mega **5V** and **GND** (breadboard rails are fine). Do **not** feed encoder 5V from the motor battery.

| Encoder wire | Mega | Left | Right | Used by firmware? |
|--------------|------|------|-------|-------------------|
| **Black** | **GND** | GND | GND | Yes (power return) |
| **Yellow** | **5V** | 5V | 5V | Yes (encoder power) |
| **Blue** (channel A) | Digital | **D2** | **D3** | Yes (counts) |
| **Green** (channel B) | Digital | unused (or D4) | unused (or D5) | No — sketch reads A only |

Typical breadboard layout:

1. Mega **5V** → red rail; Mega **GND** → blue rail  
2. Both encoder **yellow** → red rail; both **black** → blue rail  
3. Left **blue** → **D2**; right **blue** → **D3**  
4. Mega ON LED must stay **bright**. If it goes dim, a 5V/GND short is on the breadboard — unplug and fix before connecting the battery  

Demo `start.sh` uses `odom_source:=command` (odometry from commanded speeds). Encoders can stay wired for later `encoder` mode.

### 5. USB / PC

| Device | Port on Pi |
|--------|------------|
| Arduino Mega | USB → often `/dev/ttyACM0` or `/dev/arduino` |
| RPLIDAR | USB → `/dev/ttyUSB0` or `/dev/lidar` |

Keep Mega and lidar on **separate** USB paths when possible. Motor EMI can drop the Mega (`disabled by hub (EMI?)`) while the lidar keeps spinning.

### 6. Power-up order

1. Mega USB to Pi (sketch boots, `SAFE START READY`, wheels must stay still)  
2. Control + encoder wires as above  
3. Confirm driver LEDs are **off** at idle  
4. Motor battery ON last  
5. `./start.sh`  

### 7. Idle LEDs (healthy)

| Light | Idle expectation |
|-------|------------------|
| Pi / PC power | On |
| Mega ON | Bright |
| Mega TX | May flicker (encoder / status prints) |
| Left / right BLDC LED | **Off** (on = alarm / fault — power-cycle battery) |

Driver manual: [BLDC-5015](https://www.ht-gear.com/uploads/BLDC5015-EN.pdf).

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
