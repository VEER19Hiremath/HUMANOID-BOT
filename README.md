# HUMANOID-BOT — Hospital Delivery Robot

ROS 2 **Jazzy** workspace for a differential-drive hospital delivery robot: voice room commands, Nav2 navigation, RPLIDAR, and Arduino Mega + matched **BLDC-5015A** wheel drivers (left + right).

Repository: [VEER19Hiremath/HUMANOID-BOT](https://github.com/VEER19Hiremath/HUMANOID-BOT)

---

## Hardware

| Part | Notes |
|------|--------|
| Raspberry Pi / Ubuntu PC | ROS 2 Jazzy |
| Arduino Mega 2560 | USB to Pi; driver control + wheel-speed feedback |
| BLDC-5015A drivers (×2) | Matched left + right; battery on **DC+/DC−** |
| Brushless hub motors (×2) | Phase U/V/W + Hall plug into each driver |
| Wheel-speed feedback (×2) | One pulse wire per driver into D2/D3 (no separate encoders) |
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

### 1. Battery → both BLDC-5015A drivers

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
| **Purple** | **AVI** | **D10** | **D9** | Speed (PWM). Crossed on this harness: left driver ← D10, right ← D9 |
| **Yellow** | **F/R** | **D52** | **D53** | Direction |
| **Red** | **ENBL** | **D22** | **D23** | Run / stop (HIGH = run) |
| **Blue** | **BRK** | **D30** | **D31** | Brake (LOW = released) |
| **Black** | Common / GND | **GND** | **GND** | Signal ground (both drivers) |

> Bench-verified 2026-09-30: **yellow is direction, blue is brake.** An older version of this table had them the other way round. Each wheel then ran one direction only and stalled under PWM in the other, which caused the red driver LED, the "brake stuck" feel and the blown fuses. Always keep **purple → D9/D10** (speed). If purple ever lands on a direction or brake pin, the wheel can run away.

Same table by pin (quick check):

| Mega pin | Goes to |
|----------|---------|
| D10 | Left AVI (purple) |
| D9 | Right AVI (purple) |
| D52 | Left F/R (yellow) |
| D53 | Right F/R (yellow) |
| D22 | Left ENBL (red) |
| D23 | Right ENBL (red) |
| D30 | Left BRK (blue) |
| D31 | Right BRK (blue) |
| GND | Both driver commons (black) |

### 4. Wheel-speed feedback → Arduino Mega

Each driver's wheel-speed pulse wire goes to the Mega through a **1 kΩ** series resistor. The separate encoders are no longer used (they were never mounted on the wheels). **No 5V** goes to these wires; the Mega's internal pull-up reads them, and the black control wire already shares GND.

| Wire | From | Left Mega pin | Right Mega pin | Role |
|------|------|---------------|----------------|------|
| Speed pulse | Driver speed output (or motor Hall line) | **D2** via 1 kΩ | **D3** via 1 kΩ | Wheel pulses (D2/D3 are the only interrupt pins used) |

The firmware uses these pulses for three things:

- **Wheel speed control:** each wheel's speed is measured from the time between its pulses, and a PI loop sets the effort so the wheel turns at the commanded speed whatever its drag. Effort never goes above 32 (the fuse cap).
- **Odometry:** pulses are counted *signed*, in the direction the wheel is driven (a direction change waits until the wheel has coasted down). `base_controller` turns them into `/odom` using the metres-per-pulse in `src/hospital_bringup/config/odometry.yaml`, which it sends to the Mega at connect (`K:<left> <right>`).
- **Stall cut:** a driven wheel that gives no pulse for 0.5 s (after a 1 s start-up grace per wheel) stops both wheels (`STALL HARD STOP`) for a 10 s cool-down, then retries as a fresh start.

`odometry.yaml` holds provisional values until the floor is measured: run `python3 scripts/floor_calibrate.py --tape` once on the floor.

Bench tools (wheels lifted, stack stopped with `./start.sh stop`, Mega auto-detected):

| Script | Purpose |
|---|---|
| `scripts/phase_check.py` | 1.5 s forward + reverse; each wheel must show `OK — runs both ways` |
| `scripts/drive_test.py` | Drives both hubs slowly (3–4 s) with all firmware safeties on; `PASS` needs pulses from both wheels |
| `scripts/bench_matrix.py` | Full BRK / ENBL / F-R / speed case table per wheel (hands off) |
| `scripts/brake_test.py` | Holds raw BRK/ENBL levels (PWM 0) so you can feel which level brakes |
| `scripts/encoder_check.py` | Hand-turn each wheel and read the pulse counts (driver powered) |

### 5. USB / PC

| Device | Port on Pi |
|--------|------------|
| Arduino Mega | USB → often `/dev/ttyACM0` or `/dev/arduino` |
| RPLIDAR | USB → `/dev/ttyUSB0` or `/dev/lidar` |

Keep Mega and lidar on **separate** USB paths when possible. Motor EMI can drop the Mega (`disabled by hub (EMI?)`) while the lidar keeps spinning.

### 6. Power-up order

1. Mega USB to Pi (sketch boots, `SAFE START READY`, wheels must stay still)  
2. Control + speed-feedback wires as above  
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

Driver manual: [BLDC-5015A / BLDC-5015](https://www.ht-gear.com/uploads/BLDC5015-EN.pdf). Both sides must use the same switch settings (SW2 OFF, RV min).

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
3. **Place the robot in the MIDDLE of the 30 × 40 ft area, facing along the 40 ft side** (the default map `maps/area_30x40.yaml`; rooms and arrow in `docs/room_map.png`). Its position is counted from that start point, so start it in the same place and direction every time. Scanned maps (`./start.sh map`) use AMCL instead; for those, use RViz **2D Pose Estimate** if the lidar dots don't line up with the walls
4. Prefer **motor battery OFF** until the stack says ready (reduces USB EMI dropouts)
5. Then battery ON to drive

Logs: `run_logs/latest/` in the project, one folder per run (`base.log`, `nav.log`, `voice.log`, …), kept across reboots (newest 30 runs). `/tmp/hospital_robot_logs` points at the latest run. `log/` only holds `colcon build` logs.

### Map a new floor (one command)

The robot only navigates correctly on a map of the floor it is on.

1. Put the robot at the spot that should be **home**, facing the way it will always start (this becomes (0, 0) on the map).
2. Run:
   ```bash
   ./start.sh map
   ```
3. Drive with the keyboard **in the same terminal**: `i` forward, `,` back, `j`/`l` curve left/right, `k` stop. Speed is capped at 0.06 m/s. Go slowly along every wall and into every room. There is **no obstacle stop** while mapping.
4. The map **auto-saves every 30 s** to `maps/floor_map.*`. When it's complete, press **Ctrl+C**. That saves the final map.
5. In RViz pick **Publish Point** and click each spot when the terminal asks: home, room1, room2, … Enter skips a room. This writes `room_config.py` and AMCL's start pose.
6. Everything stops by itself. Put the robot at home and run `./start.sh`. It uses `maps/floor_map.yaml` automatically from now on (`MAP=maps/hospital_map.yaml ./start.sh` for the old lab map).

A previous floor map is kept as `maps/floor_map.bak.*`. To redo only the rooms: run `./start.sh`, then `python3 scripts/pick_rooms.py`.

### Safety while driving

- **Obstacle stop** (lidar, `collision_monitor`): stops for anything 0.27–0.65 m ahead in a 0.60 m wide zone, and drives at 70 % speed for 0.65–1.10 m ahead. `start.sh` prints `Obstacle stop: OK`. If it isn't active, the robot does not drive at all.
- **Microphone watchdog**: **off by default**. In a quiet room a live headset reads the same as a dead one (rms 1–3), so it stopped every trip. `mic_watchdog_s:=15.0` on the voice node turns it on. Because a dead headset can go unnoticed, keep the battery switch or `./start.sh stop` within reach.
- Always keep the motor battery switch, or `./start.sh stop`, within reach.

### Voice examples

After it says listening:

- `go to room one` / `room two` / `room three`. Say numbers as words: the speech model has no digits
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

Sketch: `arduino/wheelodom.ino`. Host tests (simulated wheels with drag, stalls, reversals, timeouts): `arduino/test/run_tests.sh`.

Test tools (stack running, wheels lifted or a clear floor):

| Script | What it checks |
|---|---|
| `scripts/wheel_speed_test.py` | Commanded vs measured speed: straight, slow, arcs, reverse, direction change, stop time |
| `scripts/e2e_test.py [N]` | N random trials through the whole chain (voice command → Nav2 → base → Mega): trips, stop mid-trip, redirects, noise phrases. Results in `run_logs/latest/e2e_results.json` |
| `scripts/plan_check.py` | Plans every room pair (no driving): length, reverses, turn radius, wall clearance |
| `scripts/floor_calibrate.py --tape` | Floor calibration of metres per pulse (drives 1.5 m after Enter) |

Type a command instead of speaking: `ros2 topic pub --once /voice_command std_msgs/String "data: go to room one"`.

---

## Build notes

- Ignore `build/`, `install/`, `log/` (see `.gitignore`)
- Optional: `~/venv` for voice deps; system needs `gTTS`, `vosk`, `sounddevice`, `ffplay` or `mpg123`
- Udev helpers often symlink Mega → `/dev/arduino`, lidar → `/dev/lidar`

---

## Known issues (demo)

| Symptom | Cause | What is in place / what to do |
|---|---|---|
| Wheels fight each other, chassis spins in place | Hubs face opposite ways; same F/R polarity on both sides | Firmware `LEFT_INVERTED=true`, `RIGHT_INVERTED=false`. Opposite-sign wheel commands are blocked in firmware. |
| Fuse blows | A hub stalled under PWM (one direction never ran, see the F/R row below) or a jammed wheel | Fixed root cause (F/R and BRK pins). Also: PWM capped at 30–32, slow ramp up, 300 ms command timeout, opposite-sign commands blocked, `TEST` disabled, and the **stall cut** stops both wheels within ~0.5 s if a wheel gives no speed pulses. Keep one correctly rated fuse per driver. |
| `STALL HARD STOP` in `logs/base.log`, bot stops for 10 s | A wheel gave no speed pulses for 0.5 s while driven: jammed wheel, driver fault (red LED), or a loose D2/D3 feedback wire | Clear the jam, power-cycle the battery if the LED is red, check the feedback wire, then run `scripts/drive_test.py` (wheels lifted). |
| `ENC RIGHT SUSPECT` (or `LEFT`) warning | Only with `ENC_SUSPECT_BYPASS = true`: one feedback line frozen while the other counts | Default is `false` (any frozen wheel = hard stop, which protects the fuses). Turn it on only to keep driving with a known-bad feedback wire. |
| Robot drives in circles; turns go the wrong way | The two **purple speed wires are crossed**: D9 drives the D23/D31/D3 driver and D10 the D22/D30/D2 one (bench 2026-09-30, one wheel at a time). "Slow the left wheel to turn left" slowed the right one, so every turn was reversed. On the floor this looked like an inverted speed signal; with the wheels lifted it's normal (a higher byte is faster) | Fixed in firmware: `L_AVI` = D10, `R_AVI` = D9. `arduino/test/run_tests.sh` checks the pin map. Different drag/speed per wheel is now handled by the firmware's wheel speed control. |
| Robot drives for minutes and never arrives | The floor isn't the mapped room (the map is the 40 × 28 ft lab; the test floor is ~18 ft) | Map the floor you drive on (see **Map a new floor**). |
| Robot doesn't stop or slow for people or objects | Collision monitor and costmap obstacle layers were all disabled | Fixed: `start.sh` runs the collision monitor (see **Safety while driving**). |
| `stop` not heard while driving | The headset mic went silent after the robot spoke (flat signal in `hfp.log`) | The microphone watchdog now stops the robot. Keep the headset close; if it happens, power-cycle the headset link. |
| Robot leaves the map in RViz, or drives somewhere other than where Nav2 thinks | AMCL was disabled. The pose came only from commanded speeds, pinned to a fixed map→odom, so the drift was never corrected | Fixed: `localization.launch.py` runs AMCL (lidar localization), and `start.sh` passes `static_map_odom:=false`. Start the robot at home, or set **2D Pose Estimate** in RViz. |
| Says "Arrived" but the robot is somewhere else (or still stuck near the start) | Pose came from **commanded** speeds (`open_loop_odom`). Log 2026-09-30: the robot stalled 9 s into a trip, the Mega cut drive 9 times, but the pose kept advancing and Nav2 "reached" room1 99 s later | Fixed: odometry comes only from the wheels' signed pulse counts (the command-based mode is removed). A stuck robot fails the goal instead of faking arrival. Calibrate metres per pulse once on the floor (`scripts/floor_calibrate.py --tape`). |
| Nav2 running but bot never moves; `/cmd_vel_nav` has no subscribers | `velocity_smoother` not activated | `velocity_smoother` is in the lifecycle list of `navigation.launch.py`. Check with `ros2 lifecycle get /velocity_smoother`. |
| Goals abort with `Failed to create plan` | The goal or the robot sits inside a wall's inflation | Planner `tolerance: 0.25`. Keep room goals at least 0.6 m from walls in `room_config.py`, then run `scripts/plan_check.py` (plans every room pair without moving the wheels). |
| Path cuts across a wall end in RViz; robot gets stuck in a small room it can't turn around in | Navfn drew sharp corners the arcs-only base can't drive (tightest radius ~0.54 m in `base_controller`); the robot swung wide. And since composition, the **costmaps ignored `nav2_params.yaml`** (Nav2 defaults: 0.1 m radius, no footprint), because the container itself wasn't given the file | Fixed: **Smac Hybrid-A\*** planner (`minimum_turning_radius: 0.60`, Reeds-Shepp = short reverse legs like a 3-point turn), RPP `allow_reversing: true`, the real body footprint 0.54 × 0.50 m, inflation 0.60 m, the params file passed to the container. Goals point along the line from the robot to the room (`goal_yaw`). `scripts/plan_check.py`: 30/30 paths inside the area, body ≥ 0.07 m from every drawn wall. |
| Robot stops for no reason; `collision_monitor: Robot to stop due to invalid source` | On a loaded Pi the odometry TF ran 1–2 s late, so matching each scan to the pose at its exact time failed and the obstacle stop held the robot | `base_shift_correction: false` (uses the latest pose; < 0.1 m error at 0.06 m/s) and replanning every 3 s instead of every 1 s. Close extra programs (browser, VS Code) on the Pi while driving. |
| Robot drives off to room 1 or room 2 by itself | Vosk turned background talk / motor noise into bare words (`one`, `the one`, `a one`, `two`); each was taken as a command (6 times in 10 min on the bench) | Fixed: a room needs a full phrase (`room one`, `go to room one`, `go to the one`, `take me to room four`, `go home`, `reception`). Bare numbers are logged as `Ignoring short phrase` and ignored. `stop` alone still works. |
| `start.sh` hangs during bringup | Lifecycle/DDS calls (map, collision monitor, `/scan`) block | Every check is wrapped in `timeout 2` and only prints a warning. |
| One wheel on straight / both on turn, red LED on driver | Driver latch, F/R, or current brownout | `motorEnable()` toggles ENBL stop→run to clear the latch, and stopping coasts instead of braking. If the red LED stays on, power-cycle the motor battery. |
| Forward: only left turns. Reverse: only right turns. Wheel "tries but can't", red driver LED, fuses blow | Firmware drove the blue wire as direction and the yellow wire as brake, but on the drivers it's the other way round. So each hub ran one way only and stalled under PWM the other way | Fixed in firmware: F/R is on D52/D53 (yellow) and BRK on D30/D31 (blue). `scripts/phase_check.py` must show both wheels `OK — runs both ways`. |
| `start.sh` prints `Obstacle stop: PROBLEM (unconfigured)` and the robot never drives | One of ~10 separate Nav2 processes hung while joining DDS at startup (a different one each time), so Nav2's lifecycle manager waited forever (bench 2026-09-30: 3 of 3 starts) | Fixed: Nav2 runs **composed** (all servers in one `component_container_isolated`, `use_composition:=true`, as in Nav2's own bringup): 4 of 4 starts were healthy. `start.sh` also restarts once if the health check fails, and `stop` cleans stale Fast DDS shared memory (`fastdds shm clean`). |
| Robot swings wide in turns and cuts through walls in RViz / leaves the area | The slowest speed the firmware used (PWM 22) still ran a wheel at ~70 % of cruise, so the inner wheel couldn't slow down: the tightest arc was ~1.5 m wide. The controller's collision check was off, and the `base_controller` map clamp froze the pose at its rectangle edge (goal timeouts) | Fixed: speed maps to effort **in proportion** (`PWM_ZERO_SPEED` 5, cruise 30, min 10; lifted bench: 6 → 9/s … 30 → 56/s), so turn ratios are real. RPP `use_collision_detection: true` stops before a mapped wall. The clamp only applies to open-loop odometry. |
| On the floor: `STALL HARD STOP` every 10 s and the robot never moves again | After a stall's 10 s cool-down the firmware restarted with the **old** motion-start time and stall timers: no start grace, no breakaway kick, and the PWM is up within a millisecond. A loaded wheel that needed a moment to start re-faulted 40 ms later, forever (floor log 2026-09-30) | Fixed: a fault and its cool-down reset `motion_start_ms`, so every retry is a fresh start. Start grace 1000 ms, breakaway 300 ms (was 400 / 150). Obstacle slow zone 70 % speed (50 % = 0.03 m/s was too little effort on the floor). Test: `arduino/test/run_tests.sh`. |
| Tight turns stall every 10 s (`stall/fault` in `base.log`) | Inner-wheel commands of 0.01–0.02 m/s got PWM 0 while still counting as "moving" | Fixed: a moving wheel always gets at least `PWM_START`. |
| Robot detours, loops or reverses "randomly" on the test floor | It planned around the drawn 30 × 40 ft walls, which don't exist on the 18 ft floor | `start.sh` now defaults to **`maps/open_floor.yaml`** (20 × 20 ft test area, no interior walls; rooms 1.3–1.6 m from home, leaving room for forward-only turn-arounds). Rooms are listed in each map (`# room:` lines) and the voice node uses the loaded map's rooms. `MAP=maps/area_30x40.yaml ./start.sh` for the walled area; other floor sizes: `python3 scripts/make_area_map.py open_floor --size W H`. |
| Robot drives in circles / curves although RViz shows a straight path | Odometry scales are provisional: if one wheel's pulses are "shorter", odometry sees a turn that isn't there and Nav2 steers against it | Calibrate once on the floor: `python3 scripts/floor_calibrate.py --tape` (drives 1.5 m after you press Enter, asks for the measured forward/sideways distance, writes `config/odometry.yaml`). |
| Robot ignores things straight ahead; stops for things beside it | The lidar sits inside the body: only ~20 % of beams return and almost none from −40° to +40° (straight ahead). Its open views are behind, the front corners (±45–60°) and the left | No software fix: the obstacle stop **cannot see straight ahead**. Stay at the battery switch. `start.sh` prints `Lidar view: WARNING` while this is so. An opening in the body ahead of the lidar (or mounting it on top) fixes it. The stop zone also watches **behind** the robot while it reverses. |
| Robot sets off or changes destination by itself | Vosk turned noise into phrases like `[unk] gotta toronto` or a bare `toronto` | Phrases with unrecognised sound (`[unk]`) never send or redirect the robot; `toronto` only counts as `go to toronto`. `stop` always works. |
| `stop` acted on ~20 s late, or not at all | The recognizer fell ~20 s behind the headset audio while the Pi was busy driving | It never lags more than ~1 s now (older audio is dropped, logged as `behind: dropped`); cheaper mic level maths; RViz runs at low priority. |
| Robot curves / circles; travels further than the goal; "forgets where to go" | The firmware set a fixed effort per speed, so a wheel with more drag turned slower and the robot curved; the base controller also added guessed motion between pulse reports (double counting) and had several half-wired odometry modes | Rewritten end to end: closed-loop wheel speed in the firmware, pulse-only signed odometry in `base_controller`. Bench: speeds within 3 % of the command, stop in 0.65 s; 100/100 end-to-end trials. |
| `stop` ignored now and then | "stop" cancelled the goal through its handle, which is missing when Nav2's "accepted" reply is lost | "stop" now cancels **all** Nav2 goals (no handle needed), acts the moment it's heard, and zeroes the wheels. 25/25 stop trials: wheels still in ≤ 0.8 s. |
| Robot stops ~0.3 m short of a room | Goal tolerance was 0.30 m | `xy_goal_tolerance: 0.15`: 100 trials arrived within 0.18 m (odometry). |
| Robot spins on the spot ("tornado"), says "Arrived" somewhere else | Wheel DIRECTION CHANGES: the left driver doesn't reliably follow an F/R change made soon after running, so after a reverse leg one wheel kept going backwards. The pulses carry no direction, so odometry saw normal driving | Navigation is **forward-only** (Smac DUBIN planner, no BackUp recovery, no reverse speed), and the firmware switches a driver off while its direction changes (re-key). Direction settings stay `LEFT_INVERTED true`, `RIGHT_INVERTED false` (a straight run from rest goes straight). |
| Robot drives much further and faster than told, crosses the boundary | The drivers give only ~12-13 speed pulses per wheel turn; the guessed 2.5 mm per pulse was ~15x too small | **Measured on the floor (tape at both wheels): 36 mm per pulse, right 0.92 of left** (`config/odometry.yaml`). Cruise 0.15 m/s (real). Re-measure after any wheel, driver or wiring change. Counting both pulse edges was tried and dropped: the right signal adds false edges. |
| Robot creeps into a wall in short stop/go steps | The collision monitor alone sees the few lidar points ahead appear and vanish: each 'continue' let it creep forward | Both costmaps now have a lidar **obstacle layer**: walls and objects stay marked until the lidar sees through them, the planner routes around them and the controller stops before them. |
| "go home" heard as just "home" (ignored) | The mic gate fed the recognizer only audio above the noise floor: the soft start of a phrase and the gaps between words were cut | The last 0.5 s before speech is fed with it, and audio keeps flowing 0.8 s after it. |
| Only one particular headset worked | `hfp_mic.py` had the AirPods' address built in and handled only the mSBC codec | Any paired hands-free headset (connected first, then the others) with mSBC or CVSD; without one, the system mic (USB / wired). New headset: pair it once (`bluetoothctl`: `pair`, `trust`, `connect`). |
| A wheel stalls on slow turns / at the start on the floor | Floor friction stops a slow inner wheel; the PI loop was too slow to free it | Stuck-wheel **kick** (breakaway effort 24 for up to 0.4 s when pulses are missed); start-up push until the first pulse; stall cut waits for 3 pulse gaps. |
| Robot curves on straight lines | Different drag per wheel; small speed differences add up | Per-wheel speed control **plus wheel synchronisation**: the real distances of the two wheels are kept at the commanded ratio (sim: 1 count apart vs 21 without). |
| One wheel gives 0 pulses / won't move (bench test) | Driver latched (red LED) | Battery off 10 s, then on. `scripts/direction_level_test.py` / `brake_level_test.py` test each wheel alone. |
| Mega USB disappears | Motor EMI (`disabled by hub (EMI?)`) | Turn the battery off, reseat the Mega on another USB port, then run `./start.sh` (the script waits up to 90 s for the device). |

Lidar can spin while Mega is missing — they are separate USB devices; start still requires the Arduino.

---

## License

Package licenses are declared in each package’s `package.xml` (mostly Apache-2.0 where specified). Third-party lidar SDK code retains upstream licenses.
