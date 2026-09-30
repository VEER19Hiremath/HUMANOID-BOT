// Mega firmware: two BLDC-5015A hub drivers with closed-loop wheel speed.
//
// Protocol
//   Host -> "VL:<m/s> VR:<m/s>\n"   wheel speeds; must repeat within 300 ms
//           "K:<left> [<right>]\n"  metres per speed pulse per wheel (host
//                                   sends its calibrated values at connect)
//           "PINS ..." / "PINS OFF" bench pin control (wheels lifted only)
//   Mega -> "SAFE START READY"
//           "L:<count> R:<count>"   every 50 ms: SIGNED pulse counts. The
//                                   driver pulses carry no direction, so each
//                                   pulse counts in the direction the wheel is
//                                   driven; F/R only flips once it has stopped,
//                                   so coasting pulses keep the old sign.
//           "S: pwmL=.. pwmR=.. enblL=.. enblR=.. brkL=.. brkR=.. brkOff=..
//               L=.. R=.. spL=.. spR=.. tgL=.. tgR=.."  while driving
//               (sp = measured, tg = target wheel speed, pulses/s)
//           "STALL HARD STOP"       a driven wheel gave no pulses: both cut
//           "K OK <left> <right>"
//
// Speed control: each wheel's speed is measured from the interval between
// its driver's speed pulses, and a PI loop sets the effort so the wheel turns
// at the commanded speed whatever its load. Open-loop effort let a wheel with
// more drag fall behind, so straight lines curved and Nav2 steered the robot
// in circles (floor, 2026-09-30).
//
// Fuse safety (unchanged rules): effort never above PWM_MAX; a driven wheel
// that stops pulsing cuts BOTH wheels for FAULT_COOLDOWN_MS; no host command
// for CMD_TIMEOUT_MS = motors off; wheels never driven in opposite directions;
// a direction change coasts to a stop first.
//
// Bench 2026-09-30: the yellow wire (D52/D53) is the driver's direction input
// and the blue wire (D30/D31) its brake (LOW = released). The purple speed
// wires are crossed: D10 feeds the D22/D30/D2 driver.
//
// Bench mode (wheels lifted): "PINS [L|R] B<0|1> E<0|1> F<0|1> A<effort>"
// holds raw BRK/ENBL/F-R levels and the speed input (capped at PWM_MAX) on one
// side or both; "W<0-255>" writes a raw speed byte (no cap, calibration
// only). "PINS OFF", any VL/VR command, or PIN_TEST_MS clears it.

// ================== PINS ==================
#define L_AVI   10   // purple (crossed: drives the D22/D30/D2 driver)
#define L_FR    52   // yellow
#define L_ENBL  22   // red
#define L_BRK   30   // blue
#define R_AVI   9    // purple (crossed: drives the D23/D31/D3 driver)
#define R_FR    53   // yellow
#define R_ENBL  23   // red
#define R_BRK   31   // blue
#define ENC_LEFT   2   // driver speed pulses, rising edges
#define ENC_RIGHT  3

// Hubs face opposite ways: the same wheel direction needs opposite F/R.
// Floor 2026-09-30: with this setting a straight run from rest went
// straight (1.6 m); with RIGHT_INVERTED true even a pure forward run spun.
// The spinning seen with this setting came from direction CHANGES the left
// driver didn't follow: hence the re-key below and forward-only navigation.
#define LEFT_INVERTED   true
#define RIGHT_INVERTED  false
const bool FR_OPEN_DRAIN = false;

// Mega HIGH = ENBL run; Mega LOW = brake released.
const bool INVERT_ENBL_BRK = true;
const bool ENBL_RUN  = INVERT_ENBL_BRK ? HIGH : LOW;
const bool ENBL_STOP = INVERT_ENBL_BRK ? LOW  : HIGH;
const bool BRK_RELEASE = INVERT_ENBL_BRK ? LOW : HIGH;
const bool L_BRK_RELEASE = BRK_RELEASE;
const bool R_BRK_RELEASE = BRK_RELEASE;

// ================== SPEED CONTROL ==================
const float VEL_DEADBAND = 0.01;      // m/s; below = wheel stopped
const float MAX_WHEEL_MPS = 0.20;     // command clamp
const float DEFAULT_M_PER_PULSE = 0.0402;   // measured on the floor 2026-09-30 (host sends its own)
// Feed-forward from the lifted-wheel curve (2026-09-30): pulses/s ~
// 2.3 * (effort - 5). Load needs more effort; the integrator adds it.
const float FF_ZERO_EFFORT = 5.0;
const float FF_PPS_PER_EFFORT = 2.3;
const float KP = 0.25;                // effort per pulse/s of error
const float KI = 1.0;                 // effort per (pulse/s * s)
const float INTEG_MAX = 25.0;
const float EFFORT_SLEW = 120.0;      // effort units per second, up and down
const unsigned long CONTROL_MS = 20;

const int PWM_START = 6;              // lowest effort for a driven wheel (lifted: ~2 pulses/s)
const int PWM_MAX = 32;               // fuse cap (stall current at ~40 blew fuses)
const int PWM_BREAKAWAY = 24;         // until a wheel's first pulse: 16 didn't start a wheel on the floor, 28 lurched
const unsigned long BREAKAWAY_MS = 600;     // floor: 300 ms didn't start a slow wheel
// Stuck-wheel kick: a driven wheel that misses its pulses (no pulse for 1.5
// expected gaps, >= 300 ms) gets breakaway effort for up to KICK_MS. Floor
// friction stopped a slow inner wheel (~2.5 pulses/s at effort 7) and the
// PI loop was too slow to free it before the stall cut (2026-09-30).
const unsigned long KICK_MS = 400;

const unsigned long PULSE_DEBOUNCE_US = 5000;  // real pulses >= 60 ms apart at our speeds
// Direction change: coast with effort 0 until the wheel is (nearly) still,
// or for at most REVERSE_COAST_MS (a lifted wheel spins down slowly), then
// switch THAT driver off, set F/R, and switch it on again REKEY_MS later.
// The left driver doesn't reliably follow an F/R change while enabled soon
// after running: the wheel kept turning the old way and the robot spun on
// the spot (floor 2026-09-30). A disabled driver takes the new direction.
const float REVERSE_FLIP_PPS = 1.5;
const unsigned long REVERSE_COAST_MS = 1500;
const unsigned long REKEY_MS = 300;
const unsigned long SPEED_TIMEOUT_US = 500000; // no pulse for 0.5 s = 0 speed

// Wheel synchronisation: while both wheels drive the same way, the real
// distance each has covered since the command last changed must keep the
// commanded ratio (1 = straight). The behind wheel is sped up and the other
// slowed by SYNC_GAIN m/s per metre of difference, at most SYNC_MAX. The
// per-wheel speed loop alone let start-up and drag differences add up, and
// the robot drifted off a straight line.
const float SYNC_GAIN = 1.0;
const float SYNC_MAX = 0.03;

const unsigned long CMD_TIMEOUT_MS = 300;
const unsigned long IDLE_ENBL_OFF_MS = 250;
const unsigned long REPORT_MS = 50;

// Stall cut: a driven wheel (effort >= STALL_PWM_MIN) with no pulse for
// STALL_CUT_MS, once STALL_GRACE_MS have passed since THAT wheel started
// (from rest, a reversal or a fault), cuts both wheels. Per wheel: a wheel
// reversing mid-drive restarts from rest and needs the grace too.
const unsigned long STALL_CUT_MS = 500;
const unsigned long STALL_GRACE_MS = 1000;   // floor: reversing from rest > 0.7 s
const int STALL_PWM_MIN = 12;
const unsigned long FAULT_COOLDOWN_MS = 10000;
// Bench mode must be refreshed like VL/VR: a stalled host left a wheel running
// for a whole minute on the floor (2026-09-30). Repeat the PINS command.
const unsigned long PIN_TEST_MS = 1500;

struct Wheel {
  int avi, fr, enbl, brk;
  bool inverted;
  bool brk_release;
  int side;                          // 0 = left, 1 = right
  volatile long count;
  volatile unsigned long last_us;    // time of the last counted pulse
  volatile unsigned long period_us;  // mean interval between edges
  volatile unsigned long last_gap_us;
  float effort;
  float integ;
  float target_pps;
  bool dir_fwd;
  bool driving;                      // driven (not stopped / coasting)
  bool pulsed_since_start;
  unsigned long start_ms;            // when this wheel last started driving
  unsigned long coast_since_ms;      // effort reached 0 while waiting to reverse
  unsigned long rekey_until_ms;      // driver off after a direction change until
  unsigned long kick_until_ms;       // stuck-wheel kick running until
  long stall_count;
  unsigned long stall_since_ms;
};

Wheel L = {L_AVI, L_FR, L_ENBL, L_BRK, LEFT_INVERTED, L_BRK_RELEASE, 0, 0, 0, 0, 0,
           0, 0, 0, true, false, false, 0, 0, 0, 0, 0, 0};
Wheel R = {R_AVI, R_FR, R_ENBL, R_BRK, RIGHT_INVERTED, R_BRK_RELEASE, 1, 0, 0, 0, 0,
           0, 0, 0, true, false, false, 0, 0, 0, 0, 0, 0};

float m_per_pulse[2] = {DEFAULT_M_PER_PULSE, DEFAULT_M_PER_PULSE};
float vl_target = 0.0;
float vr_target = 0.0;
bool command_seen = false;
bool motors_enabled = false;
unsigned long last_cmd_ms = 0;
unsigned long last_control_ms = 0;
unsigned long fully_stopped_since_ms = 0;
unsigned long fault_until_ms = 0;
String cmdBuffer = "";
bool sync_on = false;
float sync_ratio = 0.0;
long sync_l0 = 0, sync_r0 = 0;

// Bench pin test, per side [0]=left [1]=right; -1 = not overridden.
int pin_test_brk[2] = {-1, -1};
int pin_test_enbl[2] = {-1, -1};
int pin_test_fr[2] = {-1, -1};
int pin_test_avi[2] = {-1, -1};
int pin_test_raw[2] = {-1, -1};
bool pin_test_on = false;
unsigned long pin_test_until_ms = 0;

// ================== PULSES ==================
void countPulse(Wheel& w) {
  unsigned long now = micros();
  unsigned long gap = now - w.last_us;
  if (w.last_us != 0 && gap < PULSE_DEBOUNCE_US) return;   // electrical noise
  // Speed: the mean of the last two gaps (smooths pulse-to-pulse jitter).
  if (w.last_us != 0) {
    w.period_us = w.last_gap_us ? (gap + w.last_gap_us) / 2 : gap;
    w.last_gap_us = gap;
  }
  w.last_us = now;
  w.count += w.dir_fwd ? 1 : -1;
}
void encLeftIsr()  { countPulse(L); }
void encRightIsr() { countPulse(R); }

long readCount(Wheel& w) {
  noInterrupts();
  long c = w.count;
  interrupts();
  return c;
}

// Measured wheel speed in pulses/s (no direction). Between pulses the speed
// can only be lower than 1/(time since the last pulse), so a slowing wheel
// reads lower at once instead of holding its last value.
float measuredPps(Wheel& w) {
  noInterrupts();
  unsigned long last = w.last_us;
  unsigned long period = w.period_us;
  interrupts();
  if (last == 0 || period == 0) return 0.0;
  unsigned long since = micros() - last;
  if (since > SPEED_TIMEOUT_US) return 0.0;
  unsigned long p = since > period ? since : period;
  return 1e6 / (float)p;
}

// ================== OUTPUTS ==================
int aviByte(float effort) {
  return constrain((int)(effort + 0.5), 0, 255);
}

void writeAvi(int pin, float effort) {
  analogWrite(pin, aviByte(effort));
}

void writeFR(int pin, bool level) {
  if (FR_OPEN_DRAIN && level == HIGH) {
    pinMode(pin, INPUT);
  } else {
    digitalWrite(pin, level);
    pinMode(pin, OUTPUT);
  }
}

void setDir(Wheel& w, bool fwd) {
  w.dir_fwd = fwd;
  writeFR(w.fr, fwd == w.inverted ? LOW : HIGH);
}

void releaseBrake() {
  // Never leave the hubs locked.
  digitalWrite(L_BRK, L_BRK_RELEASE);
  digitalWrite(R_BRK, R_BRK_RELEASE);
}

void resetControl(Wheel& w) {
  w.effort = 0.0;
  w.integ = 0.0;
  w.target_pps = 0.0;
  w.driving = false;
  w.rekey_until_ms = 0;
  w.kick_until_ms = 0;
}

// Speed inputs to 0, drivers disabled, brakes released (coast).
void hardStop() {
  writeAvi(L_AVI, 0);
  writeAvi(R_AVI, 0);
  releaseBrake();
  digitalWrite(L_ENBL, ENBL_STOP);
  digitalWrite(R_ENBL, ENBL_STOP);
  motors_enabled = false;
  resetControl(L);
  resetControl(R);
}

void enterFault(const char* msg) {
  hardStop();
  fault_until_ms = millis() + FAULT_COOLDOWN_MS;
  Serial.println(msg);
}

void motorEnable() {
  releaseBrake();
  if (!motors_enabled) {
    // A stop->run edge with the speed input at 0 clears the drivers' latch.
    digitalWrite(L_ENBL, ENBL_STOP);
    digitalWrite(R_ENBL, ENBL_STOP);
    delay(40);
    writeAvi(L_AVI, 0);
    writeAvi(R_AVI, 0);
    delay(10);
    digitalWrite(L_ENBL, ENBL_RUN);
    delayMicroseconds(5000);
    digitalWrite(R_ENBL, ENBL_RUN);
    motors_enabled = true;
  }
}

// ================== BENCH PIN TEST ==================
void clearPinTest() {
  for (int i = 0; i < 2; i++) {
    pin_test_brk[i] = pin_test_enbl[i] = pin_test_fr[i] = pin_test_avi[i] = -1;
    pin_test_raw[i] = -1;
  }
  pin_test_on = false;
}

bool pinTestActive() {
  if (!pin_test_on) return false;
  if (millis() >= pin_test_until_ms) {
    clearPinTest();
    Serial.println("PINS OFF (timeout)");
    return false;
  }
  return true;
}

void applyPinTest() {
  Wheel* ws[2] = {&L, &R};
  for (int i = 0; i < 2; i++) {
    Wheel& w = *ws[i];
    if (pin_test_brk[i] >= 0) digitalWrite(w.brk, pin_test_brk[i] ? HIGH : LOW);
    if (pin_test_enbl[i] >= 0) digitalWrite(w.enbl, pin_test_enbl[i] ? HIGH : LOW);
    if (pin_test_fr[i] >= 0) writeFR(w.fr, pin_test_fr[i] ? HIGH : LOW);
    int effort = pin_test_avi[i] > 0 ? min(pin_test_avi[i], PWM_MAX) : 0;
    if (pin_test_raw[i] >= 0) {
      analogWrite(w.avi, pin_test_raw[i]);
      w.effort = pin_test_raw[i];
    } else {
      writeAvi(w.avi, effort);
      w.effort = effort;
    }
  }
}

// Value after " <key>" in cmd, or -1.
int pinArg(const String& cmd, const char* key) {
  int k = cmd.indexOf(key);
  if (k < 0) return -1;
  int v = 0;
  bool any = false;
  for (unsigned int n = k + 2; n < cmd.length(); n++) {
    char c = cmd[n];
    if (c < '0' || c > '9') break;
    v = v * 10 + (c - '0');
    any = true;
  }
  return any ? v : -1;
}

// ================== DRIVE ==================
float slewToward(float current, float target, float dt) {
  float step = EFFORT_SLEW * dt;
  if (target > current) return min(current + step, target);
  return max(current - step, target);
}

// One control step for one wheel. vel: signed m/s (0 = stop this wheel).
void controlWheel(Wheel& w, float vel, float dt, unsigned long now_ms) {
  bool stop = fabs(vel) < VEL_DEADBAND;
  bool fwd = vel >= 0.0;
  // Coast down before changing direction: F/R never flips under effort.
  bool must_coast = false;
  if (!stop && fwd != w.dir_fwd) {
    if (w.effort > 1.0) {
      w.coast_since_ms = 0;
      must_coast = true;
    } else {
      if (w.coast_since_ms == 0) w.coast_since_ms = now_ms;
      must_coast = measuredPps(w) > REVERSE_FLIP_PPS &&
                   now_ms - w.coast_since_ms < REVERSE_COAST_MS;
    }
  }
  if (stop || must_coast) {
    w.integ = 0.0;
    w.target_pps = 0.0;
    w.driving = false;
    w.effort = slewToward(w.effort, 0.0, dt);
    return;
  }
  w.coast_since_ms = 0;
  if (fwd != w.dir_fwd) {
    // Re-key: driver off, new direction, back on after REKEY_MS.
    digitalWrite(w.enbl, ENBL_STOP);
    setDir(w, fwd);
    w.rekey_until_ms = now_ms + REKEY_MS;
  }
  if (w.rekey_until_ms != 0) {
    w.effort = 0.0;
    w.integ = 0.0;
    w.target_pps = 0.0;
    w.driving = false;
    if (now_ms < w.rekey_until_ms) return;
    w.rekey_until_ms = 0;
    digitalWrite(w.enbl, ENBL_RUN);
  }
  if (!w.driving) {
    // A fresh start for this wheel: stall grace and breakaway from now.
    w.driving = true;
    w.start_ms = now_ms;
    w.stall_count = readCount(w);
    w.stall_since_ms = now_ms;
    w.pulsed_since_start = false;
    w.kick_until_ms = 0;
  }

  w.target_pps = fabs(vel) / m_per_pulse[w.side];
  float meas = measuredPps(w);
  float err = w.target_pps - meas;
  float ff = FF_ZERO_EFFORT + w.target_pps / FF_PPS_PER_EFFORT;
  float integ = constrain(w.integ + KI * err * dt, -INTEG_MAX, INTEG_MAX);
  float u = ff + KP * err + integ;
  // Anti-windup: don't integrate further into a limit.
  if ((u > PWM_MAX && err > 0) || (u < PWM_START && err < 0)) {
    integ = w.integ;
    u = ff + KP * err + integ;
  }
  w.integ = integ;
  if (!w.pulsed_since_start && now_ms - w.start_ms < BREAKAWAY_MS) {
    u = max(u, (float)PWM_BREAKAWAY);   // get a loaded wheel rolling
  }
  // Stuck-wheel kick (after the start-up breakaway).
  unsigned long gap = (unsigned long)max(300.0, 1500.0 / max(w.target_pps, 0.1f));
  if (w.pulsed_since_start && w.kick_until_ms == 0 && now_ms - w.stall_since_ms > gap) {
    w.kick_until_ms = now_ms + KICK_MS;
  }
  if (w.kick_until_ms != 0) {
    if (now_ms < w.kick_until_ms && now_ms - w.stall_since_ms > gap) {
      u = max(u, (float)PWM_BREAKAWAY);
    } else if (now_ms - w.stall_since_ms <= gap) {
      w.kick_until_ms = 0;              // it moved: ready for the next kick
    }
  }
  u = constrain(u, (float)PWM_START, (float)PWM_MAX);
  w.effort = slewToward(w.effort, u, dt);
}

// Stall check for one wheel: true = driven with no pulse for STALL_CUT_MS.
bool wheelFrozen(Wheel& w, unsigned long now_ms) {
  long c = readCount(w);
  if (c != w.stall_count) {
    w.stall_count = c;
    w.stall_since_ms = now_ms;
    if (w.driving) w.pulsed_since_start = true;
    return false;
  }
  // Slow wheels pulse rarely (an inner wheel in a turn: ~1.6 pulses/s), so
  // "frozen" = no pulse for 3 expected gaps, never less than STALL_CUT_MS.
  unsigned long cut = STALL_CUT_MS;
  if (w.target_pps > 0.1) cut = max(cut, (unsigned long)(3000.0 / w.target_pps));
  cut = max(cut, (unsigned long)(300 + KICK_MS + 300));   // the kick gets its chance
  return w.driving && w.effort >= STALL_PWM_MIN &&
         now_ms - w.start_ms >= STALL_GRACE_MS &&
         now_ms - w.stall_since_ms >= cut;
}

// Adjust the two wheel speeds so their real distances keep the commanded
// ratio (see SYNC_GAIN). Restarts whenever the ratio or direction changes.
void synchronise(float& leftVel, float& rightVel) {
  bool same_way = fabs(leftVel) >= VEL_DEADBAND && fabs(rightVel) >= VEL_DEADBAND &&
                  leftVel * rightVel > 0.0 && L.dir_fwd == (leftVel > 0.0) &&
                  R.dir_fwd == (rightVel > 0.0);
  if (!same_way) {
    sync_on = false;
    return;
  }
  float ratio = rightVel / leftVel;
  if (!sync_on || fabs(ratio - sync_ratio) > 0.05 * sync_ratio + 0.02) {
    sync_on = true;
    sync_ratio = ratio;
    sync_l0 = readCount(L);
    sync_r0 = readCount(R);
    return;
  }
  float dl = (readCount(L) - sync_l0) * m_per_pulse[0];   // signed metres
  float dr = (readCount(R) - sync_r0) * m_per_pulse[1];
  float corr = constrain(SYNC_GAIN * (dl * ratio - dr), -SYNC_MAX, SYNC_MAX);
  // corr > 0: the right wheel is behind (in either direction of travel).
  float nl = leftVel - corr / 2.0, nr = rightVel + corr / 2.0;
  // Never stop or reverse a wheel by synchronising.
  if (nl * leftVel > 0.0 && fabs(nl) >= VEL_DEADBAND) leftVel = nl;
  if (nr * rightVel > 0.0 && fabs(nr) >= VEL_DEADBAND) rightVel = nr;
}

void driveMotors(float leftVel, float rightVel) {
  unsigned long now_ms = millis();
  if (fault_until_ms != 0) {
    if (now_ms < fault_until_ms) {
      hardStop();
      return;
    }
    fault_until_ms = 0;   // hardStop() cleared 'driving': the retry is a fresh start
  }
  if (now_ms - last_control_ms < CONTROL_MS) return;
  float dt = last_control_ms == 0 ? CONTROL_MS / 1000.0
                                  : (now_ms - last_control_ms) / 1000.0;
  if (dt > 0.2) dt = CONTROL_MS / 1000.0;
  last_control_ms = now_ms;

  leftVel = constrain(leftVel, -MAX_WHEEL_MPS, MAX_WHEEL_MPS);
  rightVel = constrain(rightVel, -MAX_WHEEL_MPS, MAX_WHEEL_MPS);
  // Never drive the wheels in opposite directions (the hubs fight, fuses blow):
  // both take the stronger command's direction and size.
  if (fabs(leftVel) >= VEL_DEADBAND && fabs(rightVel) >= VEL_DEADBAND &&
      leftVel * rightVel < 0.0) {
    float v = fabs(rightVel) >= fabs(leftVel) ? rightVel : leftVel;
    leftVel = rightVel = v;
  }
  // Never pivot on one stopped hub: a one-wheel command becomes an arc.
  if (fabs(leftVel) >= VEL_DEADBAND && fabs(rightVel) < VEL_DEADBAND) {
    rightVel = leftVel * 0.45;
  } else if (fabs(rightVel) >= VEL_DEADBAND && fabs(leftVel) < VEL_DEADBAND) {
    leftVel = rightVel * 0.45;
  }
  bool moving = fabs(leftVel) >= VEL_DEADBAND || fabs(rightVel) >= VEL_DEADBAND;

  if (!moving) {
    sync_on = false;
    controlWheel(L, 0.0, dt, now_ms);
    controlWheel(R, 0.0, dt, now_ms);
    writeAvi(L_AVI, L.effort);
    writeAvi(R_AVI, R.effort);
    if (L.effort < 1.0 && R.effort < 1.0) {
      if (fully_stopped_since_ms == 0) fully_stopped_since_ms = now_ms;
      if (motors_enabled && now_ms - fully_stopped_since_ms >= IDLE_ENBL_OFF_MS) {
        hardStop();
      }
    }
    return;
  }
  fully_stopped_since_ms = 0;

  // Set F/R before the ENBL edge so a driver never sees it change enabled.
  if (!motors_enabled) {
    if (fabs(leftVel) >= VEL_DEADBAND) setDir(L, leftVel >= 0.0);
    if (fabs(rightVel) >= VEL_DEADBAND) setDir(R, rightVel >= 0.0);
    delayMicroseconds(2000);
  }
  motorEnable();
  synchronise(leftVel, rightVel);
  controlWheel(L, leftVel, dt, now_ms);
  controlWheel(R, rightVel, dt, now_ms);

  bool l_frozen = wheelFrozen(L, now_ms);
  bool r_frozen = wheelFrozen(R, now_ms);
  if (l_frozen || r_frozen) {
    enterFault("STALL HARD STOP");
    return;
  }
  writeAvi(L_AVI, L.effort);
  writeAvi(R_AVI, R.effort);
}

// ================== REPORTS ==================
void printEncoders() {
  Serial.print("L:");
  Serial.print(readCount(L));
  Serial.print(" R:");
  Serial.println(readCount(R));
}

void printStatus() {
  Serial.print("S: pwmL=");
  Serial.print((int)L.effort);
  Serial.print(" pwmR=");
  Serial.print((int)R.effort);
  Serial.print(" enblL=");
  Serial.print(digitalRead(L_ENBL) == ENBL_RUN ? 1 : 0);
  Serial.print(" enblR=");
  Serial.print(digitalRead(R_ENBL) == ENBL_RUN ? 1 : 0);
  Serial.print(" brkL=");
  Serial.print(digitalRead(L_BRK));
  Serial.print(" brkR=");
  Serial.print(digitalRead(R_BRK));
  Serial.print(" brkOff=");
  Serial.print((digitalRead(L_BRK) == L_BRK_RELEASE &&
                digitalRead(R_BRK) == R_BRK_RELEASE) ? 1 : 0);
  Serial.print(" L=");
  Serial.print(readCount(L));
  Serial.print(" R=");
  Serial.print(readCount(R));
  Serial.print(" spL=");
  Serial.print(measuredPps(L), 1);
  Serial.print(" spR=");
  Serial.print(measuredPps(R), 1);
  Serial.print(" tgL=");
  Serial.print(L.target_pps, 1);
  Serial.print(" tgR=");
  Serial.println(R.target_pps, 1);
}

// ================== COMMANDS ==================
void parseCommand(const String& cmd) {
  if (cmd.indexOf("PINS") == 0) {
    if (cmd.indexOf("OFF") >= 0) {
      clearPinTest();
      hardStop();
      Serial.println("PINS OFF");
      return;
    }
    if (!pin_test_on) hardStop();
    bool left = cmd.indexOf("PINS R") != 0;
    bool right = cmd.indexOf("PINS L") != 0;
    for (int i = 0; i < 2; i++) {
      if ((i == 0 && !left) || (i == 1 && !right)) continue;
      int b = pinArg(cmd, " B");
      int e = pinArg(cmd, " E");
      int f = pinArg(cmd, " F");
      int a = pinArg(cmd, " A");
      int raw = pinArg(cmd, " W");
      pin_test_brk[i] = b < 0 ? -1 : (b ? 1 : 0);
      pin_test_enbl[i] = e < 0 ? -1 : (e ? 1 : 0);
      pin_test_fr[i] = f < 0 ? -1 : (f ? 1 : 0);
      pin_test_avi[i] = a < 0 ? 0 : min(a, PWM_MAX);
      pin_test_raw[i] = raw < 0 ? -1 : min(raw, 255);
    }
    pin_test_on = true;
    pin_test_until_ms = millis() + PIN_TEST_MS;
    Serial.println("PINS ON");
    return;
  }

  if (cmd.indexOf("K:") == 0) {
    int sp = cmd.indexOf(" ");
    float kl = (sp < 0 ? cmd.substring(2) : cmd.substring(2, sp)).toFloat();
    float kr = sp < 0 ? kl : cmd.substring(sp + 1).toFloat();
    if (kl > 0.0005 && kl < 0.05 && kr > 0.0005 && kr < 0.05) {
      m_per_pulse[0] = kl;
      m_per_pulse[1] = kr;
      Serial.print("K OK ");
      Serial.print(kl, 6);
      Serial.print(" ");
      Serial.println(kr, 6);
    } else {
      Serial.println("K REJECTED");
    }
    return;
  }

  int vlIdx = cmd.indexOf("VL:");
  int vrIdx = cmd.indexOf("VR:");
  if (vlIdx >= 0 && vrIdx > vlIdx) {
    vl_target = cmd.substring(vlIdx + 3, vrIdx).toFloat();
    vr_target = cmd.substring(vrIdx + 3).toFloat();
    last_cmd_ms = millis();
    command_seen = true;
    if (pin_test_on) {
      clearPinTest();   // driving always uses the normal pin levels
      hardStop();
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(L_BRK, OUTPUT);   // brakes first: unlock the hubs before anything
  pinMode(R_BRK, OUTPUT);
  releaseBrake();
  pinMode(L_AVI, OUTPUT);
  pinMode(R_AVI, OUTPUT);
  pinMode(L_FR, OUTPUT);
  pinMode(R_FR, OUTPUT);
  pinMode(L_ENBL, OUTPUT);
  pinMode(R_ENBL, OUTPUT);
  setDir(L, true);
  setDir(R, true);
  hardStop();

  pinMode(ENC_LEFT, INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);
  // RISING only: counting both edges doubled the resolution but the RIGHT
  // signal's falling edges carry false extra edges (its counts per metre
  // changed from run to run; floor 2026-09-30).
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), encLeftIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), encRightIsr, RISING);

  last_cmd_ms = millis();
  Serial.println("SAFE START READY");
  printStatus();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (cmdBuffer.length() > 0) parseCommand(cmdBuffer);
      cmdBuffer = "";
    } else if (c != '\r') {
      cmdBuffer += c;
      if (cmdBuffer.length() > 64) cmdBuffer = "";
    }
  }

  bool host_alive = command_seen && millis() - last_cmd_ms <= CMD_TIMEOUT_MS;
  if (!host_alive) {
    command_seen = false;
    vl_target = vr_target = 0.0;
  }
  if (pinTestActive()) {
    applyPinTest();
  } else if (host_alive) {
    driveMotors(vl_target, vr_target);
  } else if (motors_enabled || L.effort > 0.0 || R.effort > 0.0) {
    hardStop();   // no host = motors dead, at once
  }

  static unsigned long last_report = 0;
  if (millis() - last_report >= REPORT_MS) {
    last_report = millis();
    printEncoders();
    if (pin_test_on || motors_enabled) printStatus();
  }
}
