// Fresh Mega firmware: BLDC-5015A control + wheel encoders.
// Protocol (unchanged for ROS base_controller):
//   Host  -> "VL:<mps> VR:<mps>\n"
//   Mega  -> "SAFE START READY"
//           "L:<count> R:<count>"
//           "S: pwmL=.. pwmR=.. enblL=.. enblR=.. brkOff=.. L=.. R=.."

// ================== LEFT MOTOR ==================
#define L_AVI   9
#define L_FR    30
#define L_ENBL  22
#define L_BRK   52

// ================== RIGHT MOTOR =================
#define R_AVI   10
#define R_FR    31
#define R_ENBL  23
#define R_BRK   53

// ================== ENCODERS ====================
#define ENC_LEFT   2
#define ENC_RIGHT  3

volatile long left_count  = 0;
volatile long right_count = 0;

void encLeftIsr()  { left_count++; }
void encRightIsr() { right_count++; }

// ================== MOTOR ORIENTATION ===========
#define LEFT_INVERTED   true
#define RIGHT_INVERTED  true

// ================== SPEED / PWM =================
// Visible creep on a ~30x40 ft floor without the old fuse-blowing band.
const float CRUISE_MPS = 0.16;
const float VEL_DEADBAND = 0.01;
const int PWM_START = 52;
const int PWM_CRUISE = 64;
const int PWM_MAX = 72;
const int PWM_STRAIGHT_MAX = 66;
const int PWM_BREAKAWAY = 70;
const unsigned long BREAKAWAY_MS = 800;

const float PWM_RAMP_UP_PER_SEC = 18.0;
const float PWM_RAMP_UP_BREAKAWAY_PER_SEC = 50.0;
const float PWM_RAMP_DOWN_PER_SEC = 40.0;
const unsigned long IDLE_ENBL_OFF_MS = 1500;
const unsigned long CMD_TIMEOUT_MS = 500;
const unsigned long REPORT_MS = 50;
const unsigned long STALL_CUT_MS = 3000;  // creep/optos tick slowly — 500ms was killing left
const unsigned long STALL_RETRY_MS = 800;  // re-breakaway sooner
const int STALL_PWM_MIN = 20;
// Stall-zeroing one side makes the robot pivot in place. Keep both sides
// driven; only use stall state to force a fresh breakaway kick.
const bool STALL_DETECT = true;
const bool STALL_ZERO_PWM = false;

// Opto polarity. Set INVERT_ENBL_BRK true if commons are on GND
// (Mega HIGH = short/run) instead of the Hetai common-+5V default.
const bool INVERT_ENBL_BRK = true;
const bool ENBL_RUN  = INVERT_ENBL_BRK ? HIGH : LOW;
const bool ENBL_STOP = INVERT_ENBL_BRK ? LOW  : HIGH;
const bool BRK_ON    = INVERT_ENBL_BRK ? HIGH : LOW;
const bool BRK_OFF   = INVERT_ENBL_BRK ? LOW  : HIGH;

float vl_target = 0.0;
float vr_target = 0.0;
float pwm_left_actual = 0.0;
float pwm_right_actual = 0.0;
bool left_dir_fwd = true;
bool right_dir_fwd = true;
bool motors_enabled = false;
bool command_seen = false;
unsigned long last_cmd_ms = 0;
unsigned long last_ramp_ms = 0;
unsigned long fully_stopped_since_ms = 0;
unsigned long motion_start_ms = 0;
unsigned long left_stall_since_ms = 0;
unsigned long right_stall_since_ms = 0;
long left_stall_count = 0;
long right_stall_count = 0;
bool left_stalled = false;
bool right_stalled = false;
String cmdBuffer = "";

void releaseBrake() {
  digitalWrite(L_BRK, BRK_OFF);
  digitalWrite(R_BRK, BRK_OFF);
}

void hardStop() {
  analogWrite(L_AVI, 0);
  analogWrite(R_AVI, 0);
  releaseBrake();
  digitalWrite(L_ENBL, ENBL_STOP);
  digitalWrite(R_ENBL, ENBL_STOP);
  motors_enabled = false;
  pwm_left_actual = 0.0;
  pwm_right_actual = 0.0;
}

void idleEnableOff() {
  digitalWrite(L_ENBL, ENBL_STOP);
  digitalWrite(R_ENBL, ENBL_STOP);
  motors_enabled = false;
}

void motorEnable() {
  releaseBrake();
  if (!motors_enabled) {
    // Open then short ENBL (stop→run edge) with left/right stagger.
    digitalWrite(L_ENBL, ENBL_STOP);
    digitalWrite(R_ENBL, ENBL_STOP);
    delayMicroseconds(5000);
    digitalWrite(L_ENBL, ENBL_RUN);
    delayMicroseconds(5000);
    digitalWrite(R_ENBL, ENBL_RUN);
    motors_enabled = true;
  } else {
    digitalWrite(L_ENBL, ENBL_RUN);
    digitalWrite(R_ENBL, ENBL_RUN);
  }
}

void setLeftDir(bool fwd) {
  if (fwd) digitalWrite(L_FR, LEFT_INVERTED ? LOW : HIGH);
  else     digitalWrite(L_FR, LEFT_INVERTED ? HIGH : LOW);
}

void setRightDir(bool fwd) {
  if (fwd) digitalWrite(R_FR, RIGHT_INVERTED ? LOW : HIGH);
  else     digitalWrite(R_FR, RIGHT_INVERTED ? HIGH : LOW);
}

float rampPWM(float current, float target, float dt, bool breakaway) {
  float up = breakaway ? PWM_RAMP_UP_BREAKAWAY_PER_SEC : PWM_RAMP_UP_PER_SEC;
  float rate = (fabs(target) < fabs(current)) ? PWM_RAMP_DOWN_PER_SEC : up;
  float step = rate * dt;
  if (current < target) return min(current + step, target);
  if (current > target) return max(current - step, target);
  return current;
}

int pwmForSpeed(float vel, int cap) {
  float cmd = fabs(vel);
  if (cmd < VEL_DEADBAND || cmd < 0.02) return 0;
  if (cmd > CRUISE_MPS) cmd = CRUISE_MPS;
  float pwm = PWM_START + ((cmd - 0.02) / (CRUISE_MPS - 0.02)) *
              (PWM_CRUISE - PWM_START);
  return constrain((int)pwm, 0, cap);
}

void printStatus() {
  noInterrupts();
  long l = left_count;
  long r = right_count;
  interrupts();
  Serial.print("S: pwmL=");
  Serial.print((int)pwm_left_actual);
  Serial.print(" pwmR=");
  Serial.print((int)pwm_right_actual);
  Serial.print(" enblL=");
  Serial.print(digitalRead(L_ENBL) == ENBL_RUN ? 1 : 0);
  Serial.print(" enblR=");
  Serial.print(digitalRead(R_ENBL) == ENBL_RUN ? 1 : 0);
  Serial.print(" brkOff=");
  Serial.print((digitalRead(L_BRK) == BRK_OFF &&
                digitalRead(R_BRK) == BRK_OFF) ? 1 : 0);
  Serial.print(" L=");
  Serial.print(l);
  Serial.print(" R=");
  Serial.println(r);
}

void printEncoders() {
  noInterrupts();
  long l = left_count;
  long r = right_count;
  interrupts();
  Serial.print("L:");
  Serial.print(l);
  Serial.print(" R:");
  Serial.println(r);
}

void driveMotor(float leftVel, float rightVel) {
  bool leftStopped  = fabs(leftVel)  < VEL_DEADBAND;
  bool rightStopped = fabs(rightVel) < VEL_DEADBAND;

  unsigned long now_ms = millis();
  float dt = (last_ramp_ms == 0) ? 0.02 : (now_ms - last_ramp_ms) / 1000.0;
  last_ramp_ms = now_ms;
  if (dt <= 0.0 || dt > 0.2) dt = 0.02;

  noInterrupts();
  long l_now = left_count;
  long r_now = right_count;
  interrupts();

  if (leftStopped && rightStopped) {
    pwm_left_actual = rampPWM(pwm_left_actual, 0, dt, false);
    pwm_right_actual = rampPWM(pwm_right_actual, 0, dt, false);
    analogWrite(L_AVI, (int)pwm_left_actual);
    analogWrite(R_AVI, (int)pwm_right_actual);
    left_stall_since_ms = 0;
    right_stall_since_ms = 0;
    left_stalled = false;
    right_stalled = false;
    motion_start_ms = 0;
    if (pwm_left_actual < 1.0 && pwm_right_actual < 1.0) {
      hardStop();
      if (fully_stopped_since_ms == 0) fully_stopped_since_ms = now_ms;
      else if (motors_enabled &&
               (now_ms - fully_stopped_since_ms) >= IDLE_ENBL_OFF_MS) {
        idleEnableOff();
      }
    }
    return;
  }

  fully_stopped_since_ms = 0;
  if (motion_start_ms == 0) motion_start_ms = now_ms;
  bool breakaway = (now_ms - motion_start_ms) < BREAKAWAY_MS;
  motorEnable();

  bool left_fwd = leftVel >= 0.0;
  bool right_fwd = rightVel >= 0.0;
  bool straight = !leftStopped && !rightStopped &&
                  (left_fwd == right_fwd) &&
                  (fabs(fabs(leftVel) - fabs(rightVel)) < 0.15);
  int l_cap = straight ? min(PWM_MAX, PWM_STRAIGHT_MAX) : PWM_MAX;
  int r_cap = straight ? min(PWM_MAX, PWM_STRAIGHT_MAX) : PWM_MAX;
  if (breakaway) {
    l_cap = max(l_cap, PWM_BREAKAWAY);
    r_cap = max(r_cap, PWM_BREAKAWAY);
  }

  // Coast through a direction reverse before flipping F/R.
  if (!leftStopped && (left_fwd != left_dir_fwd) && pwm_left_actual > 1.0) {
    pwm_left_actual = rampPWM(pwm_left_actual, 0, dt, false);
  } else if (!leftStopped) {
    left_dir_fwd = left_fwd;
    setLeftDir(left_dir_fwd);
    int target = pwmForSpeed(leftVel, l_cap);
    if (breakaway && !left_stalled) target = max(target, PWM_BREAKAWAY);
    pwm_left_actual = rampPWM(pwm_left_actual, target, dt, breakaway);
  } else {
    pwm_left_actual = rampPWM(pwm_left_actual, 0, dt, false);
  }

  if (!rightStopped && (right_fwd != right_dir_fwd) && pwm_right_actual > 1.0) {
    pwm_right_actual = rampPWM(pwm_right_actual, 0, dt, false);
  } else if (!rightStopped) {
    right_dir_fwd = right_fwd;
    setRightDir(right_dir_fwd);
    int target = pwmForSpeed(rightVel, r_cap);
    if (breakaway && !right_stalled) target = max(target, PWM_BREAKAWAY);
    pwm_right_actual = rampPWM(pwm_right_actual, target, dt, breakaway);
  } else {
    pwm_right_actual = rampPWM(pwm_right_actual, 0, dt, false);
  }

  // Stall latch with retry: detect no ticks, but do NOT zero one side
  // (that pivots forever on the other wheel). Re-breakaway instead.
  if (!STALL_DETECT) {
    // skip stall logic
  } else if (leftStopped) {
    left_stalled = false;
    left_stall_since_ms = 0;
    left_stall_count = l_now;
  } else if (left_stalled) {
    if (now_ms - left_stall_since_ms >= STALL_RETRY_MS) {
      left_stalled = false;
      left_stall_since_ms = now_ms;
      left_stall_count = l_now;
      motion_start_ms = now_ms;  // new breakaway window
    } else if (STALL_ZERO_PWM) {
      pwm_left_actual = 0;
    } else {
      pwm_left_actual = max(pwm_left_actual, (float)PWM_BREAKAWAY);
    }
  } else if (pwm_left_actual >= STALL_PWM_MIN) {
    if (l_now != left_stall_count) {
      left_stall_count = l_now;
      left_stall_since_ms = now_ms;
    } else if (left_stall_since_ms == 0) {
      left_stall_since_ms = now_ms;
    } else if (now_ms - left_stall_since_ms >= STALL_CUT_MS) {
      left_stalled = true;
      left_stall_since_ms = now_ms;
      if (STALL_ZERO_PWM) {
        pwm_left_actual = 0;
      } else {
        pwm_left_actual = max(pwm_left_actual, (float)PWM_BREAKAWAY);
        motion_start_ms = now_ms;
      }
    }
  }

  if (!STALL_DETECT) {
    // skip
  } else if (rightStopped) {
    right_stalled = false;
    right_stall_since_ms = 0;
    right_stall_count = r_now;
  } else if (right_stalled) {
    if (now_ms - right_stall_since_ms >= STALL_RETRY_MS) {
      right_stalled = false;
      right_stall_since_ms = now_ms;
      right_stall_count = r_now;
      motion_start_ms = now_ms;
    } else if (STALL_ZERO_PWM) {
      pwm_right_actual = 0;
    } else {
      pwm_right_actual = max(pwm_right_actual, (float)PWM_BREAKAWAY);
    }
  } else if (pwm_right_actual >= STALL_PWM_MIN) {
    if (r_now != right_stall_count) {
      right_stall_count = r_now;
      right_stall_since_ms = now_ms;
    } else if (right_stall_since_ms == 0) {
      right_stall_since_ms = now_ms;
    } else if (now_ms - right_stall_since_ms >= STALL_CUT_MS) {
      right_stalled = true;
      right_stall_since_ms = now_ms;
      if (STALL_ZERO_PWM) {
        pwm_right_actual = 0;
      } else {
        pwm_right_actual = max(pwm_right_actual, (float)PWM_BREAKAWAY);
        motion_start_ms = now_ms;
      }
    }
  }

  analogWrite(L_AVI, (int)pwm_left_actual);
  analogWrite(R_AVI, (int)pwm_right_actual);
}

void parseCommand(const String& cmd) {
  if (cmd == "TEST") {
    // Gentle bench kick only — never the old 160 duty that blows fuses.
    const int kick = PWM_CRUISE;
    Serial.println("TEST START");
    motorEnable();
    setLeftDir(true);
    setRightDir(true);
    analogWrite(L_AVI, kick);
    analogWrite(R_AVI, kick);
    pwm_left_actual = kick;
    pwm_right_actual = kick;
    unsigned long t0 = millis();
    while (millis() - t0 < 1500) {
      printEncoders();
      printStatus();
      delay(200);
    }
    hardStop();
    Serial.println("TEST DONE");
    printEncoders();
    printStatus();
    return;
  }

  int vlIdx = cmd.indexOf("VL:");
  int vrIdx = cmd.indexOf("VR:");
  if (vlIdx >= 0 && vrIdx >= 0) {
    vl_target = cmd.substring(vlIdx + 3, vrIdx).toFloat();
    vr_target = cmd.substring(vrIdx + 3).toFloat();
    last_cmd_ms = millis();
    command_seen = true;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(L_AVI, OUTPUT);
  pinMode(R_AVI, OUTPUT);
  pinMode(L_FR, OUTPUT);
  pinMode(R_FR, OUTPUT);
  pinMode(L_ENBL, OUTPUT);
  pinMode(R_ENBL, OUTPUT);
  pinMode(L_BRK, OUTPUT);
  pinMode(R_BRK, OUTPUT);

  pinMode(ENC_LEFT, INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), encLeftIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), encRightIsr, RISING);

  hardStop();
  last_cmd_ms = millis();
  command_seen = false;

  Serial.println("SAFE START READY");
  printStatus();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (cmdBuffer.length() > 0) {
        parseCommand(cmdBuffer);
        cmdBuffer = "";
      }
    } else if (c != '\r') {
      cmdBuffer += c;
      if (cmdBuffer.length() > 64) cmdBuffer = "";
    }
  }

  if (!command_seen || (millis() - last_cmd_ms) > CMD_TIMEOUT_MS) {
    vl_target = 0.0;
    vr_target = 0.0;
  }

  driveMotor(vl_target, vr_target);

  static unsigned long last_report = 0;
  if (millis() - last_report >= REPORT_MS) {
    last_report = millis();
    printEncoders();
    if (command_seen &&
        (fabs(vl_target) > VEL_DEADBAND || fabs(vr_target) > VEL_DEADBAND ||
         pwm_left_actual > 1.0 || pwm_right_actual > 1.0 || motors_enabled)) {
      printStatus();
    }
  }
}
