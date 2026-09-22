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

// ================== MOTOR ORIENTATION ===========
// Left forward was dead on straight (both FWD) while reverse worked in turns.
// Flip left so "forward" uses the F/R state that actually spins.
#define LEFT_INVERTED   true
#define RIGHT_INVERTED  true

// ================== SPEED PARAMETERS ============
// Max wheel velocity (m/s) - absolute top speed this robot will ever run
const float MAX_VEL = 0.12;

// ── Per-motor PWM ranges ──────────────────────────────────────────────────
// Keep moderate: both-forward at 200 often browns out one driver.
const int L_MIN_PWM = 100;
const int L_MAX_PWM = 170;

const int R_MIN_PWM = 100;
const int R_MAX_PWM = 170;

// Extra ceiling cut when both wheels drive the same way (straight).
const int STRAIGHT_MAX_PWM = 140;

// Per-motor speed trim — reduces one motor to match the other
// Start at 1.0 for both. Lower the faster motor's trim until speeds match.
const float L_TRIM = 1.0;
const float R_TRIM = 1.0;

// PWM ramp step per loop iteration (lower = smoother, less jerky)
const int RAMP_STEP = 3;

// Velocity deadband — per-wheel commands below this are treated as zero
const float VEL_DEADBAND = 0.01;

// ================== STATE =======================
float vl_target = 0.0;
float vr_target = 0.0;

int pwm_left_actual  = 0;   // currently applied PWM (for ramp)
int pwm_right_actual = 0;

// Track last commanded direction for each wheel (true = forward)
// Used to detect reversal and coast to zero before switching
bool left_dir_fwd  = true;
bool right_dir_fwd = true;

// ================== DEADMAN TIMER ===============
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT = 600;  // ms — tolerant of occasional slow serial frames
bool command_seen = false;

// Encoder report period. Kept low: USB traffic makes the EMI-induced
// USB dropouts far more frequent.
const unsigned long ENC_REPORT_MS = 250;

// Non-blocking serial accumulation buffer
String cmdBuffer = "";

// ================== LOW LEVEL ===================

// Fault-clear edge flag for motorEnable()
bool motors_enabled = false;

// Hard stop — ENBL polarity is inverted on this robot (Sep 24):
// RViz "drive" used ENBL LOW and wheels LOCKED; parked used ENBL HIGH and
// wheels CRAWLED opposite. BRK changes did not affect that, so ENBL is flipped.
void hardStop() {
  analogWrite(L_AVI, 0);
  analogWrite(R_AVI, 0);

  digitalWrite(L_ENBL, LOW);
  digitalWrite(R_ENBL, LOW);

  digitalWrite(L_BRK, LOW);
  digitalWrite(R_BRK, LOW);

  motors_enabled = false;

  pwm_left_actual  = 0;
  pwm_right_actual = 0;
  // Do NOT reset left_dir_fwd / right_dir_fwd here;
  // direction state should persist so next command ramps smoothly.
}

// Enable both motors (ENBL HIGH = run on this wiring).
// Fault-clear pulse only on STOP→RUN edge (left driver often latches).
void motorEnable() {
  digitalWrite(L_BRK, LOW);
  digitalWrite(R_BRK, LOW);

  if (!motors_enabled) {
    digitalWrite(L_ENBL, LOW);
    digitalWrite(R_ENBL, LOW);
    delayMicroseconds(5000);  // longer latch-clear pulse
    motors_enabled = true;
  }
  digitalWrite(L_ENBL, HIGH);
  digitalWrite(R_ENBL, HIGH);
}

// ================== ENCODERS (DEBOUNCED) ========
// Motor switching noise produces short spikes on the encoder lines, often
// several per millisecond. An integrator sampled every ENC_SAMPLE_US rides
// through those spikes: the level only flips after it has been mostly HIGH
// (or mostly LOW) for ENC_INTEG_MAX samples.
const unsigned long ENC_SAMPLE_US = 50;
const uint8_t ENC_INTEG_MAX = 10;

bool left_stable = false, right_stable = false;
uint8_t left_integ = 0, right_integ = 0;
unsigned long last_enc_sample = 0;

// Returns true on a filtered LOW -> HIGH transition.
bool pollEncoder(uint8_t pin, bool* stable, uint8_t* integ) {
  if (digitalRead(pin)) {
    if (*integ < ENC_INTEG_MAX) (*integ)++;
  } else {
    if (*integ > 0) (*integ)--;
  }
  if (!*stable && *integ == ENC_INTEG_MAX) {
    *stable = true;
    return true;
  }
  if (*stable && *integ == 0) {
    *stable = false;
  }
  return false;
}

void pollEncoders() {
  unsigned long now = micros();
  if (now - last_enc_sample < ENC_SAMPLE_US) return;
  last_enc_sample = now;
  if (pollEncoder(ENC_LEFT, &left_stable, &left_integ)) left_count++;
  if (pollEncoder(ENC_RIGHT, &right_stable, &right_integ)) right_count++;
}

// Maps velocity to PWM in range [minPWM..maxPWM] per motor
// Each motor has its own ceiling so hardware differences are compensated.
int velToPWM(float vel, int minPWM, int maxPWM) {
  if (abs(vel) < VEL_DEADBAND) return 0;
  float ratio = constrain(abs(vel) / MAX_VEL, 0.0, 1.0);
  return constrain((int)(minPWM + ratio * (maxPWM - minPWM)), minPWM, maxPWM);
}

// Set motor direction pin before applying PWM
void setLeftDir(float vel) {
  if (vel >= 0)
    digitalWrite(L_FR, LEFT_INVERTED ? LOW : HIGH);
  else
    digitalWrite(L_FR, LEFT_INVERTED ? HIGH : LOW);
}

void setRightDir(float vel) {
  if (vel >= 0)
    digitalWrite(R_FR, RIGHT_INVERTED ? LOW : HIGH);
  else
    digitalWrite(R_FR, RIGHT_INVERTED ? HIGH : LOW);
}

// Smooth ramp: move actual PWM toward target by at most RAMP_STEP
int rampPWM(int current, int target) {
  if (current < target) return min(current + RAMP_STEP, target);
  if (current > target) return max(current - RAMP_STEP, target);
  return current;
}

void driveMotor(float leftVel, float rightVel) {

  // Per-wheel deadband check
  bool leftStopped  = (abs(leftVel)  < VEL_DEADBAND);
  bool rightStopped = (abs(rightVel) < VEL_DEADBAND);

  // Full hard stop only when BOTH wheels commanded zero
  if (leftStopped && rightStopped) {
    hardStop();
    return;
  }

  motorEnable();

  // Detect direction reversal per wheel
  bool left_wants_fwd  = (leftVel  >= 0);
  bool right_wants_fwd = (rightVel >= 0);

  // Straight = both moving, same sign, similar speed → lower PWM to avoid
  // one driver brownout (symptom: turns OK, straight only one wheel).
  bool going_straight = !leftStopped && !rightStopped
      && (left_wants_fwd == right_wants_fwd)
      && (abs(abs(leftVel) - abs(rightVel)) < 0.04);
  int l_max = going_straight ? min(L_MAX_PWM, STRAIGHT_MAX_PWM) : L_MAX_PWM;
  int r_max = going_straight ? min(R_MAX_PWM, STRAIGHT_MAX_PWM) : R_MAX_PWM;

  // --- LEFT WHEEL ---
  if (!leftStopped && (left_wants_fwd != left_dir_fwd) && pwm_left_actual > 0) {
    // Direction change requested but motor still spinning — coast to zero first
    pwm_left_actual = rampPWM(pwm_left_actual, 0);
    analogWrite(L_AVI, pwm_left_actual);
    // Don't switch dir pin yet; return and wait for next loop iteration
  } else {
    // Safe to set direction and ramp
    if (!leftStopped) {
      left_dir_fwd = left_wants_fwd;
      // Set direction FIRST
      if (left_dir_fwd)
        digitalWrite(L_FR, LEFT_INVERTED ? LOW : HIGH);
      else
        digitalWrite(L_FR, LEFT_INVERTED ? HIGH : LOW);
    }
    int leftTarget = leftStopped ? 0 : (int)(velToPWM(leftVel, L_MIN_PWM, l_max) * L_TRIM);
    pwm_left_actual = rampPWM(pwm_left_actual, leftTarget);
    analogWrite(L_AVI, pwm_left_actual);
  }

  // --- RIGHT WHEEL ---
  if (!rightStopped && (right_wants_fwd != right_dir_fwd) && pwm_right_actual > 0) {
    // Direction change requested but motor still spinning — coast to zero first
    pwm_right_actual = rampPWM(pwm_right_actual, 0);
    analogWrite(R_AVI, pwm_right_actual);
  } else {
    if (!rightStopped) {
      right_dir_fwd = right_wants_fwd;
      if (right_dir_fwd)
        digitalWrite(R_FR, RIGHT_INVERTED ? LOW : HIGH);
      else
        digitalWrite(R_FR, RIGHT_INVERTED ? HIGH : LOW);
    }
    int rightTarget = rightStopped ? 0 : (int)(velToPWM(rightVel, R_MIN_PWM, r_max) * R_TRIM);
    pwm_right_actual = rampPWM(pwm_right_actual, rightTarget);
    analogWrite(R_AVI, pwm_right_actual);
  }
}

// ================== PARSE COMMAND ===============
void parseCommand(const String& cmd) {
  int vlIdx = cmd.indexOf("VL:");
  int vrIdx = cmd.indexOf("VR:");

  if (vlIdx >= 0 && vrIdx >= 0) {
    vl_target = cmd.substring(vlIdx + 3, vrIdx).toFloat();
    vr_target = cmd.substring(vrIdx + 3).toFloat();
    last_cmd_time = millis();
    command_seen = true;
  }
}

// ================== SETUP =======================
void setup() {
  Serial.begin(115200);

  pinMode(L_AVI,  OUTPUT);
  pinMode(R_AVI,  OUTPUT);
  pinMode(L_FR,   OUTPUT);
  pinMode(R_FR,   OUTPUT);
  pinMode(L_ENBL, OUTPUT);
  pinMode(R_ENBL, OUTPUT);
  pinMode(L_BRK,  OUTPUT);
  pinMode(R_BRK,  OUTPUT);

  pinMode(ENC_LEFT,  INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);

  left_stable = digitalRead(ENC_LEFT);
  right_stable = digitalRead(ENC_RIGHT);
  left_integ = left_stable ? ENC_INTEG_MAX : 0;
  right_integ = right_stable ? ENC_INTEG_MAX : 0;

  // Always start in hard stop
  hardStop();
  last_cmd_time = millis();
  command_seen = false;

  Serial.println("SAFE START READY");
}

// ================== LOOP ========================
void loop() {
  pollEncoders();

  // 1. NON-BLOCKING SERIAL READ
  // Accumulate characters until '\n', then parse.
  // This NEVER blocks — so driveMotor() is always called every loop.
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (cmdBuffer.length() > 0) {
        parseCommand(cmdBuffer);
        cmdBuffer = "";
      }
    } else if (c != '\r') {   // ignore carriage return
      cmdBuffer += c;
      if (cmdBuffer.length() > 64) cmdBuffer = "";  // guard against buffer overflow
    }
  }

  // 2. DEADMAN TIMEOUT — zero velocity if no command received recently
  if (!command_seen || millis() - last_cmd_time > CMD_TIMEOUT) {
    vl_target = 0.0;
    vr_target = 0.0;
  }

  // 3. DRIVE MOTORS
  driveMotor(vl_target, vr_target);

  // 4. SEND ENCODER DATA every ENC_REPORT_MS
  static unsigned long last_enc_time = 0;
  if (millis() - last_enc_time > ENC_REPORT_MS) {
    last_enc_time = millis();
    Serial.print("L:");
    Serial.print(left_count);
    Serial.print(" R:");
    Serial.println(right_count);
  }
}
