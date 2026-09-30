// Host-side tests for arduino/wheelodom.ino. Run: arduino/test/run_tests.sh
//
// Each wheel is simulated: its speed follows the effort with a lag, scaled by
// a load factor (1 = lifted wheel, lower = more drag), and it fires the speed
// pulse interrupt at that speed. The host is simulated by refreshing the
// VL/VR command every 50 ms.
#include "Arduino.h"
unsigned long g_now = 1000;
unsigned long g_now_us = 0;
int g_pins[64];
int g_mode[64];
std::string g_log;
SerialT Serial;
#include "fw_under_test.cpp"   // wheelodom.ino with setup/loop renamed

// The simulated wheel's pulse size (tests keep the finer 11.2 mm one).
const float TEST_K = 0.0112;

static int fails = 0;
static void check(const char* name, bool ok) {
  printf("%s %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) fails++;
}

// ---- wheel model ----
struct Plant {
  float load = 1.0;      // pulses/s per effort is 2.3 * load (lifted = 1)
  float pps = 0.0;       // actual speed
  float phase = 0.0;     // fraction of the next pulse
  bool jammed = false;
  unsigned long free_after = 0;  // ms: wheel can't turn before this (loaded start)
  float stiction = 0;   // effort needed to get a STOPPED wheel moving (floor friction)
  float max_effort = 0;
};
Plant PL, PR;

static void stepPlant(Plant& p, Wheel& w, void (*isr)()) {
  float effort = w.effort;
  if (effort > p.max_effort) p.max_effort = effort;
  bool enabled = motors_enabled;
  float want = enabled && !p.jammed && g_now >= p.free_after && effort > 5.0
                   ? 2.3f * p.load * (effort - 5.0f) : 0.0f;
  // Friction: a stopped wheel stays stopped below the breakaway effort, and
  // a slow wheel below ~9 effort stops (floor, 2026-09-30).
  if (p.pps < 0.5f && effort < p.stiction) want = 0.0f;
  if (p.stiction > 0 && effort < 9.0f) want = 0.0f;
  p.pps += (want - p.pps) * (1.0f / 150.0f);   // ~150 ms lag, 1 ms steps
  p.phase += p.pps / 1000.0f;
  if (p.phase >= 1.0f) {
    p.phase -= 1.0f;
    isr();
  }
}

static float vl_cmd = 0, vr_cmd = 0;
static bool host_on = true;

// Run ms milliseconds; the host refreshes VL/VR every 50 ms while host_on.
static bool run(int ms) {
  bool fault = false;
  for (int t = 0; t < ms; t++) {
    if (host_on && g_now % 50 == 0) {
      char buf[64];
      snprintf(buf, sizeof buf, "VL:%.3f VR:%.3f", vl_cmd, vr_cmd);
      parseCommand(String(buf));
    }
    stepPlant(PL, L, encLeftIsr);
    stepPlant(PR, R, encRightIsr);
    loop_unused();
    if (fault_until_ms) fault = true;
    g_now++;
  }
  return fault;
}

static void reset(float load_l = 1.0, float load_r = 1.0) {
  g_now += 20000;
  g_log.clear();
  PL = Plant();
  PR = Plant();
  PL.load = load_l;
  PR.load = load_r;
  vl_cmd = vr_cmd = 0;
  host_on = true;
  m_per_pulse[0] = m_per_pulse[1] = TEST_K;
  fault_until_ms = 0;
  hardStop();
  run(600);   // settle stopped
}

static bool near(float a, float b, float tol) { return fabs(a - b) <= tol * fabs(b); }
// Pulses/s a wheel makes at v m/s with the default pulse size.
static float pps(float v) { return v / TEST_K; }

int main() {
  setup_unused();

  // --- pin map (bench 2026-09-30: speed wires crossed, F/R yellow, BRK blue)
  check("left speed on D10, right speed on D9", L_AVI == 10 && R_AVI == 9);
  check("enable D22/D23, brake D30/D31, F/R D52/D53",
        L_ENBL == 22 && R_ENBL == 23 && L_BRK == 30 && R_BRK == 31 &&
        L_FR == 52 && R_FR == 53);

  // --- closed-loop speed
  reset();
  vl_cmd = vr_cmd = 0.06;
  bool f = run(4000);
  float tgt = pps(0.06);   // 24 pulses/s
  check("lifted: both wheels reach the commanded speed (+-10%)",
        !f && near(PL.pps, tgt, 0.10) && near(PR.pps, tgt, 0.10));
  check("measured speed tracks the real one (+-15%)",
        near(measuredPps(L), PL.pps, 0.15) && near(measuredPps(R), PR.pps, 0.15));
  check("forward: F/R D52 LOW, D53 HIGH", g_pins[52] == LOW && g_pins[53] == HIGH);
  check("driving: brakes LOW (released), enables HIGH",
        g_pins[30] == LOW && g_pins[31] == LOW && g_pins[22] == HIGH && g_pins[23] == HIGH);

  // A wheel with more drag gets more effort and still turns at the commanded
  // speed: the robot drives straight instead of curving towards it.
  reset(0.9, 0.55);
  vl_cmd = vr_cmd = 0.04;
  f = run(5000);
  tgt = pps(0.04);   // 16 pulses/s
  check("unequal drag: both wheels at the same commanded speed (+-10%)",
        !f && near(PL.pps, tgt, 0.10) && near(PR.pps, tgt, 0.10));
  check("unequal drag: the dragging wheel gets the higher effort", R.effort > L.effort + 1);

  // Wheel sync: the right wheel is so heavily loaded that even at the fuse
  // cap it can't reach the commanded speed. Without sync the left runs on
  // ahead and the robot curves; with it the left slows to match.
  reset(0.8, 0.08);
  {
    long l0 = readCount(L), r0 = readCount(R);
    vl_cmd = vr_cmd = 0.06;
    run(12000);
    float dl = (readCount(L) - l0) * TEST_K, dr = (readCount(R) - r0) * TEST_K;
    check("sync: a wheel at its power limit holds the other back (gap < 2 counts)",
          fabs(dl - dr) < 2 * TEST_K && dr > 0.3);
  }
  reset(0.8, 0.8);
  vl_cmd = 0.03;
  vr_cmd = 0.06;
  f = run(5000);
  check("arc: inner/outer wheel speed ratio is the commanded 0.5 (+-12%)",
        !f && near(PL.pps / PR.pps, 0.5, 0.12));

  // Saturation: a wheel that can't reach its speed within the fuse cap.
  reset(0.3, 0.3);
  vl_cmd = vr_cmd = 0.06;
  f = run(5000);
  check("heavy load: effort stops at the fuse cap (never above PWM_MAX)",
        !f && L.effort <= PWM_MAX && PL.max_effort <= PWM_MAX && PR.max_effort <= PWM_MAX);

  // --- K command
  reset();
  parseCommand(String("K:0.005"));
  vl_cmd = vr_cmd = 0.05;
  run(4000);
  check("K:0.005 -> 0.05 m/s = 10 pulses/s", near(PL.pps, 10.0, 0.12) && near(PR.pps, 10.0, 0.12));
  parseCommand(String("K:5"));
  check("K out of range is rejected", near(m_per_pulse[0], 0.005, 0.001));
  parseCommand(String("K:0.004 0.006"));
  check("K:<left> <right> sets each wheel", near(m_per_pulse[0], 0.004, 0.001) && near(m_per_pulse[1], 0.006, 0.001));

  // --- reverse and direction change
  reset();
  long l0 = readCount(L), r0 = readCount(R);
  vl_cmd = vr_cmd = -0.05;
  f = run(3000);
  check("reverse: F/R D52 HIGH, D53 LOW, wheels turning",
        !f && g_pins[52] == HIGH && g_pins[53] == LOW && PL.pps > 0.7 * pps(0.05));
  check("reverse: pulse counts go down (signed)", readCount(L) < l0 - 10 && readCount(R) < r0 - 10);
  vl_cmd = vr_cmd = 0.05;
  bool flipped_under_effort = false, flipped_enabled = false;
  for (int i = 0; i < 3000; i++) {
    int before = g_pins[52];
    float effort = L.effort;
    run(1);
    if (g_pins[52] != before && effort > 1.0) flipped_under_effort = true;
    if (g_pins[52] != before && g_pins[L_ENBL] == ENBL_RUN) flipped_enabled = true;
  }
  check("direction change: F/R never flips while the wheel has effort",
        !flipped_under_effort && g_pins[52] == LOW);
  check("direction change: the driver is switched off while F/R flips (re-key)",
        !flipped_enabled);
  run(3000);
  check("after the change the wheels run forward at speed", PL.pps > 0.7 * pps(0.05) && PR.pps > 0.7 * pps(0.05));

  // Floor-like load, reversing again and again (3-point turns): never a
  // false stall, F/R never flips under effort.
  reset(0.6, 0.5);
  bool rev_fault = false;
  for (int i = 0; i < 6; i++) {
    vl_cmd = vr_cmd = (i % 2) ? -0.04 : 0.05;
    rev_fault |= run(3500);   // coast <= 1.5 s + re-key 0.3 s + speed-up
  }
  check("6 reversals under load: no stall fault, wheels turning",
        !rev_fault && PL.pps > 0.5 * pps(0.04) && PR.pps > 0.5 * pps(0.04));

  // Lifted wheels spin down slowly: a pulse every ~0.4 s for a long time.
  // The reversal must still happen (F/R flips within ~1 s).
  reset();
  vl_cmd = vr_cmd = 0.05;
  run(3000);
  vl_cmd = vr_cmd = -0.05;
  int flip_ms = -1;
  for (int ms = 0; ms < 3000 && flip_ms < 0; ms++) {
    if (L.effort < 1.0 && g_now % 400 == 0) encLeftIsr();   // slow spin-down pulses
    run(1);
    if (g_pins[52] == HIGH) flip_ms = ms;
  }
  check("reversal with a slowly coasting wheel flips within 2 s", flip_ms >= 0 && flip_ms < 2000);

  reset();
  vl_cmd = 0.06;
  vr_cmd = -0.06;
  run(2000);
  check("opposite-direction command: both wheels driven the same way",
        g_pins[52] != g_pins[53]);   // mirrored hubs: same direction = opposite F/R

  reset();
  vl_cmd = 0.0;
  vr_cmd = 0.06;
  run(3000);
  check("one-wheel command becomes an arc (stopped wheel driven at 45%)",
        PL.pps > 0.3 * PR.pps && PL.pps < 0.6 * PR.pps);

  // --- stall protection
  reset();
  PR.jammed = true;
  vl_cmd = vr_cmd = 0.06;
  f = run(3000);
  check("one wheel jammed -> STALL HARD STOP", f && g_log.find("STALL HARD STOP") != std::string::npos);
  check("stall: both speed inputs 0, drivers disabled",
        g_pins[L_AVI] == 0 && g_pins[R_AVI] == 0 && g_pins[L_ENBL] == ENBL_STOP);
  run(5000);
  check("stall: wheels stay off during the cool-down", g_pins[L_AVI] == 0 && g_pins[R_AVI] == 0);

  reset();
  PL.jammed = PR.jammed = true;
  vl_cmd = vr_cmd = 0.06;
  check("both jammed -> STALL HARD STOP", run(3000));

  // Floor 2026-09-30: a slow inner wheel (~2.5 pulses/s, effort ~7) stopped
  // on floor friction and the stall cut fired. The kick must keep it going.
  reset(0.6, 0.6);
  PL.stiction = PR.stiction = 12.0;
  vl_cmd = 0.06;
  vr_cmd = 0.022;
  {
    long r0 = readCount(R);
    f = run(15000);
    float avg = (readCount(R) - r0) / 15.0;   // the wheel goes stop-and-go: average it
    check("slow inner wheel with floor friction: kicked free, no stall",
          !f && avg > 0.6 * pps(0.022) && PL.pps > 0.5 * pps(0.06));
  }
  // Slow inner wheel in a turn: ~1.6 pulses/s, one every 0.6 s.
  reset(0.5, 0.5);
  vl_cmd = 0.018;
  vr_cmd = 0.042;
  check("very slow inner wheel (~1.6 pulses/s) -> no false stall", !run(10000));
  reset(0.5, 0.5);
  vl_cmd = vr_cmd = 0.02;   // slowest real speed: ~3 pulses/s on a loaded wheel
  f = run(8000);
  check("slow loaded wheels -> no false stall", !f && PL.pps > 0.7 * pps(0.02));

  // Floor 2026-09-30: after the cool-down the retry used old timers and
  // re-faulted within 40 ms, forever. A retry is a fresh start.
  reset();
  PR.jammed = true;
  vl_cmd = vr_cmd = 0.06;
  run(3000);
  PR.jammed = false;
  // Loaded wheels: they need 0.6 s to start rolling after the restart.
  PL.free_after = PR.free_after = fault_until_ms + 600;
  g_log.clear();
  f = run(FAULT_COOLDOWN_MS + 4000);
  check("after the cool-down a slow-starting restart drives (no re-fault)",
        g_log.find("STALL HARD STOP") == std::string::npos && PL.pps > 0.7 * pps(0.06) && PR.pps > 0.7 * pps(0.06));

  // --- stopping
  reset();
  vl_cmd = vr_cmd = 0.06;
  run(3000);
  vl_cmd = vr_cmd = 0.0;
  run(400);
  check("stop: speed inputs reach 0 within 0.4 s", g_pins[L_AVI] == 0 && g_pins[R_AVI] == 0);
  run(400);
  check("stopped: drivers disabled", g_pins[L_ENBL] == ENBL_STOP && g_pins[R_ENBL] == ENBL_STOP);

  reset();
  vl_cmd = vr_cmd = 0.06;
  run(3000);
  host_on = false;
  run(CMD_TIMEOUT_MS + 30);
  check("host silent -> motors off within the 300 ms timeout",
        g_pins[L_AVI] == 0 && g_pins[R_AVI] == 0 && g_pins[L_ENBL] == ENBL_STOP);
  check("speed reading falls to 0 once the wheel stops", (run(1500), measuredPps(L) == 0.0));

  // --- pulse noise
  reset();
  long c0 = readCount(L);
  encLeftIsr();
  g_now_us = 500;   // 0.5 ms later: a glitch, not a wheel pulse
  encLeftIsr();
  g_now_us = 0;
  check("pulses closer than 2 ms are ignored (noise)", readCount(L) == c0 + 1);

  // --- bench PINS mode
  reset();
  parseCommand(String("PINS B1 E0"));
  loop_unused();
  check("PINS B1 E0: brakes HIGH, enables LOW, speed 0",
        g_pins[L_BRK] == 1 && g_pins[R_BRK] == 1 && g_pins[L_ENBL] == 0 &&
        g_pins[L_AVI] == 0 && g_pins[R_AVI] == 0);
  parseCommand(String("PINS L E1 A200"));
  loop_unused();
  check("PINS A capped at PWM_MAX", g_pins[L_AVI] == PWM_MAX);
  parseCommand(String("PINS R E1 W36"));
  loop_unused();
  check("PINS W writes the raw byte to the right speed pin", g_pins[R_AVI] == 36);
  parseCommand(String("VL:0.05 VR:0.05"));
  check("drive command clears PINS mode", !pin_test_on);
  parseCommand(String("PINS L E1 A15"));
  for (int i = 0; i < 1600; i++) { loop_unused(); g_now++; }
  check("PINS without a refresh switches itself off within 1.5 s (wheel stops)",
        !pin_test_on && g_pins[L_AVI] == 0);
  parseCommand(String("PINS B1"));
  parseCommand(String("PINS OFF"));
  loop_unused();
  check("PINS OFF clears and releases the brakes",
        !pin_test_on && g_pins[L_BRK] == L_BRK_RELEASE && g_pins[R_BRK] == R_BRK_RELEASE);

  printf("\n%d failure(s)\n", fails);
  return fails ? 1 : 0;
}
