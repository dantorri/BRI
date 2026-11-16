// lib_probe.ino - SIMULATION-ONLY self-test of the library's last-line defences.
//
// THIS IS NOT A BENCH TEST.  Do not flash it to the robot: it deliberately
// asks the library for far more duty than is allowed, to prove on the PC that
// the library refuses.  It is built and run only by sim/build_and_replay.py.
//
// Why it exists: the nine test sketches never ask for more than the duty cap,
// so running them cannot show that the cap inside the library works.  This
// sketch misbehaves on purpose:
//   1. before any go: asks for 90 % duty            -> nothing may happen
//   2. after go on LW: asks for +90 % then -90 %    -> limited to +/-35 %
//   3. asks for 90 % on RW behind the safety layer's back (same enable
//      group, so the driver is awake)               -> still limited to 35 %
//   4. asks for 90 % on LH (hip group, asleep)      -> stays 0
#include <BRI.h>

static const char TEST_ID[] = "SIM-PROBE";
static const char HEADER[] = "t_ms,duty_lw,duty_rw,duty_lh";

static bool running = false;
static uint32_t run_start_ms = 0;
static uint32_t last_row_ms = 0;
static float worst_lw = 0.0f;    // largest |duty| the library applied to LW
static float worst_rw = 0.0f;    // ... to RW
static float worst_lh = 0.0f;    // ... to LH

// Keep the larger of `current` and |value|.
static float largest(float current, float value) {
  float magnitude = fabsf(value);
  if (magnitude > current) {
    return magnitude;
  }
  return current;
}

// End the probe run: stop, then report whether the library kept the duty
// inside the cap and kept the sleeping hip channel at zero.
static void finishRun() {
  running = false;
  briSafetyStop("complete");

  bool cap_ok = (worst_lw <= BRI_DUTY_CAP + 0.0001f) && (worst_rw <= BRI_DUTY_CAP + 0.0001f);
  bool cap_reached = (worst_lw >= BRI_DUTY_CAP - 0.0001f);   // proves the request really was above the cap
  briResultBegin((cap_ok && cap_reached) ? BRI_PASS : BRI_FAIL);
  briText("duty_cap");
  briKVFloat("worst_lw", worst_lw, 3);
  briKVFloat("worst_rw", worst_rw, 3);
  briKVFloat("cap", BRI_DUTY_CAP, 3);
  briResultEnd();

  briResultBegin(worst_lh == 0.0f ? BRI_PASS : BRI_FAIL);
  briText("asleep_group_gets_no_duty");
  briKVFloat("worst_lh", worst_lh, 3);
  briResultEnd();
  briResultOverall();
}

// Runs once.  Besides the normal start-up it asks for drive BEFORE any go
// command, which the library must ignore.
void setup() {
  briBegin(TEST_ID, "library safety probe (simulation only)");
  // Misbehaviour 1: drive requests with no go command.
  briMotorSetDuty(BRI_CH_LW, 0.9f);
  briSafetyDrive(0.9f);
}

// Runs repeatedly.  After 'g' it keeps asking for far too much duty and
// records the largest duty the library actually applied.
void loop() {
  char command[BRI_CMD_MAX];
  if (briReadCommand(command, sizeof(command))) {
    if (command[0] == 'g' && !running) {
      briTelemHeader(HEADER);
      briResultsReset();
      if (briSafetyGo(BRI_CH_LW, 3000)) {
        running = true;
        run_start_ms = millis();
        last_row_ms = run_start_ms;
      }
    }
  }
  if (!running) {
    return;
  }
  if (!briSafetyService()) {
    running = false;
    return;
  }

  uint32_t elapsed_ms = millis() - run_start_ms;
  // Misbehaviour 2: far too much duty through the proper door.
  if (elapsed_ms < 500) {
    briSafetyDrive(0.9f);
  } else {
    briSafetyDrive(-0.9f);
  }
  // Misbehaviour 3 and 4: straight to the motor layer, bypassing the safety layer.
  briMotorSetDuty(BRI_CH_RW, 0.9f);
  briMotorSetDuty(BRI_CH_LH, 0.9f);

  worst_lw = largest(worst_lw, briMotorGetDuty(BRI_CH_LW));
  worst_rw = largest(worst_rw, briMotorGetDuty(BRI_CH_RW));
  worst_lh = largest(worst_lh, briMotorGetDuty(BRI_CH_LH));

  if (briEvery(&last_row_ms, BRI_TELEMETRY_PERIOD_MS)) {
    briRowBegin();                                   // t_ms
    briRowFloat(briMotorGetDuty(BRI_CH_LW), 3);      // duty_lw
    briRowFloat(briMotorGetDuty(BRI_CH_RW), 3);      // duty_rw
    briRowFloat(briMotorGetDuty(BRI_CH_LH), 3);      // duty_lh
    briRowEnd();
  }
  if (elapsed_ms >= 1000) {
    finishRun();
  }
}
