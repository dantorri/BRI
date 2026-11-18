// CV-005  IMU pitch orientation
//
// What this test shows
//   * the pitch angle computed three ways:
//       pitch_acc   from the accelerometer (which way gravity points)
//       pitch_gyro  from adding up the gyro's rotation rate over time
//       pitch_cf    a "complementary filter" blend of the two: gyro for quick
//                   changes, accelerometer to stop the slow drift
//   * that all three move the SAME way when you tilt the robot, which proves
//     the axis and sign constants in BRI_Config.h are right
//   * the pitch reading when the robot stands upright (the "zero")
//
// How to run it
//   1. hold the robot (or just the IMU, in its mounted orientation) upright
//      and STILL for the first 2 seconds - the firmware measures the gyro
//      offset and the zero angle
//   2. tilt it FORWARD about 20 to 30 degrees, back to upright, then BACKWARD
//      the same amount, slowly (a second or two each way)
//
// If "first_tilt=negative" although you tilted forward first, flip the signs
// BRI_PITCH_FWD_SIGN and BRI_PITCH_GYRO_SIGN in BRI_Config.h.
//
// What you need: Teensy + IMU.  No motors.
//
// Commands
//   g        start the run
//   s        stop the run and print the results
//   t<sec>   run length in seconds (default 20, includes the 2 s hold)
//   ?        help
#include <BRI.h>

static const char TEST_ID[] = "CV-005";
static const char TEST_TITLE[] = "IMU pitch orientation";
static const char HEADER[] = "t_ms,phase,pitch_acc_deg,pitch_gyro_deg,pitch_cf_deg,rate_dps,ax_g,ay_g,az_g,i2c_err";

// ---- test settings ------------------------------------------------------------
static const float DEFAULT_RUN_S = 20.0f;
// How long the robot must be held still at the start.  2 s of samples gives
// a gyro offset good to a small fraction of a degree per second.
static const uint32_t STILL_MS = 2000;
// The IMU is read every 5 ms (200 times a second), which is the rate the
// balance controller will eventually need; rows are still printed every 20 ms.
static const uint32_t IMU_PERIOD_US = 5000;
// A tilt has to be at least this large to count as "the operator tilted it".
// 10 degrees is far above noise and easy to do by hand.
static const float TILT_DETECT_DEG = 10.0f;
// Gyro-integrated angle and accelerometer angle should agree at the peak of
// a slow tilt.  A ratio between 0.5 and 1.5 means "same axis, same sign";
// a wrong sign gives a negative ratio and a wrong axis gives about zero.
static const float RATIO_MIN = 0.5f;
static const float RATIO_MAX = 1.5f;

// Phase numbers used in the "phase" column.
static const int PHASE_STILL = 0;   // measuring zero and gyro offset
static const int PHASE_TILT = 1;    // operator tilts the robot

// ---- run state -----------------------------------------------------------------
static bool running = false;
static int phase = PHASE_STILL;
static uint32_t run_start_ms = 0;
static uint32_t run_length_ms = 0;
static uint32_t last_row_ms = 0;
static uint32_t last_imu_us = 0;       // when the IMU was last read (microseconds)
static uint32_t read_fail_count = 0;
static BriStats still_pitch_stats;     // accelerometer pitch during the hold
static BriStats still_rate_stats;      // gyro rate during the hold
static float zero_deg = 0.0f;          // pitch while held upright
static float gyro_bias_dps = 0.0f;     // gyro reading while not rotating
static float pitch_acc = 0.0f;         // latest accelerometer pitch
static float pitch_gyro = 0.0f;        // latest integrated-gyro pitch
static float pitch_cf = 0.0f;          // latest complementary-filter pitch
static float rate_dps = 0.0f;          // latest raw pitch rate
static BriImuSample latest_sample;     // latest raw sample (for the row)
static float peak_acc_dev = 0.0f;      // largest accelerometer tilt away from zero (signed)
static float peak_gyro_dev = 0.0f;     // integrated-gyro tilt at that same moment (signed)
static int first_tilt_sign = 0;        // +1 / -1 = direction of the first tilt, 0 = none seen
static BriStats cf_error_stats;        // (pitch_cf - pitch_acc) during the tilt phase

// Print the commands this sketch understands.  Shown at boot and when the
// operator types '?', so nobody has to open the source to use the test.
static void printHelp() {
  Serial.println("# CV-005 commands:");
  briPrintCommonHelp();
  Serial.println("#   t<sec>    run length in seconds (default 20)");
  Serial.println("# hold upright and still for 2 s, then tilt FORWARD, upright, BACKWARD, slowly");
}

// Declared early because startRun() (above finishRun in this file) needs to
// call it when the IMU is missing.
static void finishRun();

// Begin a run: banner, header, clear the statistics, find and configure
// the IMU, and enter the hold-still phase in which the zero angle and the
// gyro offset are measured.
static void startRun() {
  briTelemBanner();
  briTelemHeader(HEADER);
  briResultsReset();
  briStatsReset(&still_pitch_stats);
  briStatsReset(&still_rate_stats);
  briStatsReset(&cf_error_stats);
  read_fail_count = 0;
  peak_acc_dev = 0.0f;
  peak_gyro_dev = 0.0f;
  first_tilt_sign = 0;
  phase = PHASE_STILL;

  briI2cBegin();
  briI2cClearErrors();
  uint8_t address = briImuFindAddress();
  bool imu_ok = false;
  if (address != 0) {
    imu_ok = briImuBegin(address);
  }
  run_start_ms = millis();
  last_row_ms = run_start_ms;
  last_imu_us = micros();
  running = true;
  if (!imu_ok) {
    briResultBegin(BRI_FAIL);
    briText("imu_not_found: run CV-003 first");
    briResultEnd();
    finishRun();
    return;
  }
  Serial.println("# hold STILL and upright for 2 seconds...");
}

// Read the IMU once and update the three pitch estimates.
static void updateEstimates(float dt_s) {
  BriImuSample sample = briImuRead();
  if (!sample.ok) {
    read_fail_count = read_fail_count + 1;
    return;
  }
  latest_sample = sample;
  pitch_acc = briImuAccelPitchDeg(sample);
  rate_dps = briImuPitchRateDps(sample);

  if (phase == PHASE_STILL) {
    // Just collect: the averages become the zero angle and the gyro offset.
    briStatsAdd(&still_pitch_stats, pitch_acc);
    briStatsAdd(&still_rate_stats, rate_dps);
    pitch_gyro = pitch_acc;
    pitch_cf = pitch_acc;
    return;
  }

  // Rotation rate with the at-rest offset removed.
  float rate_corrected = rate_dps - gyro_bias_dps;

  // Estimate 1 (pitch_acc) is already done above.
  // Estimate 2: gyro only.  Angle = previous angle + rate * time.  Quick and
  // smooth, but any small rate error adds up, so it drifts.
  pitch_gyro = pitch_gyro + rate_corrected * dt_s;

  // Estimate 3: complementary filter.  Take the gyro's step, then pull the
  // result a little toward the accelerometer angle.  "alpha" close to 1
  // means "mostly gyro"; the remainder (1 - alpha) is the pull.
  float alpha = BRI_COMP_FILTER_TAU_S / (BRI_COMP_FILTER_TAU_S + dt_s);
  pitch_cf = alpha * (pitch_cf + rate_corrected * dt_s) + (1.0f - alpha) * pitch_acc;
  briStatsAdd(&cf_error_stats, pitch_cf - pitch_acc);

  // Remember the largest tilt and what the gyro estimate said at that moment.
  float acc_dev = pitch_acc - zero_deg;
  if (fabsf(acc_dev) > fabsf(peak_acc_dev)) {
    peak_acc_dev = acc_dev;
    peak_gyro_dev = pitch_gyro - zero_deg;
  }
  // Remember which way the FIRST clear tilt went.
  float cf_dev = pitch_cf - zero_deg;
  if (first_tilt_sign == 0 && fabsf(cf_dev) > TILT_DETECT_DEG) {
    first_tilt_sign = (cf_dev > 0.0f) ? 1 : -1;
    briEventBegin("TILT");
    briKVText("first_tilt", first_tilt_sign > 0 ? "positive" : "negative");
    briEventEnd();
  }
}

// Move from the hold-still phase to the tilt phase.
static void endStillPhase() {
  zero_deg = briStatsMean(&still_pitch_stats);
  gyro_bias_dps = briStatsMean(&still_rate_stats);
  pitch_gyro = zero_deg;
  pitch_cf = zero_deg;
  phase = PHASE_TILT;
  briEventBegin("ZERO");
  briKVFloat("pitch_zero_deg", zero_deg, 2);
  briKVFloat("gyro_bias_dps", gyro_bias_dps, 3);
  briEventEnd();
  Serial.println("# now tilt FORWARD, back to upright, then BACKWARD");
}

// End a run and print the verdicts: the zero angle, whether the gyro and
// the accelerometer agreed about the tilt, which way the first tilt read,
// and how well the blended estimate tracked.
static void finishRun() {
  running = false;
  if (still_pitch_stats.count == 0) {
    briResultBegin(BRI_FAIL);
    briText("no_imu_samples");
    briResultEnd();
    briResultOverall();
    return;
  }

  // Result 1: the zero reference, for the record.
  briResultBegin(BRI_INFO);
  briKVFloat("pitch_zero_deg", zero_deg, 2);
  briKVFloat("gyro_bias_dps", gyro_bias_dps, 3);
  briText("put pitch_zero_deg into BRI_PITCH_ZERO_DEG once the IMU is in its final mount");
  briResultEnd();

  if (fabsf(peak_acc_dev) < TILT_DETECT_DEG) {
    // Nobody tilted it: nothing about signs can be said.
    briResultBegin(BRI_TODO);
    briKVFloat("largest_tilt_deg", peak_acc_dev, 1);
    briText("operator: no tilt of 10 deg or more seen; repeat and tilt forward then back");
    briResultEnd();
  } else {
    // Result 2: gyro and accelerometer agree about direction and size.
    float ratio = peak_gyro_dev / peak_acc_dev;
    bool agree = (ratio >= RATIO_MIN) && (ratio <= RATIO_MAX);
    briResultBegin(agree ? BRI_PASS : BRI_FAIL);
    briKVFloat("peak_acc_deg", peak_acc_dev, 1);
    briKVFloat("peak_gyro_deg", peak_gyro_dev, 1);
    briKVFloat("gyro_over_acc", ratio, 2);
    if (!agree) {
      briText("gyro axis or sign does not match the accelerometer: check BRI_PITCH_GYRO_AXIS/SIGN");
    }
    briResultEnd();

    // Result 3: which way was "positive"?  Only the operator knows which way
    // the robot was really tilted first.
    briResultBegin(BRI_TODO);
    briKVText("first_tilt", first_tilt_sign > 0 ? "positive" : (first_tilt_sign < 0 ? "negative" : "none"));
    briText("operator: must be positive if the first tilt was FORWARD");
    briResultEnd();

    // Result 4: how closely the blended estimate followed the accelerometer.
    briResultBegin(BRI_INFO);
    briKVFloat("cf_minus_acc_mean_deg", briStatsMean(&cf_error_stats), 2);
    briKVFloat("cf_minus_acc_std_deg", briStatsStd(&cf_error_stats), 2);
    briKVFloat("gyro_drift_end_deg", pitch_gyro - pitch_acc, 2);
    briResultEnd();
  }

  briResultBegin(read_fail_count == 0 ? BRI_PASS : BRI_FAIL);
  briKVInt("imu_read_failures", (long)read_fail_count);
  briResultEnd();

  briResultOverall();
}

// Act on one complete command line from the PC.  All operator input comes
// through here, so this is the only place the sketch changes what it is doing.
static void handleCommand(const char* command) {
  char letter = command[0];
  if (letter == '?') {
    printHelp();
  } else if (letter == 'g') {
    startRun();
  } else if (letter == 's') {
    briSafetyStop("operator");
    if (running) {
      finishRun();
    }
  } else if (letter == 't') {
    float seconds = briCommandNumber(command, DEFAULT_RUN_S);
    if (seconds < 4.0f) {
      seconds = 4.0f;   // must be longer than the 2 s hold
    }
    if (seconds > 3600.0f) {
      seconds = 3600.0f;
    }
    run_length_ms = (uint32_t)(seconds * 1000.0f);
  } else {
    Serial.println("# unknown command, type ? for help");
  }
}

// Runs once at power-up.  briBegin() makes the motor pins safe first
// (drivers asleep), then starts the ADC, the boot counter and USB serial.
// Nothing is measured and nothing moves until the operator sends a command.
void setup() {
  briBegin(TEST_ID, TEST_TITLE);
  run_length_ms = (uint32_t)(DEFAULT_RUN_S * 1000.0f);
  printHelp();
}

// Runs over and over and never waits.  Each pass: blink the LED, look for
// a command, and during a run update the three pitch estimates every 5 ms,
// switch from 'hold still' to 'tilt' after 2 s, and print a row every 20 ms.
void loop() {
  briHeartbeat(running);

  char command[BRI_CMD_MAX];
  if (briReadCommand(command, sizeof(command))) {
    handleCommand(command);
  }
  if (!running) {
    return;
  }

  // IMU update at a fixed 5 ms pace.  micros() is used rather than millis()
  // because the time step goes into the integration and 1 ms resolution
  // would be a 20 % error on a 5 ms step.
  uint32_t now_us = micros();
  uint32_t elapsed_us = now_us - last_imu_us;
  if (elapsed_us >= IMU_PERIOD_US) {
    last_imu_us = now_us;
    updateEstimates((float)elapsed_us / 1000000.0f);
  }

  uint32_t now_ms = millis();
  if (phase == PHASE_STILL && (now_ms - run_start_ms) >= STILL_MS) {
    endStillPhase();
  }

  if ((now_ms - last_row_ms) >= BRI_TELEMETRY_PERIOD_MS) {
    last_row_ms = now_ms;
    briRowBegin();                              // t_ms
    briRowInt(phase);                           // phase
    briRowFloat(pitch_acc, 2);                  // pitch_acc_deg
    briRowFloat(pitch_gyro, 2);                 // pitch_gyro_deg
    briRowFloat(pitch_cf, 2);                   // pitch_cf_deg
    briRowFloat(rate_dps, 2);                   // rate_dps
    briRowFloat(latest_sample.ax, 4);           // ax_g
    briRowFloat(latest_sample.ay, 4);           // ay_g
    briRowFloat(latest_sample.az, 4);           // az_g
    briRowInt((long)briI2cErrorCount());        // i2c_err
    briRowEnd();
  }

  if ((now_ms - run_start_ms) >= run_length_ms) {
    finishRun();
  }
}
