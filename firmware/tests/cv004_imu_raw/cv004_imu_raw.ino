// CV-004  Raw IMU data
//
// What this test shows
//   * the accelerometer measures gravity: with the IMU held still, the length
//     of the (ax, ay, az) vector is 0.9 to 1.1 g
//   * the gyro reads close to zero when nothing is rotating
//   * which sensor axis points up (the one reading about +1 g), which you need
//     for the axis constants in BRI_Config.h
//
// How to run it: hold the IMU still (on the table) for the whole run.  Repeat
// with the board in other known orientations and note which axis shows +1 g
// or -1 g each time.
//
// What you need: Teensy + IMU.  No motors.
//
// Commands
//   g        start the run
//   s        stop the run and print the results
//   t<sec>   run length in seconds (default 10)
//   ?        help
#include <BRI.h>

static const char TEST_ID[] = "CV-004";
static const char TEST_TITLE[] = "Raw IMU data";
static const char HEADER[] = "t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,amag_g,temp_C,i2c_err";

// ---- test settings ------------------------------------------------------------
static const float DEFAULT_RUN_S = 10.0f;
// Gravity is 1 g by definition.  +/-10 % allows for an uncalibrated sensor.
static const float GRAVITY_MIN_G = 0.90f;
static const float GRAVITY_MAX_G = 1.10f;
// An uncalibrated MPU gyro can sit several deg/s away from zero when still
// (the datasheets quote about +/-5 deg/s).  8 deg/s catches a wrong scale or
// a broken axis without failing a normal part.
static const float GYRO_OFFSET_MAX_DPS = 8.0f;
// While held still the reading should also not wander: more than 2 deg/s of
// spread means the board was moving (or the supply is noisy).
static const float GYRO_SPREAD_MAX_DPS = 2.0f;

// ---- run state -----------------------------------------------------------------
static bool running = false;
static uint32_t run_start_ms = 0;
static uint32_t run_length_ms = 0;
static uint32_t last_row_ms = 0;
static uint32_t read_fail_count = 0;   // IMU reads that failed in this run
static BriStats accel_stats[3];        // ax, ay, az
static BriStats gyro_stats[3];         // gx, gy, gz
static BriStats magnitude_stats;       // length of the acceleration vector

// Print the commands this sketch understands.  Shown at boot and when the
// operator types '?', so nobody has to open the source to use the test.
static void printHelp() {
  Serial.println("# CV-004 commands:");
  briPrintCommonHelp();
  Serial.println("#   t<sec>    run length in seconds (default 10)");
  Serial.println("# hold the IMU still for the whole run");
}

// Declared early because startRun() (above finishRun in this file) needs to
// call it when the IMU is missing.
static void finishRun();

// Begin a run: banner, header, clear the statistics, then find and
// configure the IMU.  If no IMU answers, the run ends at once with a FAIL
// rather than streaming zeros.
static void startRun() {
  briTelemBanner();
  briTelemHeader(HEADER);
  briResultsReset();
  for (uint8_t axis = 0; axis < 3; axis++) {
    briStatsReset(&accel_stats[axis]);
    briStatsReset(&gyro_stats[axis]);
  }
  briStatsReset(&magnitude_stats);
  read_fail_count = 0;

  briI2cBegin();
  briI2cClearErrors();
  uint8_t address = briImuFindAddress();
  bool imu_ok = false;
  if (address != 0) {
    imu_ok = briImuBegin(address);
  }
  run_start_ms = millis();
  last_row_ms = run_start_ms;
  running = true;
  if (!imu_ok) {
    // No IMU: say so and end the run at once (the header was still printed,
    // so the logger sees a well-formed, empty capture).
    briResultBegin(BRI_FAIL);
    briText("imu_not_found: run CV-003 first");
    briResultEnd();
    finishRun();
  }
}

// End a run and print the verdicts: gravity magnitude, gyro offset and
// steadiness, which axis points up, and whether any read failed.
static void finishRun() {
  running = false;
  if (magnitude_stats.count == 0) {
    briResultBegin(BRI_FAIL);
    briText("no_imu_samples");
    briResultEnd();
    briResultOverall();
    return;
  }

  // Check 1: gravity magnitude.
  float magnitude = briStatsMean(&magnitude_stats);
  bool gravity_ok = (magnitude >= GRAVITY_MIN_G) && (magnitude <= GRAVITY_MAX_G);
  briResultBegin(gravity_ok ? BRI_PASS : BRI_FAIL);
  briKVFloat("gravity_g", magnitude, 3);
  briKVFloat("min", GRAVITY_MIN_G, 2);
  briKVFloat("max", GRAVITY_MAX_G, 2);
  briResultEnd();

  // Check 2: gyro near zero and steady.
  bool gyro_ok = true;
  for (uint8_t axis = 0; axis < 3; axis++) {
    if (fabsf(briStatsMean(&gyro_stats[axis])) > GYRO_OFFSET_MAX_DPS) {
      gyro_ok = false;
    }
    if (briStatsStd(&gyro_stats[axis]) > GYRO_SPREAD_MAX_DPS) {
      gyro_ok = false;
    }
  }
  briResultBegin(gyro_ok ? BRI_PASS : BRI_FAIL);
  briKVFloat("gx_mean", briStatsMean(&gyro_stats[0]), 2);
  briKVFloat("gy_mean", briStatsMean(&gyro_stats[1]), 2);
  briKVFloat("gz_mean", briStatsMean(&gyro_stats[2]), 2);
  briKVFloat("gx_std", briStatsStd(&gyro_stats[0]), 2);
  briKVFloat("gy_std", briStatsStd(&gyro_stats[1]), 2);
  briKVFloat("gz_std", briStatsStd(&gyro_stats[2]), 2);
  briResultEnd();

  // Check 3: which way is up?  Reported so it can be written into
  // BRI_Config.h; no judgement, because any orientation is legitimate.
  briResultBegin(BRI_INFO);
  briKVFloat("ax_mean", briStatsMean(&accel_stats[0]), 3);
  briKVFloat("ay_mean", briStatsMean(&accel_stats[1]), 3);
  briKVFloat("az_mean", briStatsMean(&accel_stats[2]), 3);
  briText("the axis nearest +1 points up");
  briResultEnd();

  // Check 4: every read succeeded.
  briResultBegin(read_fail_count == 0 ? BRI_PASS : BRI_FAIL);
  briKVInt("imu_read_failures", (long)read_fail_count);
  briKVInt("samples", (long)magnitude_stats.count);
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
    if (seconds < 1.0f) {
      seconds = 1.0f;
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
// a command, and during a run read the IMU and print one row every 20 ms
// until the run length is reached.
void loop() {
  briHeartbeat(running);

  char command[BRI_CMD_MAX];
  if (briReadCommand(command, sizeof(command))) {
    handleCommand(command);
  }
  if (!running) {
    return;
  }

  uint32_t now_ms = millis();
  if ((now_ms - last_row_ms) >= BRI_TELEMETRY_PERIOD_MS) {
    last_row_ms = now_ms;
    BriImuSample sample = briImuRead();
    if (!sample.ok) {
      read_fail_count = read_fail_count + 1;
    } else {
      // Length of the acceleration vector (Pythagoras in three dimensions).
      float magnitude = sqrtf(sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az);
      briStatsAdd(&accel_stats[0], sample.ax);
      briStatsAdd(&accel_stats[1], sample.ay);
      briStatsAdd(&accel_stats[2], sample.az);
      briStatsAdd(&gyro_stats[0], sample.gx);
      briStatsAdd(&gyro_stats[1], sample.gy);
      briStatsAdd(&gyro_stats[2], sample.gz);
      briStatsAdd(&magnitude_stats, magnitude);

      briRowBegin();                              // t_ms
      briRowFloat(sample.ax, 4);                  // ax_g
      briRowFloat(sample.ay, 4);                  // ay_g
      briRowFloat(sample.az, 4);                  // az_g
      briRowFloat(sample.gx, 3);                  // gx_dps
      briRowFloat(sample.gy, 3);                  // gy_dps
      briRowFloat(sample.gz, 3);                  // gz_dps
      briRowFloat(magnitude, 4);                  // amag_g
      briRowFloat(sample.temp_c, 1);              // temp_C
      briRowInt((long)briI2cErrorCount());        // i2c_err
      briRowEnd();
    }
  }

  if ((now_ms - run_start_ms) >= run_length_ms) {
    finishRun();
  }
}
