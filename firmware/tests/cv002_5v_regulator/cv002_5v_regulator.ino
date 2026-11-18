// CV-002  5 V regulator under stepped load
//
// What this test shows
//   * the 5 V regulator (Pololu D24V90F5) holds 4.8 to 5.2 V while you step
//     the load up
//   * the Teensy does not reset while that happens
//
// The Teensy cannot measure the 5 V rail or the load current itself, so YOU
// are the measuring instrument: set a load, read your multimeter, type the
// numbers.  The firmware logs the battery voltage, its own uptime and its
// boot counter alongside, and the PC logger watches for a reset.
//
// What you need: battery or bench supply, an adjustable load (electronic load
// or power resistors), a multimeter.  No motors.
//
// Procedure for each load step
//   1. set the load
//   2. type i<amps>   the load current, e.g. i2.0
//   3. type v<volts>  the regulator output at that load, e.g. v4.98
//      (each "v" records one load step using the most recent "i")
//
// Commands
//   g        start the run
//   s        stop the run and print the results
//   t<sec>   run length in seconds (default 300)
//   i<amps>  load current now
//   v<volts> regulator output voltage now (records a load step)
//   ?        help
#include <BRI.h>

static const char TEST_ID[] = "CV-002";
static const char TEST_TITLE[] = "5 V regulator";
static const char HEADER[] = "t_ms,uptime_s,boot_count,vbat_V,cpu_temp_C,vout_dmm_V,iout_dmm_A,entries";

// ---- test settings ------------------------------------------------------------
// Five minutes gives time to step through several loads by hand; "s" ends it
// sooner.
static const float DEFAULT_RUN_S = 300.0f;
// Five rows per second is plenty for a voltage that should not move.
static const uint32_t ROW_PERIOD_MS = 200;
// Pass band for the 5 V rail (+/- 4 %): inside what the Teensy VIN pin and
// the 5 V logic on the encoders and drivers accept.
static const float VOUT_MIN_V = 4.80f;
static const float VOUT_MAX_V = 5.20f;

// ---- run state -----------------------------------------------------------------
static bool running = false;
static uint32_t run_start_ms = 0;
static uint32_t run_length_ms = 0;
static uint32_t last_row_ms = 0;
static float vout_dmm = NAN;        // last regulator voltage typed by the operator
static float iout_dmm = NAN;        // last load current typed by the operator
static uint32_t entry_count = 0;    // load steps recorded in this run
static uint32_t entry_bad = 0;      // how many of them were outside the pass band
static BriStats vout_stats;         // all recorded regulator voltages
static BriStats iout_stats;         // all recorded load currents
static BriStats vbat_stats;         // battery voltage over the run

// Print the commands this sketch understands.  Shown at boot and when the
// operator types '?', so nobody has to open the source to use the test.
static void printHelp() {
  Serial.println("# CV-002 commands:");
  briPrintCommonHelp();
  Serial.println("#   t<sec>    run length in seconds (default 300)");
  Serial.println("#   i<amps>   load current now, e.g. i2.0 (type this first)");
  Serial.println("#   v<volts>  regulator output now, e.g. v4.98 (records one load step)");
}

// Begin a run: print the banner and header (so the capture explains
// itself), forget the readings of any earlier run, start the clock.
static void startRun() {
  briTelemBanner();
  briTelemHeader(HEADER);
  briResultsReset();
  briStatsReset(&vout_stats);
  briStatsReset(&iout_stats);
  briStatsReset(&vbat_stats);
  vout_dmm = NAN;
  iout_dmm = NAN;
  entry_count = 0;
  entry_bad = 0;
  run_start_ms = millis();
  last_row_ms = run_start_ms;
  running = true;
}

// End a run and print the verdicts: were all the voltages you typed inside
// 4.8..5.2 V, what the input voltage was, and the facts the PC logger needs
// to decide whether the Teensy reset.
static void finishRun() {
  running = false;

  // Check 1: every recorded output voltage inside the pass band.
  if (entry_count == 0) {
    briResultBegin(BRI_TODO);
    briText("operator: no load steps recorded; type i<amps> then v<volts> at each load");
    briResultEnd();
  } else {
    briResultBegin(entry_bad == 0 ? BRI_PASS : BRI_FAIL);
    briKVInt("load_steps", (long)entry_count);
    briKVInt("out_of_band", (long)entry_bad);
    briKVFloat("vout_min_V", vout_stats.min_value, 3);
    briKVFloat("vout_max_V", vout_stats.max_value, 3);
    briKVFloat("iout_max_A", iout_stats.max_value, 2);
    briResultEnd();
  }

  // Check 2: input voltage seen during the test, for the record.
  briResultBegin(BRI_INFO);
  briKVFloat("vbat_min_V", vbat_stats.min_value, 2);
  briKVFloat("vbat_max_V", vbat_stats.max_value, 2);
  briResultEnd();

  // Check 3: reset detection.  The firmware that prints this line is, by
  // definition, still running, so it cannot see its own reset.  It reports
  // the facts the PC logger needs to decide.
  briResultBegin(BRI_INFO);
  briKVInt("boot_count", (long)briBootCount());
  briKVFloat("uptime_s", (float)millis() / 1000.0f, 1);
  briText("reset detection is done by the PC logger");
  briResultEnd();

  briResultOverall();
}

// Act on one complete command line from the PC.  All operator input comes
// through here, so this is the only place the sketch changes what it is doing.
// Here that includes the multimeter readings (i = load current, v = output voltage).
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
  } else if (letter == 'i') {
    iout_dmm = briCommandNumber(command, NAN);
    Serial.println("# load current noted; now type v<volts>");
  } else if (letter == 'v') {
    float volts = briCommandNumber(command, NAN);
    if (isnan(volts)) {
      Serial.println("# v needs a number, e.g. v4.98");
      return;
    }
    vout_dmm = volts;
    entry_count = entry_count + 1;
    briStatsAdd(&vout_stats, vout_dmm);
    if (!isnan(iout_dmm)) {
      briStatsAdd(&iout_stats, iout_dmm);
    }
    if (vout_dmm < VOUT_MIN_V || vout_dmm > VOUT_MAX_V) {
      entry_bad = entry_bad + 1;
    }
    // The event carries everything about this load step, so the capture can
    // be re-evaluated later without the operator.
    briEventBegin("DMM");
    briKVFloat("vout", vout_dmm, 3);
    briKVFloat("iout", iout_dmm, 2);
    briKVFloat("vbat", briReadVbat(), 2);
    briEventEnd();
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
// a command, and during a run print one row every 200 ms (battery, uptime,
// boot counter, your latest readings) until the run length is reached.
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
  if ((now_ms - last_row_ms) >= ROW_PERIOD_MS) {
    last_row_ms = now_ms;
    float vbat = briReadVbat();
    briStatsAdd(&vbat_stats, vbat);

    briRowBegin();                             // t_ms
    briRowFloat((float)now_ms / 1000.0f, 2);   // uptime_s
    briRowInt((long)briBootCount());           // boot_count
    briRowFloat(vbat, 2);                      // vbat_V
    briRowFloat(briReadCpuTempC(), 1);         // cpu_temp_C
    briRowFloat(vout_dmm, 3);                  // vout_dmm_V  ("nan" until entered)
    briRowFloat(iout_dmm, 2);                  // iout_dmm_A  ("nan" until entered)
    briRowInt((long)entry_count);              // entries
    briRowEnd();
  }

  if ((now_ms - run_start_ms) >= run_length_ms) {
    finishRun();
  }
}
