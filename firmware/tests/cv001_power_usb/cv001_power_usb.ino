// CV-001  Teensy power and USB
//
// What this test shows
//   * the Teensy boots, enumerates on USB and prints a banner
//   * the main loop keeps running (heartbeat LED, loop rate, uptime)
//   * the processor temperature and the battery-sense input give plausible values
//   * (with your multimeter) the 3.3 V rail is correct and USB does not
//     back-feed the robot's 5 V rail
//
// What you need: the Teensy, a USB cable, a multimeter.  No motors.
//
// Commands (type in the Serial Monitor, or let cv001_power_usb.py send them)
//   g        start the run
//   s        stop the run and print the results
//   t<sec>   run length in seconds (default 10)
//   r<volts> 3.3 V rail as measured with the multimeter, e.g. r3.29
//   b<volts> back-feed voltage: with ONLY USB plugged in, the voltage on the
//            robot's 5 V rail (should be near 0), e.g. b0.02
//   ?        help
#include <BRI.h>

static const char TEST_ID[] = "CV-001";
static const char TEST_TITLE[] = "Teensy power and USB";
static const char HEADER[] = "t_ms,uptime_s,boot_count,cpu_temp_C,vbat_V,vbat_pin_V,loop_hz";

// ---- test settings (each with its reason) -----------------------------------
// Default run length.  10 s is long enough to watch the LED and see a stable
// temperature, short enough not to be tedious.
static const float DEFAULT_RUN_S = 10.0f;
// One row every 100 ms: nothing here changes quickly.
static const uint32_t ROW_PERIOD_MS = 100;
// A Teensy on a bench should sit between room temperature and about 60 C.
// Outside 0..85 C the reading (or the board) is wrong.
static const float CPU_TEMP_MIN_C = 0.0f;
static const float CPU_TEMP_MAX_C = 85.0f;
// The 3.3 V regulator on the Teensy is good to a few percent.
static const float RAIL_MIN_V = 3.20f;
static const float RAIL_MAX_V = 3.40f;
// "No back-feed" means the other supply's rail stays near zero.  0.3 V allows
// for leakage through protection diodes without calling it powered.
static const float BACKFEED_MAX_V = 0.30f;

// ---- run state -----------------------------------------------------------------
static bool running = false;           // true between "g" and the end of the run
static uint32_t run_start_ms = 0;      // when the run started
static uint32_t run_length_ms = 0;     // how long it should last
static uint32_t last_row_ms = 0;       // when the last data row was printed
static uint32_t loop_count = 0;        // loop() passes since the last row (for loop_hz)
static uint32_t row_count = 0;         // rows printed in this run
static float rail_v = NAN;             // operator's 3.3 V reading (NAN = not entered)
static float backfeed_v = NAN;         // operator's back-feed reading (NAN = not entered)
static BriStats temp_stats;            // CPU temperature over the run
static BriStats vbat_stats;            // battery voltage over the run

// Print the list of commands this sketch understands.
static void printHelp() {
  Serial.println("# CV-001 commands:");
  briPrintCommonHelp();
  Serial.println("#   t<sec>    run length in seconds (default 10)");
  Serial.println("#   r<volts>  3.3 V rail measured with a multimeter, e.g. r3.29");
  Serial.println("#   b<volts>  5 V rail voltage with ONLY USB connected (back-feed), e.g. b0.02");
}

// Begin a run: banner and header first so every capture explains itself.
static void startRun() {
  briTelemBanner();
  briTelemHeader(HEADER);
  briResultsReset();
  briStatsReset(&temp_stats);
  briStatsReset(&vbat_stats);
  row_count = 0;
  loop_count = 0;
  run_start_ms = millis();
  last_row_ms = run_start_ms;
  running = true;
}

// End a run: print one RESULT line per check, then the overall line.
static void finishRun() {
  running = false;

  // Check 1: the board ran and the temperature sensor gives a sane value.
  float temp_mean = briStatsMean(&temp_stats);
  bool temp_ok = (row_count > 0) && (temp_mean >= CPU_TEMP_MIN_C) && (temp_mean <= CPU_TEMP_MAX_C);
  briResultBegin(temp_ok ? BRI_PASS : BRI_FAIL);
  briText("boot_and_loop");
  briKVInt("rows", (long)row_count);
  briKVFloat("cpu_temp_C", temp_mean, 1);
  briKVInt("boot_count", (long)briBootCount());
  briResultEnd();

  // Check 2: battery sense, for the record only.  With USB-only power it
  // reads about 0 V, which is correct, so no judgement is made here.
  briResultBegin(BRI_INFO);
  briKVFloat("vbat_V", briStatsMean(&vbat_stats), 2);
  if (briStatsMean(&vbat_stats) < BRI_VBAT_PRESENT_V) {
    briText("battery_not_connected");
  }
  briResultEnd();

  // Check 3: 3.3 V rail, needs the operator's multimeter.
  if (isnan(rail_v)) {
    briResultBegin(BRI_TODO);
    briText("operator: measure the 3.3V pin and type r<volts>");
    briResultEnd();
  } else {
    bool rail_ok = (rail_v >= RAIL_MIN_V) && (rail_v <= RAIL_MAX_V);
    briResultBegin(rail_ok ? BRI_PASS : BRI_FAIL);
    briKVFloat("rail3v3_V", rail_v, 3);
    briKVFloat("min", RAIL_MIN_V, 2);
    briKVFloat("max", RAIL_MAX_V, 2);
    briResultEnd();
  }

  // Check 4: no USB back-feed, needs the operator's multimeter.
  if (isnan(backfeed_v)) {
    briResultBegin(BRI_TODO);
    briText("operator: with only USB connected measure the 5V rail and type b<volts>");
    briResultEnd();
  } else {
    bool backfeed_ok = backfeed_v <= BACKFEED_MAX_V;
    briResultBegin(backfeed_ok ? BRI_PASS : BRI_FAIL);
    briKVFloat("backfeed_V", backfeed_v, 3);
    briKVFloat("max", BACKFEED_MAX_V, 2);
    briResultEnd();
  }
  briResultOverall();
}

// Act on one command line from the PC.
static void handleCommand(const char* command) {
  char letter = command[0];
  if (letter == '?') {
    printHelp();
  } else if (letter == 'g') {
    startRun();
  } else if (letter == 's') {
    // "s" always puts the motor drivers to sleep, even in a test with no
    // motors, so the key means the same thing in every sketch.
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
  } else if (letter == 'r') {
    rail_v = briCommandNumber(command, NAN);
    briEventBegin("DMM");
    briKVFloat("rail3v3", rail_v, 3);
    briEventEnd();
  } else if (letter == 'b') {
    backfeed_v = briCommandNumber(command, NAN);
    briEventBegin("DMM");
    briKVFloat("backfeed", backfeed_v, 3);
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
// a command, and during a run print one row every 100 ms (temperature,
// battery sense, loop rate) until the run length is reached.
void loop() {
  briHeartbeat(running);

  char command[BRI_CMD_MAX];
  if (briReadCommand(command, sizeof(command))) {
    handleCommand(command);
  }
  if (!running) {
    return;
  }

  loop_count = loop_count + 1;
  uint32_t now_ms = millis();
  uint32_t since_row_ms = now_ms - last_row_ms;
  if (since_row_ms >= ROW_PERIOD_MS) {
    last_row_ms = now_ms;
    float cpu_temp = briReadCpuTempC();
    float vbat_pin = briReadVbatPinVolts();
    float vbat = vbat_pin * BRI_VBAT_DIVIDER;
    // Loop rate = passes through loop() per second since the last row.
    float loop_hz = (float)loop_count * 1000.0f / (float)since_row_ms;
    loop_count = 0;
    briStatsAdd(&temp_stats, cpu_temp);
    briStatsAdd(&vbat_stats, vbat);

    briRowBegin();                                   // t_ms
    briRowFloat((float)now_ms / 1000.0f, 2);         // uptime_s
    briRowInt((long)briBootCount());                 // boot_count
    briRowFloat(cpu_temp, 1);                        // cpu_temp_C
    briRowFloat(vbat, 2);                            // vbat_V
    briRowFloat(vbat_pin, 3);                        // vbat_pin_V
    briRowFloat(loop_hz, 0);                         // loop_hz
    briRowEnd();
    row_count = row_count + 1;
  }

  if ((now_ms - run_start_ms) >= run_length_ms) {
    finishRun();
  }
}
