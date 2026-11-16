// CV-006  Motor driver logic
//
// !! MOTOR DISCONNECTED (preferred) OR SHAFT RESTRAINED !!
// This test drives the selected G2 driver at 10 % duty for a second or so in
// each direction.  With the motor unplugged nothing moves and the current
// sense shows its zero offset.  With a motor connected it WILL try to turn.
//
// What this test shows, step by step (one step per second by default)
//   step 0  driver asleep            enable read-back must be LOW
//   step 1  driver awake, PWM 0      enable read-back must be HIGH, no fault
//   step 2  DIR forward, 10 % PWM    CS shows about 50 mV (motor unplugged)
//   step 3  PWM 0 (brake)
//   step 4  DIR reverse, 10 % PWM    CS shows about 50 mV (motor unplugged)
//   step 5  PWM 0 (brake)
//   step 6  driver asleep again      enable read-back must be LOW
//
// What you need: battery or bench supply on the driver, the driver under test
// wired to the Teensy, kill switch in RUN.  Optional: multimeter across
// OUTA/OUTB to see the output follow DIR (use t5 for 5 s steps).
//
// Commands
//   c<n>     choose channel: 0 = LW, 1 = RW, 2 = LH, 3 = RH (default 0)
//   t<sec>   seconds per step (default 1)
//   m<0|1>   m0 = motor unplugged (default), m1 = a motor is connected and
//            restrained.  The firmware cannot tell which, and it matters: the
//            CS reading is "the zero offset" only when no current can flow.
//   g        GO: run the seven steps
//   s        stop now (brake, sleep) and print the results
//   ?        help
#include <BRI.h>

static const char TEST_ID[] = "CV-006";
static const char TEST_TITLE[] = "Motor driver logic";
static const char HEADER[] = "t_ms,step,en_cmd,en_sense,dir_cmd,duty,flt_ok,cs_peak_mV,cs_mean_mV,i_A,vbat_V";

// ---- test settings ------------------------------------------------------------
// Duty used in the two driving steps.  10 % is enough for the driver to
// switch and for CS to show its offset, and little enough that a connected,
// restrained motor draws well under 1 A.
static const float TEST_DUTY = 0.10f;
// Default length of each step.  One second gives 25 settled samples per step.
static const float DEFAULT_STEP_S = 1.0f;
// The CS zero offset is nominally 50 mV.  30..80 mV is "the offset, give or
// take the part's tolerance and ADC noise".  Below 30 mV CS is not responding
// at all.  NOTE: this band cannot prove that no current flows - a restrained
// motor at 10 % duty draws about 0.5 A, which adds only about 10 mV.  That is
// why the operator has to say (m0 / m1) whether a motor is connected.
static const float CS_OFFSET_MIN_MV = 30.0f;
static const float CS_OFFSET_MAX_MV = 80.0f;

// The seven steps: is the driver awake, and what duty is asked for?
static const uint8_t STEP_COUNT = 7;
static const bool STEP_AWAKE[STEP_COUNT] = { false, true, true, true, true, true, false };
static const float STEP_DUTY[STEP_COUNT] = { 0.0f, 0.0f, +TEST_DUTY, 0.0f, -TEST_DUTY, 0.0f, 0.0f };
static const uint8_t STEP_FORWARD = 2;
static const uint8_t STEP_REVERSE = 4;

// ---- run state -----------------------------------------------------------------
static bool running = false;
static uint8_t channel = 0;             // channel under test
static bool motor_connected = false;    // operator's statement (m0 / m1): is a motor plugged in?
static uint8_t step = 0;                // current step number
static uint32_t step_start_ms = 0;      // when the current step began
static uint32_t step_length_ms = 0;     // how long each step lasts
static uint32_t last_row_ms = 0;
static BriStats cs_peak_stats[STEP_COUNT];   // CS peak (mV) per step, settled half only
static uint32_t sense_wrong_asleep = 0;      // rows where read-back was HIGH while asleep
static uint32_t sense_wrong_awake = 0;       // rows where read-back was LOW while awake
static uint32_t fault_rows_awake = 0;        // rows with FLT low while awake
static uint32_t fault_rows_asleep = 0;       // rows with FLT low while asleep (information only)
static uint32_t settled_rows_asleep = 0;     // rows counted while asleep
static uint32_t settled_rows_awake = 0;      // rows counted while awake

// Print the commands this sketch understands.  Shown at boot and when the
// operator types '?', so nobody has to open the source to use the test.
static void printHelp() {
  Serial.println("# CV-006 commands:   !! motor disconnected or restrained !!");
  briPrintCommonHelp();
  Serial.println("#   c<n>      channel 0=LW 1=RW 2=LH 3=RH (default 0)");
  Serial.println("#   t<sec>    seconds per step (default 1; use 5 to probe with a multimeter)");
  Serial.println("#   m<0|1>    m0 = motor unplugged (default), m1 = motor connected and restrained");
}

// Print the RESULT lines.  `completed` is false if the run was cut short.
static void finishRun(bool completed) {
  running = false;
  uint8_t group = briMotorGroupOf(channel);

  if (!completed) {
    briResultBegin(BRI_FAIL);
    briKVText("stopped", briSafetyStopReason());
    briKVInt("at_step", (long)step);
    briResultEnd();
    briResultOverall();
    return;
  }

  // Check 1: the enable net follows the command (kill switch in RUN).
  if (briMotorHasEnableSense(group)) {
    bool follows = (sense_wrong_asleep == 0) && (sense_wrong_awake == 0);
    briResultBegin(follows ? BRI_PASS : BRI_FAIL);
    briText("enable_readback");
    briKVInt("high_while_asleep_rows", (long)sense_wrong_asleep);
    briKVInt("low_while_awake_rows", (long)sense_wrong_awake);
    briResultEnd();
  } else {
    briResultBegin(BRI_INFO);
    briText("enable_readback not available: the pin map has no sense pin for HIP_EN");
    briResultEnd();
  }

  // Check 2: no fault while the driver was awake.
  briResultBegin(fault_rows_awake == 0 ? BRI_PASS : BRI_FAIL);
  briText("no_fault_while_awake");
  briKVInt("fault_rows", (long)fault_rows_awake);
  briKVInt("rows", (long)settled_rows_awake);
  briResultEnd();

  // Check 3: current sense while driving.
  float cs_forward = briStatsMean(&cs_peak_stats[STEP_FORWARD]);
  float cs_reverse = briStatsMean(&cs_peak_stats[STEP_REVERSE]);
  bool too_low = (cs_forward < CS_OFFSET_MIN_MV) || (cs_reverse < CS_OFFSET_MIN_MV);
  bool too_high = (cs_forward > CS_OFFSET_MAX_MV) || (cs_reverse > CS_OFFSET_MAX_MV);
  if (too_low) {
    // True with or without a motor: a driving G2 always shows at least its offset.
    briResultBegin(BRI_FAIL);
    briText("cs_not_responding: expected about 50 mV or more while driving");
  } else if (motor_connected) {
    // Operator said a motor is connected: current flows, so the reading is
    // offset PLUS current and the offset alone cannot be read.  Report the
    // current it implies instead.
    briResultBegin(BRI_INFO);
    briText("cs_with_motor_connected: zero offset cannot be read");
    briKVFloat("implied_fwd_A", (cs_forward / 1000.0f - BRI_CS_OFFSET_V) / BRI_CS_V_PER_A, 2);
    briKVFloat("implied_rev_A", (cs_reverse / 1000.0f - BRI_CS_OFFSET_V) / BRI_CS_V_PER_A, 2);
  } else if (too_high) {
    // Operator said the motor is unplugged, yet CS shows current.
    briResultBegin(BRI_FAIL);
    briText("cs_too_high_for_no_motor: is a motor connected (then use m1), or is the output shorted?");
  } else {
    briResultBegin(BRI_PASS);
    briText("cs_zero_offset (valid because the motor is stated to be unplugged)");
  }
  briKVFloat("cs_fwd_mV", cs_forward, 1);
  briKVFloat("cs_rev_mV", cs_reverse, 1);
  briResultEnd();

  // For the record: CS and FLT while the driver is not driving.  The G2
  // documentation only promises CS while driving, so these are not judged.
  briResultBegin(BRI_INFO);
  briKVFloat("cs_asleep_mV", briStatsMean(&cs_peak_stats[0]), 1);
  briKVFloat("cs_brake_mV", briStatsMean(&cs_peak_stats[3]), 1);
  briKVInt("flt_low_rows_while_asleep", (long)fault_rows_asleep);
  briKVInt("asleep_rows", (long)settled_rows_asleep);
  briResultEnd();

  // DIR itself is a Teensy output; whether the driver output followed it can
  // only be seen with a meter (or a motor).
  briResultBegin(BRI_INFO);
  briText("dir_pin was HIGH in step 2 and LOW in step 4 (with default BRI_MOTOR_SIGN);");
  briText("a multimeter on OUTA-OUTB should show about +/- duty x Vbat");
  briResultEnd();

  briResultOverall();
}

// Make the outputs match the step that is now starting.
// Returns false if the safety layer refused or stopped the run.
static bool enterStep(uint8_t new_step) {
  step = new_step;
  step_start_ms = millis();
  if (step == 1) {
    // First awake step: this is the moment the driver is woken, and the
    // only place permission to drive is requested.
    uint32_t awake_ms = step_length_ms * 5 + 2000;   // five awake steps + margin
    if (!briSafetyGo(channel, awake_ms)) {
      return false;
    }
  }
  if (step == STEP_COUNT - 1) {
    // Last step: back to sleep.
    briSafetyStop("complete");
  }
  briSafetyDrive(STEP_DUTY[step]);
  return true;
}

// Begin a run: banner, header, clear the tallies and enter step 0.
// Step 0 is 'driver asleep', so starting a run does not yet wake anything;
// the driver is only woken when step 1 begins.
static void startRun() {
  briTelemBanner();
  Serial.print("# channel ");
  Serial.print(channel);
  Serial.print(" (");
  Serial.print(briMotorChannelName(channel));
  Serial.print(") motor_connected=");
  Serial.println(motor_connected ? 1 : 0);
  briTelemHeader(HEADER);
  briResultsReset();
  for (uint8_t i = 0; i < STEP_COUNT; i++) {
    briStatsReset(&cs_peak_stats[i]);
  }
  sense_wrong_asleep = 0;
  sense_wrong_awake = 0;
  fault_rows_awake = 0;
  fault_rows_asleep = 0;
  settled_rows_asleep = 0;
  settled_rows_awake = 0;
  last_row_ms = millis();
  running = true;
  enterStep(0);   // step 0 is "asleep": nothing is woken yet
}

// Act on one complete command line from the PC.  All operator input comes
// through here, so this is the only place the sketch changes what it is doing.
// 'g' is the explicit go for this motor test; 's' stops and sleeps the driver.
static void handleCommand(const char* command) {
  char letter = command[0];
  if (letter == '?') {
    printHelp();
  } else if (letter == 'g') {
    if (!running) {
      startRun();
    }
  } else if (letter == 's') {
    briSafetyStop("operator");
    if (running) {
      finishRun(false);
    }
  } else if (letter == 'c') {
    if (running) {
      Serial.println("# cannot change channel during a run");
      return;
    }
    int requested = (int)briCommandNumber(command, -1.0f);
    if (requested < 0 || requested >= (int)BRI_NUM_CHANNELS) {
      Serial.println("# channel must be 0, 1, 2 or 3");
      return;
    }
    channel = (uint8_t)requested;
    Serial.print("# channel set to ");
    Serial.println(briMotorChannelName(channel));
  } else if (letter == 'm') {
    motor_connected = (briCommandNumber(command, 0.0f) != 0.0f);
    Serial.print("# motor_connected set to ");
    Serial.println(motor_connected ? 1 : 0);
  } else if (letter == 't') {
    float seconds = briCommandNumber(command, DEFAULT_STEP_S);
    if (seconds < 0.5f) {
      seconds = 0.5f;
    }
    if (seconds > 10.0f) {
      seconds = 10.0f;   // 5 awake steps of 10 s stays inside BRI_MOTOR_TIMEOUT_MS
    }
    step_length_ms = (uint32_t)(seconds * 1000.0f);
  } else {
    Serial.println("# unknown command, type ? for help");
  }
}

// Runs once at power-up.  briBegin() makes the motor pins safe first
// (drivers asleep), then starts the ADC, the boot counter and USB serial.
// Nothing is measured and nothing moves until the operator sends a command.
void setup() {
  briBegin(TEST_ID, TEST_TITLE);
  step_length_ms = (uint32_t)(DEFAULT_STEP_S * 1000.0f);
  printHelp();
}

// Runs over and over and never waits.  Each pass: blink the LED, look for
// a command, run the safety checks while the driver is awake, print a row
// every 20 ms, and move to the next of the seven steps when its time is up.
void loop() {
  briHeartbeat(running);

  char command[BRI_CMD_MAX];
  if (briReadCommand(command, sizeof(command))) {
    handleCommand(command);
  }
  if (!running) {
    return;
  }

  // While the driver is awake the safety checks run every loop.
  bool awake_step = STEP_AWAKE[step];
  if (awake_step) {
    if (!briSafetyService()) {
      finishRun(false);
      return;
    }
  }

  uint32_t now_ms = millis();
  if ((now_ms - last_row_ms) >= BRI_TELEMETRY_PERIOD_MS) {
    last_row_ms = now_ms;
    uint8_t group = briMotorGroupOf(channel);

    // Current sense: while awake, reuse the safety layer's latest reading;
    // while asleep nothing else is measuring, so measure here.
    BriCurrent current;
    if (awake_step) {
      current = briSafetyLastCurrent();
    } else {
      current = briMotorReadCurrent(channel);
    }
    bool fault = briMotorFaultActive(channel);
    int sense = -1;   // -1 means "this group has no read-back pin"
    if (briMotorHasEnableSense(group)) {
      sense = briMotorEnableSense() ? 1 : 0;
    }

    // Only the second half of each step is used for the verdicts, so that
    // readings taken while things were still switching are ignored.
    bool settled = (now_ms - step_start_ms) >= (step_length_ms / 2);
    if (settled) {
      briStatsAdd(&cs_peak_stats[step], current.cs_peak_v * 1000.0f);
      if (awake_step) {
        settled_rows_awake = settled_rows_awake + 1;
        if (fault) {
          fault_rows_awake = fault_rows_awake + 1;
        }
        if (sense == 0) {
          sense_wrong_awake = sense_wrong_awake + 1;
        }
      } else {
        settled_rows_asleep = settled_rows_asleep + 1;
        if (fault) {
          fault_rows_asleep = fault_rows_asleep + 1;
        }
        if (sense == 1) {
          sense_wrong_asleep = sense_wrong_asleep + 1;
        }
      }
    }

    briRowBegin();                                      // t_ms
    briRowInt((long)step);                              // step
    briRowInt(briMotorIsAwake(group) ? 1 : 0);          // en_cmd
    briRowInt(sense);                                   // en_sense
    briRowInt((long)briMotorGetDirLevel(channel));      // dir_cmd
    briRowFloat(briMotorGetDuty(channel), 3);           // duty
    briRowInt(fault ? 0 : 1);                           // flt_ok
    briRowFloat(current.cs_peak_v * 1000.0f, 1);        // cs_peak_mV
    briRowFloat(current.cs_mean_v * 1000.0f, 1);        // cs_mean_mV
    briRowFloat(current.amps, 3);                       // i_A
    briRowFloat(briReadVbat(), 2);                      // vbat_V
    briRowEnd();
  }

  // Time to move to the next step?
  if ((now_ms - step_start_ms) >= step_length_ms) {
    if (step + 1 >= STEP_COUNT) {
      finishRun(true);
      return;
    }
    if (!enterStep((uint8_t)(step + 1))) {
      finishRun(false);
      return;
    }
  }
}
