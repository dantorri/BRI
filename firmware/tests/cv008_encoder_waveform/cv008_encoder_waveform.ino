// CV-008  Encoder voltage and waveform
//
// !! THE MOTOR WILL TURN (slowly).  Secure the motor and keep clear.
//
// What this test shows
//   * with the motor held at a low, constant duty, encoder channels A and B
//     both toggle at the Teensy pins
//   * the count moves in one direction only, and that direction matches the
//     sign of the duty
//   * (with your oscilloscope) the signals at the Teensy pins swing between
//     0 V and about 3.3 V, never higher, and B is a quarter-cycle from A
//
// While the motor turns, put the scope on the Teensy side of the SN74LVC245A
// (pins listed in the banner) and on the encoder side (5 V).  Type the
// highest voltage you see at the TEENSY pins with v<volts>.
//
// The firmware also samples the A and B pins itself, as fast as the loop
// runs, and counts the edges it sees.  That is a second, independent look at
// the same signals the encoder library is counting.
//
// What you need: battery, driver, motor and encoder of the chosen channel,
// the level shifter, kill switch in RUN, an oscilloscope.
//
// Commands
//   c<n>     choose channel: 0 = LW, 1 = RW, 2 = LH, 3 = RH (default 0)
//   d<duty>  constant duty, -0.35 to 0.35 (default 0.15; negative = reverse)
//   t<sec>   how long to hold it, 2 to 50 s (default 8)
//   g        GO: start turning
//   s        stop now (brake, sleep) and print the results
//   v<volts> highest voltage seen on the scope at the Teensy pins, e.g. v3.28
//   ?        help
#include <BRI.h>

static const char TEST_ID[] = "CV-008";
static const char TEST_TITLE[] = "Encoder voltage and waveform";
static const char HEADER[] = "t_ms,duty,a_level,b_level,a_edges,b_edges,bad_steps,poll_net,counts,rpm,a_high_pct,b_high_pct,flt_ok";

// ---- test settings ------------------------------------------------------------
// 15 % duty turns the output shaft at roughly 10 rpm, which puts the encoder
// edges about 0.7 ms apart: slow enough for any scope and for the loop to
// sample every edge.
static const float DEFAULT_DUTY = 0.15f;
static const float DEFAULT_RUN_S = 8.0f;
// Fewer edges than this in a whole run means "this channel is not toggling".
static const uint32_t MIN_EDGES = 20;
// Acceptable logic-high level at the Teensy pin, as read from the scope.
// The Teensy 4.1 is NOT 5 V tolerant, so anything much above its 3.3 V rail
// is a wiring fault (level shifter powered from 5 V, or bypassed).  Below
// 2.8 V the level shifter is not driving the pin properly.
static const float VHIGH_MIN_V = 2.8f;
static const float VHIGH_MAX_V = 3.5f;

// ---- run state -----------------------------------------------------------------
static bool running = false;
static uint8_t channel = 0;
static float test_duty = DEFAULT_DUTY;
static uint32_t run_start_ms = 0;
static uint32_t run_length_ms = 0;
static uint32_t last_row_ms = 0;
static int32_t last_counts = 0;
static float scope_vhigh = NAN;         // operator's scope reading (NAN = not entered)

// What the pin sampling has seen in this run.
static uint8_t last_a = 0;              // previous sampled level of A
static uint8_t last_b = 0;              // previous sampled level of B
static uint32_t a_edges = 0;            // changes seen on A
static uint32_t b_edges = 0;            // changes seen on B
static uint32_t bad_steps = 0;          // samples where A and B changed together
static int32_t poll_net = 0;            // net quadrature steps decoded from the samples
static uint32_t poll_total = 0;         // number of samples taken
static uint32_t a_high_samples = 0;     // samples with A high
static uint32_t b_high_samples = 0;     // samples with B high
static uint32_t wrong_way_rows = 0;     // rows in which the count moved against the duty
static uint32_t moving_rows = 0;        // rows in which the count moved at all
static uint32_t fault_rows = 0;

// Print the commands this sketch understands.  Shown at boot and when the
// operator types '?', so nobody has to open the source to use the test.
static void printHelp() {
  Serial.println("# CV-008 commands:   !! the motor will turn after g !!");
  briPrintCommonHelp();
  Serial.println("#   c<n>      channel 0=LW 1=RW 2=LH 3=RH (default 0)");
  Serial.println("#   d<duty>   constant duty -0.35..0.35 (default 0.15)");
  Serial.println("#   t<sec>    hold time 2..50 s (default 8)");
  Serial.println("#   v<volts>  highest scope voltage at the Teensy encoder pins, e.g. v3.28");
}

// Look at the A and B pins once and update the edge counters.
static void samplePins() {
  uint8_t a = digitalRead(BRI_PIN_ENC_A[channel]);
  uint8_t b = digitalRead(BRI_PIN_ENC_B[channel]);
  poll_total = poll_total + 1;
  if (a == HIGH) {
    a_high_samples = a_high_samples + 1;
  }
  if (b == HIGH) {
    b_high_samples = b_high_samples + 1;
  }
  bool a_changed = (a != last_a);
  bool b_changed = (b != last_b);
  if (a_changed) {
    a_edges = a_edges + 1;
  }
  if (b_changed) {
    b_edges = b_edges + 1;
  }
  if (a_changed && b_changed) {
    // In clean quadrature only one channel changes at a time.  Both changing
    // between two samples means a step was missed (sampling too slow) or the
    // signal glitched; the direction of this step cannot be known.
    bad_steps = bad_steps + 1;
  } else if (a_changed) {
    // One full cycle in one direction goes (A,B): 00 -> 10 -> 11 -> 01 -> 00.
    // In that direction, right after A changes, A and B are different.
    if (a != b) {
      poll_net = poll_net + 1;
    } else {
      poll_net = poll_net - 1;
    }
  } else if (b_changed) {
    // ...and right after B changes, A and B are the same.
    if (a == b) {
      poll_net = poll_net + 1;
    } else {
      poll_net = poll_net - 1;
    }
  }
  last_a = a;
  last_b = b;
}

// End a run and print the verdicts: do A and B both toggle, did the count
// move one way and the right way, how square the signals looked, and your
// scope reading.  `completed` is false if the run was cut short (a FAIL).
static void finishRun(bool completed) {
  running = false;

  if (!completed) {
    briResultBegin(BRI_FAIL);
    briKVText("stopped", briSafetyStopReason());
    briResultEnd();
    briResultOverall();
    return;
  }
  int32_t counts = briEncoderRead(channel);

  // Check 1: both channels toggle at the Teensy pins.
  if (!briEncoderPinsReadable()) {
    briResultBegin(BRI_TODO);
    briText("operator: pin levels are not readable with the hardware encoder backend; confirm A and B on the scope");
    briResultEnd();
  } else {
    bool both_toggle = (a_edges >= MIN_EDGES) && (b_edges >= MIN_EDGES);
    briResultBegin(both_toggle ? BRI_PASS : BRI_FAIL);
    briText("both_channels_toggle");
    briKVInt("a_edges", (long)a_edges);
    briKVInt("b_edges", (long)b_edges);
    briKVInt("min", (long)MIN_EDGES);
    briResultEnd();
  }

  // Check 2: the count went one way, and the way the duty says.
  bool sign_matches = (test_duty > 0.0f && counts > 0) || (test_duty < 0.0f && counts < 0);
  bool enough_counts = (counts >= (int32_t)MIN_EDGES) || (counts <= -(int32_t)MIN_EDGES);
  bool one_way = (wrong_way_rows == 0);
  briResultBegin((sign_matches && enough_counts && one_way) ? BRI_PASS : BRI_FAIL);
  briText("direction_consistent");
  briKVFloat("duty", test_duty, 2);
  briKVInt("counts", (long)counts);
  briKVInt("wrong_way_rows", (long)wrong_way_rows);
  briKVInt("moving_rows", (long)moving_rows);
  briResultEnd();

  // For the record: how square the signals are and how well the sampling kept up.
  float a_high_pct = 0.0f;
  float b_high_pct = 0.0f;
  if (poll_total > 0) {
    a_high_pct = 100.0f * (float)a_high_samples / (float)poll_total;
    b_high_pct = 100.0f * (float)b_high_samples / (float)poll_total;
  }
  briResultBegin(BRI_INFO);
  briKVFloat("a_high_pct", a_high_pct, 1);
  briKVFloat("b_high_pct", b_high_pct, 1);
  briKVInt("bad_steps", (long)bad_steps);
  briKVInt("poll_net", (long)poll_net);
  briKVInt("library_counts_raw", (long)briEncoderReadRaw(channel));
  briText("(about 50 pct high each, bad_steps 0, |poll_net| about |counts| is a clean quadrature signal)");
  briResultEnd();

  // Check 3: voltage at the Teensy pins, from the operator's scope.
  if (isnan(scope_vhigh)) {
    briResultBegin(BRI_TODO);
    briText("operator: scope A and B at the Teensy pins and type v<volts> (highest level seen)");
    briResultEnd();
  } else {
    bool level_ok = (scope_vhigh >= VHIGH_MIN_V) && (scope_vhigh <= VHIGH_MAX_V);
    briResultBegin(level_ok ? BRI_PASS : BRI_FAIL);
    briKVFloat("scope_vhigh_V", scope_vhigh, 2);
    briKVFloat("min", VHIGH_MIN_V, 1);
    briKVFloat("max", VHIGH_MAX_V, 1);
    briResultEnd();
  }

  briResultBegin(fault_rows == 0 ? BRI_PASS : BRI_FAIL);
  briText("no_fault");
  briKVInt("fault_rows", (long)fault_rows);
  briResultEnd();

  briResultOverall();
}

// Begin a run: banner (with the pins to scope), header, zero the encoder
// and the edge counters, then ask the safety layer for permission and apply
// the constant duty.  The motor only starts here, after the operator's 'g'.
static void startRun() {
  briTelemBanner();
  Serial.print("# channel ");
  Serial.print(channel);
  Serial.print(" (");
  Serial.print(briMotorChannelName(channel));
  Serial.print(") duty ");
  Serial.print(test_duty, 2);
  Serial.print("  scope Teensy pins A=");
  Serial.print(BRI_PIN_ENC_A[channel]);
  Serial.print(" B=");
  Serial.println(BRI_PIN_ENC_B[channel]);
  if (!briEncoderPinsReadable()) {
    Serial.println("# note: hardware encoder backend: a_level/b_level/edge columns may read 0");
  }
  briTelemHeader(HEADER);
  briResultsReset();

  briEncoderBegin(channel);
  briEncoderZero(channel);
  last_counts = 0;
  last_a = digitalRead(BRI_PIN_ENC_A[channel]);
  last_b = digitalRead(BRI_PIN_ENC_B[channel]);
  a_edges = 0;
  b_edges = 0;
  bad_steps = 0;
  poll_net = 0;
  poll_total = 0;
  a_high_samples = 0;
  b_high_samples = 0;
  wrong_way_rows = 0;
  moving_rows = 0;
  fault_rows = 0;

  running = true;
  // Time limit: the hold time plus 2 s of margin.
  if (!briSafetyGo(channel, run_length_ms + 2000)) {
    finishRun(false);
    return;
  }
  run_start_ms = millis();
  last_row_ms = run_start_ms;
  briSafetyDrive(test_duty);
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
  } else if (letter == 'd') {
    if (running) {
      Serial.println("# cannot change duty during a run");
      return;
    }
    float requested = briCommandNumber(command, DEFAULT_DUTY);
    if (requested > BRI_DUTY_CAP) {
      requested = BRI_DUTY_CAP;
    }
    if (requested < -BRI_DUTY_CAP) {
      requested = -BRI_DUTY_CAP;
    }
    if (requested == 0.0f) {
      requested = DEFAULT_DUTY;   // zero duty would make the test pointless
    }
    test_duty = requested;
    Serial.print("# duty set to ");
    Serial.println(test_duty, 2);
  } else if (letter == 't') {
    float seconds = briCommandNumber(command, DEFAULT_RUN_S);
    if (seconds < 2.0f) {
      seconds = 2.0f;
    }
    if (seconds > 50.0f) {
      seconds = 50.0f;   // stays inside BRI_MOTOR_TIMEOUT_MS with the margin
    }
    run_length_ms = (uint32_t)(seconds * 1000.0f);
  } else if (letter == 'v') {
    scope_vhigh = briCommandNumber(command, NAN);
    briEventBegin("SCOPE");
    briKVFloat("vhigh", scope_vhigh, 2);
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
// a command, and during a run: safety checks first, sample the A and B pins
// (every pass, so no edge is missed), print a row every 20 ms, stop when
// the hold time is up.
void loop() {
  briHeartbeat(running);

  char command[BRI_CMD_MAX];
  if (briReadCommand(command, sizeof(command))) {
    handleCommand(command);
  }
  if (!running) {
    return;
  }

  if (!briSafetyService()) {
    finishRun(false);
    return;
  }

  // Sample the encoder pins on every pass through loop().
  samplePins();

  uint32_t now_ms = millis();
  uint32_t since_row_ms = now_ms - last_row_ms;
  if (since_row_ms >= BRI_TELEMETRY_PERIOD_MS) {
    last_row_ms = now_ms;
    int32_t counts = briEncoderRead(channel);
    int32_t delta = counts - last_counts;
    last_counts = counts;
    float rpm = briEncoderRpm(delta, since_row_ms);
    if (delta != 0) {
      moving_rows = moving_rows + 1;
      // Did the count move against the commanded direction in this row?
      bool against = (test_duty > 0.0f && delta < 0) || (test_duty < 0.0f && delta > 0);
      if (against) {
        wrong_way_rows = wrong_way_rows + 1;
      }
    }
    bool fault = briMotorFaultActive(channel);
    if (fault) {
      fault_rows = fault_rows + 1;
    }
    float a_high_pct = 0.0f;
    float b_high_pct = 0.0f;
    if (poll_total > 0) {
      a_high_pct = 100.0f * (float)a_high_samples / (float)poll_total;
      b_high_pct = 100.0f * (float)b_high_samples / (float)poll_total;
    }

    briRowBegin();                                // t_ms
    briRowFloat(briMotorGetDuty(channel), 3);     // duty
    briRowInt((long)last_a);                      // a_level
    briRowInt((long)last_b);                      // b_level
    briRowInt((long)a_edges);                     // a_edges
    briRowInt((long)b_edges);                     // b_edges
    briRowInt((long)bad_steps);                   // bad_steps
    briRowInt((long)poll_net);                    // poll_net
    briRowInt((long)counts);                      // counts
    briRowFloat(rpm, 2);                          // rpm
    briRowFloat(a_high_pct, 1);                   // a_high_pct
    briRowFloat(b_high_pct, 1);                   // b_high_pct
    briRowInt(fault ? 0 : 1);                     // flt_ok
    briRowEnd();
  }

  if ((now_ms - run_start_ms) >= run_length_ms) {
    briSafetyStop("complete");
    finishRun(true);
  }
}
