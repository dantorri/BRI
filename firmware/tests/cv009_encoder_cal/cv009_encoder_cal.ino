// CV-009  Encoder count calibration
//
// What this test shows
//   * one full turn of the wheel gives 8400 counts, within 1 %
//   * the count goes UP when the wheel turns in the robot-forward direction
//
// Two ways to measure, both in this sketch:
//
//   A. BY HAND (no motor power needed; the drivers stay asleep)
//      1. put a tape mark on the wheel and a pointer on the chassis
//      2. type h   to start the count stream
//      3. line the mark up with the pointer, type b   ("begin")
//      4. turn the wheel exactly one turn FORWARD, mark back at the pointer
//      5. type e   ("end") - the firmware prints the counts for that turn
//      6. repeat b ... e turning BACKWARD
//      7. type s   to finish
//
//   B. BY MOTOR  (!! the motor will turn !!)
//      1. type m5  to arm a 5-turn move (negative for reverse, e.g. m-5)
//      2. type g   to GO; the motor turns until the count reaches 5 x 8400
//         and then brakes
//      3. count the turns of the tape mark yourself and type what you saw,
//         with fractions, e.g. o5.0   ("observed")
//      4. type s   to finish
//
// Commands
//   c<n>     choose channel: 0 = LW, 1 = RW, 2 = LH, 3 = RH (default 0)
//   h        start the hand-mode stream (no motion)
//   b / e    begin / end of one hand-turned revolution
//   m<revs>  arm a motor move of that many turns (0.5 to 10, sign = direction)
//   g        GO: start the armed motor move
//   o<revs>  turns you actually observed during the last motor move
//   t<sec>   stream length in seconds (default 120)
//   s        stop (brake, sleep) and print the results
//   ?        help
#include <BRI.h>

static const char TEST_ID[] = "CV-009";
static const char TEST_TITLE[] = "Encoder count calibration";
static const char HEADER[] = "t_ms,mode,counts,rev,rpm,duty,flt_ok";

// ---- test settings ------------------------------------------------------------
// Pass band: counts per revolution within 1 % of 8400 (84 counts).  One
// percent is about 3.6 degrees of wheel, which is roughly how well a tape
// mark can be lined up by eye.
static const float TOLERANCE_PCT = 1.0f;
// Two minutes is enough for several hand turns; "s" ends it sooner.
static const float DEFAULT_RUN_S = 120.0f;
// Duty for the motor-driven move.  20 % gives about 15 rpm: easy to count.
static const float MOVE_DUTY = 0.20f;
// Limits on the size of a motor move.
static const float MOVE_MIN_REVS = 0.5f;
static const float MOVE_MAX_REVS = 10.0f;
// Time limit for a motor move.  10 turns at about 15 rpm takes 40 s.
static const uint32_t MOVE_TIMEOUT_MS = 55000;
// After the target is reached the motor is braked and given this long to
// come to rest before the final count is taken.
static const uint32_t SETTLE_MS = 500;
// Room for this many hand-turn results in one run.
static const uint8_t MAX_HAND_REVS = 8;

// Values of the "mode" column.
static const int MODE_HAND = 0;      // drivers asleep, wheel turned by hand
static const int MODE_MOVING = 1;    // motor driving toward the target
static const int MODE_SETTLING = 2;  // target reached, braking

// ---- run state -----------------------------------------------------------------
static bool running = false;            // the count stream is on
static uint8_t channel = 0;
static int mode = MODE_HAND;
static uint32_t run_start_ms = 0;
static uint32_t run_length_ms = 0;
static uint32_t last_row_ms = 0;
static int32_t last_counts = 0;

// Hand-turn bookkeeping.
static bool mark_set = false;           // a "b" has been typed and not yet closed by "e"
static int32_t mark_counts = 0;         // count at the moment of "b"
static uint8_t hand_rev_count = 0;      // hand revolutions recorded
static uint8_t hand_rev_bad = 0;        // how many were outside the tolerance
static float hand_worst_pct = 0.0f;     // largest error among them (signed)
static int first_hand_sign = 0;         // +1 / -1 = count direction of the first hand turn

// Motor-move bookkeeping.
static float armed_revs = 0.0f;         // turns requested with "m" (0 = nothing armed)
static int32_t move_start_counts = 0;   // count when the move began
static int32_t move_target_counts = 0;  // how many counts to travel (always positive)
static float move_sign = 1.0f;          // +1 forward, -1 reverse
static uint32_t settle_start_ms = 0;
static bool move_done = false;          // a move finished and awaits the "o" entry
static int32_t move_counts = 0;         // counts actually travelled in the last move
static uint8_t move_cal_count = 0;      // motor calibrations recorded with "o"
static uint8_t move_cal_bad = 0;        // how many were outside the tolerance
static bool move_aborted = false;       // a move was stopped by the safety layer

// Print the commands this sketch understands.  Shown at boot and when the
// operator types '?', so nobody has to open the source to use the test.
static void printHelp() {
  Serial.println("# CV-009 commands:");
  briPrintCommonHelp();
  Serial.println("#   c<n>      channel 0=LW 1=RW 2=LH 3=RH (default 0)");
  Serial.println("#   h         start the hand-mode count stream (no motion)");
  Serial.println("#   b / e     begin / end of one revolution turned by hand");
  Serial.println("#   m<revs>   arm a motor move, 0.5..10 turns, negative = reverse  (then g)");
  Serial.println("#   o<revs>   turns you observed in the last motor move, e.g. o5.0");
  Serial.println("#   t<sec>    stream length in seconds (default 120)");
}

// Percentage error of a counts-per-revolution figure against 8400.
static float errorPercent(float counts_per_rev) {
  return (counts_per_rev - BRI_ENC_COUNTS_PER_REV) / BRI_ENC_COUNTS_PER_REV * 100.0f;
}

// End the run and print the verdicts: were the hand-turned revolutions and
// any motor moves within 1 % of 8400 counts, and which way the first hand
// turn counted (for you to compare with the way you turned it).
static void finishRun() {
  running = false;
  mode = MODE_HAND;

  // Result 1: hand-turned revolutions.
  if (hand_rev_count > 0) {
    briResultBegin(hand_rev_bad == 0 ? BRI_PASS : BRI_FAIL);
    briText("hand_revolutions");
    briKVInt("measured", (long)hand_rev_count);
    briKVInt("outside_tolerance", (long)hand_rev_bad);
    briKVFloat("worst_err_pct", hand_worst_pct, 2);
    briKVFloat("tolerance_pct", TOLERANCE_PCT, 1);
    briResultEnd();

    // Which way the count went is only meaningful with the operator's
    // knowledge of which way the wheel was turned.
    briResultBegin(BRI_TODO);
    briKVText("first_hand_turn_counted", first_hand_sign > 0 ? "positive" : "negative");
    briText("operator: must be positive if that turn was robot-FORWARD, else flip BRI_ENC_SIGN");
    briResultEnd();
  }

  // Result 2: motor-driven moves.
  if (move_aborted) {
    briResultBegin(BRI_FAIL);
    briKVText("motor_move_stopped", briSafetyStopReason());
    briResultEnd();
  }
  if (move_cal_count > 0) {
    briResultBegin(move_cal_bad == 0 ? BRI_PASS : BRI_FAIL);
    briText("motor_moves");
    briKVInt("calibrated", (long)move_cal_count);
    briKVInt("outside_tolerance", (long)move_cal_bad);
    briResultEnd();
  } else if (move_done) {
    briResultBegin(BRI_TODO);
    briKVInt("move_counts", (long)move_counts);
    briText("operator: type o<revs> with the turns you observed");
    briResultEnd();
  }

  if (hand_rev_count == 0 && move_cal_count == 0 && !move_done && !move_aborted) {
    briResultBegin(BRI_TODO);
    briText("operator: nothing measured; use b ... e by hand, or m<revs> then g");
    briResultEnd();
  }
  briResultOverall();
}

// Start the count stream in hand mode.  Nothing is woken, nothing moves.
static void startStream() {
  briTelemBanner();
  Serial.print("# channel ");
  Serial.print(channel);
  Serial.print(" (");
  Serial.print(briMotorChannelName(channel));
  Serial.println(")");
  briTelemHeader(HEADER);
  briResultsReset();
  briEncoderBegin(channel);
  briEncoderZero(channel);
  last_counts = 0;
  mode = MODE_HAND;
  mark_set = false;
  hand_rev_count = 0;
  hand_rev_bad = 0;
  hand_worst_pct = 0.0f;
  first_hand_sign = 0;
  move_done = false;
  move_cal_count = 0;
  move_cal_bad = 0;
  move_aborted = false;
  run_start_ms = millis();
  last_row_ms = run_start_ms;
  running = true;
}

// "b": remember the count at the start of a hand-turned revolution.
static void handBegin() {
  if (!running || mode != MODE_HAND) {
    Serial.println("# b only works in the hand-mode stream (type h first)");
    return;
  }
  mark_counts = briEncoderRead(channel);
  mark_set = true;
  briEventBegin("MARK");
  briText("begin");
  briKVInt("counts", (long)mark_counts);
  briEventEnd();
}

// "e": the revolution is complete; report how many counts it took.
static void handEnd() {
  if (!mark_set) {
    Serial.println("# type b at the start of the revolution first");
    return;
  }
  mark_set = false;
  int32_t delta = briEncoderRead(channel) - mark_counts;
  int32_t magnitude = delta;
  if (magnitude < 0) {
    magnitude = -magnitude;
  }
  float error_pct = errorPercent((float)magnitude);
  bool within = fabsf(error_pct) <= TOLERANCE_PCT;
  if (hand_rev_count < MAX_HAND_REVS) {
    hand_rev_count = hand_rev_count + 1;
    if (!within) {
      hand_rev_bad = hand_rev_bad + 1;
    }
    if (fabsf(error_pct) > fabsf(hand_worst_pct)) {
      hand_worst_pct = error_pct;
    }
    if (first_hand_sign == 0) {
      first_hand_sign = (delta >= 0) ? 1 : -1;
    }
  }
  briEventBegin("REV");
  briKVInt("counts", (long)delta);
  briKVFloat("err_pct", error_pct, 2);
  briKVText("direction", delta >= 0 ? "positive" : "negative");
  briKVText("within_tolerance", within ? "yes" : "no");
  briEventEnd();
}

// "g": start the armed motor move.
static void startMove() {
  if (armed_revs == 0.0f) {
    Serial.println("# nothing armed: type m<revs> first (motor move), or h for hand mode");
    return;
  }
  if (!running) {
    startStream();
  }
  if (mode != MODE_HAND) {
    Serial.println("# a move is already in progress");
    return;
  }
  move_sign = (armed_revs > 0.0f) ? 1.0f : -1.0f;
  move_target_counts = (int32_t)(fabsf(armed_revs) * BRI_ENC_COUNTS_PER_REV + 0.5f);
  armed_revs = 0.0f;   // one "m" arms one move
  move_done = false;
  if (!briSafetyGo(channel, MOVE_TIMEOUT_MS)) {
    move_aborted = true;
    return;
  }
  move_start_counts = briEncoderRead(channel);
  mode = MODE_MOVING;
  briSafetyDrive(move_sign * MOVE_DUTY);
}

// "o": the operator reports how many turns were actually seen.
static void observedRevs(float observed) {
  if (!move_done) {
    Serial.println("# no finished motor move to calibrate");
    return;
  }
  if (isnan(observed) || fabsf(observed) < 0.1f) {
    Serial.println("# o needs the number of turns, e.g. o5.0");
    return;
  }
  int32_t magnitude = move_counts;
  if (magnitude < 0) {
    magnitude = -magnitude;
  }
  float counts_per_rev = (float)magnitude / fabsf(observed);
  float error_pct = errorPercent(counts_per_rev);
  bool within = fabsf(error_pct) <= TOLERANCE_PCT;
  move_cal_count = move_cal_count + 1;
  if (!within) {
    move_cal_bad = move_cal_bad + 1;
  }
  move_done = false;
  briEventBegin("MOVECAL");
  briKVFloat("observed_rev", observed, 2);
  briKVInt("counts", (long)move_counts);
  briKVFloat("counts_per_rev", counts_per_rev, 1);
  briKVFloat("err_pct", error_pct, 2);
  briKVText("within_tolerance", within ? "yes" : "no");
  briEventEnd();
}

// Act on one complete command line from the PC.  All operator input comes
// through here, so this is the only place the sketch changes what it is doing.
// 'h' only starts the count stream; the motor can move only after 'm<revs>'
// followed by the explicit go, 'g'.
static void handleCommand(const char* command) {
  char letter = command[0];
  if (letter == '?') {
    printHelp();
  } else if (letter == 'h') {
    if (!running) {
      startStream();
    }
  } else if (letter == 'b') {
    handBegin();
  } else if (letter == 'e') {
    handEnd();
  } else if (letter == 'g') {
    startMove();
  } else if (letter == 's') {
    bool was_moving = (mode != MODE_HAND);
    briSafetyStop("operator");
    if (was_moving) {
      move_aborted = true;
    }
    if (running) {
      finishRun();
    }
  } else if (letter == 'm') {
    float requested = briCommandNumber(command, 0.0f);
    float magnitude = fabsf(requested);
    if (magnitude < MOVE_MIN_REVS || magnitude > MOVE_MAX_REVS) {
      Serial.println("# m needs 0.5 to 10 turns (negative for reverse), e.g. m5");
      return;
    }
    armed_revs = requested;
    Serial.print("# armed a motor move of ");
    Serial.print(armed_revs, 2);
    Serial.println(" turns: type g to GO");
  } else if (letter == 'o') {
    observedRevs(briCommandNumber(command, NAN));
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
  } else if (letter == 't') {
    float seconds = briCommandNumber(command, DEFAULT_RUN_S);
    if (seconds < 5.0f) {
      seconds = 5.0f;
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
// a command, during a motor move run the safety checks and watch for the
// target count, and print a row every 20 ms while the stream is on.
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

  // Motor move in progress: safety checks first, then the move logic.
  if (mode != MODE_HAND) {
    if (!briSafetyService()) {
      // The safety layer stopped the move (it has already printed why).
      mode = MODE_HAND;
      move_aborted = true;
    } else if (mode == MODE_MOVING) {
      int32_t travelled = briEncoderRead(channel) - move_start_counts;
      if (travelled < 0) {
        travelled = -travelled;
      }
      if (travelled >= move_target_counts) {
        // Target reached: brake (PWM 0 with the driver still awake) and wait.
        briSafetyDrive(0.0f);
        mode = MODE_SETTLING;
        settle_start_ms = now_ms;
      }
    } else if ((now_ms - settle_start_ms) >= SETTLE_MS) {
      // Settled: take the final count, then put the driver to sleep.
      move_counts = briEncoderRead(channel) - move_start_counts;
      briSafetyStop("complete");
      mode = MODE_HAND;
      move_done = true;
      int32_t magnitude = move_counts;
      if (magnitude < 0) {
        magnitude = -magnitude;
      }
      briEventBegin("MOVE");
      briKVFloat("target_rev", (float)move_target_counts / BRI_ENC_COUNTS_PER_REV, 2);
      briKVInt("counts", (long)move_counts);
      briKVInt("overshoot_counts", (long)(magnitude - move_target_counts));
      briEventEnd();
      Serial.println("# move finished: type o<revs> with the turns you observed");
    }
  }

  uint32_t since_row_ms = now_ms - last_row_ms;
  if (since_row_ms >= BRI_TELEMETRY_PERIOD_MS) {
    last_row_ms = now_ms;
    int32_t counts = briEncoderRead(channel);
    float rpm = briEncoderRpm(counts - last_counts, since_row_ms);
    last_counts = counts;

    briRowBegin();                                            // t_ms
    briRowInt(mode);                                          // mode
    briRowInt((long)counts);                                  // counts
    briRowFloat((float)counts / BRI_ENC_COUNTS_PER_REV, 4);   // rev
    briRowFloat(rpm, 2);                                      // rpm
    briRowFloat(briMotorGetDuty(channel), 3);                 // duty
    briRowInt(briMotorFaultActive(channel) ? 0 : 1);          // flt_ok
    briRowEnd();
  }

  // The stream ends by itself after the run length, but never in the middle
  // of a motor move (the move has its own, shorter, time limit).
  if (mode == MODE_HAND && (now_ms - run_start_ms) >= run_length_ms) {
    finishRun();
  }
}
