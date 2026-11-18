// CV-007  Motor no-load test
//
// !! THE MOTOR WILL TURN.  Secure the motor, remove the wheel (or lift it
// !! clear), keep fingers and wires away, and know where the kill switch is.
//
// What this test shows
//   * the motor turns the commanded way: positive duty = robot-forward
//   * the no-load current is near the datasheet 0.2 A
//   * the driver reports no fault
//   * (if the encoder is wired) the speed is what the duty and battery
//     voltage predict
//
// The motion, on the chosen channel only:
//   ramp up to +peak (3 s), hold (2 s), ramp down (1.5 s), pause (1 s),
//   ramp up to -peak (3 s), hold (2 s), ramp down (1.5 s).   About 14 s.
//
// What you need: battery, driver and motor of the chosen channel, kill switch
// in RUN.  Encoder optional.
//
// Commands
//   c<n>     choose channel: 0 = LW, 1 = RW, 2 = LH, 3 = RH (default 0)
//   d<duty>  peak duty, 0.05 to 0.35 (default 0.30)
//   g        GO: start the motion
//   s        stop now (brake, sleep) and print the results
//   ?        help
#include <BRI.h>

static const char TEST_ID[] = "CV-007";
static const char TEST_TITLE[] = "Motor no-load test";
static const char HEADER[] = "t_ms,phase,duty,dir_cmd,cs_peak_mV,cs_mean_mV,i_A,vbat_V,counts,rpm,flt_ok,en_sense";

// ---- test settings ------------------------------------------------------------
// Peak duty of the ramp.  0.30 turns the output shaft at roughly 20 rpm:
// clearly visible, slow enough to watch, and under the 0.35 library cap.
static const float DEFAULT_PEAK_DUTY = 0.30f;
// Smallest peak duty accepted.  Below about 5 % the motor may not overcome
// its own friction, and the test would say nothing.
static const float MIN_PEAK_DUTY = 0.05f;
// Phase lengths in milliseconds.  A slow ramp avoids a current surge, the
// hold gives a steady stretch to measure, the pause lets the shaft stop
// before it is reversed.
static const uint32_t RAMP_UP_MS = 3000;
static const uint32_t HOLD_MS = 2000;
static const uint32_t RAMP_DOWN_MS = 1500;
static const uint32_t PAUSE_MS = 1000;
// "Near 0.2 A no-load" as a band.  Gearbox friction varies from motor to
// motor and with temperature, so 0.05..0.60 A is accepted; outside it the
// motor is either not connected or loaded/binding.
static const float NOLOAD_MIN_A = 0.05f;
static const float NOLOAD_MAX_A = 0.60f;
// Speed below this (output rpm) is treated as "the encoder saw no motion".
static const float MOVING_RPM = 2.0f;

// The phases, in order.  Phase 0 means "not running".
static const uint8_t PHASE_COUNT = 8;   // 0 (idle) plus 7 motion phases
static const uint32_t PHASE_MS[PHASE_COUNT] = {
  0, RAMP_UP_MS, HOLD_MS, RAMP_DOWN_MS, PAUSE_MS, RAMP_UP_MS, HOLD_MS, RAMP_DOWN_MS
};
static const uint8_t PHASE_HOLD_FORWARD = 2;
static const uint8_t PHASE_HOLD_REVERSE = 6;

// ---- run state -----------------------------------------------------------------
static bool running = false;
static uint8_t channel = 0;
static float peak_duty = DEFAULT_PEAK_DUTY;
static uint8_t phase = 0;
static uint32_t phase_start_ms = 0;
static uint32_t last_row_ms = 0;
static int32_t last_counts = 0;         // encoder count at the previous row (for rpm)
static BriStats hold_current[2];        // [0] forward hold, [1] reverse hold
static BriStats hold_rpm[2];
static BriStats hold_vbat[2];
static uint32_t fault_rows = 0;         // rows with FLT low during the run

// Print the commands this sketch understands.  Shown at boot and when the
// operator types '?', so nobody has to open the source to use the test.
static void printHelp() {
  Serial.println("# CV-007 commands:   !! the motor will turn after g !!");
  briPrintCommonHelp();
  Serial.println("#   c<n>      channel 0=LW 1=RW 2=LH 3=RH (default 0)");
  Serial.println("#   d<duty>   peak duty 0.05..0.35 (default 0.30)");
}

// Duty the profile asks for, given the phase and how far into it we are.
static float profileDuty(uint8_t which_phase, uint32_t elapsed_ms) {
  // Fraction of the way through this phase, 0.0 to 1.0.
  float fraction = 1.0f;
  if (PHASE_MS[which_phase] > 0) {
    fraction = (float)elapsed_ms / (float)PHASE_MS[which_phase];
  }
  if (fraction > 1.0f) {
    fraction = 1.0f;
  }
  if (which_phase == 1) {
    return peak_duty * fraction;             // ramp up, forward
  }
  if (which_phase == 2) {
    return peak_duty;                        // hold, forward
  }
  if (which_phase == 3) {
    return peak_duty * (1.0f - fraction);    // ramp down, forward
  }
  if (which_phase == 5) {
    return -peak_duty * fraction;            // ramp up, reverse
  }
  if (which_phase == 6) {
    return -peak_duty;                       // hold, reverse
  }
  if (which_phase == 7) {
    return -peak_duty * (1.0f - fraction);   // ramp down, reverse
  }
  return 0.0f;                               // idle and pause
}

// End a run and print the verdicts: direction (from the encoder), no-load
// current in both directions, faults, and measured against predicted speed.
// `completed` is false if the run was cut short, which is itself a FAIL.
static void finishRun(bool completed) {
  running = false;
  phase = 0;

  if (!completed) {
    briResultBegin(BRI_FAIL);
    briKVText("stopped", briSafetyStopReason());
    briResultEnd();
    briResultOverall();
    return;
  }

  float rpm_forward = briStatsMean(&hold_rpm[0]);
  float rpm_reverse = briStatsMean(&hold_rpm[1]);

  // Check 1: direction.
  bool forward_seen = rpm_forward > MOVING_RPM;
  bool reverse_seen = rpm_reverse < -MOVING_RPM;
  bool no_motion = (fabsf(rpm_forward) <= MOVING_RPM) && (fabsf(rpm_reverse) <= MOVING_RPM);
  if (forward_seen && reverse_seen) {
    briResultBegin(BRI_PASS);
    briText("direction: encoder count rises on positive duty and falls on negative duty");
  } else if (no_motion) {
    briResultBegin(BRI_TODO);
    briText("operator: encoder saw no motion (not wired?); confirm by eye that + duty turned the wheel forward");
  } else {
    briResultBegin(BRI_FAIL);
    briText("direction: encoder disagrees with the command; if the WHEEL turned backward on + duty flip");
    briText("BRI_MOTOR_SIGN, if the wheel was right but the count was not flip BRI_ENC_SIGN (see CV-009)");
  }
  briKVFloat("rpm_fwd", rpm_forward, 1);
  briKVFloat("rpm_rev", rpm_reverse, 1);
  briResultEnd();

  // Check 2: no-load current in both directions.
  float amps_forward = briStatsMean(&hold_current[0]);
  float amps_reverse = briStatsMean(&hold_current[1]);
  bool current_ok = (amps_forward >= NOLOAD_MIN_A) && (amps_forward <= NOLOAD_MAX_A) &&
                    (amps_reverse >= NOLOAD_MIN_A) && (amps_reverse <= NOLOAD_MAX_A);
  briResultBegin(current_ok ? BRI_PASS : BRI_FAIL);
  briKVFloat("i_fwd_A", amps_forward, 2);
  briKVFloat("i_rev_A", amps_reverse, 2);
  briKVFloat("min", NOLOAD_MIN_A, 2);
  briKVFloat("max", NOLOAD_MAX_A, 2);
  briResultEnd();

  // Check 3: no fault.
  briResultBegin(fault_rows == 0 ? BRI_PASS : BRI_FAIL);
  briText("no_fault");
  briKVInt("fault_rows", (long)fault_rows);
  briResultEnd();

  // For the record: measured speed against the simple prediction
  // rpm = duty x 76 rpm x (Vbat / 12 V).  Friction makes the real speed a
  // little lower, so this is reported, not judged.
  float vbat = briStatsMean(&hold_vbat[0]);
  float predicted_rpm = peak_duty * BRI_MOTOR_NOLOAD_RPM * vbat / BRI_MOTOR_RATED_V;
  briResultBegin(BRI_INFO);
  briKVFloat("peak_duty", peak_duty, 2);
  briKVFloat("vbat_V", vbat, 2);
  briKVFloat("predicted_rpm", predicted_rpm, 1);
  briKVFloat("measured_rpm", rpm_forward, 1);
  briResultEnd();

  briResultOverall();
}

// Begin a run: banner, header, clear the statistics, start the encoder,
// then ask the safety layer for permission to drive.  This is the only
// place the motor is allowed to start, and only after the operator sent 'g'.
static void startRun() {
  briTelemBanner();
  Serial.print("# channel ");
  Serial.print(channel);
  Serial.print(" (");
  Serial.print(briMotorChannelName(channel));
  Serial.print(") peak_duty ");
  Serial.println(peak_duty, 2);
  briTelemHeader(HEADER);
  briResultsReset();
  for (uint8_t i = 0; i < 2; i++) {
    briStatsReset(&hold_current[i]);
    briStatsReset(&hold_rpm[i]);
    briStatsReset(&hold_vbat[i]);
  }
  fault_rows = 0;

  // The encoder is started even if it is not wired; it then just reads 0.
  briEncoderBegin(channel);
  briEncoderZero(channel);
  last_counts = 0;

  // Time limit: the whole profile plus 3 s of margin.
  uint32_t total_ms = 0;
  for (uint8_t i = 0; i < PHASE_COUNT; i++) {
    total_ms = total_ms + PHASE_MS[i];
  }
  running = true;
  if (!briSafetyGo(channel, total_ms + 3000)) {
    finishRun(false);
    return;
  }
  phase = 1;
  phase_start_ms = millis();
  last_row_ms = phase_start_ms;
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
    float requested = fabsf(briCommandNumber(command, DEFAULT_PEAK_DUTY));
    if (requested < MIN_PEAK_DUTY) {
      requested = MIN_PEAK_DUTY;
    }
    if (requested > BRI_DUTY_CAP) {
      requested = BRI_DUTY_CAP;
    }
    peak_duty = requested;
    Serial.print("# peak duty set to ");
    Serial.println(peak_duty, 2);
  } else {
    Serial.println("# unknown command, type ? for help");
  }
}

// Runs once at power-up.  briBegin() makes the motor pins safe first
// (drivers asleep), then starts the ADC, the boot counter and USB serial.
// Nothing is measured and nothing moves until the operator sends a command.
void setup() {
  briBegin(TEST_ID, TEST_TITLE);
  printHelp();
}

// Runs over and over and never waits.  Each pass: blink the LED, look for
// a command, and during a run: safety checks first, then set the duty the
// ramp profile asks for, print a row every 20 ms, and step through the phases.
void loop() {
  briHeartbeat(running);

  char command[BRI_CMD_MAX];
  if (briReadCommand(command, sizeof(command))) {
    handleCommand(command);
  }
  if (!running) {
    return;
  }

  // Safety first, every loop.
  if (!briSafetyService()) {
    finishRun(false);
    return;
  }

  uint32_t now_ms = millis();
  uint32_t in_phase_ms = now_ms - phase_start_ms;

  // Follow the profile.
  briSafetyDrive(profileDuty(phase, in_phase_ms));

  uint32_t since_row_ms = now_ms - last_row_ms;
  if (since_row_ms >= BRI_TELEMETRY_PERIOD_MS) {
    last_row_ms = now_ms;
    BriCurrent current = briSafetyLastCurrent();
    float vbat = briReadVbat();
    int32_t counts = briEncoderRead(channel);
    float rpm = briEncoderRpm(counts - last_counts, since_row_ms);
    last_counts = counts;
    bool fault = briMotorFaultActive(channel);
    if (fault) {
      fault_rows = fault_rows + 1;
    }
    int sense = -1;
    if (briMotorHasEnableSense(briMotorGroupOf(channel))) {
      sense = briMotorEnableSense() ? 1 : 0;
    }

    // Statistics come from the second half of each hold, when speed and
    // current have settled.
    bool second_half = in_phase_ms >= (HOLD_MS / 2);
    if (phase == PHASE_HOLD_FORWARD && second_half) {
      briStatsAdd(&hold_current[0], current.amps);
      briStatsAdd(&hold_rpm[0], rpm);
      briStatsAdd(&hold_vbat[0], vbat);
    }
    if (phase == PHASE_HOLD_REVERSE && second_half) {
      briStatsAdd(&hold_current[1], current.amps);
      briStatsAdd(&hold_rpm[1], rpm);
      briStatsAdd(&hold_vbat[1], vbat);
    }

    briRowBegin();                                      // t_ms
    briRowInt((long)phase);                             // phase
    briRowFloat(briMotorGetDuty(channel), 3);           // duty
    briRowInt((long)briMotorGetDirLevel(channel));      // dir_cmd
    briRowFloat(current.cs_peak_v * 1000.0f, 1);        // cs_peak_mV
    briRowFloat(current.cs_mean_v * 1000.0f, 1);        // cs_mean_mV
    briRowFloat(current.amps, 3);                       // i_A
    briRowFloat(vbat, 2);                               // vbat_V
    briRowInt((long)counts);                            // counts
    briRowFloat(rpm, 2);                                // rpm
    briRowInt(fault ? 0 : 1);                           // flt_ok
    briRowInt(sense);                                   // en_sense
    briRowEnd();
  }

  // Next phase?
  if (in_phase_ms >= PHASE_MS[phase]) {
    if (phase + 1 >= PHASE_COUNT) {
      briSafetyStop("complete");
      finishRun(true);
      return;
    }
    phase = (uint8_t)(phase + 1);
    phase_start_ms = now_ms;
  }
}
