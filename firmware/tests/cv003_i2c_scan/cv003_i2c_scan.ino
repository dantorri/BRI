// CV-003  I2C bus scan
//
// What this test shows
//   * the IMU answers on the I2C bus at 0x68 or 0x69, every time it is asked
//   * the bus itself reports no errors while every address is probed
//   * which chip is fitted (from its WHO_AM_I register)
//
// What you need: Teensy + IMU breakout only (3.3 V, GND, SDA = pin 18,
// SCL = pin 19, pull-ups to 3.3 V only).  No motors.
//
// Commands
//   g        start the run
//   s        stop the run and print the results
//   t<sec>   run length in seconds (default 5 = 20 scans)
//   ?        help
#include <BRI.h>

static const char TEST_ID[] = "CV-003";
static const char TEST_TITLE[] = "I2C bus scan";
static const char HEADER[] = "t_ms,scan_n,found_count,imu_addr,who_am_i,bus_errors,imu_found";

// ---- test settings ------------------------------------------------------------
// 5 s at one scan per 250 ms = 20 scans.  "Detected repeatedly" needs more
// than one scan; 20 catches an intermittent connection without taking long.
static const float DEFAULT_RUN_S = 5.0f;
static const uint32_t SCAN_PERIOD_MS = 250;
// Room to list more devices than the robot will ever have on this bus.
static const uint8_t FOUND_MAX = 16;

// ---- run state -----------------------------------------------------------------
static bool running = false;
static uint32_t run_start_ms = 0;
static uint32_t run_length_ms = 0;
static uint32_t last_scan_ms = 0;
static uint32_t scan_count = 0;        // scans done in this run
static uint32_t imu_hit_count = 0;     // scans in which the IMU answered
static uint32_t bus_error_total = 0;   // bus errors summed over all scans
static uint8_t last_imu_address = 0;   // address of the IMU in the latest scan (0 = none)
static uint8_t last_who_am_i = 0;      // its WHO_AM_I answer
static uint8_t previous_found_count = 255;  // to print the address list only when it changes

// Print the commands this sketch understands.  Shown at boot and when the
// operator types '?', so nobody has to open the source to use the test.
static void printHelp() {
  Serial.println("# CV-003 commands:");
  briPrintCommonHelp();
  Serial.println("#   t<sec>    run length in seconds (default 5; one scan every 250 ms)");
}

// Begin a run: banner, header, start the I2C bus at 100 kHz and zero the
// counters, so every run is judged only on its own scans.
static void startRun() {
  briTelemBanner();
  briTelemHeader(HEADER);
  briResultsReset();
  briI2cBegin();
  briI2cClearErrors();
  scan_count = 0;
  imu_hit_count = 0;
  bus_error_total = 0;
  last_imu_address = 0;
  last_who_am_i = 0;
  previous_found_count = 255;
  run_start_ms = millis();
  // Subtracting the period makes the first scan happen straight away.
  last_scan_ms = run_start_ms - SCAN_PERIOD_MS;
  running = true;
}

// Probe the whole bus once and print one data row.
static void doOneScan() {
  uint8_t found[FOUND_MAX];
  uint8_t bus_errors = 0;
  uint8_t found_count = briI2cScan(found, FOUND_MAX, &bus_errors);
  scan_count = scan_count + 1;
  bus_error_total = bus_error_total + bus_errors;

  // Is one of the answers at an IMU address?
  uint8_t imu_address = 0;
  uint8_t listed = found_count;
  if (listed > FOUND_MAX) {
    listed = FOUND_MAX;
  }
  for (uint8_t i = 0; i < listed; i++) {
    if (found[i] == BRI_IMU_ADDR_LOW || found[i] == BRI_IMU_ADDR_HIGH) {
      imu_address = found[i];
    }
  }
  uint8_t who_am_i = 0;
  bool who_ok = false;
  if (imu_address != 0) {
    who_am_i = briImuWhoAmI(imu_address, &who_ok);
    if (!who_ok) {
      bus_error_total = bus_error_total + 1;  // it answered its address but not the read
    }
  }
  bool imu_found = (imu_address != 0) && who_ok;
  if (imu_found) {
    imu_hit_count = imu_hit_count + 1;
    last_imu_address = imu_address;
    last_who_am_i = who_am_i;
  }

  // Human-readable list of addresses, printed when the picture changes.
  if (found_count != previous_found_count) {
    previous_found_count = found_count;
    Serial.print("# scan ");
    Serial.print(scan_count);
    Serial.print(" found ");
    Serial.print(found_count);
    Serial.print(" device(s):");
    for (uint8_t i = 0; i < listed; i++) {
      Serial.print(" 0x");
      if (found[i] < 0x10) {
        Serial.print('0');
      }
      Serial.print(found[i], HEX);
    }
    Serial.println();
  }

  briRowBegin();                         // t_ms
  briRowInt((long)scan_count);           // scan_n
  briRowInt((long)found_count);          // found_count
  briRowInt((long)imu_address);          // imu_addr (decimal: 104 = 0x68, 105 = 0x69, 0 = none)
  briRowInt((long)who_am_i);             // who_am_i (decimal: 112 = 0x70, 113 = 0x71, ...)
  briRowInt((long)bus_error_total);      // bus_errors (running total)
  briRowInt(imu_found ? 1 : 0);          // imu_found
  briRowEnd();
}

// End a run and print the verdicts: did the IMU answer in every scan with
// no bus errors, and which chip does its WHO_AM_I value say it is.
static void finishRun() {
  running = false;

  // Check 1: the IMU answered in every scan and the bus never errored.
  bool every_scan = (scan_count > 0) && (imu_hit_count == scan_count);
  bool clean_bus = (bus_error_total == 0);
  briResultBegin((every_scan && clean_bus) ? BRI_PASS : BRI_FAIL);
  briKVInt("scans", (long)scan_count);
  briKVInt("imu_hits", (long)imu_hit_count);
  briKVInt("bus_errors", (long)bus_error_total);
  briResultEnd();

  // Check 2: which chip is it?
  if (imu_hit_count == 0) {
    briResultBegin(BRI_FAIL);
    briText("no device answered at 0x68 or 0x69: check 3.3V, GND, SDA=18, SCL=19, pull-ups");
    briResultEnd();
  } else {
    briResultBegin(briImuKnown(last_who_am_i) ? BRI_PASS : BRI_FAIL);
    briKVHex("imu_addr", last_imu_address);
    briKVHex("who_am_i", last_who_am_i);
    briKVText("chip", briImuName(last_who_am_i));
    briResultEnd();
  }
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
// a command, and during a run scan the whole bus every 250 ms until the run
// length is reached.
void loop() {
  briHeartbeat(running);

  char command[BRI_CMD_MAX];
  if (briReadCommand(command, sizeof(command))) {
    handleCommand(command);
  }
  if (!running) {
    return;
  }

  if (briEvery(&last_scan_ms, SCAN_PERIOD_MS)) {
    doOneScan();
  }
  if ((millis() - run_start_ms) >= run_length_ms) {
    finishRun();
  }
}
