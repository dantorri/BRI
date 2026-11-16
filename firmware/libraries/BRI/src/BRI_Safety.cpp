// BRI_Safety.cpp - motor run permission and automatic stop conditions.
#include "BRI_Safety.h"
#include "BRI_Config.h"
#include "BRI_Telemetry.h"

static bool s_running = false;            // true only between go and stop
static uint8_t s_channel = 0;             // the one channel allowed to move
static uint32_t s_start_ms = 0;           // when the run started
static uint32_t s_timeout_ms = 0;         // how long it may last
static uint32_t s_last_check_ms = 0;      // when the periodic checks last ran
static bool s_over_current = false;       // true while current is above the trip level
static uint32_t s_over_since_ms = 0;      // when it first went above
static const char* s_stop_reason = "none";
static BriCurrent s_last_current = { 0.0f, 0.0f, 0.0f, false };

// Reset to 'not running'.  Called at boot so no motion is allowed until
// a sketch explicitly asks with briSafetyGo().
void briSafetyBegin() {
  s_running = false;
  s_channel = 0;
  s_over_current = false;
  s_stop_reason = "none";
}

// The one and only way a run ends: brake everything, sleep everything,
// say why.  Safe to call at any time, running or not.
void briSafetyStop(const char* reason) {
  // Order matters: remove the drive first (brake), then put the drivers to
  // sleep.  Both steps are done for ALL channels and BOTH groups, so a stop
  // is a stop no matter which channel was running.
  briMotorBrakeAll();
  briMotorWake(BRI_GROUP_WHEELS, false);
  briMotorWake(BRI_GROUP_HIPS, false);
  s_running = false;
  s_over_current = false;
  s_stop_reason = reason;
  briEvent("STOP", reason);
}

// Ask permission to run one channel for a limited time.  Wakes only that
// channel's drivers and refuses if the kill switch is in STOP or FLT is low.
bool briSafetyGo(uint8_t channel, uint32_t timeout_ms) {
  if (!briMotorChannelValid(channel)) {
    briSafetyStop("bad_channel");
    return false;
  }
  // Never allow a run longer than the library-wide limit.
  if (timeout_ms == 0 || timeout_ms > BRI_MOTOR_TIMEOUT_MS) {
    timeout_ms = BRI_MOTOR_TIMEOUT_MS;
  }
  uint8_t group = briMotorGroupOf(channel);

  // Start from a known state: nothing driven anywhere.
  briMotorBrakeAll();
  briMotorWake(group, true);
  // Give the driver time to wake before believing its fault pin.
  delay(BRI_DRIVER_WAKE_MS);

  // Kill switch check: the Teensy is asking for "awake" (enable net high).
  // If the read-back pin still shows low, the switch is holding the net at
  // ground, i.e. it is in STOP.  (Only the wheel net has a read-back pin.)
  if (briMotorHasEnableSense(group) && !briMotorEnableSense()) {
    briSafetyStop("kill_switch");
    return false;
  }
  if (briMotorFaultActive(channel)) {
    briSafetyStop("driver_fault");
    return false;
  }

  s_channel = channel;
  s_start_ms = millis();
  s_timeout_ms = timeout_ms;
  s_last_check_ms = s_start_ms;
  s_over_current = false;
  s_running = true;
  briEventBegin("GO");
  briKVText("channel", briMotorChannelName(channel));
  briKVInt("timeout_ms", (long)timeout_ms);
  briEventEnd();
  return true;
}

// True between a successful go and the next stop.
bool briSafetyRunning() {
  return s_running;
}

// The channel the current (or last) run was allowed to drive.
uint8_t briSafetyChannel() {
  return s_channel;
}

// The only door through which a sketch drives a motor.  Does nothing
// unless a run is in progress, so a sketch bug cannot cause motion.
void briSafetyDrive(float duty) {
  if (!s_running) {
    return;  // rule 2: no go command, no motion
  }
  briMotorSetDuty(s_channel, duty);
}

// The watchdog a sketch calls every loop while a motor runs.  Stops the
// run on USB loss, time limit, driver fault, kill switch or over-current.
bool briSafetyService() {
  if (!s_running) {
    return false;
  }
  uint32_t now_ms = millis();

  // USB check, every loop.  On a Teensy "Serial" is false when no program on
  // the PC has the port open (cable pulled, logger closed or crashed).
  if (!Serial) {
    briSafetyStop("usb_lost");
    return false;
  }
  // Time limit, every loop.
  if ((now_ms - s_start_ms) >= s_timeout_ms) {
    briSafetyStop("timeout");
    return false;
  }
  // The remaining checks involve pin and ADC reads, so they run every
  // BRI_SAFETY_PERIOD_MS rather than every loop.
  if ((now_ms - s_last_check_ms) < BRI_SAFETY_PERIOD_MS) {
    return true;
  }
  s_last_check_ms = now_ms;

  if (briMotorFaultActive(s_channel)) {
    briSafetyStop("driver_fault");
    return false;
  }
  uint8_t group = briMotorGroupOf(s_channel);
  if (briMotorHasEnableSense(group) && !briMotorEnableSense()) {
    briSafetyStop("kill_switch");
    return false;
  }

  s_last_current = briMotorReadCurrent(s_channel);
  if (s_last_current.valid && s_last_current.amps >= BRI_OVERCURRENT_A) {
    if (!s_over_current) {
      // First reading above the limit: start the clock.
      s_over_current = true;
      s_over_since_ms = now_ms;
    } else if ((now_ms - s_over_since_ms) >= BRI_OVERCURRENT_MS) {
      // Above the limit for the whole trip time: stop.
      briSafetyStop("over_current");
      return false;
    }
  } else {
    // Back under the limit: forget the earlier excursion.
    s_over_current = false;
  }
  return true;
}

// The latest current reading taken by the watchdog, so a sketch can log
// it without spending time on a second measurement.
BriCurrent briSafetyLastCurrent() {
  return s_last_current;
}

// Why the last run ended, for the RESULT lines.
const char* briSafetyStopReason() {
  return s_stop_reason;
}
