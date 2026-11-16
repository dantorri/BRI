// BRI_Safety.h - the rules that decide when a motor may move.
//
// The rules, in plain words:
//   1. At boot every driver is asleep and nothing moves.
//   2. A motor can only be driven between briSafetyGo() and briSafetyStop().
//      Sketches call briSafetyGo() only when the operator sends "g".
//   3. Only ONE channel is driven at a time: the one named in briSafetyGo().
//   4. While running, briSafetyService() must be called every loop.  It stops
//      everything if any of these happens:
//        - the USB serial connection is lost          (reason usb_lost)
//        - the run has lasted longer than its limit   (reason timeout)
//        - the driver's FLT pin goes low              (reason driver_fault)
//        - MOTOR_EN_SENSE reads low although the Teensy commands high,
//          which means the kill switch is in STOP     (reason kill_switch)
//        - current >= BRI_OVERCURRENT_A for BRI_OVERCURRENT_MS
//                                                     (reason over_current)
//   5. "Stop" always means: PWM to zero on all channels, then both enable
//      nets low (all drivers asleep), then a "!STOP" line with the reason.
//   6. Duty is capped at BRI_DUTY_CAP inside the motor driver code, whatever
//      a sketch asks for.
#ifndef BRI_SAFETY_H
#define BRI_SAFETY_H

#include <Arduino.h>
#include "BRI_Motor.h"

// Reset the safety layer to "not running".  Called by briBegin().
void briSafetyBegin();

// Ask for permission to run `channel` for at most `timeout_ms`.
// Wakes that channel's driver group, waits for it to come up, then checks the
// kill switch and the fault pin.  Returns true if the run may start.  On
// false, everything has been put back to sleep and a "!STOP" line printed.
bool briSafetyGo(uint8_t channel, uint32_t timeout_ms);

// Stop now: brake, sleep all drivers, print "!STOP,<t_ms>,<reason>".
// Safe to call at any time, running or not.
void briSafetyStop(const char* reason);

// True between a successful briSafetyGo() and the next stop.
bool briSafetyRunning();

// The channel given to the last briSafetyGo().
uint8_t briSafetyChannel();

// The only way a sketch sets motor drive.  Applies `duty` to the active
// channel if (and only if) a run is in progress; otherwise does nothing.
void briSafetyDrive(float duty);

// Run the checks in rule 4.  Call every loop while running.
// Returns true if the run may continue, false if it was just stopped.
bool briSafetyService();

// The most recent current measurement taken by briSafetyService(), so a
// sketch can log it without measuring a second time.
BriCurrent briSafetyLastCurrent();

// Why the last run ended ("none" before any run has stopped).
const char* briSafetyStopReason();

#endif  // BRI_SAFETY_H
