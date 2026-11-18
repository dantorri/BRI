// BRI_Motor.h - Pololu G2 18v17 motor driver, one per motor.
//
// How the G2 is wired here (per motor):
//   PWM  Teensy output.  High = drive the motor, low = brake (both motor leads
//        shorted together through the driver).  The fraction of time it is
//        high (the "duty") sets the average motor voltage.
//   DIR  Teensy output.  High = current flows OUTA -> OUTB, low = the reverse.
//   SLP  Sleep, active LOW.  The two wheel drivers share one net (MOTOR_EN),
//        the two hip drivers share another (HIP_EN).  Low = driver off and the
//        motor coasts.  A 10 k pull-down holds it low while the Teensy boots,
//        and the kill switch shorts it to ground.
//   FLT  Open-drain fault output, low = fault.  Read with a pull-up.
//   CS   Analog current sense: 50 mV + 20 mV per ampere, only while driving.
//
// "Sign-magnitude" drive means: DIR carries the sign, PWM carries the size.
//
// IMPORTANT: test sketches do not call briMotorSetDuty() directly.  They call
// briSafetyDrive() in BRI_Safety.h, which checks that motion is allowed first.
#ifndef BRI_MOTOR_H
#define BRI_MOTOR_H

#include <Arduino.h>

// The two enable nets.  Wheels (channels 0, 1) and hips (channels 2, 3) can
// be woken separately so a wheel test can never energise a hip motor.
const uint8_t BRI_GROUP_WHEELS = 0;
const uint8_t BRI_GROUP_HIPS = 1;
const uint8_t BRI_NUM_GROUPS = 2;

// One current measurement (see briMotorReadCurrent for the method).
struct BriCurrent {
  float cs_peak_v;  // largest CS voltage seen in the burst of readings
  float cs_mean_v;  // average CS voltage over the burst
  float amps;       // current worked out from cs_peak_v (0 if not driving)
  bool valid;       // true only if the driver was awake and PWM was non-zero
};

// Put every motor pin in its safe state: both enable nets low (drivers
// asleep), all PWM low, all DIR low, FLT pins as inputs with pull-ups.
void briMotorBegin();

// True if `channel` is a real channel number (0 to 3).
bool briMotorChannelValid(uint8_t channel);

// Two-letter name of a channel ("LW", "RW", "LH", "RH") for messages.
const char* briMotorChannelName(uint8_t channel);

// Which enable group a channel belongs to (wheels or hips).
uint8_t briMotorGroupOf(uint8_t channel);

// Wake (true) or sleep (false) one group of drivers.  Sleeping a group also
// sets the duty of its channels to zero first.
void briMotorWake(uint8_t group, bool awake);

// What the Teensy is currently commanding on a group's enable net.
bool briMotorIsAwake(uint8_t group);

// Set the drive of one channel.  `duty` runs from -1.0 (full reverse) to
// +1.0 (full forward); it is limited to +/-BRI_DUTY_CAP.  If the channel's
// group is asleep the duty is forced to 0.
void briMotorSetDuty(uint8_t channel, float duty);

// The duty last applied to a channel (after the cap and the awake check).
float briMotorGetDuty(uint8_t channel);

// Level currently commanded on a channel's DIR pin (1 = high, 0 = low).
uint8_t briMotorGetDirLevel(uint8_t channel);

// Set every channel's duty to 0 (PWM low = brake).  Drivers stay as they are.
void briMotorBrakeAll();

// True if the driver of `channel` is signalling a fault (FLT pin low).
bool briMotorFaultActive(uint8_t channel);

// True if a group has a read-back pin for its enable net.  Only the wheel
// group has one (MOTOR_EN_SENSE); the pin map has no sense pin for HIP_EN.
bool briMotorHasEnableSense(uint8_t group);

// Level actually present on the wheel enable net (MOTOR_EN_SENSE).
// Commanded awake but reading false means the kill switch is in STOP.
bool briMotorEnableSense();

// Measure the motor current of one channel through the CS pin.
BriCurrent briMotorReadCurrent(uint8_t channel);

#endif  // BRI_MOTOR_H
