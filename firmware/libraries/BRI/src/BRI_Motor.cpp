// BRI_Motor.cpp - Pololu G2 18v17 driver control.
#include "BRI_Motor.h"
#include "BRI_Config.h"
#include "BRI_Pins.h"

#include <math.h>

// What this file remembers between calls.
static bool s_group_awake[BRI_NUM_GROUPS] = { false, false };  // commanded state of each enable net
static float s_duty[4] = { 0.0f, 0.0f, 0.0f, 0.0f };           // duty last applied to each channel
static uint8_t s_dir_level[4] = { 0, 0, 0, 0 };                // level last written to each DIR pin

static const char* const CHANNEL_NAMES[4] = { "LW", "RW", "LH", "RH" };

// Which Teensy pin drives the enable net of a group.
static uint8_t enablePinOfGroup(uint8_t group) {
  if (group == BRI_GROUP_WHEELS) {
    return PIN_MOTOR_EN;
  }
  return PIN_HIP_EN;
}

// Put every motor pin into its safe state.  Called first thing at boot so
// no driver can be awake or driven by accident.
void briMotorBegin() {
  // Enable nets first, and low.  The external 10 k pull-downs already hold
  // the drivers asleep while the pins float during boot; this makes the
  // Teensy agree with them.
  pinMode(PIN_MOTOR_EN, OUTPUT);
  digitalWrite(PIN_MOTOR_EN, LOW);
  pinMode(PIN_HIP_EN, OUTPUT);
  digitalWrite(PIN_HIP_EN, LOW);
  s_group_awake[BRI_GROUP_WHEELS] = false;
  s_group_awake[BRI_GROUP_HIPS] = false;

  // The read-back pin is a plain input: the net is already driven (by the
  // Teensy through 1 k, by the pull-down, or by the kill switch).
  pinMode(PIN_MOTOR_EN_SENSE, INPUT);

  // One PWM resolution for all channels.
  analogWriteResolution(BRI_PWM_BITS);

  for (uint8_t channel = 0; channel < BRI_NUM_CHANNELS; channel++) {
    // Direction low and PWM low: if a driver were awake it would brake.
    pinMode(BRI_PIN_DIR[channel], OUTPUT);
    digitalWrite(BRI_PIN_DIR[channel], LOW);
    s_dir_level[channel] = 0;

    pinMode(BRI_PIN_PWM[channel], OUTPUT);
    digitalWrite(BRI_PIN_PWM[channel], LOW);
    analogWriteFrequency(BRI_PIN_PWM[channel], BRI_PWM_FREQ_HZ);
    analogWrite(BRI_PIN_PWM[channel], 0);
    s_duty[channel] = 0.0f;

    // FLT is open drain: the driver can only pull it low.  The pull-up
    // makes it read high when there is no fault (or no driver fitted).
    pinMode(BRI_PIN_FLT[channel], INPUT_PULLUP);
  }
}

// True for channel numbers 0 to 3.  Every function checks this first so a
// bad number can never index past the end of the pin tables.
bool briMotorChannelValid(uint8_t channel) {
  return channel < BRI_NUM_CHANNELS;
}

// Two-letter name of a channel for messages ("LW", "RW", "LH", "RH").
const char* briMotorChannelName(uint8_t channel) {
  if (!briMotorChannelValid(channel)) {
    return "??";
  }
  return CHANNEL_NAMES[channel];
}

// Which enable net a channel sits on.  Wheels and hips are separate so a
// wheel test can never wake a hip driver.
uint8_t briMotorGroupOf(uint8_t channel) {
  // Channels 0 and 1 are the wheels; 2 and 3 are the hips.
  if (channel <= BRI_CH_RW) {
    return BRI_GROUP_WHEELS;
  }
  return BRI_GROUP_HIPS;
}

// Wake or sleep one group of drivers by driving its enable net.
// Going to sleep also zeroes the group's duty so a later wake starts at rest.
void briMotorWake(uint8_t group, bool awake) {
  if (group >= BRI_NUM_GROUPS) {
    return;
  }
  if (!awake) {
    // Going to sleep: remove the drive from this group's channels first so
    // that a later wake-up cannot start with a stale duty.
    for (uint8_t channel = 0; channel < BRI_NUM_CHANNELS; channel++) {
      if (briMotorGroupOf(channel) == group) {
        analogWrite(BRI_PIN_PWM[channel], 0);
        s_duty[channel] = 0.0f;
      }
    }
  }
  if (awake) {
    digitalWrite(enablePinOfGroup(group), HIGH);
  } else {
    digitalWrite(enablePinOfGroup(group), LOW);
  }
  s_group_awake[group] = awake;
}

// What the Teensy is currently commanding on a group's enable net.
bool briMotorIsAwake(uint8_t group) {
  if (group >= BRI_NUM_GROUPS) {
    return false;
  }
  return s_group_awake[group];
}

// Apply a signed duty to one channel: DIR pin carries the sign, PWM the
// size.  This is where the duty cap and the 'asleep means zero' rule live.
void briMotorSetDuty(uint8_t channel, float duty) {
  if (!briMotorChannelValid(channel)) {
    return;
  }
  // A number that is "not a number" must never reach the PWM pin.
  if (isnan(duty)) {
    duty = 0.0f;
  }
  // A sleeping driver ignores PWM anyway; forcing 0 keeps the record honest
  // and means waking the driver later starts from rest.
  if (!s_group_awake[briMotorGroupOf(channel)]) {
    duty = 0.0f;
  }
  // The hard cap.  This is the last line of defence against a sketch bug.
  if (duty > BRI_DUTY_CAP) {
    duty = BRI_DUTY_CAP;
  }
  if (duty < -BRI_DUTY_CAP) {
    duty = -BRI_DUTY_CAP;
  }

  // Apply the per-channel sign so that positive duty means "robot forward".
  float signed_duty = duty * (float)BRI_MOTOR_SIGN[channel];
  uint8_t dir_level = 0;
  if (signed_duty >= 0.0f) {
    dir_level = 1;
  }
  float magnitude = fabsf(signed_duty);
  uint32_t pwm_count = (uint32_t)(magnitude * BRI_PWM_MAX_COUNT + 0.5f);

  // If the direction is about to change, drop PWM to zero first so the
  // driver never sees a direction flip while it is driving.
  if (dir_level != s_dir_level[channel]) {
    analogWrite(BRI_PIN_PWM[channel], 0);
    if (dir_level == 1) {
      digitalWrite(BRI_PIN_DIR[channel], HIGH);
    } else {
      digitalWrite(BRI_PIN_DIR[channel], LOW);
    }
    s_dir_level[channel] = dir_level;
  }
  analogWrite(BRI_PIN_PWM[channel], (int)pwm_count);
  s_duty[channel] = duty;
}

// The duty last applied to a channel, after the cap.  Used for the logs,
// so they show what was really output rather than what was asked for.
float briMotorGetDuty(uint8_t channel) {
  if (!briMotorChannelValid(channel)) {
    return 0.0f;
  }
  return s_duty[channel];
}

// The level last written to a channel's DIR pin, for the logs.
uint8_t briMotorGetDirLevel(uint8_t channel) {
  if (!briMotorChannelValid(channel)) {
    return 0;
  }
  return s_dir_level[channel];
}

// Set PWM to zero on every channel (brake).  First half of every stop.
void briMotorBrakeAll() {
  for (uint8_t channel = 0; channel < BRI_NUM_CHANNELS; channel++) {
    analogWrite(BRI_PIN_PWM[channel], 0);
    s_duty[channel] = 0.0f;
  }
}

// True if the driver is signalling a fault (FLT pin pulled low).
bool briMotorFaultActive(uint8_t channel) {
  if (!briMotorChannelValid(channel)) {
    return false;
  }
  // Low means fault (open-drain output pulled to ground by the driver).
  return digitalRead(BRI_PIN_FLT[channel]) == LOW;
}

// True if the group has a read-back pin for its enable net.  Only the
// wheel group does, so only there can the kill switch be detected.
bool briMotorHasEnableSense(uint8_t group) {
  return group == BRI_GROUP_WHEELS;
}

// Read the real level on the wheel enable net.  Low while commanded high
// means the kill switch is holding the net at ground (STOP).
bool briMotorEnableSense() {
  return digitalRead(PIN_MOTOR_EN_SENSE) == HIGH;
}

// Measure motor current from the driver's CS pin.  The comment inside
// explains why it takes a burst of readings instead of one.
BriCurrent briMotorReadCurrent(uint8_t channel) {
  BriCurrent result;
  result.cs_peak_v = 0.0f;
  result.cs_mean_v = 0.0f;
  result.amps = 0.0f;
  result.valid = false;
  if (!briMotorChannelValid(channel)) {
    return result;
  }

  // Why a burst of readings and not one:
  // The G2's CS pin only shows the motor current while the driver is in the
  // "on" part of the PWM cycle.  During the "off" (brake) part it does not.
  // At 20 kHz the on-time at 10 % duty is only 5 microseconds, and the ADC
  // is not synchronised to the PWM, so any single reading may land in an off
  // time.  Taking many quick readings and keeping the LARGEST picks out the
  // on-time value.  The average is kept too: it should be roughly
  // duty * peak, which is a useful cross-check in the logs.
  uint32_t total = 0;
  int peak = 0;
  for (uint8_t i = 0; i < BRI_CS_SAMPLES; i++) {
    int reading = analogRead(BRI_PIN_CS[channel]);
    total = total + (uint32_t)reading;
    if (reading > peak) {
      peak = reading;
    }
  }
  float volts_per_count = BRI_ADC_VREF / BRI_ADC_MAX_COUNT;
  result.cs_peak_v = (float)peak * volts_per_count;
  result.cs_mean_v = ((float)total / (float)BRI_CS_SAMPLES) * volts_per_count;

  // CS means something only while the driver is awake and actually driving.
  bool awake = s_group_awake[briMotorGroupOf(channel)];
  bool driving = fabsf(s_duty[channel]) > 0.0f;
  result.valid = awake && driving;
  if (result.valid) {
    float amps = (result.cs_peak_v - BRI_CS_OFFSET_V) / BRI_CS_V_PER_A;
    // Noise can put the reading a hair under the 50 mV offset; a negative
    // current would only confuse the logs.
    if (amps < 0.0f) {
      amps = 0.0f;
    }
    result.amps = amps;
  }
  return result;
}
