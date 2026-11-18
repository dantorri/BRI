// BRI.h - the one header a test sketch includes.
//
// Why one umbrella header: a sketch should not need to know how the library
// is split into files.  "#include <BRI.h>" pulls in every module below.
//
//   BRI_Pins.h       which Teensy pin carries which signal (generated from CSV)
//   BRI_Config.h     every tunable number, each with the reason for its value
//   BRI_System.h     battery voltage, CPU temperature, boot counter, ADC helper
//   BRI_Motor.h      Pololu G2 18v17 driver: sleep, direction, PWM, fault, current
//   BRI_Encoder.h    quadrature encoder counts (two interchangeable backends)
//   BRI_IMU.h        MPU-9250 / MPU-6500 over I2C: raw data and pitch
//   BRI_Telemetry.h  the text protocol spoken to the PC logger
//   BRI_Safety.h     the rules that decide when a motor may move and when it must stop
#ifndef BRI_H
#define BRI_H

#include <Arduino.h>

// This kit uses Teensy-4-only features (20 kHz FlexPWM, tempmon, 12-bit ADC).
// A clear error here is kinder than fifty confusing errors later.
#if !defined(__IMXRT1062__)
#error "BRI firmware targets the Teensy 4.1. In the Arduino IDE choose Tools > Board > Teensy 4.1."
#endif

#include "BRI_Pins.h"
#include "BRI_Config.h"
#include "BRI_System.h"
#include "BRI_Motor.h"
#include "BRI_Encoder.h"
#include "BRI_IMU.h"
#include "BRI_Telemetry.h"
#include "BRI_Safety.h"

// briBegin() - call this first thing in setup() of every test sketch.
//
// What: puts every motor pin into its safe state (drivers asleep, PWM low),
//       sets up the ADC, bumps the boot counter, opens USB serial and prints
//       the banner.
// Why one call: the order matters.  The motor pins must be made safe BEFORE
//       the (up to 3 s) wait for the PC to open the serial port, so a sketch
//       that forgot the order could leave a driver pin floating.  Doing it in
//       one place means no sketch can get the order wrong.
void briBegin(const char* test_id, const char* title);

#endif  // BRI_H
