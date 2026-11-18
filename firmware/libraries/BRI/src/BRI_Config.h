// BRI_Config.h - every tunable number in one place, each with its reason.
//
// How to read this file: each constant has a comment that says WHAT it is and
// WHY it has that value.  If a bench test tells you a value is wrong for your
// robot (for example a motor runs backwards), this is the file you change.
//
// These are "const" values, not "#define", so the compiler checks their type.
#ifndef BRI_CONFIG_H
#define BRI_CONFIG_H

#include <stdint.h>

// Version text printed in every banner, so a saved log says which firmware
// produced it.
#define BRI_FW_VERSION "0.1.0"

// ---------------------------------------------------------------------------
// Encoder backend switch
// ---------------------------------------------------------------------------
// Default (line below stays commented out): the PJRC "Encoder" library.  It
// uses pin-change interrupts, works on any pin, and leaves the pins readable
// with digitalRead(), which CV-008 needs.
//
// Uncomment the line to use the "QuadEncoder" library instead.  That one uses
// the Teensy 4 hardware quadrature decoders (no interrupts, cannot miss
// counts at high speed) but only works on the pin pairs 0/1, 2/3, 7/8, 30/31,
// which is exactly why the pin map puts the encoders there.
//
// Why the switch lives in this file and not in the sketch: the Arduino IDE
// compiles library .cpp files separately from the sketch, so a #define placed
// in a sketch would not reach BRI_Encoder.cpp.
//
// #define BRI_USE_HW_QUADENC

// ---------------------------------------------------------------------------
// Serial / telemetry
// ---------------------------------------------------------------------------
// Baud rate handed to Serial.begin().  Teensy USB serial ignores it (USB is
// always full speed) but the call still needs a number, and 115200 is what
// the PC logger opens the port with.
const uint32_t BRI_SERIAL_BAUD = 115200;

// How long setup() waits for the PC to open the port before carrying on.
// Why wait at all: text printed before the port is open is lost.  Why not
// wait forever: the board must still boot (and blink) with no PC attached.
const uint32_t BRI_SERIAL_WAIT_MS = 3000;

// Time between telemetry rows.  20 ms = 50 rows per second: fast enough to
// see a motor ramp or a hand tilt, slow enough to read and to keep log files
// small.
const uint32_t BRI_TELEMETRY_PERIOD_MS = 20;

// Longest command line accepted from the PC, including the end marker.
// Commands are a letter plus a short number, so 32 characters is generous.
const uint8_t BRI_CMD_MAX = 32;

// ---------------------------------------------------------------------------
// Analog inputs
// ---------------------------------------------------------------------------
// ADC resolution in bits.  12 bits gives 4096 steps over 3.3 V (0.8 mV per
// step), which is needed to see the 20 mV/A current-sense signal.
const uint8_t BRI_ADC_BITS = 12;

// Largest ADC reading at that resolution (2^12 - 1).
const float BRI_ADC_MAX_COUNT = 4095.0f;

// ADC reference voltage.  The Teensy 4.1 measures against its own 3.3 V rail.
const float BRI_ADC_VREF = 3.3f;

// ---------------------------------------------------------------------------
// Battery sense
// ---------------------------------------------------------------------------
// The battery reaches pin 20 through a 33 k / 10 k divider, so the pin sees
// Vbat * 10 / (33 + 10).  Multiply the pin voltage by 4.3 to get Vbat.
// Why a divider: 12.6 V on a 3.3 V pin would destroy the Teensy.
const float BRI_VBAT_DIVIDER = 4.3f;

// Number of ADC readings averaged for one battery measurement.  Averaging
// removes the ripple the motors put on the battery line.
const uint8_t BRI_VBAT_SAMPLES = 16;

// Below this the battery is treated as "not connected" (USB-only bench work)
// rather than "flat".  A 3S pack can never legitimately be this low.
const float BRI_VBAT_PRESENT_V = 6.0f;

// A 3S LiPo should not be run below about 3.3 V per cell (9.9 V).  Tests
// print a warning under this value; they do not stop, because the bench
// supply may be set lower on purpose.
const float BRI_VBAT_LOW_V = 9.9f;

// ---------------------------------------------------------------------------
// Motor driver (Pololu G2 18v17)
// ---------------------------------------------------------------------------
// PWM frequency.  20 kHz is just above human hearing, so the motor does not
// whine, and is well inside what the G2 driver accepts.
const float BRI_PWM_FREQ_HZ = 20000.0f;

// PWM resolution in bits.  At 20 kHz the Teensy 4.1 timer has about 7500
// steps per period, so 12 bits (4096 steps) is the largest round number that
// is still real resolution.
const uint8_t BRI_PWM_BITS = 12;

// PWM value that means "always on" at that resolution (2^12 - 1).
const float BRI_PWM_MAX_COUNT = 4095.0f;

// Hard limit on duty cycle for the bench tests, as a fraction (0.35 = 35 %).
// Why so low: these tests check wiring and direction, not performance.  At
// 35 % a stalled motor draws roughly 0.35 * 5.5 A = 1.9 A, which the wiring
// and the driver tolerate while you reach for the kill switch.
// The library refuses to output more than this no matter what a sketch asks.
const float BRI_DUTY_CAP = 0.35f;

// Time to wait after waking a driver before trusting FLT or driving PWM.
// The G2 needs a moment for its gate supply to come up after SLP goes high.
const uint32_t BRI_DRIVER_WAKE_MS = 10;

// Current-sense output of the G2: 50 mV when no current flows, plus 20 mV
// for every ampere.  (Only valid while the driver is actively driving; see
// briMotorReadCurrent() for what that means for the measurement.)
const float BRI_CS_OFFSET_V = 0.050f;
const float BRI_CS_V_PER_A = 0.020f;

// How many ADC readings are taken for one current measurement.  The CS pin
// only shows the current during the "on" part of each PWM cycle, so the
// library takes many quick readings and keeps the largest.  With 64 readings
// at 10 % duty the chance that none of them lands in an "on" time is about
// 0.1 %.
const uint8_t BRI_CS_SAMPLES = 64;

// Software over-current trip: if the measured current stays at or above
// BRI_OVERCURRENT_A for BRI_OVERCURRENT_MS, the safety layer stops the motor.
// Why 3.0 A: well above the 0.2 A no-load current and above a brief start-up
// surge, well below the 5.5 A stall current.  Why 200 ms: long enough to
// ignore a start-up spike, short enough to protect a stalled motor.
const float BRI_OVERCURRENT_A = 3.0f;
const uint32_t BRI_OVERCURRENT_MS = 200;

// How often the safety layer re-checks fault, kill switch and current while
// a motor runs.  5 ms means a problem is seen within a few PWM-visible
// moments, without spending all the CPU time on ADC readings.
const uint32_t BRI_SAFETY_PERIOD_MS = 5;

// Default time limit for any motor run.  A sketch passes its own limit to
// briSafetyGo(); this is the fallback and the upper bound.
// Why a limit at all: if the PC script hangs with the port still open, the
// motor must still stop by itself.
const uint32_t BRI_MOTOR_TIMEOUT_MS = 60000;

// Sign that makes "positive duty" mean "robot drives forward" for each
// channel (index 0 = LW, 1 = RW, 2 = LH, 3 = RH).  The two wheel motors are
// mounted mirror-image, so one of them will need -1.  Which one is not known
// until CV-007 is run on the real robot: start with +1 and flip the one that
// turns the wrong way.
const int8_t BRI_MOTOR_SIGN[4] = { +1, +1, +1, +1 };

// Sign that makes "positive counts" mean "robot drives forward" for each
// encoder.  Same story as BRI_MOTOR_SIGN; CV-009 tells you which to flip.
const int8_t BRI_ENC_SIGN[4] = { +1, +1, +1, +1 };

// ---------------------------------------------------------------------------
// Motor and encoder data (Pololu 4756, 37D 131.25:1, 12 V, 64 CPR encoder)
// ---------------------------------------------------------------------------
// Encoder counts for one turn of the OUTPUT shaft: 64 counts per motor turn
// (all four edges of A and B) times the 131.25:1 gearbox = 8400.
const float BRI_ENC_COUNTS_PER_REV = 8400.0f;

// Datasheet values at 12 V, used by the tests to judge "is this plausible".
const float BRI_MOTOR_RATED_V = 12.0f;
const float BRI_MOTOR_NOLOAD_RPM = 76.0f;
const float BRI_MOTOR_NOLOAD_A = 0.2f;
const float BRI_MOTOR_STALL_A = 5.5f;

// ---------------------------------------------------------------------------
// IMU (MPU-9250 or MPU-6500 on Wire, pins 18/19)
// ---------------------------------------------------------------------------
// I2C clock used for everything in these tests.  100 kHz is the slowest
// standard speed and the most tolerant of long wires and weak pull-ups; the
// workbook asks for a "conservative clock speed" for the bus scan.
const uint32_t BRI_I2C_HZ = 100000;

// Accelerometer range setting (0 = +/-2 g, 1 = +/-4 g, 2 = +/-8 g, 3 = +/-16 g).
// +/-4 g leaves headroom for bumps while keeping good resolution near 1 g.
const uint8_t BRI_IMU_ACCEL_RANGE = 1;

// Gyro range setting (0 = +/-250, 1 = +/-500, 2 = +/-1000, 3 = +/-2000 deg/s).
// A falling robot rotates at well under 500 deg/s.
const uint8_t BRI_IMU_GYRO_RANGE = 1;

// Digital low-pass filter setting inside the IMU (3 = about 41 Hz).  This
// removes motor vibration that would otherwise look like tilt.
const uint8_t BRI_IMU_DLPF = 3;

// ---- Which way the IMU is mounted -------------------------------------------
// The pitch calculation needs to know which sensor axis points FORWARD, which
// points UP, and which axis the robot ROTATES AROUND when it tips (the wheel
// axle direction).  Axis numbers: 0 = X, 1 = Y, 2 = Z.  The sign is +1 if the
// sensor axis points the stated way, -1 if it points the opposite way.
// These defaults assume X forward, Z up, Y along the axle.  CV-004 shows
// which axis reads +1 g (that one points up); CV-005 confirms the signs.
const uint8_t BRI_PITCH_FWD_AXIS = 0;
const int8_t BRI_PITCH_FWD_SIGN = +1;
const uint8_t BRI_PITCH_UP_AXIS = 2;
const int8_t BRI_PITCH_UP_SIGN = +1;
const uint8_t BRI_PITCH_GYRO_AXIS = 1;
const int8_t BRI_PITCH_GYRO_SIGN = +1;

// Angle (degrees) subtracted from the accelerometer pitch so that "robot
// standing balanced" reads 0.  Leave at 0 until CV-005 reports the zero.
const float BRI_PITCH_ZERO_DEG = 0.0f;

// Complementary filter time constant in seconds.  The filter trusts the gyro
// for changes faster than this and the accelerometer for anything slower.
// 0.5 s is a common starting point for a balancing robot.
const float BRI_COMP_FILTER_TAU_S = 0.5f;

#endif  // BRI_CONFIG_H
