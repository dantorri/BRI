// BRI_IMU.h - MPU-9250 / MPU-9255 / MPU-6500 / MPU-6050 over I2C.
//
// These chips are register-compatible for what the robot needs: three
// accelerometer axes, three gyro axes and a temperature.  (The MPU-9250's
// magnetometer is a separate chip inside the package and is not used.)
//
// The exact breakout is unknown, so nothing here assumes an address or a chip:
// the code looks at both possible addresses and reads the WHO_AM_I register
// to find out what is fitted.
#ifndef BRI_IMU_H
#define BRI_IMU_H

#include <Arduino.h>

// The two addresses an MPU breakout can have (set by its AD0 pin).
const uint8_t BRI_IMU_ADDR_LOW = 0x68;
const uint8_t BRI_IMU_ADDR_HIGH = 0x69;

// WHO_AM_I values: the chip's answer to "which part are you?".
const uint8_t BRI_WHO_MPU9250 = 0x71;
const uint8_t BRI_WHO_MPU9255 = 0x73;
const uint8_t BRI_WHO_MPU6500 = 0x70;
const uint8_t BRI_WHO_MPU6050 = 0x68;

// One reading of the IMU, already converted to physical units.
struct BriImuSample {
  float ax;      // acceleration along X, in g (1 g = gravity)
  float ay;
  float az;
  float gx;      // rotation rate about X, in degrees per second
  float gy;
  float gz;
  float temp_c;  // chip temperature in degrees C
  bool ok;       // false if the I2C read failed (values are then all zero)
};

// Start the I2C bus at BRI_I2C_HZ.  Call once before any other IMU function.
void briI2cBegin();

// Number of I2C transactions that failed since briI2cBegin() (or the last
// briI2cClearErrors()).  A healthy bus keeps this at zero.
uint32_t briI2cErrorCount();
// Set that failure count back to zero (done at the start of each run, so a
// run reports only its own errors).
void briI2cClearErrors();

// Probe every 7-bit address (0x08 to 0x77).  Addresses that answer are
// written to `found` (up to `found_max` of them).  Returns how many answered.
// `bus_errors` (if not null) receives the number of probes that ended with a
// bus error rather than a clean "nobody home".
uint8_t briI2cScan(uint8_t* found, uint8_t found_max, uint8_t* bus_errors);

// Read WHO_AM_I from the device at `address`.  `ok` (if not null) is set to
// false when the read failed.
uint8_t briImuWhoAmI(uint8_t address, bool* ok);

// Human name for a WHO_AM_I value ("MPU-6500", ...; "unknown" otherwise).
const char* briImuName(uint8_t who_am_i);

// True if the WHO_AM_I value is one of the four supported chips.
bool briImuKnown(uint8_t who_am_i);

// Look for an IMU at 0x68 then 0x69.  Returns its address, or 0 if neither
// address answers.
uint8_t briImuFindAddress();

// Wake the IMU at `address` and set ranges and filter from BRI_Config.h.
// Returns false if any of the configuration writes failed.
bool briImuBegin(uint8_t address);

// Read accelerometer, temperature and gyro in one go.
BriImuSample briImuRead();

// Pitch angle in degrees from the accelerometer alone (positive = leaning
// forward), using the axis constants in BRI_Config.h.  Correct only while
// the robot is not accelerating, because then gravity is the only thing the
// accelerometer feels.
float briImuAccelPitchDeg(const BriImuSample& sample);

// Pitch rate in degrees per second from the gyro (positive = tipping
// forward), using the axis constants in BRI_Config.h.
float briImuPitchRateDps(const BriImuSample& sample);

#endif  // BRI_IMU_H
