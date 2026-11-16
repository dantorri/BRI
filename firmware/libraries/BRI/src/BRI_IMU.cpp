// BRI_IMU.cpp - MPU-9250 / MPU-6500 family driver (accelerometer + gyro only).
#include "BRI_IMU.h"
#include "BRI_Config.h"

#include <Wire.h>
#include <math.h>

// Register addresses inside the chip (from the InvenSense register map).
static const uint8_t REG_SMPLRT_DIV = 0x19;    // sample-rate divider
static const uint8_t REG_CONFIG = 0x1A;        // gyro low-pass filter
static const uint8_t REG_GYRO_CONFIG = 0x1B;   // gyro range
static const uint8_t REG_ACCEL_CONFIG = 0x1C;  // accelerometer range
static const uint8_t REG_ACCEL_CONFIG2 = 0x1D; // accelerometer low-pass filter (not on MPU-6050)
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;  // first of 14 data bytes
static const uint8_t REG_PWR_MGMT_1 = 0x6B;    // sleep / clock source
static const uint8_t REG_WHO_AM_I = 0x75;      // chip identity

static uint8_t s_address = 0;           // address of the IMU in use (0 = none yet)
static uint8_t s_who_am_i = 0;          // identity of that IMU
static uint32_t s_i2c_errors = 0;       // failed transactions so far
static float s_accel_lsb_per_g = 8192.0f;   // raw counts per g for the chosen range
static float s_gyro_lsb_per_dps = 65.5f;    // raw counts per deg/s for the chosen range

// Start the I2C bus at the slow, tolerant speed from BRI_Config.h.
void briI2cBegin() {
  Wire.begin();
  Wire.setClock(BRI_I2C_HZ);
}

// How many I2C transactions have failed.  Tests print this so a flaky
// connection shows up as a number, not as mysterious bad data.
uint32_t briI2cErrorCount() {
  return s_i2c_errors;
}

// Zero the failure count, so each run reports only its own errors.
void briI2cClearErrors() {
  s_i2c_errors = 0;
}

// Write one byte to one register.  Returns true if the chip acknowledged.
static bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  uint8_t status = Wire.endTransmission();
  if (status != 0) {
    s_i2c_errors = s_i2c_errors + 1;
    return false;
  }
  return true;
}

// Read `count` bytes starting at register `reg` into `buffer`.
// Returns true only if every byte arrived.
static bool readRegisters(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t count) {
  // Step 1: tell the chip which register we want.  "false" keeps the bus
  // claimed (a "repeated start") so the read follows without a gap.
  Wire.beginTransmission(address);
  Wire.write(reg);
  uint8_t status = Wire.endTransmission(false);
  if (status != 0) {
    s_i2c_errors = s_i2c_errors + 1;
    return false;
  }
  // Step 2: ask for the bytes.
  uint8_t received = Wire.requestFrom(address, count);
  if (received != count) {
    s_i2c_errors = s_i2c_errors + 1;
    // Throw away whatever partial data arrived so the next read starts clean.
    while (Wire.available() > 0) {
      Wire.read();
    }
    return false;
  }
  for (uint8_t i = 0; i < count; i++) {
    buffer[i] = (uint8_t)Wire.read();
  }
  return true;
}

// Ask every legal address 'is anyone there?' and list who answers.
// This is the whole of CV-003: it finds the IMU without assuming its address.
uint8_t briI2cScan(uint8_t* found, uint8_t found_max, uint8_t* bus_errors) {
  uint8_t found_count = 0;
  uint8_t error_count = 0;
  // 0x00-0x07 and 0x78-0x7F are reserved by the I2C standard, so skip them.
  for (uint8_t address = 0x08; address <= 0x77; address++) {
    Wire.beginTransmission(address);
    uint8_t status = Wire.endTransmission();
    if (status == 0) {
      // Something acknowledged its address.
      if (found != nullptr && found_count < found_max) {
        found[found_count] = address;
      }
      found_count = found_count + 1;
    } else if (status == 2) {
      // "No acknowledge on address": simply nothing there.  Not an error.
    } else {
      // Anything else (timeout, line stuck low, lost arbitration) means the
      // bus itself misbehaved.
      error_count = error_count + 1;
      s_i2c_errors = s_i2c_errors + 1;
    }
  }
  if (bus_errors != nullptr) {
    *bus_errors = error_count;
  }
  return found_count;
}

// Read the chip's identity register.  Needed because the exact breakout
// (MPU-9250, 6500, ...) is not known in advance.
uint8_t briImuWhoAmI(uint8_t address, bool* ok) {
  uint8_t value = 0;
  bool success = readRegisters(address, REG_WHO_AM_I, &value, 1);
  if (ok != nullptr) {
    *ok = success;
  }
  return value;
}

// Turn an identity value into a readable chip name for the log.
const char* briImuName(uint8_t who_am_i) {
  if (who_am_i == BRI_WHO_MPU9250) {
    return "MPU-9250";
  }
  if (who_am_i == BRI_WHO_MPU9255) {
    return "MPU-9255";
  }
  if (who_am_i == BRI_WHO_MPU6500) {
    return "MPU-6500";
  }
  if (who_am_i == BRI_WHO_MPU6050) {
    return "MPU-6050";
  }
  return "unknown";
}

// True if the identity value is one of the four chips this code supports.
// An unknown value means a different device is sitting at the IMU address.
bool briImuKnown(uint8_t who_am_i) {
  return who_am_i == BRI_WHO_MPU9250 || who_am_i == BRI_WHO_MPU9255 ||
         who_am_i == BRI_WHO_MPU6500 || who_am_i == BRI_WHO_MPU6050;
}

// True if a device acknowledges `address`.
static bool addressAnswers(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// Try the two addresses an MPU breakout can have and return the one that
// answers (0 if neither).  Saves the user from having to know the AD0 wiring.
uint8_t briImuFindAddress() {
  if (addressAnswers(BRI_IMU_ADDR_LOW)) {
    return BRI_IMU_ADDR_LOW;
  }
  if (addressAnswers(BRI_IMU_ADDR_HIGH)) {
    return BRI_IMU_ADDR_HIGH;
  }
  return 0;
}

// Reset, wake and configure the IMU (ranges and filter from BRI_Config.h).
// The chip powers up asleep, so without this every reading would be zero.
bool briImuBegin(uint8_t address) {
  s_address = address;
  bool who_ok = false;
  s_who_am_i = briImuWhoAmI(address, &who_ok);
  if (!who_ok) {
    return false;
  }

  // Reset the chip so it starts from known settings, then give it time.
  bool ok = writeRegister(address, REG_PWR_MGMT_1, 0x80);
  delay(100);
  // Wake it (clear the sleep bit) and select the gyro-based clock, which is
  // more stable than the internal oscillator.
  ok = writeRegister(address, REG_PWR_MGMT_1, 0x01) && ok;
  delay(10);
  // Gyro low-pass filter.
  ok = writeRegister(address, REG_CONFIG, BRI_IMU_DLPF) && ok;
  // Internal sample rate 1 kHz / (1 + 4) = 200 Hz: faster than any test reads.
  ok = writeRegister(address, REG_SMPLRT_DIV, 4) && ok;
  // Ranges: the setting sits in bits 4:3 of each register.
  ok = writeRegister(address, REG_GYRO_CONFIG, (uint8_t)(BRI_IMU_GYRO_RANGE << 3)) && ok;
  ok = writeRegister(address, REG_ACCEL_CONFIG, (uint8_t)(BRI_IMU_ACCEL_RANGE << 3)) && ok;
  // The accelerometer has its own filter register on the 6500/9250 family.
  // The older MPU-6050 does not have it, so skip it there.
  if (s_who_am_i != BRI_WHO_MPU6050) {
    ok = writeRegister(address, REG_ACCEL_CONFIG2, BRI_IMU_DLPF) && ok;
  }

  // Scale factors.  Each step up in range halves the counts per unit:
  // accelerometer 16384 counts/g at +/-2 g; gyro 131 counts/(deg/s) at +/-250.
  s_accel_lsb_per_g = 16384.0f / (float)(1 << BRI_IMU_ACCEL_RANGE);
  s_gyro_lsb_per_dps = 131.0f / (float)(1 << BRI_IMU_GYRO_RANGE);
  return ok;
}

// Join two bytes (high byte first) into one signed 16-bit number.
static int16_t joinBytes(uint8_t high, uint8_t low) {
  uint16_t joined = (uint16_t)(((uint16_t)high << 8) | (uint16_t)low);
  return (int16_t)joined;
}

// Read one complete sample (acceleration, temperature, rotation) and
// convert it from raw counts to g, degrees C and degrees per second.
BriImuSample briImuRead() {
  BriImuSample sample;
  sample.ax = 0.0f;
  sample.ay = 0.0f;
  sample.az = 0.0f;
  sample.gx = 0.0f;
  sample.gy = 0.0f;
  sample.gz = 0.0f;
  sample.temp_c = 0.0f;
  sample.ok = false;
  if (s_address == 0) {
    return sample;  // briImuBegin() was never called successfully
  }

  // The 14 data bytes are: accel X, Y, Z, temperature, gyro X, Y, Z,
  // two bytes each, high byte first.  Reading them in one transaction means
  // they all belong to the same instant.
  uint8_t raw[14];
  if (!readRegisters(s_address, REG_ACCEL_XOUT_H, raw, 14)) {
    return sample;
  }
  sample.ax = (float)joinBytes(raw[0], raw[1]) / s_accel_lsb_per_g;
  sample.ay = (float)joinBytes(raw[2], raw[3]) / s_accel_lsb_per_g;
  sample.az = (float)joinBytes(raw[4], raw[5]) / s_accel_lsb_per_g;
  float temp_raw = (float)joinBytes(raw[6], raw[7]);
  if (s_who_am_i == BRI_WHO_MPU6050) {
    sample.temp_c = temp_raw / 340.0f + 36.53f;   // MPU-6050 datasheet formula
  } else {
    sample.temp_c = temp_raw / 333.87f + 21.0f;   // MPU-6500 / 9250 datasheet formula
  }
  sample.gx = (float)joinBytes(raw[8], raw[9]) / s_gyro_lsb_per_dps;
  sample.gy = (float)joinBytes(raw[10], raw[11]) / s_gyro_lsb_per_dps;
  sample.gz = (float)joinBytes(raw[12], raw[13]) / s_gyro_lsb_per_dps;
  sample.ok = true;
  return sample;
}

// Pick one of three values by axis number (0 = X, 1 = Y, 2 = Z).
static float pickAxis(uint8_t axis, float x, float y, float z) {
  if (axis == 0) {
    return x;
  }
  if (axis == 1) {
    return y;
  }
  return z;
}

// Pitch angle from gravity alone.  Slow-but-drift-free half of the
// estimate; the axis constants in BRI_Config.h say how the IMU is mounted.
float briImuAccelPitchDeg(const BriImuSample& sample) {
  // Acceleration along the robot's forward and up directions.
  float forward = pickAxis(BRI_PITCH_FWD_AXIS, sample.ax, sample.ay, sample.az) * (float)BRI_PITCH_FWD_SIGN;
  float up = pickAxis(BRI_PITCH_UP_AXIS, sample.ax, sample.ay, sample.az) * (float)BRI_PITCH_UP_SIGN;
  // Standing upright the accelerometer reads +1 g on "up" and 0 on "forward".
  // Leaning forward by an angle points the forward axis partly at the floor,
  // so it reads NEGATIVE (minus sin of the angle).  The minus sign below
  // turns that into a positive pitch for a forward lean.
  float pitch_rad = atan2f(-forward, up);
  float pitch_deg = pitch_rad * 180.0f / 3.14159265f;
  return pitch_deg - BRI_PITCH_ZERO_DEG;
}

// Pitch rate from the gyro.  Fast-but-drifting half of the estimate;
// the axis constants in BRI_Config.h pick which gyro axis is the axle.
float briImuPitchRateDps(const BriImuSample& sample) {
  return pickAxis(BRI_PITCH_GYRO_AXIS, sample.gx, sample.gy, sample.gz) * (float)BRI_PITCH_GYRO_SIGN;
}
