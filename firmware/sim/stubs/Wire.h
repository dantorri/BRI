// Wire.h (PC STUB) - stand-in for the Teensy I2C library.
// Only the calls the BRI library makes are provided.  The "bus" has one
// simulated device on it: an MPU-family IMU (see sim_bri_hw.cpp).
#ifndef BRI_SIM_WIRE_H
#define BRI_SIM_WIRE_H

#include <Arduino.h>

class TwoWire {
 public:
  void begin();
  void setClock(uint32_t hz);
  void beginTransmission(uint8_t address);
  size_t write(uint8_t value);
  // Return codes as on the real library: 0 = ok, 2 = address not acknowledged,
  // 4 = other bus error.
  uint8_t endTransmission(uint8_t send_stop = 1);
  uint8_t requestFrom(uint8_t address, uint8_t quantity);
  int available();
  int read();

 private:
  uint8_t tx_address_ = 0;
  uint8_t tx_buffer_[32];
  uint8_t tx_length_ = 0;
  uint8_t rx_buffer_[32];
  uint8_t rx_length_ = 0;
  uint8_t rx_position_ = 0;
};
extern TwoWire Wire;

#endif  // BRI_SIM_WIRE_H
