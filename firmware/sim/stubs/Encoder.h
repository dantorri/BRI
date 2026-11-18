// Encoder.h (PC STUB) - stand-in for the PJRC Encoder library.
// read() returns the count of the simulated encoder wired to the given pins.
#ifndef BRI_SIM_ENCODER_H
#define BRI_SIM_ENCODER_H

#include <Arduino.h>

class Encoder {
 public:
  Encoder(uint8_t pin_a, uint8_t pin_b);
  int32_t read();
  void write(int32_t position);

 private:
  uint8_t pin_a_;
  int32_t offset_ = 0;  // simulated count at the moment of the last write()
  int32_t base_ = 0;    // value given to the last write()
};

#endif  // BRI_SIM_ENCODER_H
