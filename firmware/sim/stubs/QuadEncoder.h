// QuadEncoder.h (PC STUB) - stand-in for the Teensy 4 hardware quadrature
// decoder library.  The constructor checks that the channel number and pins
// form one of the pairs the BRI pin map relies on (1: 0/1, 2: 2/3, 3: 7/8,
// 4: 30/31) and aborts the simulation if they do not, so a wrong table in
// the firmware shows up as a failed stub run.
#ifndef BRI_SIM_QUADENCODER_H
#define BRI_SIM_QUADENCODER_H

#include <Arduino.h>

class QuadEncoder {
 public:
  QuadEncoder(uint8_t encoder_ch = 255, uint8_t phase_a_pin = 255, uint8_t phase_b_pin = 255,
              uint8_t pin_pus = 0, uint8_t index_pin = 255, uint8_t home_pin = 255,
              uint8_t trigger_pin = 255);
  void setInitConfig();
  void init();
  int32_t read();
  void write(int32_t value);

 private:
  uint8_t pin_a_;
  bool initialised_ = false;
  int32_t offset_ = 0;
  int32_t base_ = 0;
};

#endif  // BRI_SIM_QUADENCODER_H
