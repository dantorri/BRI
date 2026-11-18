// BRI_Encoder.cpp - one interface, two encoder-counting backends.
#include "BRI_Encoder.h"
#include "BRI_Pins.h"

// One counter object per channel, created only when briEncoderBegin() asks
// for it.  A null pointer means "this channel was never started".
#if defined(BRI_USE_HW_QUADENC)
static QuadEncoder* s_encoder[4] = { nullptr, nullptr, nullptr, nullptr };
#else
static Encoder* s_encoder[4] = { nullptr, nullptr, nullptr, nullptr };
#endif

// Create the counter for one channel.  Done only when a sketch asks, so a
// channel that is not wired never gets interrupts attached to floating pins.
bool briEncoderBegin(uint8_t channel) {
  if (channel >= BRI_NUM_CHANNELS) {
    return false;
  }
  if (s_encoder[channel] != nullptr) {
    return true;  // already started; nothing to do
  }
#if defined(BRI_USE_HW_QUADENC)
  // The Teensy 4 has four hardware quadrature counters, numbered 1 to 4.
  // Channel index 0..3 maps to counter 1..4, and the pin map was laid out so
  // that each channel's A/B pins are a pair that counter can use.
  // The 4th argument (0) means "no internal pull-ups": the SN74LVC245A level
  // shifter actively drives the pins.
  uint8_t hardware_counter = (uint8_t)(channel + 1);
  s_encoder[channel] = new QuadEncoder(hardware_counter, BRI_PIN_ENC_A[channel], BRI_PIN_ENC_B[channel], 0);
  s_encoder[channel]->setInitConfig();
  s_encoder[channel]->init();
#else
  // The PJRC Encoder object attaches interrupts to both pins in its
  // constructor and counts every edge from then on.
  s_encoder[channel] = new Encoder(BRI_PIN_ENC_A[channel], BRI_PIN_ENC_B[channel]);
#endif
  briEncoderZero(channel);
  return true;
}

// Count exactly as the backend reports it.  Returns 0 for a channel that was
// never started, so forgetting briEncoderBegin() cannot crash the Teensy.
int32_t briEncoderReadRaw(uint8_t channel) {
  if (channel >= BRI_NUM_CHANNELS) {
    return 0;
  }
  if (s_encoder[channel] == nullptr) {
    return 0;  // never started: report zero rather than crash
  }
  return (int32_t)s_encoder[channel]->read();
}

// Count with BRI_ENC_SIGN applied, so that positive means robot-forward
// once the sign in BRI_Config.h has been set.
int32_t briEncoderRead(uint8_t channel) {
  if (channel >= BRI_NUM_CHANNELS) {
    return 0;
  }
  return briEncoderReadRaw(channel) * (int32_t)BRI_ENC_SIGN[channel];
}

// Put one channel's count back to 0, so each run measures from its own start.
void briEncoderZero(uint8_t channel) {
  if (channel >= BRI_NUM_CHANNELS) {
    return;
  }
  if (s_encoder[channel] == nullptr) {
    return;
  }
  s_encoder[channel]->write(0);
}

// Name of the backend compiled in.  Printed in the banner so that every
// saved log records which counting method produced it.
const char* briEncoderBackendName() {
#if defined(BRI_USE_HW_QUADENC)
  return "hw_quadenc";
#else
  return "pjrc_encoder";
#endif
}

// Tells CV-008 whether digitalRead() on the encoder pins can be trusted
// with the backend in use.
bool briEncoderPinsReadable() {
#if defined(BRI_USE_HW_QUADENC)
  return false;
#else
  return true;
#endif
}

// Turn 'this many counts in this many milliseconds' into output-shaft rpm.
// Kept in one place so every test computes speed the same way.
float briEncoderRpm(int32_t delta_counts, uint32_t dt_ms) {
  if (dt_ms == 0) {
    return 0.0f;  // no time passed: avoid dividing by zero
  }
  // counts -> revolutions -> revolutions per minute
  float revolutions = (float)delta_counts / BRI_ENC_COUNTS_PER_REV;
  float minutes = (float)dt_ms / 60000.0f;
  return revolutions / minutes;
}
