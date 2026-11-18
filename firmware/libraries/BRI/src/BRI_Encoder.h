// BRI_Encoder.h - quadrature encoder counts for the four motor channels.
//
// A quadrature encoder has two outputs, A and B, that pulse as the motor
// turns.  B is a quarter-cycle behind (or ahead of) A, and which one leads
// tells you the direction.  Counting every edge of both gives 64 counts per
// motor turn, which the 131.25:1 gearbox turns into 8400 counts per wheel turn.
//
// Two "backends" can do the counting.  The rest of the firmware does not care
// which one is used; that is the point of this wrapper.
//   default               PJRC Encoder library  (interrupts, any pin)
//   BRI_USE_HW_QUADENC    QuadEncoder library   (Teensy 4 hardware counters,
//                         channels 1-4 on pin pairs 0/1, 2/3, 7/8, 30/31)
// The switch is in BRI_Config.h.
#ifndef BRI_ENCODER_H
#define BRI_ENCODER_H

#include <Arduino.h>
#include "BRI_Config.h"

// Including the backend's header HERE (not only in the .cpp) is deliberate:
// the Arduino IDE finds the libraries a sketch needs by following #include
// lines, and this is how it learns that Encoder / QuadEncoder is required.
#if defined(BRI_USE_HW_QUADENC)
#include <QuadEncoder.h>
#else
#include <Encoder.h>
#endif

// Start counting on one channel.  Safe to call more than once.
// Returns false if the channel number is not 0 to 3.
// Why per channel: phase 1 only has the wheel encoders wired, and attaching
// interrupts to floating pins would produce random counts.
bool briEncoderBegin(uint8_t channel);

// Counts since the last zero, with BRI_ENC_SIGN applied, so positive means
// "robot forward" once the signs in BRI_Config.h have been set.
int32_t briEncoderRead(uint8_t channel);

// Counts exactly as the backend reports them (no sign applied).  Used by the
// calibration test so it can tell you which sign to set.
int32_t briEncoderReadRaw(uint8_t channel);

// Set the count of one channel back to zero.
void briEncoderZero(uint8_t channel);

// Name of the backend compiled in ("pjrc_encoder" or "hw_quadenc"), printed
// in the banner so a log records which one produced it.
const char* briEncoderBackendName();

// True if digitalRead() on the encoder pins still shows the A/B levels.
// With the hardware backend the pins are handed to the counter hardware and
// reading them as ordinary inputs is not guaranteed to work.
bool briEncoderPinsReadable();

// Turn "counts moved in dt_ms milliseconds" into output-shaft rpm.
float briEncoderRpm(int32_t delta_counts, uint32_t dt_ms);

#endif  // BRI_ENCODER_H
