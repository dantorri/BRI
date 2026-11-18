// BRI_System.h - small board-level helpers: ADC, battery, temperature, boot count.
#ifndef BRI_SYSTEM_H
#define BRI_SYSTEM_H

#include <Arduino.h>

// A running tally of numbers (count, sum, smallest, largest, sum of squares).
// Why: nearly every test needs "the average and spread of this column".
// Keeping the tally as numbers arrive avoids storing every sample in memory.
struct BriStats {
  uint32_t count;   // how many values were added
  double sum;       // sum of the values (for the mean)
  double sum_sq;    // sum of the squares (for the standard deviation)
  float min_value;  // smallest value seen
  float max_value;  // largest value seen
};

// Empty the tally so a new run starts from nothing.
void briStatsReset(BriStats* stats);
// Add one value to the tally.
void briStatsAdd(BriStats* stats, float value);
// Average of the values added so far (0 if none were added).
float briStatsMean(const BriStats* stats);
// Standard deviation: how far a typical value is from the average (0 if fewer than 2 values).
float briStatsStd(const BriStats* stats);

// Set ADC resolution and averaging, then read and increment the boot counter.
// Call once at start-up (briBegin() does it).
void briSystemBegin();

// Average `samples` ADC readings of `pin` and return the voltage at the pin.
float briAdcReadVolts(uint8_t pin, uint8_t samples);

// Voltage at the VBAT_SENSE pin itself (0 to 3.3 V).
float briReadVbatPinVolts();

// Battery voltage, worked back through the 33k/10k divider.
float briReadVbat();

// Temperature of the Teensy processor in degrees C.
float briReadCpuTempC();

// How many times this board has booted since the counter was created.
// Why: if the 5 V rail dips and the Teensy resets, this number goes up, and
// the PC logger can see that even if it missed the moment of the reset.
uint32_t briBootCount();

// Raw value of the processor's "why did I reset" register, read once at boot.
// Printed in the banner for the record; its interpretation is not relied on.
uint32_t briResetReasonRaw();

#endif  // BRI_SYSTEM_H
