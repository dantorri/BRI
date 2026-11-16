// BRI_System.cpp - ADC, battery, CPU temperature and boot counter.
#include "BRI_System.h"
#include "BRI_Config.h"
#include "BRI_Pins.h"

#include <EEPROM.h>
#include <math.h>

// tempmonGetTemp() is part of the Teensy 4 core (it reads the processor's
// built-in temperature sensor).  It is declared here as well so this file
// does not depend on which core header happens to declare it.
extern "C" float tempmonGetTemp(void);

// Where the boot counter lives in the Teensy's emulated EEPROM.
// A "magic" number sits next to it so a blank or foreign EEPROM is recognised
// and the counter starts cleanly from zero instead of from random bytes.
static const int EEPROM_ADDR_MAGIC = 0;
static const int EEPROM_ADDR_BOOTS = 4;
static const uint32_t EEPROM_MAGIC = 0x42524931;  // the letters "BRI1"

static uint32_t s_boot_count = 0;    // filled in by briSystemBegin()
static uint32_t s_reset_reason = 0;  // filled in by briSystemBegin()

// Empty a tally so a new run starts from nothing.
void briStatsReset(BriStats* stats) {
  stats->count = 0;
  stats->sum = 0.0;
  stats->sum_sq = 0.0;
  stats->min_value = 0.0f;
  stats->max_value = 0.0f;
}

// Add one value to a tally (count, sum, min, max, sum of squares).
// Keeping a tally avoids storing every sample in the Teensy's memory.
void briStatsAdd(BriStats* stats, float value) {
  // The first value sets both min and max; later values only widen them.
  if (stats->count == 0) {
    stats->min_value = value;
    stats->max_value = value;
  } else {
    if (value < stats->min_value) {
      stats->min_value = value;
    }
    if (value > stats->max_value) {
      stats->max_value = value;
    }
  }
  stats->count = stats->count + 1;
  stats->sum = stats->sum + value;
  stats->sum_sq = stats->sum_sq + (double)value * (double)value;
}

// Average of the values added so far (0 if none).
float briStatsMean(const BriStats* stats) {
  if (stats->count == 0) {
    return 0.0f;
  }
  return (float)(stats->sum / (double)stats->count);
}

// Standard deviation: how far a typical value sits from the average.
// Used to tell 'steady' from 'noisy' (for example a gyro at rest).
float briStatsStd(const BriStats* stats) {
  if (stats->count < 2) {
    return 0.0f;
  }
  double mean = stats->sum / (double)stats->count;
  double variance = stats->sum_sq / (double)stats->count - mean * mean;
  // Rounding can make a tiny variance slightly negative; sqrt would then fail.
  if (variance < 0.0) {
    variance = 0.0;
  }
  return (float)sqrt(variance);
}

// One-time board setup: ADC settings, note why the processor last reset,
// and add one to the boot counter kept in EEPROM.
void briSystemBegin() {
  // 12-bit readings: needed to resolve the small current-sense voltage.
  analogReadResolution(BRI_ADC_BITS);
  // Hardware averaging off.  Why: the current-sense reading relies on single
  // quick conversions (see briMotorReadCurrent).  Where averaging helps, the
  // library averages in software instead, so it is explicit in the code.
  analogReadAveraging(1);

  // Remember why the processor last reset (register read once, at boot).
  s_reset_reason = SRC_SRSR;

  // Boot counter: read it, add one, store it back.
  uint32_t magic = 0;
  uint32_t boots = 0;
  EEPROM.get(EEPROM_ADDR_MAGIC, magic);
  if (magic == EEPROM_MAGIC) {
    EEPROM.get(EEPROM_ADDR_BOOTS, boots);
  } else {
    // First run on this board (or EEPROM used by something else before).
    EEPROM.put(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
    boots = 0;
  }
  boots = boots + 1;
  EEPROM.put(EEPROM_ADDR_BOOTS, boots);
  s_boot_count = boots;
}

// Read an analog pin several times and return the average as volts.
// Averaging smooths out noise on slow signals such as the battery.
float briAdcReadVolts(uint8_t pin, uint8_t samples) {
  if (samples == 0) {
    samples = 1;
  }
  uint32_t total = 0;
  for (uint8_t i = 0; i < samples; i++) {
    total = total + (uint32_t)analogRead(pin);
  }
  float mean_count = (float)total / (float)samples;
  return mean_count * BRI_ADC_VREF / BRI_ADC_MAX_COUNT;
}

// Voltage at the battery-sense pin itself (after the divider).
float briReadVbatPinVolts() {
  return briAdcReadVolts(PIN_VBAT_SENSE, BRI_VBAT_SAMPLES);
}

// Battery voltage, by undoing the 33k/10k divider (multiply by 4.3).
float briReadVbat() {
  return briReadVbatPinVolts() * BRI_VBAT_DIVIDER;
}

// Processor temperature in degrees C, from the chip's built-in sensor.
float briReadCpuTempC() {
  return tempmonGetTemp();
}

// How many times this board has booted.  If it goes up during a test,
// the Teensy reset - which is what CV-002 is watching for.
uint32_t briBootCount() {
  return s_boot_count;
}

// Raw 'why did I reset' register value saved at boot, for the banner.
uint32_t briResetReasonRaw() {
  return s_reset_reason;
}
