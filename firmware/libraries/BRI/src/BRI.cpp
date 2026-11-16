// BRI.cpp - start-up sequence shared by every test sketch.
#include "BRI.h"

// Start-up for every sketch, in the one safe order: motors asleep first,
// serial last.  (BRI.h explains why the order matters.)
void briBegin(const char* test_id, const char* title) {
  // 1. Motors first: drivers asleep and PWM low before anything else happens.
  briMotorBegin();
  // 2. Safety layer starts in the "not running" state: no motion is allowed.
  briSafetyBegin();
  // 3. ADC settings and the boot counter (used for reset detection).
  briSystemBegin();
  // 4. Serial last, because it may wait a few seconds for the PC.
  briTelemBegin(test_id, title);
}
