// sim.h - the seam between the generic Arduino stub (sim_core.cpp) and the
// model of the BRI hardware (sim_bri_hw.cpp).
//
// sim_core.cpp knows nothing about motors or IMUs.  Whenever the firmware
// touches a pin or the I2C bus, it asks the hardware model through the
// functions below.
#ifndef BRI_SIM_H
#define BRI_SIM_H

#include <stdint.h>

// ---- provided by sim_core.cpp -------------------------------------------------
const int SIM_PIN_COUNT = 64;
extern uint8_t g_sim_pin_mode[SIM_PIN_COUNT];   // last pinMode() value
extern uint8_t g_sim_pin_out[SIM_PIN_COUNT];    // last digitalWrite() level
extern double g_sim_pwm_duty[SIM_PIN_COUNT];    // last analogWrite() as a 0..1 fraction
extern bool g_sim_usb_connected;                // what "if (Serial)" returns
// Simulated time in seconds since the pretend boot.
double simNowSeconds();                         // simulated time
// Print a message and end the simulation with an error code.  Used when
// the firmware does something the real hardware could not do.
void simAbort(const char* message);             // print and exit non-zero

// ---- provided by sim_bri_hw.cpp -----------------------------------------------
void simHwInit();
// Advance the hardware model by dt seconds.
void simHwStep(double dt_seconds);
// Level the outside world puts on an input pin (0 or 1).
int simHwDigitalIn(uint8_t pin);
// Voltage the outside world puts on an analog pin.
double simHwAnalogVolts(uint8_t pin);
// Count of the simulated encoder whose A channel is on `pin_a`.
int32_t simHwEncoderCount(uint8_t pin_a);
// A line from the scenario script that starts with '@' (text after the '@').
void simHwDirective(const char* text);
// I2C: does a device answer at this address?  Register access for it.
bool simHwI2cPresent(uint8_t address);
// Value the simulated I2C device returns for one register.
uint8_t simHwI2cReadRegister(uint8_t address, uint8_t reg);
// Store a value the firmware wrote into one register of the device.
void simHwI2cWriteRegister(uint8_t address, uint8_t reg, uint8_t value);
// Called once per millisecond of simulated time so the model can keep the
// safety bookkeeping that is printed at the end of the run.
void simHwPrintSummary();
// Tell the model that the script just sent a "go" command to the firmware.
void simHwNoteGoCommand();

#endif  // BRI_SIM_H
