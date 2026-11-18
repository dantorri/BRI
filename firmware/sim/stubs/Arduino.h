// Arduino.h (PC STUB) - a small stand-in for the Teensy 4.1 Arduino core.
//
// THIS IS NOT THE REAL TEENSY CORE.  It exists so the BRI library and test
// sketches can be compiled and run on a PC, with simulated hardware behind
// the pin functions.  It declares only the functions the BRI firmware uses,
// with the same names and argument order as the Teensy core.  A sketch that
// builds against this file has had its C++ checked; it has NOT been proven to
// build or behave correctly on a real Teensy.
#ifndef BRI_SIM_ARDUINO_H
#define BRI_SIM_ARDUINO_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Pretend to be a Teensy 4.1 so the "#if defined(__IMXRT1062__)" guard in
// BRI.h is satisfied.  BRI_SIM marks the build as a simulation.
#ifndef __IMXRT1062__
#define __IMXRT1062__ 1
#endif
#define ARDUINO_TEENSY41 1
#define TEENSYDUINO 159
#define BRI_SIM 1

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3
#define LED_BUILTIN 13
#define DEC 10
#define HEX 16

typedef uint8_t byte;
typedef bool boolean;

// ---- pins -------------------------------------------------------------------
void pinMode(uint8_t pin, uint8_t mode);
// Drive an output pin high or low.
void digitalWrite(uint8_t pin, uint8_t value);
// Read the level on a pin.
uint8_t digitalRead(uint8_t pin);
// Read an analog pin as ADC counts.
int analogRead(uint8_t pin);
// Set the PWM value of a pin.
void analogWrite(uint8_t pin, int value);
// Set the PWM frequency of a pin (Teensy extension).
void analogWriteFrequency(uint8_t pin, float frequency);
// Set how many bits analogWrite() values have (Teensy extension).
uint32_t analogWriteResolution(uint32_t bits);
// Set how many bits analogRead() returns.
void analogReadResolution(unsigned int bits);
// Set the ADC's built-in averaging (Teensy extension).
void analogReadAveraging(unsigned int count);

// ---- time (simulated, not wall-clock) ---------------------------------------
uint32_t millis();
// Microseconds since boot.
uint32_t micros();
// Wait this many milliseconds.
void delay(uint32_t ms);
// Wait this many microseconds.
void delayMicroseconds(uint32_t us);

// ---- Teensy-4-specific items the BRI library touches --------------------------
extern "C" float tempmonGetTemp(void);  // CPU temperature
extern uint32_t SRC_SRSR;               // reset-reason register (a plain variable here)

// ---- Print / Serial -----------------------------------------------------------
// Same overload set the firmware relies on from the Teensy "Print" class.
class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  size_t print(const char* text);
  size_t print(char c);
  size_t print(unsigned char value, int base = DEC);
  size_t print(int value, int base = DEC);
  size_t print(unsigned int value, int base = DEC);
  size_t print(long value, int base = DEC);
  size_t print(unsigned long value, int base = DEC);
  size_t print(double value, int digits = 2);
  size_t println();
  size_t println(const char* text);
  size_t println(char c);
  size_t println(int value, int base = DEC);
  size_t println(unsigned int value, int base = DEC);
  size_t println(long value, int base = DEC);
  size_t println(unsigned long value, int base = DEC);
  size_t println(double value, int digits = 2);
};

class usb_serial_class : public Print {
 public:
  void begin(long baud);
  int available();
  int read();
  void flush();
  size_t write(uint8_t c) override;
  // On a Teensy, "if (Serial)" is true while a PC program has the port open.
  // The simulator can switch this off to imitate a pulled USB cable.
  operator bool();
};
extern usb_serial_class Serial;

// The two functions every sketch defines.
void setup();
// Called again and again for as long as the program runs.
void loop();

#endif  // BRI_SIM_ARDUINO_H
