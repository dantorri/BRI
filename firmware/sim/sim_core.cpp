// sim_core.cpp - PC implementation of the Arduino functions in stubs/Arduino.h,
// plus main().
//
// How the simulation runs:
//   * Time is SIMULATED.  Every pass through loop() advances the clock by
//     50 microseconds; delay() advances it by the requested amount.  A 20 s
//     test therefore finishes in well under a second of real time, and the
//     result is the same on every run.
//   * Everything the firmware prints goes to standard output, exactly as it
//     would go up the USB cable.
//   * A scenario script supplies what a person would do at the bench: lines of
//     "<time_ms> <text>".  Plain text is typed into the firmware's serial
//     input.  Text starting with '@' is an instruction to the hardware model
//     (turn a wheel by hand, pull the USB cable, flip the kill switch, ...).
//
// Usage:  sketch.exe --ms <simulated milliseconds to run> --script <file>
#include <Arduino.h>
#include <EEPROM.h>
#include <Encoder.h>
#include <QuadEncoder.h>
#include <Wire.h>

#include <stdio.h>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <fcntl.h>
#include <io.h>
#endif

#include "sim.h"

// ---- simulator state ----------------------------------------------------------
uint8_t g_sim_pin_mode[SIM_PIN_COUNT];
uint8_t g_sim_pin_out[SIM_PIN_COUNT];
double g_sim_pwm_duty[SIM_PIN_COUNT];
bool g_sim_usb_connected = true;

static uint64_t s_time_us = 0;          // simulated microseconds since "boot"
static uint64_t s_hw_time_us = 0;       // time up to which the hardware model has been stepped
static uint32_t s_pwm_bits = 8;         // analogWrite resolution
static uint32_t s_adc_bits = 10;        // analogRead resolution
static std::string s_serial_in;         // characters waiting for Serial.read()
static bool s_quit = false;             // set by the "@quit" directive

struct ScriptLine {
  uint64_t time_us;
  std::string text;
  bool done;
};
static std::vector<ScriptLine> s_script;

usb_serial_class Serial;
TwoWire Wire;
EEPROMClass EEPROM;
uint32_t SRC_SRSR = 0x1;  // "power-on reset" flavoured value for the banner

// Simulated time in seconds (see sim.h).
double simNowSeconds() {
  return (double)s_time_us / 1e6;
}

// Stop the simulation with a message (see sim.h).
void simAbort(const char* message) {
  fflush(stdout);
  fprintf(stderr, "SIM ABORT: %s\n", message);
  exit(3);
}

// Feed any script lines whose time has come.
static void runDueScriptLines() {
  for (size_t i = 0; i < s_script.size(); i++) {
    ScriptLine& line = s_script[i];
    if (line.done || line.time_us > s_time_us) {
      continue;
    }
    line.done = true;
    if (!line.text.empty() && line.text[0] == '@') {
      if (line.text == "@quit") {
        s_quit = true;
      } else {
        simHwDirective(line.text.c_str() + 1);
      }
    } else {
      // Typed text: goes to the firmware's serial input with a newline.
      if (!line.text.empty() && line.text[0] == 'g') {
        simHwNoteGoCommand();
      }
      s_serial_in += line.text;
      s_serial_in += '\n';
    }
  }
}

// Move simulated time forward and keep the hardware model in step.
static void advanceTime(uint64_t microseconds) {
  s_time_us += microseconds;
  // Step the model in chunks of at least 50 us; finer steps would only cost
  // run time without changing the result.
  if (s_time_us - s_hw_time_us >= 50) {
    double dt = (double)(s_time_us - s_hw_time_us) / 1e6;
    s_hw_time_us = s_time_us;
    simHwStep(dt);
  }
  runDueScriptLines();
}

// ---- pins ---------------------------------------------------------------------
static bool pinOk(uint8_t pin) {
  return pin < SIM_PIN_COUNT;
}

// Remember the mode the firmware set on a pin.
void pinMode(uint8_t pin, uint8_t mode) {
  if (pinOk(pin)) {
    g_sim_pin_mode[pin] = mode;
  }
}

// Remember the level the firmware wrote to a pin.
void digitalWrite(uint8_t pin, uint8_t value) {
  if (!pinOk(pin)) {
    return;
  }
  g_sim_pin_out[pin] = value ? 1 : 0;
  g_sim_pwm_duty[pin] = value ? 1.0 : 0.0;  // a steady level is 0 % or 100 % duty
}

// Level on a pin: what the firmware wrote if it is an output, otherwise
// what the hardware model says is on it.
uint8_t digitalRead(uint8_t pin) {
  if (!pinOk(pin)) {
    return 0;
  }
  if (g_sim_pin_mode[pin] == OUTPUT) {
    return g_sim_pin_out[pin];
  }
  return (uint8_t)simHwDigitalIn(pin);
}

// Ask the hardware model for the pin's voltage and convert it to ADC counts.
int analogRead(uint8_t pin) {
  advanceTime(4);  // a conversion takes a few microseconds
  double volts = simHwAnalogVolts(pin);
  double max_count = (double)((1u << s_adc_bits) - 1u);
  double count = volts / 3.3 * max_count;
  if (count < 0.0) {
    count = 0.0;
  }
  if (count > max_count) {
    count = max_count;
  }
  return (int)(count + 0.5);
}

// Remember the PWM duty the firmware set, as a 0..1 fraction.
void analogWrite(uint8_t pin, int value) {
  if (!pinOk(pin)) {
    return;
  }
  double full_scale = (double)(1u << s_pwm_bits);
  double duty = (double)value / full_scale;
  if (duty < 0.0) {
    duty = 0.0;
  }
  if (duty > 1.0) {
    duty = 1.0;
  }
  g_sim_pwm_duty[pin] = duty;
  g_sim_pin_out[pin] = duty > 0.0 ? 1 : 0;
}

// Accepted and ignored: the model works with duty, not individual PWM cycles.
void analogWriteFrequency(uint8_t pin, float frequency) {
  (void)pin;
  (void)frequency;  // the model does not simulate individual PWM cycles
}

// Remember how many bits analogWrite values have (needed to turn them into a duty).
uint32_t analogWriteResolution(uint32_t bits) {
  uint32_t previous = s_pwm_bits;
  s_pwm_bits = bits;
  return previous;
}

// Remember how many bits analogRead should return.
void analogReadResolution(unsigned int bits) {
  s_adc_bits = bits;
}

// Accepted and ignored: the model's readings need no averaging.
void analogReadAveraging(unsigned int count) {
  (void)count;
}

// ---- time ---------------------------------------------------------------------
uint32_t millis() {
  advanceTime(1);  // reading the clock takes a moment; also guarantees that
                   // "while (millis() - start < x)" loops always finish
  return (uint32_t)(s_time_us / 1000);
}

// Simulated microseconds since boot (advances a little on every call).
uint32_t micros() {
  advanceTime(1);
  return (uint32_t)s_time_us;
}

// Let simulated time pass, stepping the hardware model along the way.
void delay(uint32_t ms) {
  // Advance in 50 us slices so the hardware model sees a smooth passage of time.
  uint64_t remaining = (uint64_t)ms * 1000;
  while (remaining > 0) {
    uint64_t slice = remaining > 50 ? 50 : remaining;
    advanceTime(slice);
    remaining -= slice;
  }
}

// Let a few simulated microseconds pass.
void delayMicroseconds(uint32_t us) {
  advanceTime(us);
}

// Pretend processor temperature (a fixed, plausible value).
float tempmonGetTemp(void) {
  return 44.5f;
}

// ---- Print / Serial -------------------------------------------------------------
static size_t writeText(Print* out, const char* text) {
  size_t count = 0;
  while (*text) {
    count += out->write((uint8_t)*text);
    text++;
  }
  return count;
}

// Shared helper: print an unsigned number in decimal or hex.
static size_t printUnsigned(Print* out, unsigned long long value, int base) {
  char buffer[72];
  if (base == HEX) {
    snprintf(buffer, sizeof(buffer), "%llX", value);
  } else {
    snprintf(buffer, sizeof(buffer), "%llu", value);
  }
  return writeText(out, buffer);
}

// Shared helper: print a signed number in decimal or hex.
static size_t printSigned(Print* out, long long value, int base) {
  char buffer[72];
  if (base == HEX) {
    snprintf(buffer, sizeof(buffer), "%llX", (unsigned long long)value);
  } else {
    snprintf(buffer, sizeof(buffer), "%lld", value);
  }
  return writeText(out, buffer);
}

// The print() and println() overloads below each turn one kind of value into
// text, exactly as the Teensy's Print class does.  They are one line each
// because all the work is in writeText / printSigned / printUnsigned above.
size_t Print::print(const char* text) { return writeText(this, text); }
size_t Print::print(char c) { return write((uint8_t)c); }
size_t Print::print(unsigned char value, int base) { return printUnsigned(this, value, base); }
size_t Print::print(int value, int base) { return printSigned(this, value, base); }
size_t Print::print(unsigned int value, int base) { return printUnsigned(this, value, base); }
size_t Print::print(long value, int base) { return printSigned(this, value, base); }
size_t Print::print(unsigned long value, int base) { return printUnsigned(this, value, base); }
// Print a decimal number with the given number of digits after the point,
// as Serial.print(value, digits) does on the Teensy.
size_t Print::print(double value, int digits) {
  char buffer[72];
  snprintf(buffer, sizeof(buffer), "%.*f", digits, value);
  return writeText(this, buffer);
}
// println() = print() followed by the line ending the firmware expects (CR LF).
size_t Print::println() { return writeText(this, "\r\n"); }
size_t Print::println(const char* text) { return print(text) + println(); }
size_t Print::println(char c) { return print(c) + println(); }
size_t Print::println(int value, int base) { return print(value, base) + println(); }
size_t Print::println(unsigned int value, int base) { return print(value, base) + println(); }
size_t Print::println(long value, int base) { return print(value, base) + println(); }
size_t Print::println(unsigned long value, int base) { return print(value, base) + println(); }
size_t Print::println(double value, int digits) { return print(value, digits) + println(); }

// Nothing to open on a PC; the baud rate is ignored (as on a real Teensy).
void usb_serial_class::begin(long baud) {
  (void)baud;
}

// How many typed characters are waiting to be read.
int usb_serial_class::available() {
  return (int)s_serial_in.size();
}

// Take the next typed character (-1 if none).
int usb_serial_class::read() {
  if (s_serial_in.empty()) {
    return -1;
  }
  int c = (unsigned char)s_serial_in[0];
  s_serial_in.erase(0, 1);
  return c;
}

// Push pending output to the terminal or file.
void usb_serial_class::flush() {
  fflush(stdout);
}

// Send one character 'up the USB cable' = to standard output.
size_t usb_serial_class::write(uint8_t c) {
  fputc(c, stdout);
  return 1;
}

// What 'if (Serial)' returns: true while the pretend PC has the port open.
// The script directive '@usb 0' makes it false, like pulling the cable.
usb_serial_class::operator bool() {
  return g_sim_usb_connected;
}

// ---- Wire -----------------------------------------------------------------------
void TwoWire::begin() {}

// Accepted and ignored: bus speed does not matter to the model.
void TwoWire::setClock(uint32_t hz) {
  (void)hz;
}

// Start collecting bytes to send to one I2C address.
void TwoWire::beginTransmission(uint8_t address) {
  tx_address_ = address;
  tx_length_ = 0;
}

// Add one byte to the message being collected.
size_t TwoWire::write(uint8_t value) {
  if (tx_length_ < sizeof(tx_buffer_)) {
    tx_buffer_[tx_length_] = value;
    tx_length_++;
    return 1;
  }
  return 0;
}

// The simulated device keeps a "register pointer", like the real chip: the
// first byte written selects a register, further bytes are stored into
// consecutive registers, and reads continue from the pointer.
static uint8_t s_i2c_register_pointer = 0;

// Deliver the collected bytes to the simulated device.  Returns 0 if it
// answered and 2 if nothing lives at that address, like the real library.
uint8_t TwoWire::endTransmission(uint8_t send_stop) {
  (void)send_stop;
  advanceTime(100 + 90 * (uint64_t)tx_length_);  // roughly 9 bits per byte at 100 kHz
  if (!simHwI2cPresent(tx_address_)) {
    return 2;  // nobody acknowledged the address
  }
  if (tx_length_ >= 1) {
    s_i2c_register_pointer = tx_buffer_[0];
    for (uint8_t i = 1; i < tx_length_; i++) {
      simHwI2cWriteRegister(tx_address_, s_i2c_register_pointer, tx_buffer_[i]);
      s_i2c_register_pointer++;
    }
  }
  return 0;
}

// Read bytes from the simulated device, starting at its register pointer.
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  advanceTime(100 + 90 * (uint64_t)quantity);
  rx_length_ = 0;
  rx_position_ = 0;
  if (!simHwI2cPresent(address)) {
    return 0;
  }
  for (uint8_t i = 0; i < quantity && i < sizeof(rx_buffer_); i++) {
    rx_buffer_[i] = simHwI2cReadRegister(address, s_i2c_register_pointer);
    s_i2c_register_pointer++;
    rx_length_++;
  }
  return rx_length_;
}

// How many received bytes are still unread.
int TwoWire::available() {
  return (int)rx_length_ - (int)rx_position_;
}

// Next received byte (-1 if none).
int TwoWire::read() {
  if (rx_position_ >= rx_length_) {
    return -1;
  }
  return rx_buffer_[rx_position_++];
}

// ---- Encoder backends -------------------------------------------------------------
Encoder::Encoder(uint8_t pin_a, uint8_t pin_b) : pin_a_(pin_a) {
  (void)pin_b;
  offset_ = simHwEncoderCount(pin_a_);
}

// Count since the last write(), taken from the hardware model.
int32_t Encoder::read() {
  return simHwEncoderCount(pin_a_) - offset_ + base_;
}

// Set the count to a given value (the firmware uses 0 to zero it).
void Encoder::write(int32_t position) {
  offset_ = simHwEncoderCount(pin_a_);
  base_ = position;
}

QuadEncoder::QuadEncoder(uint8_t encoder_ch, uint8_t phase_a_pin, uint8_t phase_b_pin, uint8_t pin_pus,
                         uint8_t index_pin, uint8_t home_pin, uint8_t trigger_pin)
    : pin_a_(phase_a_pin) {
  (void)pin_pus;
  (void)index_pin;
  (void)home_pin;
  (void)trigger_pin;
  // The pin pairs the BRI pin map assumes for hardware counters 1 to 4.
  static const uint8_t expected_a[4] = { 0, 2, 7, 30 };
  static const uint8_t expected_b[4] = { 1, 3, 8, 31 };
  if (encoder_ch < 1 || encoder_ch > 4) {
    simAbort("QuadEncoder: channel must be 1 to 4");
  }
  if (phase_a_pin != expected_a[encoder_ch - 1] || phase_b_pin != expected_b[encoder_ch - 1]) {
    simAbort("QuadEncoder: pins do not match the channel's pin pair");
  }
}

void QuadEncoder::setInitConfig() {}

// Mark the counter as started; reading before this is a firmware bug.
void QuadEncoder::init() {
  initialised_ = true;
  offset_ = simHwEncoderCount(pin_a_);
}

// Count since the last write(), taken from the hardware model.
int32_t QuadEncoder::read() {
  if (!initialised_) {
    simAbort("QuadEncoder: read() before init()");
  }
  return simHwEncoderCount(pin_a_) - offset_ + base_;
}

// Set the count to a given value (the firmware uses 0 to zero it).
void QuadEncoder::write(int32_t value) {
  offset_ = simHwEncoderCount(pin_a_);
  base_ = value;
}

// ---- main -------------------------------------------------------------------------
// Read the scenario script: one "<time_ms> <text>" per line; '#' starts a comment.
static void loadScript(const char* path) {
  FILE* file = fopen(path, "r");
  if (file == nullptr) {
    simAbort("cannot open script file");
  }
  char buffer[256];
  while (fgets(buffer, sizeof(buffer), file) != nullptr) {
    std::string line(buffer);
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r' || line.back() == ' ')) {
      line.pop_back();
    }
    if (line.empty() || line[0] == '#') {
      continue;
    }
    size_t space = line.find(' ');
    if (space == std::string::npos) {
      continue;
    }
    ScriptLine entry;
    entry.time_us = (uint64_t)(atof(line.substr(0, space).c_str()) * 1000.0);
    entry.text = line.substr(space + 1);
    entry.done = false;
    s_script.push_back(entry);
  }
  fclose(file);
}

// Program entry: read the options, load the script, call setup() once and
// loop() until the simulated time runs out, then print the summary.
int main(int argc, char** argv) {
#if defined(_WIN32)
  // Stop Windows turning "\n" into "\r\n"; the firmware already sends "\r\n".
  _setmode(_fileno(stdout), _O_BINARY);
#endif
  uint64_t limit_us = 5000000;
  for (int i = 1; i < argc; i++) {
    std::string arg(argv[i]);
    if (arg == "--ms" && i + 1 < argc) {
      limit_us = (uint64_t)(atof(argv[i + 1]) * 1000.0);
      i++;
    } else if (arg == "--script" && i + 1 < argc) {
      loadScript(argv[i + 1]);
      i++;
    }
  }

  simHwInit();
  setup();
  while (s_time_us < limit_us && !s_quit) {
    loop();
    advanceTime(50);
  }
  fflush(stdout);
  simHwPrintSummary();
  return 0;
}
