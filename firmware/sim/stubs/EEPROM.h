// EEPROM.h (PC STUB) - stand-in for the Teensy emulated EEPROM.
// Backed by an array in memory that starts erased (all 0xFF), so every
// simulated run looks like the first boot of a new board.
#ifndef BRI_SIM_EEPROM_H
#define BRI_SIM_EEPROM_H

#include <Arduino.h>

class EEPROMClass {
 public:
  EEPROMClass() { memset(data_, 0xFF, sizeof(data_)); }
  uint8_t read(int address) { return data_[address]; }
  void write(int address, uint8_t value) { data_[address] = value; }
  template <typename T> T& get(int address, T& value) {
    memcpy(&value, &data_[address], sizeof(T));
    return value;
  }
  template <typename T> const T& put(int address, const T& value) {
    memcpy(&data_[address], &value, sizeof(T));
    return value;
  }

 private:
  uint8_t data_[1024];
};
extern EEPROMClass EEPROM;

#endif  // BRI_SIM_EEPROM_H
