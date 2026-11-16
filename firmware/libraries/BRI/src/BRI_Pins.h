// BRI_Pins.h - Teensy 4.1 pin numbers for the BRI balancing robot.
//
// GENERATED FILE.  Do not edit by hand.
// Source of truth: libraries/BRI/extras/pin_map.csv
// Regenerate with:  python tools/check_pins.py --write
// Check with:       python tools/check_pins.py
//
// Why constants instead of bare numbers: a sketch that says
// digitalWrite(PIN_MOTOR_EN, HIGH) can be read and searched; a sketch that
// says digitalWrite(33, HIGH) cannot.  Every name below is 'PIN_' followed by
// the 'net' column of the CSV, so the wiring table and the code use one name.
#ifndef BRI_PINS_H
#define BRI_PINS_H

#include <stdint.h>

// ---- One constant per CSV row (CSV order) -------------------------------
// Comment format: direction | phase | what it connects to | conditioning
const uint8_t PIN_ENC_LW_A       =  0;  // in | phase 1 | Left wheel encoder A (yellow) | SN74LVC245A ch1 (A1->B1)
const uint8_t PIN_ENC_LW_B       =  1;  // in | phase 1 | Left wheel encoder B (white) | SN74LVC245A ch2 (A2->B2)
const uint8_t PIN_ENC_RW_A       =  2;  // in | phase 1 | Right wheel encoder A (yellow) | SN74LVC245A ch3 (A3->B3)
const uint8_t PIN_ENC_RW_B       =  3;  // in | phase 1 | Right wheel encoder B (white) | SN74LVC245A ch4 (A4->B4)
const uint8_t PIN_DIR_LW         =  4;  // out | phase 1 | Left wheel G2 DIR | direct 3.3 V logic
const uint8_t PIN_DIR_RW         =  5;  // out | phase 1 | Right wheel G2 DIR | direct 3.3 V logic
const uint8_t PIN_PWM_LW         =  6;  // out | phase 1 | Left wheel G2 PWM | direct 3.3 V logic
const uint8_t PIN_ENC_LH_A       =  7;  // in | phase 2 | Left hip encoder A | SN74LVC245A ch5 (A5->B5)
const uint8_t PIN_ENC_LH_B       =  8;  // in | phase 2 | Left hip encoder B | SN74LVC245A ch6 (A6->B6)
const uint8_t PIN_PWM_RW         =  9;  // out | phase 1 | Right wheel G2 PWM | direct 3.3 V logic
const uint8_t PIN_SPI_CS         = 10;  // RESERVED, keep free | - | phase reserved | Future SPI device | -
const uint8_t PIN_SPI_MOSI       = 11;  // RESERVED, keep free | - | phase reserved | Future SPI device | -
const uint8_t PIN_SPI_MISO       = 12;  // RESERVED, keep free | - | phase reserved | Future SPI device | -
const uint8_t PIN_LED_HEARTBEAT  = 13;  // out | phase 1 | On-board LED | -
const uint8_t PIN_CS_LW          = 14;  // analog in | phase 1 | Left wheel G2 CS | direct; no capacitor on CS
const uint8_t PIN_CS_RW          = 15;  // analog in | phase 1 | Right wheel G2 CS | direct; no capacitor on CS
const uint8_t PIN_CS_LH          = 16;  // analog in | phase 2 | Left hip G2 CS | direct; no capacitor on CS
const uint8_t PIN_CS_RH          = 17;  // analog in | phase 2 | Right hip G2 CS | direct; no capacitor on CS
const uint8_t PIN_I2C_SDA        = 18;  // i/o | phase 1 | IMU SDA (ToF later) | pull-up to 3.3 V only
const uint8_t PIN_I2C_SCL        = 19;  // out | phase 1 | IMU SCL (ToF later) | pull-up to 3.3 V only
const uint8_t PIN_VBAT_SENSE     = 20;  // analog in | phase 1 | Battery bus after fuse and disconnect | 33 k / 10 k divider + 100 nF to GND
const uint8_t PIN_MOTOR_EN_SENSE = 21;  // in | phase 1 | MOTOR_EN net (wheel SLP pins) | direct
const uint8_t PIN_CAN1_TX        = 22;  // RESERVED, keep free | - | phase 3 | Future CAN transceiver TXD -> ODrive S1 | 3.3 V CAN transceiver
const uint8_t PIN_CAN1_RX        = 23;  // RESERVED, keep free | - | phase 3 | Future CAN transceiver RXD | 3.3 V CAN transceiver
const uint8_t PIN_I2C2_SCL       = 24;  // RESERVED, keep free | - | phase reserved | Second I2C bus or Serial6 | -
const uint8_t PIN_I2C2_SDA       = 25;  // RESERVED, keep free | - | phase reserved | Second I2C bus or Serial6 | -
const uint8_t PIN_DIR_LH         = 26;  // out | phase 2 | Left hip G2 DIR | direct 3.3 V logic
const uint8_t PIN_DIR_RH         = 27;  // out | phase 2 | Right hip G2 DIR | direct 3.3 V logic
const uint8_t PIN_PWM_LH         = 28;  // out | phase 2 | Left hip G2 PWM | direct 3.3 V logic
const uint8_t PIN_PWM_RH         = 29;  // out | phase 2 | Right hip G2 PWM | direct 3.3 V logic
const uint8_t PIN_ENC_RH_A       = 30;  // in | phase 2 | Right hip encoder A | SN74LVC245A ch7 (A7->B7)
const uint8_t PIN_ENC_RH_B       = 31;  // in | phase 2 | Right hip encoder B | SN74LVC245A ch8 (A8->B8)
const uint8_t PIN_HIP_EN         = 32;  // out | phase 2 | Hip G2 SLP pins (both) | 1 k series + 10 k pull-down; kill switch pole 2
const uint8_t PIN_MOTOR_EN       = 33;  // out | phase 1 | Wheel G2 SLP pins (both) | 1 k series + 10 k pull-down; kill switch pole 1
const uint8_t PIN_PI_UART_RX     = 34;  // RESERVED, keep free | - | phase 4 | Future Pi 5 UART TX | 3.3 V both sides
const uint8_t PIN_PI_UART_TX     = 35;  // RESERVED, keep free | - | phase 4 | Future Pi 5 UART RX | 3.3 V both sides
const uint8_t PIN_IMU_INT        = 36;  // in | phase 1 (optional) | IMU INT pin | direct (3.3 V)
const uint8_t PIN_SPARE          = 37;  // RESERVED, keep free | - | phase reserved | Status LED / buzzer | -
const uint8_t PIN_FLT_LW         = 38;  // in | phase 1 | Left wheel G2 FLT (open drain, low = fault) | INPUT_PULLUP to 3.3 V
const uint8_t PIN_FLT_RW         = 39;  // in | phase 1 | Right wheel G2 FLT | INPUT_PULLUP to 3.3 V
const uint8_t PIN_FLT_LH         = 40;  // in | phase 2 | Left hip G2 FLT | INPUT_PULLUP to 3.3 V
const uint8_t PIN_FLT_RH         = 41;  // in | phase 2 | Right hip G2 FLT | INPUT_PULLUP to 3.3 V

// ---- Motor channel numbering ---------------------------------------------
// The library addresses a motor by a channel index so that one test sketch
// can exercise any of the four motors.  Phase 1 hardware has only LW and RW
// wired; LH and RH (the hips) are phase 2.
const uint8_t BRI_NUM_CHANNELS = 4;
const uint8_t BRI_CH_LW = 0;
const uint8_t BRI_CH_RW = 1;
const uint8_t BRI_CH_LH = 2;
const uint8_t BRI_CH_RH = 3;

// ---- Per-channel lookup tables (index = channel) -------------------------
// These let the library write BRI_PIN_PWM[channel] instead of a four-way
// if/else for every pin.
// PWM output to the G2 driver (speed)
const uint8_t BRI_PIN_PWM[4] = { PIN_PWM_LW, PIN_PWM_RW, PIN_PWM_LH, PIN_PWM_RH };
// DIR output to the G2 driver (direction)
const uint8_t BRI_PIN_DIR[4] = { PIN_DIR_LW, PIN_DIR_RW, PIN_DIR_LH, PIN_DIR_RH };
// analog current-sense input from the G2 driver
const uint8_t BRI_PIN_CS[4] = { PIN_CS_LW, PIN_CS_RW, PIN_CS_LH, PIN_CS_RH };
// fault input from the G2 driver (low = fault)
const uint8_t BRI_PIN_FLT[4] = { PIN_FLT_LW, PIN_FLT_RW, PIN_FLT_LH, PIN_FLT_RH };
// encoder channel A (after the level shifter)
const uint8_t BRI_PIN_ENC_A[4] = { PIN_ENC_LW_A, PIN_ENC_RW_A, PIN_ENC_LH_A, PIN_ENC_RH_A };
// encoder channel B (after the level shifter)
const uint8_t BRI_PIN_ENC_B[4] = { PIN_ENC_LW_B, PIN_ENC_RW_B, PIN_ENC_LH_B, PIN_ENC_RH_B };

#endif  // BRI_PINS_H
