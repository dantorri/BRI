# BRI Teensy 4.1 pin map

The pin map lives in two identical files: `Docs/pin_map.csv` (for reading) and `firmware/libraries/BRI/extras/pin_map.csv` (the copy the firmware tools use). Edit both together.
The firmware header `firmware/libraries/BRI/src/BRI_Pins.h` must agree with them. Check with `python firmware/tools/check_pins.py --csv Docs/pin_map.csv`; add `--write` to regenerate the header.

Phase numbers: 1 = simple balancer (build now), 2 = actuated hips, 3 = jump actuator (ODrive), 4 = Raspberry Pi 5.

## Why the pins are where they are

- **Encoders on 0/1, 2/3, 7/8, 30/31.** These four pairs can be routed to the Teensy's four hardware quadrature decoders (QuadEncoder library), so no pin moves if the software decoder ever becomes a load. Pins 5, 36 and 37 share decoder inputs with pins 0 and 1, so they are not used for encoders.
- **PWM on 6/9 and 28/29.** Each pair shares one timer, so both motors of a pair always run at the same 20 kHz.
- **Current sense on A0 to A3, battery on A6.** The analog inputs are grouped away from the PWM pins.
- **CAN1 (22/23), Serial8 (34/35), SPI (10 to 12) and the second I2C bus (24/25) are left free** for the ODrive, the Pi 5 and later sensors.
- **MOTOR_EN and HIP_EN are separate.** Wheels and hips can be put to sleep independently, and each net has its own pole on the kill switch.

## Build now (phase 1)

| teensy_pin | alt_function | net | direction | connects_to | conditioning | notes |
|---|---|---|---|---|---|---|
| 0 |  | ENC_LW_A | in | Left wheel encoder A (yellow) | SN74LVC245A ch1 (A1->B1) | QuadEncoder ch1 phase A capable |
| 1 |  | ENC_LW_B | in | Left wheel encoder B (white) | SN74LVC245A ch2 (A2->B2) | QuadEncoder ch1 phase B capable |
| 2 |  | ENC_RW_A | in | Right wheel encoder A (yellow) | SN74LVC245A ch3 (A3->B3) | QuadEncoder ch2 |
| 3 |  | ENC_RW_B | in | Right wheel encoder B (white) | SN74LVC245A ch4 (A4->B4) | QuadEncoder ch2 |
| 4 |  | DIR_LW | out | Left wheel G2 DIR | direct 3.3 V logic | DIR high = current OUTA->OUTB |
| 5 |  | DIR_RW | out | Right wheel G2 DIR | direct 3.3 V logic | Do not use pin 5 as an encoder input (shares XBAR input with pin 0) |
| 6 | PWM | PWM_LW | out | Left wheel G2 PWM | direct 3.3 V logic | 20 kHz; same FlexPWM timer as pin 9 |
| 9 | PWM | PWM_RW | out | Right wheel G2 PWM | direct 3.3 V logic | 20 kHz |
| 13 | LED/SCK | LED_HEARTBEAT | out | On-board LED | - | Becomes SCK if SPI is used later |
| 14 | A0 | CS_LW | analog in | Left wheel G2 CS | direct; no capacitor on CS | 50 mV + 20 mV/A |
| 15 | A1 | CS_RW | analog in | Right wheel G2 CS | direct; no capacitor on CS | 50 mV + 20 mV/A |
| 18 | SDA | I2C_SDA | i/o | IMU SDA (ToF later) | pull-up to 3.3 V only | Wire bus |
| 19 | SCL | I2C_SCL | out | IMU SCL (ToF later) | pull-up to 3.3 V only | Wire bus |
| 20 | A6 | VBAT_SENSE | analog in | Battery bus after fuse and disconnect | 33 k / 10 k divider + 100 nF to GND | 12.6 V -> 2.93 V |
| 21 |  | MOTOR_EN_SENSE | in | MOTOR_EN net (wheel SLP pins) | direct | Reads back the enable net; low while kill switch is in STOP |
| 33 |  | MOTOR_EN | out | Wheel G2 SLP pins (both) | 1 k series + 10 k pull-down; kill switch pole 1 | High = wheel drivers awake |
| 36 |  | IMU_INT | in | IMU INT pin | direct (3.3 V) | Data-ready interrupt; not needed for the bench tests |
| 38 | A14 | FLT_LW | in | Left wheel G2 FLT (open drain, low = fault) | INPUT_PULLUP to 3.3 V |  |
| 39 | A15 | FLT_RW | in | Right wheel G2 FLT | INPUT_PULLUP to 3.3 V |  |

## Reserved for later phases (do not use for anything else)

| teensy_pin | alt_function | net | phase | connects_to | conditioning | notes |
|---|---|---|---|---|---|---|
| 7 |  | ENC_LH_A | 2 | Left hip encoder A | SN74LVC245A ch5 (A5->B5) | QuadEncoder ch3 |
| 8 |  | ENC_LH_B | 2 | Left hip encoder B | SN74LVC245A ch6 (A6->B6) | QuadEncoder ch3 |
| 10 | CS | SPI_CS (reserved) | reserved | Future SPI device | - | Keep free |
| 11 | MOSI | SPI_MOSI (reserved) | reserved | Future SPI device | - | Keep free |
| 12 | MISO | SPI_MISO (reserved) | reserved | Future SPI device | - | Keep free |
| 16 | A2 | CS_LH | 2 | Left hip G2 CS | direct; no capacitor on CS |  |
| 17 | A3 | CS_RH | 2 | Right hip G2 CS | direct; no capacitor on CS |  |
| 22 | CTX1 | CAN1_TX (reserved) | 3 | Future CAN transceiver TXD -> ODrive S1 | 3.3 V CAN transceiver | Jump phase |
| 23 | CRX1 | CAN1_RX (reserved) | 3 | Future CAN transceiver RXD | 3.3 V CAN transceiver | Jump phase |
| 24 | SCL2/TX6 | I2C2_SCL (reserved) | reserved | Second I2C bus or Serial6 | - | Keep free |
| 25 | SDA2/RX6 | I2C2_SDA (reserved) | reserved | Second I2C bus or Serial6 | - | Keep free |
| 26 |  | DIR_LH | 2 | Left hip G2 DIR | direct 3.3 V logic |  |
| 27 |  | DIR_RH | 2 | Right hip G2 DIR | direct 3.3 V logic |  |
| 28 | PWM | PWM_LH | 2 | Left hip G2 PWM | direct 3.3 V logic | 20 kHz; same FlexPWM timer as pin 29 |
| 29 | PWM | PWM_RH | 2 | Right hip G2 PWM | direct 3.3 V logic | 20 kHz |
| 30 |  | ENC_RH_A | 2 | Right hip encoder A | SN74LVC245A ch7 (A7->B7) | QuadEncoder ch4 |
| 31 |  | ENC_RH_B | 2 | Right hip encoder B | SN74LVC245A ch8 (A8->B8) | QuadEncoder ch4 |
| 32 |  | HIP_EN | 2 | Hip G2 SLP pins (both) | 1 k series + 10 k pull-down; kill switch pole 2 | High = hip drivers awake |
| 34 | RX8 | PI_UART_RX (reserved) | 4 | Future Pi 5 UART TX | 3.3 V both sides | USB is the primary Pi link |
| 35 | TX8 | PI_UART_TX (reserved) | 4 | Future Pi 5 UART RX | 3.3 V both sides |  |
| 37 |  | SPARE | reserved | Status LED / buzzer | - |  |
| 40 | A16 | FLT_LH | 2 | Left hip G2 FLT | INPUT_PULLUP to 3.3 V |  |
| 41 | A17 | FLT_RH | 2 | Right hip G2 FLT | INPUT_PULLUP to 3.3 V |  |

## Sources and checks

- G2 18v17 pin behaviour (SLP pulled high on the board, FLT open drain, CS = 50 mV + 20 mV/A, no capacitor on CS): Pololu product page saved in `Docs/`.
- QuadEncoder pin list: the Teensy 4.x Quad Encoder Library README.
- PWM, analog, CAN, serial and I2C pin functions are from the PJRC Teensy 4.1 pinout card as recalled; confirm against the card before soldering.
- HIP_EN has no read-back pin. If the hips need kill-switch detection in firmware, assign spare pin 37.
