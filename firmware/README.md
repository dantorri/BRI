# BRI bench-test firmware kit (CV-001 to CV-009)

Bench bring-up tests for the BRI two-wheel balancing robot: Teensy 4.1, two
Pololu 4756 gearmotors with encoders, two Pololu G2 18v17 drivers, an
MPU-9250/6500 IMU and a 3S LiPo.

> ## STATUS: UNTESTED ON HARDWARE
> Nothing in this folder has been compiled for a Teensy or run on the robot.
> It has only been compiled for a PC against stand-in ("stub") Arduino headers
> and run against a simple software model of the hardware.
> **The stub build is not a Teensy build.** The first real compile in the
> Arduino IDE may show errors the PC build could not, and the first run on
> hardware may behave differently from the model. Treat every motor test as
> a first power-up: motor secured, kill switch within reach.

New to how the files fit together? Read `ARCHITECTURE.md` first.

## What is in this folder

| Path | What it is |
|---|---|
| `libraries/BRI/` | Arduino library shared by every test: pin map, config constants, motor driver, encoder, IMU, serial protocol, safety layer |
| `libraries/BRI/extras/pin_map.csv` | The pin map. `BRI_Pins.h` is generated from it |
| `tests/cvNNN_<name>/` | One folder per test: the sketch (`.ino`) and its PC logger (`.py`) |
| `tools/bri_logger.py` | Serial logger shared by every `.py` |
| `tools/check_pins.py` | Checks (or regenerates) `BRI_Pins.h` against the CSV |
| `sim/` | PC stand-ins for the Arduino core and libraries, a hardware model, and `build_and_replay.py` |
| `logs/` | Created when you run a logger; raw text, CSV and verdict files (git-ignored) |

## One-time setup

1. **Arduino IDE 2 with Teensy support.** Install the Arduino IDE, then add
   Teensy: *File > Preferences > Additional boards manager URLs* =
   `https://www.pjrc.com/teensy/package_teensy_index.json`, then *Boards
   Manager*, search "teensy", install. (These are the steps as I know them;
   check <https://www.pjrc.com/teensy/td_download.html> if the menus differ.)
2. **Board settings.** *Tools > Board > Teensy > Teensy 4.1*, *Tools > USB
   Type > Serial*.
3. **Let the IDE find the BRI library.** *File > Preferences > Sketchbook
   location* = this `firmware` folder. The IDE looks for libraries in
   `<sketchbook>/libraries`, so it then finds `libraries/BRI`. (Alternative:
   copy `libraries/BRI` into the `libraries` folder of your existing
   sketchbook.) The `Encoder`, `Wire` and `EEPROM` libraries come with the
   Teensy package; `QuadEncoder` is only needed for the optional hardware
   encoder backend.
4. **Python.** Python 3.8 or newer and pyserial: `python -m pip install pyserial`.
   Optional, for `--plot`: `python -m pip install matplotlib`.

## Running a test

1. Open the sketch, e.g. `tests/cv004_imu_raw/cv004_imu_raw.ino`, and upload it.
2. **Close the Arduino Serial Monitor** (only one program can hold the port).
3. In a terminal: `python tests\cv004_imu_raw\cv004_imu_raw.py`
   The logger finds the Teensy by its USB id (0x16C0), sends the start
   commands, shows events and verdicts as they arrive, and saves
   `logs/<test>_<time>.txt` (raw), `.csv` (data) and `_verdict.txt`.
4. Read the verdicts. There are two sets: the firmware's own `RESULT` lines,
   and the logger's evaluation of the recorded data. They should agree.

You can also run any test from the Serial Monitor alone: type `?` for the
commands, `g` to go, `s` to stop.

Options every logger has: `--port COM5`, `--seconds N`, `--out <name>`,
`--replay <file.txt>` (re-evaluate a saved capture with no hardware),
`--plot`, `--yes` (skip the GO prompt on motor tests). Motor and encoder
tests add `--channel 0|1|2|3` (0 = LW left wheel, 1 = RW right wheel,
2 = LH left hip, 3 = RH right hip; phase 1 hardware has only 0 and 1).

## The tests

Run them in order; each depends on the ones before it. "Workbook" is the
wording in the project workbook's Verification Plan; "numbers" are the limits
this kit applies.

| Test | Sketch and logger (in `tests/`) | You need | Pass criteria |
|---|---|---|---|
| CV-001 Teensy power and USB | `cv001_power_usb/` | Teensy, USB, multimeter | Workbook: stable boot, no USB backfeed, correct 3.3 V rail. Numbers: CPU temp 0-85 C, time never goes backwards, 3.3 V rail 3.20-3.40 V (you type `r3.29`), backfeed <= 0.30 V (you type `b0.02`) |
| CV-002 5 V regulator | `cv002_5v_regulator/` | Battery or bench supply, adjustable load, multimeter | Workbook: 5 V within tolerance, no thermal shutdown. Numbers: every reading you type (`i<amps>` then `v<volts>`) is 4.80-5.20 V; no Teensy reset (boot counter, uptime, serial link) |
| CV-003 I2C bus scan | `cv003_i2c_scan/` | Teensy + IMU | Workbook: IMU address detected repeatedly without bus errors. Numbers: found at 0x68 or 0x69 in every scan at 100 kHz, zero bus errors, WHO_AM_I is 0x71/0x73/0x70/0x68 |
| CV-004 Raw IMU data | `cv004_imu_raw/` | Teensy + IMU, held still | Workbook: axis signs and gravity magnitude consistent. Numbers: gravity 0.90-1.10 g; each gyro axis within +/-8 deg/s of zero, spread < 2 deg/s; the axis reading +1 g is reported |
| CV-005 IMU pitch | `cv005_imu_pitch/` | Teensy + IMU in its mount | Workbook: pitch sign, axis and zero documented. Numbers: hold still 2 s, tilt forward then back; first tilt reads positive; gyro agrees with accelerometer; zero angle reported |
| CV-006 Motor driver logic | `cv006_driver_logic/` | Driver powered, **motor unplugged (preferred) or restrained** | Workbook: logic responds without faults. Numbers: MOTOR_EN_SENSE low asleep / high awake; FLT never low while awake; CS 30-80 mV while driving with the motor unplugged (nominal 50 mV) |
| CV-007 Motor no-load | `cv007_motor_noload/` | **Motor secured, wheel off** | Workbook: correct direction, current near no-load, no abnormal noise. Numbers: speed positive on + duty and negative on - duty; current 0.05-0.60 A in both holds (datasheet 0.2 A); no fault |
| CV-008 Encoder waveform | `cv008_encoder_waveform/` | Motor secured, oscilloscope | Workbook: Teensy-side signals 0-3.3 V and cleanly quadrature. Numbers: A and B each >= 20 edges; count moves one way, matching the duty sign; scope high level 2.8-3.5 V (you type `v3.28`) |
| CV-009 Encoder calibration | `cv009_encoder_cal/` | Wheel on, tape mark | Workbook: 8400 counts/rev within tolerance. Numbers: each hand-turned revolution (`b` ... `e`) within 1 % of 8400; first (forward) turn counts positive; optional motor move with `--motor-revs N` then `o<turns seen>` |

Limits not given in the task brief (CPU temperature, 3.3 V and backfeed
bands, gyro limits, CS band, no-load current band, scope band) are my
engineering judgement. Each is a named constant with its reasoning at the top
of the sketch and the logger; change them if you disagree.

## Safety rules built into the firmware

- All drivers are asleep at boot and stay asleep until you send `g`. The PC
  loggers for motor tests also make you type `GO` first.
- `s` at any time brakes and sleeps every driver.
- Closing the logger, or unplugging USB, stops the motor (the Teensy sees the
  serial port close).
- Duty is capped at 35 % inside the library (`BRI_DUTY_CAP`).
- A run stops by itself on: driver fault (FLT low), kill switch in STOP
  (MOTOR_EN_SENSE low while commanded high), 3.0 A for 200 ms, or its time limit.
- Only the selected channel is driven; a wheel test never wakes the hip
  drivers and the reverse.

**These are software protections that have only been run in simulation. Your
physical kill switch is the protection that counts.**

## After the tests: constants to update in `libraries/BRI/src/BRI_Config.h`

| If this test says... | ...change this |
|---|---|
| CV-004: the +1 g axis is not Z | `BRI_PITCH_UP_AXIS` / `BRI_PITCH_UP_SIGN` (and the forward and gyro axes to match) |
| CV-005: first forward tilt read negative | `BRI_PITCH_FWD_SIGN` and `BRI_PITCH_GYRO_SIGN` |
| CV-005: zero is not 0 in the final mount | `BRI_PITCH_ZERO_DEG` |
| CV-007: the wheel turned backward on + duty | `BRI_MOTOR_SIGN[channel]` |
| CV-009: a forward turn counted negative | `BRI_ENC_SIGN[channel]` |

The two wheels are mounted mirror-image, so expect to flip one motor sign and
one encoder sign.

**Encoder backend.** The default is the PJRC `Encoder` library. To use the
Teensy 4 hardware decoders instead, uncomment `#define BRI_USE_HW_QUADENC` in
`BRI_Config.h` (it has to be there, not in a sketch, because the IDE compiles
library files separately).

**Changing a pin.** Edit `libraries/BRI/extras/pin_map.csv`, then run
`python tools/check_pins.py --write` to regenerate `BRI_Pins.h`, then
`python tools/check_pins.py` to confirm they agree.

## What was verified, and what was not

Run it yourself: `python sim\build_and_replay.py` (needs `zig`; pass
`--zig <path>` or set the `ZIG` environment variable if it is not on the
PATH). The log of the run that shipped with this kit is
`firmware_verification.txt`, delivered next to the zip.

**Verified on a PC (stub build + software model), in that run:**

| Step | Result |
|---|---|
| `BRI_Pins.h` agrees with the pin map CSV (library copy and the authoritative project CSV) | 2 of 2 |
| Each sketch's CSV header string equals its logger's expected header | 9 of 9 |
| Every sketch compiles with `zig c++` against the stubs, for both encoder backends, with `-Wall -Wextra` and no warnings | 18 of 18 |
| Every build runs in simulation and emits a header, data rows of the right length and RESULT lines, with the drivers asleep before `g` and at the end, and duty never above the cap | 18 of 18 |
| Every simulated capture replays through its logger: header and field counts agree, no bad rows, evaluator runs | 18 of 18 |
| Safety scenarios in simulation (no go, USB lost, kill switch, driver fault, over-current, operator stop, timeout, duty cap, wheel/hip enable separation) | 12 of 12 |
| Fault-finding scenarios: the model is made wrong on purpose and both firmware and logger must report it | 12 of 12 |
| Logger live-capture code against a fake serial port on a local socket, and the GO prompt sending nothing when declined | 2 of 2 |

**Not verified:**

- **No Teensy compile.** The sketches have not been built with Teensyduino.
  The stubs only imitate the functions the kit uses. Calls I believe exist in
  the Teensy 4 core and libraries but could not check here include
  `tempmonGetTemp()`, the `SRC_SRSR` register, `analogWriteFrequency()`,
  `analogReadAveraging()`, `EEPROM.get/put`, and the `QuadEncoder` constructor
  and `setInitConfig()/init()` calls.
- **No hardware.** No motor, driver, encoder, IMU, regulator or battery has
  been connected to this code.
- **The hardware model is simple and I wrote it**, from the datasheet numbers
  in the task. Agreement between firmware and model shows the code is
  self-consistent, not that it matches the real parts.
- **Current-sense method.** The G2 CS pin is only valid during the PWM
  on-time, so the library takes 64 quick ADC readings and keeps the largest.
  Whether the Teensy ADC is fast enough for this at low duty is unproven.
  CV-007 logs both the peak and the average so you can check (average/peak
  should be about the duty). Resolution is about 0.04 A per ADC step.
- **FLT while asleep.** I do not know whether the G2 pulls FLT low in sleep
  or with motor power off. The tests report it but only judge FLT while awake.
- **HIP_EN has no read-back pin** in the pin map, so the firmware cannot
  detect the kill switch on the hip pole (the switch itself still cuts drive).
- **USB-loss stop** relies on the Teensy seeing the port close. If a PC
  program hangs with the port still open, the motor runs until the test's
  time limit (at most 60 s).
- **The 3.0 A over-current trip** cannot normally be reached at 35 % duty (a
  stalled motor draws about 1.9 A there); it is a backstop for a short, and
  matters more once the cap is raised.
- **CV-008 raw pin levels with the hardware encoder backend**: reading the
  pins with `digitalRead()` may not work once the hardware decoder owns
  them; that backend's result is marked for operator confirmation.
- **Real serial port.** The logger's capture code ran against a socket, not a
  COM port; port auto-detection was only seen to fail cleanly with no Teensy
  attached. The `--plot` drawing code was not run (matplotlib was not
  installed where this was built).
- **MPU-6050 path** (skipping one register, different temperature formula)
  was not exercised in simulation; only MPU-6500 and MPU-9250 ids were.
