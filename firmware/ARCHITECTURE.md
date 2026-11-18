# How the BRI bench-test kit fits together

This is a plain-language tour. It explains what each piece is for and how a
test run flows through them. Nothing here has run on hardware yet (see
`README.md`).

## The big picture

```
   pin_map.csv ──(tools/check_pins.py)──> BRI_Pins.h ─┐
                                                      │
   BRI_Config.h (every tunable number) ───────────────┤
                                                      v
   ┌──────────────── libraries/BRI (shared code) ────────────────┐
   │ System   battery, CPU temperature, boot counter             │
   │ Motor    G2 driver: sleep, direction, PWM, fault, current   │
   │ Encoder  counts (PJRC Encoder or hardware QuadEncoder)      │
   │ IMU      MPU-9250/6500 over I2C, pitch                      │
   │ Safety   when a motor may move, when it must stop           │
   │ Telemetry  the text protocol                                │
   └──────────────────────────┬───────────────────────────────────┘
                              │ #include <BRI.h>
   tests/cvNNN_x/cvNNN_x.ino  │   one small sketch per test (runs on the Teensy)
                              │
                     USB serial, plain text
                              │
   tests/cvNNN_x/cvNNN_x.py ──┴── tools/bri_logger.py   (runs on the PC)
                                        │
                                   logs/*.txt  *.csv  *_verdict.txt
```

A second route exists for checking the code without a Teensy:

```
   cvNNN_x.ino + libraries/BRI + sim/stubs + sim/sim_core.cpp + sim/sim_bri_hw.cpp
        ──(zig c++)──> a PC program ──> the same text ──> cvNNN_x.py --replay
```

## The pieces

**The pin map (`libraries/BRI/extras/pin_map.csv` and `BRI_Pins.h`).**
The CSV is the single source of truth for wiring. `tools/check_pins.py
--write` turns it into `BRI_Pins.h`, one constant per row, named `PIN_` plus
the CSV's `net` column (`PIN_MOTOR_EN = 33`). The header also holds lookup
tables indexed by channel (`BRI_PIN_PWM[channel]`) so code can address "the
PWM pin of channel 1" without four-way if/else. Never edit the header by
hand; edit the CSV and regenerate.

**`BRI_Config.h`.** Every number you might want to change, each with a comment
saying what it is and why it has that value: duty cap, over-current trip,
encoder counts per revolution, IMU axis choice, motor and encoder signs.

**The library modules.** Each is a `.h` (what it offers, with the explanation)
and a `.cpp` (how it does it).

- `BRI_System`: ADC setup, battery voltage through the 33k/10k divider, CPU
  temperature, and a boot counter kept in EEPROM so a reset can be detected
  after the fact.
- `BRI_Motor`: the lowest layer for the G2 drivers. It knows the pins and
  enforces two things itself: the duty cap, and "a sleeping driver gets zero
  duty".
- `BRI_Safety`: the layer sketches actually use to move a motor. See below.
- `BRI_Encoder`: one small interface over two possible counting libraries.
- `BRI_IMU`: finds the chip (0x68 or 0x69), identifies it (WHO_AM_I), wakes
  and configures it, converts raw readings to g and deg/s, and computes pitch
  using the axis constants from the config.
- `BRI_Telemetry`: prints and parses the text protocol.

**The sketches.** Each `.ino` is one test and nothing else. They all have the
same shape, on purpose, so that once you have read one you can read them all:

```
settings (constants with reasons)     what "pass" means for this test
state variables                       what the sketch remembers
printHelp()                           the commands it accepts
startRun()                            banner, header, reset tallies
finishRun()                           RESULT lines
handleCommand()                       act on one line from the PC
setup()                               briBegin(...) and help
loop()                                heartbeat, commands, safety, measure, print
```

`loop()` never waits. It looks at the clock and does whatever is due, so it
can check for a stop condition thousands of times a second.

**The loggers.** Each `.py` holds only what is special about its test: the
expected header, the commands that start it, and an `evaluate()` function
with the pass rules. `tools/bri_logger.py` does everything shared: find the
port, send commands, record, parse, check the protocol, save files, print the
summary.

Why judge twice (firmware and logger)? The firmware's `RESULT` lines work
with nothing but a Serial Monitor. The logger's evaluation works on a saved
file, so a capture can be re-judged later (`--replay`), and it is an
independent calculation from the same data: if the two disagree, one of them
has a bug.

## Life of one test run

1. Power-up: `setup()` calls `briBegin()`. Motor pins are made safe first
   (drivers asleep, PWM low), then ADC and boot counter, then USB serial, then
   the banner.
2. The logger opens the port and sends the start commands (for example
   `c0`, `d0.3`, `g`). For a motor test it first makes you type `GO`.
3. On `g` the sketch prints the banner again and the CSV header, so every
   capture is self-describing, and starts the run.
4. Every 20 ms it prints one data row. Events (`!` lines) appear as they happen.
5. At the end (time up, profile finished, `s`, or a safety stop) it prints
   one `RESULT` line per check and a final `overall` line.
6. The logger stops recording one second after the last `RESULT`, sends `s`
   (harmless, and guarantees the drivers are asleep), parses, evaluates, saves.

## The serial protocol

Every line the firmware prints is one of five kinds, recognised by how it
starts:

| Starts with | Meaning | Example |
|---|---|---|
| `#` | Text for humans: banner, help, notes | `# BRI bench test CV-004: Raw IMU data` |
| `t_ms,` | The CSV header: column names for this run | `t_ms,ax_g,ay_g,az_g,...` |
| a digit | A data row: exactly as many fields as the header; `t_ms` is milliseconds since boot | `1234,0.01,-0.02,1.00,...` |
| `!` | An event or fault: `!NAME,t_ms,detail` | `!STOP,5230,timeout` |
| `RESULT,` | A verdict: `RESULT,<test id>,<PASS\|FAIL\|INFO>,<detail>` | `RESULT,CV-004,PASS,gravity_g=1.002 min=0.90 max=1.10` |

Rules:

- Lines end with CR LF. `detail` is `key=value` pairs and words separated by
  spaces and never contains a comma, so any line can be split on commas.
- A missing number is printed as `nan`.
- One run = one header, its rows, then its `RESULT` lines. The last `RESULT`
  line is always `overall`: FAIL if any check failed; INFO if nothing failed
  but something still needs the operator (or nothing was checked); otherwise
  PASS.
- Lines starting `#LOGGER` are added by the PC logger (for example when the
  serial link drops), never by the firmware.
- If a row's field count ever differs from the header, the firmware raises
  `!ROWLEN` once.

Commands (PC to Teensy) are short text lines ending in newline: one letter,
optionally followed by a number (`t10`, `v 5.02`, `d=-0.15` all work).

| Command | Tests | Meaning |
|---|---|---|
| `?` | all | print help |
| `g` | all | go: start the run (CV-009: start the armed motor move) |
| `s` | all | stop: brake, sleep all drivers, end the run and print results |
| `t<sec>` | all but CV-007 | run length (CV-006: seconds per step) |
| `c<n>` | CV-006 to CV-009 | channel 0=LW 1=RW 2=LH 3=RH |
| `d<duty>` | CV-007, CV-008 | peak / constant duty (limited to the cap) |
| `r<V>`, `b<V>` | CV-001 | multimeter: 3.3 V rail, backfeed voltage |
| `i<A>`, `v<V>` | CV-002 | multimeter: load current, regulator output |
| `m<0\|1>` | CV-006 | motor unplugged (0) or connected and restrained (1) |
| `v<V>` | CV-008 | oscilloscope: highest level at the Teensy encoder pins |
| `h`, `b`, `e` | CV-009 | start hand mode; begin / end of one hand-turned revolution |
| `m<revs>`, `o<revs>` | CV-009 | arm a motor move; turns the operator observed |

Events:

| Event | From | Detail |
|---|---|---|
| `!GO` | safety layer | `channel=LW timeout_ms=17000` |
| `!STOP` | safety layer | one word: `complete`, `operator`, `usb_lost`, `timeout`, `driver_fault`, `kill_switch`, `over_current`, `bad_channel` |
| `!DMM` | CV-001, CV-002 | the multimeter values typed in |
| `!SCOPE` | CV-008 | `vhigh=3.28` |
| `!ZERO`, `!TILT` | CV-005 | zero angle and gyro offset; direction of the first tilt |
| `!MARK`, `!REV` | CV-009 | hand revolution begin; counts for the completed revolution |
| `!MOVE`, `!MOVECAL` | CV-009 | motor move finished; calibration from the observed turns |
| `!ROWLEN` | telemetry | a row did not match the header |

## The safety layer

There are two layers, deliberately:

- `BRI_Motor` (lowest): caps duty at `BRI_DUTY_CAP` and forces zero duty for
  a sleeping driver, whatever is asked.
- `BRI_Safety` (what sketches call): a motor can only be driven between
  `briSafetyGo(channel, time limit)` and a stop. `briSafetyGo` wakes only
  that channel's driver group, waits 10 ms, and refuses if the kill switch is
  in STOP or FLT is low. While running, the sketch calls `briSafetyService()`
  every loop; it stops everything on USB loss, time limit, FLT low, kill
  switch, or 3.0 A for 200 ms. "Stop" is always: PWM to zero on all channels,
  both enable nets low, then a `!STOP` line with the reason.

Kill switch detection: MOTOR_EN (pin 33) drives the wheel drivers' sleep pins
through 1 k; the kill switch shorts that net to ground. MOTOR_EN_SENSE
(pin 21) reads the net back. Commanded high but reading low means the switch
is in STOP. HIP_EN has no read-back pin.

## Measuring current

The G2's CS pin shows 50 mV + 20 mV per ampere, but only while the driver is
in the "on" part of each PWM cycle. At 20 kHz and 10 % duty that is 5
microseconds at a time, and the ADC is not synchronised to it. So
`briMotorReadCurrent()` takes 64 quick readings and keeps the largest (the
on-time value); it also reports the average, which should be about duty times
the peak. This method is reasoned, not yet proven on hardware.

## The simulation (`sim/`)

- `sim/stubs/*.h`: stand-ins for `Arduino.h`, `Wire.h`, `Encoder.h`,
  `QuadEncoder.h`, `EEPROM.h`. Same function names as the Teensy versions,
  but only the ones this kit uses.
- `sim/sim_core.cpp`: implements those functions on a PC. Time is simulated:
  each pass through `loop()` advances the clock 50 microseconds, so a 20 s
  test runs in a fraction of a second and gives the same output every time.
  It also has `main()`, which calls `setup()` once and `loop()` repeatedly.
- `sim/sim_bri_hw.cpp`: a simple model of the electronics (enable nets and
  kill switch, G2 drivers, motors, encoders, battery divider, an IMU that
  follows a scripted tilt). It includes the same `BRI_Pins.h` as the firmware.
- Scenario scripts: lines of `<time_ms> <text>`. Plain text is typed into the
  firmware (`10 g`). Text starting with `@` instructs the model:
  `@turn <ch> <revs> <ms>` (turn a wheel by hand), `@usb 0`, `@kill 1`,
  `@fault <ch> 1`, `@extra <ch> <amps>`, `@stuck <ch> 1`,
  `@disconnect <ch> 1`, `@encflip <ch> 1`, `@tilt <deg> <period_ms> <start_ms>`,
  `@imu <addr_hex> <who_hex>`, `@vbat <volts>`.
- `sim/selftest/lib_probe/`: a simulation-only sketch that asks the library
  for 90 % duty on purpose, to prove the library refuses. Do not flash it.
- `sim/build_and_replay.py`: builds everything, runs the scenarios, replays
  the output through the loggers, and writes the verification log. Its
  docstring lists the eight steps.

What the simulation is for: catching C++ errors, protocol mismatches between
a sketch and its logger, and logic errors (does the motor stop when USB is
lost?). What it is not: evidence about the real hardware. I wrote both the
firmware and the model from the same datasheet numbers, so they can agree
with each other and both be wrong about the robot.

## Adding a test

1. Copy the nearest existing test folder and rename folder, `.ino` and `.py`
   to `cvNNN_<name>`.
2. In the `.ino`: set `TEST_ID`, `TEST_TITLE`, `HEADER`; write the row in
   `loop()` (one `briRow...` call per header column, in order); write the
   checks in `finishRun()`.
3. In the `.py`: set `TEST_ID`, `EXPECTED_HEADER` (identical to the sketch),
   `start_commands()` and `evaluate()`.
4. Add a scenario for it to `SCENARIOS` in `sim/build_and_replay.py` and run
   that script.
