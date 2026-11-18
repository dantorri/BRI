# Later phases: jumping (ODrive S1 + M8325s) and more computation (Raspberry Pi 5)

Nothing here is built. This file records what the current design reserves so these phases do not force a redesign.

## Phase 3: jump actuator (candidate, not bought)

### What the first-order model says

From `analysis/jump_first_order_estimate.csv` (4.5 kg robot, 80 A peak, guessed rotor inertia, no friction; rankings, not predictions):

| Mechanism with one M8325s | Wheel clearance at 24 V |
|---|---|
| Direct-drive knee, 60 to 160 degrees | 10 cm |
| Same knee with a parallel spring (6 N*m at crouch) | 16.5 cm |
| Constant-ratio linear drive, 97 mm stroke, 15 to 20 mm effective pinion radius | 39 to 41 cm |

### Constraints that shape the design

- **Own battery.** The ODrive S1 needs at least 12 V and is specified for 16 to 48 V. A 3S pack is below that for most of its discharge, so the jump actuator gets its own higher-voltage pack (6S is the working assumption) with a common ground.
- **Current.** S1: 40 A continuous with a heat spreader, 80 A peak, with the peak reduced by firmware as bus voltage rises. Check the datasheet curve at the chosen voltage.
- **Mass.** The motor alone is 840 g. The wheels reach only 0.44 m/s, so balance and landing recovery at the higher mass must be shown first.
- **Hip gearboxes.** Landing loads must not pass through the 2.45 N*m hip gearboxes. Lock the legs, or replace them with the jump leg.

### What is already reserved

| Item | Where |
|---|---|
| Motor mount: M4 on a 40 mm circle, 22 mm centre hole | `deck_top` in `CAD/bri_chassis.py` |
| Mass stand-in: bolt-on cup for 1.5 kg of steel shot or nuts | `ballast_cup` |
| CAN bus pins 22 (TX) and 23 (RX), needs a 3.3 V CAN transceiver | `PIN_MAP.md` |
| Purchase gate: free balance with 1.5 kg ballast | workbook test SYSV-004, requirement JMP-002 |

### Order of work

1. Pass SYSV-004 (ballasted balance).
2. Build a single-axis bench rig: motor, linear stage and dummy mass on a rail, 6S supply. Measure hop height and landing load.
3. Choose knee or linear drive from the measurements.
4. Integrate: CAN link with a heartbeat timeout, then a balance, crouch, thrust, flight, land sequence.

## Phase 4: Raspberry Pi 5 and cameras

| Topic | Plan |
|---|---|
| Role | Vision and route decisions only. The Teensy keeps the balance loop and must keep balancing if the Pi stops (CMP-001) |
| Power | A dedicated 5 V rail from the spare D24V90F5 (CMP-002). The Pi 5 can draw up to 5 A (as recalled; confirm), so it does not share the Teensy and encoder rail |
| Link | USB serial to the Teensy as the main link. Serial8 on pins 34 and 35 is reserved as a fallback |
| Commands | Low-rate set-points (forward speed, turn rate, stance height) with a timeout: no command for a set time means stop and hold balance |
| Mount | 58 x 49 mm M2.5 pattern on `deck_top` |
| Effect on the hips | The Pi adds mass high on the body, which widens the usable hip range (see `JOINT_DESIGN.md`) |

Sources: ODrive S1 datasheet and ODrive motor guide (docs.odriverobotics.com), ODrive shop page for the M8325s.
