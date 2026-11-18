#!/usr/bin/env python3
"""
CV-006  Motor driver logic - PC logger for cv006_driver_logic.ino

!! MOTOR DISCONNECTED (preferred) OR SHAFT RESTRAINED !!

    python cv006_driver_logic.py --channel 0         left wheel driver, 1 s per step
    python cv006_driver_logic.py --channel 1 --seconds 35    5 s per step (time to probe with a meter)
    python cv006_driver_logic.py --channel 0 --motor-connected   motor plugged in and restrained
    python cv006_driver_logic.py --replay some_capture.txt

Channels: 0 = LW, 1 = RW, 2 = LH, 3 = RH.  --seconds is the whole test; it is
divided by the seven steps.

Pass rules:
    MOTOR_EN_SENSE is low while the driver is asleep and high while awake
    FLT never goes low while the driver is awake
    CS reads 30..80 mV while driving with the motor unplugged (nominal 50 mV);
        with --motor-connected the reading is offset plus current, so it is
        reported (as an implied current) but not judged as an offset
    the run completes without a safety stop
"""

import pathlib
import sys

# Make "import bri_logger" work no matter which folder this script is run
# from: the shared logger lives in firmware/tools, two folders up from here.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
import bri_logger as bri  # noqa: E402  (import after the path tweak, on purpose)

TEST_ID = "CV-006"
TITLE = "Motor driver logic"
EXPECTED_HEADER = "t_ms,step,en_cmd,en_sense,dir_cmd,duty,flt_ok,cs_peak_mV,cs_mean_mV,i_A,vbat_V"

STEP_COUNT = 7
AWAKE_STEPS = (1, 2, 3, 4, 5)
ASLEEP_STEPS = (0, 6)
STEP_FORWARD, STEP_REVERSE = 2, 4
CS_OFFSET_MIN_MV, CS_OFFSET_MAX_MV = 30.0, 80.0


def add_arguments(parser):
    """Options that only this test has."""
    parser.add_argument("--channel", type=int, choices=(0, 1, 2, 3), default=0,
                        help="motor channel: 0=LW 1=RW 2=LH 3=RH (default 0)")
    parser.add_argument("--motor-connected", action="store_true",
                        help="say this if a (restrained) motor is plugged in; default assumes it is unplugged")


def start_commands(args):
    """Commands sent to the Teensy to start the run."""
    seconds_per_step = max(0.5, args.seconds / STEP_COUNT)
    return ["c{}".format(args.channel), "m{}".format(1 if args.motor_connected else 0),
            "t{:g}".format(seconds_per_step), "g"]


def motor_stated_connected(capture, args):
    """Was a motor declared connected?  Either by the option, or recorded in
    the capture itself (the sketch prints 'motor_connected=1' at the start of
    the run), so that --replay gives the same answer without the option."""
    if args.motor_connected:
        return True
    return any("motor_connected=1" in comment for comment in capture.comments)


def settled_rows(capture, step):
    """Rows of one step, second half only (first half = still switching)."""
    rows = [row for row in capture.rows if row["step"] == step]
    return rows[len(rows) // 2:]


def evaluate(capture, args):
    """Judge the recorded data.  Returns a list of (verdict, text)."""
    results = []
    if not capture.rows:
        return [("FAIL", "no data rows")]

    # Did the run get through all seven steps without a safety stop?
    bad_stops = [reason for reason in capture.stop_reasons() if reason != "complete"]
    last_step = max(capture.column("step"))
    if bad_stops:
        results.append(("FAIL", "run was stopped: " + ", ".join(bad_stops) +
                        ("  (kill switch in STOP?)" if "kill_switch" in bad_stops else "")))
    elif last_step < STEP_COUNT - 1:
        results.append(("FAIL", "run ended at step {:.0f} of {}".format(last_step, STEP_COUNT - 1)))
    else:
        results.append(("PASS", "all {} steps completed".format(STEP_COUNT)))

    # Enable read-back.
    has_sense = any(row["en_sense"] >= 0 for row in capture.rows)
    if not has_sense:
        results.append(("INFO", "no enable read-back on this channel (the pin map has no sense pin for HIP_EN)"))
    else:
        wrong_asleep = sum(1 for step in ASLEEP_STEPS for row in settled_rows(capture, step) if row["en_sense"] != 0)
        wrong_awake = sum(1 for step in AWAKE_STEPS for row in settled_rows(capture, step) if row["en_sense"] != 1)
        results.append(("PASS" if wrong_asleep == 0 and wrong_awake == 0 else "FAIL",
                        "MOTOR_EN_SENSE follows the command ({} wrong rows asleep, {} wrong rows awake)".format(
                            wrong_asleep, wrong_awake)))

    # Fault pin while awake.
    awake = [row for step in AWAKE_STEPS for row in settled_rows(capture, step)]
    faults = sum(1 for row in awake if row["flt_ok"] != 1)
    results.append(("PASS" if awake and faults == 0 else "FAIL",
                    "FLT while awake: {} fault rows out of {}".format(faults, len(awake))))

    # DIR and PWM commands really changed between the two driving steps.
    forward, reverse = settled_rows(capture, STEP_FORWARD), settled_rows(capture, STEP_REVERSE)
    if forward and reverse:
        duty_fwd = bri.mean([row["duty"] for row in forward])
        duty_rev = bri.mean([row["duty"] for row in reverse])
        dir_changed = forward[-1]["dir_cmd"] != reverse[-1]["dir_cmd"]
        results.append(("PASS" if dir_changed and duty_fwd > 0 > duty_rev else "FAIL",
                        "commands: duty {:+.2f} then {:+.2f}, DIR pin {:.0f} then {:.0f}".format(
                            duty_fwd, duty_rev, forward[-1]["dir_cmd"], reverse[-1]["dir_cmd"])))

        # Current sense.
        cs_fwd = bri.mean([row["cs_peak_mV"] for row in forward])
        cs_rev = bri.mean([row["cs_peak_mV"] for row in reverse])
        text = "CS while driving: {:.1f} mV forward, {:.1f} mV reverse".format(cs_fwd, cs_rev)
        if min(cs_fwd, cs_rev) < CS_OFFSET_MIN_MV:
            results.append(("FAIL", text + " - CS is not responding (expected about 50 mV or more)"))
        elif motor_stated_connected(capture, args):
            implied = [(value / 1000.0 - 0.050) / 0.020 for value in (cs_fwd, cs_rev)]
            results.append(("INFO", text + " - motor stated connected, so this is offset plus current "
                                           "(implies {:.2f} A / {:.2f} A); zero offset not read".format(*implied)))
        elif max(cs_fwd, cs_rev) > CS_OFFSET_MAX_MV:
            results.append(("FAIL", text + " - too high for an unplugged motor: is a motor connected "
                                           "(then use --motor-connected) or the output shorted?"))
        else:
            results.append(("PASS", text + " = the zero offset (nominal 50 mV; valid because the motor "
                                           "is stated to be unplugged)"))

    asleep = [row for step in ASLEEP_STEPS for row in settled_rows(capture, step)]
    if asleep:
        results.append(("INFO", "while asleep: CS {:.1f} mV, FLT low in {} of {} rows (not judged)".format(
            bri.mean([row["cs_peak_mV"] for row in asleep]),
            sum(1 for row in asleep if row["flt_ok"] != 1), len(asleep))))
    return results


if __name__ == "__main__":
    sys.exit(bri.run_test(
        TEST_ID, TITLE, EXPECTED_HEADER, evaluate, start_commands,
        default_seconds=7.0, motor_test=True,
        operator_notes="!! Motor DISCONNECTED (preferred) or shaft restrained.  Kill switch in RUN.\n"
                       "The driver is pulsed at 10 % duty for one step in each direction.",
        plot_columns=("step", "en_sense", "duty", "cs_peak_mV", "flt_ok"),
        add_arguments=add_arguments))
