#!/usr/bin/env python3
"""
CV-007  Motor no-load test - PC logger for cv007_motor_noload.ino

!! THE MOTOR WILL TURN.  Secure it, wheel off or clear of everything.

    python cv007_motor_noload.py --channel 0             left wheel motor
    python cv007_motor_noload.py --channel 1 --duty 0.2  right wheel, gentler
    python cv007_motor_noload.py --channel 0 --plot      also save a PNG
    python cv007_motor_noload.py --replay some_capture.txt

The motion takes about 14 s: ramp up forward, hold, ramp down, pause, the
same in reverse.  You must type GO before anything moves.

Pass rules:
    encoder speed positive during the forward hold, negative during the reverse hold
    current during both holds 0.05..0.60 A (datasheet no-load 0.2 A)
    FLT never low; no safety stop
"""

import pathlib
import sys

# Make "import bri_logger" work no matter which folder this script is run
# from: the shared logger lives in firmware/tools, two folders up from here.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
import bri_logger as bri  # noqa: E402  (import after the path tweak, on purpose)

TEST_ID = "CV-007"
TITLE = "Motor no-load test"
EXPECTED_HEADER = "t_ms,phase,duty,dir_cmd,cs_peak_mV,cs_mean_mV,i_A,vbat_V,counts,rpm,flt_ok,en_sense"

PHASE_HOLD_FORWARD, PHASE_HOLD_REVERSE = 2, 6
NOLOAD_MIN_A, NOLOAD_MAX_A = 0.05, 0.60
MOVING_RPM = 2.0
NOLOAD_RPM_AT_12V = 76.0


def add_arguments(parser):
    """Options that only this test has."""
    parser.add_argument("--channel", type=int, choices=(0, 1, 2, 3), default=0,
                        help="motor channel: 0=LW 1=RW 2=LH 3=RH (default 0)")
    parser.add_argument("--duty", type=float, default=0.30, help="peak duty 0.05..0.35 (default 0.30)")


def start_commands(args):
    """Commands sent to the Teensy to start the run."""
    return ["c{}".format(args.channel), "d{:g}".format(args.duty), "g"]


def hold_rows(capture, phase):
    """Rows of one hold phase, second half only (speed and current settled)."""
    rows = [row for row in capture.rows if row["phase"] == phase]
    return rows[len(rows) // 2:]


def evaluate(capture, args):
    """Judge the recorded data.  Returns a list of (verdict, text)."""
    results = []
    bad_stops = [reason for reason in capture.stop_reasons() if reason != "complete"]
    forward, reverse = hold_rows(capture, PHASE_HOLD_FORWARD), hold_rows(capture, PHASE_HOLD_REVERSE)
    if bad_stops:
        results.append(("FAIL", "run was stopped: " + ", ".join(bad_stops)))
    if not forward or not reverse:
        results.append(("FAIL", "the capture does not contain both hold phases"))
        return results
    if not bad_stops:
        results.append(("PASS", "motion profile completed"))

    # Direction, from the encoder.
    rpm_fwd = bri.mean([row["rpm"] for row in forward])
    rpm_rev = bri.mean([row["rpm"] for row in reverse])
    speeds = "{:+.1f} rpm on positive duty, {:+.1f} rpm on negative duty".format(rpm_fwd, rpm_rev)
    if rpm_fwd > MOVING_RPM and rpm_rev < -MOVING_RPM:
        results.append(("PASS", "direction: " + speeds))
    elif abs(rpm_fwd) <= MOVING_RPM and abs(rpm_rev) <= MOVING_RPM:
        results.append(("TODO", "encoder saw no motion ({}): if the encoder is not wired yet, confirm by eye "
                                "that positive duty turned the wheel robot-forward".format(speeds)))
    else:
        results.append(("FAIL", "direction: " + speeds + ".  If the WHEEL turned backward on positive duty flip "
                                "BRI_MOTOR_SIGN for this channel; if the wheel was right, flip BRI_ENC_SIGN"))

    # No-load current.
    amps_fwd = bri.mean([row["i_A"] for row in forward])
    amps_rev = bri.mean([row["i_A"] for row in reverse])
    current_ok = bri.within(amps_fwd, NOLOAD_MIN_A, NOLOAD_MAX_A) and bri.within(amps_rev, NOLOAD_MIN_A, NOLOAD_MAX_A)
    results.append(("PASS" if current_ok else "FAIL",
                    "no-load current {:.2f} A forward, {:.2f} A reverse (limits {:.2f}..{:.2f}, datasheet 0.2)".format(
                        amps_fwd, amps_rev, NOLOAD_MIN_A, NOLOAD_MAX_A)))

    # Fault pin over the whole run.
    faults = sum(1 for row in capture.rows if row["flt_ok"] != 1)
    results.append(("PASS" if faults == 0 else "FAIL", "FLT low in {} of {} rows".format(faults, len(capture.rows))))

    # Speed against the simple prediction, and the supply under load.
    duty = bri.mean([row["duty"] for row in forward])
    vbat = bri.mean([row["vbat_V"] for row in forward])
    predicted = duty * NOLOAD_RPM_AT_12V * vbat / 12.0
    results.append(("INFO", "at duty {:.2f} and {:.2f} V: predicted about {:.1f} rpm, measured {:.1f} rpm".format(
        duty, vbat, predicted, rpm_fwd)))
    vbat_all = capture.column("vbat_V")
    results.append(("INFO", "battery {:.2f}..{:.2f} V during the run".format(min(vbat_all), max(vbat_all))))

    # Cross-check of the current-sense method: the average CS voltage should
    # be about duty x the peak CS voltage (CS is only present during on-time).
    peak = bri.mean([row["cs_peak_mV"] for row in forward])
    average = bri.mean([row["cs_mean_mV"] for row in forward])
    if peak > 0:
        results.append(("INFO", "CS average/peak = {:.2f} (expect about the duty, {:.2f}, if CS is zero "
                                "outside the PWM on-time)".format(average / peak, duty)))
    return results


if __name__ == "__main__":
    sys.exit(bri.run_test(
        TEST_ID, TITLE, EXPECTED_HEADER, evaluate, start_commands,
        default_seconds=16.0, motor_test=True,
        operator_notes="!! THE MOTOR WILL TURN: secure it, wheel off or clear.  Kill switch in RUN.\n"
                       "Watch which way it turns on the first (positive) ramp.  Type s to stop early.",
        plot_columns=("duty", "i_A", "rpm", "vbat_V"),
        add_arguments=add_arguments))
