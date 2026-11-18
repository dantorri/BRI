#!/usr/bin/env python3
"""
CV-002  5 V regulator under stepped load - PC logger for cv002_5v_regulator.ino

    python cv002_5v_regulator.py                 run on hardware (default 300 s; type s to end)
    python cv002_5v_regulator.py --replay some_capture.txt

During the run, for each load step:
    i2.0     the load current in amps      (type this first)
    v4.98    the regulator output in volts (records the step)
    s        when you have done all the steps

Pass rules:
    every recorded output voltage is 4.80..5.20 V
    the Teensy did not reset (boot counter steady, time never goes backwards,
    serial link never dropped)
"""

import pathlib
import sys

# Make "import bri_logger" work no matter which folder this script is run
# from: the shared logger lives in firmware/tools, two folders up from here.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
import bri_logger as bri  # noqa: E402  (import after the path tweak, on purpose)

TEST_ID = "CV-002"
TITLE = "5 V regulator"
EXPECTED_HEADER = "t_ms,uptime_s,boot_count,vbat_V,cpu_temp_C,vout_dmm_V,iout_dmm_A,entries"

VOUT_MIN_V, VOUT_MAX_V = 4.80, 5.20


def start_commands(args):
    """Commands sent to the Teensy to start the run."""
    return ["t{:g}".format(args.seconds), "g"]


def evaluate(capture, args):
    """Judge the recorded data.  Returns a list of (verdict, text)."""
    results = []

    # Each "v" the operator typed produced one "!DMM" event carrying the
    # output voltage, the load current and the battery voltage at that moment.
    steps = [event["values"] for event in capture.events_named("DMM") if "vout" in event["values"]]
    if not steps:
        results.append(("TODO", "no load steps recorded: during the run type i<amps> then v<volts> for each load"))
    else:
        for index, step in enumerate(steps, start=1):
            vout = step.get("vout", float("nan"))
            iout = step.get("iout", float("nan"))
            ok = bri.within(vout, VOUT_MIN_V, VOUT_MAX_V)
            results.append(("PASS" if ok else "FAIL",
                            "load step {}: {:.3f} V at {:.2f} A, battery {:.2f} V (limits {:.2f}..{:.2f} V)".format(
                                index, vout, iout, step.get("vbat", float("nan")), VOUT_MIN_V, VOUT_MAX_V)))

    reset_reasons = capture.reset_seen()
    if capture.header_count > 1:
        reset_reasons.append("the banner/header appeared {} times".format(capture.header_count))
    if reset_reasons:
        results.append(("FAIL", "Teensy reset during the test: " + "; ".join(reset_reasons)))
    elif capture.rows:
        duration = (capture.column("t_ms")[-1] - capture.column("t_ms")[0]) / 1000.0
        results.append(("PASS", "no reset in {:.0f} s of logging".format(duration)))
    else:
        results.append(("FAIL", "no data rows"))

    vbat = capture.column("vbat_V")
    if vbat:
        results.append(("INFO", "battery/input voltage {:.2f}..{:.2f} V during the test".format(min(vbat), max(vbat))))
    return results


if __name__ == "__main__":
    sys.exit(bri.run_test(
        TEST_ID, TITLE, EXPECTED_HEADER, evaluate, start_commands,
        default_seconds=300.0,
        operator_notes="For each load: set it, type i<amps>, then v<volts>.  Type s when finished.\n"
                       "Also feel the regulator: it may get warm, it must not shut down.",
        plot_columns=("vbat_V", "vout_dmm_V", "iout_dmm_A", "cpu_temp_C")))
