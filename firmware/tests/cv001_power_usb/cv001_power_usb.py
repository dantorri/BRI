#!/usr/bin/env python3
"""
CV-001  Teensy power and USB - PC logger for cv001_power_usb.ino

    python cv001_power_usb.py                       run on hardware (finds the Teensy)
    python cv001_power_usb.py --seconds 20          longer run
    python cv001_power_usb.py --rail3v3 3.29 --backfeed 0.02
    python cv001_power_usb.py --replay some_capture.txt

While it runs you can type the multimeter readings instead of using the
options:   r3.29  (3.3 V rail)    b0.02  (5 V rail with only USB connected)

Pass rules (numbers are the same as in the sketch):
    CPU temperature 0..85 C, time never goes backwards, boot counter steady
    3.3 V rail 3.20..3.40 V            (operator reading)
    back-feed voltage <= 0.30 V        (operator reading)
"""

import pathlib
import sys

# Make "import bri_logger" work no matter which folder this script is run
# from: the shared logger lives in firmware/tools, two folders up from here.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
import bri_logger as bri  # noqa: E402  (import after the path tweak, on purpose)

TEST_ID = "CV-001"
TITLE = "Teensy power and USB"
EXPECTED_HEADER = "t_ms,uptime_s,boot_count,cpu_temp_C,vbat_V,vbat_pin_V,loop_hz"

# Limits (see the comments in the sketch for why these values).
CPU_TEMP_MIN_C, CPU_TEMP_MAX_C = 0.0, 85.0
RAIL_MIN_V, RAIL_MAX_V = 3.20, 3.40
BACKFEED_MAX_V = 0.30


def add_arguments(parser):
    """Options that only this test has."""
    parser.add_argument("--rail3v3", type=float, help="3.3 V rail measured with a multimeter (volts)")
    parser.add_argument("--backfeed", type=float, help="5 V rail voltage with only USB connected (volts)")


def start_commands(args):
    """Commands sent to the Teensy to start the run."""
    commands = ["t{:g}".format(args.seconds)]
    # Readings given on the command line are passed on, so they end up in the
    # recorded capture exactly as if they had been typed.
    if args.rail3v3 is not None:
        commands.append("r{:g}".format(args.rail3v3))
    if args.backfeed is not None:
        commands.append("b{:g}".format(args.backfeed))
    commands.append("g")
    return commands


def evaluate(capture, args):
    """Judge the recorded data.  Returns a list of (verdict, text)."""
    results = []
    temperatures = capture.column("cpu_temp_C")
    temp_mean = bri.mean(temperatures)
    results.append(("PASS" if bri.within(temp_mean, CPU_TEMP_MIN_C, CPU_TEMP_MAX_C) else "FAIL",
                    "CPU temperature {:.1f} C over {} rows (limits {:.0f}..{:.0f})".format(
                        temp_mean, len(temperatures), CPU_TEMP_MIN_C, CPU_TEMP_MAX_C)))

    reset_reasons = capture.reset_seen()
    if reset_reasons:
        results.append(("FAIL", "boot not stable: " + "; ".join(reset_reasons)))
    elif not capture.rows:
        results.append(("FAIL", "no data rows"))
    else:
        results.append(("PASS", "stable boot: time always increasing, boot counter steady at {:.0f}".format(
            capture.column("boot_count")[0])))

    vbat = bri.mean(capture.column("vbat_V"))
    results.append(("INFO", "battery sense reads {:.2f} V (about 0 V is correct when only USB is connected)".format(vbat)))
    results.append(("INFO", "loop rate about {:.0f} passes per second".format(bri.mean(capture.column("loop_hz")))))

    # Operator readings: command-line option first, otherwise what was typed.
    rail = args.rail3v3 if args.rail3v3 is not None else capture.last_event_value("DMM", "rail3v3")
    if rail is None:
        results.append(("TODO", "3.3 V rail not entered: measure the 3.3V pin and re-run with --rail3v3 <volts>"))
    else:
        results.append(("PASS" if bri.within(rail, RAIL_MIN_V, RAIL_MAX_V) else "FAIL",
                        "3.3 V rail {:.3f} V (limits {:.2f}..{:.2f})".format(rail, RAIL_MIN_V, RAIL_MAX_V)))
    backfeed = args.backfeed if args.backfeed is not None else capture.last_event_value("DMM", "backfeed")
    if backfeed is None:
        results.append(("TODO", "USB back-feed not entered: with only USB connected measure the 5 V rail, --backfeed <volts>"))
    else:
        results.append(("PASS" if backfeed <= BACKFEED_MAX_V else "FAIL",
                        "USB back-feed {:.3f} V (limit {:.2f})".format(backfeed, BACKFEED_MAX_V)))
    return results


if __name__ == "__main__":
    sys.exit(bri.run_test(
        TEST_ID, TITLE, EXPECTED_HEADER, evaluate, start_commands,
        default_seconds=10.0,
        operator_notes="Watch the LED: slow blink = idle, fast blink = running.\n"
                       "Type r<volts> for the 3.3 V rail and b<volts> for the back-feed reading.",
        plot_columns=("cpu_temp_C", "vbat_V", "loop_hz"),
        add_arguments=add_arguments))
