#!/usr/bin/env python3
"""
CV-008  Encoder voltage and waveform - PC logger for cv008_encoder_waveform.ino

!! THE MOTOR WILL TURN (slowly).

    python cv008_encoder_waveform.py --channel 0                 hold 15 % duty for 8 s
    python cv008_encoder_waveform.py --channel 1 --seconds 30    more time on the scope
    python cv008_encoder_waveform.py --channel 0 --duty -0.15    the other direction
    python cv008_encoder_waveform.py --channel 0 --vhigh 3.28    scope reading given up front
    python cv008_encoder_waveform.py --replay some_capture.txt

While the motor turns, scope encoder A and B at the Teensy pins (the banner
names them) and type the highest level you see:  v3.28

Pass rules:
    both A and B toggle (at least 20 edges each seen by the firmware's own sampling)
    the count moves one way only, and that way matches the sign of the duty
    scope level at the Teensy pins 2.8..3.5 V  (operator reading; never above 3.6 V)
    FLT never low; no safety stop
"""

import pathlib
import sys

# Make "import bri_logger" work no matter which folder this script is run
# from: the shared logger lives in firmware/tools, two folders up from here.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
import bri_logger as bri  # noqa: E402  (import after the path tweak, on purpose)

TEST_ID = "CV-008"
TITLE = "Encoder voltage and waveform"
EXPECTED_HEADER = "t_ms,duty,a_level,b_level,a_edges,b_edges,bad_steps,poll_net,counts,rpm,a_high_pct,b_high_pct,flt_ok"

MIN_EDGES = 20
VHIGH_MIN_V, VHIGH_MAX_V = 2.8, 3.5


def add_arguments(parser):
    """Options that only this test has."""
    parser.add_argument("--channel", type=int, choices=(0, 1, 2, 3), default=0,
                        help="motor channel: 0=LW 1=RW 2=LH 3=RH (default 0)")
    parser.add_argument("--duty", type=float, default=0.15, help="constant duty -0.35..0.35 (default 0.15)")
    parser.add_argument("--vhigh", type=float, help="highest scope voltage at the Teensy encoder pins (volts)")


def start_commands(args):
    """Commands sent to the Teensy to start the run."""
    commands = ["c{}".format(args.channel), "d{:g}".format(args.duty), "t{:g}".format(args.seconds)]
    if args.vhigh is not None:
        commands.append("v{:g}".format(args.vhigh))
    commands.append("g")
    return commands


def evaluate(capture, args):
    """Judge the recorded data.  Returns a list of (verdict, text)."""
    results = []
    if not capture.rows:
        return [("FAIL", "no data rows")]
    bad_stops = [reason for reason in capture.stop_reasons() if reason != "complete"]
    if bad_stops:
        results.append(("FAIL", "run was stopped: " + ", ".join(bad_stops)))

    last = capture.rows[-1]
    hardware_backend = any("encoder_backend=hw_quadenc" in comment for comment in capture.comments)

    # Both channels toggling (seen by the firmware sampling the pins).
    edges = "A {:.0f} edges, B {:.0f} edges".format(last["a_edges"], last["b_edges"])
    if hardware_backend:
        results.append(("TODO", "hardware encoder backend: pin levels are not reliably readable ({}); "
                                "confirm A and B on the scope".format(edges)))
    else:
        both = last["a_edges"] >= MIN_EDGES and last["b_edges"] >= MIN_EDGES
        results.append(("PASS" if both else "FAIL", "both channels toggle: {} (minimum {})".format(edges, MIN_EDGES)))

    # One direction only, and the right one.
    counts = capture.column("counts")
    steps = [later - earlier for earlier, later in zip(counts, counts[1:]) if later != earlier]
    duty = bri.mean([row["duty"] for row in capture.rows if row["duty"] != 0])
    forward_steps = sum(1 for step in steps if step > 0)
    backward_steps = sum(1 for step in steps if step < 0)
    total = counts[-1] - counts[0]
    one_way = (forward_steps == 0 or backward_steps == 0) and len(steps) > 0
    sign_ok = (duty > 0 and total > 0) or (duty < 0 and total < 0)
    results.append(("PASS" if one_way and sign_ok and abs(total) >= MIN_EDGES else "FAIL",
                    "direction: duty {:+.2f} gave {:+.0f} counts; {} rows up, {} rows down".format(
                        duty, total, forward_steps, backward_steps)))

    # Signal quality, for the record.
    if not hardware_backend:
        ratio = abs(last["poll_net"]) / abs(total) if total else float("nan")
        results.append(("INFO", "A high {:.1f} %, B high {:.1f} % of the time (about 50 each is square); "
                                "{:.0f} double steps; sampled steps / library counts = {:.3f}".format(
                                    last["a_high_pct"], last["b_high_pct"], last["bad_steps"], ratio)))
    rpm = [row["rpm"] for row in capture.rows[len(capture.rows) // 2:]]
    results.append(("INFO", "steady speed {:.1f} rpm (spread {:.2f})".format(bri.mean(rpm), bri.stdev(rpm))))

    # Scope reading.
    vhigh = args.vhigh if args.vhigh is not None else capture.last_event_value("SCOPE", "vhigh")
    if vhigh is None:
        results.append(("TODO", "scope level not entered: measure A and B at the Teensy pins, --vhigh <volts>"))
    else:
        results.append(("PASS" if bri.within(vhigh, VHIGH_MIN_V, VHIGH_MAX_V) else "FAIL",
                        "logic high at the Teensy pins {:.2f} V (limits {:.1f}..{:.1f}; the Teensy is not 5 V tolerant)".format(
                            vhigh, VHIGH_MIN_V, VHIGH_MAX_V)))

    faults = sum(1 for row in capture.rows if row["flt_ok"] != 1)
    results.append(("PASS" if faults == 0 else "FAIL", "FLT low in {} of {} rows".format(faults, len(capture.rows))))
    return results


if __name__ == "__main__":
    sys.exit(bri.run_test(
        TEST_ID, TITLE, EXPECTED_HEADER, evaluate, start_commands,
        default_seconds=8.0, motor_test=True,
        operator_notes="!! THE MOTOR WILL TURN slowly.  Scope A and B at the Teensy pins while it runs,\n"
                       "then type v<volts> with the highest level you saw.  Type s to stop early.",
        plot_columns=("counts", "rpm", "a_edges", "b_edges"),
        add_arguments=add_arguments))
