#!/usr/bin/env python3
"""
CV-009  Encoder count calibration - PC logger for cv009_encoder_cal.ino

BY HAND (drivers stay asleep, nothing moves by itself):
    python cv009_encoder_cal.py --channel 0
        line the tape mark up, type b, turn the wheel exactly one turn FORWARD,
        type e; repeat b ... e turning BACKWARD; type s to finish.

BY MOTOR (!! the motor will turn; you must type GO first):
    python cv009_encoder_cal.py --channel 0 --motor-revs 5
        count the turns of the tape mark, then type what you saw, e.g. o5.0,
        then s to finish.

    python cv009_encoder_cal.py --replay some_capture.txt
    python cv009_encoder_cal.py --first backward     if your first hand turn was backward

Pass rules:
    each measured revolution is within 1 % of 8400 counts
    the first hand turn (robot-forward unless --first backward) counts positive
"""

import pathlib
import sys

# Make "import bri_logger" work no matter which folder this script is run
# from: the shared logger lives in firmware/tools, two folders up from here.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
import bri_logger as bri  # noqa: E402  (import after the path tweak, on purpose)

TEST_ID = "CV-009"
TITLE = "Encoder count calibration"
EXPECTED_HEADER = "t_ms,mode,counts,rev,rpm,duty,flt_ok"

COUNTS_PER_REV = 8400.0
TOLERANCE_PCT = 1.0


def add_arguments(parser):
    """Options that only this test has."""
    parser.add_argument("--channel", type=int, choices=(0, 1, 2, 3), default=0,
                        help="motor channel: 0=LW 1=RW 2=LH 3=RH (default 0)")
    parser.add_argument("--first", choices=("forward", "backward"), default="forward",
                        help="which way you turned the wheel on the FIRST hand revolution (default forward)")
    parser.add_argument("--motor-revs", type=float,
                        help="also do a motor-driven move of this many turns (0.5..10, negative = reverse)")


def start_commands(args):
    """Commands sent to the Teensy to start the run."""
    commands = ["c{}".format(args.channel), "t{:g}".format(args.seconds), "h"]
    if args.motor_revs is not None:
        commands.append("m{:g}".format(args.motor_revs))
        commands.append("g")
    return commands


def evaluate(capture, args):
    """Judge the recorded data.  Returns a list of (verdict, text)."""
    results = []

    # A safety stop during a motor move is a failure ("complete" is the normal
    # end of a move and "operator" is the s that ends the test).
    bad_stops = [reason for reason in capture.stop_reasons() if reason not in ("complete", "operator")]
    if bad_stops:
        results.append(("FAIL", "motor move was stopped: " + ", ".join(bad_stops)))

    # Hand-turned revolutions: one "!REV" event each.
    hand = [event["values"] for event in capture.events_named("REV")]
    for index, revolution in enumerate(hand, start=1):
        counts = revolution.get("counts", float("nan"))
        error_pct = (abs(counts) - COUNTS_PER_REV) / COUNTS_PER_REV * 100.0
        results.append(("PASS" if abs(error_pct) <= TOLERANCE_PCT else "FAIL",
                        "hand revolution {}: {:+.0f} counts ({:+.2f} % from 8400, tolerance {:.0f} %)".format(
                            index, counts, error_pct, TOLERANCE_PCT)))
    if hand:
        first_positive = hand[0].get("counts", 0) > 0
        expected_positive = (args.first == "forward")
        if first_positive == expected_positive:
            results.append(("PASS", "sign: the first hand turn ({}) counted {}, as it should".format(
                args.first, "up" if first_positive else "down")))
        else:
            results.append(("FAIL", "sign: the first hand turn ({}) counted {}: flip BRI_ENC_SIGN for this "
                                    "channel in BRI_Config.h".format(args.first, "up" if first_positive else "down")))

    # Motor-driven moves: "!MOVE" when the move ends, "!MOVECAL" once the
    # operator has typed the observed number of turns.
    moves = capture.events_named("MOVE")
    calibrations = [event["values"] for event in capture.events_named("MOVECAL")]
    for index, calibration in enumerate(calibrations, start=1):
        counts = abs(calibration.get("counts", float("nan")))
        observed = abs(calibration.get("observed_rev", float("nan")))
        per_rev = counts / observed if observed else float("nan")
        error_pct = (per_rev - COUNTS_PER_REV) / COUNTS_PER_REV * 100.0
        results.append(("PASS" if abs(error_pct) <= TOLERANCE_PCT else "FAIL",
                        "motor move {}: {:.0f} counts over {:.2f} observed turns = {:.1f} counts/rev ({:+.2f} %)".format(
                            index, counts, observed, per_rev, error_pct)))
    if len(moves) > len(calibrations):
        results.append(("TODO", "{} motor move(s) finished without an observed turn count: type o<revs> after "
                                "the move".format(len(moves) - len(calibrations))))
    for move in moves:
        results.append(("INFO", "motor move coasted {:.0f} counts past its target while braking".format(
            move["values"].get("overshoot_counts", float("nan")))))

    if not hand and not moves and not bad_stops:
        results.append(("TODO", "nothing was measured: use b ... e by hand, or --motor-revs"))
    return results


if __name__ == "__main__":
    # The GO confirmation is only needed when a motor move was asked for.
    motor_requested = any(argument.startswith("--motor-revs") for argument in sys.argv[1:])
    notes = ("Put a tape mark on the wheel.  By hand: b at the start of one full turn, e at the end\n"
             "(forward first, then backward).  Type s to finish.")
    if motor_requested:
        notes = ("!! THE MOTOR WILL TURN.  Count the turns of the tape mark, then type o<turns> "
                 "(e.g. o5.0) and s to finish.\n" + notes)
    sys.exit(bri.run_test(
        TEST_ID, TITLE, EXPECTED_HEADER, evaluate, start_commands,
        default_seconds=120.0, motor_test=motor_requested,
        operator_notes=notes,
        plot_columns=("counts", "rev", "rpm", "duty"),
        add_arguments=add_arguments))
