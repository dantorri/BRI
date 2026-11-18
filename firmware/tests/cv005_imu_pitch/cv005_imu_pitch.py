#!/usr/bin/env python3
"""
CV-005  IMU pitch orientation - PC logger for cv005_imu_pitch.ino

    python cv005_imu_pitch.py                    run on hardware (20 s)
    python cv005_imu_pitch.py --first backward   if you tilted BACKWARD first
    python cv005_imu_pitch.py --plot             save a PNG of the three pitch estimates
    python cv005_imu_pitch.py --replay some_capture.txt

What to do: hold the robot upright and still for 2 s, then tilt it FORWARD
20-30 degrees, back to upright, then BACKWARD, slowly.

Pass rules:
    a tilt of at least 10 degrees was seen
    the first tilt has the expected sign (forward = positive pitch)
    the gyro rate and the change of accelerometer pitch agree in direction
    no I2C errors
The zero (pitch while upright) is reported so it can go into BRI_Config.h.
"""

import pathlib
import sys

# Make "import bri_logger" work no matter which folder this script is run
# from: the shared logger lives in firmware/tools, two folders up from here.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
import bri_logger as bri  # noqa: E402  (import after the path tweak, on purpose)

import math  # noqa: E402

TEST_ID = "CV-005"
TITLE = "IMU pitch orientation"
EXPECTED_HEADER = "t_ms,phase,pitch_acc_deg,pitch_gyro_deg,pitch_cf_deg,rate_dps,ax_g,ay_g,az_g,i2c_err"

TILT_DETECT_DEG = 10.0     # smallest tilt that counts as deliberate
CORRELATION_MIN = 0.5      # how strongly gyro rate and accel-pitch slope must agree (1.0 = perfectly)
SLOPE_HALF_WINDOW = 5      # rows either side used to measure the slope (5 rows = 0.1 s each way)


def add_arguments(parser):
    """Options that only this test has."""
    parser.add_argument("--first", choices=("forward", "backward"), default="forward",
                        help="which way you tilted the robot FIRST (default forward)")


def start_commands(args):
    """Commands sent to the Teensy to start the run."""
    return ["t{:g}".format(args.seconds), "g"]


def correlation(xs, ys):
    """Pearson correlation of two equally long lists: +1 = move together,
    -1 = move opposite, 0 = unrelated."""
    if len(xs) < 3:
        return 0.0
    mean_x, mean_y = bri.mean(xs), bri.mean(ys)
    top = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    bottom = math.sqrt(sum((x - mean_x) ** 2 for x in xs) * sum((y - mean_y) ** 2 for y in ys))
    return top / bottom if bottom > 0 else 0.0


def evaluate(capture, args):
    """Judge the recorded data.  Returns a list of (verdict, text)."""
    results = []
    still = [row for row in capture.rows if row["phase"] == 0]
    tilt = [row for row in capture.rows if row["phase"] == 1]
    if not still or not tilt:
        return [("FAIL", "the capture does not contain both the hold-still and the tilt phase")]

    # Zero reference and gyro offset, from the hold-still phase.
    zero = bri.mean([row["pitch_acc_deg"] for row in still])
    bias = bri.mean([row["rate_dps"] for row in still])
    results.append(("INFO", "zero: pitch reads {:+.2f} deg when upright; gyro offset {:+.3f} deg/s".format(zero, bias)))

    # Size and direction of the tilts, relative to the zero.
    deviations = [row["pitch_cf_deg"] - zero for row in tilt]
    largest_forward, largest_backward = max(deviations), min(deviations)
    first_sign = 0
    for deviation in deviations:
        if abs(deviation) > TILT_DETECT_DEG:
            first_sign = 1 if deviation > 0 else -1
            break
    if first_sign == 0:
        results.append(("TODO", "no tilt of {:.0f} deg or more was seen (largest {:+.1f} / {:+.1f}): "
                                "repeat and tilt further".format(TILT_DETECT_DEG, largest_forward, largest_backward)))
        return results
    results.append(("INFO", "tilt range {:+.1f} .. {:+.1f} deg about the zero".format(largest_backward, largest_forward)))

    expected_sign = 1 if args.first == "forward" else -1
    if first_sign == expected_sign:
        results.append(("PASS", "pitch sign: first tilt ({}) read {}, as it should".format(
            args.first, "positive" if first_sign > 0 else "negative")))
    else:
        results.append(("FAIL", "pitch sign: first tilt ({}) read {}: flip BRI_PITCH_FWD_SIGN and "
                                "BRI_PITCH_GYRO_SIGN in BRI_Config.h".format(
                                    args.first, "positive" if first_sign > 0 else "negative")))

    # Do gyro and accelerometer agree?  Compare the gyro rate with the slope
    # of the accelerometer pitch (how fast that angle is changing).
    rates, slopes = [], []
    for index in range(SLOPE_HALF_WINDOW, len(tilt) - SLOPE_HALF_WINDOW):
        before, after = tilt[index - SLOPE_HALF_WINDOW], tilt[index + SLOPE_HALF_WINDOW]
        dt = (after["t_ms"] - before["t_ms"]) / 1000.0
        if dt <= 0:
            continue
        slopes.append((after["pitch_acc_deg"] - before["pitch_acc_deg"]) / dt)
        rates.append(tilt[index]["rate_dps"] - bias)
    agreement = correlation(rates, slopes)
    if agreement >= CORRELATION_MIN:
        results.append(("PASS", "gyro and accelerometer agree (correlation {:+.2f})".format(agreement)))
    elif agreement <= -CORRELATION_MIN:
        results.append(("FAIL", "gyro runs OPPOSITE to the accelerometer (correlation {:+.2f}): "
                                "flip BRI_PITCH_GYRO_SIGN".format(agreement)))
    else:
        results.append(("FAIL", "gyro does not follow the tilt (correlation {:+.2f}): "
                                "BRI_PITCH_GYRO_AXIS is probably the wrong axis".format(agreement)))

    # How well the blended estimate tracks, and how far the gyro-only one drifted.
    filter_error = [row["pitch_cf_deg"] - row["pitch_acc_deg"] for row in tilt]
    drift = tilt[-1]["pitch_gyro_deg"] - tilt[-1]["pitch_acc_deg"]
    results.append(("INFO", "filter minus accelerometer: mean {:+.2f} deg, spread {:.2f} deg; "
                            "gyro-only drift at the end {:+.2f} deg".format(
                                bri.mean(filter_error), bri.stdev(filter_error), drift)))

    errors = max(capture.column("i2c_err"))
    results.append(("PASS" if errors == 0 else "FAIL", "{:.0f} I2C errors".format(errors)))
    return results


if __name__ == "__main__":
    sys.exit(bri.run_test(
        TEST_ID, TITLE, EXPECTED_HEADER, evaluate, start_commands,
        default_seconds=20.0,
        operator_notes="Hold upright and STILL for 2 s, then tilt FORWARD, upright, BACKWARD, slowly.",
        plot_columns=("pitch_acc_deg", "pitch_gyro_deg", "pitch_cf_deg", "rate_dps"),
        add_arguments=add_arguments))
