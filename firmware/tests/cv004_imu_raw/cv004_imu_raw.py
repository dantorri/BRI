#!/usr/bin/env python3
"""
CV-004  Raw IMU data - PC logger for cv004_imu_raw.ino

    python cv004_imu_raw.py                      run on hardware (10 s, IMU held still)
    python cv004_imu_raw.py --plot               also save a PNG of the six channels
    python cv004_imu_raw.py --replay some_capture.txt

Hold the IMU still for the whole run.  Repeat in other orientations to learn
which axis points where: the axis that reads about +1 g points up.

Pass rules:
    length of the acceleration vector 0.90..1.10 g
    each gyro axis within +/-8 deg/s of zero on average, spread under 2 deg/s
    no I2C errors
"""

import pathlib
import sys

# Make "import bri_logger" work no matter which folder this script is run
# from: the shared logger lives in firmware/tools, two folders up from here.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
import bri_logger as bri  # noqa: E402  (import after the path tweak, on purpose)

import math  # noqa: E402

TEST_ID = "CV-004"
TITLE = "Raw IMU data"
EXPECTED_HEADER = "t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,amag_g,temp_C,i2c_err"

GRAVITY_MIN_G, GRAVITY_MAX_G = 0.90, 1.10
GYRO_OFFSET_MAX_DPS = 8.0
GYRO_SPREAD_MAX_DPS = 2.0


def start_commands(args):
    """Commands sent to the Teensy to start the run."""
    return ["t{:g}".format(args.seconds), "g"]


def evaluate(capture, args):
    """Judge the recorded data.  Returns a list of (verdict, text)."""
    results = []
    if not capture.rows:
        return [("FAIL", "no IMU samples recorded (IMU not found? run CV-003)")]

    # Gravity: recompute the vector length from the three axes rather than
    # trusting the firmware's amag_g column, as an independent check.
    lengths = [math.sqrt(row["ax_g"] ** 2 + row["ay_g"] ** 2 + row["az_g"] ** 2) for row in capture.rows]
    gravity = bri.mean(lengths)
    results.append(("PASS" if bri.within(gravity, GRAVITY_MIN_G, GRAVITY_MAX_G) else "FAIL",
                    "gravity magnitude {:.3f} g (limits {:.2f}..{:.2f})".format(gravity, GRAVITY_MIN_G, GRAVITY_MAX_G)))

    gyro_ok = True
    parts = []
    for name in ("gx_dps", "gy_dps", "gz_dps"):
        values = capture.column(name)
        offset, spread = bri.mean(values), bri.stdev(values)
        if abs(offset) > GYRO_OFFSET_MAX_DPS or spread > GYRO_SPREAD_MAX_DPS:
            gyro_ok = False
        parts.append("{} {:+.2f} (spread {:.2f})".format(name[:2], offset, spread))
    results.append(("PASS" if gyro_ok else "FAIL",
                    "gyro at rest, deg/s: " + ", ".join(parts) + "  (limits +/-{:.0f}, spread {:.0f})".format(
                        GYRO_OFFSET_MAX_DPS, GYRO_SPREAD_MAX_DPS)))

    # Which way is up?
    axis_means = {name: bri.mean(capture.column(name)) for name in ("ax_g", "ay_g", "az_g")}
    up_axis = max(axis_means, key=lambda name: abs(axis_means[name]))
    results.append(("INFO", "gravity is mostly along {}{} ({:+.3f} g): that axis points {}".format(
        "+" if axis_means[up_axis] > 0 else "-", up_axis[1], axis_means[up_axis],
        "up" if axis_means[up_axis] > 0 else "down")))

    errors = max(capture.column("i2c_err"))
    results.append(("PASS" if errors == 0 else "FAIL", "{:.0f} I2C errors".format(errors)))
    results.append(("INFO", "IMU temperature {:.1f} C".format(bri.mean(capture.column("temp_C")))))
    return results


if __name__ == "__main__":
    sys.exit(bri.run_test(
        TEST_ID, TITLE, EXPECTED_HEADER, evaluate, start_commands,
        default_seconds=10.0,
        operator_notes="Hold the IMU perfectly still for the whole run.",
        plot_columns=("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps")))
