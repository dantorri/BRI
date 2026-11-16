#!/usr/bin/env python3
"""
CV-003  I2C bus scan - PC logger for cv003_i2c_scan.ino

    python cv003_i2c_scan.py                     run on hardware (5 s = 20 scans)
    python cv003_i2c_scan.py --seconds 30        120 scans, to hunt an intermittent contact
    python cv003_i2c_scan.py --replay some_capture.txt

Pass rules:
    the IMU answers at 0x68 or 0x69 in EVERY scan
    zero bus errors
    WHO_AM_I is one of 0x71 (MPU-9250), 0x73 (MPU-9255), 0x70 (MPU-6500), 0x68 (MPU-6050)
"""

import pathlib
import sys

# Make "import bri_logger" work no matter which folder this script is run
# from: the shared logger lives in firmware/tools, two folders up from here.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
import bri_logger as bri  # noqa: E402  (import after the path tweak, on purpose)

TEST_ID = "CV-003"
TITLE = "I2C bus scan"
EXPECTED_HEADER = "t_ms,scan_n,found_count,imu_addr,who_am_i,bus_errors,imu_found"

# WHO_AM_I value -> chip name.
KNOWN_CHIPS = {0x71: "MPU-9250", 0x73: "MPU-9255", 0x70: "MPU-6500", 0x68: "MPU-6050"}


def start_commands(args):
    """Commands sent to the Teensy to start the run."""
    return ["t{:g}".format(args.seconds), "g"]


def evaluate(capture, args):
    """Judge the recorded data.  Returns a list of (verdict, text)."""
    results = []
    found = capture.column("imu_found")
    if not found:
        return [("FAIL", "no scans recorded")]
    hits = sum(1 for value in found if value == 1)
    results.append(("PASS" if hits == len(found) else "FAIL",
                    "IMU answered in {} of {} scans".format(hits, len(found))))

    errors = max(capture.column("bus_errors"))
    results.append(("PASS" if errors == 0 else "FAIL", "{:.0f} bus errors".format(errors)))

    # Decode the identity from the last scan that found the IMU.
    address, who = 0, 0
    for row in capture.rows:
        if row["imu_found"] == 1:
            address, who = int(row["imu_addr"]), int(row["who_am_i"])
    if address == 0:
        results.append(("FAIL", "nothing at 0x68 or 0x69: check 3.3 V, GND, SDA = pin 18, SCL = pin 19, pull-ups"))
    elif who in KNOWN_CHIPS:
        results.append(("PASS", "address 0x{:02X}, WHO_AM_I 0x{:02X} = {}".format(address, who, KNOWN_CHIPS[who])))
    else:
        results.append(("FAIL", "address 0x{:02X} answers but WHO_AM_I 0x{:02X} is not a known MPU chip".format(address, who)))

    others = max(capture.column("found_count")) - 1
    if others > 0:
        results.append(("INFO", "{:.0f} other device(s) on the bus (see the '# scan' line in the raw log)".format(others)))
    return results


if __name__ == "__main__":
    sys.exit(bri.run_test(
        TEST_ID, TITLE, EXPECTED_HEADER, evaluate, start_commands,
        default_seconds=5.0,
        plot_columns=("found_count", "imu_found", "bus_errors")))
