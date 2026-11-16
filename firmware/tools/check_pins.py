#!/usr/bin/env python3
"""
check_pins.py - keep BRI_Pins.h and the pin-map CSV in agreement.

WHY THIS EXISTS
    The pin map CSV (libraries/BRI/extras/pin_map.csv) is the single source of
    truth for which Teensy pin carries which signal.  The firmware cannot read
    a CSV, so the same information also has to live in a C++ header
    (libraries/BRI/src/BRI_Pins.h).  Two copies of the same facts drift apart
    unless something checks them.  This script is that check.

WHAT IT DOES
    python tools/check_pins.py            compare header against CSV (exit 0 = agree)
    python tools/check_pins.py --write    regenerate the header from the CSV

    The check is done two independent ways so one bug cannot hide another:
      1. the header text is regenerated from the CSV and compared byte for byte;
      2. the header is parsed with a regular expression and every
         "PIN_<net> = <number>" is compared with the CSV row for that net.

Only the Python standard library is used.
"""
import argparse
import csv
import pathlib
import re
import sys

# Folder layout: this file is firmware/tools/check_pins.py
FIRMWARE_DIR = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_CSV = FIRMWARE_DIR / "libraries" / "BRI" / "extras" / "pin_map.csv"
DEFAULT_HEADER = FIRMWARE_DIR / "libraries" / "BRI" / "src" / "BRI_Pins.h"

# The four motor channels, in index order.  Index 0 = LW ... index 3 = RH.
# The per-channel lookup tables in the header are built from these suffixes.
CHANNEL_SUFFIXES = ["LW", "RW", "LH", "RH"]

# For each lookup table: (table name, net-name pattern, what it is for).
CHANNEL_TABLES = [
    ("BRI_PIN_PWM",   "PWM_{}",   "PWM output to the G2 driver (speed)"),
    ("BRI_PIN_DIR",   "DIR_{}",   "DIR output to the G2 driver (direction)"),
    ("BRI_PIN_CS",    "CS_{}",    "analog current-sense input from the G2 driver"),
    ("BRI_PIN_FLT",   "FLT_{}",   "fault input from the G2 driver (low = fault)"),
    ("BRI_PIN_ENC_A", "ENC_{}_A", "encoder channel A (after the level shifter)"),
    ("BRI_PIN_ENC_B", "ENC_{}_B", "encoder channel B (after the level shifter)"),
]


def net_to_constant(net):
    """Turn a CSV net name into the C++ constant name.

    "ENC_LW_A"            -> "PIN_ENC_LW_A"
    "SPI_CS (reserved)"   -> "PIN_SPI_CS"   (the "(reserved)" note is dropped
                                             because it is not a legal C++ name)
    """
    clean = net.replace("(reserved)", "").strip()
    return "PIN_" + clean


def load_csv(csv_path):
    """Read the pin map and return a list of row dictionaries, in file order."""
    with open(csv_path, newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        row["pin"] = int(row["teensy_pin"])
        row["const"] = net_to_constant(row["net"])
        row["reserved"] = "(reserved)" in row["net"] or row["phase"].strip() == "reserved"
    return rows


def generate_header(rows):
    """Build the full text of BRI_Pins.h from the CSV rows."""
    by_const = {row["const"]: row for row in rows}
    name_width = max(len(row["const"]) for row in rows)
    out = []
    out.append("// BRI_Pins.h - Teensy 4.1 pin numbers for the BRI balancing robot.")
    out.append("//")
    out.append("// GENERATED FILE.  Do not edit by hand.")
    out.append("// Source of truth: libraries/BRI/extras/pin_map.csv")
    out.append("// Regenerate with:  python tools/check_pins.py --write")
    out.append("// Check with:       python tools/check_pins.py")
    out.append("//")
    out.append("// Why constants instead of bare numbers: a sketch that says")
    out.append("// digitalWrite(PIN_MOTOR_EN, HIGH) can be read and searched; a sketch that")
    out.append("// says digitalWrite(33, HIGH) cannot.  Every name below is 'PIN_' followed by")
    out.append("// the 'net' column of the CSV, so the wiring table and the code use one name.")
    out.append("#ifndef BRI_PINS_H")
    out.append("#define BRI_PINS_H")
    out.append("")
    out.append("#include <stdint.h>")
    out.append("")
    out.append("// ---- One constant per CSV row (CSV order) -------------------------------")
    out.append("// Comment format: direction | phase | what it connects to | conditioning")
    for row in rows:
        tag = "RESERVED, keep free | " if row["reserved"] else ""
        comment = "{}{} | phase {} | {} | {}".format(
            tag, row["direction"], row["phase"], row["connects_to"], row["conditioning"])
        out.append("const uint8_t {} = {:>2};  // {}".format(
            row["const"].ljust(name_width), row["pin"], comment))
    out.append("")
    out.append("// ---- Motor channel numbering ---------------------------------------------")
    out.append("// The library addresses a motor by a channel index so that one test sketch")
    out.append("// can exercise any of the four motors.  Phase 1 hardware has only LW and RW")
    out.append("// wired; LH and RH (the hips) are phase 2.")
    out.append("const uint8_t BRI_NUM_CHANNELS = {};".format(len(CHANNEL_SUFFIXES)))
    for index, suffix in enumerate(CHANNEL_SUFFIXES):
        out.append("const uint8_t BRI_CH_{} = {};".format(suffix, index))
    out.append("")
    out.append("// ---- Per-channel lookup tables (index = channel) -------------------------")
    out.append("// These let the library write BRI_PIN_PWM[channel] instead of a four-way")
    out.append("// if/else for every pin.")
    for table, pattern, purpose in CHANNEL_TABLES:
        names = []
        for suffix in CHANNEL_SUFFIXES:
            const = "PIN_" + pattern.format(suffix)
            if const not in by_const:
                raise SystemExit("pin map CSV has no net for " + const)
            names.append(const)
        out.append("// {}".format(purpose))
        out.append("const uint8_t {}[{}] = {{ {} }};".format(
            table, len(CHANNEL_SUFFIXES), ", ".join(names)))
    out.append("")
    out.append("#endif  // BRI_PINS_H")
    out.append("")
    return "\n".join(out)


def parse_header(text):
    """Pull every 'const uint8_t PIN_X = n;' out of the header text.

    Returns a dictionary {constant name: pin number}.
    """
    found = {}
    for match in re.finditer(r"const\s+uint8_t\s+(PIN_\w+)\s*=\s*(\d+)\s*;", text):
        found[match.group(1)] = int(match.group(2))
    return found


def parse_tables(text):
    """Pull the per-channel tables out of the header text.

    Returns {table name: [constant names in channel order]}.
    """
    tables = {}
    for match in re.finditer(r"const\s+uint8_t\s+(BRI_PIN_\w+)\[\d+\]\s*=\s*\{([^}]*)\}", text):
        names = [item.strip() for item in match.group(2).split(",") if item.strip()]
        tables[match.group(1)] = names
    return tables


def compare(rows, header_text):
    """Return a list of human-readable problems (empty list = header agrees)."""
    problems = []
    header_pins = parse_header(header_text)
    csv_pins = {row["const"]: row["pin"] for row in rows}

    # Every CSV net must be in the header with the same pin number.
    for const, pin in csv_pins.items():
        if const not in header_pins:
            problems.append("missing in header: {} (CSV pin {})".format(const, pin))
        elif header_pins[const] != pin:
            problems.append("pin differs: {} header={} csv={}".format(const, header_pins[const], pin))
    # The header must not invent pins the CSV does not have.
    for const in header_pins:
        if const not in csv_pins:
            problems.append("extra in header (not in CSV): {}".format(const))
    # A Teensy pin can only carry one net.
    seen = {}
    for row in rows:
        if row["pin"] in seen:
            problems.append("CSV uses pin {} twice: {} and {}".format(row["pin"], seen[row["pin"]], row["const"]))
        seen[row["pin"]] = row["const"]
    # Channel tables must list the right net for each channel index.
    tables = parse_tables(header_text)
    for table, pattern, _purpose in CHANNEL_TABLES:
        expected = ["PIN_" + pattern.format(suffix) for suffix in CHANNEL_SUFFIXES]
        if tables.get(table) != expected:
            problems.append("table {} is {} but should be {}".format(table, tables.get(table), expected))
    # Byte-for-byte comparison with a fresh generation (catches anything else).
    if generate_header(rows) != header_text:
        problems.append("header text is not identical to a fresh generation from the CSV")
    return problems


def main(argv=None):
    """Command-line entry: check the header against the CSV, or rewrite it with --write.  Returns 0 if they agree."""
    parser = argparse.ArgumentParser(description="Check or regenerate BRI_Pins.h from the pin map CSV.")
    parser.add_argument("--csv", default=str(DEFAULT_CSV), help="pin map CSV (default: the copy inside the library)")
    parser.add_argument("--header", default=str(DEFAULT_HEADER), help="header to check or write")
    parser.add_argument("--write", action="store_true", help="regenerate the header instead of checking it")
    args = parser.parse_args(argv)

    rows = load_csv(args.csv)
    if args.write:
        pathlib.Path(args.header).write_text(generate_header(rows), encoding="utf-8", newline="\n")
        print("wrote {} ({} pins)".format(args.header, len(rows)))
        return 0

    header_text = pathlib.Path(args.header).read_text(encoding="utf-8")
    problems = compare(rows, header_text)
    print("pin check: csv={} header={}".format(args.csv, args.header))
    print("pin check: {} CSV rows, {} header constants".format(len(rows), len(parse_header(header_text))))
    for problem in problems:
        print("PIN MISMATCH: " + problem)
    if problems:
        print("PINCHECK,FAIL,{} problem(s)".format(len(problems)))
        return 1
    print("PINCHECK,PASS,header matches CSV")
    return 0


if __name__ == "__main__":
    sys.exit(main())
