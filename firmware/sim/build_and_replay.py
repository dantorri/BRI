#!/usr/bin/env python3
"""
build_and_replay.py - check the whole firmware kit on a PC, without a Teensy.

WHAT IT DOES, IN ORDER
    1. PIN CHECK      runs tools/check_pins.py: BRI_Pins.h must agree with the pin map CSV.
    2. HEADER CHECK   the CSV header string in each sketch must equal the one its
                      Python logger expects.
    3. STUB BUILDS    compiles every sketch for the PC with "zig c++", against the
                      stand-in Arduino headers in sim/stubs, once per encoder backend
                      (PJRC Encoder, and QuadEncoder via -DBRI_USE_HW_QUADENC).
    4. SIMULATED RUNS runs each program with a scripted "operator" and the simple
                      hardware model in sim_bri_hw.cpp; the output must contain a
                      header, data rows and RESULT lines, and the motor outputs must
                      obey the safety rules.
    5. REPLAYS        feeds each simulated capture to the matching logger with
                      --replay; the logger must accept the header and every row and
                      its evaluator must run.
    6. SAFETY RUNS    extra simulated runs that pull the USB cable, flip the kill
                      switch, raise a driver fault, force an over-current, and so on,
                      and check that the drivers end up asleep for the right reason.
                      One of them uses sim/selftest/lib_probe, a simulation-only sketch
                      that deliberately asks the library for too much duty.
    7. FAULT FINDING  simulated runs in which the hardware is wrong on purpose (IMU
                      missing, encoder reversed, motor stalled, 4.9 V on a Teensy pin,
                      regulator sagging, ...).  The firmware and the logger must both
                      report the fault: a test that cannot fail proves nothing.
    8. LIVE PATH      runs a logger through its real serial-capture code against a
                      fake serial port (a local TCP socket that plays back a simulated
                      capture), and checks that a motor logger sends nothing when the
                      operator does not type GO.

WHAT IT DOES NOT DO
    It does not compile for the Teensy and it does not touch hardware.  The stub
    headers only imitate the Teensy core.  A pass here means "the C++ is valid,
    the protocol is consistent and the logic behaves against a simple model" -
    nothing more.

USAGE
    python sim/build_and_replay.py
    python sim/build_and_replay.py --zig C:\path\to\zig.exe --log firmware_verification.txt
    python sim/build_and_replay.py --pin-csv path\to\authoritative_pin_map.csv

Exit code 0 only if every step passed.
"""
import argparse
import concurrent.futures
import datetime
import json
import os
import pathlib
import re
import shutil
import socket
import subprocess
import sys
import threading
import time

SIM_DIR = pathlib.Path(__file__).resolve().parent
FIRMWARE_DIR = SIM_DIR.parent
BUILD_DIR = SIM_DIR / "build"
TESTS_DIR = FIRMWARE_DIR / "tests"
LIBRARY_SRC = FIRMWARE_DIR / "libraries" / "BRI" / "src"
EXE_SUFFIX = ".exe" if os.name == "nt" else ""

# The duty cap in BRI_Config.h, plus a hair for rounding to PWM steps.
DUTY_CAP_LIMIT = 0.35 + 0.001

# Encoder backends: name -> extra compiler flags.
BACKENDS = {
    "pjrc": [],
    "hwquad": ["-DBRI_USE_HW_QUADENC"],
}

# One nominal scenario per test: (script lines, simulated milliseconds).
# Script lines are "<time_ms> <text>"; text starting with '@' is an instruction
# to the hardware model, anything else is typed into the firmware.
SCENARIOS = {
    "cv001_power_usb": (["0 t3", "10 g", "800 r3.29", "900 b0.02"], 5000),
    "cv002_5v_regulator": (["0 t4", "10 g", "500 i0.0", "600 v5.03", "1500 i2.0", "1600 v4.98",
                            "2500 i5.0", "2600 v4.93"], 6000),
    "cv003_i2c_scan": (["0 t3", "10 g"], 5000),
    "cv004_imu_raw": (["0 t3", "10 g"], 5000),
    "cv005_imu_pitch": (["0 @tilt 25 4000 3000", "0 t10", "10 g"], 12000),
    "cv006_driver_logic": (["0 @disconnect 0 1", "0 c0", "2 m0", "5 t1", "10 g"], 9000),
    "cv007_motor_noload": (["0 c0", "10 g"], 17000),
    "cv008_encoder_waveform": (["0 c1", "5 t3", "10 g", "2000 v3.28"], 6000),
    "cv009_encoder_cal": (["0 c0", "10 h", "500 b", "600 @turn 0 1.0 1500", "2500 e", "3000 b",
                           "3100 @turn 0 -1.0 1500", "5000 e", "5500 m2", "5600 g", "17000 o2.0",
                           "18000 s"], 19000),
}

# Safety scenarios: name, sketch, script, simulated ms, expected "!STOP" reason
# (None = no stop expected), and limits on what the simulated outputs did.
#   max_duty       largest PWM duty allowed at any time
#   drive_by_ms    the drivers must be asleep and PWM zero by this time
#   drive_after_ms ...but must still have been active at this time
#   expect         exact values required in the "SIM," summary line
#   min_duty       the duty must have reached at least this (proves the limit was really tested)
#   firmware_pass  the firmware's own final RESULT line must say PASS
#   selftest       use the simulation-only sketch sim/selftest/<name> instead of a test sketch
SAFETY_SCENARIOS = [
    dict(name="no_go_no_motion", sketch="cv007_motor_noload", script=["0 c0", "0 d0.3"], ms=3000,
         stop=None, max_duty=0.0, expect={"ever_en_wheels": 0, "ever_en_hips": 0},
         why="without a g command the drivers must stay asleep and PWM at zero"),
    dict(name="usb_lost_stops_motor", sketch="cv007_motor_noload", script=["0 c0", "10 g", "2000 @usb 0"],
         ms=6000, stop="usb_lost", drive_after_ms=1990, drive_by_ms=2020,
         why="losing the USB serial connection must brake and sleep the drivers"),
    dict(name="kill_switch_stops_motor", sketch="cv007_motor_noload", script=["0 c0", "10 g", "2000 @kill 1"],
         ms=6000, stop="kill_switch", drive_after_ms=1990, drive_by_ms=2020,
         why="MOTOR_EN_SENSE low while commanded high must be reported and the outputs dropped"),
    dict(name="kill_switch_blocks_go", sketch="cv007_motor_noload", script=["0 @kill 1", "0 c0", "10 g"],
         ms=3000, stop="kill_switch", max_duty=0.0,
         why="with the kill switch in STOP a go command must be refused before any PWM"),
    dict(name="driver_fault_stops_motor", sketch="cv007_motor_noload", script=["0 c0", "10 g", "2000 @fault 0 1"],
         ms=6000, stop="driver_fault", drive_after_ms=1990, drive_by_ms=2020,
         why="FLT low must stop the run"),
    dict(name="over_current_trips", sketch="cv007_motor_noload", script=["0 c0", "10 g", "4000 @extra 0 4.0"],
         ms=8000, stop="over_current", drive_after_ms=4190, drive_by_ms=4230,
         why="3.0 A or more for 200 ms must trip (4.2 A injected at t = 4000 ms)"),
    dict(name="operator_stop", sketch="cv007_motor_noload", script=["0 c0", "10 g", "2000 s"],
         ms=6000, stop="operator", drive_after_ms=1990, drive_by_ms=2010,
         why="the s command must stop at once"),
    dict(name="timeout_stops_motor", sketch="cv009_encoder_cal",
         script=["0 @stuck 0 1", "0 c0", "10 h", "100 m5", "200 g"], ms=58000,
         stop="timeout", drive_after_ms=55000, drive_by_ms=55300,
         why="a move that never reaches its target (shaft locked) must end at the 55 s time limit"),
    dict(name="sketch_clamps_duty_request", sketch="cv008_encoder_waveform", script=["0 c0", "0 d0.9", "5 t2", "10 g"],
         ms=5000, stop="complete", max_duty=DUTY_CAP_LIMIT,
         why="the operator typing d0.9 must be limited to 35 % (this exercises the SKETCH's clamp)"),
    dict(name="library_duty_cap", selftest="lib_probe", script=["10 g"], ms=3000,
         stop="complete", max_duty=DUTY_CAP_LIMIT, min_duty=0.34, firmware_pass=True,
         expect={"ever_en_wheels": 1, "ever_en_hips": 0},
         why="a sketch that asks the LIBRARY for 90 % (before go, after go, and behind the safety "
             "layer's back) must get 0 before go and at most 35 % after"),
    dict(name="hip_channel_uses_hip_enable", sketch="cv007_motor_noload", script=["0 c2", "10 g"],
         ms=17000, stop="complete", expect={"ever_en_wheels": 0, "ever_en_hips": 1},
         why="a hip channel must wake HIP_EN only, never the wheel drivers"),
    dict(name="wheel_channel_leaves_hips_asleep", sketch="cv007_motor_noload", script=["0 c1", "10 g"],
         ms=17000, stop="complete", expect={"ever_en_wheels": 1, "ever_en_hips": 0},
         why="a wheel channel must never wake the hip drivers"),
]


# Fault-finding scenarios: the simulated hardware is WRONG (or just different)
# on purpose, to show that a test does not simply always pass.
#   fw           verdict the firmware's final RESULT line must carry
#   text         text that must appear somewhere in the firmware output
#   logger_exit  exit code the logger must give on replay (0 = PASS/INFO, 1 = FAIL)
FAULT_SCENARIOS = [
    dict(name="imu_at_0x69_mpu9250_found", sketch="cv003_i2c_scan", script=["0 @imu 69 71", "0 t2", "10 g"], ms=4000,
         fw="PASS", text="imu_addr=0x69 who_am_i=0x71 chip=MPU-9250", logger_exit=0,
         why="the other legal address and another chip id must be recognised"),
    dict(name="imu_missing_fails_scan", sketch="cv003_i2c_scan", script=["0 @imu 10 00", "0 t2", "10 g"], ms=4000,
         fw="FAIL", text="no device answered at 0x68 or 0x69", logger_exit=1,
         why="no IMU on the bus must fail CV-003"),
    dict(name="unknown_chip_fails_scan", sketch="cv003_i2c_scan", script=["0 @imu 68 12", "0 t2", "10 g"], ms=4000,
         fw="FAIL", text="chip=unknown", logger_exit=1,
         why="a device at 0x68 that is not an MPU must fail CV-003"),
    dict(name="imu_missing_fails_raw_test", sketch="cv004_imu_raw", script=["0 @imu 10 00", "0 t2", "10 g"], ms=4000,
         fw="FAIL", text="imu_not_found", logger_exit=1,
         why="CV-004 with no IMU must fail cleanly (header, no rows, RESULT lines)"),
    dict(name="tilted_backward_first_fails_sign", sketch="cv005_imu_pitch",
         script=["0 @tilt -25 4000 3000", "0 t10", "10 g"], ms=12000,
         fw="INFO", text="first_tilt=negative", logger_exit=1,
         why="a first tilt that reads negative must be flagged (the logger assumes forward was first)"),
    dict(name="regulator_sag_fails", sketch="cv002_5v_regulator",
         script=["0 t3", "10 g", "500 i0.0", "600 v5.03", "1500 i5.0", "1600 v4.50"], ms=5000,
         fw="FAIL", text="out_of_band=1", logger_exit=1,
         why="an entered 4.50 V must fail the 4.8-5.2 V band"),
    dict(name="restrained_motor_declared", sketch="cv006_driver_logic",
         script=["0 @stuck 0 1", "0 c0", "2 m1", "5 t1", "10 g"], ms=9000,
         fw="PASS", text="cs_with_motor_connected", logger_exit=0,
         why="with a motor declared connected the CS reading must be reported as current, not as the zero offset"),
    dict(name="reversed_encoder_fails_direction", sketch="cv007_motor_noload",
         script=["0 @encflip 0 1", "0 c0", "10 g"], ms=17000,
         fw="FAIL", text="direction: encoder disagrees with the command", logger_exit=1,
         why="an encoder that counts backwards on positive duty must fail CV-007"),
    dict(name="stalled_motor_fails_noload_current", sketch="cv007_motor_noload",
         script=["0 @stuck 0 1", "0 c0", "10 g"], ms=17000,
         fw="FAIL", text="i_fwd_A=1.", logger_exit=1,
         why="a locked shaft draws well above the no-load band (and shows no encoder motion)"),
    dict(name="reversed_encoder_fails_waveform_test", sketch="cv008_encoder_waveform",
         script=["0 @encflip 1 1", "0 c1", "5 t3", "10 g", "2000 v3.28"], ms=6000,
         fw="FAIL", text="direction_consistent", logger_exit=1,
         why="count direction opposite to the duty must fail CV-008"),
    dict(name="five_volt_on_teensy_pin_fails", sketch="cv008_encoder_waveform",
         script=["0 c1", "5 t3", "10 g", "2000 v4.9"], ms=6000,
         fw="FAIL", text="scope_vhigh_V=4.90", logger_exit=1,
         why="a scope reading of 4.9 V at a Teensy pin must fail (the Teensy is not 5 V tolerant)"),
    dict(name="short_revolution_fails_calibration", sketch="cv009_encoder_cal",
         script=["0 c0", "10 h", "500 b", "600 @turn 0 0.95 1500", "2500 e", "3000 s"], ms=4000,
         fw="FAIL", text="outside_tolerance=1", logger_exit=1,
         why="7980 counts for 'one turn' is 5 % short and must fail the 1 % tolerance"),
]


class Log:
    """Collects the verification log and echoes it to the screen."""

    def __init__(self):
        """Start with an empty list of log lines."""
        self.lines = []

    def write(self, text=""):
        """Add text to the log and show it on screen, so the saved file and the screen always match."""
        for line in str(text).splitlines() or [""]:
            self.lines.append(line)
            print(line)

    def section(self, title):
        """Write a ruled heading that separates the numbered steps in the log."""
        self.write("")
        self.write("=" * 78)
        self.write(title)
        self.write("=" * 78)


def find_zig(explicit):
    """Locate a zig executable.  Order: --zig, the ZIG environment variable,
    the PATH, then the usual place inside a conda environment."""
    candidates = []
    if explicit:
        candidates.append(explicit)
    if os.environ.get("ZIG"):
        candidates.append(os.environ["ZIG"])
    for name in ("zig", "x86_64-w64-mingw32-zig"):
        found = shutil.which(name)
        if found:
            candidates.append(found)
    for prefix in (os.environ.get("CONDA_PREFIX"), sys.base_prefix, sys.prefix):
        if prefix:
            candidates.append(str(pathlib.Path(prefix) / "Library" / "bin" / "x86_64-w64-mingw32-zig.exe"))
            candidates.append(str(pathlib.Path(prefix) / "bin" / "zig"))
    for candidate in candidates:
        if candidate and pathlib.Path(candidate).is_file():
            return str(candidate)
    return None


def tool_environment(zig_path):
    """Environment variables for running zig.

    A zig installed by conda expects CONDA_PREFIX / PREFIX to point at its
    environment.  If zig lives in <env>/Library/bin, fill those in.
    """
    env = dict(os.environ)
    zig = pathlib.Path(zig_path)
    if zig.parent.name.lower() == "bin" and zig.parent.parent.name.lower() == "library":
        root = str(zig.parent.parent.parent)
        env.setdefault("CONDA_PREFIX", root)
        env.setdefault("PREFIX", root)
    return env


def run(command, env=None, timeout=600):
    """Run a program; return (exit code, stdout text, stderr text)."""
    done = subprocess.run(command, capture_output=True, env=env, timeout=timeout)
    return done.returncode, done.stdout.decode("utf-8", "replace"), done.stderr.decode("utf-8", "replace")


def sketch_dirs():
    """Every test folder that holds a sketch of the same name."""
    return sorted(folder for folder in TESTS_DIR.iterdir()
                  if folder.is_dir() and (folder / (folder.name + ".ino")).is_file())


def build_sketch(zig, env, folder, backend):
    """Compile one sketch for the PC.  Returns (ok, executable path, compiler output)."""
    name = folder.name
    ino = folder / (name + ".ino")
    # The Arduino IDE turns an .ino into C++ by putting "#include <Arduino.h>"
    # in front of it (it also adds function prototypes; the sketches here
    # define every function before using it, so that step is not needed).
    wrapper = BUILD_DIR / "{}_{}.cpp".format(name, backend)
    wrapper.write_text('#include <Arduino.h>\n#line 1 "{}"\n{}'.format(ino.as_posix(), ino.read_text(encoding="utf-8")),
                       encoding="utf-8")
    executable = BUILD_DIR / "{}_{}{}".format(name, backend, EXE_SUFFIX)
    sources = [str(wrapper)] + [str(path) for path in sorted(LIBRARY_SRC.glob("*.cpp"))]
    sources += [str(SIM_DIR / "sim_core.cpp"), str(SIM_DIR / "sim_bri_hw.cpp")]
    command = [zig, "c++", "-std=gnu++17", "-O1", "-Wall", "-Wextra",
               "-I", str(SIM_DIR / "stubs"), "-I", str(LIBRARY_SRC), "-I", str(SIM_DIR)]
    command += BACKENDS[backend] + sources + ["-o", str(executable)]
    code, out, err = run(command, env=env)
    return code == 0 and executable.is_file(), executable, (out + err).strip()


def simulate(executable, script_lines, milliseconds, tag):
    """Run one simulated scenario.  Returns (exit code, stdout, stderr, capture path)."""
    script_path = BUILD_DIR / (tag + ".script")
    script_path.write_text("\n".join(script_lines) + "\n", encoding="utf-8")
    code, out, err = run([str(executable), "--ms", str(milliseconds), "--script", str(script_path)], timeout=300)
    capture_path = BUILD_DIR / (tag + ".txt")
    capture_path.write_text(out.replace("\r\n", "\n"), encoding="utf-8")
    return code, out, err, capture_path


def parse_sim_summary(stderr_text):
    """Turn the 'SIM,key=value,...' line the simulator prints into a dictionary."""
    summary = {}
    for line in stderr_text.splitlines():
        if line.startswith("SIM,"):
            for item in line.split(",")[1:]:
                key, _, value = item.partition("=")
                try:
                    summary[key] = float(value)
                except ValueError:
                    summary[key] = value
    return summary


def outputs_safe(summary):
    """Rules every simulated run must obey.  Returns a list of violations."""
    problems = []
    if not summary:
        return ["simulator printed no summary line"]
    if summary.get("drive_before_go") != 0:
        problems.append("a driver was woken or PWM applied before any g command")
    if summary.get("final_en_wheels") != 0 or summary.get("final_en_hips") != 0:
        problems.append("a driver enable net was still high at the end")
    if summary.get("final_pwm_active") != 0:
        problems.append("PWM was still non-zero at the end")
    if summary.get("max_abs_duty", 9) > DUTY_CAP_LIMIT:
        problems.append("duty {:.3f} exceeded the cap".format(summary.get("max_abs_duty", 9)))
    return problems


def capture_shape(text):
    """Count the line kinds in a capture: (header fields or 0, data rows, RESULT lines, rows of wrong length)."""
    header_fields, rows, results, wrong = 0, 0, 0, 0
    for line in text.replace("\r", "").split("\n"):
        if line.startswith("t_ms,"):
            header_fields = len(line.split(","))
        elif line.startswith("RESULT,"):
            results += 1
        elif line[:1].isdigit():
            rows += 1
            if header_fields and len(line.split(",")) != header_fields:
                wrong += 1
    return header_fields, rows, results, wrong


class FakeTeensy:
    """A stand-in for the Teensy's serial port, on a local TCP socket.

    Why: the loggers' live-capture code (open port, send commands, read lines,
    stop after the RESULT lines) cannot be tested through --replay.  pyserial
    can open "socket://host:port" like a serial port, so this little server
    lets that code run for real: it waits for a line starting with 'g' and
    then sends a previously simulated capture, as the firmware would.
    """

    def __init__(self, capture_text):
        """Open a listening socket on a free local port and start the thread that plays the Teensy's part."""
        self.capture_text = capture_text
        self.commands = []          # every command line received, in order
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(("127.0.0.1", 0))   # port 0 = let the system pick a free port
        self.server.listen(1)
        self.server.settimeout(20)
        self.port = self.server.getsockname()[1]
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        """Background thread: accept one connection, record each command line, and answer a 'g' with the stored capture."""
        try:
            connection, _ = self.server.accept()
        except OSError:
            return
        connection.settimeout(0.2)
        pending = b""
        deadline = time.time() + 20
        try:
            while time.time() < deadline:
                try:
                    chunk = connection.recv(1024)
                except socket.timeout:
                    continue
                if not chunk:
                    break   # the logger closed the port
                pending += chunk
                while b"\n" in pending:
                    raw, pending = pending.split(b"\n", 1)
                    command = raw.decode("ascii", "replace").strip()
                    self.commands.append(command)
                    if command.startswith("g"):
                        connection.sendall(self.capture_text.replace("\n", "\r\n").encode("utf-8"))
        except OSError:
            pass
        finally:
            connection.close()
            self.server.close()


def live_path_checks(log, record):
    """Step 8: run two loggers through their live-capture code against FakeTeensy."""
    try:
        import serial  # noqa: F401  (only to find out whether pyserial is installed)
    except ImportError:
        log.write("NOT RUN: pyserial is not installed, so the live-capture path could not be exercised.")
        record("live_path", False)
        return

    # Check A: a non-motor logger captures a full run over the socket.
    capture_path = BUILD_DIR / "cv004_imu_raw_pjrc.txt"
    if not capture_path.is_file():
        log.write("NOT RUN: no simulated CV-004 capture to serve.")
        record("live_path", False)
    else:
        capture_text = capture_path.read_text(encoding="utf-8")
        expected_rows = capture_shape(capture_text)[1]
        fake = FakeTeensy(capture_text)
        out_base = BUILD_DIR / "live_cv004"
        code, out, err = run([sys.executable, str(TESTS_DIR / "cv004_imu_raw" / "cv004_imu_raw.py"),
                              "--port", "socket://127.0.0.1:{}".format(fake.port), "--seconds", "3",
                              "--out", str(out_base)], timeout=60)
        fake.thread.join(timeout=5)
        machine = [line for line in out.splitlines() if line.startswith("LOGGER,")]
        files_ok = all((BUILD_DIR / name).is_file() for name in
                       ("live_cv004.txt", "live_cv004.csv", "live_cv004_verdict.txt"))
        csv_rows = 0
        if files_ok:
            csv_rows = len((BUILD_DIR / "live_cv004.csv").read_text(encoding="utf-8").splitlines()) - 1
        ok = (code == 0 and bool(machine) and "rows={},".format(expected_rows) in machine[-1]
              and "header_match=1" in machine[-1] and fake.commands[:2] == ["t3", "g"]
              and fake.commands[-1:] == ["s"] and files_ok and csv_rows == expected_rows)
        record("live_path", ok)
        log.write("{}  live capture, CV-004 logger over socket://  exit={} commands_received={} csv_rows={} (expected {})".format(
            "PASS" if ok else "FAIL", code, fake.commands, csv_rows, expected_rows))
        log.write("        " + (machine[-1] if machine else "(no LOGGER line)"))
        if err.strip():
            log.write("\n".join("        stderr: " + line for line in err.strip().splitlines()[-8:]))

    # Check B: a motor logger must send NOTHING unless the operator types GO.
    fake = FakeTeensy("")
    done = subprocess.run([sys.executable, str(TESTS_DIR / "cv007_motor_noload" / "cv007_motor_noload.py"),
                           "--port", "socket://127.0.0.1:{}".format(fake.port), "--channel", "0"],
                          input=b"no\n", capture_output=True, timeout=60)
    fake.server.close()
    declined_ok = (done.returncode == 0 and fake.commands == [] and b"cancelled" in done.stdout)
    record("live_path", declined_ok)
    log.write("{}  motor logger with GO declined: exit={} commands_received={}".format(
        "PASS" if declined_ok else "FAIL", done.returncode, fake.commands))


def main(argv=None):
    """Run every verification step in order, write the log, and return 0 only if all of them passed."""
    parser = argparse.ArgumentParser(description="Build every BRI sketch for the PC and replay its output through its logger.")
    parser.add_argument("--zig", help="path to the zig executable (default: search ZIG, PATH, conda)")
    parser.add_argument("--log", help="also write the verification log to this file")
    parser.add_argument("--pin-csv", help="check BRI_Pins.h against this CSV as well as the copy in the library")
    parser.add_argument("--jobs", type=int, default=6, help="how many builds to run at once (default 6)")
    args = parser.parse_args(argv)

    log = Log()
    totals = {}   # step name -> [attempted, passed]

    def record(step, ok):
        """Count one attempt for a step, and one pass if it succeeded (for the summary table)."""
        attempted, passed = totals.get(step, [0, 0])
        totals[step] = [attempted + 1, passed + (1 if ok else 0)]

    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    log.write("BRI firmware kit - PC verification log")
    log.write("date: {}".format(datetime.datetime.now().isoformat(timespec="seconds")))
    log.write("python: {}".format(sys.version.split()[0]))
    log.write("NOTE: these are PC stub builds and simulated runs.  Nothing here was compiled")
    log.write("      for a Teensy or run on hardware.")

    # ---- 1. pin check ---------------------------------------------------------
    log.section("1. PIN CHECK  (BRI_Pins.h against the pin map CSV)")
    pin_ok = True
    pin_commands = [[sys.executable, str(FIRMWARE_DIR / "tools" / "check_pins.py")]]
    if args.pin_csv:
        pin_commands.append(pin_commands[0] + ["--csv", args.pin_csv])
    for command in pin_commands:
        code, out, err = run(command)
        log.write((out + err).strip())
        ok = (code == 0) and ("PINCHECK,PASS" in out)
        record("pin_check", ok)
        pin_ok = pin_ok and ok

    # ---- 2. header check --------------------------------------------------------
    log.section("2. HEADER CHECK  (sketch HEADER string against logger EXPECTED_HEADER)")
    folders = sketch_dirs()
    for folder in folders:
        ino_text = (folder / (folder.name + ".ino")).read_text(encoding="utf-8")
        py_path = folder / (folder.name + ".py")
        ino_match = re.search(r'HEADER\[\]\s*=\s*"([^"]+)"', ino_text)
        py_match = re.search(r'EXPECTED_HEADER\s*=\s*"([^"]+)"', py_path.read_text(encoding="utf-8")) if py_path.is_file() else None
        ok = bool(ino_match and py_match and ino_match.group(1) == py_match.group(1))
        record("header_check", ok)
        fields = len(ino_match.group(1).split(",")) if ino_match else 0
        log.write("{}  {}  ({} fields){}".format("PASS" if ok else "FAIL", folder.name, fields,
                                                 "" if ok else "  <-- sketch and logger disagree, or logger missing"))

    # ---- 3. stub builds -----------------------------------------------------------
    log.section("3. STUB BUILDS  (zig c++, PC target, both encoder backends)")
    zig = find_zig(args.zig)
    executables = {}
    if zig is None:
        log.write("zig was NOT FOUND.  No sketch was built, run or replayed.")
        log.write("Install zig (https://ziglang.org) or pass --zig <path>.")
        for folder in folders:
            for backend in BACKENDS:
                record("stub_build", False)
    else:
        env = tool_environment(zig)
        code, out, err = run([zig, "version"], env=env)
        log.write("zig: {}  (version {})".format(zig, (out + err).strip()))
        log.write("flags: -std=gnu++17 -O1 -Wall -Wextra   backends: pjrc = PJRC Encoder, hwquad = -DBRI_USE_HW_QUADENC")
        jobs = [(folder, backend) for folder in folders for backend in BACKENDS]
        with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
            outcomes = list(pool.map(lambda job: build_sketch(zig, env, job[0], job[1]), jobs))
        for (folder, backend), (ok, executable, output) in zip(jobs, outcomes):
            record("stub_build", ok)
            note = "no warnings" if not output else "compiler output below"
            log.write("{}  {:26s} {:7s} {}".format("PASS" if ok else "FAIL", folder.name, backend, note))
            if output:
                log.write("\n".join("      " + line for line in output.splitlines()[:40]))
            if ok:
                executables[(folder.name, backend)] = executable

    # ---- 4 and 5. simulated runs and logger replays ----------------------------------
    log.section("4. SIMULATED RUNS  and  5. LOGGER REPLAYS")
    for folder in folders:
        for backend in BACKENDS:
            tag = "{}_{}".format(folder.name, backend)
            executable = executables.get((folder.name, backend))
            if executable is None or folder.name not in SCENARIOS:
                record("sim_run", False)
                record("replay", False)
                log.write("SKIP  {}  (no executable or no scenario) - counted as not passed".format(tag))
                continue
            script, milliseconds = SCENARIOS[folder.name]
            code, out, err, capture_path = simulate(executable, script, milliseconds, tag)
            header_fields, rows, results, wrong = capture_shape(out)
            summary = parse_sim_summary(err)
            violations = outputs_safe(summary)
            run_ok = (code == 0 and header_fields > 0 and rows > 0 and results > 0 and wrong == 0
                      and "!ROWLEN" not in out and not violations)
            record("sim_run", run_ok)
            log.write("{}  RUN     {:34s} exit={} header_fields={} rows={} rows_wrong_length={} RESULT_lines={} max_duty={:.3f}".format(
                "PASS" if run_ok else "FAIL", tag, code, header_fields, rows, wrong, results,
                summary.get("max_abs_duty", float("nan"))))
            for violation in violations:
                log.write("        safety: " + violation)
            for line in out.replace("\r", "").split("\n"):
                if line.startswith("RESULT,"):
                    log.write("        " + line.strip())

            logger = folder / (folder.name + ".py")
            out_base = BUILD_DIR / (tag + "_replay")
            code, out2, err2 = run([sys.executable, str(logger), "--replay", str(capture_path), "--out", str(out_base)])
            machine = [line for line in out2.splitlines() if line.startswith("LOGGER,")]
            fields = {}
            if machine:
                for item in machine[-1].split(",")[3:]:
                    key, _, value = item.partition("=")
                    fields[key] = value
            replay_ok = (code == 0 and bool(machine) and fields.get("header_match") == "1"
                         and fields.get("bad_rows") == "0" and fields.get("rows") == str(rows)
                         and fields.get("fields") == str(header_fields) and int(fields.get("checks", "0")) > 0
                         and fields.get("fw_results") == str(results))
            record("replay", replay_ok)
            log.write("{}  REPLAY  {:34s} exit={} {}".format(
                "PASS" if replay_ok else "FAIL", tag, code, machine[-1] if machine else "(no LOGGER line)"))
            for line in out2.splitlines():
                if line.startswith(("  PASS", "  FAIL", "  INFO", "  PROTOCOL", "OVERALL")):
                    log.write("        " + line)
            if err2.strip():
                log.write("\n".join("        stderr: " + line for line in err2.strip().splitlines()[-8:]))

    # ---- 6. safety scenarios -----------------------------------------------------------
    log.section("6. SAFETY SCENARIOS  (simulated; default PJRC backend)")
    for scenario in SAFETY_SCENARIOS:
        if "selftest" in scenario:
            # Simulation-only probe sketches are built here, on demand.
            executable = None
            if zig is not None:
                built, executable, output = build_sketch(zig, env, SIM_DIR / "selftest" / scenario["selftest"], "pjrc")
                if not built:
                    executable = None
                    log.write("\n".join("      " + line for line in output.splitlines()[:20]))
        else:
            executable = executables.get((scenario["sketch"], "pjrc"))
        if executable is None:
            record("safety", False)
            log.write("SKIP  {}  (no executable) - counted as not passed".format(scenario["name"]))
            continue
        code, out, err, _ = simulate(executable, scenario["script"], scenario["ms"], "safety_" + scenario["name"])
        summary = parse_sim_summary(err)
        problems = outputs_safe(summary)
        stops = [line.split(",", 2)[2].strip() for line in out.replace("\r", "").split("\n") if line.startswith("!STOP,")]
        if scenario["stop"] is None:
            if "!GO," in out:
                problems.append("a run started although no g was sent")
        elif scenario["stop"] not in stops:
            problems.append("expected '!STOP ... {}' but saw {}".format(scenario["stop"], stops or "no stop"))
        last_drive = summary.get("last_drive_ms", float("nan"))
        if "max_duty" in scenario and summary.get("max_abs_duty", 9) > scenario["max_duty"]:
            problems.append("duty reached {:.3f}, limit {:.3f}".format(summary.get("max_abs_duty", 9), scenario["max_duty"]))
        if "drive_by_ms" in scenario and not last_drive <= scenario["drive_by_ms"]:
            problems.append("outputs still active at {:.1f} ms, should be off by {} ms".format(last_drive, scenario["drive_by_ms"]))
        if "drive_after_ms" in scenario and not last_drive >= scenario["drive_after_ms"]:
            problems.append("outputs went off at {:.1f} ms, before {} ms (scenario did not exercise the stop)".format(
                last_drive, scenario["drive_after_ms"]))
        if "min_duty" in scenario and not summary.get("max_abs_duty", 0) >= scenario["min_duty"]:
            problems.append("duty only reached {:.3f}; the limit was never tested".format(summary.get("max_abs_duty", 0)))
        if scenario.get("firmware_pass"):
            verdicts = [line.split(",")[2] for line in out.replace("\r", "").split("\n") if line.startswith("RESULT,")]
            if not verdicts or verdicts[-1] != "PASS":
                problems.append("the firmware's own verdict was {}".format(verdicts[-1] if verdicts else "missing"))
        for key, value in scenario.get("expect", {}).items():
            if summary.get(key) != value:
                problems.append("{} = {} but expected {}".format(key, summary.get(key), value))
        ok = (code == 0) and not problems
        record("safety", ok)
        log.write("{}  {:34s} stops={} outputs_off_at={:.1f} ms max_duty={:.3f}".format(
            "PASS" if ok else "FAIL", scenario["name"], stops, last_drive, summary.get("max_abs_duty", float("nan"))))
        log.write("        why: " + scenario["why"])
        for problem in problems:
            log.write("        PROBLEM: " + problem)

    # ---- 7. fault-finding scenarios ----------------------------------------------------------
    log.section("7. FAULT-FINDING SCENARIOS  (simulated hardware made wrong on purpose)")
    for scenario in FAULT_SCENARIOS:
        tag = "fault_" + scenario["name"]
        executable = executables.get((scenario["sketch"], "pjrc"))
        if executable is None:
            record("fault_finding", False)
            log.write("SKIP  {}  (no executable) - counted as not passed".format(scenario["name"]))
            continue
        code, out, err, capture_path = simulate(executable, scenario["script"], scenario["ms"], tag)
        problems = outputs_safe(parse_sim_summary(err))
        verdicts = [line.split(",")[2] for line in out.replace("\r", "").split("\n") if line.startswith("RESULT,")]
        firmware_final = verdicts[-1] if verdicts else "none"
        if firmware_final != scenario["fw"]:
            problems.append("firmware final verdict {} but expected {}".format(firmware_final, scenario["fw"]))
        if scenario["text"] not in out:
            problems.append("firmware output lacks '{}'".format(scenario["text"]))
        logger = TESTS_DIR / scenario["sketch"] / (scenario["sketch"] + ".py")
        logger_code, out2, err2 = run([sys.executable, str(logger), "--replay", str(capture_path),
                                       "--out", str(BUILD_DIR / (tag + "_replay"))])
        if logger_code != scenario["logger_exit"]:
            problems.append("logger exit code {} but expected {}".format(logger_code, scenario["logger_exit"]))
        if err2.strip():
            problems.append("logger wrote to stderr: " + err2.strip().splitlines()[-1])
        ok = (code == 0) and not problems
        record("fault_finding", ok)
        overall = [line for line in out2.splitlines() if line.startswith("OVERALL")]
        log.write("{}  {:38s} firmware={} logger_exit={}  {}".format(
            "PASS" if ok else "FAIL", scenario["name"], firmware_final, logger_code, overall[-1] if overall else ""))
        log.write("        why: " + scenario["why"])
        for line in out2.splitlines():
            if line.startswith("  FAIL"):
                log.write("        report:" + line)
        for problem in problems:
            log.write("        PROBLEM: " + problem)

    # ---- 8. live-capture path -----------------------------------------------------------------
    log.section("8. LIVE-CAPTURE PATH  (logger talking to a fake serial port on a local socket)")
    live_path_checks(log, record)

    # ---- summary ---------------------------------------------------------------------------
    log.section("SUMMARY")
    all_ok = True
    for step in ("pin_check", "header_check", "stub_build", "sim_run", "replay", "safety", "fault_finding", "live_path"):
        attempted, passed = totals.get(step, [0, 0])
        ok = attempted > 0 and attempted == passed
        all_ok = all_ok and ok
        log.write("{:14s} {:2d} of {:2d} passed   {}".format(step, passed, attempted, "OK" if ok else "NOT OK"))
    log.write("")
    log.write("VERIFICATION {}".format("PASSED (PC stub builds and simulation only)" if all_ok else "FAILED"))
    log.write("Not verified by this script: compiling for the Teensy 4.1, and anything on real hardware.")

    text = "\n".join(log.lines) + "\n"
    (BUILD_DIR / "firmware_verification.txt").write_text(text, encoding="utf-8")
    (BUILD_DIR / "verification_summary.json").write_text(json.dumps(
        {"totals": totals, "all_ok": all_ok, "zig": zig}, indent=2), encoding="utf-8")
    if args.log:
        pathlib.Path(args.log).write_text(text, encoding="utf-8")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
