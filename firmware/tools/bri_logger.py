#!/usr/bin/env python3
"""
bri_logger.py - the shared PC side of every BRI bench test.

Each test folder holds a small script (cv001_power_usb.py, ...).  Those
scripts contain only what is special about their test: the expected CSV
header, which commands start the run, and the pass/fail rules.  Everything
they have in common lives here:

    * finding the Teensy's serial port            find_teensy_port()
    * talking to the Teensy and recording output  capture_from_serial()
    * splitting the output into its line kinds    parse_capture()
    * checking the output obeys the protocol      check_protocol()
    * saving the raw text and a clean CSV         save_outputs()
    * the command line every test script shares   run_test()

THE PROTOCOL (the firmware side is documented in BRI_Telemetry.h)

    # text                       for humans
    t_ms,col2,col3,...           CSV header (always starts with "t_ms,")
    1234,0.5,7,...               data row, same number of fields as the header
    !NAME,t_ms,detail            event or fault; detail is "key=value key=value"
    RESULT,CV-00N,VERDICT,text   verdict from the firmware (PASS / FAIL / INFO)

Lines this logger adds to a capture itself start with "#LOGGER" so they can
never be mistaken for firmware output.

The only package needed is pyserial (and only when talking to hardware;
--replay works without it).  matplotlib is used only if you ask for --plot.
"""
import argparse
import datetime
import math
import pathlib
import queue
import sys
import threading
import time

# USB vendor id of PJRC, the maker of the Teensy.  Every Teensy reports it,
# which is how the right serial port is found without asking.
TEENSY_USB_VID = 0x16C0

# Folder layout: this file is firmware/tools/bri_logger.py
FIRMWARE_DIR = pathlib.Path(__file__).resolve().parent.parent
LOG_DIR = FIRMWARE_DIR / "logs"

# After a RESULT line, wait this long for more lines before deciding the
# firmware has finished talking.
QUIET_AFTER_RESULT_S = 1.0


# ---------------------------------------------------------------------------
# Small maths helpers (kept here so the scripts need no numpy)
# ---------------------------------------------------------------------------
def mean(values):
    """Average of a list of numbers; NaN for an empty list."""
    values = [v for v in values if not math.isnan(v)]
    if not values:
        return float("nan")
    return sum(values) / len(values)


def stdev(values):
    """Standard deviation (spread) of a list of numbers; 0 for fewer than 2."""
    values = [v for v in values if not math.isnan(v)]
    if len(values) < 2:
        return 0.0
    centre = sum(values) / len(values)
    return math.sqrt(sum((v - centre) ** 2 for v in values) / len(values))


def within(value, low, high):
    """True if low <= value <= high (False for NaN)."""
    return (not math.isnan(value)) and low <= value <= high


# ---------------------------------------------------------------------------
# A parsed capture
# ---------------------------------------------------------------------------
class Capture:
    """Everything found in one recording of firmware output.

    raw_lines     every line exactly as received
    comments      the "# ..." lines (text after the '#')
    header        list of column names from the LAST header line, or None
    rows          data rows of the last run, each a dict {column: number}
    row_texts     the same rows as the original text (for the CSV file)
    events        "!" lines of the whole capture, each a dict with
                  name, t_ms, detail (text) and values ({key: number or text})
    results       firmware RESULT lines of the last run: (verdict, text)
    result_ids    the test id each of those RESULT lines carried
    header_count  how many header lines were seen (more than 1 = several runs)
    bad_rows      data lines that did not match the header
    problems      protocol problems noticed while parsing
    logger_notes  "#LOGGER" lines added by this program during capture
    """

    def __init__(self):
        """Start with an empty capture; parse_capture() fills it in line by line."""
        self.raw_lines = []
        self.comments = []
        self.header = None
        self.rows = []
        self.row_texts = []
        self.events = []
        self.results = []
        self.result_ids = []
        self.header_count = 0
        self.bad_rows = 0
        self.problems = []
        self.logger_notes = []

    def column(self, name):
        """All values of one column, as a list of numbers."""
        return [row[name] for row in self.rows if name in row]

    def events_named(self, name):
        """All events with the given name, in order."""
        return [event for event in self.events if event["name"] == name]

    def last_event_value(self, name, key):
        """Value of `key` in the most recent event called `name`, or None."""
        for event in reversed(self.events_named(name)):
            if key in event["values"]:
                return event["values"][key]
        return None

    def stop_reasons(self):
        """The reason given by each "!STOP" event, in order."""
        return [event["detail"].strip() for event in self.events_named("STOP")]

    def reset_seen(self):
        """Did the Teensy restart during the capture?  Returns a list of
        reasons (empty list = no sign of a reset)."""
        reasons = []
        boots = self.column("boot_count")
        if boots and len(set(boots)) > 1:
            reasons.append("boot_count changed during the run")
        times = self.column("t_ms")
        if any(later < earlier for earlier, later in zip(times, times[1:])):
            reasons.append("t_ms went backwards")
        if any("connection lost" in note for note in self.logger_notes):
            reasons.append("serial connection was lost")
        return reasons


def parse_key_values(text):
    """Turn "vout=5.02 iout=1.5 complete" into {"vout": 5.02, "iout": 1.5}.

    Words without '=' are ignored here (they stay available in the detail
    text).  Values that are not numbers are kept as text.
    """
    values = {}
    for word in text.split():
        if "=" not in word:
            continue
        key, _, value = word.partition("=")
        try:
            # int(x, 0) understands "0x68"; float() understands the rest.
            values[key] = float(int(value, 0)) if value.lower().startswith("0x") else float(value)
        except ValueError:
            values[key] = value
    return values


def parse_capture(lines):
    """Sort the lines of a recording into a Capture object."""
    capture = Capture()
    for raw in lines:
        line = raw.rstrip("\r\n")
        capture.raw_lines.append(line)
        if not line.strip():
            continue
        if line.startswith("#LOGGER"):
            capture.logger_notes.append(line)
        elif line.startswith("#"):
            capture.comments.append(line[1:].strip())
        elif line.startswith("!"):
            parts = line[1:].split(",", 2)
            name = parts[0]
            try:
                t_ms = float(parts[1]) if len(parts) > 1 else float("nan")
            except ValueError:
                t_ms = float("nan")
            detail = parts[2] if len(parts) > 2 else ""
            capture.events.append({"name": name, "t_ms": t_ms, "detail": detail,
                                   "values": parse_key_values(detail)})
            if name == "ROWLEN":
                capture.problems.append("firmware reported a row/header length mismatch: " + detail)
        elif line.startswith("RESULT,"):
            parts = line.split(",", 3)
            if len(parts) < 4:
                capture.problems.append("malformed RESULT line: " + line)
                continue
            capture.result_ids.append(parts[1])
            capture.results.append((parts[2], parts[3].strip()))
        elif line.startswith("t_ms,"):
            # A header starts a new run: forget the rows and results of the
            # previous one so the evaluation is about the latest run.
            capture.header = line.split(",")
            capture.header_count += 1
            capture.rows = []
            capture.row_texts = []
            capture.results = []
            capture.result_ids = []
        else:
            # Anything else must be a data row.
            if capture.header is None:
                capture.bad_rows += 1
                capture.problems.append("data before any header: " + line[:60])
                continue
            fields = line.split(",")
            if len(fields) != len(capture.header):
                capture.bad_rows += 1
                capture.problems.append("row has {} fields, header has {}: {}".format(
                    len(fields), len(capture.header), line[:60]))
                continue
            try:
                numbers = [float(field) for field in fields]
            except ValueError:
                capture.bad_rows += 1
                capture.problems.append("row has a non-numeric field: " + line[:60])
                continue
            capture.rows.append(dict(zip(capture.header, numbers)))
            capture.row_texts.append(line)
    return capture


def check_protocol(capture, test_id, expected_header):
    """Return a list of protocol problems (empty list = the capture is well formed)."""
    problems = list(capture.problems)
    expected = expected_header.split(",")
    if capture.header is None:
        problems.append("no CSV header line (starting 't_ms,') was received")
    elif capture.header != expected:
        problems.append("header differs from what this script expects:\n      got      {}\n      expected {}".format(
            ",".join(capture.header), expected_header))
    if not capture.results:
        problems.append("no RESULT line was received (run cut short?)")
    for result_id in capture.result_ids:
        if result_id != test_id:
            problems.append("RESULT line is for {} but this script is for {}".format(result_id, test_id))
            break
    return problems


# ---------------------------------------------------------------------------
# Serial port
# ---------------------------------------------------------------------------
def find_teensy_port():
    """Return the name of the serial port of the first Teensy found.

    Raises SystemExit with a helpful message if none (or pyserial) is missing.
    """
    try:
        from serial.tools import list_ports
    except ImportError:
        raise SystemExit("pyserial is not installed.  Run:  python -m pip install pyserial")
    candidates = [port for port in list_ports.comports() if port.vid == TEENSY_USB_VID]
    if not candidates:
        seen = ", ".join(port.device for port in list_ports.comports()) or "none"
        raise SystemExit("No Teensy found (USB vendor id 0x16C0).  Ports present: {}.\n"
                         "Plug it in, or name the port with --port COM5".format(seen))
    if len(candidates) > 1:
        print("several Teensy ports found ({}); using the first.  Use --port to choose.".format(
            ", ".join(port.device for port in candidates)))
    return candidates[0].device


def _keyboard_reader(typed):
    """Background thread: put every line typed at the keyboard into a queue.

    Why a thread: reading the keyboard blocks, and the main loop must keep
    reading the serial port while you are thinking about what to type.
    """
    try:
        for line in sys.stdin:
            typed.put(line.strip())
    except (OSError, ValueError):
        pass  # keyboard not available (for example when run by another program)


def capture_from_serial(port, seconds, start_commands):
    """Send the start commands, then record everything the Teensy says.

    Recording ends when the firmware has printed its RESULT lines and gone
    quiet, or after `seconds`.  Whatever you type (plus Enter) is sent to the
    Teensy as a command, so the per-test entries (v5.02, b, e, ...) work.
    Returns the list of received lines.
    """
    try:
        import serial
    except ImportError:
        raise SystemExit("pyserial is not installed.  Run:  python -m pip install pyserial")

    def open_port():
        """Open the serial port (or a pyserial URL).  A small function because it is needed again after a reconnect."""
        # Opening the port raises DTR, which is what makes "if (Serial)" true
        # on the Teensy.  Closing it (or a crash of this program) drops DTR,
        # and the firmware's safety layer then stops any running motor.
        # serial_for_url() accepts an ordinary port name ("COM5", "/dev/ttyACM0")
        # and also pyserial URLs such as "socket://127.0.0.1:5000", which is
        # how sim/build_and_replay.py tests this function without a Teensy.
        return serial.serial_for_url(port, 115200, timeout=0.05)

    def send(link, command):
        """Send one command line to the Teensy, then pause briefly so commands do not run together."""
        link.write((command + "\n").encode("ascii", "ignore"))
        link.flush()
        time.sleep(0.05)   # give the firmware a moment between commands

    lines = []
    typed = queue.Queue()
    threading.Thread(target=_keyboard_reader, args=(typed,), daemon=True).start()

    link = open_port()
    time.sleep(0.5)             # let the Teensy notice the port is open
    link.reset_input_buffer()   # drop the boot banner; "g" prints it again
    for command in start_commands:
        send(link, command)

    deadline = time.time() + seconds
    pending = b""
    last_result_time = None
    row_count = 0
    last_progress = time.time()
    try:
        while time.time() < deadline:
            try:
                chunk = link.read(4096)
                while not typed.empty():
                    send(link, typed.get())
            except (serial.SerialException, OSError):
                # The port vanished: the Teensy reset or the cable came out.
                note = "#LOGGER serial connection lost at {}".format(datetime.datetime.now().isoformat(timespec="seconds"))
                lines.append(note)
                print(note)
                try:
                    link.close()
                except (serial.SerialException, OSError):
                    pass
                link = None
                while time.time() < deadline and link is None:
                    time.sleep(0.5)
                    try:
                        link = open_port()
                    except (serial.SerialException, OSError):
                        link = None
                if link is None:
                    break
                lines.append("#LOGGER serial reconnected")
                print("#LOGGER serial reconnected")
                time.sleep(0.5)
                for command in start_commands:
                    send(link, command)
                continue

            pending += chunk
            while b"\n" in pending:
                raw, pending = pending.split(b"\n", 1)
                text = raw.decode("utf-8", "replace").rstrip("\r")
                lines.append(text)
                is_row = text[:1].isdigit() or text[:1] == "-"
                if is_row:
                    row_count += 1
                else:
                    print(text)   # show banner, events and results as they arrive
                if text.startswith("RESULT,"):
                    last_result_time = time.time()
            if time.time() - last_progress > 2.0:
                last_progress = time.time()
                print("  ... {} data rows so far".format(row_count))
            if last_result_time is not None and time.time() - last_result_time > QUIET_AFTER_RESULT_S:
                break

        if link is not None:
            # Always finish with "s": it puts the drivers to sleep, and if the
            # run was still going it makes the firmware print its results.
            send(link, "s")
            if last_result_time is None:
                tail_deadline = time.time() + 1.5
                while time.time() < tail_deadline:
                    pending += link.read(4096)
                    while b"\n" in pending:
                        raw, pending = pending.split(b"\n", 1)
                        text = raw.decode("utf-8", "replace").rstrip("\r")
                        lines.append(text)
                        if not (text[:1].isdigit() or text[:1] == "-"):
                            print(text)
    finally:
        if link is not None:
            link.close()
    return lines


# ---------------------------------------------------------------------------
# Files
# ---------------------------------------------------------------------------
def save_outputs(capture, out_base, save_raw, summary_lines):
    """Write <out_base>.txt (raw text), <out_base>.csv (header + rows) and
    <out_base>_verdict.txt (the summary printed on screen).

    `save_raw` is False for --replay, where the raw text already exists.
    Returns the list of files written.
    """
    out_base = pathlib.Path(out_base)
    out_base.parent.mkdir(parents=True, exist_ok=True)
    written = []
    if save_raw:
        raw_path = out_base.with_name(out_base.name + ".txt")
        raw_path.write_text("\n".join(capture.raw_lines) + "\n", encoding="utf-8")
        written.append(raw_path)
    if capture.header is not None:
        csv_path = out_base.with_name(out_base.name + ".csv")
        csv_path.write_text("\n".join([",".join(capture.header)] + capture.row_texts) + "\n", encoding="utf-8")
        written.append(csv_path)
    verdict_path = out_base.with_name(out_base.name + "_verdict.txt")
    verdict_path.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    written.append(verdict_path)
    return written


def save_plot(capture, columns, title, out_base):
    """Plot the named columns against time and save a PNG next to the logs.

    matplotlib is imported here, not at the top, so the logger works on a
    machine that does not have it unless --plot is asked for.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")   # draw to a file; no window needed
        import matplotlib.pyplot as plt
    except ImportError:
        print("--plot needs matplotlib:  python -m pip install matplotlib")
        return None
    seconds = [t / 1000.0 for t in capture.column("t_ms")]
    columns = [name for name in columns if capture.header and name in capture.header]
    if not seconds or not columns:
        print("nothing to plot")
        return None
    figure, axes = plt.subplots(len(columns), 1, sharex=True, figsize=(9, 2.2 * len(columns)), squeeze=False)
    for axis, name in zip(axes[:, 0], columns):
        axis.plot(seconds, capture.column(name), linewidth=1.0)
        axis.set_ylabel(name)
        axis.grid(True, alpha=0.3)
    axes[-1, 0].set_xlabel("time since boot (s)")
    axes[0, 0].set_title(title)
    figure.tight_layout()
    png_path = pathlib.Path(out_base).with_name(pathlib.Path(out_base).name + ".png")
    figure.savefig(png_path, dpi=120)
    plt.close(figure)
    return png_path


# ---------------------------------------------------------------------------
# The command line shared by every test script
# ---------------------------------------------------------------------------
def overall_verdict(verdicts, protocol_ok):
    """Combine individual verdicts the same way the firmware does:
    any FAIL (or a broken protocol) -> FAIL; otherwise anything still waiting
    for the operator (TODO), or nothing actually passed -> INFO; else PASS."""
    if not protocol_ok or "FAIL" in verdicts:
        return "FAIL"
    if "TODO" in verdicts or "PASS" not in verdicts:
        return "INFO"
    return "PASS"


def run_test(test_id, title, expected_header, evaluate, start_commands,
             default_seconds, capture_margin_s=5.0, motor_test=False,
             operator_notes="", plot_columns=(), add_arguments=None, argv=None):
    """Run one bench test from the command line.  Returns the exit code:
    0 = PASS or INFO, 1 = FAIL, 2 = the capture did not obey the protocol.

    test_id          "CV-004"
    title            "Raw IMU data"
    expected_header  the exact header string the sketch prints
    evaluate         function(capture, args) -> list of (verdict, text), with
                     verdict one of "PASS", "FAIL", "INFO", "TODO"
    start_commands   function(args) -> list of command strings that start the run
    default_seconds  value of --seconds when not given
    capture_margin_s extra recording time beyond --seconds, so the RESULT
                     lines at the end are not cut off
    motor_test       True if the test moves a motor: the operator must then
                     type GO before anything is sent
    operator_notes   text shown before the run (what to do, what to type)
    plot_columns     columns drawn by --plot
    add_arguments    function(parser) adding test-specific options
    """
    parser = argparse.ArgumentParser(description="{} {} - BRI bench test logger".format(test_id, title))
    parser.add_argument("--port", help="serial port (default: find the Teensy by its USB id 0x16C0)")
    parser.add_argument("--seconds", type=float, default=default_seconds,
                        help="how long the test runs (default %(default)s)")
    parser.add_argument("--out", help="output file name without extension (default: firmware/logs/<test>_<time>)")
    parser.add_argument("--replay", help="evaluate a saved raw capture (.txt) instead of talking to hardware")
    parser.add_argument("--plot", action="store_true", help="also save a PNG plot (needs matplotlib)")
    parser.add_argument("--yes", action="store_true", help="motor tests: skip the 'type GO' confirmation")
    if add_arguments is not None:
        add_arguments(parser)
    args = parser.parse_args(argv)

    print("==== {} {} ====".format(test_id, title))
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    file_tag = test_id.lower().replace("-", "")

    if args.replay:
        replay_path = pathlib.Path(args.replay)
        lines = replay_path.read_text(encoding="utf-8", errors="replace").splitlines()
        out_base = args.out or str(LOG_DIR / (replay_path.stem + "_replay"))
        print("replaying {}".format(replay_path))
    else:
        port = args.port or find_teensy_port()
        out_base = args.out or str(LOG_DIR / "{}_{}".format(file_tag, stamp))
        if operator_notes:
            print(operator_notes.strip())
        commands = start_commands(args)
        print("port {}   will send: {}".format(port, " ".join(commands)))
        if motor_test and not args.yes:
            # The explicit go: nothing is sent to the robot until this is typed.
            answer = input("THE MOTOR WILL MOVE.  Type GO and press Enter to start (anything else cancels): ")
            if answer.strip().upper() != "GO":
                print("cancelled; nothing was sent")
                return 0
        lines = capture_from_serial(port, args.seconds + capture_margin_s, commands)

    capture = parse_capture(lines)
    problems = check_protocol(capture, test_id, expected_header)
    header_match = capture.header is not None and ",".join(capture.header) == expected_header

    summary = []
    summary.append("{} {}".format(test_id, title))
    summary.append("protocol: header {} ({} fields), {} data rows, {} bad rows, {} firmware RESULT lines".format(
        "matches" if header_match else "DOES NOT MATCH",
        len(capture.header) if capture.header else 0, len(capture.rows), capture.bad_rows, len(capture.results)))
    for problem in problems[:10]:
        summary.append("  PROTOCOL PROBLEM: " + problem)
    if len(problems) > 10:
        summary.append("  ... and {} more".format(len(problems) - 10))

    summary.append("firmware verdicts:")
    for verdict, text in capture.results:
        summary.append("  {:4s} {}".format(verdict, text))
    firmware_overall = capture.results[-1][0] if capture.results else "NONE"

    summary.append("logger evaluation (from the recorded data):")
    logger_results = []
    if capture.header is not None and header_match:
        logger_results = evaluate(capture, args)
    else:
        summary.append("  skipped: no usable header")
    for verdict, text in logger_results:
        shown = "INFO" if verdict == "TODO" else verdict
        marker = " (operator)" if verdict == "TODO" else ""
        summary.append("  {:4s}{} {}".format(shown, marker, text))

    overall = overall_verdict([verdict for verdict, _ in logger_results], not problems)
    summary.append("OVERALL (logger): {}    firmware said: {}".format(overall, firmware_overall))
    # One machine-readable line, used by sim/build_and_replay.py.
    summary.append("LOGGER,{},{},rows={},fields={},header_match={},bad_rows={},fw_results={},fw_overall={},checks={}".format(
        test_id, overall, len(capture.rows), len(capture.header) if capture.header else 0,
        1 if header_match else 0, capture.bad_rows, len(capture.results), firmware_overall, len(logger_results)))
    print("\n".join(summary))

    written = save_outputs(capture, out_base, save_raw=not args.replay, summary_lines=summary)
    if args.plot:
        png = save_plot(capture, plot_columns, "{} {}".format(test_id, title), out_base)
        if png is not None:
            written.append(png)
    for path in written:
        print("saved {}".format(path))

    if problems:
        return 2
    return 1 if overall == "FAIL" else 0
