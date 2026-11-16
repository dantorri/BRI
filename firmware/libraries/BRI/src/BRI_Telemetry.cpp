// BRI_Telemetry.cpp - serial protocol implementation (see BRI_Telemetry.h).
#include "BRI_Telemetry.h"
#include "BRI_Config.h"
#include "BRI_Pins.h"
#include "BRI_Encoder.h"
#include "BRI_System.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char* s_test_id = "CV-000";  // e.g. "CV-004"
static const char* s_title = "";          // e.g. "Raw IMU data"
static uint16_t s_header_fields = 0;      // number of columns in the current header
static uint16_t s_row_fields = 0;         // fields printed so far in the row being built
static bool s_rowlen_reported = false;    // so a mismatch is reported once, not 50 times a second

static uint16_t s_pass_count = 0;         // tally of RESULT lines in this run
static uint16_t s_fail_count = 0;
static uint16_t s_info_count = 0;
static uint16_t s_todo_count = 0;

static char s_line[BRI_CMD_MAX];          // command characters received so far
static uint8_t s_line_length = 0;

static uint32_t s_led_last_ms = 0;        // when the heartbeat LED last toggled
static bool s_led_on = false;

// Open USB serial, wait briefly for the PC, print the banner.
void briTelemBegin(const char* test_id, const char* title) {
  s_test_id = test_id;
  s_title = title;
  pinMode(PIN_LED_HEARTBEAT, OUTPUT);
  digitalWrite(PIN_LED_HEARTBEAT, LOW);

  Serial.begin(BRI_SERIAL_BAUD);
  // Wait for the PC to open the port, but not forever (see BRI_SERIAL_WAIT_MS).
  uint32_t wait_start_ms = millis();
  while (!Serial && (millis() - wait_start_ms) < BRI_SERIAL_WAIT_MS) {
    // nothing to do but wait
  }
  briTelemBanner();
}

// Print the '#' lines that identify the test, firmware and settings, so
// a saved capture explains itself.
void briTelemBanner() {
  Serial.print("# BRI bench test ");
  Serial.print(s_test_id);
  Serial.print(": ");
  Serial.println(s_title);

  Serial.print("# fw=");
  Serial.print(BRI_FW_VERSION);
  Serial.print(" encoder_backend=");
  Serial.print(briEncoderBackendName());
  Serial.print(" duty_cap=");
  Serial.print(BRI_DUTY_CAP, 2);
  Serial.print(" overcurrent_A=");
  Serial.print(BRI_OVERCURRENT_A, 1);
  Serial.println();

  Serial.print("# boot_count=");
  Serial.print(briBootCount());
  Serial.print(" reset_reason=0x");
  Serial.print(briResetReasonRaw(), HEX);
  Serial.println();
}

// Print the CSV header and remember its field count, so every data row
// can be checked against it.
void briTelemHeader(const char* header) {
  // Count the fields: one more than the number of commas.
  uint16_t fields = 1;
  for (size_t i = 0; header[i] != 0; i++) {
    if (header[i] == ',') {
      fields = fields + 1;
    }
  }
  s_header_fields = fields;
  s_rowlen_reported = false;
  Serial.println(header);
}

// Start a data row: prints the time in ms (the first column).
void briRowBegin() {
  Serial.print(millis());
  s_row_fields = 1;
}

// Add one whole-number field to the row being built.
void briRowInt(long value) {
  Serial.print(',');
  Serial.print(value);
  s_row_fields = s_row_fields + 1;
}

// Add one decimal field to the row being built ('nan' if there is no value).
void briRowFloat(float value, uint8_t digits) {
  Serial.print(',');
  if (isnan(value)) {
    // "nan" is what the Python side understands as "no value".
    Serial.print("nan");
  } else {
    Serial.print(value, (int)digits);
  }
  s_row_fields = s_row_fields + 1;
}

// Finish the row.  Also checks the row had as many fields as the header
// and raises a '!ROWLEN' event (once) if not.
void briRowEnd() {
  Serial.println();
  if (s_row_fields != s_header_fields && !s_rowlen_reported) {
    s_rowlen_reported = true;
    briEventBegin("ROWLEN");
    briKVInt("row_fields", (long)s_row_fields);
    briKVInt("header_fields", (long)s_header_fields);
    briEventEnd();
  }
}

// Print 'key=1.23 ' inside an event or result line.
void briKVFloat(const char* key, float value, uint8_t digits) {
  Serial.print(key);
  Serial.print('=');
  if (isnan(value)) {
    Serial.print("nan");
  } else {
    Serial.print(value, (int)digits);
  }
  Serial.print(' ');
}

// Print 'key=12 ' inside an event or result line.
void briKVInt(const char* key, long value) {
  Serial.print(key);
  Serial.print('=');
  Serial.print(value);
  Serial.print(' ');
}

// Print 'key=0x68 ' inside an event or result line (for I2C addresses).
void briKVHex(const char* key, uint8_t value) {
  Serial.print(key);
  Serial.print("=0x");
  // Print two hex digits even for small numbers (0x08, not 0x8).
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
  Serial.print(' ');
}

// Print 'key=word ' inside an event or result line.
void briKVText(const char* key, const char* value) {
  Serial.print(key);
  Serial.print('=');
  Serial.print(value);
  Serial.print(' ');
}

// Print a word or phrase followed by a space inside an event or result line.
void briText(const char* text) {
  Serial.print(text);
  Serial.print(' ');
}

// Start an event line: '!NAME,<time ms>,'.
void briEventBegin(const char* name) {
  Serial.print('!');
  Serial.print(name);
  Serial.print(',');
  Serial.print(millis());
  Serial.print(',');
}

// Finish an event line.
void briEventEnd() {
  Serial.println();
}

// Print a whole event whose detail is a single word (for example a stop reason).
void briEvent(const char* name, const char* detail) {
  briEventBegin(name);
  Serial.print(detail);
  briEventEnd();
}

// Zero the pass/fail tally at the start of a run.
void briResultsReset() {
  s_pass_count = 0;
  s_fail_count = 0;
  s_info_count = 0;
  s_todo_count = 0;
}

// Start a RESULT line with its verdict, and count it in the tally.
void briResultBegin(BriVerdict verdict) {
  Serial.print("RESULT,");
  Serial.print(s_test_id);
  Serial.print(',');
  if (verdict == BRI_PASS) {
    Serial.print("PASS");
    s_pass_count = s_pass_count + 1;
  } else if (verdict == BRI_FAIL) {
    Serial.print("FAIL");
    s_fail_count = s_fail_count + 1;
  } else if (verdict == BRI_TODO) {
    Serial.print("INFO");
    s_todo_count = s_todo_count + 1;
  } else {
    Serial.print("INFO");
    s_info_count = s_info_count + 1;
  }
  Serial.print(',');
}

// Finish a RESULT line.
void briResultEnd() {
  Serial.println();
}

// Print the final RESULT line of a run, combining all the verdicts so far:
// any FAIL gives FAIL; anything left for the operator (or nothing checked)
// gives INFO; otherwise PASS.
void briResultOverall() {
  Serial.print("RESULT,");
  Serial.print(s_test_id);
  Serial.print(',');
  if (s_fail_count > 0) {
    Serial.print("FAIL");
  } else if (s_todo_count > 0 || s_pass_count == 0) {
    // Nothing failed, but either the operator still owes a confirmation or
    // nothing was actually checked.  That is not a pass.
    Serial.print("INFO");
  } else {
    Serial.print("PASS");
  }
  Serial.print(",overall ");
  briKVInt("pass", (long)s_pass_count);
  briKVInt("fail", (long)s_fail_count);
  briKVInt("operator_todo", (long)s_todo_count);
  briKVInt("info", (long)s_info_count);
  Serial.println();
}

// Gather typed characters into a line without ever waiting.  Returns true
// only when a complete line has arrived.
bool briReadCommand(char* out, size_t out_size) {
  while (Serial.available() > 0) {
    int incoming = Serial.read();
    if (incoming < 0) {
      break;
    }
    char c = (char)incoming;
    if (c == '\r') {
      continue;  // Windows line endings send \r\n; the \n ends the line
    }
    if (c == '\n') {
      s_line[s_line_length] = 0;
      s_line_length = 0;
      if (s_line[0] == 0) {
        continue;  // empty line: ignore
      }
      strncpy(out, s_line, out_size - 1);
      out[out_size - 1] = 0;
      return true;
    }
    // Keep the character if there is room; extra characters are dropped so a
    // burst of garbage cannot overflow the buffer.
    if (s_line_length < (uint8_t)(BRI_CMD_MAX - 1)) {
      s_line[s_line_length] = c;
      s_line_length = s_line_length + 1;
    }
  }
  return false;
}

// Pull the number out of a command such as 't10' or 'v 5.02'.
float briCommandNumber(const char* command, float fallback) {
  // Skip the command letter, then any spaces or '=' the operator typed.
  const char* text = command;
  if (*text != 0) {
    text = text + 1;
  }
  while (*text == ' ' || *text == '=') {
    text = text + 1;
  }
  char* end = nullptr;
  double value = strtod(text, &end);
  if (end == text) {
    return fallback;  // no digits were found
  }
  return (float)value;
}

// Print the help lines that are the same in every sketch (g, s, ?).
void briPrintCommonHelp() {
  Serial.println("#   g   go: start the run");
  Serial.println("#   s   stop: end the run now, brake and sleep all motor drivers");
  Serial.println("#   ?   show this help");
}

// Blink the LED: slow when idle, fast during a run.  Shows the loop is alive.
void briHeartbeat(bool running) {
  uint32_t period_ms = 500;  // idle: one blink per second
  if (running) {
    period_ms = 100;         // running: five blinks per second
  }
  if (briEvery(&s_led_last_ms, period_ms)) {
    s_led_on = !s_led_on;
    if (s_led_on) {
      digitalWrite(PIN_LED_HEARTBEAT, HIGH);
    } else {
      digitalWrite(PIN_LED_HEARTBEAT, LOW);
    }
  }
}

// 'Has this much time passed since last time?' - the building block for
// doing things periodically without ever calling delay().
bool briEvery(uint32_t* last_ms, uint32_t period_ms) {
  uint32_t now_ms = millis();
  // Unsigned subtraction stays correct even when millis() wraps after 49 days.
  if ((now_ms - *last_ms) >= period_ms) {
    *last_ms = now_ms;
    return true;
  }
  return false;
}
