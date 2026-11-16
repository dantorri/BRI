// BRI_Telemetry.h - the text protocol between the Teensy and the PC logger.
//
// Every line the firmware prints is one of five kinds, told apart by how it
// starts.  Because the kinds are unambiguous, the same output can be read by
// a person in the Serial Monitor and parsed by a Python script.
//
//   # text                      for humans (banner, help, notes)
//   t_ms,col2,col3,...          the CSV header: names of the data columns
//   1234,0.5,7,...              a data row: same number of fields as the header
//   !NAME,t_ms,detail           an event or fault (STOP, DMM entry, mark, ...)
//   RESULT,CV-00N,VERDICT,text  a verdict; VERDICT is PASS, FAIL or INFO
//
// "detail" and "text" are written as key=value pairs separated by spaces and
// never contain a comma, so a line can always be split on commas.
//
// Commands from the PC are short lines ending in newline, for example "g",
// "s", "c1", "t10", "v5.02".  "?" prints the help of the running sketch.
#ifndef BRI_TELEMETRY_H
#define BRI_TELEMETRY_H

#include <Arduino.h>

// The verdict attached to a RESULT line.
//   BRI_PASS  the firmware checked something and it was within limits
//   BRI_FAIL  the firmware checked something and it was outside limits
//   BRI_INFO  a value reported for the record; no judgement made
//   BRI_TODO  printed as INFO, but means "the operator still has to confirm
//             something", so the overall verdict cannot be PASS yet
enum BriVerdict { BRI_PASS = 0, BRI_FAIL = 1, BRI_INFO = 2, BRI_TODO = 3 };

// Open USB serial, wait (briefly) for the PC, remember the test id and title,
// print the banner.  Called by briBegin().
void briTelemBegin(const char* test_id, const char* title);

// Print the banner: which test this is, firmware version, encoder backend,
// duty cap, boot count.  Printed at boot and again at the start of each run
// so that every capture is self-describing.
void briTelemBanner();

// Print the CSV header line and remember how many fields it has.
// `header` must start with "t_ms,".
void briTelemHeader(const char* header);

// ---- Building a data row -------------------------------------------------
// A row is built field by field:
//     briRowBegin();            prints the time (the t_ms column)
//     briRowInt(count);         prints ",<integer>"
//     briRowFloat(volts, 3);    prints ",<number with 3 decimals>"
//     briRowEnd();              ends the line
// Why not one printf: each field is on its own line of code next to the
// header, so it is easy to see that header and row have the same columns.
// briRowEnd() also counts the fields and raises a "!ROWLEN" event if the row
// does not match the header, which catches exactly that mistake.
void briRowBegin();
// Add one whole-number field to the row.
void briRowInt(long value);
// Add one decimal field with the given number of decimals ('nan' if no value).
void briRowFloat(float value, uint8_t digits);
// End the row, and check it had as many fields as the header (see above).
void briRowEnd();

// ---- Key=value output used inside events and results --------------------------
// These print one piece of the detail text.  Why pieces: the Teensy has no
// convenient way to format a whole sentence with numbers in it, and building
// the line piece by piece keeps each number next to its name in the code.
//
// Print a decimal number with its name: "key=1.23 ".
void briKVFloat(const char* key, float value, uint8_t digits);
// Same for a whole number: prints "key=12 ".
void briKVInt(const char* key, long value);
// Same for a byte shown in hex (I2C addresses): prints "key=0x68 ".
void briKVHex(const char* key, uint8_t value);
// Same for a word: prints "key=value ".  Why: lets a result carry a name
// such as chip=MPU-6500 or stopped=timeout in the same key=value form.
void briKVText(const char* key, const char* value);
// Print a plain word or phrase followed by a space.  Why: some details
// are a label or a hint for the reader rather than a key=value pair.
void briText(const char* text);

// ---- Events ("!" lines) ----------------------------------------------------------
// briEventBegin("DMM") prints "!DMM,<t_ms>," ; add key=value pieces; then
// briEventEnd() finishes the line.
void briEventBegin(const char* name);
// Finish the event line started by briEventBegin().
void briEventEnd();
// Shortcut for an event whose detail is a single word.
void briEvent(const char* name, const char* detail);

// ---- Results ------------------------------------------------------------------------
// briResultsReset() clears the pass/fail tally at the start of a run.
// briResultBegin(verdict) prints "RESULT,<id>,<VERDICT>," ; add key=value
// pieces; briResultEnd() finishes the line and updates the tally.
// briResultOverall() prints the last line of a run: FAIL if anything failed,
// INFO if nothing failed but something is still to be confirmed (or nothing
// was checked), PASS otherwise.
void briResultsReset();
// Start a RESULT line with the given verdict (and count it in the tally).
void briResultBegin(BriVerdict verdict);
// Finish the RESULT line started by briResultBegin().
void briResultEnd();
// Print the last line of a run, 'RESULT,<id>,<verdict>,overall ...'.
// Why: the PC and the reader get one line that sums up every check above it.
void briResultOverall();

// ---- Commands ----------------------------------------------------------------------
// Collect characters from the PC without waiting.  Returns true when a whole
// line has arrived, with the line (no line ending) copied into `out`.
// Why non-blocking: the loop must keep running the safety checks while the
// operator is half-way through typing.
bool briReadCommand(char* out, size_t out_size);

// The number that follows the command letter, e.g. "t10" -> 10, "v 5.02" ->
// 5.02, "d=-0.15" -> -0.15.  Returns `fallback` if there is no number.
float briCommandNumber(const char* command, float fallback);

// Print the help lines every sketch shares (g, s, ?).
void briPrintCommonHelp();

// Blink the on-board LED: slow when idle, fast while a run is in progress.
// Why: it shows at a glance that the loop is alive and what state it is in.
void briHeartbeat(bool running);

// Simple timer helper.  Returns true once every `period_ms`, and updates
// `*last_ms` when it does.
bool briEvery(uint32_t* last_ms, uint32_t period_ms);

#endif  // BRI_TELEMETRY_H
