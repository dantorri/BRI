// sim_bri_hw.cpp - a simple model of the BRI electronics for PC runs.
//
// THIS IS A MODEL, NOT THE ROBOT.  It is deliberately simple: good enough to
// make every code path in the firmware produce sensible-looking numbers, so
// the telemetry format and the pass/fail logic can be exercised without
// hardware.  It proves nothing about the real motors, drivers or IMU.
//
// What is modelled:
//   * the two enable nets (MOTOR_EN / HIP_EN) with kill switch and read-back
//   * four G2 drivers: sleep, direction, PWM duty, FLT, CS (50 mV + 20 mV/A,
//     present only during the PWM on-time)
//   * four gearmotors as "voltage in, speed out" with friction, using the
//     datasheet figures (76 rpm and 0.2 A no-load, 5.5 A stall at 12 V)
//   * four encoders: 8400 counts per output revolution, A/B pin levels
//   * the battery divider on VBAT_SENSE
//   * one MPU-family IMU on I2C that follows a scripted tilt
//
// It uses the same BRI_Pins.h as the firmware, so the model's wiring is the
// pin map's wiring.
#include <Arduino.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "BRI_Pins.h"
#include "sim.h"

// ---- motor model constants (derived from the Pololu 4756 datasheet numbers) ----
static const double MOTOR_R_OHM = 12.0 / 5.5;           // stall: 12 V gives 5.5 A
static const double MOTOR_FRICTION_A = 0.2;             // no-load current = current needed to beat friction
static const double MOTOR_K_V_PER_RPM = (12.0 - MOTOR_FRICTION_A * MOTOR_R_OHM) / 76.0;  // back-EMF at 76 rpm
static const double MOTOR_TAU_S = 0.08;                 // how quickly speed follows voltage (a guess)
static const double MOTOR_ACCEL = MOTOR_R_OHM / (MOTOR_K_V_PER_RPM * MOTOR_TAU_S);  // rpm per (A*s)
static const double COUNTS_PER_REV = 8400.0;
static const double SIM_PI = 3.14159265358979323846;

struct MotorSim {
  double speed_rpm;     // output shaft speed
  double angle_rev;     // output shaft angle in revolutions
  double current_a;     // winding current
  double hand_rpm;      // extra speed while a "hand" turns the wheel
  double hand_until_s;  // simulated time at which the hand lets go
  double extra_a;       // extra current added to the CS reading (imitates a fault)
  bool stuck;           // shaft locked
  bool disconnected;    // motor leads unplugged (no current, no motion)
  bool fault;           // driver pulls FLT low
  bool encoder_flipped; // encoder A/B swapped: it counts the opposite way
};

static MotorSim s_motor[4];
static bool s_kill_switch = false;      // true = switch in STOP (both poles)
static double s_vbat = 11.7;            // battery voltage
static uint32_t s_random_state = 12345; // fixed seed: every run gives the same numbers

// IMU model
static uint8_t s_imu_address = 0x68;
static uint8_t s_imu_who = 0x70;        // MPU-6500
static uint8_t s_imu_registers[128];
static uint8_t s_imu_data[14];          // snapshot taken when register 0x3B is read
static double s_tilt_amplitude_deg = 0.0;
static double s_tilt_period_s = 4.0;
static double s_tilt_start_s = 1e9;
static const double IMU_PITCH_OFFSET_DEG = 1.5;   // pretend the board is mounted slightly tilted
static const double IMU_GYRO_BIAS_DPS[3] = { -0.4, 0.8, 0.3 };

// Bookkeeping for the safety summary printed at the end of a run.
static bool s_go_seen = false;
static bool s_drive_before_go = false;
static double s_max_abs_duty = 0.0;
static double s_last_drive_s = -1.0;
static bool s_ever_en_wheels = false;   // MOTOR_EN was commanded high at some point
static bool s_ever_en_hips = false;     // HIP_EN was commanded high at some point

// A repeatable random number between -1 and +1 (simple linear congruential generator).
static double randomSigned() {
  s_random_state = s_random_state * 1664525u + 1013904223u;
  double unit = (double)(s_random_state >> 8) / (double)(1u << 24);
  return unit * 2.0 - 1.0;
}

// Which enable net a channel is on: 0 = wheels (channels 0, 1), 1 = hips.
static uint8_t groupOfChannel(int channel) {
  return channel <= 1 ? 0 : 1;
}

// Actual level on an enable net: high only if the Teensy drives it high AND
// the kill switch is not shorting it to ground.
static bool enableNetHigh(uint8_t group) {
  uint8_t pin = (group == 0) ? PIN_MOTOR_EN : PIN_HIP_EN;
  bool driven_high = (g_sim_pin_mode[pin] == OUTPUT) && (g_sim_pin_out[pin] == 1);
  return driven_high && !s_kill_switch;
}

// Encoder count of one channel, worked out from the shaft angle.
static int32_t countOfChannel(int channel) {
  int32_t count = (int32_t)floor(s_motor[channel].angle_rev * COUNTS_PER_REV);
  return s_motor[channel].encoder_flipped ? -count : count;
}

// Put the model in its power-on state (motors at rest, IMU asleep).
void simHwInit() {
  memset(s_motor, 0, sizeof(s_motor));
  memset(s_imu_registers, 0, sizeof(s_imu_registers));
  s_imu_registers[0x6B] = 0x40;  // powers up asleep: the firmware must wake it
}

// Remember that the scripted operator has sent 'g'.  From then on a
// woken driver is expected; before it, it is a safety violation.
void simHwNoteGoCommand() {
  s_go_seen = true;
}

// Advance the model by dt seconds: for each motor, work out voltage,
// current, speed and angle from what the firmware's pins are commanding.
void simHwStep(double dt) {
  double now_s = simNowSeconds();
  bool any_drive = false;
  for (int channel = 0; channel < 4; channel++) {
    MotorSim& motor = s_motor[channel];
    bool awake = enableNetHigh(groupOfChannel(channel));
    double duty = g_sim_pwm_duty[BRI_PIN_PWM[channel]];
    double direction = g_sim_pin_out[BRI_PIN_DIR[channel]] ? 1.0 : -1.0;

    // Safety bookkeeping uses what the Teensy COMMANDS, regardless of the kill switch.
    bool commanded_awake = g_sim_pin_out[groupOfChannel(channel) == 0 ? PIN_MOTOR_EN : PIN_HIP_EN] == 1;
    if (commanded_awake || duty > 0.0) {
      any_drive = true;
    }
    if (duty > s_max_abs_duty) {
      s_max_abs_duty = duty;
    }

    double volts = awake ? duty * direction * s_vbat : 0.0;
    double speed = motor.speed_rpm;
    double current = 0.0;
    if (motor.disconnected) {
      speed = 0.0;
      current = 0.0;
    } else if (motor.stuck) {
      speed = 0.0;
      current = awake ? volts / MOTOR_R_OHM : 0.0;
    } else if (awake) {
      // Driver awake: the winding sees `volts` (0 V when PWM is low = brake).
      current = (volts - MOTOR_K_V_PER_RPM * speed) / MOTOR_R_OHM;
      if (fabs(speed) < 1e-3 && fabs(current) <= MOTOR_FRICTION_A) {
        speed = 0.0;  // not enough torque to break away
      } else {
        double friction = MOTOR_FRICTION_A;
        if (fabs(speed) >= 1e-3) {
          friction = speed > 0.0 ? MOTOR_FRICTION_A : -MOTOR_FRICTION_A;
        } else {
          friction = current > 0.0 ? MOTOR_FRICTION_A : -MOTOR_FRICTION_A;
        }
        double new_speed = speed + MOTOR_ACCEL * (current - friction) * dt;
        if (speed != 0.0 && new_speed * speed < 0.0) {
          new_speed = 0.0;  // stop at zero rather than overshoot through it
        }
        speed = new_speed;
      }
    } else {
      // Driver asleep: outputs open, motor coasts down against friction.
      double slow_down = MOTOR_ACCEL * MOTOR_FRICTION_A * dt;
      if (fabs(speed) <= slow_down) {
        speed = 0.0;
      } else {
        speed -= (speed > 0.0 ? slow_down : -slow_down);
      }
    }
    motor.speed_rpm = speed;
    motor.current_a = current;

    double total_rpm = speed;
    if (now_s < motor.hand_until_s) {
      total_rpm += motor.hand_rpm;
    }
    motor.angle_rev += total_rpm / 60.0 * dt;
  }
  if (g_sim_pin_out[PIN_MOTOR_EN] == 1) {
    s_ever_en_wheels = true;
  }
  if (g_sim_pin_out[PIN_HIP_EN] == 1) {
    s_ever_en_hips = true;
  }
  if (any_drive) {
    s_last_drive_s = now_s;
    if (!s_go_seen) {
      s_drive_before_go = true;
    }
  }
}

// Level the model puts on an input pin: enable read-back, fault pins,
// encoder A/B.
int simHwDigitalIn(uint8_t pin) {
  if (pin == PIN_MOTOR_EN_SENSE) {
    return enableNetHigh(0) ? 1 : 0;
  }
  for (int channel = 0; channel < 4; channel++) {
    if (pin == BRI_PIN_FLT[channel]) {
      return s_motor[channel].fault ? 0 : 1;  // open drain with pull-up: high unless faulted
    }
    // Quadrature: the count's two lowest bits give the A/B state.
    //   0: A=0 B=0   1: A=1 B=0   2: A=1 B=1   3: A=0 B=1
    int phase = (int)(countOfChannel(channel) & 3);
    if (pin == BRI_PIN_ENC_A[channel]) {
      return (phase == 1 || phase == 2) ? 1 : 0;
    }
    if (pin == BRI_PIN_ENC_B[channel]) {
      return (phase == 2 || phase == 3) ? 1 : 0;
    }
  }
  return g_sim_pin_mode[pin] == INPUT_PULLUP ? 1 : 0;
}

// Voltage the model puts on an analog pin: battery divider, current sense.
double simHwAnalogVolts(uint8_t pin) {
  if (pin == PIN_VBAT_SENSE) {
    return s_vbat / 4.3 + 0.002 * randomSigned();
  }
  for (int channel = 0; channel < 4; channel++) {
    if (pin != BRI_PIN_CS[channel]) {
      continue;
    }
    bool awake = enableNetHigh(groupOfChannel(channel));
    double duty = g_sim_pwm_duty[BRI_PIN_PWM[channel]];
    if (!awake || duty <= 0.0) {
      return 0.0;  // CS shows nothing unless the bridge is driving
    }
    // Each ADC reading lands at a random point in the PWM cycle; it sees the
    // current only if that point is inside the on-time.
    double where_in_cycle = (randomSigned() + 1.0) / 2.0;
    if (where_in_cycle >= duty) {
      return 0.0;
    }
    double sensed = fabs(s_motor[channel].current_a) + s_motor[channel].extra_a;
    return 0.050 + 0.020 * sensed + 0.001 * randomSigned();
  }
  return 0.0;
}

// Count of the encoder whose A channel is on the given pin (used by the
// Encoder and QuadEncoder stand-ins).
int32_t simHwEncoderCount(uint8_t pin_a) {
  for (int channel = 0; channel < 4; channel++) {
    if (pin_a == BRI_PIN_ENC_A[channel]) {
      return countOfChannel(channel);
    }
  }
  simAbort("encoder created on a pin that is not an encoder A pin in the pin map");
  return 0;
}

// ---- IMU ------------------------------------------------------------------------
bool simHwI2cPresent(uint8_t address) {
  return address == s_imu_address;
}

// Store a signed 16-bit value high byte first, as the chip does.
static void putBigEndian(uint8_t* where, double value) {
  if (value > 32767.0) {
    value = 32767.0;
  }
  if (value < -32768.0) {
    value = -32768.0;
  }
  int16_t raw = (int16_t)lround(value);
  where[0] = (uint8_t)(((uint16_t)raw) >> 8);
  where[1] = (uint8_t)(((uint16_t)raw) & 0xFF);
}

// Work out what the IMU would measure right now and store it as 14 raw bytes.
static void latchImuData() {
  bool asleep = (s_imu_registers[0x6B] & 0x40) != 0;
  if (asleep) {
    memset(s_imu_data, 0, sizeof(s_imu_data));
    return;
  }
  double now_s = simNowSeconds();
  double pitch_deg = IMU_PITCH_OFFSET_DEG;
  double rate_dps = 0.0;
  if (now_s >= s_tilt_start_s) {
    double angle = 2.0 * SIM_PI * (now_s - s_tilt_start_s) / s_tilt_period_s;
    pitch_deg += s_tilt_amplitude_deg * sin(angle);
    rate_dps = s_tilt_amplitude_deg * (2.0 * SIM_PI / s_tilt_period_s) * cos(angle);
  }
  double pitch_rad = pitch_deg * SIM_PI / 180.0;
  // Sensor mounted X forward, Y left, Z up.  Leaning forward makes X read negative.
  double accel_g[3] = { -sin(pitch_rad), 0.01, cos(pitch_rad) };
  double gyro_dps[3] = { 0.0, rate_dps, 0.0 };

  // The scale depends on the range the firmware selected (bits 4:3).
  int accel_range = (s_imu_registers[0x1C] >> 3) & 3;
  int gyro_range = (s_imu_registers[0x1B] >> 3) & 3;
  double accel_lsb_per_g = 16384.0 / (double)(1 << accel_range);
  double gyro_lsb_per_dps = 131.0 / (double)(1 << gyro_range);
  for (int axis = 0; axis < 3; axis++) {
    double accel = accel_g[axis] + 0.003 * randomSigned();
    double gyro = gyro_dps[axis] + IMU_GYRO_BIAS_DPS[axis] + 0.05 * randomSigned();
    putBigEndian(&s_imu_data[axis * 2], accel * accel_lsb_per_g);
    putBigEndian(&s_imu_data[8 + axis * 2], gyro * gyro_lsb_per_dps);
  }
  putBigEndian(&s_imu_data[6], (27.0 - 21.0) * 333.87);  // 27 C on the MPU-6500 scale
}

// What the simulated IMU returns for one register read.
uint8_t simHwI2cReadRegister(uint8_t address, uint8_t reg) {
  (void)address;
  if (reg == 0x75) {
    return s_imu_who;
  }
  if (reg == 0x3B) {
    latchImuData();  // reading the first data register freezes one consistent sample
  }
  if (reg >= 0x3B && reg < 0x3B + 14) {
    return s_imu_data[reg - 0x3B];
  }
  if (reg < sizeof(s_imu_registers)) {
    return s_imu_registers[reg];
  }
  return 0;
}

// What the simulated IMU does with one register write (including reset).
void simHwI2cWriteRegister(uint8_t address, uint8_t reg, uint8_t value) {
  (void)address;
  if (reg >= sizeof(s_imu_registers)) {
    return;
  }
  if (reg == 0x6B && (value & 0x80)) {
    // Reset bit: all settings back to default, and the chip is asleep again.
    memset(s_imu_registers, 0, sizeof(s_imu_registers));
    s_imu_registers[0x6B] = 0x40;
    return;
  }
  s_imu_registers[reg] = value;
}

// ---- script directives -------------------------------------------------------------
void simHwDirective(const char* text) {
  int channel = 0;
  int flag = 0;
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  unsigned int hex_a = 0;
  unsigned int hex_b = 0;
  if (sscanf(text, "turn %d %lf %lf", &channel, &a, &b) == 3 && channel >= 0 && channel < 4) {
    // Turn the wheel by hand: `a` revolutions spread evenly over `b` milliseconds.
    s_motor[channel].hand_rpm = a / (b / 1000.0) * 60.0;
    s_motor[channel].hand_until_s = simNowSeconds() + b / 1000.0;
  } else if (sscanf(text, "usb %d", &flag) == 1) {
    g_sim_usb_connected = (flag != 0);
  } else if (sscanf(text, "kill %d", &flag) == 1) {
    s_kill_switch = (flag != 0);
  } else if (sscanf(text, "fault %d %d", &channel, &flag) == 2 && channel >= 0 && channel < 4) {
    s_motor[channel].fault = (flag != 0);
  } else if (sscanf(text, "extra %d %lf", &channel, &a) == 2 && channel >= 0 && channel < 4) {
    s_motor[channel].extra_a = a;
  } else if (sscanf(text, "stuck %d %d", &channel, &flag) == 2 && channel >= 0 && channel < 4) {
    s_motor[channel].stuck = (flag != 0);
  } else if (sscanf(text, "disconnect %d %d", &channel, &flag) == 2 && channel >= 0 && channel < 4) {
    s_motor[channel].disconnected = (flag != 0);
  } else if (sscanf(text, "encflip %d %d", &channel, &flag) == 2 && channel >= 0 && channel < 4) {
    s_motor[channel].encoder_flipped = (flag != 0);
  } else if (sscanf(text, "tilt %lf %lf %lf", &a, &b, &c) == 3) {
    // Rock the IMU: amplitude `a` degrees, period `b` ms, starting at time `c` ms.
    s_tilt_amplitude_deg = a;
    s_tilt_period_s = b / 1000.0;
    s_tilt_start_s = c / 1000.0;
  } else if (sscanf(text, "imu %x %x", &hex_a, &hex_b) == 2) {
    s_imu_address = (uint8_t)hex_a;
    s_imu_who = (uint8_t)hex_b;
  } else if (sscanf(text, "vbat %lf", &a) == 1) {
    s_vbat = a;
  } else {
    fprintf(stderr, "SIM: unknown directive '@%s'\n", text);
    simAbort("unknown script directive");
  }
}

// Print one 'SIM,...' line at the end of a run with the facts the safety
// checks need: largest duty, anything driven before go, final pin states.
void simHwPrintSummary() {
  bool pwm_active = false;
  for (int channel = 0; channel < 4; channel++) {
    if (g_sim_pwm_duty[BRI_PIN_PWM[channel]] > 0.0) {
      pwm_active = true;
    }
  }
  fprintf(stderr,
          "SIM,max_abs_duty=%.4f,drive_before_go=%d,final_en_wheels=%d,final_en_hips=%d,"
          "final_pwm_active=%d,ever_en_wheels=%d,ever_en_hips=%d,last_drive_ms=%.1f,end_ms=%.1f,"
          "counts=%ld/%ld/%ld/%ld\n",
          s_max_abs_duty, s_drive_before_go ? 1 : 0, (int)g_sim_pin_out[PIN_MOTOR_EN],
          (int)g_sim_pin_out[PIN_HIP_EN], pwm_active ? 1 : 0, s_ever_en_wheels ? 1 : 0,
          s_ever_en_hips ? 1 : 0, s_last_drive_s * 1000.0,
          simNowSeconds() * 1000.0, (long)countOfChannel(0), (long)countOfChannel(1),
          (long)countOfChannel(2), (long)countOfChannel(3));
}
