// App 10 - First powered reducer and output-encoder inspection.
//
// This app commands MOTOR STEPS directly. It intentionally does not use
// GEAR_RATIO or output-angle limits: before calibration, those values are not
// trustworthy. The AS5600 only observes and logs the output motion.
//
// pio run -e reducer_bench -t upload
// python scripts/serial_logger.py --port COM3 --name p1b_reducer_bench
//
// Commands:
//   e           enable/disable the driver
//   + / -       jog +/- 160 motor steps (0.1 motor revolution at 8x)
//   f / r       jog +/- 1600 motor steps (one motor revolution at 8x)
//   j <steps>   jog a signed number of motor steps, limited to +/-1600
//   x           stop and disable
//   z           zero the encoder display here; does not alter motor position
//   ?           report magnet state and commanded input turns
//
// Start with hand turning and encoder-only checks. With the motor enabled,
// make one 160-step jog, inspect the assembly, then progress to a full motor
// revolution only if there is no binding, rapid heat, or encoder fault.

#include <Arduino.h>
#include <Wire.h>

#include "AS5600Encoder.h"
#include "SerialCli.h"
#include "StepperDriver.h"
#include "TCA9548A.h"
#include "joint_config.h"
#include "pins.h"

static TCA9548A mux(TCA9548A_ADDR);
static AS5600Encoder encoder(&mux, MUX_CH_J1);
static StepperDriver motor(PIN_J1_STEP, PIN_J1_DIR, PIN_J1_EN,
                           DRIVER_EN_ACTIVE_LOW);
static SerialCli<24> cli;

static constexpr long kSmallJogSteps = 160;
static constexpr long kFullMotorRevSteps = (long)STEPS_PER_MOTOR_REV;
static constexpr float kBenchSpeedStepsPerSec = 200.0f;
static constexpr float kBenchAccelStepsPerSec2 = 400.0f;
static constexpr uint16_t kEncoderFaultCount = 5;

static bool enabled = false;
static bool faulted = false;
static uint32_t lastLogMs = 0;
static uint32_t lastStatusMs = 0;

static void stopAndDisable(const __FlashStringHelper *reason) {
  motor.emergencyStop();
  motor.enable(false);
  enabled = false;
  Serial.print(F("# STOP: "));
  Serial.println(reason);
}

static void printStatus() {
  encoder.readStatus();
  Serial.print(F("# en="));
  Serial.print(enabled);
  Serial.print(F(" fault="));
  Serial.print(faulted);
  Serial.print(F(" cmd_steps="));
  Serial.print(motor.position());
  Serial.print(F(" input_rev="));
  Serial.print(motor.position() / STEPS_PER_MOTOR_REV, 3);
  Serial.print(F(" enc_deg="));
  Serial.print(encoder.angleDeg(), 3);
  Serial.print(F(" magnet="));
  Serial.print(encoder.magnetText());
  Serial.print(F(" agc="));
  Serial.print(encoder.agc());
  Serial.print(F(" i2c_err="));
  Serial.println(encoder.errorCount());
}

static void jog(long steps) {
  if (!enabled || faulted) {
    Serial.println(F("# REFUSED: enable first and clear encoder faults by reset"));
    return;
  }
  if (motor.isMoving()) {
    Serial.println(F("# REFUSED: wait for the current move"));
    return;
  }
  if (steps > kFullMotorRevSteps) steps = kFullMotorRevSteps;
  if (steps < -kFullMotorRevSteps) steps = -kFullMotorRevSteps;
  motor.move(steps);
  Serial.print(F("# jog_motor_steps="));
  Serial.println(steps);
}

static void handleCommand(const char *line) {
  switch (line[0]) {
    case 'e':
      if (faulted || !encoder.magnetDetected()) {
        Serial.println(F("# REFUSED: encoder magnet is not valid"));
        return;
      }
      enabled = !enabled;
      motor.enable(enabled);
      Serial.println(enabled ? F("# ENABLED") : F("# disabled"));
      break;
    case '+': jog(kSmallJogSteps); break;
    case '-': jog(-kSmallJogSteps); break;
    case 'f': jog(kFullMotorRevSteps); break;
    case 'r': jog(-kFullMotorRevSteps); break;
    case 'j': jog(strtol(line + 1, nullptr, 10)); break;
    case 'x': stopAndDisable(F("operator command")); break;
    case 'z':
      encoder.zeroHere();
      Serial.println(F("# encoder display zeroed"));
      break;
    case '?': printStatus(); break;
    default:
      Serial.println(F("# e +/- f r j<steps> x z ?"));
      break;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
  }
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println(F("\n=== app_10_reducer_bench ==="));
  if (!mux.begin() || !mux.isPresent() || !encoder.begin()) {
    faulted = true;
    Serial.println(F("# FAULT: encoder unavailable; run app_00_i2c_scan"));
  }
  encoder.setDirection(ENCODER_DIRECTION);
  encoder.setFilterAlpha(0.4f);
  encoder.readStatus();
  if (!encoder.magnetDetected()) {
    faulted = true;
    Serial.println(F("# FAULT: invalid magnet; run app_01_encoder_test"));
  }

  motor.begin();
  motor.setMaxSpeed(kBenchSpeedStepsPerSec);
  motor.setAcceleration(kBenchAccelStepsPerSec2);
  motor.setPosition(0);
  motor.enable(false);

  Serial.println(F("# direct motor-step mode; GEAR_RATIO is intentionally unused"));
  Serial.println(F("# driver disabled; commands: e +/- f r j<steps> x z ?"));
  Serial.println(F("ms,cmd_steps,input_rev,enc_deg,enc_turns,magnet_ok,i2c_err"));
}

void loop() {
  const char *line = cli.poll();
  if (line && line[0]) handleCommand(line);

  encoder.read();
  motor.run();
  const uint32_t now = millis();

  if (now - lastStatusMs >= 500) {
    lastStatusMs = now;
    encoder.readStatus();
  }
  if (enabled && (!encoder.magnetDetected() || encoder.errorCount() > kEncoderFaultCount)) {
    faulted = true;
    stopAndDisable(F("encoder lost or I2C errors exceeded limit"));
  }

  if (now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;
    Serial.print(now);
    Serial.print(',');
    Serial.print(motor.position());
    Serial.print(',');
    Serial.print(motor.position() / STEPS_PER_MOTOR_REV, 4);
    Serial.print(',');
    Serial.print(encoder.angleDeg(), 3);
    Serial.print(',');
    Serial.print(encoder.turns());
    Serial.print(',');
    Serial.print(encoder.magnetOk() ? 1 : 0);
    Serial.print(',');
    Serial.println(encoder.errorCount());
  }
}