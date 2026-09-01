// App 09 - Pull-out / step-rate characterization (one motor, no reducer)
//
//   pio run -e pullout -t upload
//   python scripts/serial_logger.py --port COM3 --name pullout
//
// WHY THIS IS NOT app_02 OR app_03
// --------------------------------
// A pull-out test asks "at what step rate does the motor stop keeping up with
// the commanded steps under load." On an Uno there are TWO different ways to
// stop keeping up and they look identical in a position log:
//
//   1. the MOTOR skipped        - the real measurement
//   2. the MCU never issued the step - loop() could not run fast enough
//
// Confusing them produces a torque-speed curve that is really a plot of the
// ATmega328P's loop rate, which is a very convincing way to be wrong. So this
// app reports the ACTUAL issued step rate alongside the commanded one. If
// act_sps < cmd_sps you have hit the controller, not the motor, and that row
// is not a data point - it is a different measurement (and a D13 data point
// about when to leave the Uno).
//
// `l` toggles encoder streaming. An AS5600 read is ~158 us at 400 kHz, which
// is the single biggest consumer of the loop budget, so the highest step rates
// are only reachable with it off. With encoder logging off, detect lost steps
// mechanically: command a whole number of revolutions and check that an index
// mark on the shaft returns to the same place.
//
// CSV columns: ms,cmd_sps,act_sps,cmd_steps,enc_deg,slip_deg,i2c_err
//
// Serial commands:
//   <number>  set step rate, steps/s      c   run forward
//   a<number> set acceleration, steps/s^2 v   run reverse
//   e         toggle driver enable        x   stop and print the run summary
//   z         zero encoder + commanded    l   toggle encoder streaming
//   ?         status
//
// SAFETY: there are no soft limits here - it is a bare shaft, deliberately.
// Never leave a suspended mass unattended, and keep the spool clear of cables.

#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>

#include "AS5600Encoder.h"
#include "SerialCli.h"
#include "StepperDriver.h"
#include "TCA9548A.h"
#include "joint_config.h"
#include "pins.h"

// Deliberately above MAX_SPEED_STEPS_PER_SEC: finding the ceiling is the job.
static constexpr float PULLOUT_MAX_SPS = 20000.0f;

static TCA9548A mux(TCA9548A_ADDR);
static AS5600Encoder encoder(&mux, MUX_CH_J1);
static StepperDriver motor(PIN_J1_STEP, PIN_J1_DIR, PIN_J1_EN,
                           DRIVER_EN_ACTIVE_LOW);
static SerialCli<16> cli;

static float cmdSps = 400.0f;
static float accelSps2 = ACCEL_STEPS_PER_SEC2;
static int8_t runDir = 0;
static bool encoderOn = true;
static bool encoderPresent = false;

static uint32_t lastLogMs = 0;
static long lastLogPos = 0;
static uint32_t runStartMs = 0;
static long runStartPos = 0;
static float peakSlipDeg = 0.0f;

static float cmdDeg() { return motor.position() / STEPS_PER_OUTPUT_DEG; }

static float slipDeg() {
  if (!encoderPresent || !encoderOn) return 0.0f;
  return cmdDeg() - encoder.angleDeg();
}

static void printHeader() {
  Serial.println(F("ms,cmd_sps,act_sps,cmd_steps,enc_deg,slip_deg,i2c_err"));
}

static void startRun(int8_t dir) {
  runDir = dir;
  motor.enable(true);
  motor.setMaxSpeed(cmdSps);
  motor.setAcceleration(accelSps2);
  motor.setPosition(0);
  if (encoderPresent) encoder.zeroHere();
  runStartMs = millis();
  runStartPos = 0;
  lastLogPos = 0;
  peakSlipDeg = 0.0f;
  motor.setVelocity(cmdSps * dir);
  Serial.print(F("# RUN cmd_sps="));
  Serial.print(cmdSps, 1);
  Serial.print(F(" accel="));
  Serial.print(accelSps2, 0);
  Serial.print(F(" encoder="));
  Serial.println(encoderOn ? F("on") : F("off"));
}

// The stop summary is the deliverable. Net slip after the shaft has come to
// rest is lost steps; slip that vanishes on stop was only following error.
static void endRun() {
  motor.stop();
  runDir = 0;
  const uint32_t settleUntil = millis() + 500;
  while (millis() < settleUntil) {
    motor.run();
    if (encoderPresent && encoderOn) encoder.read();
  }
  const float elapsed = (millis() - runStartMs) * 0.001f;
  const long issued = motor.position() - runStartPos;
  const float actual = elapsed > 0.0f ? fabsf(issued) / elapsed : 0.0f;

  Serial.print(F("# STOP cmd_sps="));
  Serial.print(cmdSps, 1);
  Serial.print(F(" act_sps="));
  Serial.print(actual, 1);
  Serial.print(F(" keepup_pct="));
  Serial.print(cmdSps > 0.0f ? 100.0f * actual / cmdSps : 0.0f, 1);
  Serial.print(F(" steps="));
  Serial.print(issued);
  Serial.print(F(" net_slip_deg="));
  Serial.print(slipDeg(), 3);
  Serial.print(F(" net_slip_steps="));
  Serial.print(slipDeg() * STEPS_PER_OUTPUT_DEG, 1);
  Serial.print(F(" peak_slip_deg="));
  Serial.print(peakSlipDeg, 3);
  Serial.print(F(" i2c_err="));
  Serial.println(encoderPresent ? encoder.errorCount() : 0);
  Serial.println(F("# ^ RECORD THIS LINE. keepup_pct < 98 means the MCU ran "
                   "out, not the motor."));
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
  }
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println(F("\n=== app_09_pullout ==="));

  if (mux.begin() && mux.isPresent()) {
    encoder.setDirection(ENCODER_DIRECTION);
    encoder.setFilterAlpha(1.0f);  // no filtering: this app measures raw slip
    encoderPresent = encoder.begin();
  }
  if (!encoderPresent) {
    encoderOn = false;
    Serial.println(F("# no encoder - open-loop mode, use a shaft index mark"));
  } else {
    encoder.readStatus();
    Serial.print(F("# magnet="));
    Serial.print(encoder.magnetText());
    Serial.print(F(" agc="));
    Serial.println(encoder.agc());
  }

  motor.begin();
  motor.setMaxSpeed(cmdSps);
  motor.setAcceleration(accelSps2);
  motor.setPosition(0);

  Serial.print(F("# steps_per_output_deg="));
  Serial.println(STEPS_PER_OUTPUT_DEG, 4);
  Serial.println(F("# keys: <num>=sps a<num>=accel c/v=run x=stop e z l ?"));
  Serial.println(F("# driver disabled until you press c or v"));
  printHeader();
}

void loop() {
  const char *line = cli.poll();
  if (line && line[0]) {
    if (line[0] == 'a') {
      const float v = (float)strtod(line + 1, nullptr);  // avr-libc has no %f
      if (v > 0.0f) {
        accelSps2 = v;
        motor.setAcceleration(accelSps2);
      }
      Serial.print(F("# accel="));
      Serial.println(accelSps2, 0);
    } else if (line[0] == '-' || (line[0] >= '0' && line[0] <= '9')) {
      const float v = (float)strtod(line, nullptr);
      cmdSps = constrain(fabsf(v), 1.0f, PULLOUT_MAX_SPS);
      motor.setMaxSpeed(cmdSps);
      if (runDir != 0) motor.setVelocity(cmdSps * runDir);
      Serial.print(F("# cmd_sps="));
      Serial.print(cmdSps, 1);
      Serial.print(F(" motor_rpm="));
      Serial.println(cmdSps * 60.0f / STEPS_PER_MOTOR_REV, 1);
    } else {
      switch (line[0]) {
        case 'c':
          startRun(+1);
          break;
        case 'v':
          startRun(-1);
          break;
        case 'x':
          endRun();
          break;
        case 'e':
          motor.enable(!motor.isEnabled());
          Serial.println(motor.isEnabled() ? F("# enabled") : F("# disabled"));
          break;
        case 'z':
          if (encoderPresent) encoder.zeroHere();
          motor.setPosition(0);
          Serial.println(F("# zeroed"));
          break;
        case 'l':
          encoderOn = encoderPresent && !encoderOn;
          Serial.print(F("# encoder streaming "));
          Serial.println(encoderOn ? F("on") : F("off (max step rate)"));
          break;
        case '?':
          if (encoderPresent) {
            encoder.readStatus();
            Serial.print(F("# magnet="));
            Serial.print(encoder.magnetText());
            Serial.print(F(" agc="));
            Serial.print(encoder.agc());
            Serial.print(F(" i2c_err="));
            Serial.print(encoder.errorCount());
          }
          Serial.print(F(" cmd_sps="));
          Serial.print(cmdSps, 1);
          Serial.print(F(" accel="));
          Serial.println(accelSps2, 0);
          break;
        default:
          break;
      }
    }
  }

  motor.run();
  if (encoderPresent && encoderOn) encoder.read();

  const uint32_t now = millis();
  if (now - lastLogMs >= LOG_PERIOD_MS) {
    const float dt = (now - lastLogMs) * 0.001f;
    lastLogMs = now;
    const long pos = motor.position();
    const float actual = dt > 0.0f ? fabsf(pos - lastLogPos) / dt : 0.0f;
    lastLogPos = pos;

    const float slip = slipDeg();
    if (fabsf(slip) > peakSlipDeg) peakSlipDeg = fabsf(slip);

    Serial.print(now);
    Serial.print(',');
    Serial.print(cmdSps, 1);
    Serial.print(',');
    Serial.print(actual, 1);
    Serial.print(',');
    Serial.print(pos);
    Serial.print(',');
    Serial.print(encoderPresent && encoderOn ? encoder.angleDeg() : 0.0f, 3);
    Serial.print(',');
    Serial.print(slip, 3);
    Serial.print(',');
    Serial.println(encoderPresent ? encoder.errorCount() : 0);
  }
}
