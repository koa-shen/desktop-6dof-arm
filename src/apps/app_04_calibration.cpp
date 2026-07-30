// App 04 - Joint calibration
//
// Measures the three numbers you cannot guess:
//   1. encoder direction sign (does +steps mean +degrees?)
//   2. actual steps per output degree (catches wrong microstep jumpers and
//      wrong gear ratio assumptions)
//   3. backlash (approach the same point from both sides, take the difference)
//
//   pio run -e calibration -t upload && pio device monitor
//   then type 'g' to run the routine
//
// At the end it prints a block to paste into include/joint_config.h.
//
// Run this with the joint UNLOADED and free to move +/- the test angle.

#include <Arduino.h>
#include <Wire.h>

#include "AS5600Encoder.h"
#include "ArmMath.h"
#include "SerialCli.h"
#include "StepperDriver.h"
#include "TCA9548A.h"
#include "joint_config.h"
#include "pins.h"

static TCA9548A mux(TCA9548A_ADDR);
static AS5600Encoder encoder(&mux, MUX_CH_J1);
static StepperDriver motor(PIN_J1_STEP, PIN_J1_DIR, PIN_J1_EN,
                           DRIVER_EN_ACTIVE_LOW);
static SerialCli<16> cli;

// Test move size in *commanded* steps. 20 deg at the configured scale.
static const long kTestSteps = (long)(20.0f * STEPS_PER_OUTPUT_DEG);
static const uint8_t kScaleReps = 3;
static const float kCalSpeed = 400.0f;  // slow enough to never skip steps

/// Blocking move that keeps the encoder fresh. Only used inside calibration.
static void moveAndSettle(long relSteps) {
  motor.move(relSteps);
  while (motor.isMoving()) {
    motor.run();
    encoder.read();
  }
  const uint32_t until = millis() + 500;  // let ringing die out
  while (millis() < until) encoder.read();
}

static float measure() {
  float sum = 0.0f;
  for (uint8_t i = 0; i < 20; i++) {
    encoder.read();
    sum += encoder.angleRawDeg();
    delay(5);
  }
  return sum / 20.0f;
}

static void runCalibration() {
  Serial.println(F("\n--- calibration start ---"));
  encoder.setDirection(+1);  // measure the true sign, so start neutral
  encoder.zeroHere();
  motor.setMaxSpeed(kCalSpeed);
  motor.enable(true);
  delay(200);

  // --- 1. direction ---------------------------------------------------------
  const float before = measure();
  moveAndSettle(+kTestSteps);
  const float after = measure();
  const float delta = after - before;
  const int8_t dir = (delta >= 0.0f) ? +1 : -1;
  Serial.print(F("1) direction: +"));
  Serial.print(kTestSteps);
  Serial.print(F(" steps -> "));
  Serial.print(delta, 3);
  Serial.print(F(" deg  => ENCODER_DIRECTION = "));
  Serial.println(dir);

  if (fabsf(delta) < 1.0f) {
    Serial.println(F("   !! encoder barely moved. Magnet not on the output"));
    Serial.println(F("      shaft, motor skipping, or driver not enabled."));
    motor.enable(false);
    return;
  }

  // --- 2. scale -------------------------------------------------------------
  // Always approach from the same direction so backlash does not pollute this.
  float sumStepsPerDeg = 0.0f;
  Serial.println(F("2) scale (same-direction moves):"));
  for (uint8_t i = 0; i < kScaleReps; i++) {
    const float a = measure();
    moveAndSettle(+kTestSteps);
    const float b = measure();
    const float degMoved = fabsf(b - a);
    const float stepsPerDeg = kTestSteps / degMoved;
    sumStepsPerDeg += stepsPerDeg;
    Serial.print(F("   rep "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(degMoved, 3);
    Serial.print(F(" deg -> "));
    Serial.print(stepsPerDeg, 4);
    Serial.println(F(" steps/deg"));
  }
  const float measuredStepsPerDeg = sumStepsPerDeg / kScaleReps;
  const float impliedRatio = measuredStepsPerDeg * 360.0f / STEPS_PER_MOTOR_REV;

  // --- 3. backlash ----------------------------------------------------------
  // Land on the same commanded position from each side and compare.
  Serial.println(F("3) backlash:"));
  moveAndSettle(+kTestSteps);
  moveAndSettle(-kTestSteps);
  const float fromPositive = measure();
  moveAndSettle(-kTestSteps);
  moveAndSettle(+kTestSteps);
  const float fromNegative = measure();
  const float backlashDeg = fabsf(fromPositive - fromNegative);
  Serial.print(F("   approach+ = "));
  Serial.print(fromPositive, 3);
  Serial.print(F(" deg, approach- = "));
  Serial.print(fromNegative, 3);
  Serial.println(F(" deg"));

  motor.enable(false);

  // --- results --------------------------------------------------------------
  Serial.println(F("\n--- RESULTS: paste into include/joint_config.h ---"));
  Serial.print(F("constexpr int8_t ENCODER_DIRECTION = "));
  Serial.print(dir);
  Serial.println(F(";"));
  Serial.print(F("constexpr float GEAR_RATIO = "));
  Serial.print(impliedRatio, 4);
  Serial.print(F("f;  // measured "));
  Serial.print(measuredStepsPerDeg, 4);
  Serial.println(F(" steps/output-deg"));
  Serial.print(F("// configured now: "));
  Serial.print(STEPS_PER_OUTPUT_DEG, 4);
  Serial.print(F(" steps/deg -> error "));
  Serial.print(100.0f * (measuredStepsPerDeg - STEPS_PER_OUTPUT_DEG) /
                   STEPS_PER_OUTPUT_DEG,
               2);
  Serial.println(F(" %"));
  Serial.print(F("// backlash = "));
  Serial.print(backlashDeg, 3);
  Serial.println(F(" deg (this is your open-loop accuracy floor)"));
  Serial.println(F("--- calibration done ---\n"));
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
  }
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println(F("\n=== app_04_calibration ==="));
  if (!mux.begin() || !encoder.begin()) {
    Serial.println(F("FATAL: encoder not readable - run app_00_i2c_scan"));
  }
  motor.begin();
  motor.setAcceleration(ACCEL_STEPS_PER_SEC2);

  Serial.print(F("# test move = "));
  Serial.print(kTestSteps);
  Serial.println(F(" steps (~20 deg at configured scale)"));
  Serial.println(F("# joint must be UNLOADED and free to move"));
  Serial.println(F("# type 'g' + Enter to run"));
}

void loop() {
  const char *line = cli.poll();
  if (line && line[0] == 'g') runCalibration();
}
