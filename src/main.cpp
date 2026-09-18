#include <Arduino.h>
#include <Wire.h>

const uint8_t PIN_STEP = 2;
const uint8_t PIN_DIR = 3;
const uint8_t PIN_EN = 4;
const uint8_t DRIVER_ENABLED = LOW;
const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_STATUS_REGISTER = 0x0B;
const uint8_t AS5600_ANGLE_REGISTER = 0x0E;

const uint16_t SPEEDS[] = {
  500, 750, 1000, 1250, 1500, 1750, 2000, 2250,
  2500, 2750, 3000, 3250, 3500, 3750, 4000
};
const uint8_t SPEED_COUNT = sizeof(SPEEDS) / sizeof(SPEEDS[0]);
const uint8_t RUNS_PER_SPEED = 5;
const uint16_t START_STEP_RATE = 500;
const uint16_t RETURN_STEP_RATE = 1000;
const uint16_t RAMP_STEPS = 500;
const uint16_t SETTLE_STEPS = 100;
const uint16_t MEASURE_SAMPLE_STEPS = 100;
const uint16_t MEASURE_STEPS = 1600;
const int16_t EXPECTED_DELTA_RAW = -4096;
const uint16_t STALL_ERROR_RAW = 410;

bool readEncoder(uint16_t *rawAngle, uint8_t *status) {
  uint8_t angleBytes[2];

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_STATUS_REGISTER);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)AS5600_ADDR, 1) != 1) {
    return false;
  }
  *status = Wire.read();

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_ANGLE_REGISTER);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)AS5600_ADDR, 2) != 2) {
    return false;
  }
  angleBytes[0] = Wire.read();
  angleBytes[1] = Wire.read();
  *rawAngle = ((uint16_t)angleBytes[0] << 8 | angleBytes[1]) & 0x0FFF;
  return true;
}

bool magnetFieldIsValid(uint8_t status) {
  return (status & 0x38) == 0x20;
}

int16_t wrappedDelta(uint16_t currentAngle, uint16_t referenceAngle) {
  int16_t delta = currentAngle - referenceAngle;
  if (delta > 2047) delta -= 4096;
  if (delta < -2048) delta += 4096;
  return delta;
}

void pulseStep(uint16_t stepRate) {
  const uint16_t halfPeriodUs = 500000UL / stepRate;
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(halfPeriodUs);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(halfPeriodUs);
}

void runSteps(bool clockwise, uint16_t stepRate, uint16_t stepCount) {
  digitalWrite(PIN_DIR, clockwise ? HIGH : LOW);
  for (uint16_t step = 0; step < stepCount; step++) {
    pulseStep(stepRate);
  }
}

void runRamp(bool clockwise, uint16_t targetRate, bool accelerating) {
  for (uint16_t step = 0; step < RAMP_STEPS; step++) {
    uint16_t rampStep = accelerating ? step : RAMP_STEPS - step - 1;
    uint16_t stepRate = START_STEP_RATE +
                        (uint32_t)(targetRate - START_STEP_RATE) * rampStep / RAMP_STEPS;
    runSteps(clockwise, stepRate, 1);
  }
}

bool runMeasuredMove(uint16_t targetRate, uint16_t *startAngle, uint16_t *endAngle,
                     int16_t *measuredDelta, uint8_t *startStatus, uint8_t *endStatus) {
  digitalWrite(PIN_EN, DRIVER_ENABLED);
  runRamp(true, targetRate, true);
  runSteps(true, targetRate, SETTLE_STEPS);

  if (!readEncoder(startAngle, startStatus) || !magnetFieldIsValid(*startStatus)) return false;
  uint16_t previousAngle = *startAngle;
  *endAngle = previousAngle;
  *measuredDelta = 0;
  for (uint16_t step = 0; step < MEASURE_STEPS; step += MEASURE_SAMPLE_STEPS) {
    uint16_t currentAngle;
    runSteps(true, targetRate, MEASURE_SAMPLE_STEPS);
    if (!readEncoder(&currentAngle, endStatus) || !magnetFieldIsValid(*endStatus)) return false;
    *measuredDelta += wrappedDelta(currentAngle, previousAngle);
    previousAngle = currentAngle;
    *endAngle = previousAngle;
  }
  *endAngle = previousAngle;

  runSteps(true, targetRate, SETTLE_STEPS);
  runRamp(true, targetRate, false);
  return true;
}

void returnToStart(uint16_t targetRate) {
  runRamp(false, targetRate, true);
  runSteps(false, targetRate, SETTLE_STEPS * 2 + MEASURE_STEPS);
  runRamp(false, targetRate, false);
}

bool waitForStartCommand() {
  char command[6] = {};
  while (true) {
    if (Serial.available()) {
      size_t length = Serial.readBytesUntil('\n', command, sizeof(command) - 1);
      command[length] = '\0';
      if (strcmp(command, "START") == 0) return true;
    }
  }
}

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_EN, HIGH);

  Serial.begin(115200);
  Serial.setTimeout(100);
  Wire.begin();
  delay(500);
  uint16_t baselineAngle;
  uint8_t status;
  if (!readEncoder(&baselineAngle, &status)) {
    Serial.println("ABORT,encoder_not_found");
    return;
  }
  if (!magnetFieldIsValid(status)) {
    Serial.print("ABORT,invalid_magnet_field,0x");
    Serial.println(status, HEX);
    return;
  }

  Serial.println("READY,speed_sweep_vref_1.25V");
  if (!waitForStartCommand()) return;

  Serial.println("SWEEP_START");
  Serial.println("RESULT,speed_pps,run,start_raw,end_raw,measured_delta_raw,expected_delta_raw,status_start,status_end,outcome");
  Serial.print("RESULT,0,0,");
  Serial.print(baselineAngle);
  Serial.print(',');
  Serial.print(baselineAngle);
  Serial.print(",0,0,0x");
  Serial.print(status, HEX);
  Serial.print(",0x");
  Serial.print(status, HEX);
  Serial.println(",BASELINE");

  for (uint8_t speedIndex = 0; speedIndex < SPEED_COUNT; speedIndex++) {
    uint16_t targetRate = SPEEDS[speedIndex];
    for (uint8_t run = 1; run <= RUNS_PER_SPEED; run++) {
      uint16_t startAngle = 0;
      uint16_t endAngle = 0;
      int16_t measuredDelta = 0;
      uint8_t startStatus = 0;
      uint8_t endStatus = 0;
      bool encoderOk = runMeasuredMove(targetRate, &startAngle, &endAngle, &measuredDelta,
                   &startStatus, &endStatus);
      bool stalled = encoderOk &&
             (uint16_t)abs(measuredDelta - EXPECTED_DELTA_RAW) > STALL_ERROR_RAW;
      const char *outcome = !encoderOk ? "ENCODER_FAULT" : (stalled ? "STALL" : "PASS");

      Serial.print("RESULT,");
      Serial.print(targetRate);
      Serial.print(',');
      Serial.print(run);
      Serial.print(',');
      Serial.print(startAngle);
      Serial.print(',');
      Serial.print(endAngle);
      Serial.print(',');
      Serial.print(measuredDelta);
      Serial.print(',');
      Serial.print(EXPECTED_DELTA_RAW);
      Serial.print(",0x");
      Serial.print(startStatus, HEX);
      Serial.print(",0x");
      Serial.print(endStatus, HEX);
      Serial.print(',');
      Serial.println(outcome);

      if (!encoderOk || stalled) {
        digitalWrite(PIN_EN, HIGH);
        Serial.println(!encoderOk ? "ENCODER_FAULT_DETECTED" : "STALL_DETECTED");
        return;
      }
      returnToStart(RETURN_STEP_RATE);
    }
  }

  digitalWrite(PIN_EN, HIGH);
  Serial.println("SWEEP_COMPLETE");
}

void loop() {
  digitalWrite(PIN_EN, HIGH);
}
