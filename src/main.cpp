#include <Arduino.h>

// -------------------- Pin config (EDIT THESE) --------------------
const uint8_t PIN_STEP = 2;
const uint8_t PIN_DIR  = 3;
const uint8_t PIN_EN   = 4;

// -------------------- Motor-only isolation test --------------------
const uint16_t STEP_HIGH_US = 4000;   // very slow pulses for bring-up
const uint16_t STEP_LOW_US  = 4000;
const uint16_t HOLD_TEST_MS = 8000;
const uint16_t STEPS_PER_BURST = 200;
const uint16_t IDLE_MS_BETWEEN_PHASES = 1500;

void pulseStepOnce() {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(STEP_HIGH_US);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(STEP_LOW_US);
}

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("motor_isolation_start");
  Serial.println("Each phase: hold test first, then slow CW and CCW bursts.");
}

void runStepBurst(bool enLevel, bool dirCW, uint16_t steps) {
  digitalWrite(PIN_EN, enLevel ? HIGH : LOW);
  digitalWrite(PIN_DIR, dirCW ? HIGH : LOW);

  Serial.print("step_burst,en=");
  Serial.print(enLevel ? "HIGH" : "LOW");
  Serial.print(",dir=");
  Serial.print(dirCW ? "CW" : "CCW");
  Serial.print(",steps=");
  Serial.println(steps);

  for (uint16_t i = 0; i < steps; i++) {
    pulseStepOnce();
  }
}

void runHoldTest(bool enLevel, uint16_t holdMs) {
  digitalWrite(PIN_EN, enLevel ? HIGH : LOW);

  Serial.print("hold_test,en=");
  Serial.print(enLevel ? "HIGH" : "LOW");
  Serial.print(",ms=");
  Serial.println(holdMs);
  Serial.println("Try turning shaft by hand now.");

  uint32_t t0 = millis();
  while ((millis() - t0) < holdMs) {
    delay(20);
  }
}

void loop() {
  // Phase A: EN LOW
  runHoldTest(false, HOLD_TEST_MS);
  runStepBurst(false, true, STEPS_PER_BURST);
  runStepBurst(false, false, STEPS_PER_BURST);
  digitalWrite(PIN_EN, HIGH);
  delay(IDLE_MS_BETWEEN_PHASES);

  // Phase B: EN HIGH
  runHoldTest(true, HOLD_TEST_MS);
  runStepBurst(true, true, STEPS_PER_BURST);
  runStepBurst(true, false, STEPS_PER_BURST);
  digitalWrite(PIN_EN, LOW);
  delay(IDLE_MS_BETWEEN_PHASES);
}
