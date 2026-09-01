// App 02a - Vref setup, no motion
//
// Holds the TMC2209 interface at static levels for current-limit measurement.
// No STEP pulses, no enable toggling, no I2C, no encoder, no timed behavior.

#include <Arduino.h>

#include "pins.h"

static void holdSafePins() {
  digitalWrite(PIN_J1_STEP, LOW);
  digitalWrite(PIN_J1_DIR, LOW);
  digitalWrite(PIN_DRIVER_EN, DRIVER_EN_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  pinMode(PIN_J1_STEP, OUTPUT);
  pinMode(PIN_J1_DIR, OUTPUT);
  pinMode(PIN_DRIVER_EN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  holdSafePins();

  Serial.begin(SERIAL_BAUD);
  Serial.println(F("\n=== app_02_vref_setup ==="));
  Serial.println(F("# static pin state for TMC2209 Vref measurement"));
  Serial.println(F("# D2 STEP=LOW, D3 DIR=LOW, D4 EN=disabled/HIGH"));
  Serial.println(F("# no motor commands are accepted in this app"));
}

void loop() {
  holdSafePins();
}