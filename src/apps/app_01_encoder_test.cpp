// App 01 - Encoder only, no motor
//
// Turn the joint (or the magnet) BY HAND and watch the angle. This is where
// you validate magnet centering and gap before you ever energize the motor.
//
//   pio run -e encoder_test -t upload && pio device monitor
//
// Serial commands (type + Enter):
//   z   zero here            s   magnet status
//   f   toggle EMA filter    r   reset unwrap turns
//
// PASS: angle sweeps 0->360 smoothly with no jumps, magnet=OK across the whole
// range, and 360 deg of input gives 360 deg of output (no dead zones).

#include <Arduino.h>
#include <Wire.h>

#include "AS5600Encoder.h"
#include "SerialCli.h"
#include "TCA9548A.h"
#include "joint_config.h"
#include "pins.h"

static TCA9548A mux(TCA9548A_ADDR);
static AS5600Encoder encoder(&mux, MUX_CH_J1);
static SerialCli<16> cli;

static bool filterOn = false;
static uint32_t lastLogMs = 0;
static uint32_t lastStatusMs = 0;

static void printStatus() {
  encoder.readStatus();
  Serial.print(F("# magnet="));
  Serial.print(encoder.magnetText());
  Serial.print(F(" agc="));
  Serial.print(encoder.agc());
  Serial.print(F(" (ideal ~128, 0 or 255 = bad gap)"));
  Serial.print(F(" magnitude="));
  Serial.print(encoder.magnitude());
  Serial.print(F(" i2c_errors="));
  Serial.println(encoder.errorCount());
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
  }
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println(F("\n=== app_01_encoder_test ==="));
  if (!mux.begin() || !mux.isPresent()) {
    Serial.println(F("FATAL: TCA9548A not responding. Run app_00 first."));
  }
  encoder.setDirection(ENCODER_DIRECTION);
  if (!encoder.begin()) {
    Serial.println(F("FATAL: AS5600 not readable. Run app_00 first."));
  }
  printStatus();
  Serial.println(F("ms,raw,mech_deg,angle_deg,turns,magnet_ok"));
}

void loop() {
  const char *line = cli.poll();
  if (line && line[0]) {
    switch (line[0]) {
      case 'z':
        encoder.zeroHere();
        Serial.println(F("# zeroed"));
        break;
      case 's':
        printStatus();
        break;
      case 'f':
        filterOn = !filterOn;
        encoder.setFilterAlpha(filterOn ? 0.25f : 1.0f);
        Serial.print(F("# filter "));
        Serial.println(filterOn ? F("ON (alpha 0.25)") : F("OFF"));
        break;
      case 'r':
        encoder.begin();
        Serial.println(F("# unwrap reset"));
        break;
      default:
        Serial.println(F("# keys: z=zero s=status f=filter r=reset"));
        break;
    }
  }

  encoder.read();

  const uint32_t now = millis();
  if (now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;
    Serial.print(now);
    Serial.print(',');
    Serial.print(encoder.rawCounts());
    Serial.print(',');
    Serial.print(encoder.mechanicalDeg(), 3);
    Serial.print(',');
    Serial.print(encoder.angleDeg(), 3);
    Serial.print(',');
    Serial.print(encoder.turns());
    Serial.print(',');
    Serial.println(encoder.magnetOk() ? 1 : 0);
  }

  if (now - lastStatusMs >= 2000) {
    lastStatusMs = now;
    encoder.readStatus();  // keeps magnetOk() fresh in the CSV column
  }
}
