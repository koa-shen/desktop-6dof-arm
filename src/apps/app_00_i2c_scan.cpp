// App 00 - I2C bus scan
//
// FIRST THING YOU FLASH. No motor power needed, USB only.
// Scans the main bus, then every mux channel, and reports AS5600 magnet health.
//
//   pio run -e i2c_scan -t upload && pio device monitor
//
// PASS: mux found at 0x70, AS5600 found on the channel you wired, magnet OK.

#include <Arduino.h>
#include <Wire.h>

#include "TCA9548A.h"
#include "AS5600Encoder.h"
#include "pins.h"

static TCA9548A mux(TCA9548A_ADDR);

static uint8_t scanBus(bool skipMux) {
  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    if (skipMux && addr == TCA9548A_ADDR) continue;
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("    0x"));
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      if (addr == AS5600_ADDR) Serial.print(F("  <- AS5600"));
      if (addr == TCA9548A_ADDR) Serial.print(F("  <- TCA9548A"));
      Serial.println();
      found++;
    }
  }
  if (found == 0) Serial.println(F("    (nothing)"));
  return found;
}

static void reportEncoder(uint8_t channel) {
  AS5600Encoder enc(&mux, channel);
  if (!enc.readStatus() || !enc.read()) {
    Serial.println(F("    AS5600 present but not readable"));
    return;
  }
  Serial.print(F("    magnet="));
  Serial.print(enc.magnetText());
  Serial.print(F(" agc="));
  Serial.print(enc.agc());
  Serial.print(F(" magnitude="));
  Serial.print(enc.magnitude());
  Serial.print(F(" raw="));
  Serial.print(enc.rawCounts());
  Serial.print(F(" deg="));
  Serial.println(enc.mechanicalDeg(), 2);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
  }
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println();
  Serial.println(F("=== app_00_i2c_scan ==="));
}

void loop() {
  mux.disableAll();

  Serial.println(F("\n[1] main bus (all mux channels closed)"));
  scanBus(false);

  if (!mux.isPresent()) {
    Serial.println(F("\nFAIL: no TCA9548A at 0x70."));
    Serial.println(F("  - check SDA=A4, SCL=A5, VCC=5V, GND"));
    Serial.println(F("  - check A0/A1/A2 address jumpers"));
    Serial.println(F("  - pull-ups: most breakouts already have them"));
    delay(3000);
    return;
  }

  Serial.println(F("\n[2] per-channel scan"));
  for (uint8_t ch = 0; ch < 8; ch++) {
    if (!mux.select(ch, true)) {
      Serial.print(F("  ch"));
      Serial.print(ch);
      Serial.println(F(": select failed"));
      continue;
    }
    Serial.print(F("  ch"));
    Serial.print(ch);
    Serial.println(':');
    const uint8_t n = scanBus(true);
    if (n > 0) {
      Wire.beginTransmission(AS5600_ADDR);
      if (Wire.endTransmission() == 0) reportEncoder(ch);
    }
  }
  mux.disableAll();

  Serial.println(F("\nrescanning in 5 s (Ctrl+C to stop)"));
  delay(5000);
}
