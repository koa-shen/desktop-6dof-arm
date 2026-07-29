#include <Arduino.h>
#include <Wire.h>

// -------------------- Pin config (EDIT THESE) --------------------
const uint8_t PIN_STEP = 2;
const uint8_t PIN_DIR  = 3;
const uint8_t PIN_EN   = 4;   // TMC2209 EN (often active LOW)

// -------------------- I2C devices --------------------
const uint8_t TCA_ADDR    = 0x70; // TCA9548A default
const uint8_t AS5600_ADDR = 0x36; // AS5600 fixed address
const uint8_t MUX_CH      = 0;    // phase 1: encoder on channel 0

// -------------------- Motion config --------------------
const int STEPS_PER_MOVE = 200;   // adjust for visibility
const int STEP_US = 700;          // pulse period half-cycle (slower = safer)
const int PAUSE_MS = 500;

// -------------------- AS5600 registers --------------------
const uint8_t REG_RAW_ANGLE_H = 0x0C;
const uint8_t REG_RAW_ANGLE_L = 0x0D;

long commandedStepPos = 0;

// Select one TCA9548A channel
bool tcaSelect(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  return (Wire.endTransmission() == 0);
}

// Read 12-bit raw angle from AS5600 (0..4095)
bool as5600ReadRaw(uint16_t &raw) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(REG_RAW_ANGLE_H);
  if (Wire.endTransmission(false) != 0) return false; // repeated start

  uint8_t n = Wire.requestFrom((int)AS5600_ADDR, 2);
  if (n != 2) return false;

  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  raw = ((uint16_t)hi << 8 | lo) & 0x0FFF;
  return true;
}

float rawToDeg(uint16_t raw) {
  return (360.0f * raw) / 4096.0f;
}

void pulseStepOnce() {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(STEP_US);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(STEP_US);
}

void moveSteps(int steps, bool dirCW) {
  digitalWrite(PIN_DIR, dirCW ? HIGH : LOW);

  for (int i = 0; i < steps; i++) {
    pulseStepOnce();
    commandedStepPos += dirCW ? 1 : -1;

    // sample encoder every few steps to reduce serial spam
    if ((i % 10) == 0) {
      if (!tcaSelect(MUX_CH)) {
        Serial.println("ERR,tca_select");
        continue;
      }

      uint16_t raw = 0;
      if (!as5600ReadRaw(raw)) {
        Serial.println("ERR,as5600_read");
        continue;
      }

      float deg = rawToDeg(raw);

      // CSV: time_ms,commanded_steps,encoder_deg
      Serial.print(millis());
      Serial.print(",");
      Serial.print(commandedStepPos);
      Serial.print(",");
      Serial.println(deg, 3);
    }
  }
}

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);

  digitalWrite(PIN_EN, LOW); // enable driver (common for TMC modules)

  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(100000); // 100kHz for stability first

  Serial.println("phase1_start");
  Serial.println("ms,step_pos,angle_deg");

  // quick device sanity check
  if (!tcaSelect(MUX_CH)) {
    Serial.println("FATAL: mux channel select failed");
  } else {
    uint16_t raw;
    if (!as5600ReadRaw(raw)) {
      Serial.println("FATAL: as5600 not readable on mux channel");
    } else {
      Serial.print("as5600_raw_init,");
      Serial.println(raw);
    }
  }
}

void loop() {
  moveSteps(STEPS_PER_MOVE, true);   // CW
  delay(PAUSE_MS);
  moveSteps(STEPS_PER_MOVE, false);  // CCW
  delay(PAUSE_MS);
}
