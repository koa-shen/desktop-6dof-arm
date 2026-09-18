#include <Arduino.h>
#include <Wire.h>

const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_STATUS_REGISTER = 0x0B;
const uint8_t AS5600_ANGLE_REGISTER = 0x0E;

bool readRegister(uint8_t registerAddress, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(AS5600_ADDR, length) != length) return false;
  for (uint8_t index = 0; index < length; index++) {
    buffer[index] = Wire.read();
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("as5600_direct_i2c_test");
  Serial.println("AS5600 must be connected: VCC->5V, GND->GND, SDA->A4, SCL->A5");
}

void loop() {
  uint8_t status;
  uint8_t angleBytes[2];

  if (!readRegister(AS5600_STATUS_REGISTER, &status, 1)) {
    Serial.println("ERROR: no response from AS5600 at 0x36");
    delay(1000);
    return;
  }

  if (!readRegister(AS5600_ANGLE_REGISTER, angleBytes, 2)) {
    Serial.println("ERROR: AS5600 acknowledged but angle read failed");
    delay(1000);
    return;
  }

  const uint16_t rawAngle = ((uint16_t)angleBytes[0] << 8 | angleBytes[1]) & 0x0FFF;
  Serial.print("AS5600 OK, magnet=");
  Serial.print(status & 0x20 ? "detected" : "NOT detected");
  Serial.print(", strength=");
  if (status & 0x08) {
    Serial.print("too weak");
  } else if (status & 0x10) {
    Serial.print("too strong");
  } else if (status & 0x20) {
    Serial.print("valid");
  } else {
    Serial.print("unavailable");
  }
  Serial.print(", raw_angle=");
  Serial.println(rawAngle);
  delay(1000);
}
