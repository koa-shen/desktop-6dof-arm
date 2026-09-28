#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>

const uint8_t PIN_STEP = 2;
const uint8_t PIN_DIR = 3;
const uint8_t PIN_EN = 4;
const uint8_t PIN_SWITCH = 7;
const uint8_t DRIVER_ENABLED = LOW;
const uint8_t AS5600_ADDR = 0x36;
const uint16_t MAX_JOG_STEPS = 200;
const uint8_t ENCODER_SAMPLE_STEPS = 10;
const uint16_t STEP_RATE = 400;

bool driverEnabled = false;
bool trackingInitialized = false;
uint16_t previousRaw = 0;
int32_t accumulatedRaw = 0;

bool readEncoder(uint16_t *rawAngle, uint8_t *status) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0B);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)AS5600_ADDR, 1) != 1) {
    return false;
  }
  *status = Wire.read();

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)AS5600_ADDR, 2) != 2) {
    return false;
  }
  *rawAngle = (((uint16_t)Wire.read() << 8) | Wire.read()) & 0x0FFF;
  return true;
}

int16_t wrappedDelta(uint16_t current, uint16_t previous) {
  int16_t delta = current - previous;
  if (delta > 2047) delta -= 4096;
  if (delta < -2048) delta += 4096;
  return delta;
}

void updateTracking(uint16_t rawAngle) {
  if (trackingInitialized) {
    accumulatedRaw += wrappedDelta(rawAngle, previousRaw);
  } else {
    trackingInitialized = true;
  }
  previousRaw = rawAngle;
}

const char *fieldName(uint8_t status) {
  const bool detected = (status & 0x20) != 0;
  const bool weak = (status & 0x10) != 0;
  const bool strong = (status & 0x08) != 0;
  if (!detected) return "NO_MAGNET";
  if (weak) return "WEAK";
  if (strong) return "STRONG";
  return "GOOD";
}

const char *switchName() {
  bool first = digitalRead(PIN_SWITCH);
  for (uint8_t sample = 0; sample < 3; sample++) {
    delay(2);
    if (digitalRead(PIN_SWITCH) != first) return "BOUNCING";
  }
  return first == LOW ? "TOUCHING" : "CLEAR";
}

void printStatus() {
  uint16_t rawAngle = 0;
  uint8_t sensorStatus = 0;
  const bool encoderOk = readEncoder(&rawAngle, &sensorStatus);
  if (encoderOk) updateTracking(rawAngle);

  Serial.print("STATUS,driver=");
  Serial.print(driverEnabled ? "ENABLED" : "DISABLED");
  Serial.print(",switch=");
  Serial.print(switchName());
  Serial.print(",encoder=");
  Serial.print(encoderOk ? "OK" : "NO_RESPONSE");
  if (encoderOk) {
    Serial.print(",field=");
    Serial.print(fieldName(sensorStatus));
    Serial.print(",sensor_status=0x");
    Serial.print(sensorStatus, HEX);
    Serial.print(",raw=");
    Serial.print(rawAngle);
    Serial.print(",tracked_motor_deg=");
    Serial.print(accumulatedRaw * (360.0f / 4096.0f), 3);
  }
  Serial.println();
}

void printHelp() {
  Serial.println("COMMANDS,STATUS | ENABLE | DISABLE | F <1-200> | R <1-200> | HELP");
  Serial.println("F=DIR HIGH; R=DIR LOW; jogs require ENABLE; max 200 steps per command.");
}

void jog(bool forward, long requestedSteps) {
  if (!driverEnabled) {
    Serial.println("ERROR,driver_disabled; send ENABLE first");
    return;
  }
  if (requestedSteps < 1 || requestedSteps > MAX_JOG_STEPS) {
    Serial.println("ERROR,jog_steps_must_be_1_to_200");
    return;
  }

  uint16_t startRaw = 0;
  uint8_t startStatus = 0;
  bool trackingOk = readEncoder(&startRaw, &startStatus);
  const bool startOk = trackingOk;
  if (startOk) updateTracking(startRaw);
  const int32_t startTracked = accumulatedRaw;
  const char *startSwitch = switchName();

  digitalWrite(PIN_DIR, forward ? HIGH : LOW);
  const uint16_t halfPeriodUs = 500000UL / STEP_RATE;
  for (long step = 0; step < requestedSteps; step++) {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(halfPeriodUs);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(halfPeriodUs);
    if ((step + 1) % ENCODER_SAMPLE_STEPS == 0) {
      uint16_t sampleRaw = 0;
      uint8_t sampleStatus = 0;
      if (readEncoder(&sampleRaw, &sampleStatus)) {
        updateTracking(sampleRaw);
      } else {
        trackingOk = false;
      }
    }
  }

  uint16_t endRaw = 0;
  uint8_t endStatus = 0;
  const bool endOk = readEncoder(&endRaw, &endStatus);
  if (endOk) updateTracking(endRaw);
  trackingOk = trackingOk && endOk;

  Serial.print("JOG,direction=");
  Serial.print(forward ? "FWD" : "REV");
  Serial.print(",steps=");
  Serial.print(requestedSteps);
  Serial.print(",switch_before=");
  Serial.print(startSwitch);
  Serial.print(",switch_after=");
  Serial.print(switchName());
  Serial.print(",encoder=");
  Serial.print(trackingOk ? "OK" : "PARTIAL");
  if (startOk && endOk) {
    Serial.print(",field=");
    Serial.print(fieldName(endStatus));
    Serial.print(",raw_start=");
    Serial.print(startRaw);
    Serial.print(",raw_end=");
    Serial.print(endRaw);
    Serial.print(",tracked_delta_raw=");
    Serial.print(accumulatedRaw - startTracked);
    Serial.print(",delta_deg=");
    Serial.print((accumulatedRaw - startTracked) * (360.0f / 4096.0f), 3);
    Serial.print(",tracked_motor_deg=");
    Serial.print(accumulatedRaw * (360.0f / 4096.0f), 3);
    Serial.print(",tracked_delta_deg=");
    Serial.print((accumulatedRaw - startTracked) * (360.0f / 4096.0f), 3);
  }
  Serial.println();
}

void handleCommand(char *line) {
  char *command = strtok(line, " \t\r\n");
  if (command == NULL) return;

  if (strcmp(command, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(command, "HELP") == 0) {
    printHelp();
  } else if (strcmp(command, "ENABLE") == 0) {
    digitalWrite(PIN_EN, DRIVER_ENABLED);
    driverEnabled = true;
    Serial.println("DRIVER,ENABLED");
  } else if (strcmp(command, "DISABLE") == 0) {
    digitalWrite(PIN_EN, HIGH);
    driverEnabled = false;
    Serial.println("DRIVER,DISABLED");
  } else if (strcmp(command, "F") == 0 || strcmp(command, "R") == 0) {
    char *countText = strtok(NULL, " \t\r\n");
    char *end = NULL;
    long count = countText == NULL ? 0 : strtol(countText, &end, 10);
    if (countText == NULL || end == countText || *end != '\0') {
      Serial.println("ERROR,usage: F <1-200> or R <1-200>");
      return;
    }
    jog(command[0] == 'F', count);
  } else {
    Serial.println("ERROR,unknown_command; send HELP");
  }
}

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_SWITCH, INPUT_PULLUP);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_EN, HIGH);

  Serial.begin(115200);
  Wire.begin();
  delay(500);
  Serial.println("READY,motion_test_v1,driver=DISABLED");
  printHelp();
  printStatus();
}

void loop() {
  static char commandLine[32];
  if (!Serial.available()) return;
  size_t length = Serial.readBytesUntil('\n', commandLine, sizeof(commandLine) - 1);
  commandLine[length] = '\0';
  handleCommand(commandLine);
}