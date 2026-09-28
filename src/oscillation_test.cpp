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
const uint8_t ENCODER_SAMPLE_STEPS = 10;
const uint16_t STEP_RATE = 400;
const uint16_t HALF_TURN_RAW = 2048;
const uint16_t MAX_STEPS_PER_SWEEP = 8192;

bool driverEnabled = false;

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

bool magnetFieldIsValid(uint8_t status) {
  return (status & 0x38) == 0x20;
}

int16_t wrappedDelta(uint16_t current, uint16_t previous) {
  int16_t delta = current - previous;
  if (delta > 2047) delta -= 4096;
  if (delta < -2048) delta += 4096;
  return delta;
}

const char *switchName() {
  const bool first = digitalRead(PIN_SWITCH) == LOW;
  for (uint8_t sample = 0; sample < 3; sample++) {
    delay(2);
    if ((digitalRead(PIN_SWITCH) == LOW) != first) return "BOUNCING";
  }
  return first ? "TOUCHING" : "CLEAR";
}

void disableDriver() {
  digitalWrite(PIN_EN, HIGH);
  driverEnabled = false;
}

void printStatus() {
  uint16_t rawAngle = 0;
  uint8_t status = 0;
  const bool encoderOk = readEncoder(&rawAngle, &status);
  Serial.print("STATUS,driver=");
  Serial.print(driverEnabled ? "ENABLED" : "DISABLED");
  Serial.print(",switch=");
  Serial.print(switchName());
  Serial.print(",encoder=");
  Serial.print(encoderOk ? "OK" : "NO_RESPONSE");
  if (encoderOk) {
    Serial.print(",field=");
    Serial.print(magnetFieldIsValid(status) ? "GOOD" : "INVALID");
    Serial.print(",sensor_status=0x");
    Serial.print(status, HEX);
    Serial.print(",raw=");
    Serial.print(rawAngle);
  }
  Serial.println();
}

void printHelp() {
  Serial.println("COMMANDS,STATUS | RUN | STOP | HELP");
  Serial.println("RUN oscillates the motor shaft by measured 180-degree half-turns.");
  Serial.println("STOP disables the driver; each sweep is limited to 8192 step pulses.");
}

bool readCommandLine(char *command, size_t commandSize) {
  static char input[32];
  static size_t inputLength = 0;
  static bool overflow = false;

  while (Serial.available()) {
    const char character = Serial.read();
    if (character == '\r') continue;
    if (character == '\n') {
      if (overflow) {
        Serial.println("ERROR,command_too_long");
        inputLength = 0;
        overflow = false;
        continue;
      }
      input[inputLength] = '\0';
      strncpy(command, input, commandSize - 1);
      command[commandSize - 1] = '\0';
      inputLength = 0;
      return true;
    }
    if (overflow) continue;
    if (inputLength < sizeof(input) - 1) {
      input[inputLength++] = character;
    } else {
      overflow = true;
    }
  }
  return false;
}

bool stopRequested() {
  char command[32];
  if (!readCommandLine(command, sizeof(command))) return false;
  size_t length = strlen(command);
  while (length > 0 && command[length - 1] == ' ') command[--length] = '\0';
  if (strcmp(command, "STOP") == 0) {
    disableDriver();
    Serial.println("STOPPED,driver=DISABLED");
    return true;
  }
  Serial.println("ERROR,send_STOP_to_end_run");
  return false;
}

bool pulseStep() {
  const uint16_t halfPeriodUs = 500000UL / STEP_RATE;
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(halfPeriodUs);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(halfPeriodUs);
  return true;
}

bool runSweep(bool forward) {
  uint16_t rawAngle = 0;
  uint8_t status = 0;
  if (!readEncoder(&rawAngle, &status) || !magnetFieldIsValid(status)) {
    disableDriver();
    Serial.println("ABORT,encoder_or_magnet_fault,driver=DISABLED");
    return false;
  }

  const uint16_t startRaw = rawAngle;
  int32_t sweepDelta = 0;
  uint16_t steps = 0;
  digitalWrite(PIN_DIR, forward ? HIGH : LOW);
  delay(100);

  while (labs(sweepDelta) < HALF_TURN_RAW && steps < MAX_STEPS_PER_SWEEP) {
    for (uint8_t sampleStep = 0; sampleStep < ENCODER_SAMPLE_STEPS; sampleStep++) {
      if (steps >= MAX_STEPS_PER_SWEEP) break;
      pulseStep();
      steps++;
    }

    if (Serial.available() && stopRequested()) return false;
    if (!readEncoder(&rawAngle, &status) || !magnetFieldIsValid(status)) {
      disableDriver();
      Serial.println("ABORT,encoder_or_magnet_fault,driver=DISABLED");
      return false;
    }
    sweepDelta += wrappedDelta(rawAngle, (uint16_t)((startRaw + sweepDelta) & 0x0FFF));
  }

  if (labs(sweepDelta) < HALF_TURN_RAW) {
    disableDriver();
    Serial.print("ABORT,sweep_limit,direction=");
    Serial.print(forward ? "FWD" : "REV");
    Serial.println(",driver=DISABLED");
    return false;
  }

  Serial.print("SWEEP,direction=");
  Serial.print(forward ? "FWD" : "REV");
  Serial.print(",steps=");
  Serial.print(steps);
  Serial.print(",delta_raw=");
  Serial.print(sweepDelta);
  Serial.print(",delta_deg=");
  Serial.print(sweepDelta * (360.0f / 4096.0f), 2);
  Serial.print(",switch=");
  Serial.print(switchName());
  Serial.print(",raw_end=");
  Serial.println(rawAngle);
  delay(250);
  return true;
}

void runOscillation() {
  uint16_t rawAngle = 0;
  uint8_t status = 0;
  if (!readEncoder(&rawAngle, &status) || !magnetFieldIsValid(status)) {
    Serial.println("ABORT,encoder_or_magnet_fault,driver=DISABLED");
    return;
  }

  digitalWrite(PIN_EN, DRIVER_ENABLED);
  driverEnabled = true;
  Serial.println("RUNNING,send_STOP_to_end");
  bool forward = true;
  while (driverEnabled) {
    if (!runSweep(forward)) return;
    forward = !forward;
  }
}

void handleCommand(char *command) {
  size_t length = strlen(command);
  while (length > 0 && (command[length - 1] == '\r' || command[length - 1] == ' ')) {
    command[--length] = '\0';
  }
  if (strcmp(command, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(command, "HELP") == 0) {
    printHelp();
  } else if (strcmp(command, "RUN") == 0) {
    runOscillation();
  } else if (strcmp(command, "STOP") == 0) {
    disableDriver();
    Serial.println("STOPPED,driver=DISABLED");
  } else if (command[0] != '\0') {
    Serial.println("ERROR,unknown_command; send_HELP");
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
  Serial.println("READY,oscillation_test_v1,driver=DISABLED");
  printHelp();
  printStatus();
}

void loop() {
  char command[32];
  if (readCommandLine(command, sizeof(command))) handleCommand(command);
}