#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const uint8_t PIN_STEP = 2;
const uint8_t PIN_DIR = 3;
const uint8_t PIN_EN = 4;
const uint8_t PIN_SWITCH = 7;
const uint8_t DRIVER_ENABLED = LOW;
const uint8_t AS5600_ADDR = 0x36;
const uint8_t ENCODER_SAMPLE_STEPS = 10;
const uint16_t STEP_RATE = 800;
const uint32_t OUTPUT_HALF_TURN_RAW = 15UL * 2048UL;
const float MIN_MOVE_DEG = 0.01f;
const float MAX_MOVE_DEG = 180.0f;
const uint16_t MAX_STEPS_PER_LEG = 60000;
const uint16_t MAX_STEPS_WITHOUT_MOTION = 1600;

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
  Serial.println("COMMANDS,MOVE <signed_deg> | RUN | STOP | STATUS | HELP");
  Serial.println("MOVE accepts -180..180 degrees, minimum 0.01; positive=DIR HIGH.");
  Serial.println("RUN moves reducer output 180 degrees, then returns to its start once.");
  Serial.println("Assumes 15:1 reducer; STOP disables the driver during motion.");
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

void pulseStep() {
  const uint16_t halfPeriodUs = 500000UL / STEP_RATE;
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(halfPeriodUs);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(halfPeriodUs);
}

bool runLeg(bool forward, uint32_t targetRaw, const char *legName, int32_t *measuredDelta) {
  uint16_t rawAngle = 0;
  uint8_t status = 0;
  if (!readEncoder(&rawAngle, &status) || !magnetFieldIsValid(status)) {
    disableDriver();
    Serial.println("ABORT,encoder_or_magnet_fault,driver=DISABLED");
    return false;
  }

  uint16_t previousRaw = rawAngle;
  int32_t legDelta = 0;
  uint16_t steps = 0;
  uint16_t stepsWithoutMotion = 0;
  digitalWrite(PIN_DIR, forward ? HIGH : LOW);
  Serial.print("LEG_START,name=");
  Serial.print(legName);
  Serial.print(",target_output_deg=");
  Serial.println((float)targetRaw * 360.0f / (4096.0f * 15.0f), 2);

  while ((uint32_t)labs(legDelta) < targetRaw && steps < MAX_STEPS_PER_LEG) {
    for (uint8_t sampleStep = 0; sampleStep < ENCODER_SAMPLE_STEPS; sampleStep++) {
      if (steps >= MAX_STEPS_PER_LEG) break;
      pulseStep();
      steps++;
    }

    if (Serial.available() && stopRequested()) return false;
    if (!readEncoder(&rawAngle, &status) || !magnetFieldIsValid(status)) {
      disableDriver();
      Serial.println("ABORT,encoder_or_magnet_fault,driver=DISABLED");
      return false;
    }

    const int16_t delta = wrappedDelta(rawAngle, previousRaw);
    legDelta += delta;
    previousRaw = rawAngle;
    if (delta == 0) {
      stepsWithoutMotion += ENCODER_SAMPLE_STEPS;
      if (stepsWithoutMotion >= MAX_STEPS_WITHOUT_MOTION) {
        disableDriver();
        Serial.println("ABORT,no_encoder_motion,driver=DISABLED");
        return false;
      }
    } else {
      stepsWithoutMotion = 0;
    }
  }

  if ((uint32_t)labs(legDelta) < targetRaw) {
    disableDriver();
    Serial.print("ABORT,step_limit,leg=");
    Serial.print(legName);
    Serial.println(",driver=DISABLED");
    return false;
  }

  *measuredDelta = legDelta;
  Serial.print("LEG_COMPLETE,name=");
  Serial.print(legName);
  Serial.print(",steps=");
  Serial.print(steps);
  Serial.print(",motor_deg=");
  Serial.print(legDelta * (360.0f / 4096.0f), 2);
  Serial.print(",output_deg=");
  Serial.print(legDelta * (360.0f / (4096.0f * 15.0f)), 2);
  Serial.print(",switch=");
  Serial.print(switchName());
  Serial.print(",raw_end=");
  Serial.println(rawAngle);
  delay(300);
  return true;
}

void runCycle() {
  uint16_t rawAngle = 0;
  uint8_t status = 0;
  if (!readEncoder(&rawAngle, &status) || !magnetFieldIsValid(status)) {
    Serial.println("ABORT,encoder_or_magnet_fault,driver=DISABLED");
    return;
  }

  digitalWrite(PIN_EN, DRIVER_ENABLED);
  driverEnabled = true;
  Serial.println("CYCLE_START,output_target_deg=180,return_to_start=yes");

  int32_t outboundDelta = 0;
  if (!runLeg(true, OUTPUT_HALF_TURN_RAW, "OUTBOUND", &outboundDelta)) return;

  int32_t returnDelta = 0;
  if (!runLeg(false, (uint32_t)labs(outboundDelta), "RETURN", &returnDelta)) return;

  disableDriver();
  Serial.print("CYCLE_COMPLETE,outbound_output_deg=");
  Serial.print(outboundDelta * (360.0f / (4096.0f * 15.0f)), 2);
  Serial.print(",return_output_deg=");
  Serial.print(returnDelta * (360.0f / (4096.0f * 15.0f)), 2);
  Serial.println(",driver=DISABLED");
}

void moveOutput(float requestedDegrees) {
  if (!isfinite(requestedDegrees) || fabsf(requestedDegrees) < MIN_MOVE_DEG ||
      fabsf(requestedDegrees) > MAX_MOVE_DEG) {
    Serial.println("ERROR,MOVE_range_is_0.01_to_180_degrees");
    return;
  }

  uint16_t rawAngle = 0;
  uint8_t status = 0;
  if (!readEncoder(&rawAngle, &status) || !magnetFieldIsValid(status)) {
    Serial.println("ABORT,encoder_or_magnet_fault,driver=DISABLED");
    return;
  }

  const uint32_t targetRaw = (uint32_t)(fabsf(requestedDegrees) * 15.0f * 4096.0f / 360.0f + 0.5f);
  const bool forward = requestedDegrees > 0.0f;
  digitalWrite(PIN_EN, DRIVER_ENABLED);
  driverEnabled = true;

  int32_t measuredDelta = 0;
  if (!runLeg(forward, targetRaw, "MOVE", &measuredDelta)) return;

  disableDriver();
  Serial.print("MOVE_COMPLETE,requested_output_deg=");
  Serial.print(requestedDegrees, 3);
  Serial.print(",measured_output_deg=");
  Serial.print(measuredDelta * (360.0f / (4096.0f * 15.0f)), 3);
  Serial.println(",driver=DISABLED");
}

void handleCommand(char *command) {
  size_t length = strlen(command);
  while (length > 0 && (command[length - 1] == '\r' || command[length - 1] == ' ')) {
    command[--length] = '\0';
  }
  if (strncmp(command, "MOVE", 4) == 0 && (command[4] == ' ' || command[4] == '\t')) {
    char *valueText = command + 4;
    while (*valueText == ' ' || *valueText == '\t') valueText++;
    char *end = NULL;
    const float requestedDegrees = (float)strtod(valueText, &end);
    while (*end == ' ' || *end == '\t') end++;
    if (end == valueText || *end != '\0') {
      Serial.println("ERROR,usage: MOVE <signed_degrees>");
      return;
    }
    moveOutput(requestedDegrees);
  } else if (strcmp(command, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(command, "HELP") == 0) {
    printHelp();
  } else if (strcmp(command, "RUN") == 0) {
    runCycle();
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