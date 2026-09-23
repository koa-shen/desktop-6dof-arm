#include <Arduino.h>
#include <Wire.h>

const uint8_t PIN_STEP = 2;
const uint8_t PIN_DIR = 3;
const uint8_t PIN_EN = 4;
const uint8_t PIN_SWITCH = 7;
const uint8_t DRIVER_ENABLED = LOW;
const uint8_t SWITCH_TOUCHING = LOW;

const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_STATUS_REGISTER = 0x0B;
const uint8_t AS5600_ANGLE_REGISTER = 0x0E;

const float GEAR_RATIO = 15.0f;
const float DEG_PER_RAW = 360.0f / 4096.0f;

// 800 pps keeps wide margin under the ~2000 pps tracking-error knee measured at
// VREF 1.25 V in docs/test-results/speed-sweep-20260918-091555Z.csv.
const uint16_t SEEK_STEP_RATE = 800;
const uint8_t INCREMENT_STEPS = 2;
const uint8_t SETTLE_MS = 15;
const uint8_t DEBOUNCE_SAMPLES = 4;
const uint8_t DEBOUNCE_SAMPLE_MS = 2;
const uint16_t SEEK_MAX_INCREMENTS = 4000;
const uint16_t OVERTRAVEL_STEPS = 60;
const uint16_t BACKOFF_STEPS = 600;
const uint8_t DEADBAND_CYCLES = 6;
const uint8_t CONTROL_CYCLES = 3;

const bool DIR_TOWARD = true;
const bool DIR_AWAY = false;

int32_t accumulatedRaw = 0;
uint16_t previousRaw = 0;
int32_t stepsFromHome = 0;
uint8_t lastStatus = 0;

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

// Maintains a continuous multi-turn motor angle; increments are small enough
// that a wrapped delta can never alias past half a revolution.
bool updateAngle() {
  uint16_t rawAngle;
  uint8_t status;
  if (!readEncoder(&rawAngle, &status) || !magnetFieldIsValid(status)) return false;
  lastStatus = status;
  accumulatedRaw += wrappedDelta(rawAngle, previousRaw);
  previousRaw = rawAngle;
  return true;
}

float motorDeg() {
  return accumulatedRaw * DEG_PER_RAW;
}

void pulseStep() {
  const uint16_t halfPeriodUs = 500000UL / SEEK_STEP_RATE;
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(halfPeriodUs);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(halfPeriodUs);
}

void runSteps(bool towardContact, uint16_t stepCount) {
  digitalWrite(PIN_DIR, towardContact == DIR_TOWARD ? HIGH : LOW);
  for (uint16_t step = 0; step < stepCount; step++) {
    pulseStep();
  }
  stepsFromHome += (towardContact == DIR_TOWARD) ? (int32_t)stepCount : -(int32_t)stepCount;
}

bool switchStateIs(uint8_t expectedState) {
  for (uint8_t sample = 0; sample < DEBOUNCE_SAMPLES; sample++) {
    if (digitalRead(PIN_SWITCH) != expectedState) return false;
    delay(DEBOUNCE_SAMPLE_MS);
  }
  return true;
}

// Steps in fixed increments until the debounced switch reaches targetState.
// Returns false on encoder fault or if the travel limit is reached first.
bool seekSwitchState(bool towardContact, uint8_t targetState, float *angleAtEvent) {
  for (uint16_t increment = 0; increment < SEEK_MAX_INCREMENTS; increment++) {
    if (switchStateIs(targetState)) {
      if (!updateAngle()) return false;
      *angleAtEvent = motorDeg();
      return true;
    }
    runSteps(towardContact, INCREMENT_STEPS);
    delay(SETTLE_MS);
    if (!updateAngle()) return false;
  }
  return false;
}

void printEvent(const char *phase, uint8_t cycle, const char *event, float angle) {
  Serial.print("EVENT,");
  Serial.print(phase);
  Serial.print(',');
  Serial.print(cycle);
  Serial.print(',');
  Serial.print(event);
  Serial.print(',');
  Serial.print(angle, 4);
  Serial.print(',');
  Serial.print(stepsFromHome);
  Serial.print(",0x");
  Serial.println(lastStatus, HEX);
}

void abortRun(const char *reason) {
  digitalWrite(PIN_EN, HIGH);
  Serial.print("ABORT,");
  Serial.println(reason);
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
  pinMode(PIN_SWITCH, INPUT_PULLUP);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_EN, HIGH);

  Serial.begin(115200);
  Serial.setTimeout(100);
  Wire.begin();
  delay(500);

  uint8_t status;
  if (!readEncoder(&previousRaw, &status)) {
    Serial.println("ABORT,encoder_not_found");
    return;
  }
  if (!magnetFieldIsValid(status)) {
    Serial.print("ABORT,invalid_magnet_field,0x");
    Serial.println(status, HEX);
    return;
  }
  lastStatus = status;

  Serial.println("READY,backlash_switch_deadband");
  if (!waitForStartCommand()) return;

  Serial.println("RUN_START");
  Serial.println("HEADER,EVENT,phase,cycle,event,motor_deg,steps_from_home,status");
  Serial.println("HEADER,CONTROL,cycle,trip_motor_deg,spread_from_first_deg");
  Serial.println("HEADER,DEADBAND,cycle,delta_motor_deg,output_deg,output_arcmin");

  digitalWrite(PIN_EN, DRIVER_ENABLED);
  delay(50);

  // Start from a known separated state so the first approach is a clean
  // single-direction closure with the backlash already taken up.
  float angle;
  if (digitalRead(PIN_SWITCH) == SWITCH_TOUCHING) {
    if (!seekSwitchState(DIR_AWAY, !SWITCH_TOUCHING, &angle)) {
      abortRun("initial_release_not_found");
      return;
    }
    printEvent("init", 0, "release", angle);
  }
  runSteps(DIR_AWAY, BACKOFF_STEPS);
  delay(SETTLE_MS);
  if (!updateAngle()) {
    abortRun("encoder_fault_init");
    return;
  }
  printEvent("init", 0, "backoff", motorDeg());

  // Control pass: every trip is approached from the same direction with the
  // backlash already taken up, so the spread between trips is the measurement
  // noise floor that the dead-band numbers below have to beat to mean anything.
  float firstControlTrip = 0.0f;
  for (uint8_t cycle = 1; cycle <= CONTROL_CYCLES; cycle++) {
    if (!seekSwitchState(DIR_TOWARD, SWITCH_TOUCHING, &angle)) {
      abortRun("control_trip_not_found");
      return;
    }
    printEvent("control", cycle, "trip", angle);
    if (cycle == 1) firstControlTrip = angle;

    Serial.print("CONTROL,");
    Serial.print(cycle);
    Serial.print(',');
    Serial.print(angle, 4);
    Serial.print(',');
    Serial.println(angle - firstControlTrip, 4);

    if (!seekSwitchState(DIR_AWAY, !SWITCH_TOUCHING, &angle)) {
      abortRun("control_release_not_found");
      return;
    }
    printEvent("control", cycle, "release", angle);

    runSteps(DIR_AWAY, BACKOFF_STEPS);
    delay(SETTLE_MS);
    if (!updateAngle()) {
      abortRun("encoder_fault_control");
      return;
    }
  }

  // Dead-band pass: trip on approach, overtravel to seat the contact, then
  // reverse until release. Subtract the known forward overtravel from the
  // reverse travel before converting the result through the gear ratio.
  for (uint8_t cycle = 1; cycle <= DEADBAND_CYCLES; cycle++) {
    float tripAngle;
    float overtravelAngle;
    float releaseAngle;

    if (!seekSwitchState(DIR_TOWARD, SWITCH_TOUCHING, &tripAngle)) {
      abortRun("deadband_trip_not_found");
      return;
    }
    printEvent("deadband", cycle, "trip", tripAngle);

    runSteps(DIR_TOWARD, OVERTRAVEL_STEPS);
    delay(SETTLE_MS);
    if (!updateAngle()) {
      abortRun("encoder_fault_overtravel");
      return;
    }
    overtravelAngle = motorDeg();
    printEvent("deadband", cycle, "overtravel", overtravelAngle);

    if (!seekSwitchState(DIR_AWAY, !SWITCH_TOUCHING, &releaseAngle)) {
      abortRun("deadband_release_not_found");
      return;
    }
    printEvent("deadband", cycle, "release", releaseAngle);

    float reverseTravelDeg = fabs(overtravelAngle - releaseAngle);
    float overtravelDeg = fabs(overtravelAngle - tripAngle);
    float deltaMotorDeg = reverseTravelDeg - overtravelDeg;
    if (deltaMotorDeg < 0.0f) deltaMotorDeg = 0.0f;
    float outputDeg = deltaMotorDeg / GEAR_RATIO;

    Serial.print("DEADBAND,");
    Serial.print(cycle);
    Serial.print(',');
    Serial.print(deltaMotorDeg, 4);
    Serial.print(',');
    Serial.print(outputDeg, 5);
    Serial.print(',');
    Serial.println(outputDeg * 60.0f, 3);

    runSteps(DIR_AWAY, BACKOFF_STEPS);
    delay(SETTLE_MS);
    if (!updateAngle()) {
      abortRun("encoder_fault_backoff");
      return;
    }
  }

  digitalWrite(PIN_EN, HIGH);
  Serial.println("RUN_COMPLETE");
}

void loop() {
  digitalWrite(PIN_EN, HIGH);
}
