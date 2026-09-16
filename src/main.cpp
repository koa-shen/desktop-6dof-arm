#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>

// -------------------- Pin config (EDIT THESE) --------------------
const uint8_t PIN_STEP = 2;
const uint8_t PIN_DIR  = 3;
const uint8_t PIN_EN   = 4;
const uint8_t PIN_HX711_DOUT = 5;
const uint8_t PIN_HX711_SCK  = 6;
const uint8_t PIN_CONTACT    = 7; // Omron V-156-1C25 NO contact: LOW = touching, HIGH = separated

// -------------------- I2C addresses --------------------
// Single motor for now: AS5600 wired directly to the Uno's I2C bus (A4/A5),
// no mux. Re-add the TCA9548A when testing multiple steppers.
const uint8_t AS5600_ADDR = 0x36;

// -------------------- Drivetrain config --------------------
const uint16_t MOTOR_STEPS_PER_REV = 200;  // 1.8 deg/step, no microstepping
const uint16_t MICROSTEPS = 1;             // set to match TMC2209 MS pin config
const float GEAR_RATIO = 15.0f;            // cycloidal reducer ratio
const uint16_t STEP_HIGH_US = 1200;
const uint16_t STEP_LOW_US  = 1200;

// -------------------- Phase 2: reducer characterization --------------------
// Sweeps the output through a range, logging commanded vs measured angle in
// both directions so backlash/hysteresis and transmission error can be
// computed from the CSV afterward.
const uint16_t STEP_INCREMENT = 20;   // motor microsteps per logged sample
const uint8_t  SWEEP_CYCLES = 3;      // repeat count for repeatability stats
const float SWEEP_RANGE_OUTPUT_DEG = 60.0f; // output-shaft degrees per sweep

// -------------------- Load-cell + contact-switch backlash test --------------------
// Load cell: 5kg kitchen-scale cell (~49N max) via HX711. Preload is only a
// few percent of that range, so calibrate with a small known weight near
// PRELOAD_FORCE_N (not the full 5kg), and measure the at-rest noise floor
// (tare, log with nothing touching) before trusting FORCE_ZERO_THRESHOLD_N.
const bool  RUN_BACKLASH_TEST = false; // set true once HX711 + contact switch are wired (blocks/hangs otherwise)
const float HX711_CALIBRATION_FACTOR = 1.0f; // TODO: calibrate against a small known weight
const float PRELOAD_FORCE_N = 1.5f;
const float FORCE_ZERO_THRESHOLD_N = 0.05f; // TODO: set from measured at-rest noise, not a guess
const uint8_t  FORCE_SAMPLE_AVG = 8; // kitchen-scale cells are noisier; average more samples
const uint16_t BACKLASH_STEP_INCREMENT = 4;  // motor microsteps per reversal sample
const uint16_t BACKLASH_SETTLE_MS = 30;
const uint16_t BACKLASH_MAX_STEPS = 4000; // safety cap per phase
const uint8_t  CONTACT_DEBOUNCE_SAMPLES = 3; // consecutive open reads to confirm t2 (V-156-1C25 bounces on release)

HX711 g_scale;
long g_stepPos = 0; // signed motor microstep count since boot

// Reads the AS5600 filtered ANGLE register (0x0E/0x0F), 12-bit, 0-4095.
uint16_t readAS5600Raw() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E);
  Wire.endTransmission(false);
  Wire.requestFrom((int)AS5600_ADDR, 2);
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  return ((hi << 8) | lo) & 0x0FFF;
}

float readMotorAngleDeg() {
  return readAS5600Raw() * 360.0f / 4096.0f;
}

float commandedOutputDeg() {
  float motorRevs = (float)g_stepPos / (float)(MOTOR_STEPS_PER_REV * MICROSTEPS);
  return (motorRevs / GEAR_RATIO) * 360.0f;
}

void logSample() {
  Serial.print(millis());
  Serial.print(',');
  Serial.print(g_stepPos);
  Serial.print(',');
  Serial.print(commandedOutputDeg(), 3);
  Serial.print(',');
  Serial.println(readMotorAngleDeg(), 3);
}

void pulseStepOnce(bool dirCW) {
  digitalWrite(PIN_DIR, dirCW ? HIGH : LOW);
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(STEP_HIGH_US);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(STEP_LOW_US);
  g_stepPos += dirCW ? 1 : -1;
}

// Steps in one direction, logging a sample every STEP_INCREMENT microsteps.
void sweep(bool dirCW, uint16_t totalSteps) {
  for (uint16_t i = 0; i < totalSteps; i++) {
    pulseStepOnce(dirCW);
    if (i % STEP_INCREMENT == 0) {
      logSample();
    }
  }
  logSample(); // capture the endpoint
}

void logBacklashSample(float forceN, bool contactClosed) {
  Serial.print(millis());
  Serial.print(',');
  Serial.print(g_stepPos);
  Serial.print(',');
  Serial.print(readMotorAngleDeg(), 3);
  Serial.print(',');
  Serial.print(forceN, 3);
  Serial.print(',');
  Serial.println(contactClosed ? 1 : 0);
}

// Drives forward until preload is established against the load cell, then
// reverses in small increments to find force-zero (t1) and true mechanical
// separation (t2). backlash_output_deg = (motor_deg[t2] - motor_deg[t1]) / GEAR_RATIO.
// Assumes the motor rotation between t1 and t2 is small (no 0/360 wraparound handling).
void runLoadCellBacklashTest() {
  Serial.println("backlash_test_start");
  Serial.println("ms,step_pos,motor_deg,force_n,contact");

  // Phase 1: drive forward (toward the load cell) until preload is reached.
  for (uint16_t i = 0; i < BACKLASH_MAX_STEPS; i++) {
    float forceN = g_scale.get_units(FORCE_SAMPLE_AVG);
    bool contactClosed = digitalRead(PIN_CONTACT) == LOW;
    logBacklashSample(forceN, contactClosed);
    if (forceN >= PRELOAD_FORCE_N) break;
    pulseStepOnce(true);
  }

  // Phase 2: reverse in small increments, watching for force->0 (t1) then
  // contact-open (t2). t2 requires CONTACT_DEBOUNCE_SAMPLES consecutive open
  // reads so switch bounce on release doesn't trigger a false separation.
  bool foundT1 = false;
  float motorDegAtT1 = 0.0f;
  float motorDegAtT2Candidate = 0.0f;
  uint8_t openStreak = 0;
  for (uint16_t i = 0; i < BACKLASH_MAX_STEPS; i++) {
    for (uint16_t s = 0; s < BACKLASH_STEP_INCREMENT; s++) {
      pulseStepOnce(false);
    }
    delay(BACKLASH_SETTLE_MS);

    float forceN = g_scale.get_units(FORCE_SAMPLE_AVG);
    bool contactClosed = digitalRead(PIN_CONTACT) == LOW;
    logBacklashSample(forceN, contactClosed);

    if (!foundT1 && forceN <= FORCE_ZERO_THRESHOLD_N) {
      foundT1 = true;
      motorDegAtT1 = readMotorAngleDeg();
    }
    if (!foundT1) continue;

    if (!contactClosed) {
      if (openStreak == 0) motorDegAtT2Candidate = readMotorAngleDeg();
      openStreak++;
    } else {
      openStreak = 0; // bounced back closed; discard candidate
    }

    if (openStreak >= CONTACT_DEBOUNCE_SAMPLES) {
      float backlashOutputDeg = (motorDegAtT2Candidate - motorDegAtT1) / GEAR_RATIO;
      Serial.print("backlash_output_deg,");
      Serial.println(backlashOutputDeg, 4);
      Serial.print("backlash_output_arcmin,");
      Serial.println(backlashOutputDeg * 60.0f, 2);
      break;
    }
  }

  Serial.println("backlash_test_done");
}

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_CONTACT, INPUT_PULLUP);
  digitalWrite(PIN_EN, LOW); // driver enabled (active LOW on most TMC2209 boards)

  Wire.begin();
  Serial.begin(115200);
  while (!Serial) {}

  g_scale.begin(PIN_HX711_DOUT, PIN_HX711_SCK);
  g_scale.set_scale(HX711_CALIBRATION_FACTOR);
  g_scale.tare();

  Serial.println("reducer_characterization_start");
  Serial.println("ms,step_pos,commanded_deg,measured_deg");

  uint16_t sweepSteps = (uint16_t)((SWEEP_RANGE_OUTPUT_DEG / 360.0f) * GEAR_RATIO
                                    * MOTOR_STEPS_PER_REV * MICROSTEPS);

  for (uint8_t cycle = 0; cycle < SWEEP_CYCLES; cycle++) {
    sweep(true, sweepSteps);   // forward sweep (captures loaded-direction error)
    delay(500);
    sweep(false, sweepSteps);  // return sweep (captures backlash/hysteresis loop)
    delay(500);
  }

  digitalWrite(PIN_EN, HIGH); // de-energize when done; shaft free for manual checks
  Serial.println("reducer_characterization_done");

  if (RUN_BACKLASH_TEST) {
    digitalWrite(PIN_EN, LOW); // re-enable driver for the backlash test
    runLoadCellBacklashTest();
    digitalWrite(PIN_EN, HIGH);
  }
}

void loop() {
  // Sweep sequence runs once in setup(); idle here.
}
