// App 03 - Phase 1 bench validation (the Phase 1 deliverable)
//
// Non-blocking motion + continuous encoder streaming. Motor sweeps back and
// forth between two step targets while CSV telemetry goes out the serial port.
//
//   pio run -e phase1_bench -t upload
//   python scripts/serial_logger.py --port COM3 --name phase1
//   python scripts/analyze_log.py docs/test-results/phase1_*.csv
//
// CSV columns: ms,cycle,cmd_steps,cmd_deg,enc_deg,err_deg,i2c_err
// Every completed out-and-back prints a "# cycle" summary with the return-to-
// home error - that number IS your repeatability result.
//
// Serial commands:  s=start/stop  z=zero here  +/-=sweep size  ?=status

#include <Arduino.h>
#include <Wire.h>

#include "AS5600Encoder.h"
#include "SerialCli.h"
#include "StepperDriver.h"
#include "TCA9548A.h"
#include "joint_config.h"
#include "pins.h"

static TCA9548A mux(TCA9548A_ADDR);
static AS5600Encoder encoder(&mux, MUX_CH_J1);
static StepperDriver motor(PIN_J1_STEP, PIN_J1_DIR, PIN_J1_EN,
                           DRIVER_EN_ACTIVE_LOW);
static SerialCli<16> cli;

static float sweepDeg = 45.0f;
static bool running = false;
static bool goingOut = true;
static uint16_t cycle = 0;
static uint32_t dwellUntilMs = 0;
static uint32_t lastLogMs = 0;
static float peakErrDeg = 0.0f;

static long sweepSteps() { return (long)(sweepDeg * STEPS_PER_OUTPUT_DEG); }
static float cmdDeg() { return motor.position() / STEPS_PER_OUTPUT_DEG; }

static void printHeader() {
  Serial.println(F("ms,cycle,cmd_steps,cmd_deg,enc_deg,err_deg,i2c_err"));
}

static void logRow() {
  const float enc = encoder.angleDeg();
  const float err = cmdDeg() - enc;
  if (fabsf(err) > peakErrDeg) peakErrDeg = fabsf(err);

  Serial.print(millis());
  Serial.print(',');
  Serial.print(cycle);
  Serial.print(',');
  Serial.print(motor.position());
  Serial.print(',');
  Serial.print(cmdDeg(), 3);
  Serial.print(',');
  Serial.print(enc, 3);
  Serial.print(',');
  Serial.print(err, 3);
  Serial.print(',');
  Serial.println(encoder.errorCount());
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
  }
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println(F("\n=== app_03_phase1_bench ==="));

  if (!mux.begin() || !mux.isPresent()) {
    Serial.println(F("FATAL: TCA9548A missing - run app_00_i2c_scan"));
  }
  encoder.setDirection(ENCODER_DIRECTION);
  encoder.setFilterAlpha(0.4f);
  if (!encoder.begin()) {
    Serial.println(F("FATAL: AS5600 missing - run app_00_i2c_scan"));
  } else {
    encoder.readStatus();
    Serial.print(F("# magnet="));
    Serial.println(encoder.magnetText());
    encoder.zeroHere();
  }

  motor.begin();
  motor.setMaxSpeed(MAX_SPEED_STEPS_PER_SEC);
  motor.setAcceleration(ACCEL_STEPS_PER_SEC2);
  motor.setPosition(0);

  Serial.print(F("# steps_per_output_deg="));
  Serial.println(STEPS_PER_OUTPUT_DEG, 4);
  Serial.println(F("# keys: s=start/stop z=zero +/-=sweep ?=status"));
  Serial.println(F("# driver disabled until you press 's'"));
  printHeader();
}

void loop() {
  const char *line = cli.poll();
  if (line && line[0]) {
    switch (line[0]) {
      case 's':
        running = !running;
        motor.enable(running);
        if (running) {
          cycle = 0;
          peakErrDeg = 0.0f;
          goingOut = true;
          dwellUntilMs = 0;
          motor.moveTo(sweepSteps());
        } else {
          motor.emergencyStop();
        }
        Serial.println(running ? F("# RUN") : F("# stop"));
        break;
      case 'z':
        encoder.zeroHere();
        motor.setPosition(0);
        Serial.println(F("# zeroed (encoder + commanded)"));
        break;
      case '+':
        sweepDeg = min(sweepDeg + 15.0f, JOINT_MAX_DEG);
        Serial.print(F("# sweep_deg="));
        Serial.println(sweepDeg, 1);
        break;
      case '-':
        sweepDeg = max(sweepDeg - 15.0f, 5.0f);
        Serial.print(F("# sweep_deg="));
        Serial.println(sweepDeg, 1);
        break;
      case '?':
        encoder.readStatus();
        Serial.print(F("# magnet="));
        Serial.print(encoder.magnetText());
        Serial.print(F(" agc="));
        Serial.print(encoder.agc());
        Serial.print(F(" i2c_err="));
        Serial.print(encoder.errorCount());
        Serial.print(F(" peak_err_deg="));
        Serial.println(peakErrDeg, 3);
        break;
      default:
        break;
    }
  }

  motor.run();      // must be called as often as possible
  encoder.read();   // ~1 ms at 100 kHz; interleaves fine with stepping

  if (running && !motor.isMoving() && millis() >= dwellUntilMs) {
    if (dwellUntilMs == 0) {
      dwellUntilMs = millis() + 400;  // settle before judging the endpoint
    } else {
      if (!goingOut) {
        cycle++;
        Serial.print(F("# cycle "));
        Serial.print(cycle);
        Serial.print(F(" home_err_deg="));
        Serial.print(encoder.angleDeg(), 3);
        Serial.print(F(" peak_err_deg="));
        Serial.print(peakErrDeg, 3);
        Serial.print(F(" i2c_err="));
        Serial.println(encoder.errorCount());
        peakErrDeg = 0.0f;
      }
      goingOut = !goingOut;
      motor.moveTo(goingOut ? sweepSteps() : 0);
      dwellUntilMs = 0;
    }
  }

  if (millis() - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = millis();
    logRow();
  }
}
