// App 05 - Closed-loop position control
//
// The encoder now closes the loop: PID turns angle error into a step-rate
// command. This is the first app where the joint actually knows where it is.
//
//   pio run -e closed_loop -t upload && pio device monitor
//
// Serial commands:
//   g <deg>            go to absolute joint angle
//   j <deg>            jog relative
//   k <kp> <ki> <kd>   set gains live (tune without reflashing)
//   h                  home: define here as 0 deg
//   e                  enable/disable driver
//   x                  stop
//   ?                  status
//
// Gains are entered and printed in the SAME units as joint_config.h (1/s, the
// counts domain) and converted internally to this app's degrees domain. A
// number you find by tuning here can then be pasted straight into the config
// instead of being silently off by steps-per-degree.
//
// CSV: ms,target_deg,enc_deg,err_deg,cmd_vel,steps
// Capture a step response with scripts/serial_logger.py and read overshoot /
// settling time out of scripts/analyze_log.py. That plot is portfolio material.

#include <Arduino.h>
#include <Wire.h>

#include "AS5600Encoder.h"
#include "ArmMath.h"
#include "PidController.h"
#include "SerialCli.h"
#include "StepperDriver.h"
#include "TCA9548A.h"
#include "joint_config.h"
#include "pins.h"

static TCA9548A mux(TCA9548A_ADDR);
static AS5600Encoder encoder(&mux, MUX_CH_J1);
static StepperDriver motor(PIN_J1_STEP, PIN_J1_DIR, PIN_J1_EN,
                           DRIVER_EN_ACTIVE_LOW);

// Gains live in the counts domain (1/s). This app works in degrees, and one
// degree of error is STEPS_PER_OUTPUT_DEG steps of correction, so all three
// gains scale by the same single factor.
static constexpr float kGainScale = STEPS_PER_OUTPUT_DEG;
static constexpr float kDeadbandDeg =
    POSITION_DEADBAND_COUNTS * armmath::kDegPerCount;

static PidController pid(PID_KP * kGainScale, PID_KI * kGainScale,
                         PID_KD * kGainScale);
static SerialCli<40> cli;

static float targetDeg = 0.0f;
static bool holdEnabled = false;
static uint32_t lastControlMs = 0;
static uint32_t lastLogMs = 0;
static uint32_t moveStartMs = 0;
static bool settled = true;

static void setTarget(float deg) {
  targetDeg = armmath::clampf(deg, JOINT_MIN_DEG, JOINT_MAX_DEG);
  pid.reset();
  moveStartMs = millis();
  settled = false;
  Serial.print(F("# target="));
  Serial.println(targetDeg, 2);
}

static void printStatus() {
  encoder.readStatus();
  Serial.print(F("# en="));
  Serial.print(holdEnabled);
  Serial.print(F(" target="));
  Serial.print(targetDeg, 2);
  Serial.print(F(" enc="));
  Serial.print(encoder.angleDeg(), 2);
  Serial.print(F(" err="));
  Serial.print(targetDeg - encoder.angleDeg(), 3);
  Serial.print(F(" kp="));
  Serial.print(pid.kp() / kGainScale, 3);
  Serial.print(F(" ki="));
  Serial.print(pid.ki() / kGainScale, 3);
  Serial.print(F(" kd="));
  Serial.print(pid.kd() / kGainScale, 3);
  Serial.print(F(" magnet="));
  Serial.print(encoder.magnetText());
  Serial.print(F(" i2c_err="));
  Serial.println(encoder.errorCount());
}

static void handleCommand(const char *line) {
  switch (line[0]) {
    case 'g':
      setTarget(atof(line + 1));
      break;
    case 'j':
      setTarget(targetDeg + atof(line + 1));
      break;
    case 'k': {
      // avr-libc's sscanf() has no %f by default, so parse by hand.
      char *p = const_cast<char *>(line) + 1;
      const float kp = strtod(p, &p);
      const float ki = strtod(p, &p);
      const float kd = strtod(p, &p);
      pid.setGains(kp * kGainScale, ki * kGainScale, kd * kGainScale);
      pid.reset();
      printStatus();
      break;
    }
    case 'h':
      encoder.zeroHere();
      motor.setPosition(0);
      targetDeg = 0.0f;
      pid.reset();
      Serial.println(F("# homed: here is 0 deg"));
      break;
    case 'e':
      holdEnabled = !holdEnabled;
      motor.enable(holdEnabled);
      pid.reset();
      Serial.println(holdEnabled ? F("# ENABLED") : F("# disabled"));
      break;
    case 'x':
      targetDeg = encoder.angleDeg();
      motor.emergencyStop();
      pid.reset();
      Serial.println(F("# stopped, holding here"));
      break;
    case '?':
      printStatus();
      break;
    default:
      Serial.println(F("# g<deg> j<deg> k<kp ki kd> h e x ?"));
      break;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
  }
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println(F("\n=== app_05_closed_loop ==="));
  if (!mux.begin() || !encoder.begin()) {
    Serial.println(F("FATAL: encoder not readable - run app_00_i2c_scan"));
  }
  encoder.setDirection(ENCODER_DIRECTION);
  encoder.setFilterAlpha(0.4f);
  encoder.zeroHere();

  motor.begin();
  motor.setMaxSpeed(MAX_SPEED_STEPS_PER_SEC);
  motor.setAcceleration(ACCEL_STEPS_PER_SEC2);
  motor.setPosition(0);

  pid.setOutputLimit(MAX_SPEED_STEPS_PER_SEC);
  pid.setIntegralLimit(PID_INTEGRAL_LIMIT * armmath::kDegPerCount);

  Serial.println(F("# type 'h' to home, 'e' to enable, then 'g 30'"));
  Serial.println(F("ms,target_deg,enc_deg,err_deg,cmd_vel,steps"));
}

void loop() {
  const char *line = cli.poll();
  if (line && line[0]) handleCommand(line);

  encoder.read();
  motor.run();

  const uint32_t now = millis();

  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    const float dt = (now - lastControlMs) * 0.001f;
    lastControlMs = now;

    const float measured = encoder.angleDeg();
    const float error = targetDeg - measured;

    // Fault check: a dropped magnet must not run the motor away.
    if (!encoder.magnetDetected() && encoder.errorCount() > 20) {
      motor.emergencyStop();
      motor.enable(false);
      holdEnabled = false;
      Serial.println(F("# FAULT: encoder lost, driver disabled"));
    }

    if (holdEnabled) {
      // Deadband first: inside one encoder LSB there is no error worth
      // chasing, only quantization, and chasing it is what makes a stepper
      // hunt. Freeze the integrator rather than letting it wind on noise.
      const float active = armmath::deadband(error, kDeadbandDeg);
      if (active == 0.0f) {
        pid.trackMeasurement(measured);
        motor.setVelocity(0.0f);
      } else {
        motor.setVelocity(pid.update(active, measured, dt));
      }
    }

    if (!settled && fabsf(error) <= POSITION_TOLERANCE_DEG) {
      settled = true;
      Serial.print(F("# settled in "));
      Serial.print(now - moveStartMs);
      Serial.print(F(" ms, err="));
      Serial.println(error, 3);
    }
  }

  if (now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;
    Serial.print(now);
    Serial.print(',');
    Serial.print(targetDeg, 3);
    Serial.print(',');
    Serial.print(encoder.angleDeg(), 3);
    Serial.print(',');
    Serial.print(targetDeg - encoder.angleDeg(), 3);
    Serial.print(',');
    Serial.print(motor.velocity(), 1);
    Serial.print(',');
    Serial.println(motor.position());
  }
}
