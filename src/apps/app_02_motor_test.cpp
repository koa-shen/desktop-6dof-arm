// App 02 - Motor only, open loop
//
// Use this while setting TMC2209 current (Vref). No encoder involved, so a
// failure here is purely a driver/motor/power problem.
//
//   pio run -e motor_test -t upload && pio device monitor
//
// Serial commands (type + Enter):
//   e        toggle driver enable       +/-  speed up / down
//   f/r      tiny forward/reverse jog
//   <n>      jog n steps (e.g. 400, -400)
//   x        stop                       ?    status
//   c/v      continuous forward/reverse
//
// PASS: smooth rotation, no skipped steps at target speed, driver warm but not
// hot, motor holds position when stopped and enabled.
//
// SAFETY: never unplug motor leads while VMOT is live - it kills the driver.

#include <Arduino.h>

#include "SerialCli.h"
#include "StepperDriver.h"
#include "joint_config.h"
#include "pins.h"

static StepperDriver motor(PIN_J1_STEP, PIN_J1_DIR, PIN_J1_EN,
                           DRIVER_EN_ACTIVE_LOW);
static SerialCli<24> cli;

static float speed = 400.0f;  // steps/s
static bool continuous = false;
static int8_t continuousDir = 1;
static uint32_t lastLogMs = 0;

static void jogSteps(long steps) {
  continuous = false;
  if (!motor.isEnabled()) motor.enable(true);
  motor.move(steps);
  Serial.print(F("# jog "));
  Serial.println(steps);
}

static void runContinuous(int8_t direction) {
  continuous = true;
  continuousDir = direction;
  if (!motor.isEnabled()) motor.enable(true);
  motor.setVelocity(speed * continuousDir);
  Serial.println(direction > 0 ? F("# continuous forward")
                               : F("# continuous reverse"));
}

static void printStatus() {
  Serial.print(F("# en="));
  Serial.print(motor.isEnabled());
  Serial.print(F(" pos="));
  Serial.print(motor.position());
  Serial.print(F(" deg="));
  Serial.print(motor.position() / STEPS_PER_OUTPUT_DEG, 2);
  Serial.print(F(" vel="));
  Serial.print(motor.velocity(), 1);
  Serial.print(F(" speed_set="));
  Serial.print(speed, 1);
  Serial.print(F(" rpm="));
  Serial.println(speed * 60.0f / STEPS_PER_MOTOR_REV, 1);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
  }
  motor.begin();
  motor.setMaxSpeed(speed);
  motor.setAcceleration(ACCEL_STEPS_PER_SEC2);

  Serial.println(F("\n=== app_02_motor_test ==="));
  Serial.print(F("# "));
  Serial.print(STEPS_PER_OUTPUT_DEG, 3);
  Serial.println(F(" steps per output degree"));
  Serial.println(F("# driver starts DISABLED. type 'e' to enable."));
  Serial.println(F("# keys: e=enable f/r=tiny jog +/-=speed"));
  Serial.println(F("#       c/v=continuous fwd/rev x=stop ?=status"));
  Serial.println(F("# or type a step count: 400 / -400"));
}

void loop() {
  const char *line = cli.poll();
  if (line && line[0]) {
    if (line[0] == 'e') {
      motor.enable(!motor.isEnabled());
      Serial.println(motor.isEnabled() ? F("# ENABLED") : F("# disabled"));
    } else if (line[0] == 'f') {
      jogSteps(25);
    } else if (line[0] == 'r') {
      jogSteps(-25);
    } else if (line[0] == '+' && line[1] == '\0') {
      speed = min(speed * 1.5f, MAX_SPEED_STEPS_PER_SEC);
      motor.setMaxSpeed(speed);
      printStatus();
    } else if (line[0] == '-' && line[1] == '\0') {
      speed = max(speed / 1.5f, 10.0f);
      motor.setMaxSpeed(speed);
      printStatus();
    } else if (line[0] == 'x') {
      continuous = false;
      motor.emergencyStop();
      Serial.println(F("# stopped"));
    } else if (line[0] == 'c') {
      runContinuous(1);
    } else if (line[0] == 'v') {
      runContinuous(-1);
    } else if (line[0] == '?') {
      printStatus();
    } else {
      const long steps = atol(line);
      if (steps != 0) {
        jogSteps(steps);
      }
    }
  }

  if (continuous) {
    if (!motor.isEnabled()) motor.enable(true);
    motor.setVelocity(speed * continuousDir);
  }

  motor.run();

  if (motor.isMoving() && millis() - lastLogMs >= 250) {
    lastLogMs = millis();
    printStatus();
  }
}
