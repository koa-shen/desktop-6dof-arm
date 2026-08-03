// app_06_multi_joint - three joints, one binary, joint-node architecture.
//
// This is the same code that will later run one-joint-per-MCU over CAN. The
// only thing that changes is where the JointCommand comes from: today a serial
// command writes it directly, later a CAN frame lands in the same struct.
//
// Commands:
//   g <j> <deg>   move joint j (1-3) to an absolute output angle
//   e / x         enable / disable all drivers
//   z             zero every encoder here
//   c             clear latched faults
//   l             toggle CSV telemetry
//   ?             status
#include <Arduino.h>
#include <Wire.h>

#include "AS5600Encoder.h"
#include "JointController.h"
#include "JointProtocol.h"
#include "SerialCli.h"
#include "StepperDriver.h"
#include "TCA9548A.h"
#include "joint_config.h"
#include "pins.h"

using namespace jointnode;

constexpr uint8_t kNumJoints = 3;

TCA9548A mux(TCA9548A_ADDR, Wire);

AS5600Encoder enc[kNumJoints] = {
    AS5600Encoder(&mux, MUX_CH_J1, AS5600_ADDR, Wire),
    AS5600Encoder(&mux, MUX_CH_J2, AS5600_ADDR, Wire),
    AS5600Encoder(&mux, MUX_CH_J3, AS5600_ADDR, Wire),
};

StepperDriver drv[kNumJoints] = {
    StepperDriver(PIN_J1_STEP, PIN_J1_DIR, PIN_DRIVER_EN, DRIVER_EN_ACTIVE_LOW),
    StepperDriver(PIN_J2_STEP, PIN_J2_DIR, PIN_DRIVER_EN, DRIVER_EN_ACTIVE_LOW),
    StepperDriver(PIN_J3_STEP, PIN_J3_DIR, PIN_DRIVER_EN, DRIVER_EN_ACTIVE_LOW),
};

JointController joint[kNumJoints] = {
    JointController(&enc[0], &drv[0], 0),
    JointController(&enc[1], &drv[1], 1),
    JointController(&enc[2], &drv[2], 2),
};

JointCommand cmd[kNumJoints];
SerialCli<32> cli;

uint32_t lastCtrlUs = 0;
uint32_t lastLogMs = 0;
bool logging = true;

static void printStatus() {
  for (uint8_t i = 0; i < kNumJoints; ++i) {
    const JointState& s = joint[i].state();
    Serial.print(F("# J"));
    Serial.print(i + 1);
    Serial.print(F(" pos="));
    Serial.print(countsToDeg(s.pos_counts), 2);
    Serial.print(F("deg fault=0x"));
    Serial.print(s.fault, HEX);
    Serial.print(F(" magnet="));
    Serial.println(enc[i].magnetText());
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  mux.begin();

  JointController::Config cfg;
  cfg.kp = PID_KP;
  cfg.ki = PID_KI;
  cfg.kd = PID_KD;
  cfg.steps_per_count =
      (MOTOR_FULL_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO) / (float)kCountsPerRev;
  cfg.min_counts = degToCounts(JOINT_MIN_DEG);
  cfg.max_counts = degToCounts(JOINT_MAX_DEG);
  cfg.default_max_vel = (uint16_t)(MAX_SPEED_STEPS_PER_SEC / cfg.steps_per_count);
  cfg.following_err_max = degToCounts(15.0f);
  cfg.max_step_rate = MAX_SPEED_STEPS_PER_SEC;
  cfg.vel_ff_scale = VEL_FF_SCALE;
  cfg.integral_limit = PID_INTEGRAL_LIMIT;
  cfg.in_position_counts = POSITION_DEADBAND_COUNTS;
  cfg.limit_margin_counts = SOFT_LIMIT_MARGIN_COUNTS;
  cfg.absolute_home = ABSOLUTE_HOME;

  for (uint8_t i = 0; i < kNumJoints; ++i) {
    enc[i].begin();
    drv[i].begin();
    drv[i].setAcceleration(ACCEL_STEPS_PER_SEC2);
    cfg.home_offset_deg = JOINT_HOME_OFFSET_DEG[i];
    joint[i].begin(cfg);
    cmd[i].mode = MODE_HOLD;
    cmd[i].flags = 0;
    cmd[i].max_vel = 0;
  }

  Serial.println(F("# app_06_multi_joint"));
  Serial.println(F("# g <j> <deg> | e | x | z | c | l | ?"));
  Serial.println(F("ms,j1_cmd,j1_pos,j2_cmd,j2_pos,j3_cmd,j3_pos,fault"));
  lastCtrlUs = micros();
}

void loop() {
  for (uint8_t i = 0; i < kNumJoints; ++i) drv[i].run();

  const char* line = cli.poll(Serial);
  if (line) {
    switch (line[0]) {
      case 'g': {
        char* p = nullptr;
        long j = strtol(line + 1, &p, 10);
        float deg = strtod(p, nullptr);  // avr-libc sscanf has no %f
        if (j >= 1 && j <= kNumJoints) {
          cmd[j - 1].mode = MODE_POSITION;
          cmd[j - 1].target_counts = degToCounts(deg);
        }
        break;
      }
      case 'e':
        for (uint8_t i = 0; i < kNumJoints; ++i) cmd[i].flags |= CMD_ENABLE;
        break;
      case 'x':
        for (uint8_t i = 0; i < kNumJoints; ++i) cmd[i].flags &= ~CMD_ENABLE;
        break;
      case 'z':
        for (uint8_t i = 0; i < kNumJoints; ++i) cmd[i].flags |= CMD_ZERO_HERE;
        break;
      case 'c':
        for (uint8_t i = 0; i < kNumJoints; ++i) cmd[i].flags |= CMD_CLEAR_FAULT;
        break;
      case 'l':
        logging = !logging;
        break;
      case '?':
        printStatus();
        break;
      default:
        break;
    }
  }

  const uint32_t nowUs = micros();
  if (nowUs - lastCtrlUs >= (uint32_t)CONTROL_PERIOD_MS * 1000UL) {
    const float dt = (nowUs - lastCtrlUs) * 1e-6f;
    lastCtrlUs = nowUs;
    for (uint8_t i = 0; i < kNumJoints; ++i) {
      joint[i].setCommand(cmd[i]);
      cmd[i].flags &= ~(CMD_ZERO_HERE | CMD_CLEAR_FAULT);  // one-shot bits
      joint[i].update(dt);
    }
  }

  if (logging && millis() - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = millis();
    uint8_t faults = 0;
    Serial.print(lastLogMs);
    for (uint8_t i = 0; i < kNumJoints; ++i) {
      Serial.print(',');
      Serial.print(countsToDeg(cmd[i].target_counts), 2);
      Serial.print(',');
      Serial.print(countsToDeg(joint[i].state().pos_counts), 2);
      faults |= joint[i].state().fault;
    }
    Serial.print(',');
    Serial.println(faults);
  }
}
