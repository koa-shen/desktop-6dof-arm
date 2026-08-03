// app_07_coordinated - synchronized multi-joint motion plus the gripper.
//
// This is the first app where the arm does something rather than demonstrating
// something. Three joints move along a common trapezoidal profile, all of them
// arriving at the same instant, and the gripper opens and closes between moves.
//
//   pio run -e coordinated -t upload && pio device monitor
//
// The architecture on screen here is D7's layers 1 and 0 in one binary:
//   MotionController  = layer 1: plans, synchronizes, streams setpoints
//   JointController   = layer 0: one joint's servo loop, its own faults
// Splitting them across two MCUs later moves code across a boundary that
// already exists instead of inventing one.
//
// Commands:
//   m <d1> <d2> <d3>   synchronized move to three absolute output angles
//   s <0..1>           speed scale for subsequent moves
//   o / c              gripper open / grip
//   e / x              enable / disable drivers
//   !                  software estop (the hardware line is the real one)
//   z                  zero every encoder here
//   k                  clear latched faults
//   l                  toggle CSV telemetry
//   ?                  status
//
// CSV: ms,j1_set,j1_pos,j2_set,j2_pos,j3_set,j3_pos,grip,fault
#include <Arduino.h>
#include <Wire.h>

#include "AS5600Encoder.h"
#include "ArmMath.h"
#include "JointController.h"
#include "JointProtocol.h"
#include "MotionController.h"
#include "SerialCli.h"
#include "ServoGripper.h"
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

MotionController<kNumJoints, JERK_FILTER_TAPS> motion;
ServoGripper gripper;
SerialCli<40> cli;

uint32_t lastCtrlUs = 0;
uint32_t lastLogMs = 0;
bool logging = true;
float speedScale = 1.0f;
bool reparkPending = false;

static void currentPositions(int32_t* out) {
  for (uint8_t i = 0; i < kNumJoints; ++i) out[i] = joint[i].state().pos_counts;
}

static void printStatus() {
  for (uint8_t i = 0; i < kNumJoints; ++i) {
    const JointState& s = joint[i].state();
    Serial.print(F("# J"));
    Serial.print(i + 1);
    Serial.print(F(" set="));
    Serial.print(countsToDeg(motion.command(i).target_counts), 2);
    Serial.print(F(" pos="));
    Serial.print(countsToDeg(s.pos_counts), 2);
    Serial.print(F(" ff="));
    Serial.print(motion.command(i).velFeedforward());
    Serial.print(F(" fault=0x"));
    Serial.print(s.fault & kFaultMask, HEX);
    Serial.print(F(" inpos="));
    Serial.print(s.inPosition());
    Serial.print(F(" magnet="));
    Serial.println(enc[i].magnetText());
  }
  Serial.print(F("# grip="));
  Serial.print(gripper.angleDeg(), 1);
  Serial.print(F(" settled="));
  Serial.print(gripper.settled());
  Serial.print(F(" moving="));
  Serial.print(motion.moving());
  Serial.print(F(" speed="));
  Serial.println(speedScale, 2);
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
  cfg.absolute_home = ABSOLUTE_HOME;

  for (uint8_t i = 0; i < kNumJoints; ++i) {
    enc[i].begin();
    drv[i].begin();
    drv[i].setAcceleration(ACCEL_STEPS_PER_SEC2);
    cfg.home_offset_deg = JOINT_HOME_OFFSET_DEG[i];
    joint[i].begin(cfg);
  }

  MotionController<kNumJoints, JERK_FILTER_TAPS>::Limits lim;
  lim.max_vel = TRAJ_MAX_VEL_DEG_S / kDegPerCount;
  lim.max_acc = TRAJ_MAX_ACC_DEG_S2 / kDegPerCount;
  motion.begin(lim);

  ServoGripper::Config gcfg;
  gcfg.pin = PIN_GRIPPER_SERVO;
  gcfg.open_deg = GRIPPER_OPEN_DEG;
  gcfg.closed_deg = GRIPPER_CLOSED_DEG;
  gcfg.grip_deg = GRIPPER_GRIP_DEG;
  gcfg.slew_deg_s = GRIPPER_SLEW_DEG_S;
  gcfg.settle_ms = GRIPPER_SETTLE_MS;
  gcfg.detach_idle = GRIPPER_DETACH_IDLE;
  gripper.begin(gcfg);

  // Start the setpoint stream where the arm actually is. Skipping this is how
  // you get a full-speed lunge to zero on the first enable.
  int32_t here[kNumJoints];
  currentPositions(here);
  motion.resetTo(here);

  Serial.println(F("# app_07_coordinated"));
  Serial.println(F("# m <d1> <d2> <d3> | s <scale> | o | c | e | x | ! | z | k | l | ?"));
  Serial.println(F("ms,j1_set,j1_pos,j2_set,j2_pos,j3_set,j3_pos,grip,fault"));
  lastCtrlUs = micros();
}

void loop() {
  for (uint8_t i = 0; i < kNumJoints; ++i) drv[i].run();
  gripper.update();

  const char* line = cli.poll(Serial);
  if (line) {
    switch (line[0]) {
      case 'm': {
        char* p = const_cast<char*>(line) + 1;
        int32_t target[kNumJoints];
        for (uint8_t i = 0; i < kNumJoints; ++i) {
          target[i] = degToCounts((float)strtod(p, &p));  // avr sscanf has no %f
        }
        int32_t here[kNumJoints];
        currentPositions(here);
        Serial.print(F("# move t="));
        Serial.print(motion.moveTo(target, here, speedScale), 3);
        Serial.println(F("s"));
        break;
      }
      case 's':
        speedScale = armmath::clampf((float)atof(line + 1), 0.05f, 1.0f);
        break;
      case 'o':
        gripper.open();
        break;
      case 'c':
        gripper.grip();
        break;
      case 'e':
        motion.setEnabled(true);
        break;
      case 'x':
        motion.setEnabled(false);
        break;
      case '!':
        motion.abort();
        motion.setEnabled(false);
        for (uint8_t i = 0; i < kNumJoints; ++i) joint[i].estop();
        Serial.println(F("# ESTOP"));
        break;
      case 'z': {
        motion.setFlag(CMD_ZERO_HERE, true);
        reparkPending = true;
        break;
      }
      case 'k':
        motion.setFlag(CMD_CLEAR_FAULT, true);
        reparkPending = true;
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

    motion.update(dt);
    for (uint8_t i = 0; i < kNumJoints; ++i) {
      joint[i].setCommand(motion.command(i));
      joint[i].update(dt);
    }
    // One-shot bits: consumed above, cleared here, exactly once.
    motion.setFlag(CMD_ZERO_HERE | CMD_CLEAR_FAULT, false);

    // Zeroing and fault recovery both move the origin under the setpoint
    // stream. Re-park it on the new measurement, or the very next cycle sees a
    // huge following error that the joint did not earn.
    if (reparkPending) {
      reparkPending = false;
      int32_t here[kNumJoints];
      currentPositions(here);
      motion.resetTo(here);
    }
  }

  if (logging && millis() - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = millis();
    uint8_t faults = 0;
    Serial.print(lastLogMs);
    for (uint8_t i = 0; i < kNumJoints; ++i) {
      Serial.print(',');
      Serial.print(countsToDeg(motion.command(i).target_counts), 2);
      Serial.print(',');
      Serial.print(countsToDeg(joint[i].state().pos_counts), 2);
      faults |= joint[i].state().fault & kFaultMask;
    }
    Serial.print(',');
    Serial.print(gripper.angleDeg(), 1);
    Serial.print(',');
    Serial.println(faults);
  }
}
