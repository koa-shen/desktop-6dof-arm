// app_08_host_link - the arm as a device, not a demo.
//
// No CLI, no CSV, no human on the other end. The Uno runs D7's layer 0 only:
// six (today three) joint servo loops, the gripper, and its own safety. Layer 1
// and layer 2 - trajectory generation, synchronization, IK, task sequencing -
// run on the host in tools/. This is the split that lets the arm do
// pick-and-place without pretending an ATmega328P can plan a path.
//
//   pio run -e host_link -t upload
//   python tools/pick_place.py --port COM5
//
// Wire protocol: lib/JointNode/PacketFraming.h, mirrored in tools/joint_link.py.
// The host streams MODE_TRACK setpoints ("be here now, moving at this rate")
// and the node reports state back at a fixed rate.
//
// BANDWIDTH IS THE REAL LIMIT HERE, not CPU. One framed 8-byte payload is 13
// bytes on the wire. Three joints in and three out at 200 Hz is 7.8 kB/s each
// direction, which does not fit in 115200 baud (11.5 kB/s total, both
// directions sharing the UART's attention). Hence LINK_BAUD: at 500000 the
// same traffic is 16 % of the link instead of 68 %. Six joints at 200 Hz would
// be 15.6 kB/s each way - still fine at 500 kbaud, and precisely the number
// that makes CAN at 1 Mbps look like the right answer rather than a flex.
#include <Arduino.h>
#include <Wire.h>

#include "AS5600Encoder.h"
#include "JointController.h"
#include "JointProtocol.h"
#include "PacketFraming.h"
#include "ServoGripper.h"
#include "StepperDriver.h"
#include "TCA9548A.h"
#include "joint_config.h"
#include "pins.h"

using namespace jointnode;

constexpr uint8_t kNumJoints = 3;
constexpr uint16_t kStatePeriodMs = 3;  // one joint per tick: ~111 Hz each

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
ServoGripper gripper;
link::PacketReader reader;

uint32_t lastCtrlUs = 0;
uint32_t lastStateMs = 0;
uint8_t stateTurn = 0;

static void sendFrame(uint8_t type, uint8_t node, const uint8_t* payload,
                      uint8_t len) {
  uint8_t frame[link::kMaxFrame];
  const uint8_t n = link::buildFrame(type, node, payload, len, frame);
  if (n) Serial.write(frame, n);
}

static void handlePacket() {
  switch (reader.type()) {
    case link::PKT_CMD: {
      const uint8_t j = reader.node();
      if (j < kNumJoints && reader.payloadLen() == sizeof(JointCommand)) {
        memcpy(&cmd[j], reader.payload(), sizeof(JointCommand));
      }
      break;
    }
    case link::PKT_GRIPPER:
      if (reader.payloadLen() >= 1) gripper.setPercent(reader.payload()[0]);
      break;
    case link::PKT_ESTOP:
      for (uint8_t i = 0; i < kNumJoints; ++i) {
        cmd[i].flags &= ~CMD_ENABLE;
        cmd[i].mode = MODE_IDLE;
        joint[i].estop();
      }
      break;
    case link::PKT_CLEAR:
      for (uint8_t i = 0; i < kNumJoints; ++i) cmd[i].flags |= CMD_CLEAR_FAULT;
      break;
    case link::PKT_PING:
      sendFrame(link::PKT_PONG, reader.node(), reader.payload(),
                reader.payloadLen());
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(LINK_BAUD);
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
    cmd[i].mode = MODE_HOLD;
    cmd[i].flags = 0;
    cmd[i].max_vel = 0;
    cmd[i].target_counts = joint[i].state().pos_counts;
  }

  ServoGripper::Config gcfg;
  gcfg.pin = PIN_GRIPPER_SERVO;
  gcfg.open_deg = GRIPPER_OPEN_DEG;
  gcfg.closed_deg = GRIPPER_CLOSED_DEG;
  gcfg.grip_deg = GRIPPER_GRIP_DEG;
  gcfg.slew_deg_s = GRIPPER_SLEW_DEG_S;
  gcfg.settle_ms = GRIPPER_SETTLE_MS;
  gcfg.detach_idle = GRIPPER_DETACH_IDLE;
  gripper.begin(gcfg);

  lastCtrlUs = micros();
}

void loop() {
  for (uint8_t i = 0; i < kNumJoints; ++i) drv[i].run();
  gripper.update();

  while (Serial.available()) {
    if (reader.feed((uint8_t)Serial.read())) handlePacket();
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

  // One joint per tick, round-robin: keeps the UART from being handed three
  // frames back to back while the step generator is trying to meet a deadline.
  if (millis() - lastStateMs >= kStatePeriodMs) {
    lastStateMs = millis();
    sendFrame(link::PKT_STATE, stateTurn, (const uint8_t*)&joint[stateTurn].state(),
              sizeof(JointState));
    stateTurn = (uint8_t)((stateTurn + 1) % kNumJoints);
  }
}
