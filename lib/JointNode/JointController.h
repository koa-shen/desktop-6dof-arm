#pragma once
// One joint's entire closed loop, with no idea how it is being talked to.
//
// It consumes a JointCommand and produces a JointState. Today the caller is a
// function in the same binary; later it is a CAN receive ISR on a per-joint
// MCU. Nothing in this class changes when that happens - which is the point.
//
// Deliberately NOT owned by this class: the transport, the trajectory
// generator, and anything that knows about other joints. A joint node's whole
// job is "get to the commanded angle and tell the truth about where you are".
#include <Arduino.h>

#include "AS5600Encoder.h"
#include "JointProtocol.h"
#include "PidController.h"
#include "StepperDriver.h"

class JointController {
 public:
  struct Config {
    float kp, ki, kd;
    float steps_per_count;   // motor microsteps per encoder count
    int32_t min_counts;      // soft limits, at the output
    int32_t max_counts;
    uint16_t default_max_vel;    // counts/s
    uint16_t following_err_max;  // counts; 0 disables the check
    float max_step_rate;         // steps/s ceiling handed to the driver
  };

  JointController(AS5600Encoder* enc, StepperDriver* drv, uint8_t index)
      : enc_(enc), drv_(drv), index_(index) {}

  void begin(const Config& cfg) {
    cfg_ = cfg;
    pid_.setGains(cfg.kp, cfg.ki, cfg.kd);
    pid_.setOutputLimit(cfg.max_step_rate);
    drv_->setMaxSpeed(cfg.max_step_rate);
    state_.fault = jointnode::FAULT_NOT_HOMED;
    state_.seq = 0;
  }

  /// Apply a command. Cheap and side-effect-light; the work happens in update().
  void setCommand(const jointnode::JointCommand& cmd) {
    if ((cmd.flags & jointnode::CMD_CLEAR_FAULT) && !clearFaultPrev_) {
      state_.fault &= jointnode::FAULT_NOT_HOMED;  // homing is not clearable
      pid_.reset();
    }
    clearFaultPrev_ = (cmd.flags & jointnode::CMD_CLEAR_FAULT) != 0;

    if (cmd.flags & jointnode::CMD_ZERO_HERE) {
      enc_->zeroHere();
      state_.fault &= ~jointnode::FAULT_NOT_HOMED;
    }

    cmd_ = cmd;
    lastCmdMs_ = millis();
  }

  /// Run one control cycle. dt in seconds. Call at a fixed rate.
  void update(float dt) {
    const bool encOk = enc_->read();
    const int32_t pos = jointnode::degToCounts(enc_->angleDeg());

    if (!encOk || !enc_->magnetOk()) state_.fault |= jointnode::FAULT_ENCODER;
    if (millis() - lastCmdMs_ > kCommsTimeoutMs) {
      state_.fault |= jointnode::FAULT_COMMS;
    }

    state_.vel_counts_s = static_cast<int16_t>(
        constrain((pos - state_.pos_counts) / dt, -32767.0f, 32767.0f));
    state_.pos_counts = pos;

    const bool armed = (cmd_.flags & jointnode::CMD_ENABLE) &&
                       cmd_.mode != jointnode::MODE_IDLE &&
                       state_.fault == jointnode::FAULT_NONE;
    if (!armed) {
      drv_->setVelocity(0.0f);
      drv_->enable(false);
      pid_.reset();
      publish_();
      return;
    }
    drv_->enable(true);

    int32_t target = cmd_.target_counts;
    if (cmd_.mode == jointnode::MODE_HOLD) target = pos;

    if (target < cfg_.min_counts || target > cfg_.max_counts) {
      state_.fault |= jointnode::FAULT_SOFT_LIMIT;
      target = constrain(target, cfg_.min_counts, cfg_.max_counts);
    }

    const int32_t err = target - pos;
    if (cfg_.following_err_max && labs(err) > cfg_.following_err_max) {
      state_.fault |= jointnode::FAULT_FOLLOWING;
    }

    float stepsPerSec;
    if (cmd_.mode == jointnode::MODE_VELOCITY) {
      stepsPerSec = cmd_.target_counts * cfg_.steps_per_count;
    } else {
      stepsPerSec = pid_.update(static_cast<float>(err),
                                static_cast<float>(pos), dt) *
                    cfg_.steps_per_count;
    }

    const float vlim = (cmd_.max_vel ? cmd_.max_vel : cfg_.default_max_vel) *
                       cfg_.steps_per_count;
    drv_->setVelocity(constrain(stepsPerSec, -vlim, vlim));
    publish_();
  }

  const jointnode::JointState& state() const { return state_; }
  uint8_t index() const { return index_; }

 private:
  static constexpr uint32_t kCommsTimeoutMs = 500;

  void publish_() { state_.seq++; }

  AS5600Encoder* enc_;
  StepperDriver* drv_;
  PidController pid_;
  Config cfg_{};
  jointnode::JointCommand cmd_{};
  jointnode::JointState state_{};
  uint32_t lastCmdMs_ = 0;
  uint8_t index_;
  bool clearFaultPrev_ = false;
};
