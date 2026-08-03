#pragma once
// One joint's entire closed loop, with no idea how it is being talked to.
//
// It consumes a JointCommand and produces a JointState. Today the caller is a
// function in the same binary; later it is a CAN receive ISR on a per-joint
// MCU. Nothing in this class changes when that happens - which is the point.
//
// CONTROL LAW (D10)
//   step_rate = k_ff * v_setpoint  +  PID(setpoint - measured)
//
// The feedforward term does almost all of the work. A stepper commanded at the
// right rate already ends up in the right place, so the PID only has to clean
// up load droop, backlash and gear-ratio error. That is what lets Kp stay
// small, and a small Kp is what keeps the loop quiet.
//
// The deadband matters as much as the gains. One AS5600 count is 0.088 deg,
// which at 20:1 and 8 microsteps is eight microsteps wide - the loop can
// command a correction eight times finer than the smallest error it can see.
// Without a deadband the error changes sign every time the encoder ticks and
// the joint hunts audibly forever. Inside the band the loop stops pushing and
// freezes the integrator.
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
    float vel_ff_scale;          // 1.0 = trust the model; trim after measuring
    float integral_limit;        // count-seconds, anti-windup
    uint8_t in_position_counts;  // deadband half-width, encoder counts
    float home_offset_deg;       // mechanical angle that is joint zero (D4)
    bool absolute_home;          // true = homed in begin(), no homing move
  };

  JointController(AS5600Encoder* enc, StepperDriver* drv, uint8_t index)
      : enc_(enc), drv_(drv), index_(index) {}

  void begin(const Config& cfg) {
    cfg_ = cfg;
    if (cfg_.vel_ff_scale == 0.0f) cfg_.vel_ff_scale = 1.0f;
    pid_.setGains(cfg.kp, cfg.ki, cfg.kd);
    pid_.setOutputLimit(cfg.max_step_rate / cfg.steps_per_count);
    if (cfg.integral_limit > 0.0f) pid_.setIntegralLimit(cfg.integral_limit);
    drv_->setMaxSpeed(cfg.max_step_rate);
    state_.fault = jointnode::FAULT_NOT_HOMED;
    state_.seq = 0;

    // The payoff of an output-side absolute encoder: power on and the joint
    // already knows its angle. No homing move, no limit switch, no crash into
    // a hard stop with a printed gearbox in the load path.
    if (cfg_.absolute_home && enc_->homeAbsolute(cfg_.home_offset_deg)) {
      state_.fault &= ~jointnode::FAULT_NOT_HOMED;
      state_.pos_counts = jointnode::degToCounts(enc_->angleDeg());
    }
  }

  /// Apply a command. Cheap and side-effect-light; the work happens in update().
  void setCommand(const jointnode::JointCommand& cmd) {
    if ((cmd.flags & jointnode::CMD_CLEAR_FAULT) && !clearFaultPrev_) {
      recover_();
    }
    clearFaultPrev_ = (cmd.flags & jointnode::CMD_CLEAR_FAULT) != 0;

    if (cmd.flags & jointnode::CMD_ZERO_HERE) {
      enc_->zeroHere();
      state_.fault &= ~jointnode::FAULT_NOT_HOMED;
      holdPrimed_ = false;
    }

    cmd_ = cmd;
    lastCmdMs_ = millis();
  }

  /// Software estop. This is the SECOND line of defence: the hardware line
  /// that pulls every driver ENABLE low is the first, because it still works
  /// when this MCU is the thing that has gone wrong (D7).
  void estop() {
    drv_->emergencyStop();
    drv_->enable(false);
    state_.fault |= jointnode::FAULT_DRIVER;
    pid_.reset();
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
                       (state_.fault & jointnode::kFaultMask) ==
                           jointnode::FAULT_NONE;
    if (!armed) {
      drv_->setVelocity(0.0f);
      drv_->enable(false);
      pid_.reset();
      setInPosition_(false);
      publish_();
      return;
    }
    drv_->enable(true);

    if (cmd_.mode == jointnode::MODE_VELOCITY) {
      // Open-loop rate command: no target, so no error and no in-position.
      const float lim = cfg_.max_step_rate;
      drv_->setVelocity(
          constrain(cmd_.target_counts * cfg_.steps_per_count, -lim, lim));
      setInPosition_(false);
      publish_();
      return;
    }

    int32_t target = cmd_.target_counts;
    if (cmd_.mode == jointnode::MODE_HOLD) target = holdTarget_(pos);

    if (target < cfg_.min_counts || target > cfg_.max_counts) {
      state_.fault |= jointnode::FAULT_SOFT_LIMIT;
      target = constrain(target, cfg_.min_counts, cfg_.max_counts);
    }

    const int32_t err = target - pos;
    if (cfg_.following_err_max && labs(err) > cfg_.following_err_max) {
      state_.fault |= jointnode::FAULT_FOLLOWING;
    }

    // Only MODE_TRACK carries a feedforward, because only a streamed setpoint
    // has a meaningful instantaneous velocity to feed forward.
    float stepsPerSec = 0.0f;
    if (cmd_.mode == jointnode::MODE_TRACK) {
      stepsPerSec =
          cmd_.velFeedforward() * cfg_.steps_per_count * cfg_.vel_ff_scale;
    }

    const bool inBand = labs(err) <= (int32_t)cfg_.in_position_counts;
    if (inBand) {
      pid_.trackMeasurement(static_cast<float>(pos));
    } else {
      stepsPerSec += pid_.update(static_cast<float>(err),
                                 static_cast<float>(pos), dt) *
                     cfg_.steps_per_count;
    }
    setInPosition_(inBand);

    const uint16_t velCap =
        (cmd_.mode == jointnode::MODE_TRACK || cmd_.max_vel == 0)
            ? cfg_.default_max_vel
            : cmd_.max_vel;
    const float vlim = velCap * cfg_.steps_per_count;
    drv_->setVelocity(constrain(stepsPerSec, -vlim, vlim));
    publish_();
  }

  const jointnode::JointState& state() const { return state_; }
  bool homed() const {
    return (state_.fault & jointnode::FAULT_NOT_HOMED) == 0;
  }
  bool inPosition() const { return state_.inPosition(); }
  uint8_t index() const { return index_; }

 private:
  static constexpr uint32_t kCommsTimeoutMs = 500;

  /// Defined recovery: drop every clearable fault, forget the integral, and
  /// re-latch the hold target here, so clearing a fault can never itself
  /// command a move. Homing is deliberately not clearable - a joint that lost
  /// its encoder no longer knows where it is, and pretending otherwise is how
  /// machines drive into their own hard stops.
  void recover_() {
    state_.fault &= jointnode::FAULT_NOT_HOMED;
    pid_.reset();
    holdPrimed_ = false;
    enc_->clearErrors();
  }

  /// Latch the hold target once rather than re-reading position every cycle;
  /// otherwise the setpoint follows the drift it exists to resist.
  int32_t holdTarget_(int32_t pos) {
    if (!holdPrimed_) {
      holdPos_ = pos;
      holdPrimed_ = true;
    }
    return holdPos_;
  }

  void setInPosition_(bool on) {
    if (on) {
      state_.fault |= jointnode::STATUS_IN_POSITION;
    } else {
      state_.fault &= ~jointnode::STATUS_IN_POSITION;
      if (cmd_.mode != jointnode::MODE_HOLD) holdPrimed_ = false;
    }
  }

  void publish_() { state_.seq++; }

  AS5600Encoder* enc_;
  StepperDriver* drv_;
  PidController pid_;
  Config cfg_{};
  jointnode::JointCommand cmd_{};
  jointnode::JointState state_{};
  uint32_t lastCmdMs_ = 0;
  int32_t holdPos_ = 0;
  uint8_t index_;
  bool clearFaultPrev_ = false;
  bool holdPrimed_ = false;
};
