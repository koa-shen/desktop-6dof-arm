#pragma once
// Layer 1 of D7: the motion controller. Its defining job is not computation,
// it is TIME SYNCHRONIZATION - making N joints start and finish together so
// the tool stays on the commanded path. No single joint can do this, and the
// application layer is too jittery to.
//
// It emits a stream of MODE_TRACK setpoints ("be at X now, moving at V"), one
// per joint per control cycle. That is the streaming idiom D7 chose over
// segment handoff, and it is why the joint nodes below stay dumb.
//
// No Arduino dependency, on purpose: this is the piece most likely to move to
// a Teensy or an STM32, and keeping it host-compilable means it can be tested
// against the plant model in tools/joint_sim.py rather than on a real arm.
#include <stdint.h>

#include "ArmMath.h"
#include "JointProtocol.h"
#include "Trajectory.h"

/// TAPS is the jerk limiter width. The setpoint stream is boxcar-filtered over
/// TAPS cycles, which turns the trapezoid's instantaneous acceleration steps
/// into ramps of jmax = amax / (TAPS * dt) and lengthens the move by exactly
/// TAPS * dt. A printed 1.8 kg arm has lightly damped structural modes that an
/// acceleration step rings; this is the cheapest honest way to stop exciting
/// them. TAPS = 1 disables it.
template <uint8_t N, uint8_t TAPS = 4>
class MotionController {
 public:
  struct Limits {
    float max_vel;  // counts/s
    float max_acc;  // counts/s^2
  };

  void begin(const Limits& lim) {
    for (uint8_t i = 0; i < N; ++i) {
      limits_[i] = lim;
      cmd_[i].mode = jointnode::MODE_HOLD;
      cmd_[i].flags = 0;
      cmd_[i].max_vel = 0;
      cmd_[i].target_counts = 0;
    }
  }

  void setLimits(uint8_t joint, const Limits& lim) {
    if (joint < N) limits_[joint] = lim;
  }

  /// Plan a synchronized move from `current` to `target`. Every joint gets the
  /// slowest joint's duration, so they arrive together instead of each running
  /// flat out and finishing whenever. speedScale in (0, 1] derates the whole
  /// move without breaking synchronization.
  ///
  /// Returns the move duration in seconds (0 if there is nothing to do).
  float moveTo(const int32_t* target, const int32_t* current,
               float speedScale = 1.0f) {
    if (speedScale <= 0.0f) speedScale = 1.0f;
    if (speedScale > 1.0f) speedScale = 1.0f;

    float longest = 0.0f;
    for (uint8_t i = 0; i < N; ++i) {
      const float d = static_cast<float>(target[i] - current[i]);
      const float t = armmath::TrapezoidProfile::minDuration(
          d, limits_[i].max_vel * speedScale, limits_[i].max_acc * speedScale);
      if (t > longest) longest = t;
    }

    for (uint8_t i = 0; i < N; ++i) {
      profile_[i].planFor(static_cast<float>(current[i]),
                          static_cast<float>(target[i]),
                          limits_[i].max_vel * speedScale,
                          limits_[i].max_acc * speedScale, longest);
      filter_[i].reset(static_cast<float>(current[i]));
      lastFiltered_[i] = static_cast<float>(current[i]);
      cmd_[i].mode = jointnode::MODE_TRACK;
    }
    t_ = 0.0f;
    duration_ = longest;
    settleLeft_ = TAPS;
    return longest + TAPS * lastDt_;
  }

  /// Abandon the current move and hold wherever the setpoint stream is now.
  /// Deliberately holds the SETPOINT, not the measurement: stopping should not
  /// silently give up the following error the joints were still working off.
  void abort() {
    for (uint8_t i = 0; i < N; ++i) {
      profile_[i].planFor(lastFiltered_[i], lastFiltered_[i], limits_[i].max_vel,
                          limits_[i].max_acc, 0.0f);
      cmd_[i].setVelFeedforward(0);
      cmd_[i].target_counts = static_cast<int32_t>(lastFiltered_[i]);
    }
    t_ = duration_ = 0.0f;
    settleLeft_ = 0;
  }

  /// Park the stream on a known set of positions, e.g. straight after homing.
  void resetTo(const int32_t* current) {
    for (uint8_t i = 0; i < N; ++i) {
      profile_[i].planFor(static_cast<float>(current[i]),
                          static_cast<float>(current[i]), limits_[i].max_vel,
                          limits_[i].max_acc, 0.0f);
      filter_[i].reset(static_cast<float>(current[i]));
      lastFiltered_[i] = static_cast<float>(current[i]);
      cmd_[i].mode = jointnode::MODE_TRACK;
      cmd_[i].target_counts = current[i];
      cmd_[i].setVelFeedforward(0);
    }
    t_ = duration_ = 0.0f;
    settleLeft_ = 0;
  }

  /// Advance the setpoint stream one control cycle.
  void update(float dt) {
    if (dt > 0.0f) lastDt_ = dt;
    t_ += dt;
    if (t_ > duration_ && settleLeft_) settleLeft_--;

    for (uint8_t i = 0; i < N; ++i) {
      const armmath::MotionSample s = profile_[i].sample(t_);
      const float p = filter_[i].update(s.pos);
      // The filter is linear, so differentiating its output gives exactly the
      // filtered velocity - no second filter, no phase mismatch between the
      // position setpoint and the feedforward that is supposed to produce it.
      const float v = (dt > 0.0f) ? (p - lastFiltered_[i]) / dt : 0.0f;
      lastFiltered_[i] = p;

      cmd_[i].target_counts = static_cast<int32_t>(p < 0.0f ? p - 0.5f : p + 0.5f);
      cmd_[i].setVelFeedforward(
          static_cast<int16_t>(armmath::clampf(v, -32767.0f, 32767.0f)));
    }
  }

  void setEnabled(bool on) {
    for (uint8_t i = 0; i < N; ++i) {
      if (on) {
        cmd_[i].flags |= jointnode::CMD_ENABLE;
      } else {
        cmd_[i].flags &= ~jointnode::CMD_ENABLE;
      }
    }
  }

  void setFlag(uint8_t flag, bool on) {
    for (uint8_t i = 0; i < N; ++i) {
      if (on) {
        cmd_[i].flags |= flag;
      } else {
        cmd_[i].flags &= ~flag;
      }
    }
  }

  const jointnode::JointCommand& command(uint8_t joint) const {
    return cmd_[joint];
  }
  jointnode::JointCommand& command(uint8_t joint) { return cmd_[joint]; }

  bool moving() const { return t_ < duration_ || settleLeft_ > 0; }
  float duration() const { return duration_; }
  float elapsed() const { return t_; }

 private:
  armmath::TrapezoidProfile profile_[N];
  armmath::MovingAverage<TAPS> filter_[N];
  jointnode::JointCommand cmd_[N];
  Limits limits_[N] = {};
  float lastFiltered_[N] = {};
  float t_ = 0.0f;
  float duration_ = 0.0f;
  float lastDt_ = 0.005f;
  uint8_t settleLeft_ = 0;
};
