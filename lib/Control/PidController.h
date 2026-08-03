#pragma once
// Minimal PID with clamped integral (anti-windup) and derivative on
// measurement, which avoids the output spike you get when the setpoint jumps.
#include <math.h>

class PidController {
 public:
  PidController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f)
      : kp_(kp), ki_(ki), kd_(kd) {}

  void setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }
  void setOutputLimit(float limit) { outLimit_ = fabsf(limit); }
  void setIntegralLimit(float limit) { intLimit_ = fabsf(limit); }

  void reset() {
    integral_ = 0.0f;
    lastMeasurement_ = 0.0f;
    primed_ = false;
  }

  /// error = setpoint - measurement, dt in seconds.
  float update(float error, float measurement, float dt) {
    if (dt <= 0.0f) return lastOutput_;

    const float p = kp_ * error;

    integral_ += error * dt;
    if (integral_ > intLimit_) integral_ = intLimit_;
    if (integral_ < -intLimit_) integral_ = -intLimit_;
    const float i = ki_ * integral_;

    float d = 0.0f;
    if (primed_) d = -kd_ * (measurement - lastMeasurement_) / dt;
    lastMeasurement_ = measurement;
    primed_ = true;

    float out = p + i + d;
    if (out > outLimit_) {
      out = outLimit_;
      if (ki_ != 0.0f && error > 0.0f) integral_ -= error * dt;  // stop winding
    } else if (out < -outLimit_) {
      out = -outLimit_;
      if (ki_ != 0.0f && error < 0.0f) integral_ -= error * dt;
    }
    lastOutput_ = out;
    return out;
  }

  float kp() const { return kp_; }
  float ki() const { return ki_; }
  float kd() const { return kd_; }
  float integral() const { return integral_; }

  /// Keep the derivative state current without integrating or producing an
  /// output. Used while the joint sits inside its in-position deadband: the
  /// controller stops pushing, but the moment error leaves the band the D term
  /// is still valid instead of spiking off a stale measurement.
  void trackMeasurement(float measurement) {
    lastMeasurement_ = measurement;
    primed_ = true;
    lastOutput_ = 0.0f;
  }

  /// Drop the integral only. Gains and derivative state survive.
  void clearIntegral() { integral_ = 0.0f; }

 private:
  float kp_, ki_, kd_;
  float integral_ = 0.0f;
  float lastMeasurement_ = 0.0f;
  float lastOutput_ = 0.0f;
  float outLimit_ = 1.0e6f;
  float intLimit_ = 1.0e6f;
  bool primed_ = false;
};
