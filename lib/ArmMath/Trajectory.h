#pragma once
// Time-parameterized motion profiles. No Arduino, no hardware, no units:
// feed it counts and it gives back counts, feed it degrees and it gives back
// degrees. That is what makes it unit-testable on the host.
//
// WHY THIS EXISTS
// A PID handed a step change in setpoint is being asked to do two jobs at
// once: invent a motion profile *and* reject error. It is bad at the first
// one. The profile belongs here, where it is deterministic and bounded; the
// PID then only ever sees the small residual between a physically achievable
// setpoint and the measured position. That residual is what "following error"
// means, and it is the number worth trending.
#include <math.h>
#include <stdint.h>

namespace armmath {

struct MotionSample {
  float pos;
  float vel;
  float acc;
};

/// Symmetric trapezoidal velocity profile, optionally stretched to a
/// prescribed duration so several axes finish together.
class TrapezoidProfile {
 public:
  /// Shortest possible duration for this move under the given limits.
  static float minDuration(float distance, float vmax, float amax) {
    const float d = fabsf(distance);
    if (d <= 0.0f || vmax <= 0.0f || amax <= 0.0f) return 0.0f;
    if (d * amax <= vmax * vmax) return 2.0f * sqrtf(d / amax);  // triangular
    return vmax / amax + d / vmax;
  }

  /// Plan the fastest legal move from `start` to `goal`.
  void plan(float start, float goal, float vmax, float amax) {
    planFor(start, goal, vmax, amax, minDuration(goal - start, vmax, amax));
  }

  /// Plan a move that takes exactly `duration`, by lowering the cruise
  /// velocity while keeping acceleration at `amax`. This is how axes are
  /// synchronized: everyone gets the slowest axis's duration.
  ///
  /// If `duration` is shorter than physically possible it is clamped up, so a
  /// synchronized group can never be handed an impossible segment.
  void planFor(float start, float goal, float vmax, float amax,
               float duration) {
    start_ = start;
    goal_ = goal;
    const float delta = goal - start;
    dist_ = fabsf(delta);
    sign_ = (delta < 0.0f) ? -1.0f : 1.0f;
    accel_ = (amax > 0.0f) ? amax : 1.0f;

    if (dist_ <= 0.0f) {
      cruise_ = 0.0f;
      duration_ = (duration > 0.0f) ? duration : 0.0f;
      tAccel_ = 0.0f;
      tFlat_ = duration_;
      return;
    }

    const float tMin = minDuration(dist_, vmax, accel_);
    duration_ = (duration > tMin) ? duration : tMin;

    // Total time for a trapezoid is T = v/a + d/v. Solve for v given T; the
    // smaller root is the one that actually respects the acceleration limit.
    const float aT = accel_ * duration_;
    float disc = aT * aT - 4.0f * accel_ * dist_;
    if (disc < 0.0f) disc = 0.0f;  // only reachable via float error at T = tMin
    cruise_ = 0.5f * (aT - sqrtf(disc));
    if (cruise_ > vmax) cruise_ = vmax;
    if (cruise_ <= 0.0f) cruise_ = dist_ / duration_;

    tAccel_ = cruise_ / accel_;
    tFlat_ = duration_ - tAccel_;  // time at which deceleration begins
  }

  MotionSample sample(float t) const {
    MotionSample s{goal_, 0.0f, 0.0f};
    if (dist_ <= 0.0f || t >= duration_) return s;
    if (t <= 0.0f) {
      s.pos = start_;
      return s;
    }

    float p, v, a;
    if (t < tAccel_) {
      p = 0.5f * accel_ * t * t;
      v = accel_ * t;
      a = accel_;
    } else if (t < tFlat_) {
      p = 0.5f * accel_ * tAccel_ * tAccel_ + cruise_ * (t - tAccel_);
      v = cruise_;
      a = 0.0f;
    } else {
      const float td = duration_ - t;
      p = dist_ - 0.5f * accel_ * td * td;
      v = accel_ * td;
      a = -accel_;
    }
    s.pos = start_ + sign_ * p;
    s.vel = sign_ * v;
    s.acc = sign_ * a;
    return s;
  }

  float duration() const { return duration_; }
  float cruiseVelocity() const { return cruise_; }
  float distance() const { return dist_; }
  float goal() const { return goal_; }
  bool done(float t) const { return t >= duration_; }

 private:
  float start_ = 0.0f, goal_ = 0.0f, dist_ = 0.0f, sign_ = 1.0f;
  float accel_ = 1.0f, cruise_ = 0.0f;
  float tAccel_ = 0.0f, tFlat_ = 0.0f, duration_ = 0.0f;
};

/// Longest of the per-axis minimum durations - i.e. how long a synchronized
/// move must take. Plan every axis with planFor(..., this) afterwards.
inline float syncDuration(const float* distances, const float* vmax,
                          const float* amax, uint8_t n) {
  float longest = 0.0f;
  for (uint8_t i = 0; i < n; ++i) {
    const float t = TrapezoidProfile::minDuration(distances[i], vmax[i], amax[i]);
    if (t > longest) longest = t;
  }
  return longest;
}

}  // namespace armmath
