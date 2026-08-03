#pragma once
// Hardware-independent angle/motion math.
// Nothing in here includes Arduino, so it compiles and unit-tests on the PC
// (`pio test -e native_tests`). Keep every pure-math helper here.
#include <math.h>
#include <stdint.h>

namespace armmath {

constexpr float kPi = 3.14159265358979f;
constexpr uint16_t kAs5600Counts = 4096;  // 12-bit
constexpr float kDegPerCount = 360.0f / kAs5600Counts;

inline float degToRad(float deg) { return deg * (kPi / 180.0f); }
inline float radToDeg(float rad) { return rad * (180.0f / kPi); }

inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/// Wrap to [0, 360).
inline float wrap360(float deg) {
  float a = fmodf(deg, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a;
}

/// Wrap to [-180, 180).
inline float wrap180(float deg) { return wrap360(deg + 180.0f) - 180.0f; }

/// Shortest signed rotation from `from` to `to`.
inline float shortestDelta(float from, float to) { return wrap180(to - from); }

inline float countsToDeg(uint16_t counts) { return counts * kDegPerCount; }
inline float degToCounts(float deg) { return wrap360(deg) / kDegPerCount; }

/// Exponential moving average. alpha=1 passes input straight through.
inline float ema(float previous, float sample, float alpha) {
  return previous + alpha * (sample - previous);
}

/// Limit how far `value` may move toward `target` in one update.
inline float slew(float value, float target, float maxStep) {
  const float d = target - value;
  if (d > maxStep) return value + maxStep;
  if (d < -maxStep) return value - maxStep;
  return target;
}

/// Zero inside +/-width. On a stepper the encoder LSB (0.088 deg) is ~8
/// microsteps wide, so without this the controller chases an error it can
/// never resolve and the joint limit-cycles at rest.
inline float deadband(float value, float width) {
  if (value > width) return value - width;
  if (value < -width) return value + width;
  return 0.0f;
}

/// Boxcar filter. Convolving a trapezoidal profile with a window of length
/// N*dt yields a genuinely jerk-limited profile with jmax = amax/(N*dt), at
/// the cost of extending the move by exactly N*dt. Cheaper and harder to get
/// wrong than a 7-segment S-curve, and it is a linear operator, so filtering
/// position and velocity with the same window keeps them consistent.
template <uint8_t N>
class MovingAverage {
 public:
  void reset(float value = 0.0f) {
    for (uint8_t i = 0; i < N; ++i) buf_[i] = value;
    sum_ = value * N;
    idx_ = 0;
  }

  float update(float sample) {
    sum_ -= buf_[idx_];
    buf_[idx_] = sample;
    sum_ += sample;
    idx_ = static_cast<uint8_t>((idx_ + 1) % N);
    return sum_ / N;
  }

  float value() const { return sum_ / N; }
  static constexpr uint8_t taps() { return N; }

 private:
  float buf_[N] = {};
  float sum_ = 0.0f;
  uint8_t idx_ = 0;
};

/// Turns a wrapped 0-360 single-turn reading into a continuous multi-turn
/// angle. This is what makes a single-turn AS5600 usable on a joint that
/// rotates through its zero crossing.
class AngleUnwrapper {
 public:
  void reset(float wrappedDeg = 0.0f) { resetTo(wrappedDeg, 0); }

  /// Same, but you choose the turn count. Needed at power-on with an absolute
  /// encoder: a joint sitting at -30 deg reads 330 deg, and only the caller
  /// knows the joint's range well enough to say that means turns = -1.
  void resetTo(float wrappedDeg, int32_t turns) {
    last_ = wrap360(wrappedDeg);
    turns_ = turns;
    primed_ = true;
  }

  float update(float wrappedDeg) {
    const float a = wrap360(wrappedDeg);
    if (!primed_) reset(a);
    const float delta = a - last_;
    if (delta > 180.0f) {
      turns_--;  // wrapped backwards through 0
    } else if (delta < -180.0f) {
      turns_++;  // wrapped forwards through 360
    }
    last_ = a;
    return continuousDeg();
  }

  float continuousDeg() const { return turns_ * 360.0f + last_; }
  int32_t turns() const { return turns_; }
  bool primed() const { return primed_; }

 private:
  float last_ = 0.0f;
  int32_t turns_ = 0;
  bool primed_ = false;
};

}  // namespace armmath
