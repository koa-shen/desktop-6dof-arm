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

/// Turns a wrapped 0-360 single-turn reading into a continuous multi-turn
/// angle. This is what makes a single-turn AS5600 usable on a joint that
/// rotates through its zero crossing.
class AngleUnwrapper {
 public:
  void reset(float wrappedDeg = 0.0f) {
    last_ = wrap360(wrappedDeg);
    turns_ = 0;
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
