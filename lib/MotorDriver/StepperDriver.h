#pragma once
// Non-blocking STEP/DIR driver for a TMC2209 (or any step/dir module).
//
// Call run() as fast as you can from loop(). It never blocks, so the encoder
// keeps streaming while the motor moves - that is the whole point of Phase 1.
//
// Two modes:
//   POSITION - moveTo()/move(), trapezoidal accel + decel to a step target
//   VELOCITY - setVelocity(), used as the output of the closed-loop controller
#include <Arduino.h>

class StepperDriver {
 public:
  enum Mode : uint8_t { MODE_POSITION, MODE_VELOCITY };

  StepperDriver(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin,
                bool enableActiveLow = true)
      : stepPin_(stepPin),
        dirPin_(dirPin),
        enPin_(enablePin),
        enActiveLow_(enableActiveLow) {}

  void begin() {
    pinMode(stepPin_, OUTPUT);
    pinMode(dirPin_, OUTPUT);
    pinMode(enPin_, OUTPUT);
    digitalWrite(stepPin_, LOW);
    digitalWrite(dirPin_, LOW);
    enable(false);
    lastUpdateUs_ = lastStepUs_ = micros();
  }

  void enable(bool on) {
    enabled_ = on;
    digitalWrite(enPin_, (on == enActiveLow_) ? LOW : HIGH);
    if (!on) velocity_ = 0.0f;
  }
  bool isEnabled() const { return enabled_; }

  void setMaxSpeed(float stepsPerSec) {
    maxSpeed_ = fabsf(stepsPerSec) < kMinSpeed ? kMinSpeed : fabsf(stepsPerSec);
  }
  void setAcceleration(float stepsPerSec2) {
    accel_ = fabsf(stepsPerSec2) < 1.0f ? 1.0f : fabsf(stepsPerSec2);
  }
  /// Set true if positive steps drive the joint the "wrong" way.
  void setInvertDirection(bool invert) { invertDir_ = invert; }

  void moveTo(long absoluteSteps) {
    mode_ = MODE_POSITION;
    target_ = absoluteSteps;
  }
  void move(long relativeSteps) { moveTo(target_ + relativeSteps); }

  void setVelocity(float stepsPerSec) {
    mode_ = MODE_VELOCITY;
    targetVelocity_ = constrain(stepsPerSec, -maxSpeed_, maxSpeed_);
  }

  /// Decelerate to a stop and hold here.
  void stop() {
    mode_ = MODE_VELOCITY;
    targetVelocity_ = 0.0f;
  }

  /// Drop velocity to zero instantly. Will lose steps under load - use for faults.
  void emergencyStop() {
    velocity_ = 0.0f;
    targetVelocity_ = 0.0f;
    target_ = position_;
    mode_ = MODE_VELOCITY;
  }

  /// Call continuously. Returns true on the iterations where a step was issued.
  bool run() {
    if (!enabled_) return false;

    // Unsigned subtraction is modular, so this stays correct across the ~71
    // minute micros() rollover. Do not "fix" it with a now < last branch;
    // that is what actually breaks it.
    const unsigned long now = micros();
    float dt = (now - lastUpdateUs_) * 1e-6f;
    lastUpdateUs_ = now;
    if (dt <= 0.0f) dt = 1e-6f;
    if (dt > 0.05f) dt = 0.05f;  // ignore huge gaps (e.g. after a long print)

    float desired;
    if (mode_ == MODE_POSITION) {
      const long dist = target_ - position_;
      if (dist == 0) {
        desired = 0.0f;
      } else {
        // Fastest speed we can still decelerate from within `dist` steps.
        const float vDecel = sqrtf(2.0f * accel_ * fabsf((float)dist));
        const float v = vDecel < maxSpeed_ ? vDecel : maxSpeed_;
        desired = (dist > 0) ? v : -v;
      }
    } else {
      desired = targetVelocity_;
    }

    const float maxDelta = accel_ * dt;
    float dv = desired - velocity_;
    if (dv > maxDelta) dv = maxDelta;
    if (dv < -maxDelta) dv = -maxDelta;
    velocity_ += dv;

    if (fabsf(velocity_) < kMinSpeed) {
      const bool creepToTarget = (mode_ == MODE_POSITION) && (target_ != position_);
      if (!creepToTarget) {
        velocity_ = 0.0f;
        return false;
      }
      velocity_ = (target_ > position_) ? kMinSpeed : -kMinSpeed;
    }

    const unsigned long interval = (unsigned long)(1000000.0f / fabsf(velocity_));
    if (now - lastStepUs_ < interval) return false;
    lastStepUs_ = now;

    const int8_t dir = (velocity_ > 0.0f) ? 1 : -1;
    pulse_(dir);
    position_ += dir;
    return true;
  }

  long position() const { return position_; }
  long target() const { return target_; }
  long distanceToGo() const { return target_ - position_; }
  float velocity() const { return velocity_; }
  float maxSpeed() const { return maxSpeed_; }
  bool isMoving() const { return velocity_ != 0.0f || distanceToGo() != 0; }

  /// Declare the current physical position to be `steps` (usually 0 at home).
  void setPosition(long steps) {
    position_ = steps;
    target_ = steps;
    velocity_ = 0.0f;
  }

 private:
  void pulse_(int8_t dir) {
    const bool high = (dir > 0) != invertDir_;
    digitalWrite(dirPin_, high ? HIGH : LOW);
    delayMicroseconds(2);  // DIR setup time
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(3);  // TMC2209 needs >100 ns; 3 us is safe on an Uno
    digitalWrite(stepPin_, LOW);
  }

  static constexpr float kMinSpeed = 1.0f;  // steps/s floor, avoids /0

  uint8_t stepPin_, dirPin_, enPin_;
  bool enActiveLow_;
  bool enabled_ = false;
  bool invertDir_ = false;

  Mode mode_ = MODE_POSITION;
  long position_ = 0;
  long target_ = 0;
  float velocity_ = 0.0f;
  float targetVelocity_ = 0.0f;
  float maxSpeed_ = 800.0f;
  float accel_ = 2000.0f;

  unsigned long lastUpdateUs_ = 0;
  unsigned long lastStepUs_ = 0;
};
