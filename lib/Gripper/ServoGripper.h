#pragma once
// Open-loop servo gripper with compliant fingers (D6, D11).
//
// This is NOT a kinematic joint: no encoder, no PID, no mux channel, no entry
// in the DH table. It is a binary-ish actuator hanging off the tool flange.
//
// WHY OPEN LOOP IS THE RIGHT ANSWER HERE, NOT A SHORTCUT
// A 9 g hobby servo reports nothing - no position, no current, no torque. So
// "did I actually grab the part?" is unanswerable in software. The two ways
// out are (a) add current sensing and infer grasp from stall, or (b) make the
// MECHANISM tolerant of position error so grasp does not need to be measured.
// Compliant TPU fingers are option (b): command a close angle a few degrees
// PAST first contact, and the fingers deflect to take up the difference. The
// grip force is then set by finger stiffness and overtravel - both of which
// are print parameters you can tune - instead of by a control loop.
//
// The cost, stated plainly: the arm cannot tell a successful grasp from a
// missed one. Detecting that needs either current sensing or a camera, and
// that is a Phase 6 decision, not a Phase 3 one.
//
// TIMER NOTE: Arduino's Servo library claims Timer1 on the ATmega328P and
// interrupts every ~20 ms per attached servo. StepperDriver generates steps in
// software off micros() (Timer0), so they coexist, but the ISR does add jitter
// to step timing. detachWhenIdle() exists for exactly that: once the fingers
// have stopped moving the servo is detached, the ISR stops firing, and the
// held position is maintained by the gearbox and the part itself.
#include <Arduino.h>
#include <Servo.h>

#include "ArmMath.h"

class ServoGripper {
 public:
  struct Config {
    uint8_t pin;
    float open_deg;      // fully open
    float closed_deg;    // nominal contact with the part
    float grip_deg;      // past contact; the overtravel that becomes grip force
    float slew_deg_s;    // travel rate; slow enough not to knock the part over
    uint16_t settle_ms;  // extra dwell after the setpoint stops moving
    bool detach_idle;    // release Timer1 once settled
  };

  void begin(const Config& cfg) {
    cfg_ = cfg;
    if (cfg_.slew_deg_s <= 0.0f) cfg_.slew_deg_s = 60.0f;
    current_ = target_ = cfg_.open_deg;
    attach_();
    servo_.write(static_cast<int>(current_ + 0.5f));
    lastUpdateMs_ = millis();
    settledAtMs_ = lastUpdateMs_;
  }

  void open() { setAngle(cfg_.open_deg); }
  /// Close onto a part: overtravel past contact so the fingers load up.
  void grip() { setAngle(cfg_.grip_deg); }
  /// Close to nominal contact with no squeeze. Useful while calibrating.
  void close() { setAngle(cfg_.closed_deg); }

  void setAngle(float deg) {
    const float lo = min(cfg_.open_deg, cfg_.grip_deg);
    const float hi = max(cfg_.open_deg, cfg_.grip_deg);
    const float clamped = armmath::clampf(deg, lo, hi);
    if (clamped != target_) {
      target_ = clamped;
      settled_ = false;
      attach_();
    }
  }

  /// 0 = open, 100 = fully gripped. What the host protocol sends.
  void setPercent(float pct) {
    const float p = armmath::clampf(pct, 0.0f, 100.0f) * 0.01f;
    setAngle(cfg_.open_deg + p * (cfg_.grip_deg - cfg_.open_deg));
  }

  /// Non-blocking; call every loop. Never uses delay() - the step generator
  /// and the encoder stream are both running underneath this.
  void update() {
    const uint32_t now = millis();
    const float dt = (now - lastUpdateMs_) * 0.001f;
    if (dt <= 0.0f) return;
    lastUpdateMs_ = now;

    if (current_ != target_) {
      current_ = armmath::slew(current_, target_, cfg_.slew_deg_s * dt);
      if (attached_) servo_.write(static_cast<int>(current_ + 0.5f));
      settledAtMs_ = now;
      return;
    }

    // Setpoint has arrived. The MECHANISM has not: the fingers are still
    // deflecting and the servo is still driving toward its command.
    if (!settled_ && now - settledAtMs_ >= cfg_.settle_ms) {
      settled_ = true;
      if (cfg_.detach_idle) detach_();
    }
  }

  /// True once the commanded travel is complete and the dwell has elapsed.
  /// This is a TIMEOUT, not a measurement - it says the gripper was given
  /// enough time to close, never that it closed onto anything.
  bool settled() const { return settled_; }
  bool isGripping() const { return settled_ && target_ != cfg_.open_deg; }
  float angleDeg() const { return current_; }

 private:
  void attach_() {
    if (!attached_) {
      servo_.attach(cfg_.pin);
      attached_ = true;
    }
  }
  void detach_() {
    if (attached_) {
      servo_.detach();
      attached_ = false;
    }
  }

  Servo servo_;
  Config cfg_{};
  float current_ = 0.0f;
  float target_ = 0.0f;
  uint32_t lastUpdateMs_ = 0;
  uint32_t settledAtMs_ = 0;
  bool attached_ = false;
  bool settled_ = true;
};
