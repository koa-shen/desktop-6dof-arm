#pragma once
// AS5600 12-bit magnetic absolute encoder, read through a TCA9548A channel.
//
// Angle pipeline:
//   raw counts -> mechanical deg -> (zero offset, direction) -> unwrapped
//   continuous deg -> optional EMA filter
#include <Arduino.h>
#include <Wire.h>

#include "ArmMath.h"
#include "TCA9548A.h"

class AS5600Encoder {
 public:
  // Register map (datasheet section 6)
  static constexpr uint8_t kRegStatus = 0x0B;
  static constexpr uint8_t kRegRawAngle = 0x0C;  // 0x0C..0x0D
  static constexpr uint8_t kRegAngle = 0x0E;     // filtered/scaled
  static constexpr uint8_t kRegAgc = 0x1A;
  static constexpr uint8_t kRegMagnitude = 0x1B;  // 0x1B..0x1C

  // STATUS bits
  static constexpr uint8_t kStatusMagnetHigh = 0x08;  // magnet too close
  static constexpr uint8_t kStatusMagnetLow = 0x10;   // magnet too far
  static constexpr uint8_t kStatusMagnetDet = 0x20;   // magnet detected

  AS5600Encoder(TCA9548A *mux, uint8_t muxChannel, uint8_t address = 0x36,
                TwoWire &wire = Wire)
      : mux_(mux), channel_(muxChannel), addr_(address), wire_(wire) {}

  /// Verifies the chip answers and primes the unwrapper. Call after Wire.begin().
  bool begin() {
    if (!readStatus()) return false;
    if (!read()) return false;
    unwrapper_.reset(mechanicalDeg_);
    filtered_ = continuousDeg_ = 0.0f;
    return true;
  }

  /// One angle sample. Returns false on any bus error (errors are counted).
  bool read() {
    uint8_t buf[2];
    if (!readRegs_(kRegRawAngle, buf, 2)) {
      errors_++;
      return false;
    }
    rawCounts_ = ((uint16_t)buf[0] << 8 | buf[1]) & 0x0FFF;
    mechanicalDeg_ = armmath::countsToDeg(rawCounts_);

    const float zeroed =
        armmath::wrap360(direction_ * (mechanicalDeg_ - zeroDeg_));
    continuousDeg_ = unwrapper_.update(zeroed);
    if (primedFilter_) {
      filtered_ = armmath::ema(filtered_, continuousDeg_, filterAlpha_);
    } else {
      filtered_ = continuousDeg_;
      primedFilter_ = true;
    }
    lastReadMs_ = millis();
    return true;
  }

  /// Reads STATUS/AGC/MAGNITUDE. Slow-ish; call on demand, not in the hot loop.
  bool readStatus() {
    uint8_t s;
    if (!readRegs_(kRegStatus, &s, 1)) {
      errors_++;
      return false;
    }
    status_ = s;
    uint8_t agc;
    if (readRegs_(kRegAgc, &agc, 1)) agc_ = agc;
    uint8_t m[2];
    if (readRegs_(kRegMagnitude, m, 2)) {
      magnitude_ = ((uint16_t)m[0] << 8 | m[1]) & 0x0FFF;
    }
    return true;
  }

  /// Treat the current physical position as 0 deg.
  void zeroHere() {
    zeroDeg_ = mechanicalDeg_;
    unwrapper_.reset(0.0f);
    continuousDeg_ = filtered_ = 0.0f;
    primedFilter_ = true;
  }

  void setZeroOffsetDeg(float deg) { zeroDeg_ = armmath::wrap360(deg); }
  void setDirection(int8_t dir) { direction_ = (dir < 0) ? -1 : 1; }
  /// 1.0 = no filtering. ~0.2-0.4 is a good starting point under motor noise.
  void setFilterAlpha(float alpha) {
    filterAlpha_ = armmath::clampf(alpha, 0.01f, 1.0f);
  }

  uint16_t rawCounts() const { return rawCounts_; }
  float mechanicalDeg() const { return mechanicalDeg_; }
  float angleDeg() const { return filtered_; }        // continuous, filtered
  float angleRawDeg() const { return continuousDeg_; }  // continuous, unfiltered
  float zeroOffsetDeg() const { return zeroDeg_; }
  int8_t direction() const { return direction_; }
  int32_t turns() const { return unwrapper_.turns(); }
  uint32_t lastReadMs() const { return lastReadMs_; }
  uint16_t errorCount() const { return errors_; }
  void clearErrors() { errors_ = 0; }

  uint8_t status() const { return status_; }
  uint8_t agc() const { return agc_; }
  uint16_t magnitude() const { return magnitude_; }
  bool magnetDetected() const { return status_ & kStatusMagnetDet; }
  bool magnetTooWeak() const { return status_ & kStatusMagnetLow; }
  bool magnetTooStrong() const { return status_ & kStatusMagnetHigh; }
  bool magnetOk() const {
    return magnetDetected() && !magnetTooWeak() && !magnetTooStrong();
  }
  /// Human-readable magnet state for logs.
  const __FlashStringHelper *magnetText() const {
    if (!magnetDetected()) return F("NO_MAGNET");
    if (magnetTooWeak()) return F("TOO_FAR");
    if (magnetTooStrong()) return F("TOO_CLOSE");
    return F("OK");
  }

  uint8_t muxChannel() const { return channel_; }

 private:
  bool readRegs_(uint8_t reg, uint8_t *out, uint8_t len) {
    if (mux_ && !mux_->select(channel_)) return false;
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) return false;  // repeated start
    if (wire_.requestFrom((uint8_t)addr_, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) out[i] = wire_.read();
    return true;
  }

  TCA9548A *mux_;
  uint8_t channel_;
  uint8_t addr_;
  TwoWire &wire_;

  armmath::AngleUnwrapper unwrapper_;
  uint16_t rawCounts_ = 0;
  float mechanicalDeg_ = 0.0f;
  float continuousDeg_ = 0.0f;
  float filtered_ = 0.0f;
  float filterAlpha_ = 1.0f;
  bool primedFilter_ = false;
  float zeroDeg_ = 0.0f;
  int8_t direction_ = 1;

  uint8_t status_ = 0;
  uint8_t agc_ = 0;
  uint16_t magnitude_ = 0;
  uint32_t lastReadMs_ = 0;
  uint16_t errors_ = 0;
};
