#pragma once
// TCA9548A 8-channel I2C multiplexer.
// Every AS5600 shares address 0x36, so exactly one mux channel may be open
// when talking to an encoder.
#include <Arduino.h>
#include <Wire.h>

class TCA9548A {
 public:
  static constexpr uint8_t kNoChannel = 0xFF;

  explicit TCA9548A(uint8_t address = 0x70, TwoWire &wire = Wire)
      : addr_(address), wire_(wire) {}

  /// Does not call Wire.begin(); do that once in setup().
  bool begin() { return disableAll(); }

  bool isPresent() {
    wire_.beginTransmission(addr_);
    return wire_.endTransmission() == 0;
  }

  /// Opens exactly one channel. Repeated calls for the same channel are free.
  bool select(uint8_t channel, bool force = false) {
    if (channel > 7) return false;
    if (!force && channel == active_) return true;
    if (!write_(1 << channel)) {
      active_ = kNoChannel;
      return false;
    }
    active_ = channel;
    return true;
  }

  bool disableAll() {
    const bool ok = write_(0x00);
    active_ = ok ? kNoChannel : active_;
    return ok;
  }

  uint8_t activeChannel() const { return active_; }
  uint8_t address() const { return addr_; }

 private:
  bool write_(uint8_t mask) {
    wire_.beginTransmission(addr_);
    wire_.write(mask);
    return wire_.endTransmission() == 0;
  }

  uint8_t addr_;
  TwoWire &wire_;
  uint8_t active_ = kNoChannel;
};
