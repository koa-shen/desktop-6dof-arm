#pragma once
// Tiny non-blocking line reader for the serial command interfaces.
#include <Arduino.h>

template <uint8_t kMaxLen = 48>
class SerialCli {
 public:
  /// Returns a pointer to a complete, null-terminated line, or nullptr.
  const char *poll(Stream &s = Serial) {
    while (s.available()) {
      const char c = (char)s.read();
      if (c == '\r') continue;
      if (c == '\n') {
        buf_[len_] = '\0';
        len_ = 0;
        return buf_;
      }
      if (len_ < kMaxLen - 1) buf_[len_++] = c;
    }
    return nullptr;
  }

 private:
  char buf_[kMaxLen];
  uint8_t len_ = 0;
};
