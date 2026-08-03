#pragma once
// Byte-stream framing for the host link: COBS + CRC-8, with a delimiter that
// can never appear inside a frame.
//
// WHY NOT JUST PRINT CSV
// CSV over serial is fine for watching a bench test and useless for driving
// one. It has no way to resynchronize after a dropped byte, no way to detect a
// corrupted field, and it turns an 8-byte command into ~30 bytes of ASCII plus
// a float parse on a 16 MHz AVR. The moment a host is streaming setpoints at
// 200 Hz, all three of those become the bottleneck.
//
// COBS (consistent overhead byte stuffing) removes every zero byte from the
// payload, which frees 0x00 to mean exactly one thing: end of frame. A
// receiver that gets lost only has to scan forward to the next zero. Overhead
// is one byte per frame plus one per 254 bytes - constant and known, unlike
// escape-based schemes whose worst case doubles the frame.
//
// Wire format:  COBS( type | node | payload... | crc8 ) 0x00
// CRC-8 covers type, node and payload. It catches the corruption that a UART
// framing error does not, which is the case that actually bites on a long
// unshielded USB cable next to a stepper.
//
// No Arduino dependency: the same code is unit-tested on the host and mirrored
// byte-for-byte by tools/joint_link.py.
#include <stdint.h>

namespace link {

// Messages that are destined for CAN one day must keep their payload at 8
// bytes (D7). Anything larger is host-link only and must never be given a CAN
// id.
constexpr uint8_t kMaxPayload = 16;
constexpr uint8_t kMaxBody = kMaxPayload + 3;   // type + node + payload + crc
constexpr uint8_t kMaxFrame = kMaxBody + 2;     // COBS overhead + delimiter

enum PacketType : uint8_t {
  PKT_CMD = 0x01,      // host -> arm, node = joint index, payload = JointCommand
  PKT_STATE = 0x02,    // arm -> host, node = joint index, payload = JointState
  PKT_GRIPPER = 0x03,  // host -> arm, payload[0] = percent closed
  PKT_ESTOP = 0x04,    // either direction, no payload
  PKT_CLEAR = 0x05,    // host -> arm, clear latched faults
  PKT_PING = 0x06,
  PKT_PONG = 0x07,
  PKT_LOG = 0x08,      // arm -> host, ASCII, not for CAN
};

/// CRC-8/ATM, polynomial 0x07. Table-free: 8 shifts per byte is nothing next
/// to the I2C read this shares a loop with, and a 256-byte table is 12 % of
/// the ATmega328P's RAM if it lands there by accident.
inline uint8_t crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                         : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

/// Returns encoded length. `out` needs len + len/254 + 1 bytes.
inline uint8_t cobsEncode(const uint8_t* in, uint8_t len, uint8_t* out) {
  uint8_t readIdx = 0, writeIdx = 1, codeIdx = 0, code = 1;
  while (readIdx < len) {
    if (in[readIdx] == 0) {
      out[codeIdx] = code;
      codeIdx = writeIdx++;
      code = 1;
      readIdx++;
    } else {
      out[writeIdx++] = in[readIdx++];
      if (++code == 0xFF) {
        out[codeIdx] = code;
        codeIdx = writeIdx++;
        code = 1;
      }
    }
  }
  out[codeIdx] = code;
  return writeIdx;
}

/// Returns decoded length, or 0 if the input is not valid COBS.
inline uint8_t cobsDecode(const uint8_t* in, uint8_t len, uint8_t* out,
                          uint8_t outCapacity) {
  uint8_t readIdx = 0, writeIdx = 0;
  while (readIdx < len) {
    const uint8_t code = in[readIdx];
    if (code == 0 || readIdx + code > len) return 0;
    readIdx++;
    for (uint8_t i = 1; i < code; ++i) {
      if (writeIdx >= outCapacity) return 0;
      out[writeIdx++] = in[readIdx++];
    }
    if (code < 0xFF && readIdx < len) {
      if (writeIdx >= outCapacity) return 0;
      out[writeIdx++] = 0;
    }
  }
  return writeIdx;
}

/// Build a complete frame including the trailing 0x00. Returns frame length,
/// or 0 if the payload is too long.
inline uint8_t buildFrame(uint8_t type, uint8_t node, const uint8_t* payload,
                          uint8_t payloadLen, uint8_t* out) {
  if (payloadLen > kMaxPayload) return 0;
  uint8_t body[kMaxBody];
  body[0] = type;
  body[1] = node;
  for (uint8_t i = 0; i < payloadLen; ++i) body[2 + i] = payload[i];
  const uint8_t n = static_cast<uint8_t>(2 + payloadLen);
  body[n] = crc8(body, n);
  const uint8_t enc = cobsEncode(body, static_cast<uint8_t>(n + 1), out);
  out[enc] = 0x00;
  return static_cast<uint8_t>(enc + 1);
}

/// Streaming receiver. Feed it one byte at a time; it returns true on the byte
/// that completes a frame whose CRC checks out.
class PacketReader {
 public:
  bool feed(uint8_t b) {
    if (b != 0x00) {
      if (rawLen_ < sizeof(raw_)) {
        raw_[rawLen_++] = b;
      } else {
        overrun_ = true;  // frame too long: drop it, resync on the next zero
      }
      return false;
    }

    const uint8_t rawLen = rawLen_;
    const bool overrun = overrun_;
    rawLen_ = 0;
    overrun_ = false;
    if (rawLen == 0) return false;  // back-to-back delimiters
    if (overrun) {
      dropped_++;
      return false;
    }

    const uint8_t n = cobsDecode(raw_, rawLen, body_, sizeof(body_));
    if (n < 3) {  // type + node + crc is the shortest legal frame
      dropped_++;
      return false;
    }
    if (crc8(body_, static_cast<uint8_t>(n - 1)) != body_[n - 1]) {
      crcErrors_++;
      return false;
    }
    payloadLen_ = static_cast<uint8_t>(n - 3);
    return true;
  }

  uint8_t type() const { return body_[0]; }
  uint8_t node() const { return body_[1]; }
  const uint8_t* payload() const { return body_ + 2; }
  uint8_t payloadLen() const { return payloadLen_; }

  uint16_t crcErrors() const { return crcErrors_; }
  uint16_t dropped() const { return dropped_; }

 private:
  uint8_t raw_[kMaxFrame] = {};
  uint8_t body_[kMaxBody] = {};
  uint8_t rawLen_ = 0;
  uint8_t payloadLen_ = 0;
  uint16_t crcErrors_ = 0;
  uint16_t dropped_ = 0;
  bool overrun_ = false;
};

}  // namespace link
