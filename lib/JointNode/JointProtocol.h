#pragma once
// The joint node contract: what a host asks a joint for, and what it hears back.
//
// WHY THIS EXISTS BEFORE THE HARDWARE DOES
// Today every joint lives in one Uno binary and this struct is passed by
// reference between objects in the same address space. Later each joint gets
// its own MCU and this struct is the CAN payload. Same struct, same units,
// same semantics - only the transport changes. That is the difference between
// "upgrade" and "rewrite".
//
// Both messages are EXACTLY 8 bytes so they fit one classic CAN 2.0 frame with
// no fragmentation. Do not add fields; repurpose reserved bits instead.
//
// No Arduino dependency: this compiles on the host so the Python tooling and
// the unit tests can speak the same protocol.
#include <stdint.h>

namespace jointnode {

// ---------------------------------------------------------------- units --
// Position is in ENCODER COUNTS at the joint output, not degrees and not motor
// steps. Counts are what the sensor actually measures; degrees introduce a
// float and motor steps change every time the gear ratio does.
constexpr int32_t kCountsPerRev = 4096;  // AS5600 12-bit
constexpr float kDegPerCount = 360.0f / kCountsPerRev;

constexpr int32_t degToCounts(float deg) {
  return static_cast<int32_t>(deg / kDegPerCount + (deg < 0 ? -0.5f : 0.5f));
}
constexpr float countsToDeg(int32_t counts) { return counts * kDegPerCount; }

// ----------------------------------------------------------------- modes --
enum Mode : uint8_t {
  MODE_IDLE = 0,      // driver disabled, joint free
  MODE_HOLD = 1,      // driver enabled, hold current position
  MODE_POSITION = 2,  // servo to target
  MODE_VELOCITY = 3,  // track target velocity, ignore target position
};

// --------------------------------------------------------------- cmd bits --
constexpr uint8_t CMD_ENABLE = 1 << 0;       // arm the driver
constexpr uint8_t CMD_CLEAR_FAULT = 1 << 1;  // latched faults reset on rising edge
constexpr uint8_t CMD_ZERO_HERE = 1 << 2;    // define current position as zero

// ------------------------------------------------------------- fault bits --
// Any nonzero fault disables the driver. Faults latch until CMD_CLEAR_FAULT.
constexpr uint8_t FAULT_NONE = 0;
constexpr uint8_t FAULT_ENCODER = 1 << 0;   // I2C error or magnet lost
constexpr uint8_t FAULT_SOFT_LIMIT = 1 << 1;
constexpr uint8_t FAULT_FOLLOWING = 1 << 2; // commanded and measured diverged
constexpr uint8_t FAULT_COMMS = 1 << 3;     // no command within the timeout
constexpr uint8_t FAULT_NOT_HOMED = 1 << 4;
constexpr uint8_t FAULT_DRIVER = 1 << 5;

// ------------------------------------------------------------- the frames --
struct JointCommand {
  int32_t target_counts;  // absolute, multi-turn
  uint16_t max_vel;       // counts/s, 0 = use the node's configured default
  uint8_t mode;           // Mode
  uint8_t flags;          // CMD_*
};

struct JointState {
  int32_t pos_counts;   // measured, multi-turn
  int16_t vel_counts_s; // measured, signed
  uint8_t fault;        // FAULT_* bitfield, 0 = healthy
  uint8_t seq;          // increments per publish; lets the host detect drops
};

static_assert(sizeof(JointCommand) == 8, "JointCommand must fit one CAN frame");
static_assert(sizeof(JointState) == 8, "JointState must fit one CAN frame");

// ------------------------------------------------------------ addressing --
// CAN 11-bit IDs, laid out so a joint's command outranks its state report and
// the estop outranks everything. Lower ID = higher bus priority.
constexpr uint16_t kIdEstop = 0x000;
constexpr uint16_t kIdCmdBase = 0x100;    // 0x100 + joint index
constexpr uint16_t kIdStateBase = 0x200;  // 0x200 + joint index
constexpr uint8_t kMaxJoints = 6;

constexpr uint16_t cmdId(uint8_t joint) { return kIdCmdBase + joint; }
constexpr uint16_t stateId(uint8_t joint) { return kIdStateBase + joint; }

}  // namespace jointnode
