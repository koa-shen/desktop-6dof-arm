#pragma once
// Single source of truth for wiring. If you move a wire, change it here only.
// Matches hardware/pinouts/uno-tmc2209-as5600-tca9548a.md
#include <stdint.h>

// ---------------------------------------------------------------- Joints --
// TMC2209 step/dir interface (Arduino Uno digital pins).
// All drivers share ONE enable line: saves pins, and one write kills all
// motion. Do not split it per joint - a partial disable is a worse fault state
// than a full one.
constexpr uint8_t PIN_J1_STEP = 2;
constexpr uint8_t PIN_J1_DIR = 3;
constexpr uint8_t PIN_J2_STEP = 5;
constexpr uint8_t PIN_J2_DIR = 6;
constexpr uint8_t PIN_J3_STEP = 7;
constexpr uint8_t PIN_J3_DIR = 8;
constexpr uint8_t PIN_DRIVER_EN = 4;  // TMC2209 EN is active LOW
constexpr uint8_t PIN_J1_EN = PIN_DRIVER_EN;  // legacy alias, single-joint apps

// Gripper servo. D9 is Timer1-OC1A; the Arduino Servo library claims Timer1
// regardless, so keep hardware step generation off Timer1. See D6.
constexpr uint8_t PIN_GRIPPER_SERVO = 9;

// TMC2209 EN polarity. Leave true for stock StepStick-style boards.
constexpr bool DRIVER_EN_ACTIVE_LOW = true;

// ------------------------------------------------------------------- I2C --
// Uno hardware I2C is fixed: SDA = A4, SCL = A5.
constexpr uint8_t TCA9548A_ADDR = 0x70;  // 0x70..0x77 depending on A0/A1/A2
constexpr uint8_t AS5600_ADDR = 0x36;    // fixed, hence the mux

// Mux channel assigned to each joint encoder. Phase 1 uses J1 only.
constexpr uint8_t MUX_CH_J1 = 0;
constexpr uint8_t MUX_CH_J2 = 1;
constexpr uint8_t MUX_CH_J3 = 2;
constexpr uint8_t MUX_CH_J4 = 3;
constexpr uint8_t MUX_CH_J5 = 4;
constexpr uint8_t MUX_CH_J6 = 5;

// 400 kHz. Three encoders at 100 kHz cost ~1.9 ms per control cycle, which caps
// the loop near 500 Hz; 400 kHz brings it to ~0.5 ms. Drop back to 100000 if a
// long or unshielded cable starts throwing I2C errors.
constexpr uint32_t I2C_CLOCK_HZ = 400000;

// ------------------------------------------------------------------ Misc --
constexpr uint32_t SERIAL_BAUD = 115200;
