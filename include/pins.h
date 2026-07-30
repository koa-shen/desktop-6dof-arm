#pragma once
// Single source of truth for wiring. If you move a wire, change it here only.
// Matches hardware/pinouts/uno-tmc2209-as5600-tca9548a.md
#include <stdint.h>

// ---------------------------------------------------------------- Joint 1 --
// TMC2209 step/dir interface (Arduino Uno digital pins)
constexpr uint8_t PIN_J1_STEP = 2;
constexpr uint8_t PIN_J1_DIR = 3;
constexpr uint8_t PIN_J1_EN = 4;  // TMC2209 EN is active LOW

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

// 100 kHz first. Only raise to 400 kHz after long cables read clean.
constexpr uint32_t I2C_CLOCK_HZ = 100000;

// ------------------------------------------------------------------ Misc --
constexpr uint32_t SERIAL_BAUD = 115200;
