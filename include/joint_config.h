#pragma once
// Mechanical and motion parameters for the joint under test.
// The `calibration` app measures most of these for you and prints a block you
// can paste straight back into this file.
#include <stdint.h>

// ------------------------------------------------------------- Kinematics --
constexpr float MOTOR_FULL_STEPS_PER_REV = 200.0f;  // 1.8 deg NEMA 17

// TMC2209 microstepping set by MS1/MS2 jumpers.
//   MS2 MS1 -> microsteps
//    L   L  -> 8      (many StepStick clones default here)
//    L   H  -> 32
//    H   L  -> 64
//    H   H  -> 16
// Stay at 8 once the cycloidal reducers are in: at 20:1 the encoder is already
// ~8x coarser than one step, so extra microsteps buy nothing but step rate cost.
constexpr float MICROSTEPS = 8.0f;

// Motor revolutions per output revolution. 1.0 = direct drive (Phase 1 bench).
// A 20:1 cycloidal reducer -> 20.0f. Measure it with the `calibration` app
// rather than trusting the design value; printed reducers rarely hit nominal.
constexpr float GEAR_RATIO = 1.0f;

constexpr float STEPS_PER_MOTOR_REV = MOTOR_FULL_STEPS_PER_REV * MICROSTEPS;
constexpr float STEPS_PER_OUTPUT_REV = STEPS_PER_MOTOR_REV * GEAR_RATIO;
constexpr float STEPS_PER_OUTPUT_DEG = STEPS_PER_OUTPUT_REV / 360.0f;

// +1 if encoder angle increases when commanded steps increase, else -1.
// The calibration app reports the correct value.
constexpr int8_t ENCODER_DIRECTION = +1;

// --------------------------------------------------------------- Limits ---
// Soft limits in output degrees, relative to the zero you set at homing.
constexpr float JOINT_MIN_DEG = -90.0f;
constexpr float JOINT_MAX_DEG = 90.0f;

// ---------------------------------------------------------------- Motion ---
// Start slow. Raise only after the joint runs without skipping steps.
constexpr float MAX_SPEED_STEPS_PER_SEC = 1600.0f;
constexpr float ACCEL_STEPS_PER_SEC2 = 4000.0f;

// ------------------------------------------------------------ Closed loop --
constexpr float PID_KP = 40.0f;  // steps/sec per degree of error
constexpr float PID_KI = 0.0f;
constexpr float PID_KD = 0.0f;
constexpr float PID_INTEGRAL_LIMIT = 400.0f;  // anti-windup, steps/sec
constexpr float POSITION_TOLERANCE_DEG = 0.3f;
constexpr uint16_t CONTROL_PERIOD_MS = 5;  // 200 Hz control loop

// ------------------------------------------------------------- Telemetry ---
constexpr uint16_t LOG_PERIOD_MS = 20;  // 50 Hz CSV stream
