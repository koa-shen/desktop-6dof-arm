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
// UNITS MATTER HERE. JointController runs the loop in ENCODER COUNTS: the PID
// sees an error in counts and produces counts/s, which is then multiplied by
// steps_per_count. So Kp has units of 1/s, and its value is INDEPENDENT of
// gear ratio and microstepping - change either and this number stays valid.
// A gain expressed in "steps/s per degree" silently becomes wrong the day a
// reducer goes in, which is exactly the kind of bug that reads as mechanical.
//
// app_05 runs the same loop in degrees for hand tuning and converts on the way
// in; see the STEPS_PER_OUTPUT_DEG factor there.
//
// With feedforward doing the bulk of the work (D10), Kp only has to reject
// load droop and gear error, so it is deliberately far lower than it would be
// for a PID that has to produce the whole motion by itself.
constexpr float PID_KP = 8.0f;   // 1/s
constexpr float PID_KI = 0.0f;   // 1/s^2 - see the note below before raising
constexpr float PID_KD = 0.0f;   // dimensionless

// Anti-windup clamp on the integral itself, in count-seconds.
constexpr float PID_INTEGRAL_LIMIT = 50.0f;

// Ki is 0 on purpose. A stepper has none of the steady-state droop a DC motor
// has - it holds position by physics. Integral action here exists only to walk
// out backlash and gravity sag, and it is the term that turns a marginally
// stable loop into a buzzing one. Add it last, and only if you have MEASURED a
// persistent offset under load.

constexpr float POSITION_TOLERANCE_DEG = 0.3f;
constexpr uint16_t CONTROL_PERIOD_MS = 5;  // 200 Hz control loop

// In-position deadband, in ENCODER COUNTS. 1 count = 0.088 deg = 8 microsteps
// at 20:1, so the loop can command a correction eight times finer than the
// smallest error it can measure. Below this width the error sign flips on
// every encoder tick and the joint limit-cycles; 2 counts is one tick of
// hysteresis on either side of it. Raise it if the joint still buzzes at rest,
// but understand you are trading repeatability for silence.
constexpr uint8_t POSITION_DEADBAND_COUNTS = 2;

// Velocity feedforward trim. 1.0 means the gear ratio and microstep count in
// this file are believed exactly. If a constant-velocity move shows a constant
// following error, this is the number to correct - not Kp, and definitely not
// Ki, because the error is a model error and not a disturbance.
constexpr float VEL_FF_SCALE = 1.0f;

// --------------------------------------------------------------- Homing ---
// The encoder is on the OUTPUT side of the reducer (D4) and the AS5600 is
// absolute over one turn, so a joint whose travel is within +/-180 deg knows
// its angle at power-on. No homing move, no limit switch, no hard-stop crash
// with a printed gearbox in the load path.
//
// Commissioning: jog the joint to its mechanical zero, read the encoder's
// MECHANICAL angle (app_01 or the `?` command), and paste it here. This is the
// one number that must be re-measured whenever a magnet or a hub is disturbed.
constexpr bool ABSOLUTE_HOME = true;
constexpr float JOINT_HOME_OFFSET_DEG[3] = {0.0f, 0.0f, 0.0f};

// ---------------------------------------------------------- Trajectories --
// Limits used by the layer-1 profile generator, at the OUTPUT, in degrees.
// These MUST sit below the driver ceiling that MAX_SPEED_STEPS_PER_SEC and
// ACCEL_STEPS_PER_SEC2 imply, or the loop saturates and the resulting huge
// following error reads exactly like a badly tuned PID:
//     max deg/s   = MAX_SPEED_STEPS_PER_SEC / STEPS_PER_OUTPUT_DEG
//     max deg/s^2 = ACCEL_STEPS_PER_SEC2    / STEPS_PER_OUTPUT_DEG
// At 20:1 and 8 microsteps that is 18 deg/s and 45 deg/s^2, hence 15 and 40.
// Re-derive both when GEAR_RATIO changes - they do not scale on their own.
constexpr float TRAJ_MAX_VEL_DEG_S = 15.0f;
constexpr float TRAJ_MAX_ACC_DEG_S2 = 40.0f;

// Boxcar jerk limiter width, in control cycles. jmax = amax / (TAPS * dt), and
// the move lengthens by exactly TAPS * dt. At 200 Hz, 4 taps = 20 ms of
// smoothing for 20 ms of added time. Set to 1 to disable.
constexpr uint8_t JERK_FILTER_TAPS = 4;

// ---------------------------------------------------------------- Gripper --
// Open-loop servo with compliant TPU fingers (D6, D11). GRIPPER_GRIP_DEG is
// past first contact on purpose: the overtravel deflects the fingers, and that
// deflection is the grip force. Set GRIPPER_CLOSED_DEG to where the fingers
// first touch the part, then add overtravel until the part does not slip.
constexpr float GRIPPER_OPEN_DEG = 20.0f;
constexpr float GRIPPER_CLOSED_DEG = 85.0f;
constexpr float GRIPPER_GRIP_DEG = 100.0f;
constexpr float GRIPPER_SLEW_DEG_S = 120.0f;
constexpr uint16_t GRIPPER_SETTLE_MS = 250;
// Detach the servo once settled: Arduino's Servo library interrupts every
// 20 ms on Timer1, and that ISR adds jitter to software step generation.
constexpr bool GRIPPER_DETACH_IDLE = true;

// ------------------------------------------------------------- Telemetry ---
constexpr uint16_t LOG_PERIOD_MS = 20;  // 50 Hz CSV stream
