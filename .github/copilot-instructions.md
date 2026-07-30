# Copilot instructions: Desktop 6-DOF Arm

## What this project is
Firmware + tooling for a ground-up desktop 6-DOF robotic manipulator. It is a
**learning project**: the owner's goal is to build robotics/controls depth for a
career in robotics R&D. Favor explanations of *why* over just producing code.

## Hardware (Phase 1)
- Arduino Uno, PlatformIO
- NEMA 17 steppers (1.5 A, 42 N·cm) driven by TMC2209 in step/dir mode
- AS5600 12-bit absolute magnetic encoders behind a TCA9548A I2C mux
  (all AS5600s share address 0x36, hence the mux)
- 30 V 10 A bench PSU, Bambu P1S for PETG/TPU parts

## Target mechanical architecture
- **Universal Robots-style 6R arm**, motors in the joints, **offset
  (non-spherical) wrist**. J2/J3/J4 are parallel, so closed-form IK still
  applies via Pieper's parallel-axis branch - do not assume numeric IK is
  required.
- **Every joint is a 42 mm NEMA 17 face + printed cycloidal reducer, 20:1 to
  26:1.** Torque is tiered by **motor body length** (60 mm shoulder / 40 mm
  mid-arm / 23 mm wrist) and by ratio, never by changing the joint envelope.
  Consequences: ~89 steps/output-deg, so the encoder (not the motor) is the
  accuracy floor, and aggregate step-rate demand outgrows the Uno fast.
- The gripper is a 9 g hobby servo on a printed rack and pinion. It is **not** a
  kinematic joint - no encoder, no PID, no mux channel. Note the Arduino `Servo`
  library claims Timer1 on the ATmega328P.
- Rationale and tradeoffs live in `docs/design-decisions.md`. Append new
  decisions there rather than burying them in code comments.

## Repo conventions
- **One firmware app per bring-up step** in `src/apps/`, each wired to its own
  PlatformIO environment via `build_src_filter`. Never add a second `setup()`
  to an existing app; add a new app + env.
- Reusable, hardware-facing code goes in `lib/` as header-only classes.
- **Pure math with no Arduino dependency goes in `lib/ArmMath`** so it stays
  unit-testable via `pio test -e native_tests` / `-e uno_tests`.
- All tunable values live in `include/pins.h` (wiring) and
  `include/joint_config.h` (mechanics, limits, gains). Do not hard-code them in
  apps.
- Telemetry is CSV on serial with a header line; comment/status lines start with
  `#` so `scripts/serial_logger.py` can split them out.

## Firmware rules
- **No `delay()` in control paths.** `StepperDriver::run()` and
  `AS5600Encoder::read()` are called every loop iteration; blocking breaks both
  step timing and telemetry. `app_04_calibration` is the one deliberate
  exception, and it says so.
- Target is an ATmega328P: 2 KB RAM. Use `F()` for string literals, prefer
  `float` over `double`, avoid `String`, avoid dynamic allocation.
- avr-libc `sscanf` has no `%f` by default - use `strtod`/`atof`.
- Anything that can move the motor must have a fault path that disables the
  driver (encoder loss, soft limits).

## When making changes
- Verify with `pio run -e <env>` before claiming success; check RAM/flash usage
  in the output.
- Update `include/pins.h` and
  `hardware/pinouts/uno-tmc2209-as5600-tca9548a.md` together.
- New capability generally means a new phase entry in `docs/phase-plan.md` with
  a measurable exit criterion.

## Tone
Concise and technical. When a design choice has a tradeoff (mux vs SPI encoders,
open vs closed loop, microstep count vs step rate), name the tradeoff rather
than silently picking one.
