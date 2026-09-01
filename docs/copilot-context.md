# Copilot Context: Desktop 6-DOF Arm

## Project goal
Design and manufacture a desktop 6-DOF robotic manipulator arm and program it to move payloads across a desk.

## Mechanical architecture
- **Universal Robots-style 6R layout.** All motors housed in the joints. Axis
  order: J1 vertical base, J2/J3/J4 parallel horizontal (shoulder, elbow,
  wrist 1), J5 perpendicular, J6 along the tool axis.
- **Offset (non-spherical) wrist.** Chosen for buildability. Because J2/J3/J4
  are parallel, Pieper's criterion is still satisfied via the parallel-axis
  branch, so **closed-form IK is retained** - see `docs/design-decisions.md`.
- **Every joint is NEMA 17 + printed cycloidal reducer** constrained to the
  42 x 42 mm motor-face footprint. Ratio follows a feasible cycloid, bearing,
  output-shaft, and clearance layout; the first prototype is 15:1-class (D19).
- Encoder magnet on the **output** side of the reducer, so the reading includes
  gearbox backlash.

## Current architecture decisions
- Actuation: NEMA 17 stepper motors (1.5A, 42 N·cm), TMC2209 drivers
- Position sensing: AS5600 magnetic absolute encoders
- Bus strategy (v1): TCA9548A I²C multiplexer + AS5600
- Future plan: migrate to more advanced encoder interface later (likely SPI/CAN architecture)

## Why this approach
- Keep costs low now, prove mechanics + firmware in Phase 1
- Avoid premature hardware spend
- Build software in a way that can migrate later

## Consequences of the cycloidal reduction (important)
- At 20:1 and 8x microstepping: 32000 steps/output-rev = **88.9 steps/deg**.
- One step = 0.011 deg at the output, finer than the AS5600's 0.088 deg, so the
  **encoder is now the accuracy floor**. Do not raise microstepping for
  resolution; 14-bit SPI encoders matter sooner than originally planned.
- 60 deg/s on one joint needs 5300 steps/s. The Uno's aggregate ceiling is
  ~5 kHz, so the MCU upgrade (Teensy 4.1, hardware-timer stepping) is required
  earlier than a direct-drive design would need it.

## Available tools/supplies
- Bambu P1S (ABS/TPU/PLA/PETG)
- M3 fastener kit
- 30V 10A bench PSU
- Hand tools
- Elegoo Arduino Super Starter Kit (Uno included)
- VS Code + Arduino IDE installed

## Material guidance
- PETG for structural and encoder mounting parts
- TPU for strain relief/grommets
- Avoid PLA for warm load-bearing components near motors

## Phase 1 objective
Single-joint bench validation:
1. Motor spins reliably via TMC2209 step/dir
2. AS5600 reads through TCA9548A channel 0
3. Serial CSV output: `ms,cycle,cmd_steps,cmd_deg,enc_deg,err_deg,i2c_err`
4. Stability and repeatability checks

Full phase list with exit criteria: [phase-plan.md](phase-plan.md)

## Key constraints/notes
- AS5600 is single-turn absolute (0-360 deg); `ArmMath::AngleUnwrapper` turns
  that into a continuous multi-turn angle
- Magnet must be **diametrically** magnetized and mounted on the joint OUTPUT
  axis for a true absolute joint angle
- AS5600 I2C address conflict solved by the mux; only one channel open at a time
- Keep sensor wiring away from motor power wiring
- Shared ground required between logic and motor domains
- Uno is a 2 KB RAM / ~15 kstep/s ceiling; expect to migrate at 2-3 joints

## Repo structure
- `include/pins.h` - wiring: pins, I2C addresses, mux channels
- `include/joint_config.h` - mechanics, limits, PID gains
- `lib/ArmMath` - Arduino-free math, unit tested
- `lib/MuxTCA9548A`, `lib/EncoderAS5600`, `lib/MotorDriver`, `lib/Control`,
  `lib/SerialCli` - header-only hardware/control classes
- `src/apps/app_00..05` - one app per bring-up step, one PlatformIO env each
- `test/test_armmath` - unit tests (`pio test -e uno_tests` or `native_tests`)
- `scripts/serial_logger.py`, `scripts/analyze_log.py` - capture and analyze CSV
- `hardware/pinouts/uno-tmc2209-as5600-tca9548a.md` - wiring map
- `hardware/bom/bom.md` - BOM

## Status
- Phase 0 (software ready) complete: all six firmware apps build, tests pass
- Phase 1 blocked on hardware arrival

## Next immediate tasks
- Work [bringup-checklist.md](bringup-checklist.md) gate by gate on arrival day
- Run `calibration` and paste results into `include/joint_config.h`
- Record repeatability logs in `docs/test-results/`
- Meanwhile: torque budget + DH forward kinematics per
  [learning-roadmap.md](learning-roadmap.md)