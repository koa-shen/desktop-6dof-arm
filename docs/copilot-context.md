# Copilot Context: Desktop 6-DOF Arm

## Project goal
Design and manufacture a desktop 6-DOF robotic manipulator arm and program it to move payloads across a desk.

## Current architecture decisions
- Actuation: NEMA 17 stepper motors (1.5A, 42 N·cm), TMC2209 drivers
- Position sensing: AS5600 magnetic absolute encoders
- Single motor/encoder testing now: AS5600 wired directly to the Uno I2C bus,
  no mux. TCA9548A mux is deferred until multiple steppers are being tested
  — not relevant until then
- Future plan: migrate to more advanced encoder interface later (likely SPI/CAN architecture)
- Base joint: AXK2542 needle thrust bearing (25mm bore) to support axial load
  and moment at the base

## Why this approach
- Keep costs low now, prove mechanics + firmware in Phase 1
- Avoid premature hardware spend
- Build software in a way that can migrate later

## Available tools/supplies
- Bambu P1S (ABS/TPU/PLA/PETG)
- M3 fastener kit
- 30V 10A bench PSU
- Hand tools
- Elegoo Arduino Super Starter Kit (Uno included)
- VS Code + Arduino IDE installed
- HX711 load cell amplifiers (ordered, for the load-cell backlash test) —
  paired with a 5kg kitchen-scale load cell (~49N max, but preload is only
  ~1-2N — calibrate `HX711_CALIBRATION_FACTOR` near that low end, not with
  the full 5kg, and measure at-rest noise floor before trusting
  `FORCE_ZERO_THRESHOLD_N` in `src/main.cpp`)
- AXK2542 needle thrust bearing (ordered, base joint axial/moment support)

## Material guidance
- PETG for structural and encoder mounting parts
- TPU for strain relief/grommets
- Avoid PLA for warm load-bearing components near motors

## Phase 1 objective (complete)
Single-joint bench validation:
1. Motor spins reliably via TMC2209 step/dir
2. AS5600 reads directly over I²C (no mux, single motor)
3. Serial CSV output: `ms,step_pos,angle_deg`
4. Stability and repeatability checks

## Phase 2 objective (current)
15:1 cycloidal reducer characterization (hand-turn smoothness confirmed;
manufactured units on bench now):
1. Motor + AS5600 combined firmware sweeps the motor shaft through a known
   range, logging `ms,step_pos,commanded_deg,measured_deg` — `measured_deg`
   is motor-shaft angle, used to catch skipped/missed steps, not gearbox
   output angle
2. Current backlash test uses the motor-shaft AS5600 plus a momentary output
  contact switch. It reports switch-dead-band plus mechanical backlash; a
  later output encoder can separate those effects. HX711 load-cell integration
  is deferred.
3. Manual bench checks: breakaway holding torque, backdrivability, no-load
   current/cogging correlation
4. See `docs/reducer-test-protocol.md` for the full procedure

## Key constraints/notes
- AS5600 is single-turn absolute (0–360°)
- Current build mounts the magnet on the **motor/stepper shaft**, not the
  gearbox output, due to mounting/wiring difficulty. This means encoder angle
  detects skipped/missed steps, while the switch test infers output dead band
  from switch closure/release timing. The switch's hysteresis and fixture
  compliance remain in the result. See `docs/reducer-test-protocol.md`.
- Only one motor/encoder right now, so no mux — AS5600 direct on A4/A5.
  Don't reintroduce the TCA9548A until multi-joint testing actually starts.
- Keep sensor wiring away from motor power wiring
- Shared ground required between logic and motor domains
- Gear ratio 15:1 assumed in firmware (`GEAR_RATIO`); update
  `MOTOR_STEPS_PER_REV`/`MICROSTEPS` in `src/main.cpp` to match actual
  TMC2209 microstep pin config before trusting commanded_deg

## Current code/files
- `src/main.cpp`: Switch-only backlash/dead-band firmware (motor + AS5600, no mux)
- `hardware/pinouts/uno-tmc2209-as5600-tca9548a.md`: wiring map
- `platformio.ini`: Uno config
- `docs/reducer-test-protocol.md`: bench SOP and run record
- `tools/capture-backlash.ps1`: serial capture and summary for backlash runs
- `docs/test-results/phase2-reducer-log-template.csv`: log header for reducer data

## Next immediate tasks
- Flash the switch-only firmware and complete the bench record
- Run valid switch dead-band measurements at three output angles
- Review control-trip spread versus dead-band mean before accepting a run
- Do manual breakaway-torque, backdrivability, and loaded holding checks