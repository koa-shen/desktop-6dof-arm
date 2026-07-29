# Copilot Context: Desktop 6-DOF Arm

## Project goal
Design and manufacture a desktop 6-DOF robotic manipulator arm and program it to move payloads across a desk.

## Current architecture decisions
- Actuation: NEMA 17 stepper motors (1.5A, 42 N·cm), TMC2209 drivers
- Position sensing: AS5600 magnetic absolute encoders
- Bus strategy (v1): TCA9548A I²C multiplexer + AS5600
- Future plan: migrate to more advanced encoder interface later (likely SPI/CAN architecture)

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

## Material guidance
- PETG for structural and encoder mounting parts
- TPU for strain relief/grommets
- Avoid PLA for warm load-bearing components near motors

## Phase 1 objective
Single-joint bench validation:
1. Motor spins reliably via TMC2209 step/dir
2. AS5600 reads through TCA9548A channel 0
3. Serial CSV output: `ms,step_pos,angle_deg`
4. Stability and repeatability checks

## Key constraints/notes
- AS5600 is single-turn absolute (0–360°)
- Must mount magnet on joint OUTPUT axis for true absolute joint angle
- AS5600 I²C address conflict solved by mux
- Keep sensor wiring away from motor power wiring
- Shared ground required between logic and motor domains

## Current code/files
- `src/main.cpp`: Phase 1 combined motor + encoder bench test
- `hardware/pinouts/uno-tmc2209-as5600-tca9548a.md`: wiring map
- `platformio.ini`: Uno config

## Next immediate tasks
- Complete Phase 1 bench run with mux in-loop
- Validate clean encoder stream under motor motion
- Record repeatability logs in `docs/test-results/`