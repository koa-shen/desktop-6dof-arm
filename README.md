# Desktop 6-DOF Arm

Phase-based development for a desktop robotic manipulator arm.

## Current phase
**Phase 1:** single-joint bench validation
- TMC2209 step/dir motor control
- AS5600 absolute angle read
- TCA9548A mux channel selection
- CSV serial logging (`ms,step_pos,angle_deg`)

## Tooling
- Arduino Uno
- VS Code + PlatformIO

## Next steps
1. Wire hardware per `hardware/pinouts/uno-tmc2209-as5600-tca9548a.md`
2. Build/upload with PlatformIO
3. Open serial monitor at 115200 baud
4. Verify angle changes while motor moves
