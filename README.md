# Desktop 6-DOF Arm

Design and manufacture of a desktop 6-DOF robotic arm for moving payloads
across a desk. The public project snapshot is also available as
[`docs/arm-project-summary.json`](docs/arm-project-summary.json), intended as
a stable data source for portfolio pages and other integrations.

## At A Glance

| Area | Current information | Status |
| --- | --- | --- |
| Degrees of freedom | 6-axis manipulator | Project goal |
| Payload capacity | 0.5 kg useful payload; gripper/tool mass excluded | Provisional target; validate joint torque and structure |
| Reach | About 15 in (381 mm), base axis to tool center point at full extension | Provisional target; workspace still needs layout |
| Transfer time | Opposite sides of usable workspace in no more than 5 s | Provisional move-time target; trajectory/endpoints to define |
| End effector | 3D-printed rack-and-pinion gripper; pneumatic force control preferred, servo actuation as fallback | Concept, not selected or tested |
| Demo objects | Phone or compact camera are interesting candidates | Must fit within 0.5 kg payload including mount/adapters |
| Current phase | Phase 2: single-joint 15:1 cycloidal reducer characterization | In progress |
| Actuation | NEMA 17 stepper, 1.5 A / 42 N·cm, with TMC2209 driver | Architecture decision; confirm exact motor before final design |
| Position sensing | AS5600, currently mounted on the motor shaft | Bench setup; does not measure reducer output backlash or accuracy |
| Base support | AXK2542 needle thrust bearing, 25 mm bore | Ordered |
| Controller / supply | Arduino Uno; available 30 V, 10 A bench supply | Prototype equipment, not final multi-axis electronics or power budget |
| Reducer test settings | 15:1 assumed ratio, 60° output sweep, 3 cycles | Firmware settings; verify ratio and microstep hardware configuration |
| Other acceptance limits | Repeatability, accuracy, backlash, and torque limits | Not yet set |

The Phase 2 test plan measures reducer behavior before committing to a full
six-joint build. The new whole-arm targets are provisional design goals, not
validated capabilities. At 0.5 kg and 381 mm extension, the payload alone
applies about 1.87 N·m static torque at the shoulder; gripper/link mass and
dynamic loading add to that requirement.

## Tooling
- Arduino Uno, VS Code + PlatformIO
- Bambu P1S for printed parts; PETG is the current structural-material guidance
- M3 fastener kit and 30 V / 10 A bench supply available

## Current Work

- Complete the Phase 2 reducer sweep and save logs under `docs/test-results/`.
- Confirm `MICROSTEPS` in `src/main.cpp` matches the TMC2209 hardware setup.
- Run the load-cell/contact-switch backlash test after calibrating the HX711 at
	its 1–2 N test preload.
- Use measured reducer results and the arm use case to set performance limits.

See [`docs/copilot-context.md`](docs/copilot-context.md) for architecture and
phase details, [`docs/reducer-test-protocol.md`](docs/reducer-test-protocol.md)
for characterization steps, and [`docs/arm-design-workbook.xlsx`](docs/arm-design-workbook.xlsx)
for the BOM, targets, and parameterized geometry calculator.
