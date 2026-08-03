# Desktop 6-DOF Arm

Firmware and tooling for a ground-up desktop robotic manipulator. Built as a
phase-gated learning project: each phase ends with a measured number, not a
vibe.

**Start here → [docs/bringup-checklist.md](docs/bringup-checklist.md)** when
hardware arrives.
**Start here → [docs/learning-roadmap.md](docs/learning-roadmap.md)** until then.

---

## Layout

```
include/            pins.h, joint_config.h  <- the only files you routinely edit
lib/
  ArmMath/          angle wrapping + multi-turn unwrapping (unit tested)
  MuxTCA9548A/      TCA9548A I2C multiplexer
  EncoderAS5600/    AS5600 absolute encoder, mux-aware
  MotorDriver/      non-blocking STEP/DIR driver w/ trapezoidal profile
  Control/          PID with anti-windup, derivative-on-measurement
  SerialCli/        tiny non-blocking line reader
  JointNode/        per-joint command/state structs + transport-agnostic loop
src/apps/           one firmware app per bring-up step (see below)
test/               unit tests (host or on-target)
scripts/            serial logging + log analysis (Python)
tools/              design analysis: kinematics, torque, workspace (Python)
docs/               checklists, phase plan, troubleshooting, roadmap
hardware/           wiring maps and BOM
```

`scripts/` talks to the board. `tools/` never does - it sizes the arm before
there is anything to talk to.

## The apps

Each is a separate PlatformIO environment. Flash them **in order**; each one
isolates a different failure domain so a problem tells you exactly where to look.

| Env | What it proves | Motor power needed |
| --- | -------------- | ------------------ |
| `i2c_scan` | Mux + encoder are on the bus, magnet is healthy | no |
| `encoder_test` | Angle is smooth and linear over a full turn (turn by hand) | no |
| `motor_test` | Driver, current setting, and motor are good | yes |
| `phase1_bench` | **Phase 1 deliverable:** motor + encoder CSV telemetry | yes |
| `calibration` | Direction sign, true steps/degree, backlash | yes |
| `closed_loop` | PID position control with a live-tuning serial CLI | yes |
| `multi_joint` | Three joints closed-loop in one binary, synchronized | yes |

```powershell
pio run -e i2c_scan -t upload
pio device monitor
```

If `pio` is not on PATH, use the copy that ships with the VS Code extension:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e i2c_scan -t upload
```

## Configuration

Two files, and nothing else, should need editing during bring-up:

- [include/pins.h](include/pins.h) - wiring: pins, I2C addresses, mux channels
- [include/joint_config.h](include/joint_config.h) - mechanics and gains:
  microsteps, gear ratio, limits, speeds, PID

The `calibration` app prints a paste-ready block for `joint_config.h`.

## Host tooling

```powershell
pip install -r scripts/requirements.txt

python scripts/serial_logger.py --list                       # find your port
python scripts/serial_logger.py --port COM3 --name phase1    # capture CSV
python scripts/analyze_log.py docs/test-results/phase1_*.csv # plot + metrics
```

`analyze_log.py` reports tracking error statistics, return-to-home
repeatability, and step-response metrics (rise time, overshoot, settling time,
steady-state error), and saves a PNG next to the CSV.

## Design tooling

`tools/` answers the questions that decide the mechanical design, before any
of it is printed. Each script runs standalone and prints a table; they import
each other by flat name, so **run them from inside `tools/`**.

| Script | Question it answers |
| ------ | ------------------- |
| `arm_model.py` | link/mass/section model, DH table, joint limits - imported by the rest |
| `torque_budget.py` | what reach and payload can a printed 20:1 cycloidal actually hold up? |
| `kinematics.py` | FK, analytic Jacobian, closed-form IK (8 branches), all cross-checked |
| `workspace.py` | where can the tool reach, and where is it near-singular? |

```powershell
cd tools
python kinematics.py     # self-checks: DH vs PoE, analytic vs numeric J, IK round-trip
python workspace.py      # reach, dead zone, singularity census, link split study
```

`kinematics.py` validates itself three ways rather than trusting one
implementation - forward kinematics by DH against product-of-exponentials, the
analytic Jacobian against finite differences, and `FK(IK(pose)) == pose` over
500 random poses. The third check is what caught a sign error in $\theta_2$
caused by the UR convention's negative $a_2$/$a_3$.

Conclusions from these live in
[docs/design-decisions.md](docs/design-decisions.md) (D8, D8a, D8b).

## Tests

```powershell
pio test -e uno_tests        # runs on the board
pio test -e native_tests     # runs on your PC; needs a host g++ (MinGW-w64)
```

Only hardware-independent math lives in `lib/ArmMath`, which is what makes it
testable without a board.

## Current status

Phase 0 (software ready) complete: all seven firmware apps build, unit tests
pass, and the design tooling has fixed the target envelope at 400 mm geometric
reach / 379 mm usable with a 500 g payload. Phase 1 begins when hardware
arrives - see [docs/bringup-checklist.md](docs/bringup-checklist.md).

## Docs

- [Bring-up checklist](docs/bringup-checklist.md) - the arrival-day runbook
- [Design decisions](docs/design-decisions.md) - arm topology, transmission, and why
- [Phase plan](docs/phase-plan.md) - phases 0-6 with exit criteria
- [Learning roadmap](docs/learning-roadmap.md) - what to study and what to build for a robotics career
- [Troubleshooting](docs/troubleshooting.md) - symptom-first debugging
- [TMC2209 setup](docs/tmc2209-setup.md) - microstepping and current setting
- [Wiring](hardware/pinouts/uno-tmc2209-as5600-tca9548a.md)
- [BOM](hardware/bom/bom.md)
