# Desktop 6-DOF Arm

Firmware and tooling for a ground-up desktop robotic manipulator. Built as a
phase-gated learning project: each phase ends with a measured number, not a
vibe.

**Start here → [docs/build/README.md](docs/build/README.md)** for what to buy,
what tools you need, and how to assemble each phase.
**Start here → [docs/bringup-checklist.md](docs/bringup-checklist.md)** on the
day hardware arrives.
**Start here → [docs/simulation-plan.md](docs/simulation-plan.md)** for the work
that needs no hardware at all.

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
  Motion/           multi-axis trajectory streaming with velocity feedforward
  Gripper/          open-loop servo gripper, non-blocking slew
src/apps/           one firmware app per bring-up step (see below)
test/               unit tests (host or on-target)
scripts/            serial logging + log analysis (Python)
tools/              design analysis + host control stack (Python)
docs/               checklists, phase plan, troubleshooting, roadmap
hardware/           wiring maps and BOM
```

`scripts/` talks to the board for logging. `tools/` is both the design analysis
that sizes the arm before there is anything to talk to, and the host half of the
control stack that drives it once there is.

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
| `coordinated` | Trajectory-generated synchronized moves + gripper, CLI | yes |
| `host_link` | The arm as a device: binary link, host does IK and planning | yes |

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
| `joint_sim.py` | will this control law hunt, and by how much? (plant model) |

```powershell
cd tools
python kinematics.py     # self-checks: DH vs PoE, analytic vs numeric J, IK round-trip
python workspace.py      # reach, dead zone, singularity census, link split study
python joint_sim.py      # step responses, gain sweep, deadband sweep
```

`kinematics.py` validates itself three ways rather than trusting one
implementation - forward kinematics by DH against product-of-exponentials, the
analytic Jacobian against finite differences, and `FK(IK(pose)) == pose` over
500 random poses. The third check is what caught a sign error in $\theta_2$
caused by the UR convention's negative $a_2$/$a_3$.

`joint_sim.py` models the joint the firmware actually drives - stepper as a rate
source, compliant reducer, backlash, step loss, encoder quantisation and noise -
and runs the real control law against it. It is how the control law was chosen
rather than guessed: a naive PID reverses direction 83 times a second at rest,
and a 2-count deadband takes that to zero (D10).

Conclusions from these live in
[docs/design-decisions.md](docs/design-decisions.md) (D8, D8a, D8b, D10).

## Host control stack

Three layers (D7): the Uno runs the servo loops, the host runs everything above
them. Flash `host_link`, then drive it from Python.

| Script | Layer | Runs without hardware |
| ------ | ----- | --------------------- |
| `trajectory.py` | 1 - trapezoid profiles, sync, jerk limiting | yes, self-checks |
| `joint_link.py` | transport - COBS + CRC-8 framing at 500 kbaud | yes, self-checks |
| `pick_place.py` | 2 - IK, branch selection, task sequencing | yes, `--dry-run` |

```powershell
cd tools
python trajectory.py                  # 13 profile/sync self-checks
python joint_link.py                  # protocol self-check, no port needed
python pick_place.py --verbose        # plan and run a full cycle, no hardware

pio run -e host_link -t upload
python joint_link.py --port COM5      # live state at 1 Hz
python pick_place.py --port COM5      # for real
```

`joint_link.py` is a line-for-line mirror of
[lib/JointNode/PacketFraming.h](lib/JointNode/PacketFraming.h); both sides have
tests asserting the same properties, because two implementations of a wire
protocol drift.

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

**Planning**
- [Phase plan](docs/phase-plan.md) - phases 0-6 with exit criteria
- [Design decisions](docs/design-decisions.md) - arm topology, transmission, and why
- [Learning roadmap](docs/learning-roadmap.md) - what to study and what to build for a robotics career

**Doing**
- [Build guides](docs/build/README.md) - parts, tools, assembly, open decisions, per phase
- [Bring-up checklist](docs/bringup-checklist.md) - the arrival-day runbook
- [Troubleshooting](docs/troubleshooting.md) - symptom-first debugging
- [TMC2209 setup](docs/tmc2209-setup.md) - microstepping and current setting

**Proving**
- [Simulation plan](docs/simulation-plan.md) - fidelity ladder, stack choice, the digital twin
- [Test plan](docs/test-plan.md) - test pyramid, CI, requirements traceability, fault injection
**Reference**
- [Wiring](hardware/pinouts/uno-tmc2209-as5600-tca9548a.md)
- [BOM](hardware/bom/bom.md)
