# Phase plan

Each phase has an **exit criterion** that is a measurement, not a feeling. Do
not start the next phase until you can quote the number.

---

## Phase 0 - Software ready (do this now, before hardware)

**Goal:** every line of code that does not need hardware is written and tested.

- Firmware apps 00-05 exist and compile (`pio run -e <env>`)
- Math library unit-tested (`pio test -e uno_tests`)
- Host tooling installed (`pip install -r scripts/requirements.txt`)
- Forward kinematics understood on paper for your intended link geometry

**Exit:** all six environments build; unit tests pass.

---

## Phase 1 - Single joint, characterized

**Goal:** one joint you can trust and describe quantitatively.

Follow [bringup-checklist.md](bringup-checklist.md).

**Exit:**
- Return-to-home repeatability, in degrees, over 20+ cycles
- Backlash, in degrees
- Max reliable speed, in deg/s, without skipped steps
- A saved plot of commanded vs measured angle

---

## Phase 2 - Closed-loop single joint

**Goal:** the joint corrects its own error.

- Tune PID; capture step responses at 3 different step sizes
- Add soft limits and an encoder-loss fault that disables the driver
- Add homing: drive to a hard stop, or use the absolute encoder directly

**Exit:** step-response table (rise time, overshoot %, settling time, steady-
state error) for at least three setpoints, plus a written explanation of which
gain you changed and why.

---

## Phase 3 - Two joints, coordinated

**Goal:** more than one axis moving at once - where real problems start.

- Second encoder on mux channel 1; second driver
- Generalize the firmware from one joint to a joint array
- Synchronized moves: both joints start and finish together (time-scaled, not
  "run each to completion")
- The Uno will run out of pins/RAM/step rate here. Migrating to an ESP32 or
  Teensy is a legitimate engineering decision, and documenting *why* is the
  valuable part.

**Exit:** straight-line-in-joint-space move where both axes arrive within
tolerance simultaneously.

---

## Phase 4 - Kinematics

**Goal:** think in tool-space, not joint-space.

- Forward kinematics: DH parameters for your actual mechanical design
- Inverse kinematics: analytic solution for a spherical-wrist 6-DOF arm, or
  numeric (Jacobian pseudoinverse / damped least squares) if your wrist is not
  spherical
- Workspace analysis: where can the tool actually reach?
- Singularity awareness: what happens near a wrist singularity, and what will
  you do about it?

Prototype all of this in Python first, verify against known poses, then port.

**Exit:** command an (x, y, z, roll, pitch, yaw) pose and have the tool arrive
there measurably.

---

## Phase 5 - Trajectories

**Goal:** motion that is smooth, bounded, and predictable.

- Trapezoidal and S-curve (jerk-limited) profiles
- Joint-space vs Cartesian-space interpolation, and when each is wrong
- Multi-joint synchronization with velocity/accel limits per joint
- Gravity compensation feedforward using measured link masses

**Exit:** the tool traces a straight line in space with bounded deviation you
can plot.

---

## Phase 6 - System integration

**Goal:** something a hiring manager can watch and understand.

- Host-side control node (Python, or ROS 2 if you want the resume line)
- URDF model + RViz/Gazebo visualization matching the real arm
- Pick-and-place demo: move a payload across the desk, repeatably
- A gripper (servo or stepper) and the state machine that runs the task

**Exit:** a 60-second video, a README with real numbers, and a repo someone can
clone.

---

## Hardware architecture migration (parallel track)

The AS5600 + TCA9548A approach is deliberately a Phase 1-3 solution. Its limits:
one encoder read at a time, ~1 ms per read, no daisy-chaining, single-turn only.

When you outgrow it, the options in increasing order of seriousness:
1. **SPI magnetic encoders** (AS5047P, MA732) - fast, chainable, no mux
2. **CAN bus per-joint controller boards** - what industrial arms actually do
3. **Integrated closed-loop stepper/BLDC modules** - buy the problem away

`AS5600Encoder` deliberately hides the transport behind a small interface so the
apps above it barely change when you swap. Keep it that way.
