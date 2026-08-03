# Phase plan

Each phase has an **exit criterion** that is a measurement, not a feeling. Do
not start the next phase until you can quote the number.

---

## Phase 0 - Software ready (do this now, before hardware)

**Goal:** every line of code that does not need hardware is written and tested.

- Firmware apps 00-06 exist and compile (`pio run -e <env>`)
- Math library unit-tested (`pio test -e uno_tests`)
- Host tooling installed (`pip install -r scripts/requirements.txt`)
- Design tooling in `tools/` run and its conclusions recorded: reach, payload,
  gearbox rating, link section, and workspace (D8, D8a, D8b in
  [design-decisions.md](design-decisions.md))
- Forward *and* inverse kinematics working in Python and cross-validated
  (`tools/kinematics.py` self-checks pass)

**Exit:** all seven firmware environments build; unit tests pass;
`tools/kinematics.py` reports 500/500 exact IK round-trips.

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
- Generalize the firmware from one joint to a joint array - `app_06_multi_joint`
  (env `multi_joint`) already does this for three joints via `lib/JointNode`,
  which keeps the control loop transport-agnostic so the same code survives the
  move to CAN (D7)
- Synchronized moves: both joints start and finish together (time-scaled, not
  "run each to completion")
- The Uno will run out of pins/RAM/step rate here. `multi_joint` already sits at
  50 % RAM with three joints, which is the measurement that makes migrating to
  an ESP32 or Teensy a legitimate engineering decision rather than a guess.
  Documenting *why* is the valuable part.

**Exit:** straight-line-in-joint-space move where both axes arrive within
tolerance simultaneously.

---

## Phase 4 - Kinematics

**Goal:** think in tool-space, not joint-space.

Most of this is already done in `tools/` against the *nominal* geometry. Phase 4
is where it meets the arm that actually got built.

- Forward kinematics: DH parameters for your actual mechanical design
- Inverse kinematics: the UR-style layout (J2/J3/J4 parallel) has a **closed-form
  solution via Pieper's parallel-axis branch**, despite the offset wrist - see
  [design-decisions.md](design-decisions.md). `tools/kinematics.py` implements it
  and returns all 8 branches. Add numeric IK (Jacobian pseudoinverse / damped
  least squares) and require the two to agree.
- Measure the real wrist offsets $d_4, d_5, d_6$ on the built arm, update
  `DHParams` in `tools/arm_model.py`, and re-run `workspace.py`. They appear
  directly in the IK equations and CAD nominal will not be accurate enough.
- Workspace analysis: `tools/workspace.py` gives 379 mm usable radial reach and
  a 30 mm dead cylinder for the nominal design (D8b). Confirm against the real
  arm.
- Singularity awareness: shoulder, elbow, and wrist singularities - what happens
  near each, and what will you do about it? The offset wrist puts ~48 % of
  near-singular poses at the **shoulder**, near the base axis, so that is where
  Cartesian speed limiting belongs.

Prototype all of this in Python first, verify against known poses, then port.
The non-negotiable check is `FK(IK(pose)) == pose` over hundreds of random
poses - it is what caught the $\theta_2$ sign error that four hand-checked
poses did not.

**Exit:** command an (x, y, z, roll, pitch, yaw) pose and have the tool arrive
there measurably.

---

## Phase 5 - Trajectories

**Goal:** motion that is smooth, bounded, and predictable.

- Trapezoidal and S-curve (jerk-limited) profiles
- Joint-space vs Cartesian-space interpolation, and when each is wrong
- Multi-joint synchronization with velocity/accel limits per joint
- Gravity compensation feedforward using measured link masses - the model in
  `tools/arm_model.py` already computes gravity torque per joint; replace its
  estimated masses with weighed ones

**Exit:** the tool traces a straight line in space with bounded deviation you
can plot.

---

## Phase 6 - System integration

**Goal:** something a hiring manager can watch and understand.

- Host-side control node (Python, or ROS 2 if you want the resume line)
- URDF model + RViz/Gazebo visualization matching the real arm
- Pick-and-place demo: move a payload across the desk, repeatably
- The gripper (9 g servo on a printed rack and pinion, D6) and the state machine
  that runs the task. It is not a kinematic joint, so it needs no encoder, no
  PID, and no mux channel - but note `Servo` claims Timer1 on the ATmega328P.

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

### Which purchase belongs to which phase

Full tier list, costs, and the measured ceilings behind each trigger are in
[hardware/bom/bom.md](../hardware/bom/bom.md#tiered-upgrade-path). Summary of
when each one becomes blocking:

| Phase | Blocking purchase | Trigger |
| ----- | ----------------- | ------- |
| 1 | none | bench PSU, Uno, and one driver cover it |
| 2 | endstops + e-stop | before the joint can hurt something |
| 3 | **Teensy 4.1** + driver board | Uno tops out ~5 kHz aggregate = 56 deg/s for *one* geared joint |
| 3-4 | **SPI encoders** | the day the first cycloidal reducer goes in; 12-bit becomes the accuracy floor |
| 5 | CAN or integrated actuators | >8 wires crossing a joint, which motors-in-joints reaches fast |
| 6 | host computer | ROS 2, planning, perception |

The cycloidal decision (D2) pulled the Teensy and the SPI encoders **one phase
earlier** than a direct-drive arm would need them, because 20:1 multiplies step
demand by 20 and drops output resolution below what a 12-bit encoder can see.
