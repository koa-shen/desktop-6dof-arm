# Phase plan

Each phase has an **exit criterion** that is a measurement, not a feeling. Do
not start the next phase until you can quote the number.

This file says *what* and *how well*. The **how** - parts, tools, assembly
order, open decisions, and phase-specific failure modes - is in
[build/README.md](build/README.md). The **proof** - simulation fidelity and the
test harness - is in [simulation-plan.md](simulation-plan.md) and
[test-plan.md](test-plan.md).

**Precedence rule (D16):** when validation work and control sophistication
compete for the same evening, validation wins. A PID whose behaviour is
predicted by a model to within a stated percentage is a stronger result than an
LQR on a plant nobody has characterised. Control theory beyond PID +
feedforward is explicitly **deferred until the sim-to-real gap is a published
number** (S4 of [simulation-plan.md](simulation-plan.md)).

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

Follow [bringup-checklist.md](bringup-checklist.md) for the electrical gates and
[build/phase-1-first-joint.md](build/phase-1-first-joint.md) for the parts,
tools, mechanical assembly, and the decisions that must close here.

Phase 1 has two stages that are easy to conflate: **1A** is the bench, direct
drive, `GEAR_RATIO = 1.0`, proving electronics; **1B** is the first printed
cycloidal joint with the encoder on the output, proving the mechanism. Do 1A
first - it is the only configuration where an electrical fault and a mechanical
fault cannot be mistaken for each other.

**Exit:**
- Return-to-home repeatability, in degrees, over 20+ cycles
- Backlash, in degrees
- Max reliable speed, in deg/s, without skipped steps
- A saved plot of commanded vs measured angle

---

## Phase 2 - Closed-loop single joint

**Goal:** the joint corrects its own error.

Build guide: [build/phase-2-closed-loop.md](build/phase-2-closed-loop.md).

- Tune PID; capture step responses at 3 different step sizes
- Add soft limits and an encoder-loss fault that disables the driver
- Homing is **absolute from the output encoder** - no hard-stop drive, no limit
  switch (D9). `AS5600Encoder::homeAbsolute()` resolves the first reading into
  [-180, 180) and applies `JOINT_HOME_OFFSET_DEG`. The commissioning step is to
  jog to a mechanical reference, send `z`, and write the offset into
  `include/joint_config.h`.
- Before touching the bench, run `python tools/joint_sim.py`. It models the
  joint - stepper as a rate source, compliant reducer, backlash, step loss,
  encoder quantisation and noise - and reproduces the failure this phase exists
  to avoid: a naive PID reverses direction **83 times a second** at rest. A
  2-count deadband takes that to zero (D10). Finding that in simulation costs
  an afternoon; finding it on the bench costs an evening and a screaming motor.
- Gains are in the **encoder-count domain, units of 1/s** - gear-ratio
  independent. `app_05_closed_loop` converts on the way in.
- **Scope: stop at PID plus velocity and gravity feedforward.** Not because
  anything fancier is beyond reach, but because the interesting result here is
  *the model agreeing with the bench*, and swapping the controller mid-
  validation destroys the comparison. Feedforward is in scope because it is what
  the model directly justifies. Anything past that is Phase 7 material.

**Exit:** step-response table (rise time, overshoot %, settling time, steady-
state error) for at least three setpoints, plus a written explanation of which
gain you changed and why. Compare the measured table against
`tools/joint_sim.py`'s prediction and explain any disagreement - that is how the
model's stiffness and damping estimates get corrected. **The comparison is the
exit criterion, not the tuning.**

---

## Phase 3 - Two joints, coordinated

**Goal:** more than one axis moving at once - where real problems start.

Build guide: [build/phase-3-multi-joint.md](build/phase-3-multi-joint.md).

- Second encoder on mux channel 1; second driver
- Generalize the firmware from one joint to a joint array - `app_06_multi_joint`
  (env `multi_joint`) already does this for three joints via `lib/JointNode`,
  which keeps the control loop transport-agnostic so the same code survives the
  move to CAN (D7)
- Synchronized moves: both joints start and finish together (time-scaled, not
  "run each to completion"). `lib/Motion/MotionController.h` does this by
  stretching every axis's trapezoid to the slowest axis's minimum duration, and
  `app_07_coordinated` (env `coordinated`) is the bring-up app for it.
- The Uno will run out of pins/RAM/step rate here, and now has: `multi_joint`
  sits at 53 % RAM, and `coordinated` - three joints plus trajectory generation
  plus the gripper plus a CLI - sits at **73.9 %** (D13). That is the
  measurement that makes migrating to a Teensy a decision rather than a guess.
  Documenting *why* is the valuable part. Do not spend effort shrinking
  `app_07`; it has already done its job.

**Exit:** straight-line-in-joint-space move where both axes arrive within
tolerance simultaneously. `test/test_motion` asserts the arrival spread is
within the jerk filter's tap count; the bench has to confirm it.

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

The generator exists: `lib/ArmMath/Trajectory.h` (trapezoid + duration
stretching + cross-axis synchronization) and `lib/Motion/MotionController.h`
(setpoint stream, boxcar jerk limiter, velocity feedforward), mirrored on the
host by `tools/trajectory.py`. What remains is meeting the real arm.

- Trapezoidal is done; S-curve is approximated by a boxcar filter over the
  trapezoid, which is cheap and bounds jerk without a third integration. Decide
  whether that is good enough by looking for ringing at the reducer's first
  torsional mode.
- **Trajectory limits are not free parameters.** They must sit under
  `MAX_SPEED_STEPS_PER_SEC / STEPS_PER_OUTPUT_DEG` (18 deg/s at 20:1) and
  `ACCEL_STEPS_PER_SEC2 / STEPS_PER_OUTPUT_DEG` (45 deg/s^2). Planning above
  the ceiling saturates the loop and produces following error that reads
  exactly like bad tuning (D10). Re-derive both when `GEAR_RATIO` changes.
- Joint-space vs Cartesian-space interpolation, and when each is wrong. Today
  everything is joint-space, so the tool traces an arc between waypoints.
- Gravity compensation feedforward using measured link masses - the model in
  `tools/arm_model.py` already computes gravity torque per joint; replace its
  estimated masses with weighed ones

**Exit:** the tool traces a straight line in space with bounded deviation you
can plot. Cartesian interpolation is the missing piece for this.

---

## Phase 6 - System integration

**Goal:** something a hiring manager can watch and understand.

The host stack exists and runs without hardware:

| Piece | File | State |
| ----- | ---- | ----- |
| Layer 0 firmware, no CLI | `src/apps/app_08_host_link.cpp` | builds, 57 % RAM |
| Wire protocol | `lib/JointNode/PacketFraming.h` + `tools/joint_link.py` | self-checks pass both sides |
| Layer 1 profiles | `tools/trajectory.py` | 13 self-checks pass |
| Layer 2 task | `tools/pick_place.py` | full cycle plans and runs in `--dry-run` |

- Host-side control node: `pick_place.py --port COM5` streams `MODE_TRACK`
  setpoints at 200 Hz over COBS/CRC-8 framing at 500 kbaud (D12).
- **Branch selection is a whole-path problem.** The planner searches every
  feasible IK branch of the first waypoint and chains from each, because a plan
  that greedily picks the nearest branch commits to a configuration family that
  cannot reach the place point without wrapping J1 past its limit. Sweeping
  candidate pick points showed z = 0.05 m puts J2 on its -135 deg limit and
  fails outright - reachable is not the same as plannable.
- URDF model + RViz/Gazebo visualization matching the real arm - **not started**;
  planned as stage S0/S6 of [simulation-plan.md](simulation-plan.md), where the
  URDF is *generated* from `tools/arm_model.py` rather than hand-written (D14)
- Pick-and-place demo: move a payload across the desk, repeatably. The dry run
  currently plans an eight-waypoint cycle in ~21 s of motion.
- The gripper (9 g servo on a printed rack and pinion, D6/D11) is open loop with
  compliant TPU fingers. `ServoGripper` slews without blocking and detaches when
  idle so its Timer1 ISR stops jittering step generation. **Grasp success is not
  observable** - the task waits long enough that the servo must have finished,
  which is not the same as knowing an object is held.

**Exit:** a 60-second video, a README with real numbers, and a repo someone can
clone. Plus a repeatability figure: run the same cycle 20 times and measure the
spread at the place point.

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
| 3-4 | **SPI encoders** | read latency and the mux single point of failure - **not** accuracy; see D17 |
| 5 | CAN or integrated actuators | >8 wires crossing a joint, which motors-in-joints reaches fast |
| 6 | host computer | ROS 2, planning, perception |

The cycloidal decision (D2) pulled the Teensy and the SPI encoders **one phase
earlier** than a direct-drive arm would need them, because 20:1 multiplies step
demand by 20 and drops output resolution below what a 12-bit encoder can see.

---

## Simulation and validation (priority track)

Runs alongside every phase above, is mostly **not gated on hardware**, and
**outranks the optional parts of Phases 4-6** when time is short. Per D16, this
is where effort goes before any control law more sophisticated than PID plus
feedforward.

| Stage | What | Needs hardware? |
| ----- | ---- | --------------- |
| S0 | URDF generated from `tools/arm_model.py`, cross-checked in Drake and MuJoCo | no |
| S1 | Kinematic twin - animate `pick_place.py`'s plan, detect self-collisions | no |
| S2 | Dynamic twin - reproduce D10's control-law results in a multibody model | no |
| S3 | `SimTransport` - one script drives sim and hardware interchangeably | no |
| S4 | **Validation - measure the sim-to-real gap and report it as a number** | Phases 1-2 |
| S5 | Firmware-in-the-loop - the shipped control law against the simulated plant | host compiler |
| S6 | ROS 2 / `ros2_control` / MoveIt, if it earns its place | Phase 6 |

**S4 is the gate.** Nothing in Phase 7 starts until S4 produces its table. The
system-ID measurements S4 depends on - link masses, reducer stiffness, damping,
backlash, friction, efficiency, loop jitter - total under a day of bench time
and are the cheapest credibility in the project.

Detail and reasoning in [simulation-plan.md](simulation-plan.md) (D14, D15).
The harness that runs all of it - test pyramid, CI, requirements traceability,
fault injection - is in [test-plan.md](test-plan.md).

---

## Phase 7 - Advanced control (deferred, deliberately)

Not a to-do list. A holding pen, so that ideas that arrive mid-Phase-2 get
written down instead of acted on.

| Idea | What it would need first |
| ---- | ------------------------ |
| State-space / LQR on a joint | a validated 2nd-order plant model - i.e. S4 |
| Computed-torque / inverse-dynamics control | measured link inertias, not CAD estimates |
| Backlash compensation | backlash measured under load, not just statically |
| Friction feedforward (Stribeck) | the constant-velocity sweeps in the S4 system-ID set |
| Disturbance observer | a trusted model of what "no disturbance" looks like |
| Impedance / admittance control | torque sensing or current sensing, which does not exist yet |

The pattern in every row is the same: **each one requires a model that S4
produces.** That is not a coincidence, and it is the actual argument for doing
validation first rather than a matter of taste. Attempting any of these on
estimated parameters produces a controller that works on one build of one joint
and cannot be explained - the opposite of the goal.

If one of these is attempted anyway, it goes on a branch, and the acceptance
test is beating the Phase 2 PID on the *same* step-response table. "It feels
smoother" is not a result.
