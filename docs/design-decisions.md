# Design decisions

Running log of architectural choices and the reasoning behind them. "Why did you
choose that?" is most of a design interview, and reconstructing the reasoning
six months later is miserable. Append, do not rewrite.

---

## D1 - Arm topology: UR-style 6R, offset wrist

**Decision.** Target a Universal Robots-style layout: six revolute joints, all
motors housed in the joints, with a non-spherical (offset) wrist.

Axis arrangement:

| Joint | Axis | Note |
| ----- | ---- | ---- |
| J1 | vertical (base) | |
| J2 | horizontal (shoulder) | |
| J3 | horizontal (elbow) | **parallel to J2** |
| J4 | horizontal (wrist 1) | **parallel to J2, J3** |
| J5 | perpendicular (wrist 2) | |
| J6 | along tool axis (wrist 3) | |

In DH terms this means $\alpha_2 = \alpha_3 = 0$, with nonzero link offsets
$d_4, d_5, d_6$ at the wrist.

**Why.**
- Every joint is the same kind of thing: a motor, a cycloidal reducer, and an
  output-mounted encoder in a pancake housing. Design **one** joint module,
  iterate on it until it is good, then scale it to three sizes. This is exactly
  how UR builds theirs, and it collapses six mechanical design problems into one.
- An offset wrist is dramatically easier to build than a spherical wrist. A true
  spherical wrist needs three axes intersecting at a point, which forces bevel
  gear or belt differentials in a tiny package - the hardest and most
  backlash-prone part of any printed arm.
- The form factor is legible. "It looks like a UR5" communicates instantly.

**The tradeoff we thought we were making, and why it turned out not to apply.**

The usual rule is: no spherical wrist means no closed-form inverse kinematics,
so you are stuck with numeric IK. That rule comes from **Pieper's criterion**,
which guarantees a closed-form solution if *either*:

1. three consecutive joint axes **intersect** at a point (spherical wrist), or
2. three consecutive joint axes are **parallel**.

The UR layout fails condition 1 but satisfies **condition 2** - J2, J3, and J4
are parallel. So this geometry keeps an analytic IK solution with all 8
configuration branches (shoulder left/right, elbow up/down, wrist flip), despite
the offset wrist. Hawkins (2013), *Analytic Inverse Kinematics for the Universal
Robots UR-5/UR-10 Arms*, is the standard reference derivation.

This is the best available outcome: easy-to-print wrist **and** closed-form IK.
Implementing numeric IK (damped least squares) anyway remains worthwhile as a
cross-check and as a more transferable skill - the two must agree.

**Consequences.**
- The wrist offsets $d_4, d_5, d_6$ appear directly in the IK equations, so they
  must be **measured on the built arm**, not taken from CAD nominal.
- Keeping J2/J3/J4 genuinely parallel is now a manufacturing requirement, not a
  nicety. Non-parallelism silently invalidates the closed-form solution.
- Motors inside the joints means cables route through the arm, which pushes
  toward a CAN/distributed architecture earlier than a
  motors-at-the-base design would.

**Singularities to expect:** shoulder (wrist center over the J1 axis), elbow
(arm fully extended), and wrist (J5 near 0, J4 and J6 aligned).

---

## D2 - Transmission: NEMA 17 + printed cycloidal reducer at every joint

**Decision.** Every joint is a NEMA 17 stepper driving a printed cycloidal
gearbox, target ratio in the 20:1 to 40:1 range.

**Why.**
- Torque. NEMA 17 gives 0.42 N·m holding. At 20:1 with an assumed 80 % gearbox
  efficiency that is ~6.7 N·m at the output - comfortably enough for a desktop
  arm with a few hundred grams of payload, at every joint. One motor size and
  one gearbox design covers the whole arm.
- Cycloidal reaches high single-stage ratios in a compact pancake, which is the
  shape a joint module wants to be.
- Many teeth in contact means good shock tolerance, and low backlash *if* the
  tolerances come out right.
- It is printable, which is the constraint that actually decides this.

**Known risks, in rough order of how much they will hurt.**

1. **Backlash from print tolerance.** The cycloidal profile is sensitive; FDM
   tolerance of 0.1-0.2 mm shows up directly at the output. Expect ~0.5-2 deg on
   a first attempt versus ~0.1-0.3 deg for a well-tensioned belt reduction.
   `app_04_calibration` measures this - it is the first number to get.
2. **Imbalance.** A single cycloidal disc rides on an eccentric and is
   inherently unbalanced. Use two discs 180 deg apart, or a counterweight.
3. **Pin wear.** Printed ring pins wear quickly. Use steel dowel pins, ideally
   with needle roller sleeves.
4. **Efficiency is a guess until measured.** Cycloidal is typically 70-90 %;
   printed will be at the low end. Derate the torque budget accordingly.
5. **Bearing count.** A cycloidal joint needs a lot of bearings. Budget for it.

**Backdrivability.** Using $\eta_{rev} \approx 2 - 1/\eta_{fwd}$, an 80 %
efficient stage is ~75 % backdrivable, so this should *not* be self-locking.
That is good for safety and for future force sensing, but it means the arm will
sag when unpowered - keep the drivers enabled to hold position, or design in a
brake for J2/J3.

---

## D3 - Consequences of D2 for electronics

The reduction ratio is not just a mechanical choice; it moves two electrical
constraints hard.

**Step resolution vs encoder resolution.** At 20:1 with 8x microstepping:

$$200 \times 8 \times 20 = 32000 \text{ steps per output rev} = 88.9 \text{ steps/deg}$$

so one step is **0.011 deg** at the output. Compare against encoders mounted on
the output shaft:

| Encoder | Bits | Output resolution | vs one step |
| ------- | ---- | ----------------- | ----------- |
| AS5600 (current) | 12 | 0.088 deg | 8x coarser |
| AS5047P (SPI) | 14 | 0.022 deg | 2x coarser |

**The encoder, not the motor, is now the accuracy floor.** Two implications:

- Do **not** raise microstepping for resolution. You already have far more
  mechanical resolution than you can measure. Stay at 8x and spend the step-rate
  budget on speed instead.
- 14-bit SPI encoders move up the priority list relative to the estimate in the
  BOM, because they roughly quadruple the measurable resolution.
- The proper long-term answer is the dual-encoder arrangement real robot joints
  use: a motor-side encoder for the velocity loop (cheap resolution, since it
  multiplies by the ratio) plus an output-side absolute encoder for position
  accuracy and backlash immunity.

**Step rate.** 88.9 steps/deg means moving one joint at 60 deg/s needs
**5300 steps/s for that joint alone**. The Uno's measured aggregate ceiling is
around 5 kHz across all joints. Six joints at that speed needs ~32 kHz.

This is a hard, quantified argument that the Uno cannot run this arm past joint
one, and that the microcontroller upgrade (Teensy 4.1, hardware-timer step
generation) is required earlier than a direct-drive design would have needed it.

---

## D4 - Encoder placement

Magnet on the **output** side of the cycloidal reducer, on the axis of rotation,
0.5-3 mm gap, no ferrous material nearby.

Output-side placement means the encoder reads true joint angle including all
gearbox backlash and compliance - which is the point. A motor-side encoder would
report a beautiful number that the tool does not agree with.

---

## D5 - Joint torque tiering: same face, different stack length

**Decision.** Every joint uses a **42 mm NEMA 17 face and one common joint
envelope**. Torque is tiered two independent ways, neither of which requires a
second mechanical design:

1. **Motor body length** - 60 mm at the shoulder, ~40 mm mid-arm, ~23 mm at the
   wrist. Same bolt circle, same shaft, same mount.
2. **Reduction ratio** - high at the shoulder, low at the wrist.

**Why.** The original framing of this decision was wrong. I had it as a binary:
one NEMA 17 everywhere (cheap to design, badly over-torqued at the wrist) versus
a second motor size such as NEMA 11 (right-sized, but a second gearbox, a second
bearing stack, and a second round of print iterations).

Stack-length tiering is a third option that dominates both. NEMA 17 is a
**face** specification, not a torque specification - the 42 x 42 mm flange and
bolt pattern are fixed while the rotor stack varies:

| Body | Holding torque | Mass | Output @ 20:1 | Output @ 26:1 |
| ---- | -------------- | ---- | ------------- | ------------- |
| 23 mm | ~0.13 N.m | ~0.15 kg | 2.1 N.m | 2.7 N.m |
| 34 mm | ~0.28 N.m | ~0.24 kg | 4.5 N.m | 5.8 N.m |
| 40 mm | ~0.42 N.m | ~0.28 kg | 6.7 N.m | 8.7 N.m |
| 48 mm | ~0.59 N.m | ~0.39 kg | 9.4 N.m | 12.3 N.m |
| 60 mm | ~0.68 N.m | ~0.60 kg | 10.9 N.m | 14.1 N.m |

Catalog values at 1.5-2.0 A, vendor-dependent, +/-25 %. Output assumes 80 %
gearbox efficiency.

That is a **5.2x** torque spread from motor length alone, before any ratio
tiering. UR's big-to-small joint spread on a UR3 is about 4.7x. The same
mechanical envelope reaches UR-class tiering for the cost of ordering different
part numbers.

**Consequences.**

- The gearbox, bearing pocket, output flange, encoder mount, and link interface
  are designed **once**. Ratio and motor length become per-joint configuration,
  not per-joint engineering. This preserves the "design one joint module"
  principle from D1 that keeps the project finishable.
- Mass ends up where it does no harm. The 60 mm motor is the heaviest part of
  the arm and sits at the shoulder, where its moment arm is near zero. The 23 mm
  motors sit at the wrist, where moment arm is maximal. Choosing 23 mm over
  40 mm for two wrist motors saves ~0.26 kg and removes ~0.77 N.m of shoulder
  demand - which then permits a lower shoulder ratio, hence more shoulder speed.
  The benefit compounds inboard.
- Rotor inertia scales roughly with stack length, so a 60 mm motor has ~2.6x the
  inertia of a 23 mm one. Long motors at the wrist would hurt acceleration even
  where holding torque was adequate. Another reason not to use one motor
  everywhere.
- **Ratio selection is deferred, not decided.** Nothing about the envelope
  depends on it. Settle ratios from `tools/torque_budget.py` once real link
  masses exist.

**Reference point.** Sweep Dynamics' Armold (475 mm reach, 750 g payload, 1.8 kg,
+/-1 mm repeatability, 24 V) uses exactly this scheme: 1x 60 mm + 3x 37.5 mm +
2x 23 mm NEMA 17, with a family of drives that all share a 42 x 42 x 26 mm
envelope in 4:1, 8:1, 20:1 and 26:1. Their combined motor-plus-ratio spread is
roughly 17x. Verified from the vendor product pages; joint-by-joint assignment
is inferred, not published.

**Revision note.** The earlier version of this entry sized against a 0.3 kg
payload on a ~2 kg arm. A shipping peer-class arm does 750 g on 1.8 kg, so that
payload assumption was ~2.5x low. Back-solving Armold's shoulder at full
extension gives ~5.7 N.m, with the payload contributing more than the arm's own
mass, and that sits at the rated limit of their 20:1 drive. **The shoulder is
the binding constraint and there is no comfortable margin in this class of
machine.** Size it first; everything else has slack.

---

## D6 - End effector is a hobby servo, off the kinematic chain

**Decision.** Gripper actuation is a 9 g metal-gear digital servo driving a
printed rack and pinion, not a seventh stepper joint.

**Why.** The gripper is not part of the 6-DOF pose solution. It has one degree
of freedom, needs no absolute accuracy, and never participates in FK or IK. A
servo is a closed-loop position system in a package: one PWM line, no driver, no
encoder, no mux channel, no calibration app, no soft-limit fault path. Making it
a stepper joint would add all of that to solve a problem that does not exist.

A rack and pinion converts the servo's rotation to parallel jaw travel, and the
pinion radius sets the force/travel tradeoff. At a typical 9 g servo's
0.15-0.25 N.m stall torque:

| Pinion radius | Jaw force @ 0.15 N.m | @ 0.25 N.m |
| ------------- | -------------------- | ---------- |
| 4 mm | 38 N | 62 N |
| 6 mm | 25 N | 42 N |
| 8 mm | 19 N | 31 N |

Any of these vastly exceeds what is needed to hold a 750 g payload. Pick the
pinion for **jaw travel**, not force, and expect the printed rack teeth to be
the weak link before the servo is.

**Firmware consequence - read before wiring.** The Arduino `Servo` library uses
**Timer1** on the ATmega328P. `StepperDriver` currently generates steps by
polling `micros()`, so there is no conflict today. If step generation ever moves
to a hardware timer - which multi-axis coordination will want - Timer1 is taken.
Plan to put step generation on Timer2, or move the gripper to a separate PWM
source, or accept that this is one more reason the Uno is a bring-up platform
rather than the final controller.

**Consequence.** Gripper commands stay outside the joint controller entirely:
an open-loop `g <percent>` command, not a PID axis.

---

## D7 - Control topology: three layers, and where "the brain" lives

**Decision.** Three layers with hard boundaries, each running at its own rate.
The boundaries are fixed now; which silicon runs each layer is allowed to change.

| Layer | Rate | Owns | Runs on (now -> later) |
| ----- | ---- | ---- | ---------------------- |
| 2 Application | 1-50 Hz | goals, planning, vision, teach | PC -> PC or Pi w/ ROS 2 |
| 1 Motion controller | 100-500 Hz | IK, trajectory interpolation, sync, whole-arm safety | Uno `app_06` -> STM32/ESP32 |
| 0 Joint node | 1-10 kHz | one joint's servo loop, its own limits and faults | `JointController` in one binary -> one MCU per joint |

**Why a separate layer 1 at all.** The defining job of the middle layer is not
computation, it is **time synchronization**. Six joints must reach their
waypoints simultaneously or the tool leaves the commanded path. No joint can do
this alone, and layer 2 is too jittery to do it. That is the whole reason the
layer exists.

**The command idiom.** Layer 1 does **not** send "go to 45 degrees" and wait. It
sends a *stream* of interpolated setpoints at a fixed rate - "be at 12.3 deg
now" every few milliseconds. Joint nodes stay dumb; the host stays smart. Two
alternatives, with the tradeoff named:

- **Streaming setpoints** (UR and most industrial arms): host interpolates and
  transmits every cycle. Maximum flexibility, but a dropped or late frame is
  immediately visible in the motion. Needs low-jitter comms.
- **Segment handoff**: host sends "reach X in T ms with this profile" and the
  node interpolates locally. Tolerant of comms jitter, but the host can no
  longer change its mind mid-segment.

Start with streaming. Move to segments only if comms jitter proves to be the
limiting factor.

**Bus bandwidth sets the control rate.** A classic CAN 2.0A frame with 8 data
bytes is ~111 bits worst case with stuffing:

| Bitrate | Per frame | 6 cmd + 6 state | Ceiling | Joint-to-joint skew |
| ------- | --------- | --------------- | ------- | ------------------- |
| 250 kbps | 444 us | 5.33 ms | 188 Hz | 2.7 ms |
| 500 kbps | 222 us | 2.66 ms | 375 Hz | 1.3 ms |
| 1 Mbps | 111 us | 1.33 ms | 751 Hz | 0.7 ms |

Skew is what it costs to send six command frames back to back. At 0.3 m/s tool
speed, 1.3 ms of skew is ~0.4 mm of path error - the same order as the +/-1 mm
repeatability this class of arm achieves anyway. **Run 1 Mbps** and keep the
bus short; there is no reason to leave the margin on the table.

**Non-negotiable: layer 0 must be safe alone.** A joint node holds position or
stops on its own if the host stops talking. It never depends on the host for
safety. Already implemented as `FAULT_COMMS` with a 500 ms timeout in
`JointController`. The physical estop is a **hardware** line that pulls every
driver ENABLE directly - it does not route through any MCU, because the failure
mode it exists for is "the brain is wrong or hung".

**Consequence for IK.** IK runs at layer 1, never at a joint - it needs every
joint angle and the full kinematic model. Retaining closed-form IK (D1) is what
makes this affordable on a microcontroller: a UR-style analytic solution is
microseconds, where a numeric solver would force layer 1 onto Linux and drag
its jitter into the motion.

**Consequence for the model.** The kinematic model is a single source of truth
in `tools/arm_model.py`, exported to a firmware header and to URDF. A joint node
knows only its own ratio, direction, and limits - never the arm's geometry.

**Consequence for today.** `app_06_multi_joint` already *is* layer 1 plus three
instances of layer 0 in one binary. Splitting it later moves code across a
boundary that already exists rather than inventing one.
