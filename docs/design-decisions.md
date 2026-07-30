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

## D5 - Motor sizing by joint tier - **OPEN**

**Question.** Should the wrist joints use a smaller motor than NEMA 17, rather
than only a smaller reduction ratio?

**Where this came from.** UR tiers their joints by **physical module size**, not
by ratio - three module sizes reused across the product line with overlap
(UR3 = 3x size-1 + 3x size-0; UR5 = 3x size-2 + 3x size-1). Their big-to-small
joint torque ratio is roughly 4.7:1 (~56 N.m vs ~12 N.m on a UR3, from service
documentation - unverified). Ratio tiering alone, at 40:1 and 20:1 with one
motor, only buys 2:1.

**Placeholder numbers** - these use assumed geometry (~400 mm reach, ~2 kg arm,
0.3 kg payload) and **must be redone once real link masses exist**:

| Joint | Static demand | x2 safety | Available | Margin |
| ----- | ------------- | --------- | --------- | ------ |
| Shoulder (J2) @ 40:1 | 5.1 N.m | 10.2 N.m | 13.4 N.m | 1.3x |
| Wrist (J5) @ 20:1 | 0.44 N.m | 0.9 N.m | 6.7 N.m | **7.4x** |

Available torque assumes 0.42 N.m motor and 80 % gearbox efficiency.

**The argument for a smaller wrist motor.** Three NEMA 17s is ~0.84 kg sitting
at maximum moment arm on a ~2 kg arm. Dropping to NEMA 14 or 11 saves ~0.5 kg
exactly where it hurts most, and it **compounds**: lighter wrist -> lower
shoulder torque -> lower shoulder ratio -> higher shoulder speed. The wrist is
7x over-torqued anyway, so the torque is not being used.

**The argument against.** A second motor size means a second gearbox size, a
second bearing stack, and a second set of print iterations - directly against
the "design one joint module" principle in D1 that is supposed to keep this
project finishable. Two mechanical designs instead of one is real cost.

**Resolve this by:** completing the torque budget with measured/CAD link masses,
not the placeholders above. If the wrist margin is still > 3x, the smaller motor
is probably worth the second design. Note that UR pays this cost three times
over, which suggests it is worth it at production scale - but this is not
production scale.

**Related:** at 20:1 the wrist could also run a *lower* ratio for more speed,
since torque is not the constraint there. That is a cheaper way to capture part
of the same benefit without a second motor size.
