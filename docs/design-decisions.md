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

---

## D8 - Target envelope: 400 mm reach, 500 g payload

**Decision.** Design to **400 mm reach** (base axis to flange, arm horizontal
and fully extended) and **500 g payload at full extension**, with a printed
gripper counted separately at ~120 g. *Superseded in part by D8b: 400 mm is the
geometric sum; measured usable radial reach is 379 mm.*

**Why not larger.** From `tools/torque_budget.py`, requiring a 1.5x design
factor on the shoulder gearbox rating:

| Reach | Payload | Shoulder | Gearbox needed | Longest link | Link deflection |
| ----- | ------- | -------- | -------------- | ------------ | --------------- |
| 350 mm | 500 g | 3.69 N.m | 5.5 N.m | 147 mm | 0.09 mm |
| **400 mm** | **500 g** | **4.26 N.m** | **6.4 N.m** | **168 mm** | **0.14 mm** |
| 450 mm | 500 g | 4.84 N.m | 7.3 N.m | 189 mm | 0.20 mm |
| 475 mm | 750 g | 6.29 N.m | 9.4 N.m | 199 mm | 0.28 mm |
| 550 mm | 750 g | 7.37 N.m | 11.1 N.m | 231 mm | 0.44 mm |

400 mm / 500 g needs a 6.4 N.m shoulder box - inside a high-torque cycloidal in
the NEMA 17 envelope, which is a demonstrated product. 475 mm / 750 g needs
9.4 N.m, which no printed drive in that envelope delivers with margin.

**Note on the reference arm.** Armold publishes 475 mm and 750 g against a drive
family rated 4-5 N.m. Our model puts that configuration at 6.29 N.m static, so
their headline payload is running at roughly **1.0x or less** against the
gearbox rating - almost certainly not simultaneously available at full
extension. That is a normal way to spec a hobby arm, but it is not a design
target to copy. Building to 1.5x is the difference between a demo and a machine
that survives being used.

**The print bed is not the binding constraint - correct that assumption.**
Usable Bambu P1S footprint is ~240 mm flat, ~339 mm on the diagonal. The longest
link at 400 mm reach is **168 mm**, and even at 550 mm reach it is only 231 mm.
The bed does not bind until roughly 570 mm of reach. **The printed gearbox binds
first, by a wide margin.** So do not split links to fit the bed: a bolted
mid-link splice adds compliance exactly where the bending moment is highest,
which costs repeatability to solve a problem that does not exist.

The bed constraint that *does* bite is orientation, not size. A link printed
lying flat has its layer planes parallel to the bending load; printed standing
up they are perpendicular, at 40-70 % of the strength. Keep every structural
link flat on the bed. 168 mm leaves 70 mm of margin to iterate the section
without re-planning the print.

**Links are not the accuracy problem - stop optimising them.** Upper-arm tip
deflection at the chosen spec is **0.14 mm**. One AS5600 count at 400 mm reach
is **0.61 mm**, over 4x larger. The structure is already four times stiffer than
the sensor can observe. Two consequences:

- Further stiffening the links is wasted mass that the shoulder pays for. The
  section was already cut from 50x60x3.5 mm to 38x45x2.2 mm on this basis.
- The remaining compliance that matters is in the **joints** - gearbox windup,
  bearing play, backlash - which this model does not capture and which
  `app_04_calibration`'s backlash test is the only way to measure.

**Expected repeatability: +/-1 mm**, matching the reference arm - but **only
under unidirectional approach**, and set by backlash rather than by the encoder
whenever approach direction varies. The original version of this paragraph said
the 12-bit encoder set the floor; `tools/error_budget.py` shows quantization
contributes 0.37 mm against backlash's 4.25 mm. See **D17**, which supersedes
the claim. Better repeatability requires a **better gearbox or a one-way
approach**, not a different encoder.

**Provisional joint assignment at this spec:**

| Joint | Demand | Motor | Ratio | Drive |
| ----- | ------ | ----- | ----- | ----- |
| J2 shoulder | 4.26 N.m | 60 mm | 26:1 | high-torque cycloidal |
| J3 elbow | 1.96 N.m | 40 mm | 20:1 | standard cycloidal |
| J1, J4 | low | 40 mm | 20:1 | standard cycloidal |
| J5, J6 | < 0.5 N.m | 23 mm | 20:1 | standard cycloidal |

J1 carries no gravity torque at all - it rotates about a vertical axis. J4 and
J6 are roll axes along the arm, so gravity produces no moment about them either.
Only J2, J3 and J5 are sized by statics.

**Revisit when** link masses are measured rather than estimated. Update
`tools/arm_model.py` and re-run; every number above follows from it.

### D8a - This spec is a phase, not a ceiling

Machined gearboxes are a planned upgrade. The question that matters now is which
parts of the design have to change when they arrive, and the answer is **only
the joints**.

| Payload | Shoulder | Link deflection | vs one encoder count |
| ------- | -------- | --------------- | -------------------- |
| 500 g | 4.26 N.m | 0.14 mm | 23 % |
| 1000 g | 6.22 N.m | 0.19 mm | 31 % |
| 2000 g | 10.14 N.m | 0.29 mm | 48 % |

The 38x45x2.2 mm link section stays inside half an encoder count at **4x** the
design payload. Structure, kinematics, DH table, and link lengths therefore need
no revision when the drives improve - the arm scales by replacing gearboxes and
motors inside an unchanged envelope. This is the payoff of the D5 fixed-envelope
rule, and it is worth protecting: **keep the machined units on the same 42 mm
face and the same output flange** so they are drop-in.

**But machined drives alone buy nothing measurable.** Order-of-magnitude
backlash, at the joint output:

| Source | Backlash |
| ------ | -------- |
| Printed cycloidal, stainless rollers in bushings | ~0.3-1.0 deg |
| Machined cycloidal | ~0.05-0.1 deg |
| AS5600 quantisation | 0.088 deg |

With printed drives the gearbox dominates the encoder by roughly 10x, so the
AS5600 is the correct, cheap choice. With machined drives the two become
comparable - and the encoder, which the controller cannot see past, becomes the
limit. Spending funding on machined gearboxes while keeping a 12-bit encoder
buys precision that nothing in the system can observe or act on.

**So they are one purchase, not two.** Machined drives must be accompanied by a
higher-resolution absolute encoder - 14-bit or better, and preferably SPI, which
also retires the TCA9548A mux and its per-read latency. Reaching 0.1 mm at
400 mm reach needs ~15 bits (see D3).

Until then, `app_04_calibration`'s backlash measurement is the number that
decides whether the upgrade is worth funding. Measure the printed drive first.

### D8b - Reach is 379 mm usable, and the link split barely matters

`tools/workspace.py` samples 20 000 configurations inside `JOINT_LIMITS_DEG`,
runs FK on each, and takes the smallest singular value of the linear block of
the Jacobian. Three results correct or qualify D8.

**Reach.** D8 quoted 400 mm, which is the arithmetic sum
$|a_2| + |a_3| + d_5 + d_6$. The measured maximum radial reach is **379 mm**.
The gap is not joint limits - it is that the wrist offset $d_4 = 48$ mm and the
final two links do not all project into the horizontal plane when the tool is
pointed usefully. **Restate the spec as 400 mm geometric, 379 mm usable.** Do
not lengthen the links to recover it: the missing 21 mm costs shoulder torque
proportionally and buys reach only in poses where the tool points away from the
work.

Also measured: a **30 mm dead cylinder** about the base axis, smaller than $d_4$
because J1 yaw plus wrist articulation lets the tool reach back inside; and a
vertical span of **-264 to +484 mm**, the negative half of which is only usable
if the arm is mounted at a table edge rather than on the surface.

**Singularities.** 3.4 % of sampled poses fall below $\sigma_{min} = 0.02$,
split shoulder 48 %, elbow 33 %, wrist 21 %. Shoulder dominance is the expected
signature of the D1 offset wrist: the wrist-centre singularity that a spherical
wrist concentrates at one point is spread into a region near the base axis
instead. The practical consequence is that **the near-base region needs
Cartesian-speed limiting, not the fully-extended region**, which is the opposite
of the usual intuition.

**Link split.** Holding $a_2 + a_3 = 316$ mm and sweeping the ratio from 0.67 to
1.50 moves reachable volume by 3 % and shoulder torque by 5 %. Max reach does
not move at all, because it depends only on the sum. **The split is a weak
lever.** The model mildly prefers a shorter upper arm - it pulls the elbow motor
inboard - but that is inside model error, and it is the reverse of UR5's 1.08.
The disagreement is a modelling artefact: the elbow motor is treated as a point
mass at $r = a_2$, while a real forearm carries more distributed mass than the
model gives it. **Keep $a_2 = 168$, $a_3 = 148$ mm.** Revisit only after the
first links are printed and weighed.

**Method note.** The voxel-occupancy volume estimate is *not converged* at
20 000 samples - it is still rising roughly linearly. 98 L is a lower bound. It
remains valid for comparing designs at equal sample count, which is all the
split study needs, but it must not be quoted as the arm's workspace volume. The
tool now prints a convergence table and warns when the estimate has not settled.

---

## D9 - Homing: absolute, from the output encoder, with no homing move

**Decision.** The arm does **not** home by driving to a limit switch. On power-up
each joint reads its AS5600 once, resolves that reading into the range
[-180, +180) degrees, applies a per-joint commissioning offset
(`JOINT_HOME_OFFSET_DEG`), and declares itself homed. No motion, no switches, no
sequence.

**Why this is available at all.** The AS5600 is an *absolute* encoder mounted on
the **output** side of the reducer (D4). It knows where the joint is before the
motor has ever moved. A homing move exists to convert an incremental sensor into
an absolute one; there is no incremental sensor here, so there is nothing to
convert.

**What it buys.**
- A crash or a power cut is recoverable by cycling power, not by re-homing a
  possibly-collided arm through an unknown path.
- No limit switches: six switches, six wires, six pins, and six things to
  debounce that the Uno does not have to spare.
- Startup is instant, which matters more than it sounds when bring-up means
  power-cycling forty times an evening.

**The subtlety that bit us.** A raw AS5600 read is 0-4095, i.e. 0 to 360 deg. A
joint parked at -30 deg reads 330. Fed straight into a soft-limit check against
`JOINT_MIN_DEG = -135`, that is an instant `FAULT_SOFT_LIMIT` on a joint that is
nowhere near its limit. `AS5600Encoder::homeAbsolute()` therefore resolves the
first sample by choosing turn number -1 when the wrapped angle exceeds 180. Every
subsequent read is unwrapped relative to that, so multi-turn tracking still
works.

**The cost, and it is real.** Absolute homing is only as good as the offset
table. `JOINT_HOME_OFFSET_DEG` has to be measured once per assembly, per joint,
and re-measured whenever a magnet, a hub, or an encoder board is disturbed.
Commissioning procedure: jog the joint to a known mechanical reference, send
`z` (`CMD_ZERO_HERE`), read back the offset, write it into
`include/joint_config.h`. That is a manual step a limit switch would have
automated. It is worth it, but it is a step that must be documented rather than
remembered.

**Revisit when** a joint can turn more than one output revolution. Absolute
homing then genuinely cannot recover the turn count and something else -
a multi-turn encoder, a switch, or a mechanical hard stop - has to supply it.
No joint on this arm exceeds 300 degrees of travel, so it does not apply yet.

---

## D10 - Control law: feedforward first, PID as a trim, deadband to stop the buzz

**Decision.** The joint servo loop is

```
step_rate = vel_ff * steps_per_count * vel_ff_scale     (MODE_TRACK only)
          + PID(setpoint - measured) * steps_per_count
```

with a **2-count deadband** around the setpoint and **Ki = 0**. Gains are
defined in the **encoder-count domain** with units of 1/s.

**Why feedforward carries the move and PID only trims it.** A stepper is a rate
source. Layer 1 already knows the velocity its trajectory implies at every
instant, so handing that velocity straight to the driver means the feedback term
only has to correct what the model got wrong - friction, gravity sag, gearbox
windup - rather than generate the whole motion. `tools/joint_sim.py` measures
the difference on a modelled J3 (0.041 kg.m^2, 1.96 N.m gravity, 300 N.m/rad
reducer, 0.5 deg backlash, 0.5-count encoder noise):

| Case | Overshoot | Settle | Reversals/s at rest |
| ---- | --------- | ------ | ------------------- |
| A step setpoint, PID only, no deadband | 16.0 % | 1.37 s | **83** |
| B step setpoint, PID + 2-count deadband | 16.0 % | 1.37 s | 0 |
| C profiled setpoint + feedforward + deadband | **1.0 %** | **0.93 s** | 0 |
| D as C, plus Ki = 4 | 1.1 % | 0.93 s | 0 |

Feedforward turns a 16 % overshoot into 1 %, and settles a third faster. Nothing
about the gains changed.

**Why Ki stays 0.** Case D is the experiment. Integral action buys nothing
measurable here because the plant has no steady-state error to remove: a stepper
holds position with detent and holding torque, so once the setpoint is reached
the error is already inside the encoder's resolution. What integral action *does*
have is a state that keeps growing while the joint is stuck in backlash or
against a soft limit, and then discharges as a lurch. `PID_INTEGRAL_LIMIT`
exists to bound that if it is ever enabled, and `JointController` now actually
calls `setIntegralLimit()` - it did not before, which made the limit decorative.

**Why the deadband, and why 2 counts.** One AS5600 count is 0.088 deg. At 20:1
and 8 microsteps that is **7.81 microsteps**. Without a deadband the controller
chases a target it can only resolve to within eight steps, and the encoder's own
noise floor is enough to keep it commanding a reversal every 12 ms:

| Deadband | Degrees | Reversals/s at rest | Steady-state error |
| -------- | ------- | ------------------- | ------------------ |
| 0 counts | 0.000 | **83** | 0.00 deg |
| 1 count | 0.088 | 0 | -0.02 deg |
| 2 counts | 0.176 | 0 | 0.06 deg |
| 3 counts | 0.264 | 0 | 0.01 deg |
| 5 counts | 0.439 | 0 | 0.00 deg |

That is the whole tradeoff, and it is not a subtle one: the deadband is free.
One count already silences it; 2 is chosen for margin against a noisier magnet
than the model assumes. Above ~3 counts the band exceeds the encoder resolution
by enough that it becomes the dominant accuracy term, so it does not go higher.
Inside the band the controller calls `PidController::trackMeasurement()` rather
than simply skipping the update, so the derivative term does not see a
discontinuity when the joint leaves the band again.

**Why the gains live in the count domain.** `PID_KP` was previously "steps/s per
degree" in `app_05_closed_loop` and "1/s in the count domain" inside
`JointController` - the same constant meaning two different things, differing by
a factor of `STEPS_PER_OUTPUT_DEG`. A 1-count error commanded 3.5 deg/s in one
and 0.025 deg/s in the other. The count domain is now canonical because it is
**gear-ratio independent**: change `GEAR_RATIO` and `steps_per_count` absorbs it,
leaving the gains valid. `app_05` multiplies by `STEPS_PER_OUTPUT_DEG` on the way
in and prints the same units it accepts.

**The gain that does not matter, which is the surprising result.** With
feedforward doing the work, sweeping Kp from 2 to 64 changes peak following
error by less than 0.03 deg:

| Kp | Peak following error | Hunt (pk-pk at rest) |
| -- | -------------------- | -------------------- |
| 2 | 0.088 deg | 0.008 deg |
| 8 | 0.105 deg | 0.008 deg |
| 64 | 0.112 deg | 0.008 deg |

So do not tune Kp for tracking. `PID_KP = 8` is chosen low deliberately: the
modelled first torsional mode of the reducer is ~14 Hz, and a high-gain loop on
a compliant drive is how a joint learns to sing. Tune Kp only if disturbance
rejection is measurably poor, and expect to find that the answer is a stiffer
gearbox rather than a bigger number.

**Trajectory limits are not free parameters.** The planner's limits must sit
below what the driver can deliver:
`MAX_SPEED_STEPS_PER_SEC / STEPS_PER_OUTPUT_DEG` = 1600 / 88.9 = **18 deg/s**,
and 4000 / 88.9 = **45 deg/s^2**. `TRAJ_MAX_VEL_DEG_S` was originally 30, above
that ceiling; the loop saturated and produced 1.6-3.4 deg of following error that
read exactly like bad tuning. It is now 15 and 40. **Re-derive both whenever
`GEAR_RATIO` or `MICROSTEPS` changes** - they do not scale on their own.

**Caveat on the model.** Reducer stiffness (300 N.m/rad) and damping (15 % of
critical) are estimates, not measurements. Measure them before treating the
absolute numbers as more than a ranking: hang a known mass and read the encoder
deflection for stiffness; tap the link and count the ring-down for damping. The
*ordering* of the four cases is robust to those numbers; the settling times are
not.

---

## D11 - Gripper: open loop, compliant, and honest about it

**Decision.** A 9 g hobby servo on a printed rack and pinion, with **compliant
TPU fingers**, driven open loop. No encoder, no current sensing, no grasp
detection. Grip force is set by finger stiffness multiplied by commanded
overtravel: `GRIPPER_GRIP_DEG` (100) closes past `GRIPPER_CLOSED_DEG` (85), and
those 15 degrees of interference are what squeezes the object.

**Why compliance instead of feedback.** A rigid gripper on a position-controlled
servo has exactly one correct closure angle per object, and being 1 mm wrong
either drops the object or stalls the servo. A compliant finger converts
position error into force error along a gentle slope, so a single "grip"
command works across a range of object sizes. That is a mechanism solving a
controls problem, which is the cheaper trade every time it is available.

**What this explicitly cannot do.** There is no way to know whether the grasp
succeeded. `pick_place.py` commands the gripper and waits
`travel/slew + settle_ms`; "gripped" means "long enough has passed that the
servo must have finished", not "an object is held". Detecting a failed grasp
needs current sensing on the servo or a camera, and neither exists. This is
written down rather than discovered later, because the failure mode is an arm
that confidently places nothing.

**Timer1 and the step generator.** The Arduino `Servo` library claims Timer1 on
the ATmega328P and its ISR fires every 20 ms. That ISR runs while
`StepperDriver::run()` is trying to meet step deadlines, and the resulting jitter
is visible as roughness at high step rates. `ServoGripper` therefore **detaches
the servo when idle** (`GRIPPER_DETACH_IDLE`), which stops the ISR entirely
between commands. The cost is that a detached servo does not resist an external
push, so an object can be worked loose by inertia during a fast move. If that
shows up on the bench, the fix is to keep it attached while carrying and accept
the jitter - not to remove the detach entirely.

`ServoGripper::update()` slews the commanded angle at `GRIPPER_SLEW_DEG_S`
rather than jumping, and contains no `delay()`. A blocking gripper move would
stall three step generators for a quarter of a second.

**Revisit when** there is a reason to know whether the grasp worked. The cheapest
honest upgrade is a current-sense resistor on the servo supply; the useful one is
a camera, which is a different project.

---

## D12 - Host link: COBS + CRC-8 framing, and the bandwidth that points at CAN

**Decision.** `app_08_host_link` speaks a framed binary protocol at
**500 000 baud**, not CSV at 115200. Wire format:

```
COBS( type | node | payload | crc8 ) 0x00
```

**Why not CSV.** CSV is for a human reading a scroll, and the bench apps keep
using it. It is unusable for driving a control loop: there is no way to
resynchronize after a dropped byte, no way to detect a corrupted field, and an
8-byte command becomes ~30 bytes of ASCII plus a float parse on a 16 MHz AVR.
All three become the bottleneck the moment a host streams setpoints at 200 Hz.

**Why COBS.** Consistent overhead byte stuffing removes every zero byte from the
frame, which frees `0x00` to mean exactly one thing: end of frame. A receiver
that gets lost scans forward to the next zero and is synchronized. Overhead is
one byte per frame plus one per 254 - constant and known, unlike escape-based
schemes whose worst case doubles the frame length.

**Why CRC-8 as well as the UART's own framing.** A UART framing error catches a
lost bit boundary; it does not catch a bit flipped by a stepper's commutation
transient coupling into an unshielded USB cable a hand's width away. That is the
corruption that actually happens on this bench. The CRC is computed table-free -
eight shifts per byte is nothing next to the I2C read it shares a loop with, and
a 256-byte table would be 12 % of the ATmega328P's RAM.

**Why 500 000 baud.** One framed 8-byte payload is 13 bytes on the wire. Three
joints in and three out at 200 Hz is 7.8 kB/s each way. A 115200 link carries
11.5 kB/s *total*, so that traffic is 68 % of the link before any logging - and
UART bandwidth is shared, not duplex-budgeted, in practice on a USB-serial
bridge. At 500 kbaud the same traffic is 16 %.

**And that is the argument for CAN, stated as a number.** Six joints at 200 Hz
is 15.6 kB/s each way. That still fits 500 kbaud, but it is a single point of
failure on a shared bus with no arbitration, no per-node addressing, and no
priority. Classic CAN 2.0 at 1 Mbps carries the same traffic with hardware
arbitration and an 8-byte frame that `JointCommand` and `JointState` were
deliberately sized to fit (D7). **The protocol does not change when the
transport does** - only the framing layer is discarded.

**Status bits inside the fault byte.** `STATUS_IN_POSITION` lives in bit 6 of
`JointState::fault` rather than in a new field, because there is no spare byte
in an 8-byte CAN frame. Consequence: **test faults with
`state.fault & kFaultMask`, never `state.fault != 0`**. Both the firmware and
`tools/joint_link.py` have a unit test asserting exactly that, because it is the
kind of mistake that produces a phantom fault only when the arm is working
correctly.

**Mirrored, not reimplemented.** `tools/joint_link.py` is a line-for-line mirror
of `lib/JointNode/PacketFraming.h`, with a self-check that round-trips frames,
verifies COBS overhead, and confirms that a single flipped bit is rejected. Two
implementations of a wire protocol drift; a mirror plus a shared test suite is
the cheapest defence available without a shared language.

---

## D13 - The Uno is out of RAM, and that is now a measurement

`app_07_coordinated` - three joints, trajectory generation, gripper, and a CLI
in one binary - uses **73.9 % of the ATmega328P's 2 KB** (1513 bytes) and 66.5 %
of flash. `app_08_host_link`, which moves layers 1 and 2 to the host, uses
57.1 % and 47.5 %.

That is the D3/D7 argument stopping being a prediction. Three joints fit; six do
not, and the headroom that remains is not enough for the fault handling, the
logging, and the CAN driver that six joints would need. **The Uno is a bring-up
platform, not the target.** The next controller is a Teensy 4.1 (or one MCU per
joint node, per D7), and the split that makes that a port rather than a rewrite
already exists in the layer boundary.

Do not spend effort shrinking `app_07`. It has already done its job by producing
this number.

---

## D14 - Simulation: pick a model format, not a simulator

**Decision.** URDF generated from `tools/arm_model.py` is the interchange
format. **Drake** is the primary engine, **MuJoCo** is a secondary engine kept
alive purely as an independent cross-check, and the simulator attaches to the
system as a **transport** behind `tools/joint_link.py` rather than as a separate
application. Full plan in [simulation-plan.md](simulation-plan.md).

**Why not just pick one simulator.** Every engine in this space is replaceable
and several will be replaced within the project's lifetime. What is not
replaceable is the kinematic and inertial model, which already has a single
source of truth in `arm_model.py`. Committing to a format and generating the
model means the same arm loads in Drake, MuJoCo, PyBullet, RViz and MoveIt, and
the engine choice stops being architectural. This is the same move already made
for the encoder transport (`AS5600Encoder`) and the joint protocol
(`JointCommand`/`JointState`) - hide the thing that will change behind the thing
that will not.

**The URDF must be generated, never hand-edited.** A hand-written URDF that
drifts from `arm_model.py` produces a simulation that is confidently wrong,
which is worse than having none. CI diffs the generated file against the
committed one.

**Why Drake as primary.** Of the candidates it is the only one whose systems
framework natively expresses *sampled-data* structure: systems with declared
discrete update periods and explicit ports. That maps 1:1 onto the D7 layering -
layer 0 at 1 kHz, layer 1 at 200 Hz, layer 2 at 10 Hz - in one diagram, with
correct semantics at each boundary. A physics engine alone makes that the user's
problem. Drake is also the tool where *verification* rather than *visualisation*
is the organising idea, which is the point of the exercise, and it is an
existing skill from the LCLS/XCS digital-twin work, so the marginal cost is low.

**The cost, and it is real.** Drake officially supports Ubuntu and macOS only.
On this Windows machine that means WSL2 or Docker for every Drake session. That
is a genuine daily friction, and it is why MuJoCo - `pip install mujoco`,
natively - is kept as the second engine rather than dropped. The second engine
also earns its keep: two independent implementations agreeing on forward
kinematics and gravity torque is the same validation pattern `kinematics.py`
already uses internally when it checks DH against product-of-exponentials.

**Why the simulator is a transport, not an application.** D12 put a framed
binary protocol at the layer-0/layer-1 boundary, and `tools/joint_link.py`
already hides the serial port behind an interface. A `SimTransport` that speaks
the same 8-byte structs makes the simulator a **drop-in replacement for the
hardware**: `pick_place.py --sim` and `pick_place.py --port COM5` run the
identical planner, trajectory generator, and task FSM. This is what
`ros2_control`'s `hardware_interface` is for, and what every industrial HIL rig
does. Building it here costs almost nothing because the seam already exists.

**Explicitly deferred.** ROS 2, Gazebo and MoveIt are a Phase 6 decision. The
hiring signal is real and the URDF makes them cheap later, but a ROS stack
standing on an unvalidated model is impressive-looking and proves nothing.

**Revisit when** contact-rich manipulation matters (MuJoCo's contact model
becomes the better primary), or when a second person needs to run the stack
(the ROS interfaces stop being overhead and start being the point).

---

## D15 - A model is not trusted until its disagreement is a number

**Decision.** Every simulation level must reproduce a measurement from an
adjacent level before its results are quoted. The sim-to-real gap is reported as
a table of six canonical manoeuvres with a percentage error each, produced by
`sim/validate.py`. Detail in [test-plan.md](test-plan.md).

**Why this is a decision and not a platitude.** The repo already quotes
simulation results as if they were facts - D10's settling times, D8b's workspace
volume, the 83 reversals per second. Two of those three are load-bearing in
other decisions. But `joint_sim.py`'s reducer stiffness (300 N·m/rad) and
damping (15 % of critical) are **estimates**, D10 says so in a caveat, and the
caveat is easy to lose. Making validation a gate rather than a footnote is the
difference between a model that informs decisions and one that launders guesses
into them.

The parameters that have to be measured rather than assumed:

| Parameter | Currently | Measurement | Time |
| --------- | --------- | ----------- | ---- |
| Link masses and CoM | estimated | scale + knife-edge balance | 20 min |
| Reducer torsional stiffness | estimated 300 N.m/rad | hang a known mass, read encoder deflection | 20 min |
| Damping ratio | estimated 15 % critical | tap test, log-decrement on the ring-down | 20 min |
| Backlash | estimated 0.5 deg | dial indicator on a lever arm | 30 min |
| Coulomb + viscous friction | not modelled | constant-velocity sweeps | 1 h |
| Gearbox efficiency | assumed 80 % | stall torque vs motor torque x ratio | 30 min |
| Loop rate and jitter | assumed | GPIO toggle + logic analyser | 30 min |

Under a day of bench time. It converts every simulation number in the repo from
plausible to defensible, and it is the cheapest credibility available.

**Predicted worst disagreements, recorded before measuring** - because
predicting first is the only version of this exercise that is honest:
Stribeck friction near zero velocity (the sim will be optimistic on small
steps), backlash under load exceeding the static measurement, thermal drift,
and print-to-print variation between nominally identical joints.

**Consequence for the artifact.** The deliverable is one figure: commanded,
simulated and measured on one time axis with the residual underneath. It is
worth more than any rendering, because it is the only one that states how wrong
the model is.

**Consequence for testing.** Assertions go on *relationships*, not absolute
numbers - "a deadband of at least 1 count gives zero reversals", "feedforward
reduces overshoot by more than 5x". Those survive a model refinement; a
hard-coded 1.37 s settling time does not.

---

## D16 - Validation depth over control sophistication

**Decision.** Control law scope is frozen at **PID plus velocity and gravity
feedforward** until the sim-to-real gap is published as a number (S4 of
[simulation-plan.md](simulation-plan.md)). State-space, LQR, computed-torque,
backlash compensation, friction feedforward, observers and impedance control are
collected in Phase 7 of [phase-plan.md](phase-plan.md) and not started before
then. When validation work and controller work compete for the same evening,
validation wins.

**Why, in one line.** Every deferred technique needs a plant model that does not
exist yet, so doing them first means tuning them against estimates.

That is not a discipline argument, it is a dependency:

| Technique | Model parameter it depends on | Currently |
| --------- | ---------------------------- | --------- |
| LQR / pole placement | 2nd-order plant: inertia, stiffness, damping | estimated |
| Computed torque | link inertia tensors | CAD estimates |
| Backlash compensation | backlash under load | not measured |
| Friction feedforward | Coulomb + viscous + Stribeck coefficients | not modelled |
| Disturbance observer | a trusted nominal model | the thing S4 produces |

Fit one of these to wrong parameters and it works on one build of one joint,
cannot be explained, and silently stops working when the next joint is printed
0.1 mm tighter. That failure is hard to even detect, which is what makes it
worth a decision rather than a preference.

**The second reason is what the result is worth.** "I implemented LQR" invites
"on what model?" and there is no good answer without the validation. "My model
predicted 180 ms settling, the hardware did 205 ms, the 14 % gap is Stribeck
friction near zero velocity which the model does not capture" is a stronger
statement, is harder to fake, and is the thing that separates a robotics project
from a robotics *engineering* project. The scarce skill at this level is not
knowing more controllers; it is being able to say how wrong you are.

**Consequences.**
- Phase 2 exits on the **model-vs-bench comparison table**, not on the tuning.
  Swapping controllers mid-validation destroys the comparison, which is the
  concrete reason to freeze scope rather than a stylistic one.
- The simulation track (S0-S4) outranks the optional parts of Phases 4-6.
  S0-S3 need no hardware, so this is also the answer to "the bench is blocked,
  what do I work on."
- The learning roadmap inserts **system identification** and **model
  validation** between PID and LQR, rather than jumping from one to the other.
- Ideas that arrive mid-Phase-2 get written into Phase 7 instead of acted on.

**The tradeoff, named.** This trades breadth of demonstrated technique for depth
on one. A reviewer scanning for buzzwords sees fewer of them. That is a real
cost and it is accepted deliberately: the depth signal survives follow-up
questions and the breadth signal does not.

**Revisit when** S4 publishes its agreement table. At that point Phase 7 opens,
and the acceptance test for anything in it is beating the Phase 2 PID on the
*same* step-response table - not "it feels smoother."

---

## D17 - The encoder is not the accuracy floor. Backlash and windup are.

**Decision.** The headline spec changes from a bare "+/-1 mm" to two separate
numbers with stated conditions, and the justification for the SPI encoder
upgrade is withdrawn. Produced by `tools/error_budget.py`, which propagates
per-joint angular error through the linear part of the geometric Jacobian:
$\delta p = J_v(q)\,\delta q$.

**What the budget says** at full extension, 400 mm nominal reach:

| Source | Tip error | Kind |
| ------ | --------- | ---- |
| Reducer windup under gravity | 6.25 mm | systematic, load-dependent |
| Encoder non-linearity (INL) | 4.64 mm | systematic, calibratable |
| Backlash | 4.25 mm | random if approach direction varies |
| Axis misalignment (0.2 deg) | 3.09 mm | systematic, calibratable |
| Link length tolerance | 1.50 mm | systematic, calibratable |
| Magnet eccentricity | 1.55 mm | systematic, calibratable |
| **Encoder quantization** | **0.37 mm** | random |
| Link bending deflection | 0.21 mm | systematic |
| Encoder noise | 0.13 mm | random |

**Encoder quantization is the third-smallest term in the budget.** D8 and the
BOM both assert the 12-bit AS5600 "is the accuracy floor." That is wrong as
stated. One count at full reach is ~0.5 mm, which is true and is where the
claim came from - but backlash at an assumed 0.5 deg is 4.25 mm, roughly 8x
larger, and reducer windup is larger still.

**D8's claim survives, but only under an unstated condition.** "Repeatability
set by the encoder" is correct *if every point is approached from the same
direction*, because unidirectional approach removes backlash from the random
budget:

| Approach | Repeatability |
| -------- | ------------- |
| Bidirectional | 4.27 mm |
| Unidirectional | 0.43 mm |

That is a **10x improvement from a firmware change, not a hardware purchase** -
overshoot every target and come back to it the same way, which is what a CMM
and most machine tools do. It is the cheapest performance in the entire
project and it was not previously written down anywhere.

**Consequence 1 - the SPI encoder trigger is wrong.** `phase-plan.md` listed
"12-bit becomes the accuracy floor" as the reason to buy AS5047P/MA732 at Phase
3-4. Going 12-bit to 14-bit shrinks a 0.37 mm term to 0.09 mm while 4-6 mm
terms sit untouched. The upgrade is still justified - by **read latency and the
mux being a single point of failure** (D3, D7) - but not by accuracy. Buying
encoders to fix a backlash problem is the expensive version of this mistake and
is exactly what the budget exists to prevent.

**Consequence 2 - the spec is restated.** Accuracy and repeatability are
different numbers and the project was quoting one figure for both:

| Metric | Target | Condition |
| ------ | ------ | --------- |
| Repeatability | **< 1.0 mm** | unidirectional approach, fixed payload, after warm-up |
| Accuracy, uncalibrated | ~20 mm | do not quote this without the qualifier |
| Accuracy, after kinematic calibration | goal < 5 mm | requires Phase 4 DH identification |

Sub-millimetre *accuracy* was never achievable on printed reducers and claiming
it would not have survived one question from anyone who has built an arm.
Sub-millimetre *repeatability* is achievable and is the honest headline.

**Consequence 3 - calibration is worth a phase, not an afternoon.** Six of the
nine terms are systematic, which means repeatable, which means removable by a
lookup table. The budget says calibration is worth ~22 % on its own and much
more once windup is compensated from the pose-dependent gravity torque the
model already computes.

**The tradeoff, named.** This makes the README's headline number worse and its
qualifiers longer. A spec with conditions attached looks less impressive than a
bare number and is worth considerably more, because the bare number is the one
that gets challenged.

**Caveats.** Backlash (0.5 deg) and stiffness (300 N.m/rad) are the two largest
inputs and both are currently *estimates* - the exact situation D15 exists to
flag. Phase 1B measures both, and the budget is re-run that day. Encoder INL is
a datasheet typical, not a measured value for this part.

**Revisit when** Phase 1B produces measured backlash and stiffness. If measured
backlash comes in under 0.2 deg the ranking changes and windup dominates alone.
