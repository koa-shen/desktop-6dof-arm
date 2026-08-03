# Learning roadmap

The arm is the vehicle. The goal is being able to hold a technical conversation
with a robotics R&D team and have evidence behind every claim.

---

## The next two days (hardware still in transit)

Everything here is doable with zero hardware and pays off the moment the box
arrives.

### 1. Set up the host toolchain (30 min)

```powershell
pip install -r scripts/requirements.txt
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e phase1_bench
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" test -e uno_tests --without-uploading --without-testing
```

If you already have the Uno from the Elegoo kit, you can flash and run
`i2c_scan`, `motor_test`, and `uno_tests` **today** - `i2c_scan` will correctly
report "no mux," which proves your build/upload/monitor loop works. Debugging
the toolchain now means you are debugging *hardware only* on arrival day.

### 2. Read the code you are about to depend on (1-2 h)

In this order:
- [lib/ArmMath/ArmMath.h](../lib/ArmMath/ArmMath.h) - why does `AngleUnwrapper`
  exist? What breaks without it?
- [lib/MotorDriver/StepperDriver.h](../lib/MotorDriver/StepperDriver.h) - find
  the line that computes `vDecel`. Derive it yourself from
  $v^2 = 2 a d$. That single line is the whole trapezoidal profile.
- [lib/Control/PidController.h](../lib/Control/PidController.h) - why is the
  derivative taken on the measurement instead of the error?

Then change something and re-run `pio test -e uno_tests`. Break a test on
purpose so you know the harness actually works.

### 3. Nail down the joint module (2-3 h)

Topology and transmission are **decided** - UR-style 6R with an offset wrist,
NEMA 17 + printed cycloidal at every joint. See
[design-decisions.md](design-decisions.md) for the reasoning. What is still open:

- **Cycloidal ratio per joint**, driven by the torque budget below. Pick from a
  small set (e.g. 20:1 and 40:1) so you print two variants, not six.
- **Joint module envelope** - the same design scaled to ~3 sizes. Get the
  bearing stack, eccentric, and output flange right *once*.
- **Where the AS5600 magnet mounts** - output side of the reducer, on the axis
  of rotation, 0.5-3 mm gap, no steel nearby.
- **Cable routing through the joint.** Motors in the joints means wires cross
  every axis. Decide now whether that is a bundle with a service loop or a slip
  ring, because it changes the housing.

Keep appending decisions and reasoning to `docs/design-decisions.md`. "Why did
you choose that?" is most of a design interview.

### 4. Torque budget (2 h) - do not skip this

Your NEMA 17 is 42 N·cm = 0.42 N·m. For a shoulder joint holding a link of mass
$m$ with its center of mass at distance $r$:

$$\tau = m g r$$

A 0.3 kg forearm with CoM at 0.15 m needs $0.3 \times 9.81 \times 0.15 = 0.44$
N·m **just to hold still, unloaded**. That already exceeds a direct-drive NEMA
17. So:
- Compute the static holding torque for each joint at worst-case extension
- Add payload, add a 2x safety factor
- Divide by 0.42 N·m to get the reduction ratio each joint needs
- Derate for gearbox efficiency - assume 70-80 % for a printed cycloidal until
  you measure it, so a 20:1 stage delivers more like 16:1 of useful torque

This calculation determines your entire mechanical design. Doing it in a
spreadsheet now saves you from printing parts that cannot lift themselves. It is
also what tells you how many distinct cycloidal ratios you actually need to
design - hopefully two.

### 5. Forward kinematics on paper, then in Python (3-4 h)

Assign Denavit-Hartenberg parameters to your intended geometry and write a
Python script that computes the tool pose from six joint angles. Verify: at all
zeros, does the tool land where you expect? Rotate joint 1 by 90 deg - does the
tool swing the way your intuition says?

You do not need the arm to do this, and it is the single highest-value skill on
this list.

### 6. Print the mounts (overnight)

PETG for the encoder mount and motor bracket. Print the encoder mount with the
magnet pocket as a separate test coupon first so you can check the gap with
feeler gauges before committing to a full bracket.

---

## Core competencies, in dependency order

These map to what robotics R&D roles actually screen for.

### Controls
1. **PID** - you will tune it in Phase 2. Understand *why* each term does what it
   does, not just the recipe.
2. **System modeling** - first/second-order response, time constants, damping
   ratio. Your step-response plots are the data; learn to read them.
3. **System identification** - going the other way: from the step response and
   the ring-down back to the plant parameters. This is the skill that turns
   `joint_sim.py`'s estimated stiffness into a measured one, and it is
   underrepresented in hobby projects and expected in industry.
4. **Model validation** - stating the disagreement between model and hardware as
   a number, and knowing which discrepancies matter. See
   [simulation-plan.md](simulation-plan.md) S4.
5. **Feedforward** - PID reacts to error; feedforward prevents it. Gravity
   compensation is the intuitive first example, and the model you validated in
   step 4 is what supplies the feedforward term.
6. **State-space and LQR** - the vocabulary of modern control. **Read about it;
   do not implement it here yet.** It needs a validated plant model, which is
   what 3 and 4 produce. Deferred to Phase 7 by D16.

Items 3 and 4 are the deliberate insertion. The conventional list jumps from PID
straight to LQR, which is how people end up with a controller they cannot
explain running on a plant they never measured. Being able to say "my model
predicted 180 ms settling and the hardware did 205 ms, and here is why" is a
better interview answer than naming a controller.

Resource: Brian Douglas's control lectures, then Åström & Murray's
*Feedback Systems* (free PDF). For system ID specifically, search
"log decrement damping ratio" and "least squares parameter estimation robot
dynamics".

### Kinematics and dynamics
1. Rotation representations: matrices, Euler angles, quaternions. Know why
   quaternions exist (gimbal lock) and when Euler angles are fine.
2. Homogeneous transforms and DH parameters
3. Forward and inverse kinematics
4. Jacobians: velocity mapping, singularities, and the pseudoinverse
5. Lagrangian dynamics and the manipulator equation

Resource: Lynch & Park, *Modern Robotics* (free PDF + free Coursera series).
This is the standard reference and it is genuinely worth working through.

### Embedded and real-time
1. Non-blocking loop design - already demonstrated in this repo; `delay()` in a
   control loop is a red flag
2. Timing and jitter: measure your actual loop rate, do not assume it
3. Interrupts and timer-driven step generation (the natural upgrade from
   `micros()` polling)
4. Fixed-point vs floating-point tradeoffs on small MCUs
5. CAN bus and distributed motor control

### Software
1. Python for analysis, prototyping, and host control
2. C++ for firmware; get comfortable with classes, templates, and RAII
3. Git with meaningful commit messages and a readable history
4. ROS 2 basics: nodes, topics, services, URDF, tf2, RViz. Frequently listed as
   a requirement; a working URDF of *your own* arm is a strong artifact.

---

## Turning this into job evidence

What actually moves a hiring conversation, roughly in order of impact:

1. **Numbers.** "Repeatability of 0.4 deg over 50 cycles, 0.9 deg backlash,
   step response settling in 180 ms with 8 % overshoot" beats any adjective.
   Every phase in [phase-plan.md](phase-plan.md) exits with numbers - collect
   them.
2. **A validated model.** One plot with commanded, simulated and measured on the
   same time axis and the residual underneath, plus a table of agreement across
   several manoeuvres. This is the rarest item on the list at this level and the
   one that most directly signals "can be trusted with a real system" - because
   it states how wrong the model is instead of hoping nobody asks. See
   [simulation-plan.md](simulation-plan.md).
3. **Plots.** Commanded vs measured, error over time, step response. Generated
   automatically by `scripts/analyze_log.py`.
4. **A failure you diagnosed.** Pick the hardest bug you hit, write it up:
   symptom, hypotheses, how you discriminated between them, root cause, fix.
   This is the single most convincing artifact an early-career engineer can
   produce, because it demonstrates method rather than luck.
5. **Design tradeoffs, documented.** Why AS5600 + mux and not SPI encoders? Why
   belt reduction and not cycloidal? Why did you migrate off the Uno?
6. **A 60-second video** of the arm doing the task, in the README.

Notably absent: a list of control techniques implemented. "I implemented LQR"
invites "on what model?", and there is no good answer to that without item 2.

Keep `docs/test-results/` and a running `docs/design-decisions.md` from day one.
Reconstructing them later is miserable and the details that make you sound
credible are exactly the ones you forget.

---

## Honest scope warning

A 6-DOF arm is a large project and the most common failure mode is trying to
build all six joints before one joint works well. One fully characterized,
closed-loop, well-documented joint is worth more - both as learning and as a
portfolio piece - than six joints that jitter. The phase structure exists to
enforce that.
