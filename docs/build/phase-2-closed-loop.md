# Phase 2 - build guide: closed loop and the safety layer

Phase 1 produced a joint you can describe. Phase 2 produces a joint that
**corrects itself**, and - equally important - one that fails safe. This is the
first phase where the arm can damage itself, so the safety hardware is a
prerequisite, not an afterthought.

Prerequisite: Phase 1 exit table complete, including the two-way backlash
measurement and the torsional stiffness number.

---

## Objectives

1. A step-response table - rise time, overshoot %, settling time, steady-state
   error - at **three** step sizes, with plots.
2. A written explanation of which gain you changed and why, and what it did.
3. **A comparison of measured against `tools/joint_sim.py`'s prediction**, with
   the disagreement explained rather than excused.
4. Soft limits and an encoder-loss fault that provably disable the driver -
   demonstrated by deliberately triggering each.
5. A commissioned `JOINT_HOME_OFFSET_DEG` and a documented procedure for
   re-measuring it.

The exit criterion is the sim-vs-bench comparison. Anyone can tune a PID; the
thing worth showing is a model whose error you have quantified.

**Scope freeze (D16).** PID plus velocity and gravity feedforward, and nothing
else, until the sim-to-real gap is a published number. If an idea for a better
controller arrives while you are tuning - and it will, around the third time the
joint overshoots - write it into Phase 7 of
[phase-plan.md](../phase-plan.md) and keep tuning. Changing the controller
halfway through invalidates every row of the comparison table you have collected
so far, which is the actual cost and it is not a small one.

---

## Parts

| Qty | Part | Notes |
| --- | ---- | ----- |
| 1 | **Latching mushroom e-stop, NC contacts**, 22 mm | must break **VMOT**, not logic - see below |
| 1 | Automotive relay or contactor, 30 V / 15 A+ | if the e-stop contacts are not rated for the motor current |
| 2 | Mechanical hard stops (printed or aluminium) | physical travel limits so a runaway ends against material, not against the wiring |
| 1 | Inline fuse holder + 5 A blade fuse | on the 24 V rail |
| 2-6 | Endstop switches, mechanical or optical | **optional** - D9 does not use them for homing, but they are a firmware-independent safety layer |
| 1 | Counterweight or gas spring *(nice to have)* | balances a shoulder joint so a fault does not slam it |
| - | Known masses, 100 g-1 kg | disturbance rejection tests |
| - | Foam/soft padding | put it under the joint's travel path before the first closed-loop run |

### The e-stop, specifically

It must cut **motor power**, with the logic still alive. Two reasons:

- The failure mode an e-stop exists for is "the firmware is wrong or hung". A
  button that a hung MCU has to read is not an e-stop.
- Logic staying up means the encoder still reads, so you can see where the arm
  ended up and recover cleanly instead of power-cycling blind.

Wire the NC contact in series with the coil of a relay that carries VMOT, or in
series with VMOT directly if the switch is rated for it. Also pull every driver
`EN` line to the disabled state from the same contact, so the drivers are
commanded off *and* unpowered.

Test it before you need it: with the joint holding position, hit the button. The
joint should go limp immediately, the Uno should stay connected, and the
telemetry should keep streaming.

---

## Tools

Everything from Phase 1, plus:

| Tool | Used for | Note |
| ---- | -------- | ---- |
| **The plotting pipeline** | `scripts/serial_logger.py` → `analyze_log.py` | this *is* the instrument for this phase |
| Stopwatch / phone slow-motion video | sanity-checking settling time against the plot | a plot that disagrees with the video means a timestamping bug |
| Known masses + a lever arm | disturbance-rejection step tests | |
| Thermal probe or IR thermometer | motor and driver temperature under holding load | closed-loop holding runs hotter than open-loop jogging |
| Oscilloscope *(nice to have)* | STEP pulse train, actual loop period | the honest way to check for jitter; a GPIO toggle + scope beats any software timer |
| **A second person or a phone tripod** | recording the first closed-loop move | you will want the footage and you should not be reaching for a camera with a live arm |

---

## Procedure

### 1. Simulate before you touch the bench

```powershell
cd tools
python joint_sim.py
```

Update the model with the Phase 1 measurements **first**:

| `joint_sim.py` parameter | Replace estimate with |
| ------------------------ | --------------------- |
| reducer stiffness (300 N·m/rad, estimated) | your hang-a-mass measurement |
| backlash (0.5° assumed) | your dial-indicator measurement |
| inertia | computed from weighed masses |
| encoder noise | the spread you actually saw at rest in Phase 1 |

Then re-run. If the predicted overshoot and settling time move a lot, that is
the model telling you which parameter it was most sensitive to - write that down,
because it is the parameter worth measuring more carefully.

D10 already establishes what this simulation predicts: naive PID limit-cycles at
83 reversals/s at rest, a deadband of ≥ 1 count removes it entirely, and
feedforward takes overshoot from 16 % to 1 %. Phase 2's job is to confirm or
refute that on hardware.

### 2. Commission the home offset (D9)

1. Flash `closed_loop`, drivers **disabled**.
2. Jog or hand-move the joint to its mechanical reference - a printed witness
   mark, a hard stop, or a machined face. Use the same feature every time.
3. Read the mechanical angle, send `z` (`CMD_ZERO_HERE`), read back the offset.
4. Write it into `JOINT_HOME_OFFSET_DEG` in
   [../../include/joint_config.h](../../include/joint_config.h).
5. Power-cycle and confirm the joint reports the correct angle with no motion.

Re-do this whenever a magnet, hub, or encoder board is disturbed. It is the cost
of not having limit switches, and D9 says so explicitly.

### 3. Set the soft limits *before* enabling

`JOINT_MIN_DEG` / `JOINT_MAX_DEG` should sit **at least 5° inside** the
mechanical hard stops. Then prove the fault path works:

- Command a setpoint outside the limit → expect `FAULT_SOFT_LIMIT` and the
  driver disabled, not a clamp-and-continue.
- Unplug the encoder mid-move → expect `FAULT_ENCODER` and the driver disabled.
- Stop sending host packets (if running `host_link`) → expect `FAULT_COMMS`
  after 500 ms.

Each of these is a deliberate, filmed test. "Anything that can move the motor
must have a fault path that disables the driver" is a project rule; this is
where you prove it holds.

### 4. Tune, in this order

The order matters because feedforward changes what the gains have to do (D10).

1. **Feedforward alone**, PID zeroed. Command a profiled move. The joint should
   roughly follow. A constant following error at constant velocity is a **model**
   error - fix `VEL_FF_SCALE`, `GEAR_RATIO`, or `MICROSTEPS`, not Kp.
2. **Add the deadband.** `POSITION_DEADBAND_COUNTS = 2`. Confirm the joint is
   silent at rest. If it still buzzes, the encoder noise is worse than modelled -
   measure the noise before raising the deadband.
3. **Raise Kp** until disturbance rejection is adequate. Adequate means: push the
   joint by hand and it returns without oscillating. D10 found that Kp from 2 to
   64 changes tracking error by < 0.03° once feedforward is on, so if you find
   yourself needing a large Kp, ask what the feedforward is getting wrong.
4. **Kd only to damp** a real overshoot you measured, not one you expect.
5. **Ki last, and probably never.** Only if you have *measured* a persistent
   steady-state offset under load. `PID_INTEGRAL_LIMIT` must be in force before
   you enable it.

Live tuning: `k <kp> <ki> <kd>` in `closed_loop`.

### 5. Capture the step-response table

Three step sizes - something like 2°, 10°, 45° - because the plant is not
linear and a single step size hides that:

| Step (deg) | Rise time (ms) | Overshoot (%) | Settling ±0.2° (ms) | SS error (deg) |
| ---------- | -------------- | ------------- | ------------------- | -------------- |
| 2 | | | | |
| 10 | | | | |
| 45 | | | | |

Then the row that makes it engineering rather than tinkering:

| Step (deg) | Measured overshoot | `joint_sim.py` predicted | Δ | Explanation |
| ---------- | ------------------ | ------------------------ | - | ----------- |

A 30 % disagreement is normal and fine **if you can say why**. Common causes,
in order: stiffness estimate wrong, Coulomb friction not modelled, backlash
larger under load than measured statically, encoder filter phase lag.

### 6. Compare closed-loop steady-state error against open-loop backlash

This is the insight the bring-up checklist flags as interview-worthy. Open loop,
your accuracy floor is the backlash. Closed loop, the encoder sees through the
backlash, so the floor becomes the encoder resolution plus the deadband. Quote
both numbers and the ratio.

---

## Decisions you have to make in Phase 2

| ID | Decision | Trigger | What decides it |
| -- | -------- | ------- | --------------- |
| **P2-a** | **Deadband width** - 1, 2, or 3 counts | first closed-loop run | measured hunting at rest vs. the repeatability you are willing to give up. D10's sweep says 1 count is already enough; 2 is margin |
| **P2-b** | **Does Ki get enabled at all** | after a load test | only a *measured* persistent offset justifies it |
| **P2-c** | **Encoder filter alpha** | if the loop oscillates | filtering adds phase lag, which destabilises the very loop it is smoothing. Lower Kp before filtering harder |
| **P2-d** | **Hold-when-idle or disable-when-idle** | after measuring motor temperature holding a load | the drive is ~75 % backdrivable (D2), so a disabled joint sags. Holding costs heat and current; sagging costs your absolute-homing story |
| **P2-e** | **Brake on J2/J3?** | if sag under power-off is unacceptable | D2 raised this and left it open |
| **P2-f** | **Control loop rate** - currently 200 Hz (`CONTROL_PERIOD_MS = 5`) | if you measure loop overruns | the I2C encoder read is ~158 µs at 400 kHz; that budget shrinks fast per joint |
| **P2-g** | **Whether the measured plant justifies re-tuning the trajectory limits** | after the max-speed test | `TRAJ_MAX_VEL_DEG_S` must stay under the driver ceiling (D10) |

---

## Troubleshooting - Phase 2 specific

### The joint hums at rest even with a deadband
The deadband is in encoder counts and the buzz is in microsteps. Confirm the
controller is actually inside the band (log the error in counts, not degrees). If
it is, the noise is downstream of the controller - usually the driver's
`stealthChop`/`spreadCycle` transition, not the loop.

### It tracks well but overshoots every time by the same amount
Feedforward scale. `VEL_FF_SCALE` is trimming a **model** error. Overshoot that
is proportional to commanded velocity is feedforward too high; undershoot is too
low.

### Following error is large and looks like bad tuning
Check saturation first. If `TRAJ_MAX_VEL_DEG_S` exceeds
`MAX_SPEED_STEPS_PER_SEC / STEPS_PER_OUTPUT_DEG`, the loop is asking for step
rates the driver cannot produce and no gain will fix it. This is documented in
D10 because it already cost a debugging session once.

### Oscillation that starts only above a certain amplitude
You have excited the reducer's torsional mode. The modelled first mode is
~14 Hz. Confirm by measuring the oscillation frequency from the log - if it
matches, the fix is lower gain or a stiffer drive, not more Kd.

### The joint drifts slowly over many cycles
Skipped steps, and the encoder is now hiding them from you. That is the closed
loop working as intended, but it means the open-loop capability is degrading.
Log commanded steps vs encoder-derived position and look for a monotonic gap.

### It faults on `FAULT_SOFT_LIMIT` immediately at power-on
The D9 wrap bug's signature. A joint parked at −30° reads 330 raw.
`AS5600Encoder::homeAbsolute()` handles this; if you see it, the offset or the
`ENCODER_DIRECTION` sign is wrong.

### Everything works, then it doesn't, after ~20 minutes
Thermal. Check motor temperature, driver temperature, and whether the printed
parts near the motor have softened. PETG's glass transition is ~80 °C and a
motor can reach that.

---

## Resources

### Control theory, in the order it is useful here
- **Brian Douglas** (YouTube, `@BrianBDouglas`) - "PID Control" series, then
  "Understanding Bode Plots". The intuition is the point.
- **MATLAB Tech Talks** (YouTube, `@MATLAB`) - "Control System Design" and
  "Understanding PID Control" series. Short and unusually well made.
- **Feedback Systems**, Åström & Murray - free at `fbsbook.org`. Chapters 1-2
  and 10 (PID) are the relevant ones.
- Search: *"derivative on measurement vs derivative on error"* - this repo
  already does the former; understanding why is a good interview answer.
- Search: *"integrator windup anti-windup clamping"*.

### Feedforward and why it dominates here
- Search: *"feedforward vs feedback control why feedforward"*.
- Search: *"velocity feedforward servo tuning"* - the CNC/machine-tool community
  explains this better than the academic sources, because they live with it.

### System identification - how to get the plant model honestly
- Search: *"step response second order system identification damping ratio"* -
  from a single step response you can extract $\omega_n$ and $\zeta$, which is
  how you back out stiffness and damping without a shaker.
- Search: *"log decrement damping ratio ring down"* - the tap test.
- **Modern Robotics** chapter 8 (dynamics of open chains) for where the inertia
  numbers come from.

### Safety
- Search: *"machine safety category 0 stop vs category 1 stop"* - the vocabulary
  for "cut power immediately" vs "decelerate then cut power". Knowing which one
  you built and why is a real answer to a real interview question.
- Search: *"ISO 10218 collaborative robot safety"* - background only; nothing
  here is compliant and you should not claim it is.

---

## Exit checklist

- [ ] Step-response table filled for three step sizes, plots saved
- [ ] Sim-vs-measurement comparison table filled, disagreements explained
- [ ] `joint_sim.py` updated with measured stiffness, backlash, inertia
- [ ] `FAULT_SOFT_LIMIT`, `FAULT_ENCODER`, `FAULT_COMMS` each deliberately
      triggered and filmed
- [ ] E-stop cuts VMOT, logic survives, verified under load
- [ ] `JOINT_HOME_OFFSET_DEG` commissioned and the procedure written down
- [ ] Closed-loop steady-state error vs open-loop backlash, both quoted, ratio
      explained
- [ ] Motor and driver temperature after 20 min of holding a rated load
