# Phase 3 - build guide: two joints, then the controller migration

Phase 3 is where the project stops being "a joint" and becomes "a machine". Two
things happen and they are easy to conflate:

1. **A second joint** - coordination, synchronisation, and the fact that joint 2
   is a disturbance source for joint 1.
2. **The Uno running out** - `coordinated` already measures 73.9 % RAM with three
   joints (D13), and the step-rate ceiling binds before that. This is the phase
   where you migrate controllers.

Do them in that order. Migrating hardware while also debugging coordination
means every bug has two possible homes.

---

## Objectives

1. A synchronised two-joint move where **both axes arrive within tolerance at
   the same time**, plotted, with the arrival spread quoted in milliseconds.
2. A measured aggregate step-rate ceiling on the Uno - the number that turns
   D3's prediction into a fact.
3. A migration to the Teensy 4.1 with a **before/after** measurement of loop
   rate and jitter.
4. Cross-axis disturbance quantified: how much does moving J2 perturb J1?

---

## Parts

### Second joint module

Everything from [phase-1-first-joint.md](phase-1-first-joint.md) part 1B, again.
Ratio and motor length per the (now closed) decisions P1-a and P1-b.

Plus, per additional joint:

| Qty | Part | Notes |
| --- | ---- | ----- |
| 1 | TMC2209 | |
| 1 | AS5600 + diametric magnet | next free mux channel |
| 1 | Link between the joints | printed **flat on the bed** - layer planes perpendicular to bending load (D8) |
| - | Shielded twisted pair, encoder | shield grounded at the controller end only |
| - | Drag chain or wire loom | motors-in-joints means wires cross every axis |

### Controller migration

| Qty | Part | ~Cost | Why now |
| --- | ---- | ----- | ------- |
| 1 | **Teensy 4.1** | $32 | 600 MHz M7 **with FPU**, hardware timers, built-in CAN. Every `lib/` class is header-only Arduino API, so this is a port and not a rewrite |
| 1 | Printer control board (SKR Pro / Octopus) *or* a carrier you make | $50-70 | 8 driver sockets, TMC UART, endstop headers, 24 V distribution, **fusing** - solves five problems at once |
| 1 | Level shifter board | $5 | Teensy is 3.3 V. TMC2209 STEP/DIR/EN tolerate it; **check your AS5600 breakout** - many are 5 V-only |
| 1 | Micro-SD card | $8 | on-board high-rate logging without a USB tether. This is how you get a 1 kHz trace |
| 1 | Spare TMC2209 | $8 | |

### Rigid base

Not optional once two joints move together. A joint decelerating a link will
walk an unclamped arm across the desk, and that shows up in your repeatability
number as a mystery. An 18 mm plywood or 6 mm aluminium plate, clamped to the
bench, is enough.

---

## Tools

Phase 1 and 2 tools, plus:

| Tool | Used for | Note |
| ---- | -------- | ---- |
| **Oscilloscope or logic analyser** | measuring the *actual* loop period and step-pulse jitter | a $15 8-channel logic analyser + PulseView is enough and is the single highest-value tool purchase of this phase |
| **Crimp tool for JST-GH / Micro-Fit** | joint-to-joint harnesses | Dupont connectors on a moving arm will intermittently open, and you will chase it as a firmware bug |
| Label maker or tape + marker | wire identification | six joints × 6 wires is the point where "I'll remember" fails |
| Dial indicator + magnetic base | two-axis arrival test at the tool point | |
| Digital angle gauge | independently checking joint angles against the encoders | |
| Continuity tester with long leads | tracing a harness through a drag chain | |

---

## Procedure

### 1. Second joint, alone first

Repeat Phase 1 gates 3-5 and Phase 2 on the new joint **in isolation**, on mux
channel 1. Each joint gets its own row in the calibration table: measured ratio,
`ENCODER_DIRECTION`, backlash, home offset. They will differ, and assuming they
do not is a class of bug that presents as coordination failure.

### 2. Measure the Uno's real ceiling before you leave it

This is a deliverable, not a formality. D3 predicts ~5 kHz aggregate from
`digitalWrite` costing ~4 µs on AVR. Measure it:

1. Toggle a spare GPIO once per control-loop iteration. Scope it.
2. Record the loop period with 1 joint, 2 joints, 3 joints
   (`multi_joint` already instantiates three).
3. Ramp commanded speed until you see missed step deadlines.
4. Report: **aggregate steps/s, loop rate in Hz, and jitter in µs** for each
   joint count.

That table plus the D13 RAM figures is the complete, quantified argument for
migrating. It is worth more in an interview than the migration itself.

### 3. Coordinated motion on the Uno

`app_07_coordinated` (env `coordinated`) stretches every axis's trapezoid to the
slowest axis's minimum duration, so all axes start and finish together.
`test/test_motion` already asserts the arrival spread is within the jerk
filter's tap count; the bench has to confirm it.

Test to run:

- Command J1 through 45° and J2 through 10°, simultaneously.
- Plot both encoders on one time axis.
- Quote the **arrival spread**: the time between the first axis entering
  tolerance and the last. At 200 Hz with `JERK_FILTER_TAPS = 4`, expect
  ≤ 20 ms.
- Then run the naive version (each axis at its own max speed) and plot the tool
  path. The difference between the two paths is the entire reason layer 1 exists
  (D7) and it makes an excellent single figure for a poster.

### 4. Cross-axis disturbance

Hold J1 closed-loop at a fixed setpoint. Move J2 through its full range at max
speed. Log J1's error.

What you are measuring: base compliance, harness drag, and the reaction torque
of J2's acceleration. If J1's error under this test is comparable to its
repeatability, the arm's stiffness - not the controller - is your accuracy limit,
and that is a finding worth writing up.

### 5. Migrate to the Teensy

Do this as a **port with a measurement on both sides**, not a rewrite.

1. Add a `teensy41` environment in `platformio.ini` reusing the same
   `build_src_filter` as the app you are porting.
2. Fix the 3.3 V issues: level shift anything 5 V, re-check I2C pull-up
   voltages, confirm the AS5600 breakout's regulator.
3. Build unchanged first. The header-only `lib/` classes are plain Arduino API
   and should compile as-is. **Anything that does not compile is a portability
   bug worth understanding, not worth papering over.**
4. Re-run the same loop-rate and jitter measurement from step 2. Report
   before/after.
5. *Then* move step generation to a hardware timer (`IntervalTimer` on Teensy).
   Not before - you want the naive port's number for comparison.

Note the Timer1 constraint from D6 does not follow you: the ATmega328P `Servo`
library's Timer1 claim is an AVR problem. On Teensy the gripper uses a different
timer entirely, so one of the Uno's structural constraints simply evaporates.
Say so when you explain the migration.

### 6. Consider SPI encoders

The trigger fires here (bom.md tier 2): third encoder, **or** the first cycloidal
reducer, whichever comes first - and Phase 1B already fired the second condition.
At 20:1 one microstep is 0.011° at the output and the AS5600 resolves 0.088°, so
the encoder is the accuracy floor (D3). **Note:** D17's error budget shows this
is about read latency and mux fragility, not accuracy - encoder quantization is
near the bottom of the tip-error budget.

`AS5600Encoder` deliberately hides the transport behind a small interface so this
swap barely touches the apps. Keep it that way. When you do swap:

| | AS5600 + TCA9548A | AS5047P / MA732 SPI |
| - | ----------------- | ------------------- |
| Resolution | 12 bit, 0.088° | 14 bit, 0.022° |
| 6 encoders, read time | ~4200 µs @ 100 kHz | ~30 µs |
| Wiring | 4 wires + a mux | shared SPI bus + 1 CS each |
| Absolute | yes, single turn | yes, single turn |

---

## Decisions you have to make in Phase 3

| ID | Decision | Trigger | What decides it |
| -- | -------- | ------- | --------------- |
| **P3-a** | **When to migrate off the Uno** | your measured step-rate ceiling | you now have the number. D13's 73.9 % RAM is the other half |
| **P3-b** | **Teensy 4.1 vs one MCU per joint (CAN)** | at migration time | D7 fixes the *layer boundaries*, not the silicon. Teensy first is strictly cheaper and the CAN split stays available |
| **P3-c** | **Printer control board vs a hand-wired carrier** | when the third driver goes in | a board buys fusing, 24 V distribution and TMC UART; hand-wiring buys understanding and costs evenings |
| **P3-d** | **SPI encoder swap now or at Phase 4** | third encoder or first reducer | already triggered. The question is only budget timing |
| **P3-e** | **Harness: bundle with service loop vs slip ring** | before the joint housings are final | joint travel range. D9 assumes ≤ 300°, which a service loop supports |
| **P3-f** | **Streaming setpoints vs segment handoff** | if comms jitter shows up in the motion | D7 says start with streaming; move to segments only if jitter proves limiting. Now you can *measure* jitter |
| **P3-g** | **Base mounting - desk surface or table edge** | before the base plate is drilled | `workspace.py` reports a −264 to +484 mm vertical span; the negative half only exists at a table edge (D8b) |
| **P3-h** | **Whether to add endstops** | before an unattended run | D9 does not need them for homing, but they are a firmware-independent safety layer, which is a different argument |

---

## Troubleshooting - Phase 3 specific

### Both joints work alone, neither works together
Almost always power. Two motors at 1.4 A RMS from a supply set for one is a
brownout, and a browned-out Uno resets mid-move. Check the 5 V rail with a scope,
not a multimeter - the meter averages away the dip.

### `i2c_err` appears only when both motors run
Coupling, now doubled. The mux is the single point of failure: every encoder read
goes through it, so one corrupted mux-select byte loses *all* joints. This is a
strong argument for the SPI swap, and it is worth stating that way.

### One joint's error spikes exactly when the other accelerates
Expected - that is the cross-axis disturbance from step 4. Quantify it before
"fixing" it. If it is a real problem, the fixes in order are: stiffer base, lower
acceleration, then feedforward of the coupling term (which is Phase 5 territory).

### Arrival spread is much larger than the jerk filter's taps
Check that both axes are using the *same* stretched duration.
`MotionController` stretches to the slowest axis; if one axis's limits were
edited and the other's were not, they will plan different durations and the
synchronisation is silently defeated.

### The arm walks across the desk
Not a joke. Clamp the base. Then re-measure repeatability, because the number you
had was partly measuring the desk.

### After the Teensy port, everything is subtly worse
Check in this order: 3.3 V logic thresholds at the TMC2209 (usually fine, but
check), I2C pull-ups now pulling to 3.3 V into a 5 V AS5600, `micros()` rollover
assumptions, and `float` vs `double` - on AVR they were the same type, on ARM
they are not, and a `double` in a hot loop is now genuinely slower than a
`float`.

### Wires break inside the joint
Solid-core wire, or no service loop. Silicone-jacketed stranded, generous loops,
and strain relief at both ends of every crossing. Budget for redoing this once.

---

## Resources

### Multi-axis coordination and motion control
- Search: *"trajectory time scaling synchronised multi-axis motion"*.
- Search: *"S-curve vs trapezoidal motion profile jerk"* - this repo approximates
  S-curve with a boxcar filter over the trapezoid; understanding what that
  approximates is the interesting part.
- **Modern Robotics** chapter 9 (trajectory generation) - short and directly
  applicable.

### Embedded real-time
- Search: *"stepper step generation hardware timer vs bit banging"*.
- **PJRC's Teensy documentation** (`pjrc.com/teensy`) - the `IntervalTimer` and
  FlexCAN pages specifically.
- Search: *"measuring interrupt latency and jitter with a logic analyser"*.
- Search: *"why real-time control does not belong on stock Linux"* - the answer
  is the D7 layer split, and being able to explain it is interview material.

### CAN and distributed control
- Search: *"CAN bus tutorial arbitration bit stuffing"* - the arbitration
  mechanism is why D7 chose 8-byte frames.
- Search: *"ODrive CAN protocol"* and *"moteus"* - read how shipping products
  structure a per-joint protocol; both are open and both are more thought-through
  than a first attempt.

### Encoders
- Search: *"AS5047P datasheet"*, *"MA732 datasheet"*.
- Search: *"dual encoder motor side and load side servo"* - the arrangement real
  robot joints use, and the eventual right answer here (D3).

### Mechanical
- Search: *"machine base stiffness why it matters robot repeatability"*.
- Search: *"cable management robot arm drag chain service loop"*.

---

## Exit checklist

- [ ] Both joints individually characterised - separate calibration rows
- [ ] Uno ceiling table: aggregate steps/s, loop Hz, jitter µs at 1/2/3 joints
- [ ] Synchronised move plotted, arrival spread quoted in ms
- [ ] Naive vs synchronised tool path, both plotted on one figure
- [ ] Cross-axis disturbance quantified in degrees of induced error
- [ ] Teensy port builds, runs, and has before/after loop-rate numbers
- [ ] Base rigidly mounted, repeatability re-measured after clamping
- [ ] Harness uses stranded wire and proper connectors, with a service loop
