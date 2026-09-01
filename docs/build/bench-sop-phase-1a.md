# Bench SOP - Phase 1A, electronics only

Two sessions you can run while the reducers print. Nothing here needs a
gearbox, and every item produces a number that something downstream is
currently guessing at.

Companion to [../bringup-checklist.md](../bringup-checklist.md) (the electrical
gate order) and [../test-plan.md](../test-plan.md) §5 (the record protocol).
This file is the sequence, the stop points, and the abort criteria.

---

## Before you start

**Two small prints jump the queue.** Both are under 30 minutes total and every
item below is blocked without them:

1. The bench plate from
   [phase-1-first-joint.md](phase-1-first-joint.md#assembly-in-order) step 2 -
   NEMA 17 bolt pattern plus a slotted AS5600 boss. Slot the boss; you will
   change the gap three times tonight.
2. A **spool**: a printed drum that presses onto the motor shaft, with a known
   radius and a hole for a string. Make it 10.00 mm radius and measure the
   as-printed value with calipers - that measured number is a term in every
   torque result tomorrow.

Also put a **witness mark** on the motor shaft and the spool face. Half of
tomorrow's protocol uses it.

### Record protocol

One folder per session. Every stop point below writes a line into the `.md`:

```
docs/test-results/p1a_<app>_<YYYYMMDD-HHMM>.csv     raw telemetry
docs/test-results/p1a_<app>_<YYYYMMDD-HHMM>.md      the note
```

First line of every note, without exception:

```
git rev-parse --short HEAD
```

A measurement you cannot tie to a firmware version is not a measurement.

### Standing abort criteria

Stop, power down, and write down what you saw if any of these happen:

- Motor case above 80 °C, or driver too hot to touch
- PSU leaves constant-voltage mode (it is current limiting - you have a fault)
- `magnet=TOO_FAR` / `TOO_CLOSE` appears and does not clear
- Any smell of hot plastic or varnish
- `i2c_err` climbing continuously rather than sitting at a fixed count

**Never unplug a motor lead with VMOT live.** It is the one mistake that ends
the session by killing the driver.

---

## Session 1 - tonight, the sensing chain (~3 h)

Motor power stays OFF for steps 1.1-1.5. The point of this session is that an
electrical fault and a mechanical fault cannot be confused, because there is no
mechanism yet.

### 1.0 Snapshot the configuration - 5 min - P0

```powershell
git rev-parse --short HEAD
```

Copy `MICROSTEPS`, `GEAR_RATIO`, `I2C_CLOCK_HZ`, `MAX_SPEED_STEPS_PER_SEC`,
`ACCEL_STEPS_PER_SEC2` out of [../../include/joint_config.h](../../include/joint_config.h)
and [../../include/pins.h](../../include/pins.h) into the note. `GEAR_RATIO`
must read `1.0` for this whole session.

> ■ **STOP AND RECORD** - git SHA, the five config values, PSU voltage setting,
> ambient room temperature.

### 1.1 Gate 1 - bus alive - 10 min - P0

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e i2c_scan -t upload
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" device monitor
```

Expect `0x70` (mux) and `0x36` behind channel 0.

**Abort if** the mux answers but the AS5600 does not: that is wiring on the
channel-0 header, and no later step is meaningful until it is fixed.

> ■ **STOP AND RECORD** - addresses found, on which channels.

### 1.2 Resolve phase current - 30 min - P0 (blocks P1-a, P1-b, P1-g)

This is the single biggest hole in the 2026-08-31 session.
[torque_budget.py](../../tools/torque_budget.py) sizes the entire arm from
`MOTORS = {"23 mm": 0.13, "40 mm": 0.42, "60 mm": 0.68}` N·m - datasheet
holding torque **at rated current**. You do not know your current, so you do
not know your torque, so the motor-length tiering in D5 is currently a guess.

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e vref_setup -t upload
```

That app holds STEP/DIR low and the driver disabled, with no I2C and no timing,
so the board is electrically static while you probe.

**Finding the wiper.** Your 4.33 V reading was almost certainly VCC_IO or the
5 V rail; Vref on a TMC2209 StepStick is normally 0.3-1.5 V. Identify it by
*behaviour, not by pad label*: put the meter on a candidate point and turn the
trimpot a quarter turn. **Vref is the only point whose voltage moves.** Probe
the pot's three legs directly if the board has no via. Use a plastic trimmer
tool - a steel driver bridges the wiper to the pad and gives a false reading.

Then `I_rms = Vref / (2.5 * R_sense)` for the common StepStick topology -
**check your board's sense resistor** (0.11 Ω vs 0.15 Ω) before trusting any
formula off the internet, they differ by 36 %.

**If the wiper stays unfindable, do not burn the evening on it.** Two fallbacks,
both better than a label:

- *PSU power method.* Driver enabled, motor stationary and holding. Read PSU
  current. `I_psu * V_psu ≈ 2 * I_rms² * R_phase + ~0.5 W driver loss`, so
  `I_rms ≈ sqrt((I_psu*V_psu - 0.5) / (2*R_phase))`. Good to ~15 %, which is
  enough to tell 0.5 A from 1.5 A.
- *Torque method (best, and you are building the rig anyway).* Defer to step
  2.3 tomorrow. Holding torque is linear in current, so
  `I_actual ≈ I_rated * T_measured / T_datasheet` gives you the current **in
  the units the torque budget actually wants**, and sidesteps Vref entirely.

> ■ **STOP AND RECORD** - Vref in volts *or* the words "wiper unresolved",
> sense resistor value, computed `I_rms`, which method you used, and the exact
> pot position (photograph the screw slot angle - it is the only way to return
> to this setting).

### 1.3 Gate 2 - magnet gap sweep - 30 min - P1

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e encoder_test -t upload
```

Press `s` for status. Set the gap with feeler gauges at **1.0, 1.5, and
2.0 mm**, recording AGC at each. Target AGC ≈ 128; the acceptable band is
40-200. Pick the gap whose AGC is closest to 128 and lock the boss.

Then turn the shaft **by hand through three full revolutions**, slowly, watching
the streaming angle. You are looking for flat spots and jumps, not accuracy.

**Abort if** AGC pins at 0 or 255 at every gap - the magnet is axially
magnetised, not diametric, and no amount of gap fixes that.

> ■ **STOP AND RECORD** - AGC at each of the three gaps, chosen gap, magnitude,
> and whether three hand revolutions produced 1080° with no jumps.

### 1.4 Encoder noise floor, motor unpowered - 20 min - P0

**This is the most under-rated measurement of the night.** Every error bar you
quote for the rest of the project is compared against this number, and the D17
budget currently assumes quantisation-only noise.

Motor power OFF. Do not touch the bench. Log 60 s of a stationary encoder:

```powershell
python scripts/serial_logger.py --port COM3 --name p1a_noise_static
```

Compute standard deviation and peak-to-peak in **counts** (1 count = 0.088°).

> ■ **STOP AND RECORD** - σ in counts, peak-to-peak in counts, sample count,
> `i2c_err` at start and end. If σ > 1 count with the motor unpowered, you have
> a wiring or supply problem, not a sensor limit - fix it before step 1.5.

### 1.5 Encoder noise with the motor energised - 20 min - P0 (decides P1-e)

Motor power ON. Driver **enabled but commanded to hold** - no motion. Repeat the
60 s log. Then repeat again while stepping slowly at 400 steps/s.

The delta between 1.4 and 1.5 is coupling from the phase wires into the I2C
pair, and it is the entire basis of the P1-e cabling decision. Ordered fixes if
the delta is large: physically separate the encoder cable from the phase wires,
twist SDA/SCL, add 100 nF at the AS5600 VCC, then shielded cable grounded at the
Uno end only.

Dropping `I2C_CLOCK_HZ` is a **diagnostic, not a fix** - if it helps you have
confirmed signal integrity and should go fix the wiring.

> ■ **STOP AND RECORD** - three σ values (unpowered / holding / stepping),
> `i2c_err` accumulated over 10 minutes of stepping, and your P1-e call with the
> number behind it.

### 1.6 Encoder linearity by hand - 40 min - P1

D17 carries an INL term worth **4.64 mm** at the tool - the second largest in
the budget - and it is currently a datasheet number. Measure yours.

Tape a printed 360° protractor behind the shaft. Hand-turn to each 30° mark,
twelve positions, and log the encoder reading at each. Deviation from the ideal
line is your combined INL + magnet-eccentricity error.

> ■ **STOP AND RECORD** - twelve (protractor_deg, encoder_deg) pairs. Worst
> deviation in degrees. This replaces a datasheet row in `error_budget.py`.

### 1.7 Start the lubricant compatibility coupons - 15 min - P1

**Do this last, before you go to bed.** It needs ~12 h to develop, so it costs
you nothing but sleep time, and it gates a decision you make at 1B assembly.

`cycloidal_layout.py` says grease is worth ~27 efficiency points, so the
lubricant is not a detail. But PETG is a copolyester and susceptible to
**environmental stress cracking** - a part under load plus a marginal chemical
crazes at a stress neither would cause alone. Your disc lobes sit at ~47 MPa
contact stress. That is exactly the ESC scenario, and it does not show up in a
"drop it on a flat coupon and see" test.

So load the coupons:

1. Print 5-6 strips, roughly 60 × 10 × 1.5 mm, same material and settings as
   the discs will use.
2. Bend each over a former (a socket, a marker barrel - anything ~25 mm) and
   zip-tie it so it holds a constant bend. **The strain is the whole point.**
3. Wet the outside (tension side) of each with one candidate: white lithium
   from a tub, silicone grease, PTFE synthetic grease, gun oil, WD-40, and one
   control left dry.
4. Label them. Leave overnight.

Next morning, wipe clean and look at the tension face under a bright light at a
low angle. You are looking for **crazing** - fine white cracks perpendicular to
the strain. Also flex each one and see if it snaps where the dry control does
not.

> ■ **STOP AND RECORD** - which candidates crazed, which didn't, photographs of
> the tension face. Any candidate that crazes a strained coupon is disqualified
> regardless of how good its lubricity is. Add the result to
> [../design-decisions.md](../design-decisions.md) - it is a materials decision
> the whole build inherits.

---

## Session 2 - tomorrow, torque and step rate (~4-5 h)

Needs the spool, string, and known masses. Everything here uses the new
`pullout` app.

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e pullout -t upload
```

Keys: `<number>` = step rate, `a<number>` = acceleration, `c`/`v` = run
forward/reverse, `x` = stop and print the summary, `l` = toggle encoder
streaming, `z` = zero, `?` = status.

**The `x` summary line is the deliverable.** It prints `keepup_pct`, and that
column is why this app exists - see 2.1.

### 2.1 The Uno's step-rate ceiling - 30 min - P0, DO THIS FIRST

There are two ways to "not keep up" and they look identical in a position log:
the motor skipped, or `loop()` never issued the step. An AS5600 read is ~158 µs
at 400 kHz, so with the encoder streaming the loop cannot run much past a few
kHz, and **a torque-speed curve measured above that ceiling is a plot of the
ATmega328P, not of the motor.**

No load, no string. Encoder ON. For each rate in
400, 800, 1600, 3200, 4800, 6400, 8000 steps/s: type the rate, `c`, wait 5 s,
`x`. Read `keepup_pct` off the summary.

Then press `l` to turn encoder streaming off and repeat.

> ■ **STOP AND RECORD** - the highest rate with `keepup_pct` ≥ 98 in each mode.
> Call these `f_max_enc` and `f_max_open`. Their difference is the cost of the
> encoder in the control loop and is direct D13 evidence for leaving the Uno.
> **Every rate in steps 2.4 and 2.5 must stay below the relevant ceiling.**

### 2.2 Reality check on what is reachable - 10 min - P0

Run:

```powershell
cd tools
& "C:/Users/koashen/AppData/Local/Python/pythoncore-3.14-64/python.exe" motor_id.py
```

`corner_step_rate()` puts the torque roll-off knee near **2700 full steps/s at
12 V** and **5400 at 24 V**. At 8× microstepping those are 21,600 and 43,200
microsteps/s. Compare against `f_max_open` from 2.1.

You will almost certainly find the corner is **not reachable on an Uno**. That
is a finding, not a failure - write it down. It means:

- the pull-out sweep measures `T0` and confirms the curve is flat over your
  working range, but **cannot identify `fc`**;
- `fc` comes from the electrical model plus the motor's datasheet `L` and `R`
  until better hardware exists;
- self-check [4] in `motor_id.py` shows what happens if you fit `fc` to
  low-speed data anyway: half the noise seeds go non-physical and the rest span
  4×. Do not quote a corner speed you did not reach.

> ■ **STOP AND RECORD** - `f_max_open` vs the modelled corner, as a percentage.
> Whether `fc` is measurable on this controller: yes/no.

### 2.3 Holding torque, and phase current from it - 45 min - P0

Spool on the shaft, string over the edge of the bench, mass hanging. Driver
**enabled, zero commanded velocity**. Add mass in steps until the shaft slips.

`T_hold = m * g * r` using your **measured** spool radius.

Then close out step 1.2: `I_actual ≈ I_rated * T_measured / T_datasheet`.

Do this in **both directions** and at three shaft index positions 120° apart -
a stepper's holding torque varies with where in the electrical cycle it is
resting, and one measurement at one position can be 20 % off.

> ■ **STOP AND RECORD** - six `T_hold` values (2 directions × 3 positions),
> their mean and spread, spool radius, resulting `I_actual`, and the Vref /
> pot position it corresponds to. **This number replaces a datasheet row in
> `torque_budget.py`.**

### 2.4 Pull-out points - 90 min - P0

For each of three masses (light / medium / near-stall from 2.3):

1. Type a starting rate well under `f_max_enc`. Press `c` to lift. Run 5 s.
   Press `x`.
2. Read `net_slip_steps` off the summary. **Zero net slip = passed.**
   Also confirm `keepup_pct` ≥ 98 - otherwise the row is invalid, not a skip.
3. Bisect the rate until you find the highest passing rate.
4. Cross-check the winner with the witness mark: command a whole number of
   revolutions and confirm the mark returns.

Convert to full steps/s with `full_steps_per_sec(rate, microsteps=8)`.

**Rules that decide whether the data means anything:**

- Measure only the constant-velocity window. Use a low acceleration
  (`a1000`) so the ramp is clearly separate from the cruise. Accelerating
  samples put `J·α` into the torque number.
- **Do not touch the trimpot for the whole sweep.** Phase current is the other
  axis of this surface; changing it mid-sweep makes the fit meaningless.
- Run at the voltage you intend to ship (24 V), not 12 V. Corner speed is
  proportional to supply voltage.
- Record the failures. A run that stalled is data about the stall boundary.

> ■ **STOP AND RECORD** - a table of `full_steps_per_sec, torque_nm` rows, one
> per mass. Save it as `docs/test-results/p1a_pullout_<date>.csv` with the
> header `# full_steps_per_sec,torque_nm` so `motor_id.load_pullout_csv()`
> reads it directly. Then run `pullout_fit()` on it and record `T0`.

### 2.5 Acceleration ceiling - 30 min - P1

Fix the step rate at 60 % of the best passing rate from 2.4 with the medium
mass. Sweep acceleration: `a1000`, `a2000`, `a4000`, `a8000`, `a16000`, each
with `c` … `x`. Find the highest acceleration with zero net slip.

`ACCEL_STEPS_PER_SEC2` is currently 4000 by assumption. This measures it.

> ■ **STOP AND RECORD** - highest passing acceleration, and whether 4000 was
> conservative or optimistic.

### 2.6 Thermal soak - 30 min, mostly waiting - P0 (blocks housing design)

Run continuously at a typical working rate with the medium load. Log motor case
temperature every 2 minutes for 20 minutes, plus the driver.

This is a **mechanical** requirement, not an electrical one: the printed joint
housing bolts directly to this motor's face, and PETG softens near 80 °C. If
the case stabilises above ~60 °C the reducer design needs a thermal break or a
lower current, and you want to know that before the housing is drawn.

> ■ **STOP AND RECORD** - case temperature vs time, the plateau value, driver
> temperature, ambient. State whether PETG is acceptable at that plateau.

### 2.7 Microstep linearity - 30 min - P1

Slowly single-step 64 microsteps (one full electrical cycle at 8×), logging the
encoder after each. Ideal is 0.225° per microstep at `GEAR_RATIO = 1.0`.

TMC2209 stealthChop has real microstep positional error. Measuring it now, with
no gearbox in the way, is the only chance to separate motor+driver error from
mechanism error. **Whatever you do not measure here you will blame on the
gearbox in Phase 1B.**

> ■ **STOP AND RECORD** - σ and worst-case deviation in degrees per microstep,
> and the peak-to-peak error over the full electrical cycle.

### 2.8 Stretch: closed loop at ratio 1 - P2

If there is time, Gates 4 and 5 with `calibration` then `closed_loop`. Three
step sizes, record rise time / overshoot / settling / steady-state error via
`scripts/analyze_log.py`.

The value is not tuning - it is that `tools/joint_sim.py` **predicts** these,
and D16 says validation depth comes before control sophistication. Two
predictions worth checking:

- a deadband of ≥ 1 count takes dither from ~83 reversals/s to zero
- feedforward cuts overshoot from ~16 % to ~1 %

> ■ **STOP AND RECORD** - the four step-response numbers per step size, and the
> measured-vs-predicted gap as a percentage. That percentage is the first real
> entry for R-12.

---

## What each number unblocks

| Step | Produces | Currently a guess in |
| ---- | -------- | -------------------- |
| 1.2 / 2.3 | phase current, `T_hold` | `torque_budget.MOTORS`, D5 motor tiering, P1-b |
| 1.4 / 1.5 | encoder noise floor, coupling | `error_budget.py`, P1-e |
| 1.6 | encoder INL | the 4.64 mm INL term in D17 |
| 2.1 | Uno step-rate ceiling | D13's "when to leave the Uno" |
| 2.4 | `T0`, pull-out points | `motor_id.ratio_sweep()`, P1-a ratio choice |
| 2.5 | acceleration ceiling | `ACCEL_STEPS_PER_SEC2`, trajectory limits |
| 2.6 | motor case temperature | whether the housing can be PETG at all |
| 2.7 | microstep positional error | the Phase 1B backlash measurement's baseline |

## What you still cannot learn without a gearbox

Do not try to infer these tonight - they need the reducer and they are the
Phase 1B deliverables: backlash, torsional stiffness, transmission efficiency,
and measured gear ratio. `joint_sim.py`'s 300 N·m/rad stiffness stays flagged as
an estimate until then, and so does the friction coefficient that
[cycloidal_layout.py](../../tools/cycloidal_layout.py) computes efficiency
from - the model is only as good as its `mu`, and `mu` for greased PETG is a
literature range, not a measurement.
