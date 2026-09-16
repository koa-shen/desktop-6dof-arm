# 15:1 Cycloidal Reducer Characterization Protocol

Goal: turn "it moves smoothly by hand" into numbers you can design around and
compare against future reducer revisions.

## Setup
- Current build has the AS5600 magnet on the **motor/stepper shaft** (not the
  gearbox output) — see `hardware/pinouts/uno-tmc2209-as5600-tca9548a.md`.
  This means `measured_deg` is actual motor rotation, useful for catching
  skipped/missed steps, but it does **not** see the gearbox output — it
  cannot measure transmission error or backlash on its own. Output-side
  backlash needs the load-cell test in step 2 below.
- Run `src/main.cpp` (Phase 2 build). It auto-runs a sweep sequence on boot and
  logs `ms,step_pos,commanded_deg,measured_deg` over serial.
- Capture serial output to a file and save it into
  `docs/test-results/` using the `phase2-reducer-log-template.csv` header.
- Log the specific unit under test (serial/print number, date, motor current
  setting) in the filename or a header comment line.

## Automated (firmware-driven) measurements

1. **Stepper tracking error (not transmission error)**
   - From the sweep log: `error_deg = measured_deg - commanded_deg`.
   - With the encoder on the motor shaft this reports skipped/missed steps,
     not gearbox transmission error. Non-zero error here means the motor
     stalled or lost steps during the sweep — invalidates any log-derived
     numbers below it, so check this first.

2. **Repeatability (motor-shaft only)**
   - With `SWEEP_CYCLES > 1`, compare measured_deg across cycles at the same
     commanded_deg and same approach direction.
   - Report as +/- range (or std dev) in arcmin — this is motor-shaft
     unidirectional repeatability (skipped-step consistency), not output
     repeatability.

## Manual measurements (no firmware needed yet)

3. **Output-side backlash — load-cell + contact-switch dead-band test**
   - Requires new hardware: a load cell + amplifier (e.g. HX711 + small
     S-type/button load cell), and an Omron V-156-1C25 snap-action lever
     switch at the contact point (NO contact: closed while touching, open
     when separated — see `hardware/pinouts/uno-tmc2209-as5600-tca9548a.md`).
     No output encoder needed.
   - Mount the output arm to press against the load cell with a small,
     repeatable preload (~1–2 N).
   - Command the motor to reverse in small fixed increments (a few
     microsteps at a time), settling briefly before each sample. Log
     `motor_actual_deg` (AS5600), `force_N` (load cell), and `contact`
     (digital pin) per sample.
   - Find **t1**: first sample where `force_N` falls to ~0 (below a noise
     threshold, e.g. <2–3% of preload, sustained for a couple samples).
   - Find **t2**: first sample after t1 where `contact` opens (true
     mechanical separation — force alone can't distinguish this from
     "touching with zero force").
   - `backlash_output_deg = (motor_actual_deg[t2] - motor_actual_deg[t1]) / GEAR_RATIO`.
     Report in arcmin (`deg * 60`).
   - Repeat over several reversal cycles and at a few different output
     angles (cycloidal lash can vary with position); average.

4. **Holding torque / static friction (breakaway torque)**
   - With motor energized and holding position, apply increasing torque via a
     calibrated torque wrench or hanging-weight-on-lever-arm until slip.
   - Record breakaway torque in the loaded (output) direction.

5. **Backdrivability**
   - With motor de-energized (`PIN_EN` HIGH — firmware does this after the
     sweep), try to rotate the output by hand/torque wrench.
   - Note whether it backdrives freely, backdrives with resistance, or is
     self-locking. Record the torque needed if it moves.

6. **No-load input current / smoothness**
   - Measure motor current draw during a slow, unloaded sweep (bench PSU
     ammeter or inline shunt). Note any current spikes correlating with
     step-1 tracking error — indicates cogging/binding at specific
     rotor angles.

7. **Efficiency (optional, needs load cell/torque sensor)**
   - Output torque delivered / (input torque x gear ratio) at a fixed speed.
     Can reuse the load cell from step 3. Defer until step 3 is working —
     not required for Phase 2.

## Data to keep per unit tested
- Backlash (arcmin, from the load-cell dead-band test)
- Motor-shaft repeatability (arcmin) and tracking error (skipped steps)
- Breakaway holding torque
- Backdrivable? (Y/N + torque if yes)
- Any audible/tactile anomalies (clicking, binding at specific angles)

## Suggested next hardware step after this data is in
- If backlash/repeatability are within spec for wrist/end joints, move to
  loaded testing (hang a known mass at a known lever arm) to validate
  holding torque under real conditions before committing to all 6 joints.
