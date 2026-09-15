# Phase 1B - Reducer and Output Encoder Bench

This is an adaptive evidence sequence, not a rigid checklist. Repeat, reorder,
or stop after any step when an observation changes the likely failure mode.
Two gates are non-negotiable: validate the output encoder before motor power,
and measure the actual ratio before commanding output angles or enabling PID.

## Before motor power

1. Make a session folder and record the current Git SHA, assembly state,
   ambient temperature, supply setting, driver-current setting, and any changes
   from the prior assembly.
2. Turn the unpowered reducer through at least one input revolution in each
   direction. Record the input angle of every tight spot. Do not power a
   reducer that binds by hand.
3. Upload `encoder_test` and verify the AS5600 with the output turned by hand:

   ```powershell
   & "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e encoder_test -t upload
   & "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" device monitor
   ```

   Record magnet status, AGC, and whether one physical output revolution gives
   a continuous 360 degrees without jumps. Fix a marginal magnet gap or cable
   before connecting motor power.

## First powered observation

`reducer_bench` sends motor steps directly and streams the output encoder. It
does not use `GEAR_RATIO`, output soft limits, or PID, because none of those is
validated yet.

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e reducer_bench -t upload
python scripts/serial_logger.py --port COM3 --name p1b_reducer_bench
```

The driver starts disabled. In a serial monitor, first use `?`, then `e`, then
make one `+` jog: 160 motor steps, or 0.1 motor revolution at 8x microstepping.
Inspect the mechanism before continuing. `f` and `r` command one motor
revolution (1600 steps); `j <steps>` accepts a signed command capped at one
motor revolution. `x` immediately stops and disables the driver.

For every increment, record commanded motor steps, output direction, encoder
angle/turns, `magnet_ok`, `i2c_err`, audible behavior, and motor/driver case
temperature. Stop, power down, and record the configuration on binding, rapid
temperature rise, a changing I2C error count, invalid magnet status, or any
hot-plastic/varnish smell.

## How the measurements decide the next test

- If an incremental jog binds or produces an abnormal tight spot, return to
  hand inspection and check concentricity, disc-face rub, and pin clearance.
- If the output encoder loses status or jumps, return to magnet-gap and cable
  tests; do not diagnose mechanics from corrupted position data.
- If direct-step motion is smooth, mark the input and output and count input
  turns for one output revolution. Run in both directions and average the
  result. This measured ratio replaces `GEAR_RATIO = 1.0f` in
  [../../include/joint_config.h](../../include/joint_config.h).
- After the ratio is measured, derive output speed limits from the motor-step
  ceiling and run `calibration` unloaded. Its scale and backlash result is a
  check on the hand ratio measurement, not a substitute for it.
- Only after encoder validity, ratio, and unloaded calibration agree should
  `phase1_bench` and then `closed_loop` command joint output angles.

Run-in, backlash, stiffness, and efficiency tests may be interleaved as the
assembly stabilizes. Record each run and its raw CSV in the workbook and
`docs/test-results/`; test outcomes, rather than calendar order, decide the
next action.