# Troubleshooting

Symptom-first. Each entry names the failure domain so you know which app to go
back to.

---

## I2C

### `no TCA9548A at 0x70`
- SDA must be **A4**, SCL must be **A5** on an Uno. They are not swappable.
- Check A0/A1/A2 address jumpers - they shift the address to 0x71-0x77. Update
  `TCA9548A_ADDR` in [include/pins.h](../include/pins.h) if you changed them.
- VCC must be 5 V and GND must be shared with the Uno.
- Long/unshielded I2C wiring: shorten it, or drop `I2C_CLOCK_HZ` to 50000.
- Missing pull-ups. Most breakouts include 10k; bare chips do not.

### Mux found but no AS5600 on any channel
- The AS5600 breakout needs both its SDA/SCL going to the **channel** pins
  (SD0/SC0 for channel 0), not back to the Uno.
- Some AS5600 breakouts have a DIR pin that must be tied to GND or VCC.
- Check the encoder's own VCC/GND - it is easy to power it from the mux's
  channel header by mistake (channel headers carry data only on some boards).

### Intermittent `i2c_err` climbing during motion
- Sensor wiring is coupling with motor phase wires. Separate and/or twist them.
- No common ground between logic and motor domains.
- Add 100 nF from AS5600 VCC to GND at the encoder.
- Drop to 50 kHz to confirm it is a signal-integrity problem, then fix the
  wiring rather than living at 50 kHz.

---

## Encoder

### `magnet=NO_MAGNET`
Magnet is too far, not diametrically magnetized, or not over the die. AS5600
needs a **diametric** magnet (poles across the diameter, not through the
thickness). An axially magnetized magnet will never work.

### `magnet=TOO_FAR` / `TOO_CLOSE`
Target gap is 0.5-3 mm. AGC should read near 128. AGC pinned at 0 means too
close, pinned at 255 means too far.

### Angle is non-linear (fast in one region, slow in another)
Magnet is off-center relative to the sensor die. Even 0.5 mm of eccentricity
shows up clearly. Fix the mount, do not compensate in software.

### Angle jumps by ~360 deg
That is the single-turn wrap. `AngleUnwrapper` handles it, but only if you
sample faster than 180 deg of motion per sample. If you spin fast and log
slowly, you will alias. Raise the sample rate or lower the speed.

### Encoder does not move when the motor does
The magnet is on the motor shaft but the encoder reads the **output** axis, or
the coupling is slipping. Phase 1 requires the magnet on the joint output axis
for true absolute joint angle.

---

## Motor / TMC2209

### Motor buzzes but does not turn
- Coil pairs wired wrong. Use a multimeter continuity check: the two wires that
  beep together are one coil -> A1/A2, the other pair -> B1/B2.
- Current too low (Vref too low).
- Acceleration too aggressive for the current setting.

### Motor turns, then loses position over time
Skipped steps. Reduce speed/accel, raise current, or reduce load. Open-loop
steppers have no way to know - which is exactly why you added an encoder.

### Driver gets very hot / shuts down
Overcurrent. Lower Vref. Confirm the heatsink is on and airflow exists. Thermal
shutdown looks like the motor randomly going limp.

### Motor does not respond at all
- EN polarity. Most StepStick-style boards are **active LOW**; flip
  `DRIVER_EN_ACTIVE_LOW` in [include/pins.h](../include/pins.h).
- VMOT not connected (logic can be powered over USB while VMOT is dead - the
  board looks alive but the motor cannot move).
- Driver installed backwards. Check the EN/DIR/STEP silkscreen against the
  carrier board.

### Direction is backwards
`motor.setInvertDirection(true)`, or swap one coil pair. Do not "fix" it by
negating step counts - that will confuse the closed-loop sign convention.

---

## Motion quality

### Measured steps/deg is exactly 2x or 4x off
Microstep jumpers do not match `MICROSTEPS` in
[include/joint_config.h](../include/joint_config.h). MS1/MS2 combinations are in
the comments there.

### Encoder angle lags the command, then catches up
Backlash or belt stretch. Measure it with the `calibration` app. Always approach
final positions from the same direction if you need repeatability without a
closed loop.

### Closed loop oscillates
Kp too high, or the encoder filter is adding phase lag. Lower Kp first, then
lower `setFilterAlpha` aggressiveness (higher alpha = less filtering = less lag).

### Closed loop hums/vibrates at rest
Deadband problem: the controller keeps commanding tiny step rates. Increase
`POSITION_TOLERANCE_DEG`, or add a small deadband so the PID output is zeroed
inside the tolerance.

---

## Toolchain

### `pio` not recognized in PowerShell
Use the CLI shipped with the VS Code extension:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e phase1_bench
```

Or add `%USERPROFILE%\.platformio\penv\Scripts` to PATH.

### `pio test -e native_tests` says `'gcc' is not recognized`
No host compiler installed. Either install MinGW-w64 (MSYS2:
`pacman -S mingw-w64-ucrt-x86_64-gcc`, then add its `bin` to PATH), or run the
same tests on the board with `pio test -e uno_tests`.

### Upload fails / port busy
Close the serial monitor and any `serial_logger.py` process first. Only one
program can hold the port.
