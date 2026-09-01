# TMC2209 setup

## Microstepping

Set with the MS1/MS2 pins/jumpers on the carrier board, then make
`MICROSTEPS` in [include/joint_config.h](../include/joint_config.h) match.

| MS2 | MS1 | Microsteps |
| --- | --- | ---------- |
| L   | L   | 8          |
| L   | H   | 32         |
| H   | L   | 64         |
| H   | H   | 16         |

The `calibration` app will catch a mismatch: your measured steps/deg will be off
by an exact power-of-two factor.

Higher microstepping buys smoothness and resolution, not torque, and costs step
rate. An Uno can comfortably issue roughly 10-20 k steps/s from `loop()`; at 64x
microstepping that is only ~1 motor rev/s. **8x or 16x is the right choice for
Phase 1.**

## Setting motor current (Vref)

NEMA 17 at 1.5 A/phase rated. Start at ~0.5 A RMS and only raise it if the motor
stalls under real load.

For the common StepStick-style TMC2209 boards:

$$I_{RMS} \approx \frac{V_{ref}}{2.5} \cdot 1.77$$

so `Vref = I_rms * 1.41`. For 0.5 A RMS, set Vref ≈ 0.71 V.

> Board revisions differ in sense-resistor value. **Confirm the formula printed
> on your specific board's product page** before trusting a number.

Procedure:
1. Power VMOT, leave the motor connected but idle (driver enabled, not stepping).
2. Measure DC volts between the trimpot wiper and GND.
3. Turn the pot in small increments. Clockwise usually increases current. On the
  current StepStick-style board, "tightening" the screw empirically increased
  usable current; backing it out lowered torque margin and brought back
  stutter near 1600 steps/s.
4. Run `motor_test` and feel the motor after 2 minutes. Warm is fine, painful is
   not.

Do not adjust the pot with the driver unpowered - you will not see the effect,
and shorting the pot to a neighboring pin kills the board.

## Wiring cautions

- **Never unplug motor leads while VMOT is live.** The inductive kick destroys
  the driver.
- Bulk capacitor (100-470 uF, >= 25 V) across VMOT/GND **at the driver**, correct
  polarity. Without it, hot-plugging power can spike above the driver's rating.
- Common ground between the Uno and the motor supply, or STEP/DIR will float.
- Bench PSU current limit at ~1 A while bringing up a single joint. It is a
  cheap insurance policy against a wiring mistake.
- Heatsink on the driver chip, with airflow if you run continuous motion.

## UART mode (later, optional)

The TMC2209 has a single-wire UART that exposes StallGuard (sensorless homing),
CoolStep, and software current control. That is a Phase 3+ upgrade; step/dir is
the right level of complexity for Phase 1. When you get there, the `TMCStepper`
Arduino library is the standard choice, and `PDN_UART` needs a 1k series
resistor for the half-duplex trick on an Uno.
