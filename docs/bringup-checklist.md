# Bring-up checklist (day hardware arrives)

Work top to bottom. **Do not skip a step because the previous one "probably
works."** Each app isolates one failure domain, which is the whole reason they
are separate. Write the result of each gate in `docs/test-results/`.

This file is the electrical gate sequence.
[build/phase-1-first-joint.md](build/phase-1-first-joint.md) is the physical
build that goes with it - parts, tools, mechanical assembly, and the decisions
that have to close during Phase 1.

Find your COM port first:

```powershell
python scripts/serial_logger.py --list
```

---

## Gate 0 - Bench safety setup (no power yet)

- [ ] Bench PSU set to **12 V, current limit 1.0 A** before anything is connected
- [ ] 100-470 uF electrolytic across VMOT/GND, **at the driver**, correct polarity
- [ ] Uno GND and PSU GND tied together (single common ground)
- [ ] Motor mechanically restrained or on a fixture; nothing can whip a wire
- [ ] Encoder wiring routed away from motor phase wires

> Never disconnect motor leads while VMOT is live. That is the #1 way TMC2209s
> die.

---

## Gate 1 - I2C bus alive (`i2c_scan`)

USB power only. Motor power **off**.

```powershell
pio run -e i2c_scan -t upload
pio device monitor
```

Pass when:
- [ ] `0x70 <- TCA9548A` appears on the main bus
- [ ] `0x36 <- AS5600` appears on the channel you wired (default ch0)
- [ ] `magnet=OK`, `agc` roughly 40-200 (128 is ideal)

If not, see [troubleshooting.md](troubleshooting.md#i2c).

---

## Gate 2 - Encoder quality (`encoder_test`)

Still no motor power. Rotate the magnet **by hand**.

```powershell
pio run -e encoder_test -t upload
pio device monitor
```

Pass when:
- [ ] Angle sweeps smoothly through a full turn, no jumps or flat spots
- [ ] `magnet_ok` stays `1` for the entire rotation
- [ ] One full physical turn = one full 0-360 sweep (no double/half counting)
- [ ] `i2c_errors` stays 0 over a minute

Record: magnet gap used, AGC value, any dead zones. Magnet gap should be
0.5-3 mm and the magnet must be **centered on the shaft axis** - off-center is
the most common cause of a non-linear angle.

---

## Gate 3 - Motor and driver current (`motor_test`)

Motor **disconnected from any linkage**. Now bring up motor power.

```powershell
pio run -e motor_test -t upload
pio device monitor
```

1. Set TMC2209 Vref per [tmc2209-setup.md](tmc2209-setup.md) **with the driver
   powered but the motor idle**. Start at ~0.5 A RMS.
2. Type `e` to enable, then `400` to jog one full motor revolution (at 8x
   microstepping that is 1600 steps - jog `1600`).
3. Ramp speed with `+` until it stalls, then back off 30 %.

Pass when:
- [ ] Motor rotates the commanded amount, both directions
- [ ] No skipped steps or grinding at your working speed
- [ ] Driver is warm, not untouchable; motor below ~60 C
- [ ] Holding torque present when stopped and enabled

Record: Vref, measured current, max reliable speed (steps/s and RPM).

---

## Gate 4 - Calibration (`calibration`)

Joint unloaded and free to swing +/- 20 deg.

```powershell
pio run -e calibration -t upload
pio device monitor
# type: g
```

- [ ] Paste the printed `ENCODER_DIRECTION` and `GEAR_RATIO` into
      [include/joint_config.h](../include/joint_config.h)
- [ ] Record measured steps/deg error vs configured (should be < 1 % once the
      microstep jumpers are right)
- [ ] Record backlash in degrees - **this is your open-loop accuracy floor**

---

## Gate 5 - Phase 1 deliverable (`phase1_bench`)

```powershell
pio run -e phase1_bench -t upload
python scripts/serial_logger.py --port COM3 --name phase1 --seconds 120
# in the monitor window that the logger replaces, type: s   (start)
python scripts/analyze_log.py docs/test-results/phase1_<stamp>.csv
```

Pass when:
- [ ] 20+ out-and-back cycles with no `ERR` lines and `i2c_err` = 0
- [ ] Return-to-home standard deviation reported by `analyze_log.py`
- [ ] Encoder trace tracks commanded trace with a stable, bounded error
- [ ] Plot saved next to the CSV

**Phase 1 is done when you can state your joint's repeatability in degrees and
show the plot that proves it.**

---

## Gate 6 - Closed loop (`closed_loop`)

```powershell
pio run -e closed_loop -t upload
pio device monitor
# h   (home here)
# e   (enable)
# g 30
```

Tune with `k <kp> <ki> <kd>` live:
1. Ki = Kd = 0. Raise Kp until it just starts to overshoot, then back off ~30 %.
2. Add Kd to damp the overshoot.
3. Add a small Ki only if there is a persistent steady-state error.

- [ ] Capture a step response log and run `analyze_log.py`
- [ ] Record rise time, overshoot %, settling time, steady-state error
- [ ] Compare closed-loop steady-state error against the open-loop backlash from
      Gate 4 - explaining that difference is the interview-worthy insight

---

## What to write down every session

Copy `docs/test-results/phase1-log-template.csv` conventions and keep a short
note per run: date, app, config values, what changed, what the numbers were,
what you concluded. Six months of these notes is the difference between "I built
a robot arm" and "I characterized a robot joint."
