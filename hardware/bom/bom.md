# Bill of materials

Fill in the actuals as parts arrive. Track cost so you can talk about the
cost/performance tradeoff later.

## Phase 1 (single joint bench)

| Qty | Item | Notes | Have? | Cost |
| --- | ---- | ----- | ----- | ---- |
| 1 | Arduino Uno | Elegoo starter kit | yes | - |
| 1 | NEMA 17 stepper, 1.5 A, 42 N·cm | | | |
| 1 | TMC2209 driver module | step/dir mode, heatsink included | | |
| 1 | TCA9548A I2C mux breakout | default addr 0x70 | | |
| 1 | AS5600 encoder breakout | | | |
| 1 | **Diametric** magnet, 6x2.5 mm | must be diametric, not axial | | |
| 1 | 100-470 uF electrolytic, >= 25 V | across VMOT/GND at driver | | |
| 1 | Bench PSU 30 V 10 A | have | yes | - |
| - | Dupont jumpers, breadboard | starter kit | yes | - |
| - | M3 fastener kit | have | yes | - |
| - | PETG filament | encoder + motor mounts | | |

## Phase 3+ (per additional joint)

Every joint is the same module: NEMA 17 + printed cycloidal reducer + output-side
encoder. See [design-decisions.md](../../docs/design-decisions.md) D2.

| Qty | Item | Notes |
| --- | ---- | ----- |
| 1 | NEMA 17 | same motor at every joint; the reducer does the work |
| 1 | TMC2209 | |
| 1 | AS5600 + diametric magnet | on the reducer **output** shaft |
| 2 | Cycloidal discs (printed) | 180 deg apart for balance |
| ~21 | Steel dowel pins, ring gear | printed pins wear out fast |
| ~21 | Needle roller sleeves (optional) | large efficiency gain over bare pins |
| 1 | Eccentric bearing (6902/6802) | input eccentric |
| 4-8 | Output roller bearings | output pin followers |
| 1-2 | Main output bearing | thin-section or 6807/6810 |

Cycloidal ratio = number of lobes on the disc, with one more pin than lobes
(20 lobes + 21 pins -> 20:1). Design two ratios at most, per the torque budget.

## Tiered upgrade path

Buy against a **trigger**, not a wishlist. Buying ahead of the trigger is how
projects stall. Prices are rough and need checking at purchase time.

### Trigger summary

| Buy this | When this happens |
| -------- | ----------------- |
| Teensy 4.1 | 2nd or 3rd joint, or you want real-time kinematics |
| Printer control board | 3rd driver - cheaper than loose drivers plus wiring |
| SPI encoders | 3rd encoder, **or** the first cycloidal reducer goes in |
| Endstops + e-stop | **Before** the arm can hurt something |
| CAN / integrated actuators | More than ~8 wires crossing one joint |
| Host computer | You want ROS 2, planning, or a camera |
| Power supply | **Never - the bench PSU is already 300 W.** Run it at 24 V. |

### Where the current stack actually runs out

| Subsystem | Ceiling | The number |
| --------- | ------- | ---------- |
| Uno step generation | ~2 joints | `digitalWrite` ~4 us on AVR; step pulse + float profile ~30-40 us per joint per iteration -> **~5 kHz aggregate**. At 88.9 steps/output-deg that is 56 deg/s for *one* joint. |
| I2C encoder bus | ~3 joints | Mux select + 2-byte AS5600 read = ~50 bit-times = **700 us at 100 kHz**. Six encoders = 4.2 ms -> ~240 Hz aggregate, blocking stepping the whole time. |
| Uno pins | 6 joints, nothing else | D2-D13 is exactly 12 pins for 6x step/dir. Nothing left for endstops, gripper, e-stop. |
| Uno RAM | ~6 joints | ~145 bytes/joint. Not the binding constraint, which surprises people. |
| No FPU | 1 joint | Software float 5-10 us/op. 6-DOF FK is ~250 flops + 48 trig calls. Real-time IK is off the table. |

### Tier 1 - joints 2-3 (~$100-150)

| Item | ~Cost | Why |
| ---- | ----- | --- |
| **Teensy 4.1** | $32 | 600 MHz M7 **with FPU**, 1 MB RAM, 55 I/O, hardware timers for stepping, built-in CAN. Every `lib/` class is header-only Arduino API, so the port cost is near zero. |
| Printer control board (SKR Pro / Octopus) | $50-70 | 8 driver sockets, TMC UART, endstop headers, 24 V distribution, fusing. Solves five problems at once. |
| Endstops x6 | $10 | Homing plus a hardware safety layer independent of firmware. |
| Latching e-stop | $10 | Cuts VMOT, not logic. |
| Level shifters | $5 | Teensy is 3.3 V. TMC2209 STEP/DIR/EN are fine; check your AS5600 breakout. |
| Spare TMC2209 | $8 | They die from hot-plugged motor leads, always at the worst time. |

### Tier 2 - joints 4-6 and the encoder bus (~$100-200)

| Item | ~Cost | Why |
| ---- | ----- | --- |
| **AS5047P / MA732 SPI encoders** | $8-12 ea | 6 encoders in ~30 us vs ~4200 us. 14-bit = 0.022 deg vs 0.088 deg. |

The cycloidal reduction moves this up the list. At 20:1 and 8x microstepping one
step is 0.011 deg at the output, so the 12-bit AS5600 is already **8x coarser
than a single step** - it is the accuracy floor the day the first reducer goes
in. `AS5600Encoder` hides the transport behind a small interface specifically so
this swap barely touches the apps.

### Tier 3 - distributed control

Trigger: more than ~8 wires crossing a single joint. With motors *in* the joints
(D1), this arrives sooner than a motors-at-the-base design.

| Option | ~Cost | Tradeoff |
| ------ | ----- | -------- |
| CAN transceivers + own joint boards | $2 ea + effort | Max learning, max work |
| **MKS SERVO42C / 57D** | $25-40 ea | Closed-loop stepper with encoder in the motor cap. Deletes encoder wiring *and* skipped steps in one part. Best cost/benefit here. |
| ODrive / moteus (BLDC + FOC + CAN) | $150-400 ea | Buys the problem away. 6 joints = $1200+, which is a real "learning project or product?" decision. |

### Tier 4 - host compute (~$80-150)

Raspberry Pi 5 or a mini PC for ROS 2, kinematics, planning, and perception.
Hard real-time motion control stays on the MCU - do not run a 1 kHz loop on
stock Linux. Being able to explain *why* that split exists is itself interview
material.

### Infrastructure people forget (~30 % of real cost)

- Bearings: 6807/6810 deep groove ($5-10) or crossed-roller for joint moment
  stiffness ($30-80). Printed bushings will dominate your backlash number.
- Connectors: JST-GH or Molex Micro-Fit; **shielded twisted pair for encoders**,
  shield grounded at one end only.
- Drag chain and silicone-jacketed stranded wire. Solid core fails on flex.
- Slip ring (~$20) for any continuously rotating joint - UR does this on wrist 3.
- A rigid base plate so the arm does not walk when it decelerates.

## Notes on part selection

- **Magnet type is the #1 ordering mistake.** AS5600 needs a *diametrically*
  magnetized magnet. Axial magnets are far more common and will not work.
- TMC2209 boards vary in sense resistor value, which changes the Vref/current
  formula. Note your exact board revision here when it arrives.
- Buy one spare TMC2209. They die from hot-plugged motor leads, and they die at
  the worst time.
