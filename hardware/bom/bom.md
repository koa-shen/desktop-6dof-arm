# Bill of materials

Fill in the actuals as parts arrive. Track cost so you can talk about the
cost/performance tradeoff later.

**Tools** - as opposed to parts - are listed per phase, with what is blocking
and what is merely nice to have, in
[docs/build/README.md](../../docs/build/README.md#master-tool-list).

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

Every joint is the same module: 42 mm NEMA 17 face + printed cycloidal reducer +
output-side encoder. See [design-decisions.md](../../docs/design-decisions.md)
D2 and D5.

| Qty | Item | Notes |
| --- | ---- | ----- |
| 1 | NEMA 17 | same 42 mm face everywhere; **body length varies by joint** |
| 1 | TMC2209 | |
| 1 | AS5600 + diametric magnet | on the reducer **output** shaft |
| 2 | Cycloidal discs (printed) | 180 deg apart for balance |
| design-dependent | Steel ring pins | count and diameter follow the 42 x 42 mm footprint layout; printed pins wear out fast |
| design-dependent | Needle roller sleeves (optional) | selected to suit the ring pins; large efficiency gain over bare pins |
| 1 | Eccentric bearing | selected from the 42 x 42 mm layout; start near 0.65 mm eccentricity |
| design-dependent | M3 shoulder bolts, 4 mm shoulder | reference starting point for output shafts |
| design-dependent | Output roller and main bearings | selected from the 42 x 42 mm footprint and joint moment requirement |

Cycloidal ratio is set by the ring-pin and lobe counts. Start the first layout
at 15:1-class, then accept only the ratio that fits the 42 x 42 mm NEMA 17 face
footprint and meets the measured torque budget (D19). Do not order bearings or
pins until the dimensioned layout and single-disc coupon pass.

### Motor length by joint (provisional)

Same bolt pattern, same shaft, same mount - only the rotor stack changes. Final
assignment comes from the torque budget, not from this table.

| Joint | Body | Holding torque | Rationale |
| ----- | ---- | -------------- | --------- |
| J2 shoulder | 60 mm | ~0.68 N.m | binding constraint; mass here has ~0 moment arm |
| J1, J3, J4 | 40 mm | ~0.42 N.m | the motor already on hand |
| J5, J6 | 23 mm | ~0.13 N.m | max moment arm; also lowest rotor inertia |

### End effector

| Qty | Item | Notes |
| --- | ---- | ----- |
| 1 | 9 g metal-gear digital servo | e.g. PTK 7465W MG; PWM, not a stepper axis |
| 1 | Printed rack and pinion | pinion radius sets jaw travel; force is never the limit |

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

The cycloidal reduction moves this up the list, but **for latency, not for
accuracy**. At 20:1 and 8x microstepping one step is 0.011 deg at the output, so
the 12-bit AS5600 is 8x coarser than a single step - which sounds like an
accuracy floor and is not one. `tools/error_budget.py` puts encoder
quantization at 0.37 mm of tip error against backlash's 4.25 mm and reducer
windup's 6.25 mm (D17). Going 14-bit shrinks the smallest term in the budget.

Buy these for the **~30 us vs ~4200 us read time across six joints** and for
retiring the mux as a single point of failure. Do not buy them expecting
sub-millimetre accuracy - that is a gearbox problem. `AS5600Encoder` hides the
transport behind a small interface specifically so this swap barely touches the
apps.

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
