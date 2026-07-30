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

| Qty | Item | Notes |
| --- | ---- | ----- |
| 1 | NEMA 17 (size per torque budget) | shoulder/elbow may need NEMA 23 or a reducer |
| 1 | TMC2209 | |
| 1 | AS5600 + diametric magnet | one mux channel each |
| 1 | Reduction (GT2 belt or printed planetary) | see torque budget |
| - | Bearings (608 / thin-section) | |

## Likely upgrades

| Item | Why |
| ---- | --- |
| ESP32 or Teensy 4.x | Uno runs out of pins, RAM, and step rate at 2-3 joints |
| 24 V PSU | More torque headroom at speed than 12 V |
| SPI encoders (AS5047P) | Removes the mux bottleneck, faster reads |
| Logic level shifters | If mixing 3.3 V MCU with 5 V peripherals |

## Notes on part selection

- **Magnet type is the #1 ordering mistake.** AS5600 needs a *diametrically*
  magnetized magnet. Axial magnets are far more common and will not work.
- TMC2209 boards vary in sense resistor value, which changes the Vref/current
  formula. Note your exact board revision here when it arrives.
- Buy one spare TMC2209. They die from hot-plugged motor leads, and they die at
  the worst time.
