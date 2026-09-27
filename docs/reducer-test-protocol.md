# Backlash Bench SOP: 15:1 Cycloidal Reducer

## Purpose and measurement limit

This run measures the motor-shaft angle between a contact switch closing while
approaching the load and opening while reversing away. Dividing that angle by
the assumed 15:1 ratio gives output-side dead band in degrees and arcminutes.

With no output encoder, this is not a pure mechanical backlash measurement. It
includes the switch's release hysteresis and mechanical compliance. Use the
same switch, fixture, preload, direction, and procedure to compare reducer
revisions. A later output encoder can separate these effects.

The AS5600 is on the motor shaft. It detects missed steps and supplies the
angular measurement; it does not observe output transmission error directly.

## Bench record

Fill this in before pressing START:

| Field | Value |
|---|---|
| Date / UTC | |
| Operator | |
| Unit ID / revision | |
| Motor | NEMA 17, serial/notes: |
| Gear ratio used | 15:1 |
| VREF | 1.25 V target; actual: |
| Supply voltage | |
| Microstep setting | |
| Switch model | Omron V-156-1C25 or: |
| Switch mounting direction | |
| Output arm / fixture | |
| Nominal preload | 1-2 N, actual/notes: |
| Ambient / temperature | |
| Firmware commit/build | |

## Equipment

- Uno and TMC2209 assembly with AS5600 motor-shaft encoder
- Reducer and output arm fixed to a rigid bench fixture
- Momentary SPDT switch positioned so NO is closed while touching the load
- Load cell or force gauge for setting repeatable preload; HX711 is optional
  for this switch-only firmware
- Bench PSU, multimeter, USB cable, computer, and terminal capture script
- 100-470 uF electrolytic capacitor across TMC2209 VMOT/GND
- Mechanical stops or a hand on the emergency power switch

## Wiring and safety

1. Power off before changing any wire. Never hot-plug motor leads.
2. Confirm D2 STEP, D3 DIR, D4 EN, A4 SDA, A5 SCL, and D7 switch input.
3. Wire switch COM to GND and NO to D7. Leave NC open. `INPUT_PULLUP` means
   LOW = touching and HIGH = separated.
4. Confirm common ground between Uno and TMC2209. Keep I2C wiring away from
   motor power wiring.
5. Confirm the output arm cannot enter a hard stop during the commanded travel.
6. Start with the documented conservative VREF. Disable power immediately if
   the mechanism binds, chatters, heats unexpectedly, or approaches a stop.

## Firmware configuration

The installed firmware is `src/main.cpp` and runs at 800 pulses/second. It
uses 2-step increments, 15 ms settling, four-sample switch debounce, 60 steps
of contact overtravel, three same-direction control trips, and six dead-band
cycles. Do not change these values between comparison runs without recording
 the change.

Before flashing, verify:

- `GEAR_RATIO` matches the reducer under test.
- `SEEK_STEP_RATE` remains below the validated speed margin.
- `OVERTRAVEL_STEPS` is enough to seat the contact but does not overload it.
- `BACKOFF_STEPS` fully releases the switch and leaves clearance.
- The physical TMC2209 microstep setting does not affect the encoder-derived
  result, but record it for repeatability and future commanded-step work.

Build and upload from the project directory:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run --target upload --upload-port COM3
```

Replace `COM3` with the actual port. A successful build is required before
uploading. Open the serial monitor at 115200 baud and verify:

```text
READY,backlash_switch_deadband
```

## Fixture setup

1. Mount the reducer so the output arm presses the load cell or force gauge.
2. Adjust the switch so its lever is depressed at the same contact condition,
   with enough margin that it cannot be crushed during overtravel.
3. Apply the chosen preload. Record the value and do not change it during a
   run.
4. Rotate or position the reducer at the first test angle. Mark this position.
5. Confirm the switch state by hand: touching must read closed/LOW; separation
   must read open/HIGH.
6. Remove loose tools from the travel path and keep one hand near power-off.

## Data collection

From the project directory, run:

```powershell
.\tools\capture-backlash.ps1 -PortName COM3 -UnitId reducer-01 -VrefVolts 1.25
```

The script waits for READY, sends START, captures the serial log, and writes:

- `docs/test-results/backlash-<timestamp>.csv` for the six dead-band rows
- `docs/test-results/backlash-<timestamp>-serial.log` for the complete trace

The firmware sequence is:

1. Release the switch if needed, then back off 600 steps.
2. Take three same-direction control trips. Their spread is the measurement
   noise/fixture repeatability check.
3. For each of six cycles, approach until the switch closes, overtravel 60
   steps, reverse until the switch opens, and back off.
4. Report `delta_motor_deg` after subtracting the measured overtravel, then
   divide by 15 to obtain `deadband_output_deg` and arcminutes.

Stop the run if the arm approaches a hard stop, the switch does not change,
the encoder reports a fault, the motor stalls, or the load becomes unsafe.
The firmware disables the driver on completion or abort.

## Run acceptance and fill-in

After capture, enter the script summary here:

| Result | Value |
|---|---|
| Control trip spread at output | ___ arcmin |
| Dead-band mean | ___ arcmin |
| Dead-band minimum / maximum | ___ / ___ arcmin |
| Cycle count | 6 |
| Encoder faults / stalls | |
| Switch bounce observed | |
| Fixture movement | |
| Audible or tactile anomalies | |

Treat the run as invalid and repeat it if the control spread is a large
fraction of the dead-band mean, any cycle aborts, the switch is intermittent,
or the fixture moves. The capture script warns when control spread is at least
half the measured mean; this is a screening rule, not a mechanical spec.

Repeat the valid run at three output angles and average only after checking
that the individual runs are stable. Keep every CSV and serial log.

## Follow-on tests

After backlash data is stable, perform breakaway holding torque,
backdrivability, and loaded holding tests. Do not replicate the reducer across
the other joints until its measured dead band, torque, and thermal behavior
fit the joint's requirements.
