# Phase 1 - build guide: the first joint

**Read [../bringup-checklist.md](../bringup-checklist.md) alongside this.** That
file is the electrical gate sequence. This file is the physical build, the
shopping list, the tool list, and the decisions you have to actually make.

Phase 1 splits into two stages that are frequently and expensively conflated:

| Stage | What it is | `GEAR_RATIO` | What it proves |
| ----- | ---------- | ------------ | -------------- |
| **1A** | Motor + encoder on a flat plate, magnet on the motor shaft, no reducer | `1.0` | electronics, firmware, telemetry, toolchain |
| **1B** | The first printed cycloidal joint module, encoder on the **output** | `20.0` | mechanism, backlash, efficiency, stiffness |

Do 1A first even though it feels like a detour. It is the only configuration in
which an electrical fault and a mechanical fault cannot be confused for each
other, and you will be back here every time something later goes wrong.

---

## Objectives

By the end of Phase 1 you can state, with a plot behind each:

1. **Return-to-home repeatability** in degrees, 1σ over ≥ 20 cycles
2. **Backlash** in degrees, measured two independent ways
3. **Max reliable speed** in deg/s at the output with zero skipped steps
4. **Measured gear ratio** vs nominal, as a percentage error
5. **Encoder health** - AGC value, gap in mm, i2c error count over 10 minutes

Number 2 is the one that matters. It is the input to D8a - the decision of
whether machined gearboxes are worth funding - and no other measurement in the
project substitutes for it.

---

## Part 1A - the bench

### Parts

Everything here is in [../../hardware/bom/bom.md](../../hardware/bom/bom.md);
this is the "did it actually arrive" list.

| Qty | Part | Spec that matters | Common mistake |
| --- | ---- | ----------------- | -------------- |
| 1 | Arduino Uno | ATmega328P, 5 V logic | a 3.3 V clone breaks the 5 V AS5600 assumption |
| 1 | NEMA 17 stepper | 42 mm face, 1.5-2.0 A/phase, 4-wire **bipolar** | 6-wire unipolar motors need two wires taped off |
| 1 | TMC2209 StepStick | note the **sense resistor value** (0.11 Ω vs 0.15 Ω) | the Vref formula depends on it |
| 1 | TCA9548A breakout | addr 0x70, A0/A1/A2 unjumpered | |
| 1 | AS5600 breakout | 5 V tolerant, 10 k pull-ups on board | some boards are 3.3 V only |
| 1 | Magnet, 6 × 2.5 mm | **diametrically** magnetised | axial magnets are 10× more common and useless here |
| 1 | Electrolytic, 100-470 µF, ≥ 35 V | low ESR, at the driver | 25 V rating is marginal at 24 V |
| 1 | Bench PSU | 24 V, current limit adjustable | TMC2209 absolute max is ~29 V |
| - | Silicone-jacketed stranded wire, 22 AWG | motor phases | solid core work-hardens and snaps |
| - | Dupont jumpers | logic only | never for motor phases |

**Buy one spare TMC2209 now.** They die when a motor lead is unplugged with
VMOT live, and that happens at the worst time.

### Tools

Bold = you cannot do Phase 1 without it.

| Tool | Used for | Note |
| ---- | -------- | ---- |
| **Digital multimeter** | Vref, continuity, coil pairing, supply rails | any $20 meter is fine; you need DC volts and continuity beep |
| **Small flat screwdriver** | TMC2209 Vref trimpot | ceramic/plastic trimmer tool is better - a steel driver shorts the pot to the pad |
| **Digital calipers, 0.01 mm** | magnet gap, shaft diameters, printed part QC | |
| **Feeler gauge set** | setting the 0.5-3 mm magnet gap repeatably | a 1 mm stack of gauges is the reference |
| **2.5 mm hex key / M3 driver** | everything | |
| Wire strippers + ferrule crimper | motor phase terminations | ferrules stop strands from bridging |
| Soldering iron, solder, flux | breakout headers, encoder pigtails | |
| Heat-shrink assortment | strain relief | |
| Kapton or fibreglass tape | holding thermistors/wires clear of the motor | |
| Zip ties + adhesive mounts | keeping encoder wire away from phase wire | this is a signal-integrity fix, not tidiness |
| **3D printer (PETG)** | mounts | you have a P1S |
| IPA + lint-free cloth | cleaning bed and magnet faces | |

### Assembly, in order

Power is **off** for steps 1-6.

1. **Identify the motor coils.** Multimeter on continuity. The two wires that
   beep together are one coil. Write the colour pairs down - you will need them
   again on every joint. Pair 1 → `A1/A2`, pair 2 → `B1/B2`. Getting a wire from
   each coil into the same pair is the classic "buzzes but does not turn".
2. **Print the bench plate.** A flat PETG plate with the NEMA 17 bolt pattern
   (31 mm square, 4 × M3, 22 mm centre bore) plus a boss that holds the AS5600
   breakout coaxial with the motor shaft. Print it **flat on the bed**. Slot the
   encoder boss holes so the gap is adjustable after assembly - you will change
   it three times.
3. **Bond the magnet to a printed hub**, and press the hub onto the motor shaft.
   Cyanoacrylate is fine. Two rules that are not negotiable:
   - the magnet's rotation axis must be the **shaft** axis. Eccentricity of
     0.5 mm produces a visibly non-linear angle that no software fixes.
   - no steel within ~10 mm. That includes the M3 screws holding the encoder
     board, so use nylon or brass hardware there if you have it.
4. **Set the magnet gap to ~1.5 mm** with feeler gauges and lock the boss.
   Target AGC ≈ 128; 40-200 is acceptable.
5. **Wire logic only.** Uno D2/D3/D4 → TMC2209 STEP/DIR/EN, A4/A5 → mux SDA/SCL,
   5 V and GND to both. Mux `SD0/SC0` → AS5600 SDA/SCL, and the AS5600's own
   VCC/GND to the Uno's 5 V rail (the mux channel headers carry data only).
6. **Wire motor power, capacitor first.** Electrolytic across VMOT/GND *at the
   driver*, polarity checked twice. Then PSU + to VMOT, PSU − to driver GND, and
   a wire tying driver GND to Uno GND. **A single common ground is mandatory** -
   without it the step signal has no reference and the driver does arbitrary
   things.
7. **Gates 1 and 2** from the bring-up checklist: `i2c_scan`, then
   `encoder_test`, both on USB power with motor power off. Turn the shaft by
   hand through several full turns and watch for flat spots.
8. **Set Vref before enabling anything.** See
   [../tmc2209-setup.md](../tmc2209-setup.md). Start at 0.5 A RMS. Measure Vref
   with the driver powered and the motor **idle**, probing the trimpot wiper -
   not the motor leads.
9. **Gate 3** - `motor_test`. Then Gates 4 and 5.

### Safety rules that are not optional

- Never unplug a motor lead with VMOT live. This is the single most common way
  to kill a TMC2209 - the collapsing coil field has nowhere to go.
- PSU current limit at 1.0 A during bring-up. A wiring error then browns out
  instead of welding.
- The motor gets hot. 60 °C is normal, 80 °C means lower Vref.
- Nothing spins near a face. A 6 mm magnet leaving a hub at 300 rpm is a
  projectile.

---

## Part 1B - the first joint module

This is where the project stops being an electronics exercise. The cycloidal
reducer is the highest-risk item in the whole build (D2), and it is normal for
the first one to be scrap.

### Parts

| Qty | Part | Spec | Why |
| --- | ---- | ---- | --- |
| 2 | Cycloidal disc, printed | 20 lobes, 180° apart on the eccentric | one disc is inherently unbalanced |
| 21 | Ring pin, **steel dowel** ⌀3 × 16 mm | h7 ground dowel, not cut rod | printed pins wear out in hours |
| 21 | Needle roller sleeve (optional) | to suit the dowel | large efficiency gain; adds cost and stack height |
| 1 | Eccentric input, printed or turned | on a 6902 or 6802 bearing | |
| 1 | 6902ZZ or 6802ZZ bearing | eccentric ride | |
| 4-8 | Output pin bearings, MR63/MR83 | output roller followers | |
| 1-2 | Main output bearing, 6807 / 6810 / thin-section | carries the joint moment | **this bearing sets your joint stiffness** |
| 1 | Output flange, printed | carries the AS5600 magnet hub | |
| - | M3 SHCS assortment, 6-25 mm | | |
| - | M3 heat-set inserts | printed threads strip | |
| - | PTFE or lithium grease | run-in and efficiency | |

### Additional tools

| Tool | Used for |
| ---- | -------- |
| **Soldering iron + heat-set insert tip** | M3 inserts in PETG |
| **Arbor press or a good vise with soft jaws** | bearing and dowel pin seating - never hammer a bearing |
| **Dial indicator, 0.01 mm, + magnetic base** | the independent backlash measurement |
| **Digital angle gauge or printed 360° protractor disc** | verifying the encoder against physical reality |
| Deburring tool, needle files, 400-800 grit | cleaning printed bores and pin seats |
| Small torque driver (0.5-3 N·m) *nice to have* | consistent preload on the output bearing |
| Kitchen scale, 1 g resolution | **weighing links and the assembly** - this feeds `tools/arm_model.py` |
| Known masses (200 g, 500 g, 1 kg) | stiffness measurement and payload testing |
| Fish scale or luggage scale | crude torque measurement on a lever arm |

### Print settings that decide whether this works

The cycloidal profile is the most tolerance-sensitive part in the project. FDM
error of 0.1-0.2 mm lands directly on the output as backlash (D2).

| Setting | Value | Why |
| ------- | ----- | --- |
| Material | PETG (discs, housing) | stiffer and less creep-prone than PLA at load; ABS/ASA if you have an enclosure |
| Layer height | 0.15 mm | profile fidelity on the lobe flanks |
| Walls | 4-5 | the disc is loaded in bending at the lobes |
| Infill | 40-60 % gyroid | |
| Horizontal expansion / XY compensation | **calibrate it** | see below |
| Orientation | discs flat, housing flat | layer planes perpendicular to the load |
| Seam | random or aligned **away** from the lobes | a visible seam on a flank is a hard point |

**Calibrate XY compensation before printing a disc.** Print a coupon with five
20 mm bores at −0.1, −0.05, 0, +0.05, +0.1 mm offsets, measure with calipers,
and derive the single number your printer needs. Doing this once saves three
full disc prints. Do the same for the pin bores: print a strip of 21 pin
pockets in 0.05 mm increments and find the one where the dowel slides in with
finger pressure and no rock.

### Assembly, in order

1. **Install heat-set inserts** in every printed part before assembly. Retro-
   fitting one means disassembling the joint.
2. **Deburr every bore.** The first layer's elephant foot on a pin pocket will
   cock the dowel by a degree, and 21 cocked dowels is a gearbox that binds.
3. **Seat the ring pins** in the housing with the arbor press. All 21 must sit
   at the same height and stand perpendicular. Check with a straightedge across
   the top; any pin standing proud will foul the second disc.
4. **Press the 6902 onto the eccentric**, then the disc onto the bearing. The
   disc should rotate on the bearing with light drag, no rock.
5. **Assemble both discs 180° apart** on the eccentric. If your eccentric is a
   single lobe with a keyed hub, the two discs need a 180° index feature - if
   your model does not have one, that is a design fix, not an assembly problem.
6. **Fit the output roller followers**, then close the housing and fit the main
   output bearing. Preload it lightly - just enough to remove axial play.
   Overpreload converts directly into lost efficiency.
7. **Turn it by hand before it ever sees a motor.** It should turn with steady,
   moderate resistance, all the way around, with no tight spot. A tight spot
   once per output revolution is a housing/bore concentricity problem; a tight
   spot 20 times per output revolution is a lobe profile or pin spacing problem.
   **Do not power a gearbox that binds by hand.**
8. **Grease lightly and run it in.** Motor at low speed, unloaded, both
   directions, 10-15 minutes. Expect the resistance to drop noticeably. Wipe out
   the plastic dust afterwards - it is abrasive.
9. **Move the magnet to the output flange.** This is D4 and it is the whole
   point: the encoder must read the joint, not the motor.
10. **Set `GEAR_RATIO = 20.0`** in
    [../../include/joint_config.h](../../include/joint_config.h), then
    **re-derive the trajectory ceilings** - `MAX_SPEED_STEPS_PER_SEC /
    STEPS_PER_OUTPUT_DEG` and `ACCEL_STEPS_PER_SEC2 / STEPS_PER_OUTPUT_DEG`.
    They drop by 20×. Leaving `TRAJ_MAX_VEL_DEG_S` where it was is exactly the
    D10 failure: a saturated loop that reads as bad tuning.
11. Re-run Gates 3, 4, and 5 with the reducer in place. Every number changes.

### Measuring backlash two ways, and why

The `calibration` app reports backlash from the encoder. That number includes
gearbox lost motion **and** encoder quantisation **and** any magnet hub slip.
The dial indicator separates them:

1. Clamp a lever arm of known length $L$ (100 mm is convenient) to the output.
2. Dial indicator against the lever tip, motor **enabled and holding**.
3. Push the lever gently one way until it stops moving further, zero the
   indicator. Push the other way, read the deflection $\delta$.
4. Backlash in degrees is $\arctan(\delta / L)$ - for $L = 100$ mm,
   1 mm of indicator travel is 0.57°.

If the dial indicator and the encoder disagree by more than a count or two,
the magnet hub is slipping on the output shaft. That is a failure mode that
looks exactly like gearbox backlash and is not.

Also measure **torsional stiffness** while the fixture is set up - hang a known
mass at a known radius, read the encoder deflection, and compute
$k = \tau / \Delta\theta$ in N·m/rad. `tools/joint_sim.py` currently *assumes*
300 N·m/rad, and D10 explicitly flags that as an estimate rather than a
measurement. Replacing it is a 20-minute job that makes every simulation result
in the repo defensible.

---

## Decisions you have to make in Phase 1

These are open. Nothing downstream can be finished until they are closed, and
each one belongs in [../design-decisions.md](../design-decisions.md) with its
reasoning, not just its answer.

| ID | Decision | When it must be made | What decides it | Cost of deciding late |
| -- | -------- | -------------------- | --------------- | --------------------- |
| **P1-a** | **Cycloidal ratio per joint** (20:1 vs 26:1, and how many distinct ratios to design) | before printing the second joint | `tools/torque_budget.py` re-run with **weighed** link masses | every ratio is a separate print/tune cycle |
| **P1-b** | **Motor body length per joint** (23 / 40 / 60 mm) | before ordering motors 2-6 | same torque budget; D5's table is provisional | wrong motors are ~$15 each and a 2-week lead time |
| **P1-c** | **Link lengths $a_2, a_3$** - currently 168 / 148 mm | before printing links | D8b says the split is a weak lever; the **sum** sets reach and shoulder torque | changing $a_2$ invalidates the DH table, the IK validation, and the workspace study |
| **P1-d** | **Magnet hub retention** - press fit, CA bond, grub screw, or moulded-in | during 1B assembly | the two-way backlash measurement above | slip is indistinguishable from backlash until you check |
| **P1-e** | **Encoder cabling** - Dupont vs crimped connector vs shielded twisted pair | before joint 2 | `i2c_err` count during motion at full current | rework on an assembled joint means disassembly |
| **P1-f** | **Cable routing through the joint** - service loop vs slip ring | before the joint housing is final | joint travel range; D9 assumes ≤ 300° | this changes the housing, i.e. a reprint of everything |
| **P1-g** | **Bench Vref / phase current** | Gate 3 | motor temperature at your working duty cycle | running hot ages the magnets and the printed parts around them |

Two decisions people expect to make here and should **not**:

- **Microstepping.** Decided: 8×. At 20:1 one microstep is 0.011° at the output
  and the AS5600 resolves 0.088°, so more microsteps buy resolution nothing can
  observe while multiplying step rate (D3).
- **Homing method.** Decided: absolute from the output encoder, no switches,
  no homing move (D9). The commissioning step is measuring
  `JOINT_HOME_OFFSET_DEG`, not choosing a strategy.

---

## Troubleshooting - Phase 1 specific

The electrical failures are in [../troubleshooting.md](../troubleshooting.md).
These are the mechanical ones, which that file does not cover.

### The gearbox binds once per output revolution
Concentricity. Either the main output bearing pocket is not concentric with the
ring pin circle, or the output flange is not concentric with the bearing. Check
by removing the discs and turning the output alone - if it still binds, the
discs are innocent.

### The gearbox binds ~20 times per output revolution
Lobe engagement. In order of likelihood: XY compensation not calibrated (discs
oversize), pin pockets misplaced by print shrinkage, or the disc's eccentricity
does not match the eccentric you printed. Print a **single-disc, half-height
test article** to iterate on this rather than a full gearbox.

### It turns but the ratio is not 20:1
Count it by hand: mark the input and output, turn the input 20 full revolutions,
and see where the output lands. A cycloidal stage's ratio is (number of pins −
number of lobes) based; if you printed 20 lobes and 20 pins instead of 21, the
ratio is undefined and the thing will lock.

### Backlash is > 2° on the first assembly
Expected (D2 predicts 0.5-2°). Attack in this order, measuring after each:
1. pin pocket clearance - the biggest single contributor
2. disc-to-pin clearance from XY compensation
3. output roller follower clearance
4. main bearing preload
Only after all four should you consider that the design needs revising.

### Backlash is near zero and it barely turns
You over-corrected the clearance. A cycloidal drive needs a few hundredths of
clearance to run; zero clearance is a press fit. Back off XY compensation by
0.05 mm.

### Efficiency is terrible - the motor stalls at loads it should hold
Measure before theorising. Lever arm + fish scale on the output, known current
setting, find the stall torque, and compare against
`motor_torque × ratio × η`. If $\eta$ comes out below ~50 %, suspect: bare
printed pins (add needle rollers), overpreloaded output bearing, or a disc
rubbing on the housing face.

### The encoder reads fine by hand but goes noisy under power
Coupling from the motor phases. In order: physically separate the encoder cable
from the phase wires, twist the SDA/SCL pair, add 100 nF at the AS5600's VCC,
then shielded cable grounded at the Uno end only. Dropping `I2C_CLOCK_HZ` to
50 000 is a diagnostic, not a fix - if it helps, you have confirmed signal
integrity and should go fix the wiring.

### `magnet=OK` at rest, `TOO_FAR` when the joint moves
The output flange is deflecting axially under load, or the main bearing has
axial play. This is a real mechanical finding disguised as a sensor complaint -
chase the bearing preload.

### Steps skip only in one direction
Gravity. The joint is unbalanced and one direction is fighting the load. Either
the current is too low, the acceleration is too aggressive, or you have found
the reason gravity feedforward exists (Phase 5).

---

## Resources

Grouped by what you are trying to understand. Where a deep link is likely to
rot, the search term is given instead.

### Cycloidal drives and printed joint modules
- **Aaed Musa** (YouTube, `@AaedMusa`) - printed cycloidal actuators and full
  arms, with published CAD. The closest existing work to this project.
- **Skyentific** (YouTube, `@Skyentific`) - years of printed cycloidal, harmonic
  and strain-wave drive experiments, including honest failure videos.
- **Michael Rechtin** (YouTube, `@MichaelRechtin`) - design-for-3D-printing of
  gearboxes; the tolerance and clearance content is directly applicable.
- **James Bruton** (YouTube, `@jamesbruton`) - large printed robots; useful for
  how he instruments and debugs mechanisms.
- Search: *"cycloidal drive theory backlash"*, *"cycloidal reducer pin gear
  ratio derivation"*.

### Steppers, TMC drivers, and current setting
- **Trinamic/Analog Devices TMC2209 datasheet** - search
  *"TMC2209 datasheet"*. The Vref/current table and the sense-resistor
  dependency are the parts you need.
- **Pololu's stepper driver pages** (`pololu.com`) - the clearest plain-English
  explanation of current limiting on StepStick-format drivers.
- Search: *"stepper motor torque speed curve why torque drops"* - understanding
  that curve is what makes your max-speed measurement meaningful rather than
  arbitrary.

### Magnetic encoders
- **ams-OSRAM AS5600 datasheet** - search *"AS5600 datasheet"*. Read the AGC and
  magnet-placement sections specifically; they explain every failure in the
  troubleshooting list above.
- Search: *"diametric vs axial magnetization"* - one diagram will make the
  ordering mistake impossible to repeat.

### 3D printing for mechanisms
- **CNC Kitchen** (YouTube, `@CNCKitchen`) - actual tensile and creep testing of
  printed parts. Watch the layer-orientation strength videos before you decide
  how to lay out a link.
- Search: *"3D printing tolerance calibration XY compensation"*.

### Metrology and measurement method
- Search: *"repeatability vs accuracy ISO 9283"* - ISO 9283 is the standard
  industrial robots are actually specified against. Using its vocabulary
  (pose repeatability, pose accuracy, path accuracy) correctly is a genuine
  signal in an interview.
- Search: *"measuring backlash with dial indicator"*.

### Background theory (start now, it pays off in Phase 4)
- **Modern Robotics**, Lynch & Park - free PDF and free video course,
  `hades.mech.northwestern.edu/index.php/Modern_Robotics`. Chapters 3-5 cover
  everything Phase 4 needs.
- **Feedback Systems**, Åström & Murray - free PDF at `fbsbook.org`.
- **Brian Douglas** (YouTube, `@BrianBDouglas`) - the control intuition videos.
  Watch the PID series before Phase 2, not during it.

---

## Exit checklist

Phase 1 is done when this table is filled in with your numbers and the plots
are committed under `docs/test-results/`.

| Metric | Stage 1A | Stage 1B | How measured |
| ------ | -------- | -------- | ------------ |
| Repeatability, 1σ (deg) | | | `analyze_log.py`, ≥ 20 cycles |
| Backlash (deg), encoder | ~0 expected | | `calibration` app |
| Backlash (deg), dial indicator | n/a | | lever arm + indicator |
| Measured / nominal gear ratio | 1.00 | | `calibration` app |
| Max reliable speed (deg/s, output) | | | ramp until skip, back off 30 % |
| Torsional stiffness (N·m/rad) | n/a | | hang a mass, read deflection |
| Gearbox efficiency (%) | n/a | | stall torque vs motor × ratio |
| AGC / magnet gap | | | `i2c_scan` |
| `i2c_err` per 10 min under power | | | `phase1_bench` |
| Link / assembly mass (g) | | | kitchen scale → `tools/arm_model.py` |

The last row is not bookkeeping. Every torque, workspace, and simulation number
in this repo currently rests on **estimated** masses. Weighing the first joint
is what turns the design tooling from a plausible model into a validated one.
