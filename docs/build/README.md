# Build guides

[../phase-plan.md](../phase-plan.md) says *what* each phase must achieve and what
number it exits on. These guides say *how*: what to buy, what tools you need,
what order to assemble things in, which decisions are still open, and what goes
wrong.

| Guide | Covers | Prerequisite |
| ----- | ------ | ------------ |
| [phase-1-first-joint.md](phase-1-first-joint.md) | bench bring-up, then the first printed cycloidal joint | Phase 0 complete |
| [phase-2-closed-loop.md](phase-2-closed-loop.md) | PID + feedforward, fault paths, e-stop, commissioning | Phase 1 exit table filled |
| [phase-3-multi-joint.md](phase-3-multi-joint.md) | second joint, coordination, the Uno → Teensy migration | Phase 2 exit checklist |

Phases 4-6 (kinematics, trajectories, integration) are mostly *software* against
an arm that already exists, and are covered by
[../phase-plan.md](../phase-plan.md), [../simulation-plan.md](../simulation-plan.md)
and [../test-plan.md](../test-plan.md). They will get build guides when there is
hardware to build against.

---

## Master tool list

Ranked by when you need it. "Blocking" means the phase cannot be completed
without it.

### Blocking, Phase 1

| Tool | Rough cost | Notes |
| ---- | ---------- | ----- |
| Digital multimeter | have | pointed and alligator-clip probes; DC volts + continuity is the whole requirement |
| Digital calipers, 0.01 mm | have | |
| Feeler gauge set | $10 | magnet gap, repeatably |
| Ceramic/plastic trimmer tool | $5 | a steel screwdriver on a Vref pot shorts it to the pad |
| M3 hex drivers / 2.5 mm keys | verify | M3 fastener kit is on hand; confirm the matching drivers/keys are included |
| Wire strippers | have | |
| Soldering iron + solder + flux | have | also drives heat-set inserts |
| 3D printer | have (P1S) | |

### Blocking, Phase 1B (the reducer)

| Tool | Rough cost | Notes |
| ---- | ---------- | ----- |
| Heat-set insert tip for the iron | $10 | |
| Arbor press, or a vise with soft jaws | $40 / $25 | **never hammer a bearing** |
| Dial indicator 0.01 mm + magnetic base | $35 | the independent backlash measurement |
| Kitchen scale, 1 g | $15 | weighing links is what validates `tools/arm_model.py` |
| Known masses (200 g / 500 g / 1 kg) | $20 | stiffness and payload tests |
| Deburring tool + needle files | $15 | |

### Blocking, Phase 2

| Tool | Rough cost | Notes |
| ---- | ---------- | ----- |
| Latching e-stop + relay | $20 | must break VMOT, not logic |
| Inline fuse + holder | $5 | |
| IR thermometer | $20 | motor/driver temperature under holding load |

### Blocking, Phase 3

| Tool | Rough cost | Notes |
| ---- | ---------- | ----- |
| **8-channel logic analyser** + PulseView | $15 | highest value-per-dollar tool in the project. Loop period, step jitter, I2C decode |
| JST-GH or Micro-Fit crimp tool + connectors | $40 | Dupont on a moving arm fails intermittently and you will blame firmware |
| Rigid base plate + clamps | $25 | an unclamped arm walks, and it shows up in your repeatability number |

### Worth having, not blocking

| Tool | Rough cost | What it unlocks |
| ---- | ---------- | --------------- |
| Benchtop oscilloscope | $150-400 | analogue signal integrity; the logic analyser covers most digital needs |
| Small torque driver, 0.5-3 N·m | $50 | repeatable bearing preload, which is repeatable efficiency |
| Digital angle gauge | $25 | checking encoders against physical reality |
| Fish/luggage scale | $10 | crude output-torque measurement on a lever arm |
| Ultrasonic cleaner | $60 | printed parts after run-in |
| Second bench PSU | $80 | separating logic and motor supplies during fault-hunting |

---

## Open decision register

Every decision that is still open, in one place. Closing one means appending an
entry to [../design-decisions.md](../design-decisions.md) with the reasoning -
not just editing a constant.

| ID | Decision | Phase | Blocks |
| -- | -------- | ----- | ------ |
| P1-a | Cycloidal footprint layout and resulting ratio per joint | 1 | reducer parts and joints 2-6 |
| P1-b | Motor body length per joint | 1 | ordering motors |
| P1-c | Link lengths $a_2, a_3$ | 1 | DH table, IK validation, all of `tools/` |
| P1-d | Magnet hub retention method | 1 | trusting the backlash number |
| P1-e | Encoder cabling standard | 1 | joint 2 harness |
| P1-f | Cable routing: service loop vs slip ring | 1 | joint housing geometry |
| P1-g | Phase current / Vref per motor size | 1 | thermal budget |
| P2-a | Deadband width in counts | 2 | repeatability spec |
| P2-b | Enable Ki at all | 2 | - |
| P2-c | Encoder filter alpha | 2 | loop stability margin |
| P2-d | Hold vs disable when idle | 2 | thermal, and the absolute-homing story |
| P2-e | Brake on J2/J3 | 2 | joint housing geometry |
| P2-f | Control loop rate | 2 | encoder bus budget |
| P2-g | Re-tune trajectory limits to the measured plant | 2 | Phase 5 |
| P3-a | When to migrate off the Uno | 3 | everything past 3 joints |
| P3-b | Teensy 4.1 vs one MCU per joint | 3 | harness, protocol transport |
| P3-c | Printer control board vs hand-wired carrier | 3 | wiring effort |
| P3-d | SPI encoder swap timing | 3 | accuracy floor |
| P3-e | Harness: bundle vs slip ring (confirms P1-f) | 3 | joint travel range |
| P3-f | Streaming setpoints vs segment handoff | 3 | host protocol |
| P3-g | Base mounting: desk surface vs table edge | 3 | usable workspace |
| P3-h | Endstops as an independent safety layer | 3 | unattended operation |

Decisions that are **closed** and should not be relitigated without new
evidence: arm topology (D1), transmission type (D2), encoder placement (D4),
torque tiering scheme (D5), gripper architecture (D6/D11), control layering (D7),
target envelope (D8), homing method (D9), control law shape (D10), host framing
(D12), microstepping = 8 (D3).

---

## How to use these guides

1. Read the whole phase guide before buying anything for it. Several parts are
   only obvious as necessary once you have read the assembly steps.
2. Work the [../bringup-checklist.md](../bringup-checklist.md) gates in order.
   Each app isolates one failure domain; skipping a gate means the next failure
   has two possible causes instead of one.
3. Write the number down the day you measure it, in `docs/test-results/`. Six
   months of these notes is the difference between "I built a robot arm" and
   "I characterised a robot joint."
4. When a guide's advice turns out to be wrong on your hardware, **edit the
   guide**. A build guide that has been corrected by contact with reality is a
   better artifact than one that has not.
