# Simulation plan

The goal is not "a simulation." It is **a model whose disagreement with the
hardware you can state as a number**, and a set of tests that run without the
hardware present. Those are two different things and both are the actual job in
industry.

This plan is deliberately written to be executable in stages, each of which
produces something showable, so that it degrades gracefully if the hardware
timeline slips.

Per D16, this work has **priority over control sophistication**: stages S0-S4
are done before any control law beyond PID and feedforward is attempted, because
every one of those techniques needs a plant model that S4 is what produces.

**Status:** S0's generator exists - `sim/export_urdf.py` emits `sim/arm.urdf`
from `tools/arm_model.py` and validates the generated chain against
`tools/kinematics.py`'s FK over 500 random configurations, agreeing to 1e-16 m.
What remains for S0 is loading it in Drake and MuJoCo and confirming both agree
with the same reference.

---

## 1. What simulation is actually for here

Four distinct jobs get called "simulation" and conflating them is why hobby
projects end up with a pretty visualiser that proves nothing.

| Job | Question it answers | Fidelity needed | Where it lives now |
| --- | ------------------- | --------------- | ------------------ |
| **Design verification** | is the mechanism sized right? | statics + geometry | `tools/torque_budget.py`, `workspace.py` |
| **Control design** | will this control law hunt, saturate, or ring? | 1-DOF dynamics | `tools/joint_sim.py` |
| **Integration test** | does the whole stack produce the motion I asked for? | multibody + timing | **not started** |
| **Regression test** | did today's change break yesterday's behaviour? | whatever is cheapest and deterministic | **not started** |

The third and fourth are the gap. The first two already exist and already earned
their keep: `joint_sim.py` found that a naive PID reverses direction 83 times a
second at rest, before any hardware existed (D10). That is the template for
everything below - **the value of a simulation is measured in bench sessions it
prevented**, and that story is what an interviewer is actually listening for.

---

## 2. The fidelity ladder

Every level answers questions the level below cannot, and costs more to build,
run, and *validate*. The discipline is to use the lowest level that can answer
the question, and to make each level agree with its neighbours.

| Level | Model | Answers | Cannot answer | Runtime |
| ----- | ----- | ------- | ------------- | ------- |
| **L0** | closed-form statics + geometry | reach, torque, workspace, singularities | anything time-varying | ms |
| **L1** | 1-DOF plant: rate source, compliant reducer, backlash, quantisation | control law stability, deadband, feedforward benefit | multi-axis coupling, collisions | ms |
| **L2** | rigid multibody, whole arm, contact | coordination, path deviation, gravity coupling, reachability of a real task | firmware timing, comms | seconds |
| **L3** | L2 plant + **the real firmware control law** in the loop | does the shipped code behave? timing, saturation, integer/float issues | electrical faults, thermal | seconds-minutes |
| **L4** | hardware | everything, expensively and slowly | - | evenings |

L0 and L1 exist. **L2 and L3 are what this plan builds.** L4 is the phase plan.

The rule that makes the ladder worth anything: **a level is only trusted after it
has reproduced a measurement from the level above it.** L1 is currently
*unvalidated* - its stiffness (300 N·m/rad) and damping (15 % of critical) are
estimates, and D10 says so explicitly. Phase 2's exit criterion is precisely to
close that.

---

## 3. Choosing the stack

### The candidates

| Tool | Strengths for this project | Costs / risks |
| ---- | -------------------------- | ------------- |
| **Drake** | Best-in-class multibody + optimisation. Its `systems` framework has explicit input/output ports and **per-system discrete update rates**, which maps 1:1 onto the D7 layer split - you can run layer 0 at 1 kHz and layer 1 at 200 Hz in one diagram with correct sampled-data semantics. MeshCat viewer. Already a demonstrated skill from the LCLS/XCS work. | **No Windows support** (Ubuntu + macOS only) → WSL2 or Docker. Steeper API. ROS 2 integration (`drake-ros`) is explicitly listed as unsupported. |
| **MuJoCo** | `pip install mujoco` works on Windows natively. Excellent contact, fast, great built-in viewer, loads URDF. Now the default in the learning/RL world. | Physics engine, not a systems framework - you write the control scheduling yourself. Weaker story for classical model-based verification. |
| **Gazebo + `ros2_control`** | This is what a ROS shop actually runs. Exercises the real `hardware_interface` boundary, so sim and hardware are driven by *identical* controller code. Strong hiring signal for ROS roles. | Heavy. Poor Windows support. Slow iteration. Large amount of yak-shaving before the first useful result. |
| **PyBullet** | Trivial install, loads URDF, fine for kinematics demos and quick videos. | Ageing, mediocre contact fidelity, weak analysis tooling. |
| **Isaac Sim** | Photorealistic, GPU, RL at scale. | Overkill, GPU-bound, enormous. Nothing in this project needs it. |
| **The existing custom Python** | Zero dependencies, models exactly the effects that matter (backlash, quantisation, step loss) - which no general-purpose engine models by default. | No geometry, no collisions, single DOF. |

### The decision, and the reasoning

**Do not choose a simulator. Choose a model format and keep the simulator
swappable.**

This is the same principle already used elsewhere in the repo: `AS5600Encoder`
hides the transport so the encoder can be swapped, and `JointCommand`/
`JointState` are transport-agnostic so serial can become CAN. Apply it here.

- **URDF is the interchange format**, generated from `tools/arm_model.py`, which
  is already the single source of truth for links, masses, and the DH table.
  Drake, MuJoCo, PyBullet, RViz, Gazebo and MoveIt all load URDF. Hand-writing a
  URDF that then drifts from `arm_model.py` is the failure mode to avoid, so it
  must be **generated and regenerated**, never edited.
- **Drake is the primary engine.** Reasons, in order: the sampled-data systems
  framework is the only one of these that natively expresses "layer 0 at 1 kHz,
  layer 1 at 200 Hz, layer 2 at 10 Hz", which is the architecture this project
  already committed to (D7); it is the tool where *verification* rather than
  *visualisation* is the organising idea; and it is a skill already in hand, so
  the marginal cost is low.
- **MuJoCo is the secondary engine, and it is not optional.** Loading the same
  URDF into a second, independently-implemented engine and getting the same
  answer is the cheapest available cross-check on the model - the same trick
  `tools/kinematics.py` already uses when it validates DH against
  product-of-exponentials. It also runs natively on Windows, which means there
  is always a path to a quick answer without booting WSL.
- **`ros2_control` + Gazebo is a Phase 6 decision, not now.** The hiring signal
  is real, but the cost is high and it buys nothing the arm needs before there
  are six joints. Revisit when the answer to "how do I integrate perception and
  planning" becomes urgent.

### The Windows constraint, stated plainly

Drake officially supports **Ubuntu 24.04 / 26.04 and macOS only**. On this
machine that means WSL2:

```powershell
wsl --install -d Ubuntu-24.04
```

then inside WSL:

```bash
python3 -m venv ~/venv-drake && source ~/venv-drake/bin/activate
pip install drake
```

MeshCat serves over HTTP, so the viewer opens in the Windows browser against
the WSL port with no extra setup. The repo is reachable from WSL at
`/mnt/c/Users/koashen/desktop-6dof-arm`, but **clone it into the WSL filesystem
instead** - `/mnt/c` I/O is slow enough to be annoying in a tight iterate loop.

MuJoCo, by contrast, is `pip install mujoco` in the existing Windows Python. Keep
that path working; it is the escape hatch on an evening when WSL is being
tiresome.

---

## 4. Architecture: the digital twin as a transport

The important design insight is that **this project already has the right
seam for a digital twin, and did not build it on purpose.**

D7 fixed three layers with hard boundaries. D12 put a framed binary protocol at
the layer-0/layer-1 boundary, and `tools/joint_link.py` already hides the serial
port behind an interface. So:

```
             layer 2            layer 1              transport            layer 0 + plant
          pick_place.py  →  trajectory.py  →  ┌── SerialTransport ──→ Uno/Teensy → real joint
                                              │
                                              └── SimTransport ─────→ Drake diagram
```

A simulation that speaks the same 8-byte `JointCommand`/`JointState` structs is
a **drop-in replacement for the hardware**, and every line of layer-1 and
layer-2 code is exercised unchanged. `pick_place.py --sim` and
`pick_place.py --port COM5` run the identical planner.

This is worth stating explicitly in interviews, because it is the difference
between "I made a simulation of my robot" and "I built my system so the
simulator and the hardware are interchangeable at a defined interface." The
second is what `ros2_control`'s `hardware_interface`, and every industrial HIL
rig, is doing.

### Files to create

```
sim/
  export_urdf.py      generates sim/arm.urdf from tools/arm_model.py - NEVER hand-edit the output
  arm.urdf            generated, committed, with a CI check that it is up to date
  meshes/             optional STLs; primitives are enough to start
  plant.py            Drake MultibodyPlant + reducer compliance + backlash + encoder quantisation
  joint_node.py       LeafSystem replicating JointController's control law at 1 kHz
  sim_transport.py    implements joint_link.py's transport interface, backed by plant.py
  station.py          the full diagram + MeshCat, runnable standalone
  validate.py         replays a recorded bench CSV through the sim and reports error metrics
  mujoco_check.py     loads the same URDF in MuJoCo and cross-checks FK and gravity torques
```

### Modelling the things a stock physics engine gets wrong

A general-purpose multibody engine models a robot arm as ideal revolute joints
with a torque input. That is not this arm. The effects that dominate its
behaviour must be added deliberately, and each has a measurement behind it:

| Effect | How to model it in Drake | Measured in |
| ------ | ------------------------ | ----------- |
| Reducer compliance | rotational spring-damper between a motor-side and load-side body, or a `LinearBushingRollPitchYaw` | Phase 1B hang-a-mass test |
| Backlash | dead zone in the transmitted torque as a function of relative angle | Phase 1B dial-indicator test |
| Stepper as a rate source | velocity-source actuator with a rate limit, not a torque source | Phase 1 max-speed test |
| Step loss | clamp: if required torque > available, drop the commanded position | Phase 1 stall test |
| Encoder quantisation + noise | quantise the measured angle to 12 bits, add Gaussian noise | Phase 1 rest-noise measurement |
| Coulomb + viscous friction | joint damping + a friction term | Phase 2 constant-velocity sweeps |
| Sample-and-hold + comms latency | discrete-update systems at the real rates, with a delay block | Phase 3 jitter measurement |

`tools/joint_sim.py` already implements the first five for one axis. **Port that
model, do not re-derive it** - and keep `joint_sim.py` alive as the fast L1
check that the L2 model must agree with.

---

## 5. Closing the sim-to-real gap: the system-ID plan

A simulation nobody validated is a video game. This is the part that turns it
into engineering, and it is the part almost no portfolio project has.

### Parameters to measure, not assume

| Parameter | Currently | Measurement | Effort |
| --------- | --------- | ----------- | ------ |
| Link masses, CoM | **estimated** in `arm_model.py` | kitchen scale + balance point on a knife edge | 20 min |
| Reducer torsional stiffness | **estimated** 300 N·m/rad | hang known mass at known radius, read encoder deflection, $k = \tau/\Delta\theta$ | 20 min |
| Damping ratio | **estimated** 15 % critical | tap test, log the ring-down, log-decrement | 20 min |
| Backlash | **estimated** 0.5° | dial indicator (Phase 1B) | 30 min |
| Coulomb + viscous friction | not modelled | constant-velocity sweeps at several speeds; intercept = Coulomb, slope = viscous | 1 h |
| Gearbox efficiency | assumed 80 % | stall torque on a lever arm vs motor torque × ratio | 30 min |
| Rotor inertia | catalogue | vendor spec is adequate | - |
| Loop rate + jitter | assumed | GPIO toggle + logic analyser (Phase 3) | 30 min |

Total: under a day of bench time, and it converts every simulation number in the
repo from plausible to defensible.

### The validation protocol

For each of a small set of **canonical manoeuvres**, run the identical command on
hardware and in sim, and report the error:

| Manoeuvre | Metric | Target agreement |
| --------- | ------ | ---------------- |
| 10° step, single joint | overshoot %, settling time | within 20 % |
| 45° step, single joint | peak velocity, settling time | within 20 % |
| Constant-velocity sweep | steady following error | within 0.05° |
| Rest with drivers enabled | hunting pk-pk | same order of magnitude |
| Two-joint synchronised move | arrival spread | within 10 ms |
| Full pick-place cycle | tool-path deviation | within 2 mm |

`sim/validate.py` should take a bench CSV from `docs/test-results/`, replay the
same commands through the sim, and emit a table plus an overlay plot. **That
overlay plot is the single most valuable figure this project can produce.** It
is the figure that says "I know how wrong my model is."

Report the gap as one headline number, e.g. *"the simulator predicts settling
time within 12 % and steady-state following error within 0.03° across six
canonical manoeuvres"*, and keep the per-manoeuvre table behind it.

### Where the gap will be worst, predicted in advance

Writing this down before measuring is the honest version of the exercise:

1. **Friction near zero velocity.** Stribeck effects are not in any of these
   models and they dominate small moves. Expect the sim to be optimistic about
   small-step settling.
2. **Backlash under load.** Measured statically, it will be smaller than what
   the joint exhibits dynamically, because the printed structure also winds up.
3. **Thermal drift.** Nothing models a motor going from 25 °C to 65 °C, which
   changes torque and the magnet's field.
4. **Print-to-print variation.** Two nominally identical joints will not have
   identical stiffness or backlash. The model has one number; reality has six.

---

## 6. Staged roadmap

Each stage is independently showable and each has an exit criterion that is a
number or an artifact, matching the phase-plan's convention.

### S0 - URDF as the single source of truth *(no hardware needed, do now)*
- `sim/export_urdf.py` generating `sim/arm.urdf` from `tools/arm_model.py`
- Load it in MuJoCo (Windows) and in Drake (WSL); both viewers show the arm
- **Cross-check:** Drake's and MuJoCo's forward kinematics must both agree with
  `tools/kinematics.py` to < 1e-9 m at 500 random configurations. Three
  independent implementations agreeing is the same validation pattern
  `kinematics.py` already uses internally
- **Exit:** 500/500 FK agreement across three implementations; a screenshot

### S1 - Kinematic twin *(no hardware)*
- Drive the URDF from `tools/pick_place.py`'s planned joint trajectory
- MeshCat animation of the full eight-waypoint cycle
- Collision checking: does the arm hit itself or the table on the planned path?
  This is a question `pick_place.py` **cannot currently answer** and the first
  genuinely new capability the sim buys
- **Exit:** the planned pick-place cycle animates end to end; a self-collision
  is detected and reported for at least one deliberately bad waypoint

### S2 - Dynamic twin *(no hardware)*
- `MultibodyPlant` with gravity, weighed masses, reducer compliance, backlash
- Joint-level control law replicated as a Drake `LeafSystem` at 1 kHz
- Reproduce every result in D10's table (naive PID hunting, deadband, feedforward)
- **Exit:** L2 reproduces L1's four-case table to within 20 %. Any disagreement
  is investigated, because two models of the same thing disagreeing means at
  least one is wrong

### S3 - `SimTransport` *(no hardware)*
- `sim/sim_transport.py` implements the same interface as `tools/joint_link.py`
- `python pick_place.py --sim` runs the entire host stack against the simulator
- **Exit:** the same script, unmodified, drives sim and hardware. A recorded
  video of both side by side

### S4 - Validation *(needs Phases 1-2)*
- System-ID measurements above, fed back into both L1 and L2
- `sim/validate.py` overlay plots and the sim-to-real gap table
- **Exit:** the six-manoeuvre agreement table filled in, with a headline gap
  figure

### S5 - Firmware-in-the-loop *(needs a host compiler)*
- Compile `lib/JointNode/JointController.h` for the host and bind it to Python
  (pybind11), or run it as a native process talking COBS frames over a socket
- Now the **shipped firmware code** is the controller in the loop, not a Python
  replica. Every bug the replica cannot have - integer overflow, float
  precision, saturation, a `uint8_t` fault mask - becomes testable
- This needs the host toolchain that `pio test -e native_tests` currently lacks
  (MSYS2: `pacman -S mingw-w64-ucrt-x86_64-gcc`), so it also fixes a standing gap
- **Exit:** a CI job that runs the real control law against the simulated plant
  and asserts on overshoot and settling time

### S6 - ROS 2, if and when *(Phase 6)*
- URDF already exists, so `robot_state_publisher` + RViz is nearly free
- `ros2_control` `hardware_interface` with a real and a simulated backend is the
  industry-standard version of the S3 seam
- MoveIt 2 for planning, if collision-aware planning becomes the bottleneck
- **Do not start this before S4.** A ROS stack on an unvalidated model is
  impressive-looking and proves nothing

---

## 7. What the simulation will *not* model, stated up front

Being explicit about this is a signal of maturity, and the omissions are the
first thing a good interviewer probes.

- **Grasp success.** D11 already says the gripper is open loop with no way to
  know whether an object is held. Simulating a successful grasp does not change
  that; if anything it makes it easier to fool yourself.
- **Thermal behaviour.** Motor torque falls as it heats; printed PETG softens
  near 80 °C. Neither is modelled.
- **Electrical faults.** I2C corruption from motor commutation is a real,
  measured failure mode (D12's justification for CRC-8) and no physics engine
  produces it.
- **Print-to-print variation.** One set of parameters, six real joints.
- **Wear.** Printed pins and discs change over hundreds of cycles. The model is
  a snapshot of a new gearbox.
- **Contact realism at the fingertips.** TPU compliance under a rack-and-pinion
  is exactly the regime where rigid-body contact models are least trustworthy.

---

## 8. Turning this into the artifact

For a poster or a portfolio page, the figures that carry weight, in order:

1. **The overlay plot** - commanded, simulated, and measured, on one time axis,
   with the residual underneath. Everything else is decoration.
2. **The sim-to-real gap table** - six manoeuvres, predicted vs measured, %
   error, and a sentence on the largest disagreement.
3. **A "the simulation caught this" story** - you already have one (D10's 83
   reversals/s). Get a second one from S1's collision checking or S2's
   coordination.
4. **The interchangeability diagram** - the layer/transport figure from §4,
   showing that one script drives both sim and hardware.
5. **The fidelity ladder table** - it demonstrates that you chose fidelity
   deliberately rather than reaching for the biggest tool available.

Three sentences worth having ready:

> *"The simulator and the hardware are interchangeable behind the same 8-byte
> joint protocol, so the planner and trajectory generator are tested unchanged
> against both."*

> *"The model predicts settling time within N % across six canonical manoeuvres;
> the largest disagreement is at small step sizes, and it is Stribeck friction,
> which I don't model."*

> *"I found the control law's limit cycle in simulation before the hardware
> arrived - 83 direction reversals per second at rest - and the fix was a
> one-count deadband, not a gain change."*

---

## 9. Resources

### Drake
- `drake.mit.edu` - installation, tutorials, and the API. Start with the
  *Authoring a Multibody Simulation* and *Dynamical Systems* tutorials.
- **Robotic Manipulation**, Russ Tedrake - `manipulation.mit.edu`. Free, taught
  from Drake, and chapters 2-3 and 6 and 8 are directly this project. Chapter 2
  ("Let's get you a robot") covers robot description files and the
  `HardwareStation` abstraction, which is the same seam as §4.
- **Underactuated Robotics**, Tedrake - `underactuated.mit.edu`. For when
  control gets past PID.
- Search: *"Drake LeafSystem discrete update periodic"* - this is the mechanism
  that expresses the D7 layer rates.

### MuJoCo
- `mujoco.readthedocs.io` - the *Overview* and *Modeling* chapters. Note it
  loads URDF as well as its native MJCF.
- `github.com/google-deepmind/mujoco_menagerie` - production-quality models of
  real robots, including arms. Read them as examples of how to specify inertia,
  damping and actuators properly.

### URDF, ROS 2, and the industry interfaces
- `docs.ros.org` - the ROS 2 documentation, URDF tutorials specifically.
- `control.ros.org` - `ros2_control`. Read the `hardware_interface` concept page
  even if you never adopt ROS; it is the canonical statement of the sim/hardware
  interchangeability idea.
- **Articulated Robotics** (YouTube, `@ArticulatedRobotics`) - the clearest
  practical ROS 2 + URDF + Gazebo series available, built around an actual robot.
- `moveit.ai` - MoveIt 2, for Phase 6 planning.
- Search: *"URDF inertia tensor common mistakes"* - almost every hand-written
  URDF has wrong inertias, which makes the dynamics silently meaningless.

### Modelling and system identification
- **Modern Robotics**, Lynch & Park - chapter 8 (dynamics of open chains) is
  what the multibody model *is*.
- Search: *"system identification step response least squares"*.
- Search: *"Stribeck friction model servo"* - the effect the model will miss.
- Search: *"harmonic drive stiffness hysteresis model"* - cycloidal and strain
  wave drives share the nonlinear stiffness-plus-backlash behaviour, and the
  harmonic-drive literature is much richer.

### The general practice
- Search: *"sim to real gap robotics domain randomisation"* - the RL framing, but
  the vocabulary is standard now.
- Search: *"hardware in the loop testing definition"* - what S5 is, in industry
  terms.
- Search: *"verification and validation V&V model credibility NASA-STD-7009"* -
  the aerospace standard for how much to trust a model. Overkill to comply with,
  but the vocabulary (verification vs validation, credibility assessment) is
  exactly what a systems-minded interviewer is checking for.
