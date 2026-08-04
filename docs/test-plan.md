# Test plan

Simulation answers "what will it do." Testing answers "is it still doing it."
They are separate disciplines and this repo currently has good instincts about
the first and no infrastructure for the second.

Companion to [simulation-plan.md](simulation-plan.md). That file builds the
models; this one builds the harness that runs them automatically and the
protocol that turns bench sessions into evidence.

---

## 1. Where testing stands today

Honest inventory, because a test plan that misrepresents the starting point is
useless.

| Layer | What exists | Runs automatically? |
| ----- | ----------- | ------------------- |
| Firmware unit tests | `test/test_armmath`, `test_link`, `test_motion` under Unity | no - manual `pio test` |
| Firmware build coverage | 9 environments | no |
| Python correctness | `__main__` self-checks in `kinematics.py`, `trajectory.py`, `joint_link.py`, `workspace.py`, `joint_sim.py` | **no - they are in `__main__` blocks, so nothing can gate on them** |
| Cross-implementation checks | DH vs PoE, analytic vs numeric Jacobian, `FK(IK(pose)) == pose`, COBS/CRC mirrored both sides | run by hand |
| Hardware acceptance | `bringup-checklist.md` gates | manual, by design |
| Regression detection | none | - |

Two gaps do real damage:

1. **`pio test -e native_tests` does not run here** - no host g++ installed. So
   the tests that could run in a second run on a 16 MHz AVR over USB instead.
   Installing MinGW-w64 fixes this and is also a prerequisite for
   simulation-plan S5.
2. **The Python self-checks live in `__main__`.** They are good tests trapped in
   a place no CI can reach. Moving them to `pytest` is a mechanical change with
   a large payoff.

---

## 2. The test pyramid, instantiated

| Tier | What | Runs in | Gate |
| ---- | ---- | ------- | ---- |
| **T1 Unit** | `ArmMath`, `Trajectory`, `PacketFraming`, `PidController` | < 1 s, host | every commit |
| **T2 Property** | `FK(IK(pose)) == pose`; DH ≡ PoE; analytic J ≡ finite-difference J; COBS round-trip; CRC rejects a flipped bit | seconds, host | every commit |
| **T3 Build** | all 9 PlatformIO environments compile; **RAM/flash recorded** | ~2 min | every commit |
| **T4 Model** | `joint_sim.py` reproduces D10's four-case table; `trajectory.py` limits are respected | seconds | every commit |
| **T5 Integration (sim)** | `pick_place.py --sim` completes a full cycle through `SimTransport`; no self-collision; arrival spread within tolerance | ~1 min | every commit, once S3 exists |
| **T6 HIL smoke** | flash `host_link`, home, single joint step, read state back, fault on a deliberate limit violation | minutes, needs hardware | before every recorded bench session |
| **T7 Acceptance** | the phase exit criteria - repeatability, backlash, step response, path deviation | evenings | once per phase, recorded |

T1-T5 are free after the first setup and should be a gate. T6-T7 are human and
should be a **procedure**, which is what the bring-up checklist already is.

### Property tests deserve emphasis

`FK(IK(pose)) == pose` over 500 random poses caught the $\theta_2$ quadrant
error that four hand-checked poses did not. That is not luck; it is the
structural advantage of property-based testing over example-based testing, and
this codebase is unusually full of properties worth asserting:

- FK ∘ IK is the identity on reachable poses (all 8 branches)
- IK branches all map to the same tool pose
- `AngleUnwrapper` is continuous under any sample sequence with < 180° steps
- A trapezoidal profile never exceeds `TRAJ_MAX_VEL` or `TRAJ_MAX_ACC`
- Synchronised axes have identical total duration
- `COBS(decode(encode(x))) == x` for all payloads, and encoded frames contain no
  `0x00` except the terminator
- CRC-8 rejects every single-bit flip
- `JointCommand`/`JointState` are exactly 8 bytes (a `static_assert` already
  does this; keep it)
- Fault testing uses `fault & kFaultMask`, never `fault != 0` (D12 - both sides
  already assert this, because the bug only appears when the arm is working)

Adding `hypothesis` to the Python side is a small dependency for a large
increase in the class of bug these catch.

---

## 3. CI

GitHub Actions, one workflow, three jobs. Nothing here needs a self-hosted
runner.

```yaml
# .github/workflows/ci.yml  (sketch - write it when you create the repo remote)
jobs:
  firmware:      # T1, T3 - pio run for all 9 envs + pio test -e native_tests
  python:        # T1, T2, T4 - pytest over tools/ and scripts/
  model:         # T5 - pick_place.py --sim, once SimTransport exists
```

Details that matter:

- **Record RAM/flash per environment as a CI artifact.** D13's 74.2 % figure is
  a load-bearing measurement in the architecture argument; a job that prints it
  every commit turns it into a tracked trend instead of a one-off. Fail the
  build if any environment exceeds 85 % RAM.
- **Assert the generated URDF is up to date.** Run `sim/export_urdf.py`, diff
  against the committed `sim/arm.urdf`, fail on difference. This is what stops
  the model and the geometry from silently diverging - the single most likely
  way this project ends up with a lying simulation.
- **Pin the Python version and dependencies.** `scripts/requirements.txt` exists;
  add `tools/requirements.txt` (numpy, pytest, hypothesis, matplotlib) and a
  `constraints` pin so a numpy release does not break the build on a random
  Tuesday.
- **`pio test -e native_tests` in CI is free** - Ubuntu runners have g++. The
  local gap does not have to block the automated gate.

### Migrating the `__main__` self-checks

Mechanical, and worth doing in one sitting:

```
tests/
  test_kinematics.py     FK/IK round trip, DH vs PoE, Jacobian vs finite difference
  test_trajectory.py     the 13 profile/sync checks
  test_joint_link.py     COBS/CRC properties, 8-byte struct sizes, fault masking
  test_joint_sim.py      D10's four cases, as regression assertions with tolerances
  test_workspace.py      reach, dead zone, singularity fractions - loose bounds
  test_arm_model.py      mass/CoM consistency, DH table self-consistency
```

Keep the `__main__` blocks - they are good interactive documentation. Have them
call the same functions the tests call, so there is one implementation.

---

## 4. Requirements, and how each is verified

This is the table that makes the project legible to a systems engineer, and it
is the one a hiring manager can scan in fifteen seconds. Fill the "measured"
column as phases close.

| ID | Requirement | Target | Verified by | Tier | Measured |
| -- | ----------- | ------ | ----------- | ---- | -------- |
| R-1 | Geometric reach, base axis to flange | 400 mm | `tools/workspace.py` + tape measure on the built arm | T4/T7 | 379 mm usable (D8b) |
| R-2 | Payload at full extension | 500 g | static hold test, 60 s, encoder drift < 1 count | T7 | |
| R-3 | Pose repeatability | < 1 mm, unidirectional approach | 20-cycle return-to-place, dial indicator at the tool | T7 | |
| R-3b | Pose accuracy, uncalibrated | budget predicts ~20 mm | `tools/error_budget.py` vs measured (D17) | T4 | |
| R-4 | Joint backlash | ≤ 1.0° | `calibration` app + dial indicator, **axis vertical** (D18) | T7 | |
| R-5 | Shoulder gearbox design factor | ≥ 1.5× | `tools/torque_budget.py` with **weighed** masses | T4 | |
| R-6 | Closed-loop overshoot | ≤ 5 % | step response, three sizes | T7 | |
| R-7 | Multi-axis arrival spread | ≤ 20 ms | two-joint synchronised move | T5/T7 | `test_motion` asserts; bench pending |
| R-8 | Fault response: soft limit, encoder loss, comms loss | driver disabled < 50 ms | deliberate fault injection | T6 | |
| R-9 | E-stop | cuts VMOT, logic survives | pressed under load | T6 | |
| R-10 | Host link headroom | ≤ 30 % of link bandwidth at 200 Hz | packet accounting | T2 | 16 % at 500 kbaud (D12) |
| R-11 | Controller RAM headroom | ≤ 85 % | CI build artifact | T3 | 74.2 % on Uno (D13 + soft-limit margin) |
| R-12 | Sim-to-real agreement | within 20 % on six canonical manoeuvres | `sim/validate.py` | T5+T7 | |
| R-13 | Task cycle repeatability | spread at place point over 20 cycles | pick-place demo | T7 | |

R-12 is the one that does not appear on hobby projects and does appear on real
ones.

---

## 5. Bench test protocol

The habit that separates data from anecdote. One record per run, always:

```
docs/test-results/<phase>_<app>_<YYYYMMDD-HHMM>.csv     raw telemetry
docs/test-results/<phase>_<app>_<YYYYMMDD-HHMM>.png     generated plot
docs/test-results/<phase>_<app>_<YYYYMMDD-HHMM>.md      the note
```

The note is five lines and takes two minutes:

```markdown
Date / app / env / git SHA
Config: GEAR_RATIO, MICROSTEPS, Vref, PID gains, TRAJ limits
Setup: what was physically attached, what was loaded, what was clamped
Changed since last run: one line
Result: the numbers
Conclusion: one sentence, including "this contradicts X" if it does
```

**The git SHA is the important field.** A measurement you cannot tie to a
firmware version is a measurement you cannot reproduce, and six months later
that is every measurement you have.

Rules:

- Change **one thing** between runs. Two changes and a surprising result is an
  afternoon lost.
- Record the failures. A run where the joint stalled is data about the stall
  boundary, and deleting it is discarding the most interesting half of the
  dataset.
- Re-run the previous configuration occasionally. If yesterday's number does not
  reproduce, something drifted - a magnet, a set screw, a temperature - and you
  want to know that before it corrupts a week of results.

### Golden traces

Once a configuration is trusted, freeze one run as a **golden trace** and commit
it. Then `sim/validate.py` and the T5 job can assert against it forever. This is
how a regression in the trajectory generator gets caught by CI instead of by a
confusing evening at the bench.

---

## 6. Fault injection

Safety claims are only worth what you have deliberately provoked. Each of these
is a scripted test with a recorded outcome, and each should be filmed once.

| Injected fault | How | Expected | Tier |
| -------------- | --- | -------- | ---- |
| Soft limit violation | command a setpoint past `JOINT_MAX_DEG` | `FAULT_SOFT_LIMIT`, driver disabled | T6 |
| Encoder loss | unplug the AS5600 mid-move | `FAULT_ENCODER`, driver disabled | T6 |
| Comms loss | kill the host process mid-trajectory | `FAULT_COMMS` after 500 ms, joint holds then stops | T6 |
| Corrupted frame | inject a flipped bit into a COBS frame | CRC rejects, no state change | T2 |
| Frame resync | inject garbage, then a valid frame | receiver resynchronises at the next `0x00` | T2 |
| Step saturation | plan above `MAX_SPEED_STEPS_PER_SEC` | large following error, **not** silent corruption - and the log should make the saturation visible | T4 |
| Power brownout | drop the PSU below the driver's UVLO mid-move | driver faults, arm does not run away on recovery | T6 |
| E-stop under load | press it while holding a payload | VMOT gone, logic alive, state still readable | T6 |

The last one is the important one. D7 says "layer 0 must be safe alone" and D12
says "test faults with `fault & kFaultMask`". Both are claims. This table is how
they become facts.

---

## 7. What not to test

Test budget is finite and testing the wrong things is how test suites become
maintenance burdens people delete.

- **Don't test the Arduino core.** `digitalWrite` works.
- **Don't unit-test the apps.** They are wiring; the logic lives in `lib/` and
  that is where the tests belong. This is exactly why the repo separates them.
- **Don't assert on absolute simulation numbers.** Assert on *relationships* -
  "deadband ≥ 1 count gives zero reversals", "feedforward reduces overshoot by
  more than 5×" - because those survive a model refinement and a hard-coded
  1.37 s settling time does not.
- **Don't test the physics engine.** Cross-check two engines against each other
  and against closed-form kinematics; do not verify Drake's integrator.
- **Don't chase coverage percentage.** Coverage of `ArmMath` and
  `PacketFraming` matters. Coverage of a bring-up app does not.

---

## 8. Order of work

Roughly a weekend's worth, and it unblocks everything in
[simulation-plan.md](simulation-plan.md).

1. Install MinGW-w64 so `pio test -e native_tests` runs locally *(30 min)*
2. Move the Python `__main__` self-checks into `tests/` under pytest *(2 h)*
3. Write `.github/workflows/ci.yml` covering firmware builds + pytest *(1 h)*
4. Add the RAM/flash reporting step and the 85 % gate *(30 min)*
5. Add `hypothesis` and convert the FK/IK round-trip into a property test *(1 h)*
6. Fill in the requirements table's targets, leaving "measured" blank *(30 min)*
7. Write the test-note template into `docs/test-results/` *(15 min)*
8. Then start simulation-plan S0

---

## 9. Resources

- Search: *"test pyramid embedded systems"* - the standard shape, and why the
  embedded version is squatter than the web version.
- Search: *"property based testing hypothesis python"* - `hypothesis` docs are
  excellent and the falsifying-example shrinking is genuinely magic the first
  time it finds a bug.
- Search: *"hardware in the loop testing automotive"* - the industry where HIL is
  most mature; the vocabulary transfers directly.
- Search: *"ISO 9283 industrial robot performance criteria"* - pose
  repeatability, pose accuracy, path accuracy, and how they are actually
  measured. Using these terms correctly is a real differentiator.
- Search: *"verification vs validation engineering"* - verification is "did I
  build it right", validation is "did I build the right thing". Mixing them up
  in an interview is noticeable.
- `docs.platformio.org` - the unit-testing and `test_filter` documentation.
  Note: **do not** put `test_filter` on one line as a space-separated list; it
  silently collects zero test cases.
- **Modern Robotics**, Lynch & Park - for the properties worth asserting about
  kinematics in the first place.
