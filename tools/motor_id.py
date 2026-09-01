"""Turn one-motor bench measurements into the numbers the reducer design needs.

Run:  python tools/motor_id.py

WHY THIS EXISTS
---------------
`tools/torque_budget.py` currently sizes the whole arm from two unmeasured
inputs: a datasheet holding torque per motor length, and a 5 N.m gearbox
rating borrowed from someone else's product. D15 says no model gets quoted
until its gap against the level above it is a number. This file is the level
above: it converts a bench session with ONE motor, no reducer, into a measured
torque-speed curve.

HOLDING TORQUE IS THE WRONG NUMBER
----------------------------------
A stepper's holding torque is a DC measurement. The reducer is sized by the
torque available at the speed the joint actually runs, and a chopper drive's
torque collapses above a corner speed set by the winding inductance:

    V = I * sqrt(R^2 + (w_e L)^2)        # the chopper runs out of volts

Above that the current never reaches its setpoint within a step and torque
rolls off roughly as a single pole. The consequence for P1-a is the part
people miss: **ratio and speed are coupled.** Doubling the ratio doubles the
output torque only if the motor can still make rated torque at twice the step
rate. Past the corner it cannot, and the extra ratio buys nothing but noise.
`ratio_sweep()` is that tradeoff evaluated against measured data.

The 2026-08-31 bench ran at 12 V and 1600 microsteps/s = 200 full steps/s.
`corner_step_rate()` puts the corner for a typical 42 mm motor at 12 V near
2700 full steps/s, so that session explored under 10 % of the usable speed
range and told us nothing about the roll-off. That is the gap this file closes.

Every estimator here is validated in `_self_check()` against synthetic data
with known truth, on the same principle as `sim/sysid.py`: an estimator that
cannot recover parameters from clean simulated data has no business being
pointed at a noisy bench.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

G = 9.80665
FULL_STEPS_PER_REV = 200.0          # 1.8 deg
ELEC_CYCLE_FULL_STEPS = 4.0         # 2-phase bipolar: 4 full steps per electrical rev


# --------------------------------------------------------------- measurement --

def torque_from_lever(mass_kg: float, lever_m: float) -> float:
    """N.m from a hanging mass on a lever arm. The whole torque rig."""
    return mass_kg * G * lever_m


def full_steps_per_sec(microstep_rate: float, microsteps: float = 8.0) -> float:
    """Torque roll-off is an electrical effect, so it lives in FULL steps/s.

    Microstepping subdivides a step, it does not change the electrical
    frequency at a given shaft speed - so a 1600 microstep/s command at 8x is
    200 full steps/s, not 1600.
    """
    return microstep_rate / microsteps


def output_deg_s(step_rate_full: float, ratio: float) -> float:
    """Output speed, deg/s, for a full-step rate through a reducer."""
    return 360.0 * step_rate_full / (FULL_STEPS_PER_REV * ratio)


def step_rate_for_output(out_deg_s: float, ratio: float) -> float:
    """Inverse of output_deg_s: full steps/s needed for an output speed."""
    return out_deg_s * FULL_STEPS_PER_REV * ratio / 360.0


# ------------------------------------------------------------------- models --

def corner_step_rate(volts: float, inductance_h: float, current_a: float,
                     resistance_ohm: float = 0.0) -> float:
    """Full steps/s where the supply can no longer drive rated phase current.

    Derived, not fitted - use it to sanity-check a measured fit, and to decide
    a supply voltage before buying anything. Note it scales linearly with V:
    this is the entire argument for running the bench at 24 V rather than 12 V.
    """
    if current_a <= 0 or inductance_h <= 0:
        raise ValueError("current and inductance must be positive")
    headroom = (volts / current_a) ** 2 - resistance_ohm ** 2
    if headroom <= 0:
        raise ValueError("supply cannot even drive DC rated current through R")
    omega_e = math.sqrt(headroom) / inductance_h
    return ELEC_CYCLE_FULL_STEPS * omega_e / (2.0 * math.pi)


def torque_at(step_rate_full: float | np.ndarray, t0_nm: float,
              corner_sps: float) -> float | np.ndarray:
    """Single-pole pull-out model: T(f) = T0 / sqrt(1 + (f/fc)^2)."""
    f = np.asarray(step_rate_full, float)
    return t0_nm / np.sqrt(1.0 + (f / corner_sps) ** 2)


def pullout_fit(step_rate_full, torque_nm) -> tuple[float, float]:
    """(T0, corner) from measured pull-out points, by exact linearisation.

    1/T^2 = (1/T0^2) + (1/(T0^2 fc^2)) f^2 is linear in f^2, so this is an
    ordinary least squares fit with no initial guess and no solver to babysit.
    """
    f = np.asarray(step_rate_full, float)
    t = np.asarray(torque_nm, float)
    if f.size < 2:
        raise ValueError("need at least two pull-out points")
    if np.any(t <= 0):
        raise ValueError("torque must be positive")
    a, b = np.polyfit(f ** 2, 1.0 / t ** 2, 1)[::-1]
    if a <= 0 or b <= 0:
        raise ValueError("fit is non-physical - check for accelerating samples")
    return 1.0 / math.sqrt(a), math.sqrt(a / b)


def load_pullout_csv(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Read `full_steps_per_sec,torque_nm` rows; `#` lines are comments."""
    data = np.genfromtxt(path, delimiter=",", comments="#")
    data = np.atleast_2d(data)
    return data[:, 0], data[:, 1]


# ------------------------------------------------------------ design sweep --

def ratio_sweep(t0_nm: float, corner_sps: float, ratios,
                required_out_nm: float, target_out_deg_s: float,
                efficiency: float = 0.80,
                design_factor: float = 1.5) -> list[dict]:
    """Available vs required output torque at each candidate ratio.

    The point of the table: available torque is NOT `T0 * ratio * eta`. It is
    the torque at the step rate that ratio forces the motor to run at for the
    target output speed, which is why the column stops rising.
    """
    rows = []
    for r in ratios:
        f = step_rate_for_output(target_out_deg_s, r)
        t_motor = float(torque_at(f, t0_nm, corner_sps))
        avail = t_motor * r * efficiency
        rows.append({
            "ratio": r,
            "step_rate_sps": f,
            "motor_nm": t_motor,
            "motor_frac": t_motor / t0_nm,
            "output_nm": avail,
            "margin": avail / (required_out_nm * design_factor),
        })
    return rows


def max_output_speed(t0_nm: float, corner_sps: float, ratio: float,
                     required_out_nm: float, efficiency: float = 0.80,
                     design_factor: float = 1.5) -> float:
    """Fastest output speed, deg/s, still meeting the torque requirement."""
    need_motor = required_out_nm * design_factor / (ratio * efficiency)
    if need_motor >= t0_nm:
        return 0.0
    f = corner_sps * math.sqrt((t0_nm / need_motor) ** 2 - 1.0)
    return output_deg_s(f, ratio)


# ------------------------------------------------------------- bench plan ---

def bench_plan() -> str:
    return """\
ONE-MOTOR PULL-OUT SESSION (no reducer, no mux, encoder on the motor shaft)

Rig: printed spool of known radius r on the shaft, string, hanging mass m.
     Torque = m*g*r. Two or three masses cover the useful range.
     The encoder IS the instrument: commanded steps vs encoder angle. A skip
     is a permanent offset, not a transient - integrate, do not eyeball.

For each mass:
  1. Command a constant microstep rate, lifting, for 5 s.
  2. Record commanded steps and encoder angle at >= 200 Hz.
  3. Step loss = (cmd_deg - enc_deg) that does not return to zero on stop.
  4. Bisect the rate until you find the highest rate with zero lost steps.
     That point is one (full_steps_per_sec, torque_nm) row.

Rules that decide whether the data means anything:
  - Accelerate into the test rate, then measure only the CONSTANT-velocity
    window. Accelerating samples put J*alpha into the torque number, the same
    trap sysid.friction_lsq has.
  - Hold Vref fixed for the whole sweep and record it. Phase current is the
    other axis of this surface; changing it mid-sweep makes the fit meaningless.
  - Run the sweep at the supply voltage you intend to SHIP (24 V), not the
    12 V bring-up voltage. The corner speed is proportional to V.
  - Log motor case temperature at the end of each run. The thermal ceiling and
    the torque ceiling are different limits and either can be the binding one.

Also worth capturing in the same session, because they are free once rigged:
  - Holding torque: increase the hanging mass at zero step rate until the
    shaft slips. That is the f=0 anchor of the fit.
  - Thermal soak: 20 min at working current, case temperature vs time. PETG
    softens near 80 C and the joint housing bolts to this motor's face.
  - Microstep linearity: slow single steps, encoder angle per step. Whatever
    error you do not measure here you will blame on the gearbox later.
"""


# ------------------------------------------------------------- self-check ---

def _self_check() -> None:
    print("[1] corner speed against a hand calculation")
    fc = corner_step_rate(volts=24.0, inductance_h=0.0028, current_a=1.0,
                          resistance_ohm=1.5)
    assert 5000 < fc < 6000, fc
    fc12 = corner_step_rate(12.0, 0.0028, 1.0, 1.5)
    assert abs(fc / fc12 - 2.0) < 0.02, "corner speed must scale with volts"
    print(f"    24 V -> {fc:.0f} full steps/s, 12 V -> {fc12:.0f} ({fc/fc12:.2f}x)")

    print("[2] pull-out fit recovers known truth from clean data")
    t0_true, fc_true = 0.42, 3000.0
    f = np.array([0.0, 250.0, 500.0, 1000.0, 2000.0, 4000.0, 6000.0])
    t = torque_at(f, t0_true, fc_true)
    t0, fcx = pullout_fit(f, t)
    assert abs(t0 - t0_true) / t0_true < 1e-6
    assert abs(fcx - fc_true) / fc_true < 1e-6
    print(f"    T0 {t0:.4f} N.m (truth {t0_true}), fc {fcx:.1f} (truth {fc_true})")

    print("[3] fit survives 3 % measurement noise")
    rng = np.random.default_rng(0)
    t_noisy = t * (1.0 + 0.03 * rng.standard_normal(t.size))
    t0n, fcn = pullout_fit(f, t_noisy)
    assert abs(t0n - t0_true) / t0_true < 0.05
    assert abs(fcn - fc_true) / fc_true < 0.20
    print(f"    T0 {t0n:.4f} ({100*abs(t0n-t0_true)/t0_true:.1f} % off), "
          f"fc {fcn:.0f} ({100*abs(fcn-fc_true)/fc_true:.1f} % off)")

    print("[4] a low-speed-only sweep cannot see the corner - the 2026-08-31 case")
    f_low = np.array([50.0, 100.0, 150.0, 200.0])
    truth_low = torque_at(f_low, t0_true, fc_true)
    ests, failures = [], 0
    for seed in range(20):
        r = np.random.default_rng(seed)
        try:
            ests.append(pullout_fit(f_low, truth_low * (1.0 + 0.03 * r.standard_normal(4)))[1])
        except ValueError:
            failures += 1                      # the fit goes non-physical outright
    spread = max(ests) / min(ests) if ests else float("inf")
    assert failures + sum(e < 0.5 * fc_true or e > 2 * fc_true for e in ests) >= 18
    print(f"    f <= 200 sps over 20 noise seeds: {failures} fits non-physical, "
          f"the rest span {min(ests):.0f}-{max(ests):.0f} ({spread:.0f}x) vs truth "
          f"{fc_true:.0f} - sweep past the corner or do not quote one")

    print("[5] speed/torque coupling: available output torque is not linear in ratio")
    rows = ratio_sweep(t0_true, fc_true, (5, 10, 20, 40, 80),
                       required_out_nm=2.0, target_out_deg_s=90.0)
    naive = [r["ratio"] * t0_true * 0.80 for r in rows]
    real = [r["output_nm"] for r in rows]
    assert real[-1] < naive[-1] * 0.9, "expected roll-off to bite at high ratio"
    assert real[0] / naive[0] > real[-1] / naive[-1], "penalty must grow with ratio"
    print(f"    80:1 naive {naive[-1]:.2f} N.m vs measured-curve {real[-1]:.2f} N.m "
          f"({100*(1-real[-1]/naive[-1]):.0f} % optimistic)")

    print("[6] max_output_speed inverts torque_at")
    v = max_output_speed(t0_true, fc_true, ratio=20, required_out_nm=2.0)
    f_at_v = step_rate_for_output(v, 20)
    assert abs(float(torque_at(f_at_v, t0_true, fc_true)) * 20 * 0.80
               - 2.0 * 1.5) < 1e-6
    print(f"    20:1 meets 2.0 N.m x1.5 up to {v:.1f} deg/s")

    print("\nall motor_id self-checks passed")


def main() -> None:
    _self_check()

    print("\n" + "=" * 72)
    print(bench_plan())

    # Placeholder curve until the bench session replaces it. Datasheet holding
    # torque for a 40 mm NEMA 17, corner from the electrical model at 24 V.
    t0, fc = 0.42, corner_step_rate(24.0, 0.0028, 1.0, 1.5)
    print("=" * 72)
    print(f"PROVISIONAL curve (NOT measured): T0 = {t0:.2f} N.m, "
          f"corner = {fc:.0f} full steps/s")

    try:
        from arm_model import default_arm
        arm = default_arm(0.400, 0.500)
        required, _ = arm.worst_case_torque(1)
    except Exception:
        required = 2.0
    print(f"shoulder worst-case demand: {required:.2f} N.m, design factor 1.5x\n")

    print(f"{'ratio':>6} {'motor sps':>10} {'T_motor':>8} {'of T0':>7} "
          f"{'T_out':>7} {'margin':>7}   at 90 deg/s output")
    print("-" * 72)
    for r in ratio_sweep(t0, fc, (10, 15, 20, 26, 40, 60), required, 90.0):
        print(f"{r['ratio']:6.0f} {r['step_rate_sps']:10.0f} {r['motor_nm']:8.3f} "
              f"{100*r['motor_frac']:6.0f}% {r['output_nm']:7.2f} "
              f"{r['margin']:6.2f}x")
    print("\nRead the 'of T0' column before the 'T_out' column: once it drops "
          "below ~70 % the extra ratio is being spent on step rate, not torque.")


if __name__ == "__main__":
    main()
