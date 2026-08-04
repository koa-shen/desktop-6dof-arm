"""Parameter estimators for the joint plant, and the experiments that feed them.

This is the machinery behind S4 of docs/simulation-plan.md. D15 says a model is
not trusted until its disagreement is a number; this file is how the model's
inputs stop being guesses in the first place.

Every estimator here is validated against synthetic data whose truth is known,
because an estimator that cannot recover parameters from clean simulated data
has no business being pointed at a noisy bench. Run it:

    python sysid.py

WHAT IS ACTUALLY MEASURABLE
---------------------------
A free ring-down gives you the natural frequency and the damping ratio. It does
NOT give you stiffness and inertia separately - only their ratio, because
omega_n = sqrt(k/J). Everyone forgets this and quietly substitutes a CAD
inertia, which imports the CAD's error into the stiffness number.

The fix is the added-inertia method: measure omega_n, bolt on a known extra
inertia, measure again. Two equations, two unknowns, and neither depends on
trusting the solid model. See inertia_from_added_mass().

THE TRAP IN LOG DECREMENT
-------------------------
Log decrement assumes VISCOUS damping, which decays exponentially. Coulomb
friction decays LINEARLY. A printed cycloidal reducer has plenty of Coulomb
friction, so fitting an exponential to its ring-down reports a damping ratio
that is too high - and it gets worse the smaller the oscillation, which is the
opposite of the intuition that small signals are cleaner.

_self_check() measures that bias rather than asserting it is absent. Estimate
damping from the FIRST few cycles at LARGE amplitude, where the viscous term
dominates, and treat the number as an upper bound.
"""

from __future__ import annotations

import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

from joint_sim import JointPlant, build_default_plant  # noqa: E402

# ------------------------------------------------------------- estimators --

def find_peaks(y: np.ndarray, min_separation: int = 1) -> np.ndarray:
    """Indices of strict local maxima, at least `min_separation` apart."""
    idx = []
    last = -min_separation - 1
    for i in range(1, len(y) - 1):
        if y[i] > y[i - 1] and y[i] >= y[i + 1] and i - last > min_separation:
            idx.append(i)
            last = i
    return np.asarray(idx, dtype=int)


def ring_down(t: np.ndarray, theta: np.ndarray,
              max_cycles: int = 4) -> tuple[float, float]:
    """(natural frequency Hz, damping ratio) from a free decay.

    `max_cycles` is deliberately small: later cycles are where Coulomb friction
    dominates and where the exponential fit stops being valid.
    """
    t = np.asarray(t, float)
    y = np.asarray(theta, float)

    # Equilibrium is where it ends up, not zero - the joint sags under gravity.
    equilibrium = float(np.mean(y[-max(len(y) // 10, 2):]))
    y = y - equilibrium

    peaks = find_peaks(y)
    peaks = peaks[y[peaks] > 0]
    if len(peaks) < 3:
        raise ValueError(f"need >=3 peaks for log decrement, found {len(peaks)}")
    peaks = peaks[:max_cycles + 1]

    n = len(peaks) - 1
    a0, an = y[peaks[0]], y[peaks[-1]]
    if a0 <= 0 or an <= 0:
        raise ValueError("non-positive peak amplitudes")

    delta = math.log(a0 / an) / n                    # log decrement per cycle
    zeta = delta / math.sqrt(4.0 * math.pi ** 2 + delta ** 2)

    td = float(np.mean(np.diff(t[peaks])))           # damped period
    fd = 1.0 / td
    fn = fd / math.sqrt(max(1.0 - zeta ** 2, 1e-12))
    return fn, zeta


def stiffness_from_ringdown(fn_hz: float, zeta: float,
                            inertia: float) -> tuple[float, float]:
    """(stiffness N.m/rad, damping N.m.s/rad) given an inertia you trust."""
    wn = 2.0 * math.pi * fn_hz
    k = inertia * wn * wn
    c = 2.0 * zeta * math.sqrt(k * inertia)
    return k, c


def inertia_from_added_mass(fn_bare: float, fn_loaded: float,
                            added_inertia: float) -> tuple[float, float]:
    """(inertia, stiffness) from two ring-downs, without trusting CAD.

    k = J w1^2 = (J + dJ) w2^2  ->  J = dJ w2^2 / (w1^2 - w2^2)

    Adding inertia must LOWER the frequency; if it did not, the added mass was
    not rigidly coupled and the measurement is void.
    """
    w1 = 2.0 * math.pi * fn_bare
    w2 = 2.0 * math.pi * fn_loaded
    if w1 <= w2:
        raise ValueError("loaded frequency must be lower - is the mass rigid?")
    j = added_inertia * w2 * w2 / (w1 * w1 - w2 * w2)
    return j, j * w1 * w1


def stiffness_static(torque_nm: float, deflection_deg: float) -> float:
    """Stiffness from a hung mass. The simplest experiment that works."""
    if deflection_deg == 0:
        raise ValueError("zero deflection")
    return torque_nm / math.radians(deflection_deg)


def friction_lsq(omega_deg_s, torque_nm) -> tuple[float, float]:
    """(Coulomb N.m, viscous N.m.s/rad) by least squares on tau = tc*sgn(w)+b*w.

    Feed it CONSTANT-VELOCITY segments only. Accelerating samples carry J*alpha,
    which this model has no term for and which will be absorbed into the viscous
    coefficient as a pure artefact.
    """
    w = np.radians(np.asarray(omega_deg_s, float))
    tau = np.asarray(torque_nm, float)
    keep = np.abs(w) > 1e-6
    w, tau = w[keep], tau[keep]
    if len(w) < 2:
        raise ValueError("need at least two non-zero velocity samples")
    A = np.column_stack([np.sign(w), w])
    (tc, b), *_ = np.linalg.lstsq(A, tau, rcond=None)
    return float(tc), float(b)


def backlash_hysteresis(motor_deg, output_deg, output_vel_deg_s,
                        vel_threshold: float = 0.5,
                        reversing_load_nm: float = 0.0,
                        stiffness: float = 0.0) -> float:
    """Backlash width in degrees from a slow reversal.

    OBSERVABILITY - read this before trusting a result. Backlash is only
    traversed if the TRANSMITTED TORQUE CHANGES SIGN between the two
    directions. If it does not, the same tooth flank stays loaded the whole
    time, the gap never opens, and the estimate comes back near zero - not
    because there is no backlash but because nothing ever crossed it. The
    failure is silent, which is what earns it a paragraph.

    Torque reverses only when the direction-reversing load (friction) exceeds
    the constant load (gravity). Measured on the simulated joint, sweeping
    gravity against a fixed 0.15 N.m of Coulomb friction:

        gravity 0.00 N.m -> gap 0.540 deg   (correct)
        gravity 0.05 N.m -> gap 0.319 deg   (already 40 % low)
        gravity 0.15 N.m -> gap 0.040 deg   (dead)
        gravity 1.50 N.m -> gap 0.040 deg

    So on the bench: measure backlash with the JOINT AXIS VERTICAL, where
    gravity produces no torque about it, or counterbalance the link. Measuring
    a horizontal shoulder axis will report a beautifully repeatable near-zero
    backlash that is entirely an artefact. My earlier reasoning that "a
    constant load cancels because it offsets both directions equally" is
    wrong - it only cancels if the flank still switches.

    Given that it does switch, the twist sits at half the backlash plus the
    windup from the transmitted torque, so

        gap = backlash + 2 * windup(reversing load)

    and `reversing_load_nm` should be the friction AT THE TEST SPEED. Creeping
    too slowly puts you in the pre-sliding regime where Coulomb friction is not
    fully developed and the correction over-subtracts.
    """
    m = np.asarray(motor_deg, float)
    o = np.asarray(output_deg, float)
    v = np.asarray(output_vel_deg_s, float)
    twist = m - o

    fwd = twist[v > vel_threshold]
    rev = twist[v < -vel_threshold]
    if len(fwd) < 5 or len(rev) < 5:
        raise ValueError("not enough samples in both directions")

    gap = float(np.median(fwd) - np.median(rev))
    if stiffness > 0.0 and reversing_load_nm != 0.0:
        gap -= 2.0 * math.degrees(abs(reversing_load_nm) / stiffness)
    return abs(gap)


# ------------------------------------------------------------ excitation --

def chirp_deg(t, amplitude_deg: float, f0: float, f1: float, duration: float):
    """Linear-frequency chirp. Sweep through the expected resonance.

    Amplitude has to clear the backlash band or the joint spends the low-force
    part of every cycle disconnected and the response is meaningless. A useful
    floor is ~3x the backlash.
    """
    t = np.asarray(t, float)
    k = (f1 - f0) / duration
    return amplitude_deg * np.sin(2.0 * math.pi * (f0 * t + 0.5 * k * t * t))


def velocity_staircase(v_max_deg_s: float, steps: int = 8,
                       dwell_s: float = 1.5):
    """(velocity, dwell) pairs for a friction sweep, both directions.

    Dwell long enough that the acceleration transient has died before you start
    averaging, or J*alpha contaminates the fit.
    """
    speeds = np.linspace(v_max_deg_s / steps, v_max_deg_s, steps)
    plan = [(float(v), dwell_s) for v in speeds]
    plan += [(-float(v), dwell_s) for v in speeds]
    return plan


# ------------------------------------------------------------ validation --

def _simulate_ringdown(plant: JointPlant, release_deg: float,
                       duration: float = 3.0, dt: float = 2e-5):
    """Hold the motor, displace the output, let go. Records (t, theta_o)."""
    plant.reset(0.0)
    plant.theta_o += release_deg          # pull the link aside and release
    plant.omega_o = 0.0
    n = int(duration / dt)
    t = np.empty(n)
    th = np.empty(n)
    for i in range(n):
        plant.step(dt, 0.0)               # motor commanded to hold
        t[i] = i * dt
        th[i] = plant.theta_o
    return t, th


def _self_check() -> None:
    print("=" * 70)
    print("SYSTEM ID ESTIMATOR VALIDATION")
    print("=" * 70)

    # --- 1. analytic decaying sinusoid: pure viscous, truth known exactly ---
    fn_true, zeta_true = 12.0, 0.08
    wn = 2 * math.pi * fn_true
    wd = wn * math.sqrt(1 - zeta_true ** 2)
    t = np.arange(0.0, 2.0, 1e-5)
    y = 5.0 * np.exp(-zeta_true * wn * t) * np.cos(wd * t)
    fn, zeta = ring_down(t, y, max_cycles=6)
    print(f"[1] pure viscous decay")
    print(f"    fn   truth {fn_true:6.3f} Hz   est {fn:6.3f} Hz   "
          f"err {abs(fn - fn_true) / fn_true * 100:.2f} %")
    print(f"    zeta truth {zeta_true:6.4f}      est {zeta:6.4f}      "
          f"err {abs(zeta - zeta_true) / zeta_true * 100:.2f} %")
    assert abs(fn - fn_true) / fn_true < 0.02
    assert abs(zeta - zeta_true) / zeta_true < 0.05

    # --- 2. added-inertia method recovers J and k without CAD ---------------
    j_true, k_true = 0.05, 300.0
    dj = 0.02
    f1 = math.sqrt(k_true / j_true) / (2 * math.pi)
    f2 = math.sqrt(k_true / (j_true + dj)) / (2 * math.pi)
    j_est, k_est = inertia_from_added_mass(f1, f2, dj)
    print(f"[2] added-inertia method")
    print(f"    J truth {j_true:.5f}  est {j_est:.5f} kg.m^2")
    print(f"    k truth {k_true:.2f}    est {k_est:.2f} N.m/rad")
    assert abs(j_est - j_true) < 1e-9 and abs(k_est - k_true) < 1e-6

    # --- 3. friction least squares -----------------------------------------
    tc_true, b_true = 0.15, 0.05
    w_deg = np.array([2, 5, 10, 15, 20, -2, -5, -10, -15, -20], float)
    tau = tc_true * np.sign(w_deg) + b_true * np.radians(w_deg)
    rng = np.random.default_rng(0)
    tau_noisy = tau + rng.normal(0, 0.002, len(tau))
    tc, b = friction_lsq(w_deg, tau_noisy)
    print(f"[3] friction least squares (with noise)")
    print(f"    Coulomb truth {tc_true:.4f}  est {tc:.4f} N.m")
    print(f"    viscous truth {b_true:.4f}  est {b:.4f} N.m.s/rad")
    assert abs(tc - tc_true) < 0.01 and abs(b - b_true) < 0.01

    # --- 4. the Coulomb bias, measured rather than assumed ------------------
    # Same viscous damping, but add Coulomb friction and re-estimate. The
    # reported zeta should come out HIGH, and more so over more cycles.
    env = np.exp(-zeta_true * wn * t)
    coulomb_decay = np.maximum(5.0 * env - 0.9 * t, 0.0)
    y_c = coulomb_decay * np.cos(wd * t)
    fn_c, zeta_early = ring_down(t, y_c, max_cycles=3)
    fn_c2, zeta_late = ring_down(t, y_c, max_cycles=10)
    print(f"[4] Coulomb friction biases the damping estimate")
    print(f"    viscous truth        {zeta_true:.4f}")
    print(f"    est over  3 cycles   {zeta_early:.4f}  "
          f"(+{(zeta_early / zeta_true - 1) * 100:.0f} %)")
    print(f"    est over 10 cycles   {zeta_late:.4f}  "
          f"(+{(zeta_late / zeta_true - 1) * 100:.0f} %)")
    assert zeta_late > zeta_early > zeta_true, "expected upward bias"
    print("    -> fit EARLY cycles at LARGE amplitude; treat as an upper bound")

    # --- 5. end-to-end against the actual simulator -------------------------
    # Frictionless and weightless so the analytic answer is unambiguous; this
    # tests that joint_sim's own parameterisation is self-consistent.
    plant = replace(build_default_plant(joint_index=2),
                    coulomb_nm=0.0, viscous_nms=0.0, gravity_nm=0.0,
                    backlash_deg=0.0, encoder_noise_counts=0.0)
    t5, th5 = _simulate_ringdown(plant, release_deg=3.0, duration=2.0)
    fn5, zeta5 = ring_down(t5, th5, max_cycles=4)
    k5, c5 = stiffness_from_ringdown(fn5, zeta5, plant.inertia)
    print(f"[5] round trip through joint_sim.JointPlant")
    print(f"    fn        expected {plant.natural_hz:6.3f} Hz   "
          f"recovered {fn5:6.3f} Hz")
    print(f"    zeta      expected {plant.damping_ratio:6.4f}      "
          f"recovered {zeta5:6.4f}")
    print(f"    stiffness expected {plant.stiffness:6.1f}        "
          f"recovered {k5:6.1f} N.m/rad")
    assert abs(fn5 - plant.natural_hz) / plant.natural_hz < 0.05
    assert abs(k5 - plant.stiffness) / plant.stiffness < 0.10

    # --- 6. backlash recovery from a simulated reversal --------------------
    # Gravity torque is zero here, standing in for a VERTICAL joint axis. That
    # is what lets the transmitted torque reverse and the flanks swap over.
    bl_true = 0.5
    coulomb = 0.15
    plant6 = replace(build_default_plant(joint_index=2),
                     backlash_deg=bl_true, gravity_nm=0.0,
                     coulomb_nm=coulomb, viscous_nms=0.0,
                     encoder_noise_counts=0.0)

    def _reversal(p, speed_deg_s=2.0, dt=2e-5, half=2.0):
        p.reset(0.0)
        rate = speed_deg_s * p.steps_per_output_deg
        m, o, v = [], [], []
        for i in range(int(2 * half / dt)):
            p.step(dt, rate if i < int(half / dt) else -rate)
            m.append(p.theta_m)
            o.append(p.theta_o)
            v.append(p.omega_o)
        return m, o, v

    m, o, v = _reversal(plant6)
    bl = backlash_hysteresis(m, o, v, vel_threshold=0.5,
                             reversing_load_nm=coulomb,
                             stiffness=plant6.stiffness)
    print(f"[6] backlash from reversal, vertical axis (no gravity torque)")
    print(f"    truth {bl_true:.3f} deg   est {bl:.3f} deg   "
          f"err {abs(bl - bl_true) / bl_true * 100:.1f} %")
    assert abs(bl - bl_true) / bl_true < 0.10

    # --- 7. the silent failure: gravity preload holds one flank ------------
    plant7 = replace(plant6, gravity_nm=1.5)
    m7, o7, v7 = _reversal(plant7)
    bl7 = backlash_hysteresis(m7, o7, v7, vel_threshold=0.5)
    print(f"[7] SAME joint, same backlash, horizontal axis (1.5 N.m gravity)")
    print(f"    truth {bl_true:.3f} deg   est {bl7:.3f} deg   "
          f"reads {(1 - bl7 / bl_true) * 100:.0f} % low, and silently")
    assert bl7 < 0.25 * bl_true, "expected the preloaded test to under-report"
    print("    -> gravity preload (1.5 N.m) swamps friction (0.15 N.m), so the")
    print("       transmitted torque never changes sign and the flanks never")
    print("       swap. Measure backlash about a VERTICAL axis.")

    print()
    print("all estimator validations passed")


def bench_plan() -> None:
    """The order to actually run these on hardware, and why."""
    plant = build_default_plant(joint_index=2)
    fn = plant.natural_hz
    print()
    print("=" * 70)
    print("BENCH PLAN - Phase 1B, one joint, roughly half a day")
    print("=" * 70)
    rows = [
        ("1. Link mass + CoM", "kitchen scale; balance on a knife edge",
         "20 min", "feeds inertia and gravity torque"),
        ("2. Static stiffness", "hang a known mass, read encoder deflection",
         "20 min", "stiffness_static()"),
        ("3. Ring-down, bare", "tap the link, log at >= 1 kHz",
         "20 min", "ring_down()"),
        ("4. Ring-down, +dJ", "repeat with a known mass bolted on",
         "20 min", "inertia_from_added_mass()"),
        ("5. Friction sweep", "constant-velocity staircase, both directions",
         "1 h", "friction_lsq()"),
        ("6. Backlash", "slow reversal about a VERTICAL axis",
         "30 min", "backlash_hysteresis() - see its docstring first"),
        ("7. Loop jitter", "toggle a GPIO, catch it on a logic analyser",
         "30 min", "feeds the sim's sample-and-hold"),
    ]
    for name, how, dur, feeds in rows:
        print(f"  {name:<22} {dur:>7}  {how}")
        print(f"  {'':<22} {'':>7}  -> {feeds}")

    print()
    print(f"  Sampling: the predicted resonance is {fn:.1f} Hz, so log the")
    print(f"  ring-down at >= {fn * 20:.0f} Hz. The 200 Hz control loop is")
    print(f"  only {200 / fn:.0f}x the mode - fine for the loop, marginal for")
    print("  identifying it. Use a dedicated fast logging app, not app_05.")
    print()
    print("  Cross-check: step 2 and steps 3-4 both produce a stiffness by")
    print("  independent routes. If they disagree by more than ~20 % you have")
    print("  a compliant mount, not a compliant reducer - the usual culprit is")
    print("  the bench plate, not the gearbox.")


if __name__ == "__main__":
    _self_check()
    bench_plan()
