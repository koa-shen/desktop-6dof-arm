"""Tool-tip position error budget for the 6R arm.

The README claims +/-1 mm. Nothing in this repo has ever checked whether that
number survives a 12-bit encoder at 400 mm of reach. This does.

The core relationship is the linear part of the geometric Jacobian:

    dp = J_v(q) . dq

so a joint angle error dq_i produces a tool-tip error of ||J_v[:, i]|| * dq_i.
That column norm is exactly the perpendicular distance from joint i's axis to
the tool - the moment arm. At full extension the shoulder's moment arm is the
whole reach, which is why base joints dominate every budget and why the wrist
almost never matters.

THE DISTINCTION THAT MATTERS
----------------------------
Error sources split into two kinds, and conflating them is the most common
mistake in a spec sheet:

  * RANDOM      - different every time you visit the pose. Sets REPEATABILITY.
                  Cannot be calibrated out. Combine in RSS, because independent
                  errors do not conspire.
  * SYSTEMATIC  - the same every time you visit the pose. Degrades ACCURACY but
                  not repeatability, and is therefore CALIBRATABLE. Combine
                  worst-case, because they are not independent.

Hobby projects quote repeatability, because it is the easy one and it is what a
return-to-home test measures. Accuracy is the hard number and the one an
industrial spec leads with. This tool reports both, and reports what calibration
would buy, which is the whole argument for doing Phase 4 properly.

Units: SI internally (metres, radians). Reported in mm and degrees.

Run from this directory:  python error_budget.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

from arm_model import DH, DHParams, JOINT_LIMITS_DEG, G, default_arm
from kinematics import fk_all, jacobian

# --------------------------------------------------------------- sources --

# AS5600, 12-bit, from the datasheet. VERIFY THESE against your own part before
# quoting them anywhere - they are the load-bearing numbers in this whole file.
AS5600_BITS = 12
AS5600_LSB_DEG = 360.0 / (1 << AS5600_BITS)   # 0.0879 deg
AS5600_INL_DEG = 0.30       # integral non-linearity, typical. Max spec is ~1.4.
AS5600_NOISE_DEG = 0.015    # RMS output noise

# A magnet mounted off the axis of rotation produces a once-per-revolution
# sinusoidal error.
#
# CAREFUL - the obvious geometric model is wrong for this sensor. It is
# tempting to write atan(e / r), as if the chip were tracking a pole sitting at
# radius r. That is the right model for a ring magnet with poles at the rim,
# and it gives ~2.9 deg for a 0.1 mm offset, which is alarming and false. The
# AS5600 reads a DIAMETRIC magnet ON AXIS, where the field near the centre is
# deliberately quasi-uniform - that homogeneity is the entire reason diametric
# magnets are used. Displacing the magnet slightly therefore rotates the field
# direction only weakly. The datasheet permits +/-0.25 mm of displacement while
# still meeting its +/-1.4 deg accuracy spec, which bounds the coefficient far
# below the geometric estimate.
#
# So this is an empirical coefficient, not a derivation. Measure it: sweep the
# joint against a reference and look for the 1/rev harmonic.
MAGNET_ECCENTRICITY_M = 100e-6        # 0.1 mm - a good press-fit printed hub
ECCENTRICITY_DEG_PER_MM = 1.0         # EMPIRICAL. Bounded by the datasheet
                                      # displacement spec; verify on your part.


def eccentricity_error_deg(e: float = MAGNET_ECCENTRICITY_M) -> float:
    """Peak 1/rev angular error from a radially offset diametric magnet."""
    return e * 1000.0 * ECCENTRICITY_DEG_PER_MM


@dataclass(frozen=True)
class JointErrors:
    """Per-joint angular error at the OUTPUT of the reducer, in degrees.

    Every term here is an amplitude (peak), not an RMS, except `noise_rms`.
    """

    # --- random: sets repeatability ---
    quantization: float = AS5600_LSB_DEG / 2.0
    noise_rms: float = AS5600_NOISE_DEG
    # Backlash is listed as random because on an unknown approach direction it
    # is unpredictable within its band. Approach every point from the same
    # direction - as a CMM does - and it becomes systematic and vanishes from
    # the repeatability column. That is a free factor-of-two and it is a
    # firmware change, not a hardware one.
    backlash: float = 0.50
    # Step loss is not modelled as an error term: it is a fault, not a
    # tolerance, and the encoder detects it. See FAULT_FOLLOWING.

    # --- systematic: degrades accuracy, calibratable ---
    nonlinearity: float = AS5600_INL_DEG
    eccentricity: float = field(default_factory=eccentricity_error_deg)
    # Reducer windup is systematic *for a given load*, and the load is known
    # from the pose, so it is feedforward-compensatable.
    stiffness_nm_per_rad: float = 300.0   # ESTIMATED - measure in Phase 1B

    def random_rss(self) -> float:
        """Random terms combined in RSS, degrees."""
        return math.sqrt(self.quantization ** 2 +
                         self.noise_rms ** 2 +
                         self.backlash ** 2)

    def systematic_sum(self) -> float:
        """Systematic encoder terms, worst case, degrees. Excludes windup,
        which is load dependent and handled per-pose."""
        return self.nonlinearity + self.eccentricity


# ------------------------------------------------------------ structural --

@dataclass(frozen=True)
class StructuralErrors:
    """Error sources that act on the tool position directly, in metres."""

    # Printed link length vs the CAD number. Measure with calipers and correct
    # the DH table; until then this is a real accuracy term.
    link_length_tol: float = 0.5e-3
    # Bending of the links under gravity plus payload. Computed from the arm
    # model rather than assumed.
    include_deflection: bool = True
    # Assembly: joint axis perpendicularity/parallelism error, radians.
    axis_alignment_rad: float = math.radians(0.2)


# ------------------------------------------------------------- mechanics --

def link_point_masses(arm) -> list[tuple[int, float]]:
    """(frame index, mass) point masses for static torque.

    Approximates each link's mass as a point at the origin of the DH frame at
    its distal end. Coarse, but it captures the moment arms that dominate, and
    it is honest about being coarse. Frames are 1-indexed to match fk_all().
    """
    masses = [
        (1, arm.links[0].motor_mass),   # shoulder structure, above J1
        (2, arm.links[1].mass),         # upper arm, at the elbow
        (3, arm.links[2].mass),         # forearm, at wrist 1
        (5, arm.links[3].mass),         # wrist stack
    ]
    tip = arm.payload + arm.tool_mass
    if tip:
        masses.append((6, tip))
    return masses


def gravity_joint_torques(q, arm, dh: DHParams = DH) -> np.ndarray:
    """Static gravity torque at each joint, N.m, via the Jacobian transpose.

    tau = sum_k J_v(p_k)^T . (m_k * g)

    This is the principled version of arm_model.gravity_torque(): it uses the
    real kinematics instead of a planar projection, so it stays correct when
    J1 is rotated and when the wrist is not level.
    """
    frames = fk_all(q, dh)
    origins = [np.zeros(3)] + [T[:3, 3] for T in frames]
    axes = [np.array([0.0, 0.0, 1.0])] + [T[:3, 2] for T in frames]

    gvec = np.array([0.0, 0.0, -G])
    tau = np.zeros(6)

    for frame_idx, mass in link_point_masses(arm):
        if mass <= 0:
            continue
        p = origins[frame_idx]
        force = mass * gvec
        for i in range(6):
            if i >= frame_idx:
                continue  # joint is outboard of this mass; no moment
            # linear velocity Jacobian column of point p w.r.t. joint i
            jv = np.cross(axes[i], p - origins[i])
            tau[i] += float(jv @ force)
    return tau


# ---------------------------------------------------------------- budget --

@dataclass
class PoseBudget:
    q: np.ndarray
    sensitivity_mm_per_deg: np.ndarray   # per joint, ||J_v[:,i]|| in mm/deg
    repeatability_mm: float
    accuracy_mm: float
    contributions_mm: dict
    reach_mm: float

    def dominant(self) -> tuple:
        return max(self.contributions_mm.items(), key=lambda kv: kv[1])


def pose_budget(q, arm, joint_err: JointErrors = JointErrors(),
                struct: StructuralErrors = StructuralErrors(),
                dh: DHParams = DH) -> PoseBudget:
    """Full error budget at one configuration."""
    q = np.asarray(q, dtype=float)
    J = jacobian(q, dh)
    Jv = J[:3, :]                      # linear part, m/rad

    # Moment arm of each joint, expressed as mm of tip error per degree of
    # joint error. This is the number worth internalising.
    col_norms_m_per_rad = np.linalg.norm(Jv, axis=0)
    sens_mm_per_deg = col_norms_m_per_rad * 1000.0 * math.pi / 180.0

    # --- random terms, RSS across joints (independent) ---
    rand_deg = joint_err.random_rss()
    repeat_mm = float(np.sqrt(np.sum((sens_mm_per_deg * rand_deg) ** 2)))

    # --- systematic encoder terms, worst case across joints ---
    sys_deg = joint_err.systematic_sum()
    sys_mm = float(np.sum(sens_mm_per_deg * sys_deg))

    # --- reducer windup, load dependent ---
    tau = gravity_joint_torques(q, arm, dh)
    windup_deg = np.degrees(np.abs(tau) / joint_err.stiffness_nm_per_rad)
    windup_mm = float(np.sum(sens_mm_per_deg * windup_deg))

    # --- structural ---
    defl_mm = 0.0
    if struct.include_deflection:
        defl_mm = sum(arm.tip_deflection(i)
                      for i in range(1, len(arm.links))) * 1000.0
    # Link length tolerance adds along the chain, worst case.
    dh_mm = struct.link_length_tol * 1000.0 * 3
    # An axis misalignment tilts everything outboard of it.
    align_mm = float(np.sum(sens_mm_per_deg *
                            math.degrees(struct.axis_alignment_rad)))

    contributions = {
        "encoder quantization": float(np.sqrt(np.sum(
            (sens_mm_per_deg * joint_err.quantization) ** 2))),
        "encoder noise": float(np.sqrt(np.sum(
            (sens_mm_per_deg * joint_err.noise_rms) ** 2))),
        "backlash": float(np.sqrt(np.sum(
            (sens_mm_per_deg * joint_err.backlash) ** 2))),
        "encoder nonlinearity": float(np.sum(
            sens_mm_per_deg * joint_err.nonlinearity)),
        "magnet eccentricity": float(np.sum(
            sens_mm_per_deg * joint_err.eccentricity)),
        "reducer windup": windup_mm,
        "link deflection": defl_mm,
        "link length tolerance": dh_mm,
        "axis misalignment": align_mm,
    }

    accuracy_mm = repeat_mm + sys_mm + windup_mm + defl_mm + dh_mm + align_mm

    tool = fk_all(q, dh)[-1][:3, 3]
    return PoseBudget(
        q=q,
        sensitivity_mm_per_deg=sens_mm_per_deg,
        repeatability_mm=repeat_mm,
        accuracy_mm=accuracy_mm,
        contributions_mm=contributions,
        reach_mm=float(np.linalg.norm(tool)) * 1000.0,
    )


def worst_over_workspace(arm, n: int = 3000, seed: int = 0,
                         joint_err: JointErrors = JointErrors(),
                         struct: StructuralErrors = StructuralErrors()):
    """Sample the joint space and return (worst accuracy, worst repeatability)."""
    rng = np.random.default_rng(seed)
    lo = np.radians([a for a, _ in JOINT_LIMITS_DEG])
    hi = np.radians([b for _, b in JOINT_LIMITS_DEG])

    worst_acc = None
    worst_rep = None
    for _ in range(n):
        q = rng.uniform(lo, hi)
        b = pose_budget(q, arm, joint_err, struct)
        if worst_acc is None or b.accuracy_mm > worst_acc.accuracy_mm:
            worst_acc = b
        if worst_rep is None or b.repeatability_mm > worst_rep.repeatability_mm:
            worst_rep = b
    return worst_acc, worst_rep


def required_joint_accuracy(target_mm: float, arm,
                            q=None, dh: DHParams = DH) -> float:
    """Back-solve: what per-joint accuracy would hit `target_mm` at the tip?

    Assumes the error is equal across joints and combines in RSS, which is the
    optimistic case. If this number is below what any encoder you can buy
    delivers, the spec is not achievable by better sensing alone.
    """
    if q is None:
        q = np.radians([0.0, -60.0, 100.0, 0.0, 45.0, 0.0])
    J = jacobian(np.asarray(q, float), dh)
    sens = np.linalg.norm(J[:3, :], axis=0) * 1000.0 * math.pi / 180.0
    denom = math.sqrt(float(np.sum(sens ** 2)))
    return target_mm / denom if denom else float("inf")


# ------------------------------------------------------------------ main --

def _bar(x: float, full: float, width: int = 28) -> str:
    n = 0 if full <= 0 else int(round(width * x / full))
    return "#" * min(n, width)


def main() -> None:
    arm = default_arm(reach=0.400, payload=0.5)
    je = JointErrors()
    st = StructuralErrors()

    print("=" * 74)
    print("TOOL-TIP ERROR BUDGET")
    print("=" * 74)
    print(f"AS5600 LSB                  : {AS5600_LSB_DEG:.4f} deg")
    print(f"magnet eccentricity error   : {eccentricity_error_deg():.3f} deg "
          f"({MAGNET_ECCENTRICITY_M * 1e3:.2f} mm offset, empirical coeff)")
    print(f"assumed backlash            : {je.backlash:.2f} deg")
    print(f"assumed reducer stiffness   : {je.stiffness_nm_per_rad:.0f} N.m/rad"
          "   <- ESTIMATE, measure in 1B")

    # Worst case is the arm straight out: every joint has its full moment arm.
    q_ext = np.radians([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    q_typ = np.radians([0.0, -60.0, 100.0, 0.0, 45.0, 0.0])

    for label, q in (("EXTENDED (worst)", q_ext), ("TYPICAL (folded)", q_typ)):
        b = pose_budget(q, arm, je, st)
        print()
        print("-" * 74)
        print(f"{label}   tool at {b.reach_mm:.0f} mm from base axis")
        print("-" * 74)
        print("  joint sensitivity, mm of tip error per deg of joint error:")
        print("    " + "  ".join(f"J{i+1}:{s:6.2f}"
                                 for i, s in enumerate(b.sensitivity_mm_per_deg)))
        print()
        full = max(b.contributions_mm.values())
        for name, val in sorted(b.contributions_mm.items(),
                                key=lambda kv: -kv[1]):
            print(f"    {name:<24} {val:7.3f} mm  {_bar(val, full)}")
        print()
        print(f"    {'REPEATABILITY (random)':<24} {b.repeatability_mm:7.3f} mm")
        print(f"    {'ACCURACY (rand+system)':<24} {b.accuracy_mm:7.3f} mm")

    print()
    print("=" * 74)
    print("WORKSPACE SWEEP")
    print("=" * 74)
    wa, wr = worst_over_workspace(arm, n=2000, joint_err=je, struct=st)
    print(f"worst accuracy      : {wa.accuracy_mm:7.2f} mm "
          f"at reach {wa.reach_mm:.0f} mm")
    print(f"worst repeatability : {wr.repeatability_mm:7.2f} mm "
          f"at reach {wr.reach_mm:.0f} mm")
    dom, domv = wa.dominant()
    print(f"dominant term       : {dom} ({domv:.2f} mm)")

    print()
    print("=" * 74)
    print("WHAT WOULD +/-1 mm REQUIRE?")
    print("=" * 74)
    need = required_joint_accuracy(1.0, arm)
    print(f"per-joint accuracy needed for 1.0 mm at the tip : {need:.4f} deg")
    print(f"                                        in bits : "
          f"{math.log2(360.0 / need):.1f} bits/rev")
    print(f"AS5600 quantization alone                       : "
          f"{AS5600_LSB_DEG:.4f} deg  ({AS5600_BITS} bits)")
    print(f"AS5600 quantization + INL + eccentricity        : "
          f"{je.quantization + je.systematic_sum():.4f} deg")

    # What calibration buys: systematic terms are removable.
    print()
    print("=" * 74)
    print("WHAT CALIBRATION BUYS")
    print("=" * 74)
    b = pose_budget(q_ext, arm, je, st)
    calibrated = replace(je, nonlinearity=0.05, eccentricity=0.05)
    bc = pose_budget(q_ext, arm, calibrated, st)
    print(f"accuracy, uncalibrated : {b.accuracy_mm:7.2f} mm")
    print(f"accuracy, calibrated   : {bc.accuracy_mm:7.2f} mm  "
          f"({100 * (1 - bc.accuracy_mm / b.accuracy_mm):.0f} % better)")
    print("  (a per-joint lookup table from a reference-angle sweep removes")
    print("   most of INL and eccentricity; both are repeatable functions of")
    print("   shaft angle, which is exactly what makes them calibratable)")

    # Backlash is the one random term that firmware can convert to systematic.
    print()
    one_dir = replace(je, backlash=0.02)
    bd = pose_budget(q_ext, arm, one_dir, st)
    print(f"repeatability, bidirectional approach : "
          f"{b.repeatability_mm:6.2f} mm")
    print(f"repeatability, one-way approach       : "
          f"{bd.repeatability_mm:6.2f} mm  <- firmware change, not hardware")


def _self_check() -> None:
    """Relationships that must hold regardless of the parameter values."""
    arm = default_arm(reach=0.400, payload=0.5)

    # 1. Extended arm must be less accurate than folded: longer moment arms.
    ext = pose_budget(np.radians([0, 0, 0, 0, 0, 0]), arm)
    fold = pose_budget(np.radians([0, -90, 150, 0, 0, 0]), arm)
    assert ext.accuracy_mm > fold.accuracy_mm, "extended should be worse"

    # 2. Accuracy is never better than repeatability.
    assert ext.accuracy_mm >= ext.repeatability_mm

    # 3. J6 sensitivity must be tiny: the last axis is ~at the tool.
    assert ext.sensitivity_mm_per_deg[5] < ext.sensitivity_mm_per_deg[1]

    # 4. Zero joint error must give zero joint-driven error.
    zero = JointErrors(quantization=0, noise_rms=0, backlash=0,
                       nonlinearity=0, eccentricity=0,
                       stiffness_nm_per_rad=1e12)
    z = pose_budget(np.radians([0, 0, 0, 0, 0, 0]), arm, zero,
                    StructuralErrors(link_length_tol=0.0,
                                     include_deflection=False,
                                     axis_alignment_rad=0.0))
    # 1e-6 mm = 1 nm. Tighter than this is meaningless and only tests float
    # residue from the finite stiffness above.
    assert z.repeatability_mm < 1e-6, z.repeatability_mm
    assert z.accuracy_mm < 1e-6, z.accuracy_mm

    # 5. Gravity torque must vanish when the arm hangs straight down and must
    #    be maximal when it is horizontal.
    q_down = np.radians([0.0, -90.0, 0.0, 0.0, 0.0, 0.0])
    q_out = np.radians([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    t_down = abs(gravity_joint_torques(q_down, arm)[1])
    t_out = abs(gravity_joint_torques(q_out, arm)[1])
    assert t_out > t_down, f"horizontal {t_out} should exceed hanging {t_down}"

    # 6. J1 carries no gravity torque in the nominal upright mounting.
    assert abs(gravity_joint_torques(q_out, arm)[0]) < 1e-9

    print("error_budget self-checks passed")


if __name__ == "__main__":
    _self_check()
    print()
    main()
