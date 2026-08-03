"""What volume can this arm actually reach, and how much of it is usable?

Maximum reach is a marketing number. The decisions that matter before you print
a link are:

  * How big is the dead cylinder around the base? The d4 wrist offset makes a
    region near the J1 axis unreachable, and it is larger than people expect.
  * Where should the a2/a3 split fall? Equal links maximise workspace volume;
    a shorter forearm reduces shoulder torque. Those pull opposite ways.
  * How much of the workspace is close enough to a singularity that the motion
    controller has to handle it?

Run:  python tools/workspace.py
"""

from __future__ import annotations

import math

import numpy as np

from arm_model import DH, JOINT_LIMITS_DEG, DHParams, arm_from_links
from kinematics import fk, jacobian

# A pose is called near-singular when its smallest singular value falls below
# this. Below it, holding a Cartesian velocity needs joint rates that a stepper
# under load will not deliver.
SIGMA_MIN = 0.02
VOXEL = 0.020  # m, for the occupancy-based volume estimate


def sample(n: int, dh: DHParams = DH, seed: int = 0, with_sigma: bool = True):
    """Random joint configurations within limits -> tool positions and sigma_min."""
    rng = np.random.default_rng(seed)
    lo = np.radians([a for a, _ in JOINT_LIMITS_DEG])
    hi = np.radians([b for _, b in JOINT_LIMITS_DEG])
    q = rng.uniform(lo, hi, size=(n, 6))

    pos = np.empty((n, 3))
    sigma = np.empty(n) if with_sigma else None
    for i, qi in enumerate(q):
        pos[i] = fk(qi, dh)[:3, 3]
        if with_sigma:
            # Linear block only: mixing metres and radians in one SVD is meaningless.
            sigma[i] = np.linalg.svd(jacobian(qi, dh)[:3, :], compute_uv=False)[-1]
    return q, pos, sigma


def voxel_volume(pos: np.ndarray, size: float = VOXEL) -> float:
    """Occupancy estimate. Converges from below - see volume_convergence()."""
    keys = np.floor(pos / size).astype(np.int64)
    return len(np.unique(keys, axis=0)) * size ** 3


def volume_convergence(pos: np.ndarray) -> bool:
    """Occupancy volume is biased low until the samples saturate the voxels.

    Reporting a volume without this check is how you end up quoting 28 L and
    98 L for the same arm. Returns True if the estimate has settled.
    """
    print(f"\nvolume convergence (voxel {VOXEL*1000:.0f} mm):")
    vals = []
    for frac in (0.05, 0.1, 0.25, 0.5, 1.0):
        k = max(int(len(pos) * frac), 1)
        v = voxel_volume(pos[:k]) * 1000
        vals.append(v)
        print(f"  {k:6d} samples -> {v:6.1f} L")
    settled = abs(vals[-1] - vals[-2]) / vals[-1] < 0.05
    if not settled:
        print("  NOT CONVERGED - treat the volume as a lower bound. It is still")
        print("  valid for comparing designs at equal sample count, not as an")
        print("  absolute figure.")
    return settled


def metrics(pos: np.ndarray, sigma: np.ndarray) -> dict:
    radial = np.hypot(pos[:, 0], pos[:, 1])
    return {
        "max_reach": float(radial.max()),
        "dead_radius": float(radial.min()),
        "z_min": float(pos[:, 2].min()),
        "z_max": float(pos[:, 2].max()),
        "volume_l": voxel_volume(pos) * 1000,
        "near_singular_pct": float(100 * np.mean(sigma < SIGMA_MIN)),
        "median_sigma": float(np.median(sigma)),
    }


def link_split_study(total: float, payload: float, n: int = 20000) -> None:
    """Trade workspace volume against shoulder torque as a2/a3 varies."""
    print(f"\nlink split study (a2 + a3 held at {total*1000:.0f} mm, "
          f"{n} samples each)")
    print(f"{'a2':>6} {'a3':>6} {'ratio':>6} {'volume':>9} {'max r':>7} "
          f"{'shoulder':>9} {'arm kg':>7}")
    print("-" * 62)
    for frac in (0.40, 0.45, 0.50, 0.53, 0.55, 0.60):
        a2, a3 = total * frac, total * (1 - frac)
        dh = DHParams(a2=-a2, a3=-a3)
        _, pos, _ = sample(n, dh, seed=1, with_sigma=False)
        radial = np.hypot(pos[:, 0], pos[:, 1])

        # Rebuild the arm so structural mass tracks the new lengths.
        arm = arm_from_links(a2, a3, dh.d5 + dh.d6, payload)
        torque, _ = arm.worst_case_torque(1)

        print(f"{a2*1000:5.0f} {a3*1000:5.0f} {a2/a3:6.2f} "
              f"{voxel_volume(pos)*1000:7.1f} L {radial.max()*1000:6.0f} "
              f"{torque:8.2f} {arm.moving_mass:7.2f}")
    print("Max reach is fixed by a2+a3, so the split is a weak lever: both")
    print("columns move only a few percent. A shorter upper arm wins slightly")
    print("on both counts here, because it pulls the elbow motor inboard.")
    print("That is within model error - do not treat it as a strong result.")


def singularity_report(q: np.ndarray, sigma: np.ndarray) -> None:
    """Classify near-singular samples into the three UR families."""
    near = sigma < SIGMA_MIN
    if not near.any():
        print("\nno near-singular samples")
        return
    qn = q[near]
    wrist = np.abs(np.sin(qn[:, 4])) < 0.15          # J5 near 0: axes 4 and 6 align
    elbow = np.abs(np.sin(qn[:, 2])) < 0.15          # J3 near 0: arm straight
    shoulder = ~(wrist | elbow)                      # wrist centre near the J1 axis

    print(f"\nnear-singular poses (sigma_min < {SIGMA_MIN}): "
          f"{100*near.mean():.1f} % of the workspace")
    for name, mask in (("wrist  (J5 -> 0)", wrist),
                       ("elbow  (J3 -> 0)", elbow),
                       ("shoulder", shoulder)):
        print(f"  {name:18s} {100*mask.mean():5.1f} % of those")


def main() -> None:
    n = 20000
    q, pos, sigma = sample(n)
    m = metrics(pos, sigma)

    print(f"sampled {n} configurations within joint limits\n")
    print(f"max reach (radial)     {m['max_reach']*1000:7.0f} mm  "
          f"(DH sum says {DH.horizontal_reach*1000:.0f} - the wrist offset")
    print(f"                                    does not all project radially)")
    print(f"dead cylinder radius   {m['dead_radius']*1000:7.0f} mm  "
          f"(the d4={DH.d4*1000:.0f} mm wrist offset)")
    print(f"vertical span          {m['z_min']*1000:7.0f} .. {m['z_max']*1000:.0f} mm")
    print(f"reachable volume       {m['volume_l']:7.1f} L")
    print(f"median sigma_min       {m['median_sigma']:7.3f}")

    volume_convergence(pos)
    singularity_report(q, sigma)
    link_split_study(abs(DH.a2) + abs(DH.a3), payload=0.5)

    print(f"\ncurrent design: a2={abs(DH.a2)*1000:.0f} a3={abs(DH.a3)*1000:.0f} mm, "
          f"ratio {abs(DH.a2/DH.a3):.2f}")
    print("UR5 runs 425/392 (ratio 1.08). This model mildly prefers the opposite")
    print("split, because it treats the elbow motor as a point mass at r=a2 and")
    print("a real forearm carries more distributed mass than that. Keep the")
    print("current split; revisit only once link masses are measured.")


if __name__ == "__main__":
    main()
